#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArdiunoJson.h>

const char* ssid = "stewart-control";
const char* password = "1234";

WebServer server(80);
Adafruit_PWMServoDriver pwm;

#define SERVOMIN 120
#define SERVOMAX 620

int angleToPulse(int angle) {
    angle = constrain(angle, 0, 180); // incase kinematics are wrong somehow
    return map(angle, 0, 180, SERVOMIN, SERVOMAX); // return calculated pulse
}

void handleSet() {
    if (server.method() != HTTP_POST) { // reminder if wrong method is tried
        server.send(405, "text/plain", "Unsupported method, please use POST");
        return;
    }
    
    StaticJsonDocument<1024> doc;

    DeserializationError err = deserializeJson(doc, server.arg("plain"));

    // err here would be an error in deserializing the user's json
    if (err) {
        server.send(400, "text/plain", "Invalid JSON notation");
    }

    // user's array of servos to update
    JsonArray servos = doc["servos"];

    // iterate through the objects and update the positions accordingly to angles in request
    for (JsonObject servo : servos) {
        int id = servo["id"];
        int angle = servo["angle"];

        if (id >= 0 && id < 16) {
            pwm.setPWM(id, 0, angleToPulse(angle));
        }
    }

    server.send(200, "application/json", "{\"success\":true}");

}

void setup() {
    Serial.begin (115200);
    Wire.begin(21, 22);
    pwm.begin();
    pwm.setPWMFreq(50);

    WiFi.softAP(ssid, password);

    Serial.print("API live on: http://");
    Serial.println(WiFi.softAPIP());

    server.on("/set", HTTP_POST, handleSet);

    server.begin();
}

void loop() {
    server.handleClient();
}

