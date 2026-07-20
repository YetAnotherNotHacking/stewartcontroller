from tkinter import constants
from scipy.spatial.transform import Rotation as R
from kinematics import StewartKinematicSolver
import tkinter as tk
from tkinter import ttk
import numpy as np
from render import PlatformRenderer
import urllib.request
import json
import threading

class ControlApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Stewart platform control")
        self.geometry("1400x1000")
        self.minsize(1400, 1000)

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # movement values
        self.motion = {
            "x": 0,
            "y": 0,
            "z": 0,
            "tilt_x": 0,
            "tilt_y": 0,
            "spin": 0
        }

        # connection state
        self.isconnect = tk.BooleanVar(value=False)
        self.deviceip = tk.StringVar(value="192.168.4.1")
        self.deviceport = tk.IntVar(value=80)

        self.movescale = tk.IntVar(value=25)
        self.solver = StewartKinematicSolver.create_hexagonal()

        self.saved_positions = {'c1': None, 'c2': None}
        self.animating = False

        self.create_widgets()
        
        # update display to initial position
        self.move(0, 0, 0, 0, 0, 0)

    # Update handlers
    def on_scale_change(self):
        print(f"Move scale updated to {self.movescale.get()}mm")

    def move_absolute(self, x, y, z, tilt_x, tilt_y, spin):
        revert_state = self.motion.copy()
        self.motion["x"] = x
        self.motion["y"] = y
        self.motion["z"] = z
        self.motion["tilt_x"] = tilt_x
        self.motion["tilt_y"] = tilt_y
        self.motion["spin"] = spin
        return self._apply_motion(revert_state)

    def _apply_motion(self, revert_state=None):
        target_translation = [self.motion["x"], self.motion["y"], self.motion["z"]]
        target_rotation = R.from_euler('xyz', [self.motion["tilt_x"], self.motion["tilt_y"], self.motion["spin"]], degrees=True)
        
        angles = self.solver.solve(target_translation, target_rotation)
        
        if angles is not None:
            angles_deg = np.degrees(angles)
            for i in range(6):
                self.servo_labels[i].config(text=f"Servo {i+1}: {angles_deg[i]:.2f}°")
            self.posstatus.config(text=f"pos is {self.motion['x']:.1f}, {self.motion['y']:.1f}, {self.motion['z']:.1f} at {self.motion['tilt_x']:.1f}, {self.motion['tilt_y']:.1f}, {self.motion['spin']:.1f}")
            
            if hasattr(self, 'renderer'):
                self.renderer.render(target_translation, target_rotation)
                
            self.send_angles(angles_deg)
            return True
        else:
            self.posstatus.config(text="Out of mechanical bounds!")
            if revert_state is not None:
                self.motion = revert_state.copy()
            return False

    def move(self, x=0, y=0, z=0, tilt_x=0, tilt_y=0, spin=0):
        scale = self.movescale.get()
        revert_state = self.motion.copy()
        
        self.motion["x"] += x * scale
        self.motion["y"] += y * scale
        self.motion["z"] += z * scale
        self.motion["tilt_x"] += tilt_x * scale
        self.motion["tilt_y"] += tilt_y * scale
        self.motion["spin"] += spin * scale

        self._apply_motion(revert_state)

    def save_pos(self, slot):
        self.saved_positions[slot] = self.motion.copy()
        
    def jump_saved(self, slot):
        if self.saved_positions[slot]:
            target = self.saved_positions[slot]
            self.jump_to(target['x'], target['y'], target['z'], target['tilt_x'], target['tilt_y'], target['spin'])

    def jump_to(self, x, y, z, tx, ty, spin):
        if self.animating: return
        self.animating = True
        
        start = self.motion.copy()
        target = {'x': x, 'y': y, 'z': z, 'tilt_x': tx, 'tilt_y': ty, 'spin': spin}
        
        steps = 20
        delay_ms = 20
        
        def animate_step(step):
            if step > steps:
                self.animating = False
                return
                
            t = step / steps
            smooth_t = t * t * (3 - 2 * t)
            
            curr_x = start['x'] + (target['x'] - start['x']) * smooth_t
            curr_y = start['y'] + (target['y'] - start['y']) * smooth_t
            curr_z = start['z'] + (target['z'] - start['z']) * smooth_t
            curr_tx = start['tilt_x'] + (target['tilt_x'] - start['tilt_x']) * smooth_t
            curr_ty = start['tilt_y'] + (target['tilt_y'] - start['tilt_y']) * smooth_t
            curr_spin = start['spin'] + (target['spin'] - start['spin']) * smooth_t
            
            success = self.move_absolute(curr_x, curr_y, curr_z, curr_tx, curr_ty, curr_spin)
            if not success:
                self.animating = False
                return
                
            self.after(delay_ms, lambda: animate_step(step + 1))
            
        animate_step(1)

    def create_widgets(self):
        main = ttk.Frame(self, padding=5)
        main.grid(row=0, column=0, sticky="nsew")
        main.grid_rowconfigure(0, weight=0) # top bar
        main.grid_rowconfigure(1, weight=1) # body
        main.grid_columnconfigure(0, weight=0)
        main.grid_columnconfigure(1, weight=1)

        # --- Top Bar ---
        topbar = ttk.Frame(main)
        topbar.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        topbar.grid_columnconfigure(1, weight=1) # Allow position log to expand
        
        # Network Connection
        connframe = ttk.LabelFrame(topbar, text="Network Connection", padding=5)
        connframe.grid(row=0, column=0, sticky="w", padx=5)
        
        ttk.Label(connframe, text="IP:").grid(row=0, column=0, padx=2)
        ttk.Entry(connframe, textvariable=self.deviceip, width=15).grid(row=0, column=1, padx=2)
        ttk.Label(connframe, text="Port:").grid(row=0, column=2, padx=2)
        ttk.Entry(connframe, textvariable=self.deviceport, width=5).grid(row=0, column=3, padx=2)
        self.connect_btn = ttk.Button(connframe, text="Connect", command=self.toggle_connect)
        self.connect_btn.grid(row=0, column=4, padx=5)
        
        # Position Log
        statusframe = ttk.LabelFrame(topbar, text="Status", padding=5)
        statusframe.grid(row=0, column=1, sticky="ew", padx=5)
        self.posstatus = ttk.Label(statusframe, text='pos is 0.0, 0.0, 0.0 at 0.0, 0.0, 0.0')
        self.posstatus.pack(fill="both", expand=True)

        # Jumps
        jumpsframe = ttk.LabelFrame(topbar, text="Jumps", padding=5)
        jumpsframe.grid(row=0, column=2, sticky="e", padx=5)
        
        ttk.Button(jumpsframe, text="Home", command=lambda: self.jump_to(0,0,0,0,0,0)).grid(row=0, column=0, columnspan=2, pady=2, sticky="ew")
        
        ttk.Button(jumpsframe, text="Save C1", command=lambda: self.save_pos('c1')).grid(row=1, column=0, padx=2)
        ttk.Button(jumpsframe, text="Jump C1", command=lambda: self.jump_saved('c1')).grid(row=1, column=1, padx=2)
        
        ttk.Button(jumpsframe, text="Save C2", command=lambda: self.save_pos('c2')).grid(row=2, column=0, padx=2)
        ttk.Button(jumpsframe, text="Jump C2", command=lambda: self.jump_saved('c2')).grid(row=2, column=1, padx=2)

        # --- Body ---
        jogframe = ttk.LabelFrame(main, text="Jog", padding=10)
        jogframe.grid(row=1, column=0, padx=10, pady=10, sticky="nw")

        planeframe = ttk.LabelFrame(jogframe, text="2D Movement", padding=10)
        planeframe.grid(row=0, column=0, padx=10, pady=10, sticky="w")
        
        # 2d plane (primary?) movement
        forwardbutton = tk.Button(planeframe,
            height=2,
            width=4,
            text = "fw",
            command=lambda: self.move(y=1))
        backbutton = tk.Button(planeframe,
            height=2,
            width=4,
            text="bk",
            command=lambda: self.move(y=-1))

        leftbutton = tk.Button(planeframe,
            height=2,
            width=4,
            text="lf",
            command=lambda: self.move(x=-1))
        rightbutton = tk.Button(planeframe,
            height=2,
            width=4,
            text="rt",
            command=lambda: self.move(x=1))

        forwardbutton.grid(row=0, column=1)
        leftbutton.grid(row=1, column=0)
        rightbutton.grid(row=1, column=2)
        backbutton.grid(row=2, column=1)

        # height adjustment
        heightframe = ttk.LabelFrame(jogframe, text="Height", padding=10)
        heightframe.grid(row=0, column=1, padx=10, pady=10, sticky="e")
        upbutton = tk.Button(heightframe,
            height=2,
            width=4,
            text="up",
            command=lambda: self.move(z=1))
        downbutton = tk.Button(heightframe,
            height=2,
            width=4,
            text="dn",
            command=lambda: self.move(z=-1))
        upbutton.grid(row=0, column=0)
        downbutton.grid(row=2, column=0)

        # tilt adjustments
        tiltframe = ttk.LabelFrame(jogframe, text="Tilt Control", padding=10)
        tiltframe.grid(row=1, column=0, padx=10, pady=10, sticky="w")
        tiltforward = tk.Button(tiltframe,
            height=2,
            width=4,
            text="fw",
            command=lambda: self.move(tilt_x=1))
        tiltback = tk.Button(tiltframe,
            height=2,
            width=4,
            text="bk",
            command=lambda: self.move(tilt_x=-1))

        tiltleft = tk.Button(tiltframe,
            height=2,
            width=4,
            text="lf",
            command=lambda: self.move(tilt_y=-1))
        tiltright = tk.Button(tiltframe,
            height=2,
            width=4,
            text="rt",
            command=lambda: self.move(tilt_y=1))

        spinleft = tk.Button(tiltframe,
            height=2,
            width=4,
            text="s-",
            command=lambda: self.move(spin=-1))
        spinright = tk.Button(tiltframe,
            height=2,
            width=4,
            text="s+",
            command=lambda: self.move(spin=1))

        tiltforward.grid(row=3, column=0)
        tiltback.grid(row=4, column=0)
        tiltleft.grid(row=3, column=1)
        tiltright.grid(row=4, column=1)
        spinleft.grid(row=3, column=2)
        spinright.grid(row=4, column=2)

        scaleframe = ttk.LabelFrame(jogframe, text="Scale", padding=10)
        scaleframe.grid(row=1, column=1, padx=10, pady=10, sticky="se")

        op1 = ttk.Radiobutton(scaleframe, 
            text="1mm", 
            variable=self.movescale, 
            value=1, 
            command=self.on_scale_change)
        
        op2 = ttk.Radiobutton(scaleframe, 
            text="10mm", 
            variable=self.movescale, 
            value=10,
            command=self.on_scale_change)
        
        op3 = ttk.Radiobutton(scaleframe, 
            text="15mm", 
            variable=self.movescale, 
            value=15, 
            command=self.on_scale_change)
    
        op4 = ttk.Radiobutton(scaleframe, 
            text="20mm", 
            variable=self.movescale, 
            value=20, 
            command=self.on_scale_change)

        op1.grid(row=0, column=0, pady=2)
        op2.grid(row=1, column=0, pady=2)
        op3.grid(row=2, column=0, pady=2)
        op4.grid(row=3, column=0, pady=2)

        servosframe = ttk.LabelFrame(main, text="Servo Angles", padding=10)
        servosframe.grid(row=2, column=0, padx=10, pady=10, sticky="nw")
        
        self.servo_labels = []
        for i in range(6):
            lbl = ttk.Label(servosframe, text=f"Servo {i+1}: 0.00°")
            lbl.grid(row=i//2, column=i%2, padx=10, pady=2, sticky="w")
            self.servo_labels.append(lbl)

        visframe = ttk.LabelFrame(main, text="Visualization", padding=10)
        visframe.grid(row=1, column=1, rowspan=2, padx=10, pady=10, sticky="nsew")
        
        self.renderer = PlatformRenderer(visframe, self.solver)
        self.renderer.pack(expand=True, fill="both")

    def toggle_connect(self):
        if not self.isconnect.get():
            self.isconnect.set(True)
            self.connect_btn.config(text="Disconnect")
            self.move() # Trigger a send of current angles immediately
        else:
            self.isconnect.set(False)
            self.connect_btn.config(text="Connect")

    def send_angles(self, angles_deg):
        if not self.isconnect.get():
            return
            
        # Shift -90 to +90 degrees into 0 to 180 degrees for standard servos
        payload = {
            "servos": [{"id": i, "angle": int(angles_deg[i] + 90)} for i in range(6)]
        }
        
        url = f"http://{self.deviceip.get()}:{self.deviceport.get()}/set"
        
        def _send():
            try:
                req = urllib.request.Request(url, method="POST")
                req.add_header('Content-Type', 'text/plain') # ESP32 WebServer parses 'plain' nicely
                data = json.dumps(payload).encode('utf-8')
                urllib.request.urlopen(req, data=data, timeout=0.5)
            except Exception as e:
                print(f"Failed to send to ESP: {e}")
                
        # Send in a background thread so the GUI doesn't freeze on network timeouts
        threading.Thread(target=_send, daemon=True).start()

    def run(self):
        self.mainloop()

if __name__ == "__main__":
    app = ControlApp()
    app.run()