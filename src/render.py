# Disclosure: This file was generated with gemini.
import tkinter as tk
import numpy as np

class PlatformRenderer(tk.Canvas):
    def __init__(self, parent, solver, width=400, height=400, **kwargs):
        super().__init__(parent, width=width, height=height, bg='white', **kwargs)
        self.solver = solver
        self.width = width
        self.height = height
        
        self.scale = 1.0  # pixels per mm
        self.cx = width / 2
        self.cy = height / 2 + 100
        
        # Camera state
        self.angle_z = np.radians(30)
        self.angle_x = np.radians(-60)
        
        # Bind resize event to re-center
        self.bind("<Configure>", self.on_resize)
        
        # Bind mouse events for rotation (left click) and pan (right click)
        self.bind("<ButtonPress-1>", self.on_press)
        self.bind("<B1-Motion>", self.on_drag_rotate)
        self.bind("<ButtonPress-3>", self.on_press)
        self.bind("<B3-Motion>", self.on_drag_pan)
        
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        
        self.last_translation = np.array([0, 0, 0])
        self.last_rotation = None

    def on_press(self, event):
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y

    def on_drag_rotate(self, event):
        dx = event.x - self.last_mouse_x
        dy = event.y - self.last_mouse_y
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y
        
        # Adjust angles (sensitivity)
        self.angle_z -= dx * 0.01
        self.angle_x -= dy * 0.01
        
        if self.last_rotation is not None:
            self.render(self.last_translation, self.last_rotation)

    def on_drag_pan(self, event):
        dx = event.x - self.last_mouse_x
        dy = event.y - self.last_mouse_y
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y
        
        self.cx += dx
        self.cy += dy
        
        if self.last_rotation is not None:
            self.render(self.last_translation, self.last_rotation)

    def on_resize(self, event):
        self.width = event.width
        self.height = event.height
        self.cx = self.width / 2
        self.cy = self.height / 2 + 100
        if self.last_rotation is not None:
            self.render(self.last_translation, self.last_rotation)

    def project(self, point):
        x, y, z = point
        
        # Use interactive camera angles
        angle_z = self.angle_z
        angle_x = self.angle_x
        
        # Rotate around Z (yaw)
        x1 = x * np.cos(angle_z) - y * np.sin(angle_z)
        y1 = x * np.sin(angle_z) + y * np.cos(angle_z)
        z1 = z
        
        # Rotate around X (pitch)
        x2 = x1
        y2 = y1 * np.cos(angle_x) - z1 * np.sin(angle_x)
        
        sx = self.cx + x2 * self.scale
        sy = self.cy - y2 * self.scale
        return sx, sy

    def sort_points_circularly(self, points):
        if len(points) == 0:
            return points
        # Calculate centroid to find center
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        # Sort based on angle from center point (using atan2)
        return sorted(points, key=lambda p: np.arctan2(p[1] - cy, p[0] - cx))

    def render(self, translation, rotation):
        self.last_translation = translation
        self.last_rotation = rotation
        
        self.delete("all")
        if self.solver is None:
            return
            
        angles = self.solver.solve(translation, rotation)
        if angles is None:
            self.create_text(self.cx, self.cy, text="Target out of mechanical bounds", fill="red")
            return
            
        B = self.solver.B
        P_rotated = rotation.apply(self.solver.P)
        P_global = translation + P_rotated + self.solver.T0
        
        # Calculate horn ends (H)
        H = np.zeros((6, 3))
        for i in range(6):
            alpha = angles[i]
            beta = self.solver.beta[i]
            
            hx = self.solver.horn_length * np.cos(alpha) * np.cos(beta)
            hy = self.solver.horn_length * np.cos(alpha) * np.sin(beta)
            hz = self.solver.horn_length * np.sin(alpha)
            
            H[i] = B[i] + np.array([hx, hy, hz])

        # Original points for leg attachment
        proj_B = [self.project(b) for b in B]
        proj_H = [self.project(h) for h in H]
        proj_P = [self.project(p) for p in P_global]
        
        # Sorted points for drawing the physical hexagonal plates without criss-crossing
        B_sorted = self.sort_points_circularly(B)
        P_global_sorted = self.sort_points_circularly(P_global)
        
        proj_B_poly = [self.project(b) for b in B_sorted]
        proj_P_poly = [self.project(p) for p in P_global_sorted]
        
        # Draw base plate
        flat_B = [coord for p in proj_B_poly for coord in p]
        if flat_B:
            self.create_polygon(flat_B, fill='#e0e0e0', outline='blue', width=2)
        
        # Draw servo horns and rods
        for i in range(6):
            pb = proj_B[i]
            ph = proj_H[i]
            pp = proj_P[i]
            alpha = angles[i]
            
            # Horn
            self.create_line(pb[0], pb[1], ph[0], ph[1], fill='#00aa00', width=4)
            # Rod
            self.create_line(ph[0], ph[1], pp[0], pp[1], fill='black', width=2)
            
            # Joints
            self.create_oval(pb[0]-4, pb[1]-4, pb[0]+4, pb[1]+4, fill='blue') # Motor shaft
            self.create_oval(ph[0]-3, ph[1]-3, ph[0]+3, ph[1]+3, fill='#00ff00') # Horn joint
            self.create_oval(pp[0]-3, pp[1]-3, pp[0]+3, pp[1]+3, fill='red') # Platform joint
            
            # Angle label
            self.create_text(pb[0], pb[1] + 12, text=f"{np.degrees(alpha):.0f}°", fill="black", font=("Arial", 8))
            
        # Draw platform plate
        flat_P = [coord for p in proj_P_poly for coord in p]
        if flat_P:
            self.create_polygon(flat_P, fill='#ffcccc', outline='red', width=2)
