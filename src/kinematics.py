# Kinematics utilities for stewart's platform movement.
# NOTICE: This is LLM code. Generated with Gemini 3.1 Pro.

import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import List, Tuple, Union, Optional

class StewartKinematicSolver:
    """
    Inverse Kinematics Solver for a Stewart Platform.
    All distance units are assumed to be in millimeters (mm) and angles in radians.
    """

    def __init__(
        self,
        base_joints: List[np.ndarray],
        platform_joints: List[np.ndarray],
        motor_rotations: List[float],
        rod_length: float,
        horn_length: float,
        servo_range: Tuple[float, float] = (-math.pi / 2, math.pi / 2),
        absolute_height: bool = False
    ):
        """
        Initialize the Stewart Platform kinematic solver.
        
        :param base_joints: List of 6 3D coordinates [x, y, z] for base mount points (mm).
        :param platform_joints: List of 6 3D coordinates [x, y, z] for platform mount points (mm).
        :param motor_rotations: List of 6 angles (radians) representing the pan angle of each motor on the base.
        :param rod_length: Length of the connecting rods (mm).
        :param horn_length: Length of the servo horns (mm).
        :param servo_range: Tuple of (min_angle, max_angle) in radians for the servos.
        :param absolute_height: If False, automatically calculates the resting Z-height (T0).
        """
        assert len(base_joints) == 6, "Must provide exactly 6 base joints"
        assert len(platform_joints) == 6, "Must provide exactly 6 platform joints"
        assert len(motor_rotations) == 6, "Must provide exactly 6 motor rotations"

        self.B = np.array(base_joints, dtype=float)
        self.P = np.array(platform_joints, dtype=float)
        self.beta = np.array(motor_rotations, dtype=float)
        self.sin_beta = np.sin(self.beta)
        self.cos_beta = np.cos(self.beta)
        
        self.rod_length = rod_length
        self.horn_length = horn_length
        self.servo_range = servo_range

        # Calculate base offset (T0)
        if absolute_height:
            self.T0 = np.array([0.0, 0.0, 0.0])
        else:
            # Calculate resting z-height based on rod and horn lengths
            dx = self.P[0][0] - self.B[0][0]
            dy = self.P[0][1] - self.B[0][1]
            z_sq = (rod_length**2 + horn_length**2) - (dx**2 + dy**2)
            
            if z_sq < 0:
                raise ValueError("Platform geometry is physically impossible with these rod/horn lengths.")
                
            self.T0 = np.array([0.0, 0.0, math.sqrt(z_sq)])

    def solve(self, translation: Union[List[float], np.ndarray], rotation: R) -> Optional[np.ndarray]:
        """
        Calculates the required servo horn angles to achieve the requested translation and rotation.
        
        :param translation: [x, y, z] translation vector in mm.
        :param rotation: scipy.spatial.transform.Rotation object representing the platform's orientation.
        :return: A numpy array of 6 servo angles in radians, or None if the position is unreachable.
        """
        translation = np.array(translation, dtype=float)
        
        # Calculate rotated platform joints
        P_rotated = rotation.apply(self.P)
        
        # Calculate platform joint position in base frame (q)
        # q_i = T + R * P_i + T0
        q = translation + P_rotated + self.T0
        
        # Vector from Base joint to Platform joint (l)
        l = q - self.B
        
        angles = np.zeros(6)
        
        for i in range(6):
            lx, ly, lz = l[i]
            
            gk = lx**2 + ly**2 + lz**2 - self.rod_length**2 + self.horn_length**2
            ek = 2 * self.horn_length * lz
            fk = 2 * self.horn_length * (self.cos_beta[i] * lx + self.sin_beta[i] * ly)
            
            sq_sum = ek**2 + fk**2
            
            # Check if geometry allows a valid rod intersection
            val_under_sqrt = 1 - (gk**2 / sq_sum)
            if val_under_sqrt < 0:
                return None  # Unreachable position (rod is too short/long)
                
            sqrt1 = math.sqrt(val_under_sqrt)
            sqrt2 = math.sqrt(sq_sum)
            
            sin_alpha = (gk * ek) / sq_sum - (fk * sqrt1) / sqrt2
            cos_alpha = (gk * fk) / sq_sum + (ek * sqrt1) / sqrt2
            
            # Use atan2 for robust angle resolution instead of raw arcsin
            alpha = math.atan2(sin_alpha, cos_alpha)
            
            # Constrain to servo limits
            if not (self.servo_range[0] <= alpha <= self.servo_range[1]):
                return None  # Out of mechanical bounds
                
            angles[i] = alpha
            
        return angles

    def is_reachable(self, translation: Union[List[float], np.ndarray], rotation: R) -> bool:
        """
        Checks if a specific pose can be mechanically reached by the platform.
        """
        return self.solve(translation, rotation) is not None

    @classmethod
    def create_hexagonal(
        cls,
        base_radius: float = 97.342,
        base_radius_outer: float = 110.0,
        platform_radius: float = 81.293,
        platform_radius_outer: float = 123.198,
        rod_length: float = 169.00,
        horn_length: float = 82.50,
        horn_direction: int = 0,
        shaft_distance: float = 30.00,
        anchor_distance: float = 112.50,
        platform_turn: bool = True,
        servo_range: Tuple[float, float] = (-math.pi / 2, math.pi / 2)
    ):
        """
        Factory method to generate a Hexagonal Stewart Platform configuration.
        Matches `initHexagonal` behavior from the JS library.
        """
        def _get_hex_plate(r_i, r_o, rot):
            ret = []
            a_2 = (2 * r_i - r_o) / math.sqrt(3)
            for i in range(6):
                phi = (i - i % 2) / 3 * math.pi + rot
                ap = a_2 * (-1 if (i & 1) else 1)
                ret.append([
                    r_o * math.cos(phi) + ap * math.sin(phi),
                    r_o * math.sin(phi) - ap * math.cos(phi)
                ])
            return np.array(ret)

        base_ints = _get_hex_plate(base_radius, base_radius_outer, 0)
        plat_ints = _get_hex_plate(platform_radius, platform_radius_outer, math.pi if platform_turn else 0)

        base_points = []
        plat_points = []
        motor_angles = []

        for i in range(6):
            mid_k = i | 1
            base_c = base_ints[mid_k]
            base_n = base_ints[(mid_k + 1) % 6]
            plat_c = plat_ints[mid_k]
            plat_n = plat_ints[(mid_k + 1) % 6]

            base_d = base_n - base_c
            len_base_side = np.linalg.norm(base_d)
            base_d /= len_base_side

            pm = -1 if (i & 1) else 1

            base_mid = (base_c + base_n) / 2
            plat_mid = (plat_c + plat_n) / 2

            base_points.append([
                base_mid[0] + base_d[0] * shaft_distance * pm,
                base_mid[1] + base_d[1] * shaft_distance * pm,
                0.0
            ])
            plat_points.append([
                plat_mid[0] + base_d[0] * anchor_distance * pm,
                plat_mid[1] + base_d[1] * anchor_distance * pm,
                0.0
            ])
            motor_angles.append(math.atan2(base_d[1], base_d[0]) + ((i + horn_direction) % 2) * math.pi)

        plat_index = [4, 3, 0, 5, 2, 1] if platform_turn else [0, 1, 2, 3, 4, 5]
        plat_points_reordered = [plat_points[idx] for idx in plat_index]

        return cls(
            base_joints=base_points,
            platform_joints=plat_points_reordered,
            motor_rotations=motor_angles,
            rod_length=rod_length,
            horn_length=horn_length,
            servo_range=servo_range
        )


if __name__ == "__main__":
    # --- Quick Start Example ---
    
    # 1. Initialize using the Hexagonal factory with default mm values
    solver = StewartKinematicSolver.create_hexagonal(
        base_radius=80, 
        platform_radius=50, 
        rod_length=130, 
        horn_length=50
    )

    # 2. Define the desired position (jog/jump to)
    # E.g., Move up 20mm, shift left 15mm, tilt 10 degrees on the X-axis
    target_translation = [-15.0, 0.0, 20.0] 
    
    # Using Scipy Rotation allows you to easily specify euler angles (degrees or radians)
    target_rotation = R.from_euler('xyz', [10, 0, 0], degrees=True)

    # 3. Check reachability before executing
    if solver.is_reachable(target_translation, target_rotation):
        
        # 4. Solve for servo angles (Returns array in radians)
        servo_angles_rad = solver.solve(target_translation, target_rotation)
        
        # Convert to degrees if your motors require it
        servo_angles_deg = np.degrees(servo_angles_rad)
        
        print("Success! Target is reachable.")
        print("Required Servo Angles (Degrees):")
        for i, angle in enumerate(servo_angles_deg):
            print(f"  Servo {i + 1}: {angle:6.2f}°")
            
    else:
        print("Error: Target position is out of mechanical bounds.")