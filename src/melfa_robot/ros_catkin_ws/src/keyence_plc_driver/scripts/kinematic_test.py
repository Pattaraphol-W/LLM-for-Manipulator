#!/usr/bin/env python3
import numpy as np
import math

class RobotArmIK:
    def __init__(self):
        # Robot arm constants
        self.L1 = 350.0
        self.L2 = 310.0
        self.L3 = 50.0
        self.L4 = 285.15
        self.L5 = 124.85
        
    def A0(self, t1):
        """Transformation matrix A0"""
        t1_rad = math.radians(t1)
        return np.array([
            [math.cos(t1_rad), -math.sin(t1_rad), 0.0, 0.0],
            [math.sin(t1_rad),  math.cos(t1_rad), 0.0, 0.0],
            [0.0,               0.0,               1.0, 0.0],
            [0.0,               0.0,               0.0, 1.0]
        ])
    
    def A1(self, t2):
        """Transformation matrix A1"""
        t2_rad = math.radians(t2)
        return np.array([
            [math.cos(t2_rad),  0.0, math.sin(t2_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t2_rad), 0.0, math.cos(t2_rad), 350.0],
            [0.0,               0.0, 0.0,              1.0]
        ])
    
    def A2(self, t3):
        """Transformation matrix A2"""
        t3_rad = math.radians(t3)
        return np.array([
            [math.cos(t3_rad),  0.0, math.sin(t3_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t3_rad), 0.0, math.cos(t3_rad), 310.0],
            [0.0,               0.0, 0.0,              1.0]
        ])
    
    def A3(self, t4):
        """Transformation matrix A3"""
        t4_rad = math.radians(t4)
        return np.array([
            [math.cos(t4_rad), -math.sin(t4_rad), 0.0, -50.0],
            [math.sin(t4_rad),  math.cos(t4_rad), 0.0, 0.0],
            [0.0,               0.0,              1.0, 50.0],
            [0.0,               0.0,              0.0, 1.0]
        ])
    
    def A4(self, t5):
        """Transformation matrix A4"""
        t5_rad = math.radians(t5)
        return np.array([
            [math.cos(t5_rad),  0.0, math.sin(t5_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t5_rad), 0.0, math.cos(t5_rad), 285.15],
            [0.0,               0.0, 0.0,              1.0]
        ])
    
    def forward_kinematics(self, t1, t2, t3, t4, t5):
        """Calculate end-effector position given joint angles (in DEGREES)"""
        # Multiply all transformation matrices
        T = self.A0(t1) @ self.A1(t2) @ self.A2(t3) @ self.A3(t4) @ self.A4(t5)
        
        # Apply to end-effector offset
        pos_homogeneous = T @ np.array([0.0, 0.0, 124.85, 1.0])
        
        # Return x, y, z position
        return pos_homogeneous[:3]
    
    def compute_jacobian(self, t1, t2, t3, t4, t5):
        """Compute the Jacobian matrix (3x5) - angles in DEGREES"""
        # Convert to radians for computation
        t1_rad = math.radians(t1)
        t2_rad = math.radians(t2)
        t3_rad = math.radians(t3)
        t4_rad = math.radians(t4)
        t5_rad = math.radians(t5)
        
        s1, c1 = math.sin(t1_rad), math.cos(t1_rad)
        s2, c2 = math.sin(t2_rad), math.cos(t2_rad)
        s3, c3 = math.sin(t3_rad), math.cos(t3_rad)
        s4, c4 = math.sin(t4_rad), math.cos(t4_rad)
        s5, c5 = math.sin(t5_rad), math.cos(t5_rad)
        
        # Pre-compute common terms
        s23 = math.sin(t2_rad + t3_rad)
        c23 = math.cos(t2_rad + t3_rad)
        K = 124.85 * c5 + 335.15
        
        J = np.zeros((3, 5))
        
        # ∂/∂t1 (convert to per-degree by multiplying by π/180)
        J[0, 0] = c1 * (124.85 * s5 * (c4 * c23 - s4) + K * s23 - 50 * c23 + 310 * s2) * math.pi / 180
        J[1, 0] = (c1 * (124.85 * s5 * c4 * c23 + K * s23 - 50 * c23 + 310 * s2) - 124.85 * s5 * s4 * s1) * math.pi / 180
        J[2, 0] = 0
        
        # ∂/∂t2
        J[0, 1] = s1 * (-124.85 * s5 * c4 * s23 + K * c23 + 50 * s23 + 310 * c2) * math.pi / 180
        J[1, 1] = s1 * (-124.85 * s5 * c4 * s23 + K * c23 + 50 * s23 + 310 * c2) * math.pi / 180
        J[2, 1] = (-124.85 * s5 * c4 * c23 - K * s23 + 50 * c23 - 310 * s2) * math.pi / 180
        
        # ∂/∂t3
        J[0, 2] = s1 * (-124.85 * s5 * c4 * s23 + K * c23 + 50 * s23) * math.pi / 180
        J[1, 2] = s1 * (-124.85 * s5 * c4 * s23 + K * c23 + 50 * s23) * math.pi / 180
        J[2, 2] = (-124.85 * s5 * c4 * c23 - K * s23 + 50 * c23) * math.pi / 180
        
        # ∂/∂t4
        J[0, 3] = s1 * 124.85 * s5 * (-s4 * c23 - c4) * math.pi / 180
        J[1, 3] = (s1 * (-124.85 * s5 * s4 * c23) + 124.85 * s5 * c4 * c1) * math.pi / 180
        J[2, 3] = 124.85 * s5 * s4 * s23 * math.pi / 180
        
        # ∂/∂t5
        J[0, 4] = s1 * (124.85 * c5 * (c4 * c23 - s4) - 124.85 * s5 * s23) * math.pi / 180
        J[1, 4] = (s1 * (124.85 * c5 * c4 * c23 - 124.85 * s5 * s23) + 124.85 * c5 * s4 * c1) * math.pi / 180
        J[2, 4] = (-124.85 * c5 * c4 * s23 - 124.85 * s5 * c23) * math.pi / 180
        
        return J
    
    def inverse_kinematics(self, target_pos, initial_guess=None, max_iterations=200, 
                          tolerance=0.01, damping=0.05, joint_weight=0.001):
        """
        Solve inverse kinematics using damped least squares (Levenberg-Marquardt)
        with joint deviation penalty for smooth micro-movements
        
        Args:
            target_pos: Target position [x, y, z]
            initial_guess: Initial joint angles [t1, t2, t3, t4, t5] (DEGREES)
            max_iterations: Maximum number of iterations
            tolerance: Position error tolerance (0.01mm = very accurate)
            damping: Damping factor for stability
            joint_weight: Weight for penalizing large joint changes (higher = prefer smaller movements)
            
        Returns:
            joint_angles: Solution [t1, t2, t3, t4, t5] in DEGREES
            success: Boolean indicating if solution was found
        """
        # Initialize joint angles in degrees
        if initial_guess is None:
            theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            theta = np.array(initial_guess, dtype=float)
        
        target = np.array(target_pos)
        theta_start = theta.copy()  # Remember starting position
        
        # Adaptive parameters
        lambda_factor = damping
        best_error = float('inf')
        best_theta = theta.copy()
        stuck_count = 0
        
        for iteration in range(max_iterations):
            # Compute current position
            current_pos = self.forward_kinematics(*theta)
            
            # Compute position error
            position_error = target - current_pos
            error_norm = np.linalg.norm(position_error)
            
            # Compute joint deviation from start (penalty for large movements)
            joint_deviation = theta - theta_start
            joint_deviation_norm = np.linalg.norm(joint_deviation)
            
            # Combined error (position error + small penalty for joint movement)
            total_error = error_norm + joint_weight * joint_deviation_norm
            
            # Track best solution based on position error only
            if error_norm < best_error:
                best_error = error_norm
                best_theta = theta.copy()
                stuck_count = 0
            else:
                stuck_count += 1
            
            # Check convergence (very tight tolerance)
            if error_norm < tolerance:
                return theta, True
            
            # If stuck, try to escape
            if stuck_count > 15:
                lambda_factor *= 1.5  # Increase damping
                stuck_count = 0
            
            # Compute Jacobian
            J = self.compute_jacobian(*theta)
            
            # Modified cost function: minimize both position error AND joint movement
            # Add small identity matrix to prefer staying close to current angles
            JTJ = J.T @ J + joint_weight * np.eye(5)
            damping_matrix = lambda_factor * np.eye(5)
            
            try:
                # Compute update with both position and joint deviation penalty
                delta_theta = np.linalg.solve(JTJ + damping_matrix, J.T @ position_error)
            except np.linalg.LinAlgError:
                # If singular, use standard pseudo-inverse
                delta_theta = np.linalg.pinv(J) @ position_error
            
            # Adaptive step size based on position error
            step_size = 1.0
            if error_norm > 100:
                step_size = 0.3  # Very slow when far away
            elif error_norm > 50:
                step_size = 0.5
            elif error_norm > 10:
                step_size = 0.7
            else:
                step_size = 0.9  # Almost full speed when close
            
            # Update joint angles (in degrees)
            theta += step_size * delta_theta
            
            # Keep angles in reasonable range [-360, 360]
            theta = np.where(theta > 180, theta - 360, theta)
            theta = np.where(theta < -180, theta + 360, theta)
            
            # Reduce damping if making progress
            if error_norm < best_error * 0.9:
                lambda_factor = max(damping * 0.5, lambda_factor * 0.8)
        
        # Return best solution found
        final_pos = self.forward_kinematics(*best_theta)
        final_error = np.linalg.norm(target - final_pos)
        
        # Accept if error is less than 5mm
        if final_error < 5.0:
            return best_theta, True
        
        return best_theta, False
    
    def move_to_position(self, current_angles, target_position, max_error=5.0):
        """
        MAIN FUNCTION: Calculate new joint angles to move to target position
        
        Args:
            current_angles: Current joint angles [t1, t2, t3, t4, t5] in DEGREES
            target_position: Desired position [x, y, z] in mm
            max_error: Maximum acceptable position error in mm (default 5.0mm)
            
        Returns:
            new_angles: New joint angles [t1, t2, t3, t4, t5] in DEGREES
            success: True if solution found within max_error, False otherwise
            error: Position error in mm
        """
        # Try with current angles as initial guess first
        new_angles, success = self.inverse_kinematics(
            target_position, 
            initial_guess=current_angles,
            tolerance=0.01,  # Very tight tolerance
            max_iterations=200
        )
        
        # Calculate actual reached position
        reached_pos = self.forward_kinematics(*new_angles)
        error = np.linalg.norm(np.array(target_position) - reached_pos)
        
        # If error is acceptable, return
        if error <= max_error:
            return new_angles, True, error
        
        # If not successful, try with different initial guesses
        print(f"First attempt error: {error:.2f}mm, trying alternative solutions...")
        
        best_angles = new_angles
        best_error = error
        
        # Try multiple starting positions
        alternative_guesses = [
            [0, 45, 45, 0, 0],
            [0, 30, 30, 0, 0],
            [45, 30, 30, 0, 0],
            [-45, 30, 30, 0, 0],
            [0, 60, 20, 0, 0],
        ]
        
        for guess in alternative_guesses:
            angles, success = self.inverse_kinematics(
                target_position,
                initial_guess=guess,
                tolerance=0.01,
                max_iterations=200
            )
            
            pos = self.forward_kinematics(*angles)
            err = np.linalg.norm(np.array(target_position) - pos)
            
            if err < best_error:
                best_error = err
                best_angles = angles
                
                if err <= max_error:
                    return best_angles, True, best_error
        
        # Return best attempt
        success = best_error <= max_error
        return best_angles, success, best_error


# Example usage
if __name__ == "__main__":
    robot = RobotArmIK()
    print("="*60)
    current_angles = [4.49, 13.745, 155.175, 3.467, 44.25]  # degrees
    print(f"Current angles (degrees): {current_angles}")
    current_pos = robot.forward_kinematics(*current_angles)
    print(f"Current position: [{current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}] mm")
    
    print("="*60)
    
    target_position = [537.12, 73.17, 800.0]  # [x, y, z] in mm
    print(f"Target position: [{target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f}] mm")
    
    # Calculate new angles
    new_angles, success, error = robot.move_to_position(current_angles, target_position)
    
    print(f"\n--- RESULT ---")
    if success:
        print(f"✓ Solution found! (error: {error:.3f} mm)")
    else:
        print(f"✗ Could not achieve ±5mm accuracy (error: {error:.3f} mm)")
    
    print(f"New angles (degrees): {new_angles}")
