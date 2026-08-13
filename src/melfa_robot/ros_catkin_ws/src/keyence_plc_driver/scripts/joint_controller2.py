#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk
import time
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
        
        # Joint limits in degrees
        self.joint_limits = np.array([
            [-240, 240],   # j1
            [-120, 120],   # j2
            [0, 161],      # j3
            [-200, 200],   # j4
            [-120, 120]    # j5
        ])
        
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
        T = self.A0(t1) @ self.A1(t2) @ self.A2(t3) @ self.A3(t4) @ self.A4(t5)
        pos_homogeneous = T @ np.array([0.0, 0.0, 124.85, 1.0])
        return pos_homogeneous[:3]
    
    def compute_jacobian(self, t1, t2, t3, t4, t5):
        """Compute the Jacobian matrix (3x5) - angles in DEGREES"""
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
        
        s23 = math.sin(t2_rad + t3_rad)
        c23 = math.cos(t2_rad + t3_rad)
        K = 124.85 * c5 + 335.15
        
        J = np.zeros((3, 5))
        
        J[0, 0] = c1 * (124.85 * s5 * (c4 * c23 - s4) + K * s23 - 50 * c23 + 310 * s2) * math.pi / 180
        J[1, 0] = (c1 * (124.85 * s5 * c4 * c23 + K * s23 - 50 * c23 + 310 * s2) - 124.85 * s5 * s4 * s1) * math.pi / 180
        J[2, 0] = 0
        
        J[0, 1] = s1 * (-124.85 * s5 * c4 * s23 + K * c23 + 50 * s23 + 310 * c2) * math.pi / 180
        J[1, 1] = s1 * (-124.85 * s5 * c4 * s23 + K * c23 + 50 * s23 + 310 * c2) * math.pi / 180
        J[2, 1] = (-124.85 * s5 * c4 * c23 - K * s23 + 50 * c23 - 310 * s2) * math.pi / 180
        
        J[0, 2] = s1 * (-124.85 * s5 * c4 * s23 + K * c23 + 50 * s23) * math.pi / 180
        J[1, 2] = s1 * (-124.85 * s5 * c4 * s23 + K * c23 + 50 * s23) * math.pi / 180
        J[2, 2] = (-124.85 * s5 * c4 * c23 - K * s23 + 50 * c23) * math.pi / 180
        
        J[0, 3] = s1 * 124.85 * s5 * (-s4 * c23 - c4) * math.pi / 180
        J[1, 3] = (s1 * (-124.85 * s5 * s4 * c23) + 124.85 * s5 * c4 * c1) * math.pi / 180
        J[2, 3] = 124.85 * s5 * s4 * s23 * math.pi / 180
        
        J[0, 4] = s1 * (124.85 * c5 * (c4 * c23 - s4) - 124.85 * s5 * s23) * math.pi / 180
        J[1, 4] = (s1 * (124.85 * c5 * c4 * c23 - 124.85 * s5 * s23) + 124.85 * c5 * s4 * c1) * math.pi / 180
        J[2, 4] = (-124.85 * c5 * c4 * s23 - 124.85 * s5 * c23) * math.pi / 180
        
        return J
    
    def inverse_kinematics(self, target_pos, initial_guess=None, max_iterations=50, 
                          tolerance=1.0, damping=0.05, joint_weight=0.001, control_axis=None):
        """Solve inverse kinematics using damped least squares (OPTIMIZED)
        
        Args:
            control_axis: 0=X, 1=Y, 2=Z - adjusts joint preferences based on which axis is being controlled
        """
        if initial_guess is None:
            theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            theta = np.array(initial_guess, dtype=float)
        
        target = np.array(target_pos)
        theta_start = theta.copy()
        
        lambda_factor = damping
        best_error = float('inf')
        best_theta = theta.copy()
        
        # Pre-allocate matrices
        eye5 = np.eye(5)
        
        # Joint preference weights based on control axis
        if control_axis == 1:  # Y-axis: prefer joint1, joint3, joint4
            joint_weights = np.array([0.5, 2.0, 0.8, 0.8, 1.5])
            # j1 preferred, j2 penalized, j3&j4 preferred, j5 slightly penalized
        elif control_axis in [0, 2]:  # X or Z-axis: prefer joint2, joint3
            joint_weights = np.array([2.5, 0.7, 0.7, 2.0, 1.5])
            # j1 penalized, j2&j3 preferred, j4 penalized, j5 slightly penalized
        else:  # Default: balanced
            joint_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
        
        W = np.diag(joint_weights)
        
        for iteration in range(max_iterations):
            current_pos = self.forward_kinematics(*theta)
            position_error = target - current_pos
            error_norm = np.linalg.norm(position_error)
            
            if error_norm < best_error:
                best_error = error_norm
                best_theta = theta.copy()
            
            # Early exit with relaxed tolerance
            if error_norm < tolerance:
                return theta, True
            
            J = self.compute_jacobian(*theta)
            
            # Weighted damped least squares
            JTJ = J.T @ J + (joint_weight * W + lambda_factor * eye5)
            
            try:
                delta_theta = np.linalg.solve(JTJ, J.T @ position_error)
            except np.linalg.LinAlgError:
                return best_theta, False
            
            # Simpler step size
            step_size = 0.8 if error_norm > 50 else 1.0
            
            theta += step_size * delta_theta
            
            # Apply joint limits
            for i in range(5):
                theta[i] = np.clip(theta[i], self.joint_limits[i, 0], self.joint_limits[i, 1])
            
            # Angle wrapping (only for joints without strict limits)
            # theta = np.where(theta > 180, theta - 360, theta)
            # theta = np.where(theta < -180, theta + 360, theta)
        
        return best_theta, best_error < 10.0
    
    def move_to_position(self, current_angles, target_position, max_error=10.0, control_axis=None):
        """Calculate new joint angles to move to target position (OPTIMIZED)
        
        Args:
            control_axis: 0=X, 1=Y, 2=Z - which axis is being controlled
        """
        # Single attempt with current position as initial guess
        new_angles, success = self.inverse_kinematics(
            target_position, 
            initial_guess=current_angles,
            tolerance=1.0,  # Relaxed tolerance
            max_iterations=50,  # Fewer iterations
            control_axis=control_axis
        )
        
        reached_pos = self.forward_kinematics(*new_angles)
        error = np.linalg.norm(np.array(target_position) - reached_pos)
        
        success = error <= max_error
        return new_angles, success, error


class RobotControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm IK Controller")
        self.root.geometry("700x600")

        # Initialize IK solver
        self.ik_solver = RobotArmIK()
        
        # Joint names (first 5 joints for IK)
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # Joint limits in radians (for joints 1-5, joint6 has no limit specified)
        self.joint_limits_rad = [
            [math.radians(-240), math.radians(240)],  # j1
            [math.radians(-120), math.radians(120)],  # j2
            [math.radians(0), math.radians(161)],     # j3
            [math.radians(-200), math.radians(200)],  # j4
            [math.radians(-120), math.radians(120)],  # j5
            [None, None]  # j6 - no limit specified
        ]
        
        # State variables
        self.joint_positions = [0.0] * len(self.joint_names)
        self.current_joint_index = 0
        self.last_button_press_time = 0
        self.debounce_duration = 0.2
        self.control_mode = "JOINT"  # "JOINT" or "CARTESIAN"
        
        # Movement control
        self.last_movement_time = 0
        self.movement_delay = 0.05  # 50ms delay between movements (adjust this value)
        
        # Trajectory smoothing for cartesian mode
        self.target_joint_positions = [0.0] * 6  # Target positions for smooth interpolation
        self.trajectory_alpha = 0.15  # Smoothing factor: Lower = less swing (0.05-0.5)
        # 0.05-0.10 = very smooth, minimal swing
        # 0.15-0.20 = balanced
        # 0.30+ = fast but more swing
        self.last_cartesian_input_time = 0
        
        # Cartesian position
        self.current_cartesian = [0.0, 0.0, 0.0]
        self.cartesian_update_pending = False
        self.last_ik_time = 0
        self.ik_cooldown = 0.03  # Faster updates (30ms) for responsive control
        
        # Track if we're actively controlling in cartesian mode
        self.cartesian_control_active = False
        self.cartesian_control_timeout = 0.5  # Stop overriding after 0.5s of no input
        
        # ROS Publishers & Subscribers
        self.command_pub = rospy.Publisher(
            "/joint_trajectory_controller/command",
            JointTrajectory, 
            queue_size=10
        )
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Build GUI
        self.setup_gui()
        
        rospy.loginfo("Robot IK Controller started.")
        rospy.loginfo("L1/R1: Switch control mode | Triangle/Cross: Switch joint/axis | Circle/Square: Increase/Decrease")

    def setup_gui(self):
        """Build the GUI interface"""
        # Mode selector
        mode_frame = tk.Frame(self.root, padx=10, pady=10)
        mode_frame.pack()
        
        tk.Label(mode_frame, text="Control Mode:", font=("Arial", 14, "bold")).pack(side=tk.LEFT, padx=5)
        self.mode_label = tk.Label(mode_frame, text="JOINT", font=("Arial", 14, "bold"), fg="green")
        self.mode_label.pack(side=tk.LEFT, padx=5)
        
        # Separator
        ttk.Separator(self.root, orient='horizontal').pack(fill='x', pady=10)
        
        # Joint control panel
        joint_frame = tk.LabelFrame(self.root, text="Joint Angles (rad)", padx=10, pady=10)
        joint_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.joint_labels = {}
        for i, name in enumerate(self.joint_names):
            row_frame = tk.Frame(joint_frame)
            row_frame.pack(fill='x', pady=2)
            
            tk.Label(row_frame, text=name, font=("Arial", 11, "bold"), width=10, anchor='w').pack(side=tk.LEFT)
            self.joint_labels[name] = tk.Label(row_frame, text=f"{self.joint_positions[i]:.4f}", 
                                               font=("Arial", 11), width=12, anchor='e')
            self.joint_labels[name].pack(side=tk.LEFT)
        
        # Cartesian control panel
        cartesian_frame = tk.LabelFrame(self.root, text="Cartesian Position (mm)", padx=10, pady=10)
        cartesian_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.cartesian_labels = {}
        for axis in ['X', 'Y', 'Z']:
            row_frame = tk.Frame(cartesian_frame)
            row_frame.pack(fill='x', pady=2)
            
            tk.Label(row_frame, text=axis, font=("Arial", 11, "bold"), width=10, anchor='w').pack(side=tk.LEFT)
            self.cartesian_labels[axis] = tk.Label(row_frame, text="0.00", 
                                                    font=("Arial", 11), width=12, anchor='e')
            self.cartesian_labels[axis].pack(side=tk.LEFT)
        
        # Selected indicator
        self.selected_label = tk.Label(self.root, text="", font=("Arial", 13, "bold"), fg="blue")
        self.selected_label.pack(pady=10)
        
        # Status bar
        self.status_label = tk.Label(self.root, text="Ready", font=("Arial", 10), fg="gray", anchor='w')
        self.status_label.pack(fill='x', padx=10, pady=5)

    def joint_states_callback(self, msg):
        """Updates joint positions from /joint_states topic"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
        
        # Only update cartesian if not actively controlling in cartesian mode
        current_time = time.time()
        if not self.cartesian_update_pending:
            # Check if recent cartesian input (within timeout window)
            if current_time - self.last_cartesian_input_time > self.cartesian_control_timeout:
                # Safe to update from real position
                angles_deg = [math.degrees(self.joint_positions[i]) for i in range(5)]
                self.current_cartesian = self.ik_solver.forward_kinematics(*angles_deg).tolist()
            # Otherwise keep the user's commanded cartesian position

    def joy_callback(self, msg):
        """Handles PS4 controller input"""
        button_cross = msg.buttons[0]
        button_circle = msg.buttons[1]
        button_triangle = msg.buttons[2]
        button_square = msg.buttons[3]
        button_l1 = msg.buttons[4]
        button_r1 = msg.buttons[5]
        
        current_time = time.time()
        
        # Switch control mode with L1/R1
        if current_time - self.last_button_press_time > self.debounce_duration:
            if button_l1 == 1 or button_r1 == 1:
                self.control_mode = "CARTESIAN" if self.control_mode == "JOINT" else "JOINT"
                self.mode_label.config(text=self.control_mode)
                self.last_button_press_time = current_time
                rospy.loginfo(f"Switched to {self.control_mode} mode")
            
            # Switch selected joint/axis
            if button_triangle == 1:
                self.current_joint_index = (self.current_joint_index - 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time
            elif button_cross == 1:
                self.current_joint_index = (self.current_joint_index + 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time

        # Move joint or axis
        moved = False
        
        if self.control_mode == "JOINT":
            # Rate limit joint movements
            if current_time - self.last_movement_time < self.movement_delay:
                return
                
            step = 0.003  # Smaller step for smoother control (adjust this value)
            if button_circle == 1:
                new_position = self.joint_positions[self.current_joint_index] + step
                # Check joint limits
                limits = self.joint_limits_rad[self.current_joint_index]
                if limits[0] is not None and limits[1] is not None:
                    if new_position <= limits[1]:
                        self.joint_positions[self.current_joint_index] = new_position
                        moved = True
                    else:
                        self.status_label.config(text=f"{self.joint_names[self.current_joint_index]} at upper limit", fg="orange")
                else:
                    self.joint_positions[self.current_joint_index] = new_position
                    moved = True
                self.last_movement_time = current_time
                
            elif button_square == 1:
                new_position = self.joint_positions[self.current_joint_index] - step
                # Check joint limits
                limits = self.joint_limits_rad[self.current_joint_index]
                if limits[0] is not None and limits[1] is not None:
                    if new_position >= limits[0]:
                        self.joint_positions[self.current_joint_index] = new_position
                        moved = True
                    else:
                        self.status_label.config(text=f"{self.joint_names[self.current_joint_index]} at lower limit", fg="orange")
                else:
                    self.joint_positions[self.current_joint_index] = new_position
                    moved = True
                self.last_movement_time = current_time
                
            if moved:
                self.send_command()
                
        else:  # CARTESIAN mode
            # Rate limit cartesian movements with IK cooldown
            if current_time - self.last_ik_time < self.ik_cooldown:
                return
            
            step = 0.5  # Small step size (0.5mm) for fine control
            if button_circle == 1:
                self.current_cartesian[self.current_joint_index] += step
                moved = True
                self.last_cartesian_input_time = current_time  # Track user input
            elif button_square == 1:
                self.current_cartesian[self.current_joint_index] -= step
                moved = True
                self.last_cartesian_input_time = current_time  # Track user input
            
            if moved:
                self.last_ik_time = current_time
                self.cartesian_update_pending = True
                
                # Solve IK to get joint angles
                current_angles_deg = [math.degrees(self.joint_positions[i]) for i in range(5)]
                new_angles, success, error = self.ik_solver.move_to_position(
                    current_angles_deg, 
                    self.current_cartesian,
                    control_axis=self.current_joint_index  # Pass which axis is being controlled
                )
                
                # Check if any joint would exceed limits
                joints_within_limits = True
                for i in range(5):
                    angle_rad = math.radians(new_angles[i])
                    limits = self.joint_limits_rad[i]
                    if angle_rad < limits[0] or angle_rad > limits[1]:
                        joints_within_limits = False
                        break
                
                if success and joints_within_limits:
                    # Smooth interpolation to prevent jerky movements
                    for i in range(5):
                        target_pos = math.radians(new_angles[i])
                        # Blend between current and target position
                        self.joint_positions[i] += self.trajectory_alpha * (target_pos - self.joint_positions[i])
                    
                    self.status_label.config(text=f"IK OK ({error:.1f}mm)", fg="green")
                    self.send_command()
                else:
                    # Revert cartesian position - can't move further
                    if button_circle == 1:
                        self.current_cartesian[self.current_joint_index] -= step
                    elif button_square == 1:
                        self.current_cartesian[self.current_joint_index] += step
                    
                    if not joints_within_limits:
                        self.status_label.config(text=f"Joint limit reached!", fg="orange")
                    else:
                        self.status_label.config(text=f"IK Failed ({error:.1f}mm)", fg="red")
                
                self.cartesian_update_pending = False

    def send_command(self):
        """Publish joint trajectory command"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(0.2)  # Slower trajectory execution (was 0.1)
        
        msg.points.append(point)
        self.command_pub.publish(msg)

    def update_gui(self):
        """Periodically refresh the GUI (optimized)"""
        # Update joint labels
        for i, name in enumerate(self.joint_names):
            self.joint_labels[name].config(text=f"{self.joint_positions[i]:.3f}")
        
        # Update cartesian labels
        for i, axis in enumerate(['X', 'Y', 'Z']):
            self.cartesian_labels[axis].config(text=f"{self.current_cartesian[i]:.1f}")
        
        # Update selection indicator
        if self.control_mode == "JOINT":
            selected = self.joint_names[self.current_joint_index]
            self.selected_label.config(text=f"Selected: {selected}")
            
            # Highlight selected joint
            for name, label in self.joint_labels.items():
                label.config(fg="blue" if name == selected else "black")
            for label in self.cartesian_labels.values():
                label.config(fg="black")
        else:  # CARTESIAN
            axes = ['X', 'Y', 'Z']
            selected = axes[self.current_joint_index]
            self.selected_label.config(text=f"Selected: {selected} axis")
            
            # Highlight selected axis
            for axis, label in self.cartesian_labels.items():
                label.config(fg="blue" if axis == selected else "black")
            for label in self.joint_labels.values():
                label.config(fg="black")
        
        self.root.after(150, self.update_gui)  # Slower GUI update (150ms)


def main():
    rospy.init_node("robot_ik_controller")
    root = tk.Tk()
    gui = RobotControllerGUI(root)
    gui.update_gui()
    root.mainloop()

if __name__ == "__main__":
    main()
