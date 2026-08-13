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
        # Joint limits in radians
        self.joint_limits = np.array([
            [math.radians(-240), math.radians(240)],  # j1
            [math.radians(-120), math.radians(120)],  # j2
            [math.radians(0),    math.radians(161)],  # j3
            [math.radians(-200), math.radians(200)],  # j4
            [math.radians(-120), math.radians(120)]   # j5
        ])
        
        # PERSISTENT direction tracking across IK calls
        self.locked_direction = None
        self.last_q = None
    
    def forward_kinematics(self, joint_angles_rad):
        """Calculate FK using A-matrices
        
        Args:
            joint_angles_rad: List of 5 joint angles in radians
            
        Returns:
            [x, y, z] in mm
        """
        # Convert to degrees for A-matrix functions
        joint_angles_deg = [math.degrees(a) for a in joint_angles_rad]
        
        t1, t2, t3, t4, t5 = joint_angles_deg
        
        # A0
        t1_rad = math.radians(t1)
        A0 = np.array([
            [math.cos(t1_rad), -math.sin(t1_rad), 0.0, 0.0],
            [math.sin(t1_rad),  math.cos(t1_rad), 0.0, 0.0],
            [0.0,               0.0,               1.0, 0.0],
            [0.0,               0.0,               0.0, 1.0]
        ])
        
        # A1
        t2_rad = math.radians(t2)
        A1 = np.array([
            [math.cos(t2_rad),  0.0, math.sin(t2_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t2_rad), 0.0, math.cos(t2_rad), 350.0],
            [0.0,               0.0, 0.0,              1.0]
        ])
        
        # A2
        t3_rad = math.radians(t3)
        A2 = np.array([
            [math.cos(t3_rad),  0.0, math.sin(t3_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t3_rad), 0.0, math.cos(t3_rad), 310.0],
            [0.0,               0.0, 0.0,              1.0]
        ])
        
        # A3
        t4_rad = math.radians(t4)
        A3 = np.array([
            [math.cos(t4_rad), -math.sin(t4_rad), 0.0, -50.0],
            [math.sin(t4_rad),  math.cos(t4_rad), 0.0, 0.0],
            [0.0,               0.0,              1.0, 50.0],
            [0.0,               0.0,              0.0, 1.0]
        ])
        
        # A4
        t5_rad = math.radians(t5)
        A4 = np.array([
            [math.cos(t5_rad),  0.0, math.sin(t5_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t5_rad), 0.0, math.cos(t5_rad), 285.15],
            [0.0,               0.0, 0.0,              1.0]
        ])
        
        # Compute total transformation
        T = A0 @ A1 @ A2 @ A3 @ A4
        
        # End effector position
        pos_homogeneous = T @ np.array([0.0, 0.0, 124.85, 1.0])
        return pos_homogeneous[:3].tolist()
    
    def inverse_kinematics(self, target_pos_mm, initial_guess_rad, control_axis=None):
        """Solve IK with MONOTONIC MOVEMENT (no reverse direction)
        
        Args:
            target_pos_mm: [x, y, z] in mm (desired end-effector position)
            initial_guess_rad: Initial joint angles in radians (5 values)
            control_axis: 0=X, 1=Y, 2=Z - which axis is being controlled
            
        Returns:
            (joint_angles_rad, success) - returns 5 joint angles
        """
        max_iterations = 250
        tolerance = 0.5  # mm
        
        # Start with initial guess
        q = np.array(initial_guess_rad)
        q_initial = q.copy()
        target = np.array(target_pos_mm)
        
        joint_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
        
        # Track initial movement direction for each joint
        initial_direction = None
        direction_locked = False
        
        for iteration in range(max_iterations):
            # Compute current position (FK in radians)
            current_pos = np.array(self.forward_kinematics(q))
            
            # Compute error
            error = target - current_pos
            error_magnitude = np.linalg.norm(error)
            
            # Check convergence
            if error_magnitude < tolerance:
                if self.check_joint_limits(q):
                    return q.tolist(), True
                else:
                    return q.tolist(), False
            
            # Compute Jacobian numerically
            J = self.compute_jacobian_numerical(q)
            
            # Weighted damped least squares
            W = np.diag(joint_weights)
            damping = 0.55  # Heavy damping for stability
            JTJ = J.T @ J + (0.05 * W + damping * np.eye(5))
            
            try:
                delta_q = np.linalg.solve(JTJ, J.T @ error)
            except:
                return initial_guess_rad, False
            
            # MONOTONIC MOVEMENT: Lock direction after first step
            if not direction_locked and np.linalg.norm(delta_q) > 1e-4:
                initial_direction = np.sign(delta_q)
                direction_locked = True
            
            # Prevent reverse motion
            if direction_locked:
                for i in range(5):
                    # If joint wants to reverse, block it
                    if initial_direction[i] * delta_q[i] < 0:
                        delta_q[i] = 0.0
            
            # Conservative step size
            max_step = 0.065  # radians - very small for smooth motion
            step_norm = np.linalg.norm(delta_q)
            if step_norm > max_step:
                delta_q = delta_q * (max_step / step_norm)
            
            q = q + delta_q
            
            # Clip to joint limits
            for i in range(5):
                q[i] = np.clip(q[i], self.joint_limits[i][0], self.joint_limits[i][1])
        
        # Failed to converge, return best attempt
        return q.tolist(), False
    
    def compute_jacobian_numerical(self, joint_angles_rad):
        """Compute Jacobian matrix numerically using finite differences
        
        Args:
            joint_angles_rad: numpy array of 5 joint angles in radians
            
        Returns:
            3x5 Jacobian matrix
        """
        epsilon = 1e-3  # Small perturbation
        J = np.zeros((3, 5))
        
        # Current position
        pos_0 = np.array(self.forward_kinematics(joint_angles_rad))
        
        # Compute each column of Jacobian
        for i in range(5):
            # Perturb joint i
            q_perturbed = joint_angles_rad.copy()
            q_perturbed[i] += epsilon
            
            pos_perturbed = np.array(self.forward_kinematics(q_perturbed))
            
            # Finite difference
            J[:, i] = (pos_perturbed - pos_0) / epsilon
        
        return J
    
    def check_joint_limits(self, joint_angles_rad):
        """Check if within limits"""
        for i in range(5):
            if joint_angles_rad[i] < self.joint_limits[i][0] or \
               joint_angles_rad[i] > self.joint_limits[i][1]:
                return False
        return True


class RobotControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm Controller (Monotonic)")
        self.root.geometry("700x600")

        # Initialize IK solver
        self.ik_solver = RobotArmIK()
        
        # Joint names
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # Joint limits in radians
        self.joint_limits_rad = [
            [math.radians(-240), math.radians(240)],
            [math.radians(-120), math.radians(120)],
            [math.radians(0), math.radians(161)],
            [math.radians(-200), math.radians(200)],
            [math.radians(-120), math.radians(120)],
            [None, None]
        ]
        
        # State variables
        self.joint_positions = [0.0] * len(self.joint_names)
        self.current_joint_index = 0
        self.last_button_press_time = 0
        self.debounce_duration = 0.2
        self.control_mode = "JOINT"
        
        # Movement control
        self.last_movement_time = 0
        self.movement_delay = 0.05
        
        # Cartesian control - ANTI-JITTER
        self.current_cartesian = [0.0, 0.0, 0.0]
        self.cartesian_update_pending = False
        self.last_ik_time = 0
        self.ik_cooldown = 0.01  # Fast: 30ms
        self.last_cartesian_input_time = 0
        self.cartesian_control_timeout = 0.5
        self.fk_calculated = False
        
        # Anti-jitter
        self.locked_joint_positions = None
        self.position_deadband = 0.0003  # radians - INCREASED (was 0.0001)
        self.last_sent_positions = None  # Track last sent command to prevent jitter
        
        # ROS Publishers & Subscribers
        self.command_pub = rospy.Publisher(
            "/joint_trajectory_controller/command",
            JointTrajectory, 
            queue_size=10
        )
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.setup_gui()
        
        rospy.loginfo("Robot Controller started (Monotonic Movement)")

    def setup_gui(self):
        """Build GUI"""
        mode_frame = tk.Frame(self.root, padx=10, pady=10)
        mode_frame.pack()
        
        tk.Label(mode_frame, text="Control Mode:", font=("Arial", 14, "bold")).pack(side=tk.LEFT, padx=5)
        self.mode_label = tk.Label(mode_frame, text="JOINT", font=("Arial", 14, "bold"), fg="green")
        self.mode_label.pack(side=tk.LEFT, padx=5)
        
        ttk.Separator(self.root, orient='horizontal').pack(fill='x', pady=10)
        
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
        
        self.selected_label = tk.Label(self.root, text="", font=("Arial", 13, "bold"), fg="blue")
        self.selected_label.pack(pady=10)
        
        self.status_label = tk.Label(self.root, text="Ready", font=("Arial", 10), fg="gray", anchor='w')
        self.status_label.pack(fill='x', padx=10, pady=5)

    def joint_states_callback(self, msg):
        """Updates joint positions and calculates FK"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
        
        current_time = time.time()
        
        should_update_fk = (
            not self.cartesian_update_pending and 
            (self.control_mode == "JOINT" or 
             current_time - self.last_cartesian_input_time > self.cartesian_control_timeout)
        )
        
        if should_update_fk:
            try:
                joint_angles_rad = self.joint_positions[:5]
                self.current_cartesian = self.ik_solver.forward_kinematics(joint_angles_rad)
                if not self.fk_calculated:
                    self.fk_calculated = True
            except Exception as e:
                rospy.logwarn(f"FK calculation failed: {e}")

    def joy_callback(self, msg):
        """Handles PS4 controller"""
        button_cross = msg.buttons[0]
        button_circle = msg.buttons[1]
        button_triangle = msg.buttons[2]
        button_square = msg.buttons[3]
        button_l1 = msg.buttons[4]
        button_r1 = msg.buttons[5]
        
        current_time = time.time()
        
        # Switch mode
        if current_time - self.last_button_press_time > self.debounce_duration:
            if button_l1 == 1 or button_r1 == 1:
                self.control_mode = "CARTESIAN" if self.control_mode == "JOINT" else "JOINT"
                self.mode_label.config(text=self.control_mode)
                self.last_button_press_time = current_time
                
                if self.control_mode == "CARTESIAN":
                    try:
                        joint_angles_rad = self.joint_positions[:5]
                        self.current_cartesian = self.ik_solver.forward_kinematics(joint_angles_rad)
                        self.status_label.config(text="Switched to Cartesian mode", fg="green")
                    except Exception as e:
                        rospy.logwarn(f"FK calculation failed on mode switch: {e}")
            
            if button_triangle == 1:
                self.current_joint_index = (self.current_joint_index - 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time
            elif button_cross == 1:
                self.current_joint_index = (self.current_joint_index + 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time

        moved = False
        
        if self.control_mode == "JOINT":
            if current_time - self.last_movement_time < self.movement_delay:
                return
                
            step = 0.02
            if button_circle == 1:
                new_position = self.joint_positions[self.current_joint_index] + step
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
                
        else:  # CARTESIAN - MONOTONIC MOVEMENT
            if current_time - self.last_ik_time < self.ik_cooldown:
                return
            
            step = 0.5  # Very small step (0.2mm)
            if button_circle == 1:
                self.current_cartesian[self.current_joint_index] += step
                moved = True
                self.last_cartesian_input_time = current_time
                self.locked_joint_positions = None  # Unlock
            elif button_square == 1:
                self.current_cartesian[self.current_joint_index] -= step
                moved = True
                self.last_cartesian_input_time = current_time
                self.locked_joint_positions = None  # Unlock
            
            if moved:
                self.last_ik_time = current_time
                self.cartesian_update_pending = True
                
                current_angles_rad = self.joint_positions[:5]
                new_angles_rad, success = self.ik_solver.inverse_kinematics(
                    self.current_cartesian,
                    current_angles_rad,
                    control_axis=self.current_joint_index
                )
                
                if success and self.ik_solver.check_joint_limits(new_angles_rad):
                    # Check if change is significant compared to LAST SENT command (anti-jitter)
                    significant_change = False
                    
                    if self.last_sent_positions is None:
                        significant_change = True
                    else:
                        for i in range(5):
                            # Compare to last SENT position, not current position
                            if abs(new_angles_rad[i] - self.last_sent_positions[i]) > self.position_deadband:
                                significant_change = True
                                break
                    
                    if significant_change:
                        # Update positions
                        for i in range(5):
                            self.joint_positions[i] = new_angles_rad[i]
                        
                        # Remember what we sent
                        self.last_sent_positions = [self.joint_positions[i] for i in range(5)]
                        
                        reached_pos = self.ik_solver.forward_kinematics(new_angles_rad)
                        error = np.linalg.norm(np.array(self.current_cartesian) - np.array(reached_pos))
                        
                        self.status_label.config(text=f"IK OK ({error:.1f}mm)", fg="green")
                        self.send_command()
                    # else: skip this update - change too small, would cause jitter
                else:
                    if button_circle == 1:
                        self.current_cartesian[self.current_joint_index] -= step
                    elif button_square == 1:
                        self.current_cartesian[self.current_joint_index] += step
                    
                    if not success:
                        self.status_label.config(text=f"IK Failed", fg="red")
                    else:
                        self.status_label.config(text=f"Joint limit reached!", fg="orange")
                
                self.cartesian_update_pending = False
            else:
                # No button pressed - lock position to prevent drift
                if self.locked_joint_positions is None:
                    self.locked_joint_positions = self.joint_positions[:5].copy()
                else:
                    for i in range(5):
                        self.joint_positions[i] = self.locked_joint_positions[i]

    def send_command(self):
        """Publish command"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(0.05)  # Fast execution
        
        msg.points.append(point)
        self.command_pub.publish(msg)

    def update_gui(self):
        """Update GUI"""
        for i, name in enumerate(self.joint_names):
            self.joint_labels[name].config(text=f"{self.joint_positions[i]:.3f}")
        
        try:
            for i, axis in enumerate(['X', 'Y', 'Z']):
                value = self.current_cartesian[i]
                self.cartesian_labels[axis].config(text=f"{value:.1f}")
        except (IndexError, TypeError):
            pass
        
        if self.control_mode == "JOINT":
            selected = self.joint_names[self.current_joint_index]
            self.selected_label.config(text=f"Selected: {selected}")
            
            for name, label in self.joint_labels.items():
                label.config(fg="blue" if name == selected else "black")
            for label in self.cartesian_labels.values():
                label.config(fg="black")
        else:
            axes = ['X', 'Y', 'Z']
            selected = axes[self.current_joint_index]
            self.selected_label.config(text=f"Selected: {selected} axis")
            
            for axis, label in self.cartesian_labels.items():
                label.config(fg="blue" if axis == selected else "black")
            for label in self.joint_labels.values():
                label.config(fg="black")
        
        self.root.after(150, self.update_gui)


def main():
    rospy.init_node("robot_controller_monotonic")
    root = tk.Tk()
    gui = RobotControllerGUI(root)
    gui.update_gui()
    root.mainloop()

if __name__ == "__main__":
    main()
