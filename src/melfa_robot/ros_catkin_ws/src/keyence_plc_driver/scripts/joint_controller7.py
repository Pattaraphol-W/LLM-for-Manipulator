#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk
import time
import numpy as np
import math
import threading

class RobotArmIK:
    def __init__(self):
        # Joint limits in radians
        self.joint_limits = np.array([
            [math.radians(-240), math.radians(240)],
            [math.radians(-120), math.radians(120)],
            [math.radians(0),    math.radians(161)],
            [math.radians(-200), math.radians(200)],
            [math.radians(-120), math.radians(120)]
        ])
        
        # Velocity smoothing - lighter filter for faster response
        self.last_delta_q = np.zeros(5)
        self.alpha = 0.85  # Increased from 0.6 - less smoothing, faster response
    
    def reset_velocity_filter(self):
        """Reset the velocity filter to stop momentum"""
        self.last_delta_q = np.zeros(5)
    
    def forward_kinematics(self, joint_angles_rad):
        """Calculate FK using A-matrices"""
        joint_angles_deg = [math.degrees(a) for a in joint_angles_rad]
        t1, t2, t3, t4, t5 = joint_angles_deg
        
        t1_rad = math.radians(t1)
        A0 = np.array([
            [math.cos(t1_rad), -math.sin(t1_rad), 0.0, 0.0],
            [math.sin(t1_rad),  math.cos(t1_rad), 0.0, 0.0],
            [0.0,               0.0,               1.0, 0.0],
            [0.0,               0.0,               0.0, 1.0]
        ])
        
        t2_rad = math.radians(t2)
        A1 = np.array([
            [math.cos(t2_rad),  0.0, math.sin(t2_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t2_rad), 0.0, math.cos(t2_rad), 350.0],
            [0.0,               0.0, 0.0,              1.0]
        ])
        
        t3_rad = math.radians(t3)
        A2 = np.array([
            [math.cos(t3_rad),  0.0, math.sin(t3_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t3_rad), 0.0, math.cos(t3_rad), 310.0],
            [0.0,               0.0, 0.0,              1.0]
        ])
        
        t4_rad = math.radians(t4)
        A3 = np.array([
            [math.cos(t4_rad), -math.sin(t4_rad), 0.0, -50.0],
            [math.sin(t4_rad),  math.cos(t4_rad), 0.0, 0.0],
            [0.0,               0.0,              1.0, 50.0],
            [0.0,               0.0,              0.0, 1.0]
        ])
        
        t5_rad = math.radians(t5)
        A4 = np.array([
            [math.cos(t5_rad),  0.0, math.sin(t5_rad), 0.0],
            [0.0,               1.0, 0.0,              0.0],
            [-math.sin(t5_rad), 0.0, math.cos(t5_rad), 285.15],
            [0.0,               0.0, 0.0,              1.0]
        ])
        
        T = A0 @ A1 @ A2 @ A3 @ A4
        pos_homogeneous = T @ np.array([0.0, 0.0, 124.85, 1.0])
        return pos_homogeneous[:3].tolist()
    
    def differential_ik_step(self, current_joints_rad, target_delta_mm):
        """Differential IK with velocity smoothing"""
        q = np.array(current_joints_rad)
        delta_x = np.array(target_delta_mm)
        
        J = self.compute_jacobian_numerical(q)
        
        damping = 0.12  # Reduced for faster response (was 0.15)
        JTJ = J.T @ J + damping * np.eye(5)
        
        try:
            delta_q_raw = np.linalg.solve(JTJ, J.T @ delta_x)
        except:
            return np.zeros(5)
        
        # Smooth velocity
        delta_q = self.alpha * delta_q_raw + (1 - self.alpha) * self.last_delta_q
        self.last_delta_q = delta_q.copy()
        
        max_step = 0.08  # Increased for faster motion (was 0.03)
        step_norm = np.linalg.norm(delta_q)
        if step_norm > max_step:
            delta_q = delta_q * (max_step / step_norm)
        
        # Check limits
        q_new = q + delta_q
        for i in range(5):
            if q_new[i] < self.joint_limits[i][0]:
                delta_q[i] = self.joint_limits[i][0] - q[i]
            elif q_new[i] > self.joint_limits[i][1]:
                delta_q[i] = self.joint_limits[i][1] - q[i]
        
        return delta_q
    
    def compute_jacobian_numerical(self, joint_angles_rad):
        """Compute Jacobian numerically"""
        epsilon = 5e-5
        J = np.zeros((3, 5))
        
        pos_0 = np.array(self.forward_kinematics(joint_angles_rad))
        
        for i in range(5):
            q_perturbed = joint_angles_rad.copy()
            q_perturbed[i] += epsilon
            pos_perturbed = np.array(self.forward_kinematics(q_perturbed))
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
        self.root.title("Robot Arm Controller (No Swing)")
        self.root.geometry("700x650")

        self.ik_solver = RobotArmIK()
        
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
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
        self.target_joint_positions = [0.0] * len(self.joint_names)
        self.current_joint_index = 0
        self.last_button_press_time = 0
        self.debounce_duration = 0.2
        self.control_mode = "JOINT"
        
        # CONTINUOUS MOTION - velocity-based control
        self.joint_velocity = [0.0] * len(self.joint_names)
        self.cartesian_velocity = [0.0, 0.0, 0.0]
        self.is_moving = False
        self.was_moving = False  # Track previous state
        
        # Cartesian control
        self.current_cartesian = [0.0, 0.0, 0.0]
        
        # ROS Publishers & Subscribers
        self.command_pub = rospy.Publisher(
            "/joint_trajectory_controller/command",
            JointTrajectory, 
            queue_size=10
        )
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.setup_gui()
        
        # Start continuous control loop in separate thread
        self.control_thread = threading.Thread(target=self.continuous_control_loop, daemon=True)
        self.control_thread.start()
        
        rospy.loginfo("Robot Controller started (No Swing Version)")

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
        
        # Velocity indicator
        self.velocity_label = tk.Label(self.root, text="Velocity: 0.000 rad/s", 
                                       font=("Arial", 9), fg="blue", anchor='w')
        self.velocity_label.pack(fill='x', padx=10, pady=2)

    def joint_states_callback(self, msg):
        """Updates joint positions"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
        
        # Update cartesian from actual positions
        try:
            joint_angles_rad = self.joint_positions[:5]
            self.current_cartesian = self.ik_solver.forward_kinematics(joint_angles_rad)
        except Exception as e:
            rospy.logwarn(f"FK calculation failed: {e}")

    def joy_callback(self, msg):
        """Handles PS4 controller - sets velocities"""
        button_l2 = msg.buttons[6] #l2
        button_r1 = msg.buttons[5] #r1
        button_l1 = msg.buttons[4] #l1 
        button_r2 = msg.buttons[7] #r2
        button_logo = msg.buttons[0]
        
        current_time = time.time()
        
        # Switch mode
        if current_time - self.last_button_press_time > self.debounce_duration:
            if button_logo == 1: 
                self.control_mode = "CARTESIAN" if self.control_mode == "JOINT" else "JOINT"
                self.mode_label.config(text=self.control_mode)
                self.last_button_press_time = current_time
                self.status_label.config(text=f"Switched to {self.control_mode} mode", fg="green")
                # Reset velocities on mode switch
                self.joint_velocity = [0.0] * len(self.joint_names)
                self.cartesian_velocity = [0.0, 0.0, 0.0]
                # Reset IK velocity filter
                self.ik_solver.reset_velocity_filter()
            
            if button_l1 == 1:
                self.current_joint_index = (self.current_joint_index - 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time
            elif button_l2 == 1:
                self.current_joint_index = (self.current_joint_index + 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time
        
        # Set velocities based on button state
        if self.control_mode == "JOINT":
            velocity = 0.3  # rad/s 
            
            # Reset all velocities first
            self.joint_velocity = [0.0] * len(self.joint_names)
            
            if button_r1 == 1:
                self.joint_velocity[self.current_joint_index] = velocity
                self.is_moving = True
            elif button_r2 == 1:
                self.joint_velocity[self.current_joint_index] = -velocity
                self.is_moving = True
            else:
                self.is_moving = False
                
        else:  # CARTESIAN
            velocity_mm_s = 90.0  # mm/s 
            
            # Reset velocities
            self.cartesian_velocity = [0.0, 0.0, 0.0]
            
            if button_r1 == 1:
                self.cartesian_velocity[self.current_joint_index] = velocity_mm_s
                self.is_moving = True
            elif button_r2 == 1:
                self.cartesian_velocity[self.current_joint_index] = -velocity_mm_s
                self.is_moving = True
            else:
                self.is_moving = False
        
        # SIMPLE FIX: Reset IK filter whenever stopping (both modes)
        if self.was_moving and not self.is_moving:
            self.ik_solver.reset_velocity_filter()
        
        # Track previous state
        self.was_moving = self.is_moving

    def continuous_control_loop(self):
        """Runs continuously at 50Hz to generate smooth motion"""
        rate = rospy.Rate(50)  # 50Hz for smooth motion
        
        while not rospy.is_shutdown():
            try:
                if self.control_mode == "JOINT":
                    self.update_joint_control()
                else:
                    self.update_cartesian_control()
                
                rate.sleep()
            except Exception as e:
                rospy.logerr(f"Control loop error: {e}")

    def update_joint_control(self):
        """Update joint positions based on velocity"""
        dt = 0.02  # 50Hz = 20ms
        
        has_velocity = False
        
        for i in range(len(self.joint_names)):
            if abs(self.joint_velocity[i]) > 0.001:
                has_velocity = True
                # Calculate new position
                new_position = self.joint_positions[i] + self.joint_velocity[i] * dt
                
                # Check limits
                limits = self.joint_limits_rad[i]
                if limits[0] is not None and limits[1] is not None:
                    if limits[0] <= new_position <= limits[1]:
                        self.target_joint_positions[i] = new_position
                    else:
                        # Hit limit, stop
                        self.joint_velocity[i] = 0.0
                        self.target_joint_positions[i] = self.joint_positions[i]
                else:
                    self.target_joint_positions[i] = new_position
            else:
                # No velocity, hold current position
                self.target_joint_positions[i] = self.joint_positions[i]
        
        # Only send command if there's actual movement
        if has_velocity:
            self.send_command()

    def update_cartesian_control(self):
        """Update joint positions for cartesian velocity"""
        dt = 0.02  # 50Hz
        
        # Only run IK if actually moving
        if self.is_moving and np.linalg.norm(self.cartesian_velocity) > 0.1:
            # Calculate desired cartesian displacement
            delta_cartesian_mm = np.array(self.cartesian_velocity) * dt
            
            # Use differential IK to get joint changes
            current_angles_rad = self.joint_positions[:5]
            delta_q = self.ik_solver.differential_ik_step(current_angles_rad, delta_cartesian_mm)
            
            # Apply changes
            new_angles_rad = current_angles_rad + delta_q
            
            if self.ik_solver.check_joint_limits(new_angles_rad):
                for i in range(5):
                    self.target_joint_positions[i] = new_angles_rad[i]
                
                # Show velocity
                vel_mag = np.linalg.norm(delta_q) / dt
                self.velocity_label.config(text=f"Velocity: {vel_mag:.3f} rad/s")
                
                # Send command while moving
                self.send_command()
            else:
                # Hit limit - stop motion
                self.cartesian_velocity = [0.0, 0.0, 0.0]
                self.is_moving = False
                self.ik_solver.reset_velocity_filter()
        else:
            # NOT MOVING: Don't send any commands
            # Robot will hold last position naturally
            self.velocity_label.config(text=f"Velocity: 0.000 rad/s")

    def send_command(self):
        """Publish trajectory command"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.target_joint_positions
        point.time_from_start = rospy.Duration(0.02)  # 20ms lookahead
        
        msg.points.append(point)
        self.command_pub.publish(msg)

    def update_gui(self):
        """Update GUI display"""
        for i, name in enumerate(self.joint_names):
            self.joint_labels[name].config(text=f"{self.joint_positions[i]:.3f}")
        
        # Show actual cartesian values
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
        
        # Update status based on motion
        if self.is_moving:
            self.status_label.config(text="Moving smoothly...", fg="green")
        else:
            self.status_label.config(text="Stopped", fg="gray")
        
        self.root.after(50, self.update_gui)  # 20Hz GUI update


def main():
    rospy.init_node("robot_controller_no_swing")
    root = tk.Tk()
    gui = RobotControllerGUI(root)
    gui.update_gui()
    root.mainloop()

if __name__ == "__main__":
    main()
