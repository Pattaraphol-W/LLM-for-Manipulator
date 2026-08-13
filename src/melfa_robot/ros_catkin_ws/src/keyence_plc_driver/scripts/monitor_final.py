#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, String
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


class CombinedGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm & PLC Control System")
        self.root.geometry("1400x900")
        self.root.configure(bg='#2c3e50')
        
        # ===== ROBOT ARM STATE =====
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
        
        self.joint_positions = [0.0] * len(self.joint_names)
        self.target_joint_positions = [0.0] * len(self.joint_names)
        self.current_joint_index = 0
        self.control_mode = "JOINT"
        
        # CONTINUOUS MOTION - velocity-based control
        self.joint_velocity = [0.0] * len(self.joint_names)
        self.cartesian_velocity = [0.0, 0.0, 0.0]
        self.is_moving = False
        self.was_moving = False
        
        self.current_cartesian = [0.0, 0.0, 0.0]
        
        # ===== PLC STATE =====
        self.plc_states = {}
        self.last_active_mr = None
        self.last_stick_update_time = 0
        self.stick_update_interval = 0.1
        
        # ===== CONTROLLER STATE =====
        self.last_buttons = [0] * 13
        self.last_axes = [0.0] * 10
        self.last_button_press_time = 0
        self.debounce_duration = 0.2
        
        # Setup GUI
        self.setup_gui()
        
        # ===== ROS SETUP =====
        rospy.init_node('combined_controller_gui', anonymous=True)
        self._rate = rospy.Rate(10)
        
        # Robot Publishers & Subscribers
        self.command_pub = rospy.Publisher(
            "/joint_trajectory_controller/command",
            JointTrajectory, 
            queue_size=10
        )
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        
        # PLC Publishers & Subscribers
        self._write_pub = rospy.Publisher('/write_command', String, queue_size=10)
        
        # Initialize all PLC states
        all_mr_addresses = ['R603', 'R604', 'R605', 'MR206', 'MR207', 'MR208', 'MR209', 
                           'MR400', 'MR401', 'MR402', 'MR403', 'MR404', 'MR405', 'MR406', 
                           'MR602', 'MR603', 'MR604', 'MR605',
                           'MR012', 'MR013', 'MR100', 'MR101', 'MR102', 'MR103', 
                           'MR104', 'MR105', 'MR106', 'MR107']
        
        for addr in all_mr_addresses:
            self.plc_states[addr] = 0
            rospy.Subscriber(f"/plc_protocol_driver/read/{addr}", Float32, 
                           self._create_plc_callback(addr))
        
        # Joy subscriber (handles both robot and PLC)
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        
        # Start continuous control loop
        self.control_thread = threading.Thread(target=self.continuous_control_loop, daemon=True)
        self.control_thread.start()
        
        rospy.loginfo("Combined Robot & PLC Controller started")

    def setup_gui(self):
        """Build combined GUI"""
        # Main container
        main_container = tk.Frame(self.root, bg='#2c3e50')
        main_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        # ========== LEFT SIDE: ROBOT ARM CONTROL ==========
        left_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        tk.Label(left_frame, text="ROBOT ARM CONTROL", font=("Arial", 16, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=10)
        
        # Mode display
        mode_frame = tk.Frame(left_frame, bg='#2c3e50', relief="sunken", bd=2)
        mode_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(mode_frame, text="Control Mode:", font=("Arial", 12, "bold"), 
                bg='#2c3e50', fg='#3498db').pack(side=tk.LEFT, padx=10, pady=5)
        self.mode_label = tk.Label(mode_frame, text="JOINT", font=("Arial", 12, "bold"), 
                                   bg='#2c3e50', fg='#2ecc71')
        self.mode_label.pack(side=tk.LEFT, padx=10, pady=5)
        
        # Joint angles
        joint_frame = tk.LabelFrame(left_frame, text="Joint Angles (rad)", 
                                    bg='#34495e', fg='#ecf0f1', font=("Arial", 11, "bold"))
        joint_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.joint_labels = {}
        for i, name in enumerate(self.joint_names):
            row_frame = tk.Frame(joint_frame, bg='#2c3e50')
            row_frame.pack(fill='x', pady=2, padx=5)
            
            tk.Label(row_frame, text=name, font=("Arial", 10, "bold"), 
                    bg='#2c3e50', fg='#ecf0f1', width=10, anchor='w').pack(side=tk.LEFT)
            self.joint_labels[name] = tk.Label(row_frame, text=f"{self.joint_positions[i]:.4f}", 
                                               font=("Arial", 10), bg='#2c3e50', fg='#f39c12',
                                               width=12, anchor='e')
            self.joint_labels[name].pack(side=tk.LEFT)
        
        # Cartesian position
        cartesian_frame = tk.LabelFrame(left_frame, text="Cartesian Position (mm)", 
                                       bg='#34495e', fg='#ecf0f1', font=("Arial", 11, "bold"))
        cartesian_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.cartesian_labels = {}
        for axis in ['X', 'Y', 'Z']:
            row_frame = tk.Frame(cartesian_frame, bg='#2c3e50')
            row_frame.pack(fill='x', pady=2, padx=5)
            
            tk.Label(row_frame, text=axis, font=("Arial", 10, "bold"), 
                    bg='#2c3e50', fg='#ecf0f1', width=10, anchor='w').pack(side=tk.LEFT)
            self.cartesian_labels[axis] = tk.Label(row_frame, text="0.00", 
                                                   font=("Arial", 10), bg='#2c3e50', fg='#f39c12',
                                                   width=12, anchor='e')
            self.cartesian_labels[axis].pack(side=tk.LEFT)
        
        # Selected joint/axis
        self.selected_label = tk.Label(left_frame, text="", font=("Arial", 11, "bold"), 
                                       bg='#34495e', fg='#3498db')
        self.selected_label.pack(pady=5)
        
        # Status
        self.status_label = tk.Label(left_frame, text="Ready", font=("Arial", 9), 
                                     bg='#34495e', fg='#95a5a6', anchor='w')
        self.status_label.pack(fill='x', padx=10, pady=5)
        
        self.velocity_label = tk.Label(left_frame, text="Velocity: 0.000 rad/s", 
                                      font=("Arial", 9), bg='#34495e', fg='#3498db', anchor='w')
        self.velocity_label.pack(fill='x', padx=10, pady=2)
        
        # ========== RIGHT SIDE: PLC CONTROL ==========
        right_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        right_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        
        tk.Label(right_frame, text="PLC CONTROL", font=("Arial", 16, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=10)
        
        # Stick values
        sticks_frame = tk.Frame(right_frame, bg='#2c3e50', relief="sunken", bd=2)
        sticks_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(sticks_frame, text="Left Stick (Y):", font=("Arial", 11, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=0, column=0, padx=10, pady=5, sticky="w")
        self.left_stick_var = tk.StringVar(value="0.00")
        tk.Label(sticks_frame, textvariable=self.left_stick_var, font=("Arial", 14, "bold"), 
                bg='#2c3e50', fg='#f39c12', width=8).grid(row=0, column=1, padx=10, pady=5)
        
        tk.Label(sticks_frame, text="Right Stick (X):", font=("Arial", 11, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=1, column=0, padx=10, pady=5, sticky="w")
        self.right_stick_var = tk.StringVar(value="0.00")
        tk.Label(sticks_frame, textvariable=self.right_stick_var, font=("Arial", 14, "bold"), 
                bg='#2c3e50', fg='#f39c12', width=8).grid(row=1, column=1, padx=10, pady=5)
        
        tk.Label(sticks_frame, text="Direction:", font=("Arial", 11, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=2, column=0, padx=10, pady=5, sticky="w")
        self.active_mr_var = tk.StringVar(value="STOP")
        tk.Label(sticks_frame, textvariable=self.active_mr_var, font=("Arial", 12, "bold"), 
                bg='#2c3e50', fg='#2ecc71', width=15).grid(row=2, column=1, padx=10, pady=5)
        
        # Button mappings
        mappings_frame = tk.Frame(right_frame, bg='#34495e')
        mappings_frame.pack(padx=10, pady=5, fill="both", expand=True)
        
        tk.Label(mappings_frame, text="BUTTON MAPPINGS", font=("Arial", 12, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=5)
        
        self.button_frames = {}
        mappings = [
            ("^ Triangle", "MR207 Motor ON", "btn^", "MR207"),
            ("[] Square", "MR208 Start", "btn[]", "MR208"),
            ("o Circle", "MR209 Break", "btnO", "MR209"),
            ("⬆️ D-Pad UP", "MR603/604 Switch", "btn4", "MR603"),
            ("➡️ D-Pad RIGHT", "MR605 Motor DC", "btn5", "MR605"),
            ("⬇️ D-Pad DOWN", "MR206 Error Reset", "btn6", "MR206"),
            ("⬅️ D-Pad LEFT", "MR602 Power Hold", "btn7", "MR602"),
        ]
        
        for btn_name, function, key, mr_address in mappings:
            frame = tk.Frame(mappings_frame, bg='#2c3e50', relief="raised", bd=2)
            frame.pack(padx=5, pady=3, fill="x")
            
            tk.Label(frame, text=btn_name, font=("Arial", 9, "bold"), 
                    bg='#2c3e50', fg='#ecf0f1', width=15, anchor="w").pack(side="left", padx=5, pady=5)
            
            tk.Label(frame, text=function, font=("Arial", 8), 
                    bg='#2c3e50', fg='#f39c12', anchor="w").pack(side="left", padx=5, pady=5, fill="x", expand=True)
            
            indicator = tk.Canvas(frame, width=20, height=20, bg='#2c3e50', highlightthickness=0)
            indicator.pack(side="right", padx=5, pady=5)
            circle = indicator.create_oval(2, 2, 18, 18, fill='#7f8c8d', outline='')
            
            self.button_frames[key] = {
                'frame': frame, 
                'indicator': indicator, 
                'circle': circle,
                'mr_address': mr_address
            }
        
        # PLC values
        plc_frame = tk.LabelFrame(right_frame, text="PLC Values", 
                                 bg='#34495e', fg='#ecf0f1', font=("Arial", 11, "bold"))
        plc_frame.pack(padx=10, pady=5, fill="both", expand=True)
        
        self.plc_vars = {}
        self.plc_indicators = {}
        
        display_config = [
            ('MR012', 'Ready'),
            ('MR013', 'Running'),
            ('MR100', 'Back FAR'),
            ('MR101', 'Back Mid'),
            ('MR102', 'Back Close'),
            ('MR104', 'Front FAR'),
            ('MR105', 'Front Mid'),
            ('MR106', 'Front Close'),
            ('R603', '24V ON'),
            ('R604', '12V ON'),
            ('R605', '5V ON'),
        ]
        
        for addr, label in display_config:
            frame = tk.Frame(plc_frame, bg='#2c3e50', relief="raised", bd=1)
            frame.pack(padx=5, pady=2, fill="x")
            
            tk.Label(frame, text=f"{label}:", font=("Arial", 9, "bold"), 
                    bg='#2c3e50', fg='#9b59b6', width=18, anchor="w").pack(side="left", padx=8, pady=5)
            
            indicator = tk.Canvas(frame, width=20, height=20, bg='#2c3e50', highlightthickness=0)
            indicator.pack(side="right", padx=8, pady=5)
            circle = indicator.create_oval(2, 2, 18, 18, fill='#7f8c8d', outline='#34495e', width=1)
            
            var = tk.StringVar(value="OFF")
            self.plc_vars[addr] = var
            self.plc_indicators[addr] = {'indicator': indicator, 'circle': circle}

    # ========== ROBOT ARM CALLBACKS ==========
    def joint_states_callback(self, msg):
        """Updates joint positions"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
        
        try:
            joint_angles_rad = self.joint_positions[:5]
            self.current_cartesian = self.ik_solver.forward_kinematics(joint_angles_rad)
        except Exception as e:
            rospy.logwarn(f"FK calculation failed: {e}")

    def joy_callback(self, msg):
        """Handles PS4 controller for both robot and PLC"""
        current_time = time.time()
        
        # Robot control buttons
        button_l2 = msg.buttons[6]
        button_r1 = msg.buttons[5]
        button_l1 = msg.buttons[4]
        button_r2 = msg.buttons[7]
        button_logo = msg.buttons[12]
        
        # ===== ROBOT CONTROL =====
        # Mode switch
        if current_time - self.last_button_press_time > self.debounce_duration:
            if button_logo == 1:
                self.control_mode = "CARTESIAN" if self.control_mode == "JOINT" else "JOINT"
                self.mode_label.config(text=self.control_mode)
                self.last_button_press_time = current_time
                self.status_label.config(text=f"Switched to {self.control_mode} mode", fg="green")
                self.joint_velocity = [0.0] * len(self.joint_names)
                self.cartesian_velocity = [0.0, 0.0, 0.0]
                self.ik_solver.reset_velocity_filter()
            
            if button_l1 == 1:
                self.current_joint_index = (self.current_joint_index - 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time
            elif button_l2 == 1:
                self.current_joint_index = (self.current_joint_index + 1) % (6 if self.control_mode == "JOINT" else 3)
                self.last_button_press_time = current_time
        
        # Robot velocity control
        if self.control_mode == "JOINT":
            velocity = 0.3
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
            velocity_mm_s = 90.0
            self.cartesian_velocity = [0.0, 0.0, 0.0]
            
            if button_r1 == 1:
                self.cartesian_velocity[self.current_joint_index] = velocity_mm_s
                self.is_moving = True
            elif button_r2 == 1:
                self.cartesian_velocity[self.current_joint_index] = -velocity_mm_s
                self.is_moving = True
            else:
                self.is_moving = False
        
        if self.was_moving and not self.is_moving:
            self.ik_solver.reset_velocity_filter()
        
        self.was_moving = self.is_moving
        
        # ===== PLC CONTROL =====
        # Analog sticks
        left_y = msg.axes[1]
        right_x = msg.axes[3]
        
        self.left_stick_var.set(f"{left_y:.2f}")
        self.right_stick_var.set(f"{right_x:.2f}")
        
        if current_time - self.last_stick_update_time >= self.stick_update_interval:
            self._handle_stick_control(left_y, right_x)
            self.last_stick_update_time = current_time
        
        # PLC Buttons
        if msg.buttons[2] == 1 and self.last_buttons[2] == 0:
            current_state = self.plc_states.get('MR207', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR207', new_state)
            rospy.loginfo(f"MR207 toggled: {current_state} -> {new_state}")
        
        if msg.buttons[3] == 1 and self.last_buttons[3] == 0:
            current_state = self.plc_states.get('MR208', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR208', new_state)
            rospy.loginfo(f"MR208 toggled: {current_state} -> {new_state}")
        
        if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
            current_state = self.plc_states.get('MR209', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR209', new_state)
            rospy.loginfo(f"MR209 toggled: {current_state} -> {new_state}")
        
        if msg.axes[7] > 0.5 and self.last_axes[7] <= 0.5:
            rospy.loginfo(f"D-Pad UP pressed! axes[7]={msg.axes[7]}")
            mr603_state = self.plc_states.get('MR603', 0)
            if mr603_state == 0:
                self._publish_write_command('MR603', 1)
                self._publish_write_command('MR604', 0)
                rospy.loginfo("D-Pad UP: MR603/604 Switch: MR603=1, MR604=0")
            else:
                self._publish_write_command('MR603', 0)
                self._publish_write_command('MR604', 1)
                rospy.loginfo("D-Pad UP: MR603/604 Switch: MR603=0, MR604=1")
        
        if msg.axes[6] < -0.5 and self.last_axes[6] >= -0.5:
            rospy.loginfo(f"D-Pad RIGHT pressed! axes[6]={msg.axes[6]}")
            current_state = self.plc_states.get('MR605', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR605', new_state)
            rospy.loginfo(f"D-Pad RIGHT: MR605 toggled: {current_state} -> {new_state}")
        
        if msg.axes[7] < -0.5 and self.last_axes[7] >= -0.5:
            rospy.loginfo(f"D-Pad DOWN pressed! axes[7]={msg.axes[7]}")
            current_state = self.plc_states.get('MR206', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR206', new_state)
            rospy.loginfo(f"D-Pad DOWN: MR206 toggled: {current_state} -> {new_state}")
        
        if msg.axes[6] > 0.5 and self.last_axes[6] <= 0.5:
            rospy.loginfo(f"D-Pad LEFT pressed! axes[6]={msg.axes[6]}")
            current_state = self.plc_states.get('MR602', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR602', new_state)
            rospy.loginfo(f"D-Pad LEFT: MR602 toggled: {current_state} -> {new_state}")
        
        self.last_buttons = list(msg.buttons)
        self.last_axes = list(msg.axes)
    
    # ========== PLC CONTROL METHODS ==========
    def _handle_stick_control(self, left_y, right_x):
        """Handle stick-based PLC control"""
        active_mr = None
        direction_text = "STOP"
        
        if left_y > 0.05:
            if abs(right_x) < 0.2:
                active_mr = 'MR400'
                direction_text = "FORWARD"
            elif right_x > 0.2:
                active_mr = 'MR402'
                direction_text = "FORWARD LEFT"
            elif right_x < -0.2:
                active_mr = 'MR401'
                direction_text = "FORWARD RIGHT"
        elif left_y < -0.05:
            if abs(right_x) < 0.2:
                active_mr = 'MR403'
                direction_text = "BACKWARD"
            elif right_x > 0.2:
                active_mr = 'MR405'
                direction_text = "BACKWARD LEFT"
            elif right_x < -0.2:
                active_mr = 'MR404'
                direction_text = "BACKWARD RIGHT"
        elif abs(left_y) < 0.2 and abs(right_x) < 0.2:
            active_mr = 'MR406'
            direction_text = "STOP"
        
        if active_mr != self.last_active_mr:
            rospy.loginfo(f"Stick control: {self.last_active_mr} -> {active_mr} ({direction_text})")
            
            if self.last_active_mr is not None:
                self._publish_write_command(self.last_active_mr, 0)
                rospy.sleep(0.05)
            
            if active_mr is not None:
                self._publish_write_command(active_mr, 1)
            
            self.last_active_mr = active_mr
            self.active_mr_var.set(direction_text)

    def _create_plc_callback(self, address):
        """Creates callback for PLC values"""
        def callback(msg):
            self.plc_states[address] = msg.data
            
            if address in self.plc_indicators:
                self.root.after(0, lambda: self._update_plc_led(address, msg.data))
            
            self.root.after(0, lambda: self._update_button_indicators())
        return callback

    def _update_plc_led(self, address, value):
        """Update LED indicator"""
        if address not in self.plc_indicators:
            return
        
        indicator = self.plc_indicators[address]['indicator']
        circle = self.plc_indicators[address]['circle']
        
        if value == -1.0:
            indicator.itemconfig(circle, fill='#e74c3c', outline='#c0392b')
            self.plc_vars[address].set("ERROR")
        elif value == 1.0 or value == 1:
            indicator.itemconfig(circle, fill='#2ecc71', outline='#27ae60')
            self.plc_vars[address].set("ON")
        else:
            indicator.itemconfig(circle, fill='#7f8c8d', outline='#34495e')
            self.plc_vars[address].set("OFF")

    def _update_button_indicators(self):
        """Update button indicators"""
        for key, button_info in self.button_frames.items():
            mr_address = button_info['mr_address']
            indicator = button_info['indicator']
            circle = button_info['circle']
            
            state = self.plc_states.get(mr_address, 0)
            
            if state == 1:
                indicator.itemconfig(circle, fill='#2ecc71')
            else:
                indicator.itemconfig(circle, fill='#7f8c8d')

    def _publish_write_command(self, address, data):
        """Publish PLC write command"""
        msg = f"{address},{data}"
        rospy.loginfo(f"[GUI] Publishing write command: {msg}")
        self._write_pub.publish(msg)

    # ========== ROBOT CONTROL LOOP ==========
    def continuous_control_loop(self):
        """Robot control loop at 50Hz"""
        rate = rospy.Rate(50)
        
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
        """Update joint positions"""
        dt = 0.02
        has_velocity = False
        
        for i in range(len(self.joint_names)):
            if abs(self.joint_velocity[i]) > 0.001:
                has_velocity = True
                new_position = self.joint_positions[i] + self.joint_velocity[i] * dt
                
                limits = self.joint_limits_rad[i]
                if limits[0] is not None and limits[1] is not None:
                    if limits[0] <= new_position <= limits[1]:
                        self.target_joint_positions[i] = new_position
                    else:
                        self.joint_velocity[i] = 0.0
                        self.target_joint_positions[i] = self.joint_positions[i]
                else:
                    self.target_joint_positions[i] = new_position
            else:
                self.target_joint_positions[i] = self.joint_positions[i]
        
        if has_velocity:
            self.send_command()

    def update_cartesian_control(self):
        """Update cartesian control"""
        dt = 0.02
        
        if self.is_moving and np.linalg.norm(self.cartesian_velocity) > 0.1:
            delta_cartesian_mm = np.array(self.cartesian_velocity) * dt
            
            current_angles_rad = self.joint_positions[:5]
            delta_q = self.ik_solver.differential_ik_step(current_angles_rad, delta_cartesian_mm)
            
            new_angles_rad = current_angles_rad + delta_q
            
            if self.ik_solver.check_joint_limits(new_angles_rad):
                for i in range(5):
                    self.target_joint_positions[i] = new_angles_rad[i]
                
                vel_mag = np.linalg.norm(delta_q) / dt
                self.velocity_label.config(text=f"Velocity: {vel_mag:.3f} rad/s")
                
                self.send_command()
            else:
                self.cartesian_velocity = [0.0, 0.0, 0.0]
                self.is_moving = False
                self.ik_solver.reset_velocity_filter()
        else:
            self.velocity_label.config(text=f"Velocity: 0.000 rad/s")

    def send_command(self):
        """Publish trajectory command"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.target_joint_positions
        point.time_from_start = rospy.Duration(0.02)
        
        msg.points.append(point)
        self.command_pub.publish(msg)

    # ========== GUI UPDATE ==========
    def update_gui(self):
        """Update GUI display"""
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
                label.config(fg="#3498db" if name == selected else "#f39c12")
            for label in self.cartesian_labels.values():
                label.config(fg="#f39c12")
        else:
            axes = ['X', 'Y', 'Z']
            selected = axes[self.current_joint_index]
            self.selected_label.config(text=f"Selected: {selected} axis")
            
            for axis, label in self.cartesian_labels.items():
                label.config(fg="#3498db" if axis == selected else "#f39c12")
            for label in self.joint_labels.values():
                label.config(fg="#f39c12")
        
        if self.is_moving:
            self.status_label.config(text="Moving smoothly...", fg="#2ecc71")
        else:
            self.status_label.config(text="Stopped", fg="#95a5a6")
        
        self.root.after(50, self.update_gui)

    def spin(self):
        """Main loop"""
        def check_ros():
            if rospy.is_shutdown():
                self.root.destroy()
            else:
                self.root.after(100, check_ros)
        
        self.root.after(100, check_ros)
        self.root.mainloop()


def main():
    root = tk.Tk()
    app = CombinedGUI(root)
    app.update_gui()
    app.spin()

if __name__ == '__main__':
    main()
