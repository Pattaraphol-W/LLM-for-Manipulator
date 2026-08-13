#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
import os
import threading
import time
import numpy as np
import math


class RobotArmIK:
    def __init__(self):
        self.joint_limits = np.array([
            [math.radians(-240), math.radians(240)],
            [math.radians(-120), math.radians(120)],
            [math.radians(0),    math.radians(161)],
            [math.radians(-200), math.radians(200)],
            [math.radians(-120), math.radians(120)]
        ])
        self.last_delta_q = np.zeros(5)
        self.alpha = 0.85
    
    def reset_velocity_filter(self):
        self.last_delta_q = np.zeros(5)
    
    def forward_kinematics(self, joint_angles_rad):
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
        q = np.array(current_joints_rad)
        delta_x = np.array(target_delta_mm)
        
        J = self.compute_jacobian_numerical(q)
        
        damping = 0.12
        JTJ = J.T @ J + damping * np.eye(5)
        
        try:
            delta_q_raw = np.linalg.solve(JTJ, J.T @ delta_x)
        except:
            return np.zeros(5)
        
        delta_q = self.alpha * delta_q_raw + (1 - self.alpha) * self.last_delta_q
        self.last_delta_q = delta_q.copy()
        
        max_step = 0.08
        step_norm = np.linalg.norm(delta_q)
        if step_norm > max_step:
            delta_q = delta_q * (max_step / step_norm)
        
        q_new = q + delta_q
        for i in range(5):
            if q_new[i] < self.joint_limits[i][0]:
                delta_q[i] = self.joint_limits[i][0] - q[i]
            elif q_new[i] > self.joint_limits[i][1]:
                delta_q[i] = self.joint_limits[i][1] - q[i]
        
        return delta_q
    
    def compute_jacobian_numerical(self, joint_angles_rad):
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
        for i in range(5):
            if joint_angles_rad[i] < self.joint_limits[i][0] or \
               joint_angles_rad[i] > self.joint_limits[i][1]:
                return False
        return True


class EnhancedTeachingPendant(Node):
    def __init__(self, root):
        super().__init__('enhanced_teaching_pendant')
        
        self.root = root
        self.root.title("Enhanced Robot Teaching Pendant")
        self.root.geometry("1200x900")
        self.root.configure(bg='#2c3e50')
        
        # Robot arm state
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
        self.joint_velocity = [0.0] * len(self.joint_names)
        self.cartesian_velocity = [0.0, 0.0, 0.0]
        self.is_moving = False
        self.was_moving = False
        self.current_cartesian = [0.0, 0.0, 0.0]
        
        # Speed control
        self.speed_percentage = 100
        self.base_joint_velocity = 0.3
        self.base_cartesian_velocity = 90.0
        
        # Teaching state
        self.taught_positions = []
        self.is_playing = False
        self.is_recording = False
        self.current_filename = None
        self.playback_speed = 1.0
        self.loop_playback = False
        
        # Controller state
        self.last_buttons = [0] * 13
        self.last_axes = [0.0] * 10
        self.last_button_press_time = 0
        self.debounce_duration = 0.2
        
        self.setup_gui()
        self.setup_ros()
        
        # Start robot control thread
        self.control_thread = threading.Thread(target=self.continuous_control_loop, daemon=True)
        self.control_thread.start()
        
        self.get_logger().info("Enhanced Teaching Pendant started")
    
    def setup_gui(self):
        main_container = tk.Frame(self.root, bg='#2c3e50')
        main_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        # ========== LEFT: ROBOT STATUS ==========
        left_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        tk.Label(left_frame, text="ROBOT STATUS", font=("Arial", 14, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=10)
        
        # Control mode
        mode_frame = tk.Frame(left_frame, bg='#34495e')
        mode_frame.pack(pady=5)
        tk.Label(mode_frame, text="Control Mode:", font=("Arial", 11, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(side=tk.LEFT, padx=5)
        self.mode_label = tk.Label(mode_frame, text="JOINT", font=("Arial", 11, "bold"), 
                                   fg="#2ecc71", bg='#34495e')
        self.mode_label.pack(side=tk.LEFT, padx=5)
        
        # Joint angles
        joint_frame = tk.LabelFrame(left_frame, text="Joint Angles (rad)", 
                                    bg='#2c3e50', fg='#ecf0f1', padx=10, pady=5)
        joint_frame.pack(fill='x', padx=10, pady=5)
        
        self.joint_labels = {}
        for i, name in enumerate(self.joint_names):
            row = tk.Frame(joint_frame, bg='#2c3e50')
            row.pack(fill='x', pady=2)
            tk.Label(row, text=name, font=("Courier", 10, "bold"), width=8, 
                    anchor='w', bg='#2c3e50', fg='#3498db').pack(side=tk.LEFT)
            self.joint_labels[name] = tk.Label(row, text=f"{self.joint_positions[i]:.3f}", 
                                               font=("Courier", 10), width=10, 
                                               anchor='e', bg='#2c3e50', fg='#f39c12')
            self.joint_labels[name].pack(side=tk.LEFT)
        
        # Cartesian position
        cart_frame = tk.LabelFrame(left_frame, text="Cartesian Position (mm)", 
                                   bg='#2c3e50', fg='#ecf0f1', padx=10, pady=5)
        cart_frame.pack(fill='x', padx=10, pady=5)
        
        self.cartesian_labels = {}
        for axis in ['X', 'Y', 'Z']:
            row = tk.Frame(cart_frame, bg='#2c3e50')
            row.pack(fill='x', pady=2)
            tk.Label(row, text=axis, font=("Courier", 10, "bold"), width=8, 
                    anchor='w', bg='#2c3e50', fg='#3498db').pack(side=tk.LEFT)
            self.cartesian_labels[axis] = tk.Label(row, text="0.0", 
                                                    font=("Courier", 10), width=10, 
                                                    anchor='e', bg='#2c3e50', fg='#f39c12')
            self.cartesian_labels[axis].pack(side=tk.LEFT)
        
        self.selected_label = tk.Label(left_frame, text="", font=("Arial", 10, "bold"), 
                                       fg="#3498db", bg='#34495e')
        self.selected_label.pack(pady=5)
        
        self.velocity_label = tk.Label(left_frame, text="Velocity: 0.000 rad/s", 
                                       font=("Arial", 9), fg="#95a5a6", bg='#34495e')
        self.velocity_label.pack(pady=2)
        
        # Speed control
        speed_frame = tk.LabelFrame(left_frame, text="Speed Control", 
                                    bg='#2c3e50', fg='#ecf0f1', padx=10, pady=5)
        speed_frame.pack(fill='x', padx=10, pady=5)
        
        speed_control = tk.Frame(speed_frame, bg='#2c3e50')
        speed_control.pack(fill='x', pady=5)
        
        tk.Label(speed_control, text="0%", font=("Arial", 9), 
                bg='#2c3e50', fg='#95a5a6').pack(side=tk.LEFT, padx=5)
        
        self.speed_slider = tk.Scale(speed_control, from_=0, to=200, orient=tk.HORIZONTAL,
                                     command=self.on_speed_change, bg='#34495e', fg='#ecf0f1',
                                     troughcolor='#2c3e50', highlightthickness=0, 
                                     activebackground='#3498db', length=180)
        self.speed_slider.set(100)
        self.speed_slider.pack(side=tk.LEFT, fill='x', expand=True, padx=5)
        
        tk.Label(speed_control, text="200%", font=("Arial", 9), 
                bg='#2c3e50', fg='#95a5a6').pack(side=tk.LEFT, padx=5)
        
        self.speed_value_label = tk.Label(speed_frame, text="Speed: 100%", 
                                          font=("Arial", 10, "bold"),
                                          bg='#2c3e50', fg='#2ecc71')
        self.speed_value_label.pack(pady=2)
        
        # Controller help
        help_frame = tk.LabelFrame(left_frame, text="Controller Guide", 
                                   bg='#2c3e50', fg='#ecf0f1', padx=10, pady=5)
        help_frame.pack(fill='x', padx=10, pady=5)
        
        help_text = [
            "X: Switch JOINT/CARTESIAN",
            "L1/L2: Change joint/axis",
            "R1: Move positive",
            "R2: Move negative"
        ]
        
        for text in help_text:
            tk.Label(help_frame, text=text, font=("Arial", 8), 
                    bg='#2c3e50', fg='#95a5a6', anchor='w').pack(fill='x', pady=1)
        
        # ========== RIGHT: TEACHING PANEL ==========
        right_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        right_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        
        tk.Label(right_frame, text="TEACHING PANEL", font=("Arial", 14, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=10)
        
        # Teach buttons
        teach_btn_frame = tk.Frame(right_frame, bg='#34495e')
        teach_btn_frame.pack(pady=5)
        
        self.teach_button = tk.Button(teach_btn_frame, text="TEACH POSITION", 
                                      font=("Helvetica", 11), bg='#27ae60', 
                                      fg='white', width=18, height=2,
                                      command=self.teach_current_position)
        self.teach_button.pack(side=tk.LEFT, padx=5)
        
        self.record_button = tk.Button(teach_btn_frame, text="START RECORDING", 
                                       font=("Helvetica", 11), bg='#e74c3c', 
                                       fg='white', width=18, height=2,
                                       command=self.toggle_recording)
        self.record_button.pack(side=tk.LEFT, padx=5)
        
        # Position list
        list_frame = tk.LabelFrame(right_frame, text="Taught Positions", 
                                   bg='#2c3e50', fg='#ecf0f1', font=("Helvetica", 11))
        list_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.position_listbox = tk.Listbox(list_frame, font=("Courier", 9), 
                                          bg='#34495e', fg='#ecf0f1',
                                          selectmode=tk.SINGLE,
                                          yscrollcommand=scrollbar.set)
        self.position_listbox.pack(fill="both", expand=True, padx=5, pady=5)
        scrollbar.config(command=self.position_listbox.yview)
        
        # Position management buttons
        btn_frame = tk.Frame(right_frame, bg='#34495e')
        btn_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Button(btn_frame, text="Move Up", font=("Helvetica", 9), 
                 bg='#3498db', fg='white', width=10,
                 command=self.move_position_up).pack(side=tk.LEFT, padx=2)
        
        tk.Button(btn_frame, text="Move Down", font=("Helvetica", 9), 
                 bg='#3498db', fg='white', width=10,
                 command=self.move_position_down).pack(side=tk.LEFT, padx=2)
        
        tk.Button(btn_frame, text="Delete", font=("Helvetica", 9), 
                 bg='#e74c3c', fg='white', width=10,
                 command=self.delete_position).pack(side=tk.LEFT, padx=2)
        
        tk.Button(btn_frame, text="Clear All", font=("Helvetica", 9), 
                 bg='#c0392b', fg='white', width=10,
                 command=self.clear_all_positions).pack(side=tk.LEFT, padx=2)
        
        # File controls
        file_frame = tk.LabelFrame(right_frame, text="File Operations", 
                                  bg='#2c3e50', fg='#ecf0f1', font=("Helvetica", 10))
        file_frame.pack(fill="x", padx=10, pady=5)
        
        file_btn_row = tk.Frame(file_frame, bg='#2c3e50')
        file_btn_row.pack(pady=5)
        
        tk.Button(file_btn_row, text="Save Program", font=("Helvetica", 10), 
                 bg='#16a085', fg='white', width=15,
                 command=self.save_program).pack(side=tk.LEFT, padx=5)
        
        tk.Button(file_btn_row, text="Load Program", font=("Helvetica", 10), 
                 bg='#2980b9', fg='white', width=15,
                 command=self.load_program).pack(side=tk.LEFT, padx=5)
        
        self.filename_label = tk.Label(file_frame, text="No file loaded", 
                                      font=("Helvetica", 9), bg='#2c3e50', fg='#95a5a6')
        self.filename_label.pack(pady=2)
        
        # Playback controls
        playback_frame = tk.LabelFrame(right_frame, text="Playback Control", 
                                      bg='#2c3e50', fg='#ecf0f1', font=("Helvetica", 10))
        playback_frame.pack(fill="x", padx=10, pady=5)
        
        # Speed control
        speed_row = tk.Frame(playback_frame, bg='#2c3e50')
        speed_row.pack(pady=5)
        
        tk.Label(speed_row, text="Playback Speed:", font=("Helvetica", 9), 
                bg='#2c3e50', fg='#ecf0f1').pack(side=tk.LEFT, padx=5)
        
        self.playback_speed_slider = tk.Scale(speed_row, from_=0.1, to=2.0, resolution=0.1,
                                             orient=tk.HORIZONTAL, length=150,
                                             bg='#34495e', fg='#ecf0f1', troughcolor='#7f8c8d',
                                             command=self.update_playback_speed)
        self.playback_speed_slider.set(1.0)
        self.playback_speed_slider.pack(side=tk.LEFT, padx=5)
        
        self.playback_speed_label = tk.Label(speed_row, text="1.0x", 
                                             font=("Helvetica", 9), 
                                             bg='#2c3e50', fg='#f39c12', width=6)
        self.playback_speed_label.pack(side=tk.LEFT, padx=5)
        
        # Loop checkbox
        self.loop_var = tk.BooleanVar()
        loop_check = tk.Checkbutton(playback_frame, text="Loop Playback", 
                                   variable=self.loop_var, font=("Helvetica", 9),
                                   bg='#2c3e50', fg='#ecf0f1', selectcolor='#34495e',
                                   command=self.toggle_loop)
        loop_check.pack(pady=5)
        
        # Play buttons
        play_btn_row = tk.Frame(playback_frame, bg='#2c3e50')
        play_btn_row.pack(pady=5)
        
        self.play_button = tk.Button(play_btn_row, text="PLAY", 
                                     font=("Helvetica", 11), bg='#27ae60', 
                                     fg='white', width=15, height=2,
                                     command=self.play_program)
        self.play_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = tk.Button(play_btn_row, text="STOP", 
                                     font=("Helvetica", 11), bg='#e74c3c', 
                                     fg='white', width=15, height=2,
                                     command=self.stop_playback, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        # Status bar
        self.status_label = tk.Label(right_frame, text="Ready", 
                                     font=("Helvetica", 10), bg='#2c3e50', 
                                     fg='#2ecc71', relief=tk.SUNKEN, anchor='w')
        self.status_label.pack(fill="x", padx=10, pady=5)
    
    def setup_ros(self):
        # Publishers
        self.robot_command_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/command",
            10
        )
        
        # Subscribers
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
    
    def on_speed_change(self, value):
        self.speed_percentage = int(value)
        self.speed_value_label.config(text=f"Speed: {self.speed_percentage}%")
        
        if self.speed_percentage < 50:
            color = '#e74c3c'
        elif self.speed_percentage < 100:
            color = '#f39c12'
        elif self.speed_percentage == 100:
            color = '#2ecc71'
        else:
            color = '#3498db'
        
        self.speed_value_label.config(fg=color)
    
    def joint_states_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
        
        try:
            joint_angles_rad = self.joint_positions[:5]
            self.current_cartesian = self.ik_solver.forward_kinematics(joint_angles_rad)
        except Exception as e:
            self.get_logger().warn(f"FK calculation failed: {e}")
    
    def _get_axis(self, axes, index, default=0.0):
        """Safely get an axis value, returning default if index is out of range."""
        return axes[index] if index < len(axes) else default

    def _get_button(self, buttons, index, default=0):
        """Safely get a button value, returning default if index is out of range."""
        return buttons[index] if index < len(buttons) else default

    def joy_callback(self, msg):
        buttons = msg.buttons
        axes    = msg.axes

        # buttons[9]=L1, buttons[10]=R1, axes[4]=L2, axes[5]=R2, buttons[0]=X
        button_l1 = self._get_button(buttons, 9)
        button_r1 = self._get_button(buttons, 10)
        button_l2 = 1 if self._get_axis(axes, 4) < -0.5 else 0   # L2 axis: pushed=-1, released=1
        button_r2 = 1 if self._get_axis(axes, 5) < -0.5 else 0   # R2 axis: pushed=-1, released=1
        button_X  = self._get_button(buttons, 0)

        current_time = time.time()

        # Mode switch (X button)
        if current_time - self.last_button_press_time > self.debounce_duration:
            if button_X == 1:
                self.control_mode = "CARTESIAN" if self.control_mode == "JOINT" else "JOINT"
                self.mode_label.config(text=self.control_mode)
                self.last_button_press_time = current_time
                self.joint_velocity = [0.0] * len(self.joint_names)
                self.cartesian_velocity = [0.0, 0.0, 0.0]
                self.ik_solver.reset_velocity_filter()
                self.current_joint_index = 0

            if button_l1 == 1:
                max_index = 5 if self.control_mode == "JOINT" else 2
                self.current_joint_index = (self.current_joint_index - 1) % (max_index + 1)
                self.last_button_press_time = current_time
            elif button_l2 == 1:
                max_index = 5 if self.control_mode == "JOINT" else 2
                self.current_joint_index = (self.current_joint_index + 1) % (max_index + 1)
                self.last_button_press_time = current_time

        # Robot arm control
        if self.control_mode == "JOINT":
            velocity = self.base_joint_velocity * (self.speed_percentage / 100.0)
            self.joint_velocity = [0.0] * len(self.joint_names)

            if button_r1 == 1:
                self.joint_velocity[self.current_joint_index] = velocity
                self.is_moving = True
            elif button_r2 == 1:
                self.joint_velocity[self.current_joint_index] = -velocity
                self.is_moving = True
            else:
                self.is_moving = False
        else:
            velocity_mm_s = self.base_cartesian_velocity * (self.speed_percentage / 100.0)
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

        self.last_buttons = list(buttons)
        self.last_axes = list(axes)
    
    def continuous_control_loop(self):
        rate_hz = 50
        dt = 1.0 / rate_hz
        
        while rclpy.ok():
            try:
                if self.control_mode == "JOINT":
                    self.update_joint_control()
                else:
                    self.update_cartesian_control()
                
                time.sleep(dt)
            except Exception as e:
                self.get_logger().error(f"Control loop error: {e}")
    
    def update_joint_control(self):
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
    
    def _make_duration(self, seconds):
        """Convert float seconds to builtin_interfaces/Duration."""
        dur = Duration()
        dur.sec = int(seconds)
        dur.nanosec = int((seconds - dur.sec) * 1e9)
        return dur

    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.target_joint_positions
        point.time_from_start = self._make_duration(0.02)
        
        msg.points.append(point)
        self.robot_command_pub.publish(msg)
    
    def teach_current_position(self):
        if self.is_playing:
            self.update_status("Cannot teach while playing!", '#e74c3c')
            return
        
        position = {
            'joints': self.joint_positions.copy(),
            'cartesian': self.current_cartesian.copy(),
            'time': 2.0
        }
        self.taught_positions.append(position)
        self.update_position_listbox()
        self.update_status(f"Position {len(self.taught_positions)} taught", '#2ecc71')
        self.get_logger().info(f"Taught position {len(self.taught_positions)}")
    
    def toggle_recording(self):
        if self.is_playing:
            self.update_status("Cannot record while playing!", '#e74c3c')
            return
        
        if not self.is_recording:
            self.is_recording = True
            self.record_button.config(text="STOP RECORDING", bg='#c0392b')
            self.update_status("Recording... (1 pos/sec)", '#e67e22')
            threading.Thread(target=self.recording_loop, daemon=True).start()
        else:
            self.is_recording = False
            self.record_button.config(text="START RECORDING", bg='#e74c3c')
            self.update_status("Recording stopped", '#95a5a6')
    
    def recording_loop(self):
        while self.is_recording and rclpy.ok():
            self.teach_current_position()
            time.sleep(1.0)
    
    def update_position_listbox(self):
        self.position_listbox.delete(0, tk.END)
        for i, pos in enumerate(self.taught_positions):
            joints_str = ", ".join([f"{j:.3f}" for j in pos['joints']])
            cart_str = f"[{pos['cartesian'][0]:.1f}, {pos['cartesian'][1]:.1f}, {pos['cartesian'][2]:.1f}]"
            self.position_listbox.insert(tk.END, 
                f"#{i+1:02d} | Joints: {joints_str[:40]}... | Cart: {cart_str} | t={pos['time']:.1f}s")
    
    def move_position_up(self):
        selection = self.position_listbox.curselection()
        if not selection or selection[0] == 0:
            return
        
        idx = selection[0]
        self.taught_positions[idx], self.taught_positions[idx-1] = \
            self.taught_positions[idx-1], self.taught_positions[idx]
        
        self.update_position_listbox()
        self.position_listbox.selection_set(idx-1)
    
    def move_position_down(self):
        selection = self.position_listbox.curselection()
        if not selection or selection[0] == len(self.taught_positions) - 1:
            return
        
        idx = selection[0]
        self.taught_positions[idx], self.taught_positions[idx+1] = \
            self.taught_positions[idx+1], self.taught_positions[idx]
        
        self.update_position_listbox()
        self.position_listbox.selection_set(idx+1)
    
    def delete_position(self):
        selection = self.position_listbox.curselection()
        if not selection:
            return
        
        idx = selection[0]
        del self.taught_positions[idx]
        self.update_position_listbox()
        self.update_status(f"Position {idx+1} deleted", '#e74c3c')
    
    def clear_all_positions(self):
        if messagebox.askyesno("Clear All", "Delete all taught positions?"):
            self.taught_positions.clear()
            self.update_position_listbox()
            self.update_status("All positions cleared", '#95a5a6')
    
    def save_program(self):
        if not self.taught_positions:
            messagebox.showwarning("No Positions", "No positions to save!")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~")
        )
        
        if filename:
            try:
                data = {
                    'joint_names': self.joint_names,
                    'positions': self.taught_positions,
                    'version': '2.0',
                    'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
                }
                
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=2)
                
                self.current_filename = filename
                self.filename_label.config(text=f"File: {os.path.basename(filename)}")
                self.update_status(f"Program saved: {os.path.basename(filename)}", '#27ae60')
                messagebox.showinfo("Success", 
                    f"Program saved successfully!\n{len(self.taught_positions)} positions")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save: {str(e)}")
    
    def load_program(self):
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~")
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                
                self.taught_positions = data['positions']
                self.update_position_listbox()
                
                self.current_filename = filename
                self.filename_label.config(text=f"File: {os.path.basename(filename)}")
                self.update_status(f"Program loaded: {os.path.basename(filename)}", '#2980b9')
                messagebox.showinfo("Success", 
                    f"Program loaded successfully!\n{len(self.taught_positions)} positions")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load: {str(e)}")
    
    def update_playback_speed(self, value):
        self.playback_speed = float(value)
        self.playback_speed_label.config(text=f"{self.playback_speed:.1f}x")
    
    def toggle_loop(self):
        self.loop_playback = self.loop_var.get()
    
    def play_program(self):
        if not self.taught_positions:
            messagebox.showwarning("No Program", "No positions to play!")
            return
        
        if self.is_playing:
            return
        
        self.is_playing = True
        self.play_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.teach_button.config(state=tk.DISABLED)
        self.record_button.config(state=tk.DISABLED)
        
        threading.Thread(target=self.playback_loop, daemon=True).start()
    
    def stop_playback(self):
        self.is_playing = False
        self.play_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.teach_button.config(state=tk.NORMAL)
        self.record_button.config(state=tk.NORMAL)
        self.update_status("Playback stopped", '#e74c3c')
    
    def playback_loop(self):
        try:
            while self.is_playing and rclpy.ok():
                for i, position in enumerate(self.taught_positions):
                    if not self.is_playing:
                        break
                    
                    self.update_status(
                        f"Playing position {i+1}/{len(self.taught_positions)}", '#3498db')
                    
                    # Send command to robot
                    msg = JointTrajectory()
                    msg.joint_names = self.joint_names
                    
                    point = JointTrajectoryPoint()
                    point.positions = position['joints']
                    point.time_from_start = self._make_duration(
                        position['time'] / self.playback_speed)
                    
                    msg.points.append(point)
                    self.robot_command_pub.publish(msg)
                    
                    # Wait for movement to complete
                    time.sleep(position['time'] / self.playback_speed)
                
                if not self.loop_playback:
                    break
            
            if self.is_playing:
                self.update_status("Playback completed", '#27ae60')
        except Exception as e:
            self.get_logger().error(f"Playback error: {e}")
            self.update_status(f"Playback error: {e}", '#e74c3c')
        finally:
            self.root.after(100, self.stop_playback)
    
    def update_status(self, text, color='#ecf0f1'):
        self.status_label.config(text=text, fg=color)
    
    def update_gui(self):
        # Update position displays
        for i, name in enumerate(self.joint_names):
            self.joint_labels[name].config(text=f"{self.joint_positions[i]:.3f}")
        
        try:
            for i, axis in enumerate(['X', 'Y', 'Z']):
                value = self.current_cartesian[i]
                self.cartesian_labels[axis].config(text=f"{value:.1f}")
        except (IndexError, TypeError):
            pass
        
        # Update selected joint/axis highlighting
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
        
        self.root.after(50, self.update_gui)
    
    def spin_ros(self):
        """Run rclpy.spin in a background thread."""
        rclpy.spin(self)

    def run(self):
        # Run ROS2 spin in a background thread
        ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        ros_thread.start()

        def check_ros():
            if not rclpy.ok():
                self.root.destroy()
            else:
                self.root.after(100, check_ros)

        self.root.after(100, check_ros)
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    pendant = EnhancedTeachingPendant(root)
    pendant.update_gui()
    
    try:
        pendant.run()
    except KeyboardInterrupt:
        pass
    finally:
        pendant.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
