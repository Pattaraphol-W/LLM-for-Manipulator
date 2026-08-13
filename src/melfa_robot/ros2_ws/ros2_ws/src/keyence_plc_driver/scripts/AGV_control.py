#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Joy
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
import os
import time
import threading


class AGVTeachingControl(Node):
    def __init__(self, root):
        super().__init__('agv_teaching_control')
        
        self.root = root
        self.root.title("AGV Controller with Teaching Mode")
        self.root.geometry("1400x750")
        self.root.configure(bg='#2c3e50')
        
        # State tracking
        self.last_buttons = [0] * 13
        self.last_axes = [0.0] * 10
        self.plc_states = {}
        self.last_active_mr = None
        self.last_stick_update_time = 0
        self.stick_update_interval = 0
        
        # Teaching state
        self.taught_directions = []
        self.is_playing = False
        self.is_recording = False
        self.current_filename = None
        self.loop_playback = False
        self.current_direction = None
        self.direction_start_time = None
        
        self.setup_gui()
        self.setup_ros()
        
        self.get_logger().info("AGV Teaching Control started")
    
    def setup_gui(self):
        main_container = tk.Frame(self.root, bg='#2c3e50')
        main_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        # ========== LEFT SIDE: CONTROLLER ==========
        left_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        tk.Label(left_frame, text="CONTROLLER", font=("Arial", 12, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=8)
        
        # Stick values display
        sticks_frame = tk.Frame(left_frame, bg='#2c3e50', relief="sunken", bd=2)
        sticks_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(sticks_frame, text="Left Y:", font=("Arial", 10, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=0, column=0, padx=8, pady=5, sticky="w")
        self.left_stick_var = tk.StringVar(value="0.00")
        tk.Label(sticks_frame, textvariable=self.left_stick_var, font=("Arial", 12, "bold"), 
                bg='#2c3e50', fg='#f39c12', width=6).grid(row=0, column=1, padx=8, pady=5)
        
        tk.Label(sticks_frame, text="Right X:", font=("Arial", 10, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=1, column=0, padx=8, pady=5, sticky="w")
        self.right_stick_var = tk.StringVar(value="0.00")
        tk.Label(sticks_frame, textvariable=self.right_stick_var, font=("Arial", 12, "bold"), 
                bg='#2c3e50', fg='#f39c12', width=6).grid(row=1, column=1, padx=8, pady=5)
        
        tk.Label(sticks_frame, text="Direction:", font=("Arial", 10, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=2, column=0, padx=8, pady=5, sticky="w")
        self.active_mr_var = tk.StringVar(value="STOP")
        tk.Label(sticks_frame, textvariable=self.active_mr_var, font=("Arial", 11, "bold"), 
                bg='#2c3e50', fg='#2ecc71', width=12).grid(row=2, column=1, padx=8, pady=5)
        
        # Button mappings display
        mappings_frame = tk.Frame(left_frame, bg='#34495e')
        mappings_frame.pack(padx=10, pady=5, fill="both", expand=True)
        
        tk.Label(mappings_frame, text="BUTTON MAPPINGS", font=("Arial", 11, "bold"), 
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
            frame = tk.Frame(mappings_frame, bg='#2c3e50', relief="raised", bd=1)
            frame.pack(padx=8, pady=3, fill="x")
            
            tk.Label(frame, text=btn_name, font=("Arial", 9, "bold"), 
                    bg='#2c3e50', fg='#ecf0f1', width=14, anchor="w").pack(side="left", padx=5, pady=5)
            
            tk.Label(frame, text=function, font=("Arial", 8), 
                    bg='#2c3e50', fg='#f39c12', anchor="w").pack(side="left", padx=5, fill="x", expand=True)
            
            indicator = tk.Canvas(frame, width=20, height=20, bg='#2c3e50', highlightthickness=0)
            indicator.pack(side="right", padx=5, pady=5)
            circle = indicator.create_oval(2, 2, 18, 18, fill='#7f8c8d', outline='')
            
            self.button_frames[key] = {
                'indicator': indicator, 
                'circle': circle,
                'mr_address': mr_address
            }
        
        # ========== MIDDLE: PLC VALUES ==========
        middle_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        middle_frame.pack(side="left", fill="both", expand=True, padx=5)
        
        tk.Label(middle_frame, text="PLC STATUS", font=("Arial", 12, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=8)
        
        # PLC values frame
        plc_frame = tk.Frame(middle_frame, bg='#34495e')
        plc_frame.pack(padx=10, pady=5, fill="both", expand=True)
        
        self.plc_vars = {}
        self.plc_indicators = {}
        
        display_config = [
            ('MR012', 'Ready'),
            ('MR013', 'Running'),
            ('MR100', 'Back FAR'),
            ('MR101', 'Back Mid'),
            ('MR102', 'Back Close'),
            ('MR103', 'Back Broken'),
            ('MR104', 'Front FAR'),
            ('MR105', 'Front Mid'),
            ('MR106', 'Front Close'),
            ('MR107', 'Front Broken'),
            ('R603', 'DC 24V'),
            ('R604', 'DC 12V'),
            ('R605', 'DC 5V'),
        ]
        
        all_mr_addresses = ['R603', 'R604', 'R605', 'MR206', 'MR207', 'MR208', 'MR209', 
                           'MR400', 'MR401', 'MR402', 'MR403', 'MR404', 'MR405', 'MR406', 
                           'MR602', 'MR603', 'MR604', 'MR605', 'MR012', 'MR013', 
                           'MR100', 'MR101', 'MR102', 'MR103', 'MR104', 'MR105', 'MR106', 'MR107']
        
        for addr in all_mr_addresses:
            self.plc_states[addr] = 0
        
        for addr, label in display_config:
            frame = tk.Frame(plc_frame, bg='#2c3e50', relief="raised", bd=1)
            frame.pack(padx=8, pady=2, fill="x")
            
            tk.Label(frame, text=f"{label}:", font=("Arial", 9, "bold"), 
                    bg='#2c3e50', fg='#9b59b6', width=12, anchor="w").pack(side="left", padx=5, pady=4)
            
            indicator = tk.Canvas(frame, width=20, height=20, bg='#2c3e50', highlightthickness=0)
            indicator.pack(side="right", padx=5, pady=4)
            circle = indicator.create_oval(2, 2, 18, 18, fill='#7f8c8d', outline='#34495e', width=2)
            
            var = tk.StringVar(value="OFF")
            self.plc_vars[addr] = var
            self.plc_indicators[addr] = {'indicator': indicator, 'circle': circle}
        
        # ========== RIGHT: TEACHING PANEL ==========
        right_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        right_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        
        tk.Label(right_frame, text="TEACHING PANEL", font=("Arial", 14, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=10)
        
        # Teach buttons
        teach_btn_frame = tk.Frame(right_frame, bg='#34495e')
        teach_btn_frame.pack(pady=5)
        
        self.record_button = tk.Button(teach_btn_frame, text="START RECORDING", 
                                       font=("Helvetica", 11), bg='#e74c3c', 
                                       fg='white', width=20, height=2,
                                       command=self.toggle_recording)
        self.record_button.pack(padx=5)
        
        # Direction list
        list_frame = tk.LabelFrame(right_frame, text="Taught Route", 
                                   bg='#2c3e50', fg='#ecf0f1', font=("Helvetica", 11))
        list_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.direction_listbox = tk.Listbox(list_frame, font=("Courier", 9), 
                                          bg='#34495e', fg='#ecf0f1',
                                          selectmode=tk.SINGLE,
                                          yscrollcommand=scrollbar.set)
        self.direction_listbox.pack(fill="both", expand=True, padx=5, pady=5)
        scrollbar.config(command=self.direction_listbox.yview)
        
        # Direction management buttons
        btn_frame = tk.Frame(right_frame, bg='#34495e')
        btn_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Button(btn_frame, text="Move Up", font=("Helvetica", 9), 
                 bg='#3498db', fg='white', width=10,
                 command=self.move_direction_up).pack(side=tk.LEFT, padx=2)
        
        tk.Button(btn_frame, text="Move Down", font=("Helvetica", 9), 
                 bg='#3498db', fg='white', width=10,
                 command=self.move_direction_down).pack(side=tk.LEFT, padx=2)
        
        tk.Button(btn_frame, text="Delete", font=("Helvetica", 9), 
                 bg='#e74c3c', fg='white', width=10,
                 command=self.delete_direction).pack(side=tk.LEFT, padx=2)
        
        tk.Button(btn_frame, text="Clear All", font=("Helvetica", 9), 
                 bg='#c0392b', fg='white', width=10,
                 command=self.clear_all_directions).pack(side=tk.LEFT, padx=2)
        
        # File controls
        file_frame = tk.LabelFrame(right_frame, text="File Operations", 
                                  bg='#2c3e50', fg='#ecf0f1', font=("Helvetica", 10))
        file_frame.pack(fill="x", padx=10, pady=5)
        
        file_btn_row = tk.Frame(file_frame, bg='#2c3e50')
        file_btn_row.pack(pady=5)
        
        tk.Button(file_btn_row, text="Save Route", font=("Helvetica", 10), 
                 bg='#16a085', fg='white', width=15,
                 command=self.save_route).pack(side=tk.LEFT, padx=5)
        
        tk.Button(file_btn_row, text="Load Route", font=("Helvetica", 10), 
                 bg='#2980b9', fg='white', width=15,
                 command=self.load_route).pack(side=tk.LEFT, padx=5)
        
        self.filename_label = tk.Label(file_frame, text="No file loaded", 
                                      font=("Helvetica", 9), bg='#2c3e50', fg='#95a5a6')
        self.filename_label.pack(pady=2)
        
        # Playback controls
        playback_frame = tk.LabelFrame(right_frame, text="Playback Control", 
                                      bg='#2c3e50', fg='#ecf0f1', font=("Helvetica", 10))
        playback_frame.pack(fill="x", padx=10, pady=5)
        
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
        
        self.play_button = tk.Button(play_btn_row, text="PLAY ROUTE", 
                                     font=("Helvetica", 11), bg='#27ae60', 
                                     fg='white', width=15, height=2,
                                     command=self.play_route)
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
        self._write_pub = self.create_publisher(String, '/write_command', 10)
        
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        all_mr_addresses = ['R603', 'R604', 'R605', 'MR206', 'MR207', 'MR208', 'MR209', 
                           'MR400', 'MR401', 'MR402', 'MR403', 'MR404', 'MR405', 'MR406', 
                           'MR602', 'MR603', 'MR604', 'MR605', 'MR012', 'MR013', 
                           'MR100', 'MR101', 'MR102', 'MR103', 'MR104', 'MR105', 'MR106', 'MR107']
        
        for addr in all_mr_addresses:
            self.create_subscription(
                Float32,
                f"/plc_ethernet_driver/read/{addr}",
                self._create_plc_callback(addr),
                10
            )
    
    def _get_axis(self, axes, index, default=0.0):
        """Safely get an axis value, returning default if index is out of range."""
        return axes[index] if index < len(axes) else default

    def _get_button(self, buttons, index, default=0):
        """Safely get a button value, returning default if index is out of range."""
        return buttons[index] if index < len(buttons) else default

    def joy_callback(self, msg):
        if self.is_playing:
            return

        axes    = msg.axes
        buttons = msg.buttons

        # axes[1]=Left Y (forward/back),  axes[2]=Right X (left/right)
        left_y  = self._get_axis(axes, 1)
        right_x = self._get_axis(axes, 2)

        self.left_stick_var.set(f"{left_y:.2f}")
        self.right_stick_var.set(f"{right_x:.2f}")

        current_time = time.time()
        if current_time - self.last_stick_update_time >= self.stick_update_interval:
            self._handle_stick_control(left_y, right_x)
            self.last_stick_update_time = current_time

        # Triangle (buttons[3]) - MR207 Motor ON
        if self._get_button(buttons, 3) == 1 and self._get_button(self.last_buttons, 3) == 0:
            self._toggle_plc('MR207')

        # Square (buttons[2]) - MR208 Start
        if self._get_button(buttons, 2) == 1 and self._get_button(self.last_buttons, 2) == 0:
            self._toggle_plc('MR208')

        # Circle (buttons[1]) - MR209 Break
        if self._get_button(buttons, 1) == 1 and self._get_button(self.last_buttons, 1) == 0:
            self._toggle_plc('MR209')

        # D-Pad UP (buttons[11]) - MR603/604 Switch
        if self._get_button(buttons, 11) == 1 and self._get_button(self.last_buttons, 11) == 0:
            mr603_state = self.plc_states.get('MR603', 0)
            if mr603_state == 0:
                self._publish_write_command('MR603', 1)
                self._publish_write_command('MR604', 0)
            else:
                self._publish_write_command('MR603', 0)
                self._publish_write_command('MR604', 1)

        # D-Pad RIGHT (buttons[14]) - MR605 Motor DC
        if self._get_button(buttons, 14) == 1 and self._get_button(self.last_buttons, 14) == 0:
            self._toggle_plc('MR605')

        # D-Pad DOWN (buttons[12]) - MR206 Error Reset
        if self._get_button(buttons, 12) == 1 and self._get_button(self.last_buttons, 12) == 0:
            self._toggle_plc('MR206')

        # D-Pad LEFT (buttons[13]) - MR602 Power Hold
        if self._get_button(buttons, 13) == 1 and self._get_button(self.last_buttons, 13) == 0:
            self._toggle_plc('MR602')

        self.last_buttons = list(buttons)
        self.last_axes = list(axes)
    
    def _toggle_plc(self, address):
        current_state = self.plc_states.get(address, 0)
        new_state = 0 if current_state == 1 else 1
        self._publish_write_command(address, new_state)
        self.get_logger().info(f"{address} toggled: {current_state} -> {new_state}")
    
    def _handle_stick_control(self, left_y, right_x):
        active_mr = None
        direction_text = None

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

        if active_mr is None:
            if self.last_active_mr is not None and self.last_active_mr != 'MR406':
                active_mr = 'MR406'
                direction_text = "STOP"
            else:
                return

        if active_mr != self.last_active_mr:
            # Recording: Save previous direction before switching
            if self.is_recording and self.current_direction and self.direction_start_time:
                duration = time.time() - self.direction_start_time
                if duration > 0.1:
                    self.taught_directions.append({
                        'mr_address': self.current_direction,
                        'direction': self._get_direction_name(self.current_direction),
                        'duration': duration
                    })
                    self.update_direction_listbox()
                    self.get_logger().info(f"Recorded: {self._get_direction_name(self.current_direction)} for {duration:.2f}s")
                
                self.current_direction = None
                self.direction_start_time = None
            
            self.get_logger().info(f"Stick control: {self.last_active_mr} -> {active_mr} ({direction_text})")

            if self.last_active_mr is not None:
                self._publish_write_command(self.last_active_mr, 0)
                time.sleep(0.05)

            self._publish_write_command(active_mr, 1)
            
            # Recording: Start timing new direction (including STOP)
            if self.is_recording:
                self.current_direction = active_mr
                self.direction_start_time = time.time()
            
            self.last_active_mr = active_mr
            if direction_text:
                self.active_mr_var.set(direction_text)
    
    def _get_direction_name(self, mr_address):
        direction_map = {
            'MR400': 'FORWARD',
            'MR401': 'FORWARD RIGHT',
            'MR402': 'FORWARD LEFT',
            'MR403': 'BACKWARD',
            'MR404': 'BACKWARD RIGHT',
            'MR405': 'BACKWARD LEFT',
            'MR406': 'STOP'
        }
        return direction_map.get(mr_address, 'UNKNOWN')
    
    def toggle_recording(self):
        if self.is_playing:
            self.update_status("Cannot record while playing!", '#e74c3c')
            return
        
        if not self.is_recording:
            self.is_recording = True
            self.current_direction = None
            self.direction_start_time = None
            self.record_button.config(text="STOP RECORDING", bg='#c0392b')
            self.update_status("Recording route...", '#e67e22')
            self.get_logger().info("Recording started")
        else:
            # Save last direction if recording
            if self.current_direction and self.direction_start_time:
                duration = time.time() - self.direction_start_time
                if duration > 0.1:
                    self.taught_directions.append({
                        'mr_address': self.current_direction,
                        'direction': self._get_direction_name(self.current_direction),
                        'duration': duration
                    })
                    self.update_direction_listbox()
                    self.get_logger().info(f"Recorded: {self._get_direction_name(self.current_direction)} for {duration:.2f}s")
            
            self.is_recording = False
            self.current_direction = None
            self.direction_start_time = None
            self.record_button.config(text="START RECORDING", bg='#e74c3c')
            self.update_status("Recording stopped", '#95a5a6')
            self.get_logger().info("Recording stopped")
    
    def update_direction_listbox(self):
        self.direction_listbox.delete(0, tk.END)
        for i, direction in enumerate(self.taught_directions):
            self.direction_listbox.insert(tk.END, 
                f"#{i+1:02d} | {direction['direction']:15s} | Duration: {direction['duration']:.2f}s")
    
    def move_direction_up(self):
        selection = self.direction_listbox.curselection()
        if not selection or selection[0] == 0:
            return
        
        idx = selection[0]
        self.taught_directions[idx], self.taught_directions[idx-1] = \
            self.taught_directions[idx-1], self.taught_directions[idx]
        
        self.update_direction_listbox()
        self.direction_listbox.selection_set(idx-1)
    
    def move_direction_down(self):
        selection = self.direction_listbox.curselection()
        if not selection or selection[0] == len(self.taught_directions) - 1:
            return
        
        idx = selection[0]
        self.taught_directions[idx], self.taught_directions[idx+1] = \
            self.taught_directions[idx+1], self.taught_directions[idx]
        
        self.update_direction_listbox()
        self.direction_listbox.selection_set(idx+1)
    
    def delete_direction(self):
        selection = self.direction_listbox.curselection()
        if not selection:
            return
        
        idx = selection[0]
        del self.taught_directions[idx]
        self.update_direction_listbox()
        self.update_status(f"Direction {idx+1} deleted", '#e74c3c')
    
    def clear_all_directions(self):
        if messagebox.askyesno("Clear All", "Delete all taught directions?"):
            self.taught_directions.clear()
            self.update_direction_listbox()
            self.update_status("All directions cleared", '#95a5a6')
    
    def save_route(self):
        if not self.taught_directions:
            messagebox.showwarning("No Route", "No directions to save!")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~")
        )
        
        if filename:
            try:
                data = {
                    'directions': self.taught_directions,
                    'version': '1.0',
                    'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
                }
                
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=2)
                
                self.current_filename = filename
                self.filename_label.config(text=f"File: {os.path.basename(filename)}")
                self.update_status(f"Route saved: {os.path.basename(filename)}", '#27ae60')
                messagebox.showinfo("Success", 
                    f"Route saved successfully!\n{len(self.taught_directions)} directions")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save: {str(e)}")
    
    def load_route(self):
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~")
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = json.load(f)
                
                self.taught_directions = data['directions']
                self.update_direction_listbox()
                
                self.current_filename = filename
                self.filename_label.config(text=f"File: {os.path.basename(filename)}")
                self.update_status(f"Route loaded: {os.path.basename(filename)}", '#2980b9')
                messagebox.showinfo("Success", 
                    f"Route loaded successfully!\n{len(self.taught_directions)} directions")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load: {str(e)}")
    
    def toggle_loop(self):
        self.loop_playback = self.loop_var.get()
    
    def play_route(self):
        if not self.taught_directions:
            messagebox.showwarning("No Route", "No directions to play!")
            return
        
        if self.is_playing:
            return
        
        self.is_playing = True
        self.play_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.record_button.config(state=tk.DISABLED)
        
        threading.Thread(target=self.playback_loop, daemon=True).start()
    
    def stop_playback(self):
        self.is_playing = False
        
        # Stop any active movement
        if self.last_active_mr and self.last_active_mr != 'MR406':
            self._publish_write_command(self.last_active_mr, 0)
            self.last_active_mr = None
        
        self.play_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.record_button.config(state=tk.NORMAL)
        self.update_status("Playback stopped", '#e74c3c')
    
    def playback_loop(self):
        try:
            while self.is_playing and rclpy.ok():
                for i, direction in enumerate(self.taught_directions):
                    if not self.is_playing:
                        break
                    
                    self.update_status(
                        f"Playing step {i+1}/{len(self.taught_directions)}: {direction['direction']}", '#3498db')
                    
                    mr_address = direction['mr_address']
                    duration = direction['duration']
                    
                    # Activate direction
                    self.get_logger().info(f"Playing: {direction['direction']} for {duration:.2f}s")
                    self._publish_write_command(mr_address, 1)
                    self.last_active_mr = mr_address
                    self.active_mr_var.set(direction['direction'])
                    
                    # Hold for duration
                    time.sleep(duration)
                    
                    # Deactivate direction
                    self._publish_write_command(mr_address, 0)
                    time.sleep(0.05)
                
                if not self.loop_playback:
                    break
            
            # Ensure stop - activate MR406 then deactivate
            if self.last_active_mr:
                self._publish_write_command(self.last_active_mr, 0)
                time.sleep(0.05)
            
            # Send MR406 = 1 then 0 to properly stop
            self._publish_write_command('MR406', 1)
            time.sleep(0.1)
            self._publish_write_command('MR406', 0)
            self.last_active_mr = None
            
            if self.is_playing:
                self.update_status("Playback completed", '#27ae60')
        except Exception as e:
            self.get_logger().error(f"Playback error: {e}")
            self.update_status(f"Playback error: {e}", '#e74c3c')
        finally:
            self.root.after(100, self.stop_playback)
    
    def _create_plc_callback(self, address):
        def callback(msg):
            self.plc_states[address] = msg.data
            
            if address in self.plc_indicators:
                self.root.after(0, lambda: self._update_plc_led(address, msg.data))
            
            self.root.after(0, lambda: self._update_button_indicators())
        return callback
    
    def _update_plc_led(self, address, value):
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
        msg = String()
        msg.data = f"{address},{data}"
        self.get_logger().info(f"[GUI] Publishing write command: {msg.data}")
        self._write_pub.publish(msg)
    
    def update_status(self, text, color='#ecf0f1'):
        self.status_label.config(text=text, fg=color)
    
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
    app = AGVTeachingControl(root)
    
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        app.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
