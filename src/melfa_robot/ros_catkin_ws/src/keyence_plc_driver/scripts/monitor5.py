#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Joy
import tkinter as tk
from tkinter import ttk
import time

class CombinedMonitorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PS4 Controller & PLC Monitor")
        self.root.geometry("1200x1000")
        self.root.configure(bg='#2c3e50')
        
        # State tracking for controller
        self.last_buttons = [0] * 13
        self.last_axes = [0.0] * 10
        
        # PLC states - stores current state from PLC read values
        self.plc_states = {}
        
        # Stick control state tracking
        self.last_active_mr = None
        self.last_stick_update_time = 0
        self.stick_update_interval = 0  # Update stick control every 100ms (10Hz max)
        
        # Style configuration
        style = ttk.Style()
        style.theme_use('clam')
        
        # ========== MAIN CONTAINER ==========
        main_container = tk.Frame(self.root, bg='#2c3e50')
        main_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        # ========== LEFT SIDE: CONTROLLER ==========
        left_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        tk.Label(left_frame, text="CONTROLLER", font=("Arial", 18, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=15)
        
        # Stick values display
        sticks_frame = tk.Frame(left_frame, bg='#2c3e50', relief="sunken", bd=2)
        sticks_frame.pack(padx=15, pady=10, fill="x")
        
        tk.Label(sticks_frame, text="Left Stick (Y):", font=("Arial", 12, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=0, column=0, padx=10, pady=8, sticky="w")
        self.left_stick_var = tk.StringVar(value="0.00")
        tk.Label(sticks_frame, textvariable=self.left_stick_var, font=("Arial", 16, "bold"), 
                bg='#2c3e50', fg='#f39c12', width=8).grid(row=0, column=1, padx=10, pady=8)
        
        tk.Label(sticks_frame, text="Right Stick (X):", font=("Arial", 12, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=1, column=0, padx=10, pady=8, sticky="w")
        self.right_stick_var = tk.StringVar(value="0.00")
        tk.Label(sticks_frame, textvariable=self.right_stick_var, font=("Arial", 16, "bold"), 
                bg='#2c3e50', fg='#f39c12', width=8).grid(row=1, column=1, padx=10, pady=8)
        
        # Active MR indicator with direction
        tk.Label(sticks_frame, text="Direction:", font=("Arial", 12, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=2, column=0, padx=10, pady=8, sticky="w")
        self.active_mr_var = tk.StringVar(value="STOP")
        tk.Label(sticks_frame, textvariable=self.active_mr_var, font=("Arial", 14, "bold"), 
                bg='#2c3e50', fg='#2ecc71', width=15).grid(row=2, column=1, padx=10, pady=8)
        
        # Button mappings display
        mappings_frame = tk.Frame(left_frame, bg='#34495e')
        mappings_frame.pack(padx=15, pady=10, fill="both", expand=True)
        
        tk.Label(mappings_frame, text="BUTTON MAPPINGS", font=("Arial", 14, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=10)
        
        # Create button mapping displays
        self.button_frames = {}
        mappings = [
            ("^ Triangle", "MR207 Motor ON", "btn^", "MR207"),
            ("[] Square", "MR208 Start", "btn[]", "MR208"),
            ("o Circle", "MR209 Break", "btnO", "MR209"),
            ("⬆️ D-Pad UP", "MR603/604 Switch", "btn4", "MR603"),  # Lights when MR603 is ON
            ("➡️ D-Pad RIGHT", "MR605 Motor DC", "btn5", "MR605"),
            ("⬇️ D-Pad DOWN", "MR206 Error Reset", "btn6", "MR206"),
            ("⬅️ D-Pad LEFT", "MR602 Power Hold", "btn7", "MR602"),
        ]
        
        for btn_name, function, key, mr_address in mappings:
            frame = tk.Frame(mappings_frame, bg='#2c3e50', relief="raised", bd=2)
            frame.pack(padx=10, pady=5, fill="x")
            
            tk.Label(frame, text=btn_name, font=("Arial", 11, "bold"), 
                    bg='#2c3e50', fg='#ecf0f1', width=18, anchor="w").pack(side="left", padx=10, pady=8)
            
            tk.Label(frame, text=function, font=("Arial", 10), 
                    bg='#2c3e50', fg='#f39c12', anchor="w").pack(side="left", padx=10, pady=8, fill="x", expand=True)
            
            indicator = tk.Canvas(frame, width=25, height=25, bg='#2c3e50', highlightthickness=0)
            indicator.pack(side="right", padx=10, pady=8)
            circle = indicator.create_oval(3, 3, 22, 22, fill='#7f8c8d', outline='')
            
            self.button_frames[key] = {
                'frame': frame, 
                'indicator': indicator, 
                'circle': circle,
                'mr_address': mr_address
            }
        
        # ========== RIGHT SIDE: PLC VALUES ==========
        right_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        right_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        
        tk.Label(right_frame, text="PLC VALUES", font=("Arial", 18, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=15)
        
        # PLC values frame
        plc_frame = tk.Frame(right_frame, bg='#34495e')
        plc_frame.pack(padx=10, pady=10, fill="both", expand=True)
        
        # Create PLC value displays with descriptive names and LED indicators
        self.plc_vars = {}
        self.plc_indicators = {}  # Store LED canvas references
        
        # MR registers to display with their descriptive names
        display_config = [
            ('MR012', 'Ready'),
            ('MR013', 'Running'),
            ('MR100', 'Back Object FAR Range'),
            ('MR101', 'Back Object Mid Range'),
            ('MR102', 'Back Object Close Range'),
            ('MR103', 'Back Object Sensor Broken'),
            ('MR104', 'Front Object FAR Range'),
            ('MR105', 'Front Object Mid Range'),
            ('MR106', 'Front Object Close Range'),
            ('MR107', 'Front Object Sensor Broken'),
            ('R603', 'DC 24V ON'),
            ('R604', 'DC 12V ON'),
            ('R605', 'DC 5V ON'),
        ]
        
        # All MR registers that need to be tracked (for button control + display)
        all_mr_addresses = ['R603', 'R604', 'R605', 'MR206', 'MR207', 'MR208', 'MR209', 'MR400', 'MR401', 'MR402', 
                           'MR403', 'MR404', 'MR405', 'MR406', 'MR602', 'MR603', 'MR604', 'MR605',
                           'MR012', 'MR013', 'MR100', 'MR101', 'MR102', 'MR103', 
                           'MR104', 'MR105', 'MR106', 'MR107']
        
        # Initialize all PLC states (for internal tracking)
        for addr in all_mr_addresses:
            self.plc_states[addr] = 0
        
        # Create display with descriptive names and LED indicators
        for addr, label in display_config:
            frame = tk.Frame(plc_frame, bg='#2c3e50', relief="raised", bd=2)
            frame.pack(padx=10, pady=5, fill="x")
            
            # Label (descriptive name)
            tk.Label(frame, text=f"{label}:", font=("Arial", 12, "bold"), 
                    bg='#2c3e50', fg='#9b59b6', width=24, anchor="w").pack(side="left", padx=15, pady=10)
            
            # LED Indicator (like PS4 controller buttons)
            indicator = tk.Canvas(frame, width=30, height=30, bg='#2c3e50', highlightthickness=0)
            indicator.pack(side="right", padx=15, pady=10)
            circle = indicator.create_oval(3, 3, 27, 27, fill='#7f8c8d', outline='#34495e', width=2)
            
            # Store references
            var = tk.StringVar(value="OFF")
            self.plc_vars[addr] = var
            self.plc_indicators[addr] = {'indicator': indicator, 'circle': circle}
        
        # ========== ROS SETUP ==========
        rospy.init_node('combined_monitor_gui', anonymous=True)
        self._rate = rospy.Rate(10)
        self._write_pub = rospy.Publisher('/write_command', String, queue_size=10)
        
        # PS4 Controller subscriber
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # PLC subscribers for read topics - subscribe to ALL addresses we need to track
        for addr in all_mr_addresses:
            rospy.Subscriber(f"/plc_ethernet_driver/read/{addr}", Float32, 
                           self._create_plc_callback(addr))

    def joy_callback(self, msg):
        """Callback for PS4 controller data."""
        # Update stick displays
        left_y = msg.axes[1]
        right_x = msg.axes[3]
        
        self.left_stick_var.set(f"{left_y:.2f}")
        self.right_stick_var.set(f"{right_x:.2f}")
        
        # Rate-limited stick control - only update if enough time has passed
        current_time = time.time()
        if current_time - self.last_stick_update_time >= self.stick_update_interval:
            self._handle_stick_control(left_y, right_x)
            self.last_stick_update_time = current_time
        
        # Button 2: Triangle - MR207 Motor ON (Toggle based on current PLC state)
        if msg.buttons[2] == 1 and self.last_buttons[2] == 0:
            current_state = self.plc_states.get('MR207', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR207', new_state)
            rospy.loginfo(f"MR207 toggled: {current_state} -> {new_state}")
        
        # Button 3: Square - MR208 Start (Toggle based on current PLC state)
        if msg.buttons[3] == 1 and self.last_buttons[3] == 0:
            current_state = self.plc_states.get('MR208', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR208', new_state)
            rospy.loginfo(f"MR208 toggled: {current_state} -> {new_state}")
        
        # Button 1: Circle - MR209 Break (Toggle based on current PLC state)
        if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
            current_state = self.plc_states.get('MR209', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR209', new_state)
            rospy.loginfo(f"MR209 toggled: {current_state} -> {new_state}")
        
        # D-Pad UP: Axes 7 = 1 - MR603/604 Switch (based on MR603 state)
        if msg.axes[7] > 0.5 and self.last_axes[7] <= 0.5:
            rospy.loginfo(f"D-Pad UP pressed! axes[7]={msg.axes[7]}")
            mr603_state = self.plc_states.get('MR603', 0)
            if mr603_state == 0:  # MR603 is OFF
                self._publish_write_command('MR603', 1)
                self._publish_write_command('MR604', 0)
                rospy.loginfo("D-Pad UP: MR603/604 Switch: MR603=1, MR604=0")
            else:  # MR603 is ON
                self._publish_write_command('MR603', 0)
                self._publish_write_command('MR604', 1)
                rospy.loginfo("D-Pad UP: MR603/604 Switch: MR603=0, MR604=1")
        
        # D-Pad RIGHT: Axes 6 = -1 - MR605 Motor DC (Toggle based on current PLC state)
        if msg.axes[6] < -0.5 and self.last_axes[6] >= -0.5:
            rospy.loginfo(f"D-Pad RIGHT pressed! axes[6]={msg.axes[6]}")
            current_state = self.plc_states.get('MR605', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR605', new_state)
            rospy.loginfo(f"D-Pad RIGHT: MR605 toggled: {current_state} -> {new_state}")
        
        # D-Pad DOWN: Axes 7 = -1 - MR206 Error Reset (Toggle based on current PLC state)
        if msg.axes[7] < -0.5 and self.last_axes[7] >= -0.5:
            rospy.loginfo(f"D-Pad DOWN pressed! axes[7]={msg.axes[7]}")
            current_state = self.plc_states.get('MR206', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR206', new_state)
            rospy.loginfo(f"D-Pad DOWN: MR206 toggled: {current_state} -> {new_state}")
        
        # D-Pad LEFT: Axes 6 = 1 - MR602 Power Holding (Toggle based on current PLC state)
        if msg.axes[6] > 0.5 and self.last_axes[6] <= 0.5:
            rospy.loginfo(f"D-Pad LEFT pressed! axes[6]={msg.axes[6]}")
            current_state = self.plc_states.get('MR602', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR602', new_state)
            rospy.loginfo(f"D-Pad LEFT: MR602 toggled: {current_state} -> {new_state}")
        
        # Update last button and axes states
        self.last_buttons = list(msg.buttons)
        self.last_axes = list(msg.axes)
    
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
            rospy.loginfo(f"Stick control: {self.last_active_mr} -> {active_mr} ({direction_text}) [L_Y:{left_y:.2f}, R_X:{right_x:.2f}]")

            if self.last_active_mr is not None:
                self._publish_write_command(self.last_active_mr, 0)
                rospy.sleep(0.05)

            self._publish_write_command(active_mr, 1)
            
            self.last_active_mr = active_mr
            if direction_text:
                self.active_mr_var.set(direction_text)

    def _create_plc_callback(self, address):
        """Creates a callback function to update PLC values and store current state."""
        def callback(msg):
            # Store the actual PLC state
            self.plc_states[address] = msg.data
            
            # Update LED display ONLY if this address is in the display list
            if address in self.plc_indicators:
                self.root.after(0, lambda: self._update_plc_led(address, msg.data))
            
            # Update button indicators based on relay state
            self.root.after(0, lambda: self._update_button_indicators())
        return callback

    def _update_plc_led(self, address, value):
        """Update the LED indicator for a PLC value."""
        if address not in self.plc_indicators:
            return
        
        indicator = self.plc_indicators[address]['indicator']
        circle = self.plc_indicators[address]['circle']
        
        # Update LED color based on value
        if value == -1.0:
            # Error state - Red
            indicator.itemconfig(circle, fill='#e74c3c', outline='#c0392b')
            self.plc_vars[address].set("ERROR")
        elif value == 1.0 or value == 1:
            # ON state - Bright Green
            indicator.itemconfig(circle, fill='#2ecc71', outline='#27ae60')
            self.plc_vars[address].set("ON")
        else:
            # OFF state - Gray
            indicator.itemconfig(circle, fill='#7f8c8d', outline='#34495e')
            self.plc_vars[address].set("OFF")

    def _update_button_indicators(self):
        """Update button indicator lights based on actual relay states."""
        for key, button_info in self.button_frames.items():
            mr_address = button_info['mr_address']
            indicator = button_info['indicator']
            circle = button_info['circle']
            
            # Get the current state of the associated MR
            state = self.plc_states.get(mr_address, 0)
            
            # Light up green if relay is ON (1), gray if OFF (0 or other)
            if state == 1:
                indicator.itemconfig(circle, fill='#2ecc71')  # Green
            else:
                indicator.itemconfig(circle, fill='#7f8c8d')  # Gray

    def _publish_write_command(self, address, data):
        """Publishes a write command to the ROS topic."""
        msg = f"{address},{data}"
        rospy.loginfo(f"[GUI] Publishing write command: {msg}")
        self._write_pub.publish(msg)

    def spin(self):
        """Main GUI loop that checks for ROS shutdown."""
        def check_ros():
            if rospy.is_shutdown():
                self.root.destroy()
            else:
                self.root.after(100, check_ros)
        
        self.root.after(100, check_ros)
        self.root.mainloop()

def main():
    root = tk.Tk()
    app = CombinedMonitorGUI(root)
    app.spin()

if __name__ == '__main__':
    main()
