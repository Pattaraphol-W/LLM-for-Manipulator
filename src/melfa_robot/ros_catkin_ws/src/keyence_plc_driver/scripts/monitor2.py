#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Joy
import tkinter as tk
from tkinter import ttk

class CombinedMonitorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PS4 Controller & PLC Monitor")
        self.root.geometry("1000x700")
        self.root.configure(bg='#2c3e50')
        
        # State tracking for controller
        self.last_buttons = [0] * 13
        self.last_axes = [0.0] * 10  # Track axes states for edge detection
        
        # PLC states - stores current state from PLC read values
        self.plc_states = {}
        
        # Style configuration
        style = ttk.Style()
        style.theme_use('clam')
        
        # ========== MAIN CONTAINER ==========
        main_container = tk.Frame(self.root, bg='#2c3e50')
        main_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        # ========== LEFT SIDE: CONTROLLER ==========
        left_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        tk.Label(left_frame, text="🎮 PS4 CONTROLLER", font=("Arial", 18, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=15)
        
        # Stick values display
        sticks_frame = tk.Frame(left_frame, bg='#2c3e50', relief="sunken", bd=2)
        sticks_frame.pack(padx=15, pady=10, fill="x")
        
        tk.Label(sticks_frame, text="Left Stick (Y):", font=("Arial", 12, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=0, column=0, padx=10, pady=8, sticky="w")
        self.left_stick_var = tk.StringVar(value="0.00")
        tk.Label(sticks_frame, textvariable=self.left_stick_var, font=("Arial", 16, "bold"), 
                bg='#2c3e50', fg='#f39c12', width=8).grid(row=0, column=1, padx=10, pady=8)
        
        tk.Label(sticks_frame, text="Right Stick (Y):", font=("Arial", 12, "bold"), 
                bg='#2c3e50', fg='#3498db').grid(row=1, column=0, padx=10, pady=8, sticky="w")
        self.right_stick_var = tk.StringVar(value="0.00")
        tk.Label(sticks_frame, textvariable=self.right_stick_var, font=("Arial", 16, "bold"), 
                bg='#2c3e50', fg='#f39c12', width=8).grid(row=1, column=1, padx=10, pady=8)
        
        # Button mappings display
        mappings_frame = tk.Frame(left_frame, bg='#34495e')
        mappings_frame.pack(padx=15, pady=10, fill="both", expand=True)
        
        tk.Label(mappings_frame, text="BUTTON MAPPINGS", font=("Arial", 14, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=10)
        
        # Create button mapping displays
        self.button_frames = {}
        mappings = [
            ("🔺 Triangle", "MR207 Motor ON", "btn^"),
            ("⬜ Square", "MR208 Start", "btn[]"),
            ("⭕ Circle", "MR209 Break", "btnO"),
            ("⬆️ D-Pad UP", "MR603/604 Switch", "btn4"),
            ("➡️ D-Pad RIGHT", "MR605 Motor DC", "btn5"),
            ("⬇️ D-Pad DOWN", "MR206 Error Reset", "btn6"),
            ("⬅️ D-Pad LEFT", "MR602 Power Hold", "btn7"),
        ]
        
        for btn_name, function, key in mappings:
            frame = tk.Frame(mappings_frame, bg='#2c3e50', relief="raised", bd=2)
            frame.pack(padx=10, pady=5, fill="x")
            
            tk.Label(frame, text=btn_name, font=("Arial", 11, "bold"), 
                    bg='#2c3e50', fg='#ecf0f1', width=18, anchor="w").pack(side="left", padx=10, pady=8)
            
            tk.Label(frame, text=function, font=("Arial", 10), 
                    bg='#2c3e50', fg='#f39c12', anchor="w").pack(side="left", padx=10, pady=8, fill="x", expand=True)
            
            indicator = tk.Canvas(frame, width=25, height=25, bg='#2c3e50', highlightthickness=0)
            indicator.pack(side="right", padx=10, pady=8)
            circle = indicator.create_oval(3, 3, 22, 22, fill='#7f8c8d', outline='')
            
            self.button_frames[key] = {'frame': frame, 'indicator': indicator, 'circle': circle}
        
        # ========== RIGHT SIDE: PLC VALUES ==========
        right_frame = tk.Frame(main_container, bg='#34495e', relief="raised", bd=3)
        right_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        
        tk.Label(right_frame, text="📊 PLC VALUES", font=("Arial", 18, "bold"), 
                bg='#34495e', fg='#ecf0f1').pack(pady=15)
        
        # PLC values frame
        plc_frame = tk.Frame(right_frame, bg='#34495e')
        plc_frame.pack(padx=10, pady=10, fill="both", expand=True)
        
        # Create PLC value displays
        self.plc_vars = {}
        
        # Only the MR registers used in button mappings
        mr_addresses = ['MR206', 'MR207', 'MR208', 'MR209', 'MR602', 'MR603', 'MR604', 'MR605']
        
        for addr in mr_addresses:
            frame = tk.Frame(plc_frame, bg='#2c3e50', relief="raised", bd=2)
            frame.pack(padx=10, pady=5, fill="x")
            
            tk.Label(frame, text=f"{addr}:", font=("Arial", 12, "bold"), 
                    bg='#2c3e50', fg='#9b59b6', width=10, anchor="w").pack(side="left", padx=15, pady=10)
            
            var = tk.StringVar(value="--")
            self.plc_vars[addr] = var
            self.plc_states[addr] = 0  # Initialize state
            tk.Label(frame, textvariable=var, font=("Arial", 16, "bold"), 
                    bg='#2c3e50', fg='#2ecc71', width=10).pack(side="right", padx=15, pady=10)
        
        # ========== ROS SETUP ==========
        rospy.init_node('combined_monitor_gui', anonymous=True)
        self._rate = rospy.Rate(10)
        self._write_pub = rospy.Publisher('/write_command', String, queue_size=10)
        
        # PS4 Controller subscriber
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # PLC subscribers for read topics
        for addr in mr_addresses:
            rospy.Subscriber(f"/plc_protocol_driver/read/{addr}", Float32, 
                           self._create_plc_callback(addr))

    def joy_callback(self, msg):
        """Callback for PS4 controller data."""
        # Update stick displays (no condition, always show values)
        left_y = msg.axes[1]
        right_x = msg.axes[3]
        
        self.left_stick_var.set(f"{left_y:.2f}")
        self.right_stick_var.set(f"{right_x:.2f}")
        
        
        # Button 2: Triangle - MR207 Motor ON (Toggle based on current PLC state)
        if msg.buttons[2] == 1 and self.last_buttons[2] == 0:
            self._flash_button('btn^')
            current_state = self.plc_states.get('MR207', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR207', new_state)
            rospy.loginfo(f"MR207 toggled: {current_state} -> {new_state}")
        
        # Button 3: Square - MR208 Start (Toggle based on current PLC state)
        if msg.buttons[3] == 1 and self.last_buttons[3] == 0:
            self._flash_button('btn[]')
            current_state = self.plc_states.get('MR208', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR208', new_state)
            rospy.loginfo(f"MR208 toggled: {current_state} -> {new_state}")
        
        # Button 1: Circle - MR209 Break (Toggle based on current PLC state)
        if msg.buttons[1] == 1 and self.last_buttons[1] == 0:
            self._flash_button('btnO')
            current_state = self.plc_states.get('MR209', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR209', new_state)
            rospy.loginfo(f"MR209 toggled: {current_state} -> {new_state}")
        
        # D-Pad UP: Axes 7 = 1 - MR603/604 Switch (based on MR603 state)
        if msg.axes[7] > 0.5 and self.last_axes[7] <= 0.5:
            self._flash_button('btn4')
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
            self._flash_button('btn5')
            rospy.loginfo(f"D-Pad RIGHT pressed! axes[6]={msg.axes[6]}")
            current_state = self.plc_states.get('MR605', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR605', new_state)
            rospy.loginfo(f"D-Pad RIGHT: MR605 toggled: {current_state} -> {new_state}")
        
        # D-Pad DOWN: Axes 7 = -1 - MR206 Error Reset (Toggle based on current PLC state)
        if msg.axes[7] < -0.5 and self.last_axes[7] >= -0.5:
            self._flash_button('btn6')
            rospy.loginfo(f"D-Pad DOWN pressed! axes[7]={msg.axes[7]}")
            current_state = self.plc_states.get('MR206', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR206', new_state)
            rospy.loginfo(f"D-Pad DOWN: MR206 toggled: {current_state} -> {new_state}")
        
        # D-Pad LEFT: Axes 6 = 1 - MR602 Power Holding (Toggle based on current PLC state)
        if msg.axes[6] > 0.5 and self.last_axes[6] <= 0.5:
            self._flash_button('btn7')
            rospy.loginfo(f"D-Pad LEFT pressed! axes[6]={msg.axes[6]}")
            current_state = self.plc_states.get('MR602', 0)
            new_state = 0 if current_state == 1 else 1
            self._publish_write_command('MR602', new_state)
            rospy.loginfo(f"D-Pad LEFT: MR602 toggled: {current_state} -> {new_state}")
        
        
        # Update last button and axes states
        self.last_buttons = list(msg.buttons)
        self.last_axes = list(msg.axes)

    def _flash_button(self, button_key):
        """Flash the button indicator green when pressed."""
        if button_key in self.button_frames:
            indicator = self.button_frames[button_key]['indicator']
            circle = self.button_frames[button_key]['circle']
            
            # Turn green
            indicator.itemconfig(circle, fill='#2ecc71')
            
            # Turn back to gray after 200ms
            self.root.after(200, lambda: indicator.itemconfig(circle, fill='#7f8c8d'))
            
    

    def _create_plc_callback(self, address):
        """Creates a callback function to update PLC values and store current state."""
        def callback(msg):
            # Store the actual PLC state
            self.plc_states[address] = msg.data
            
            # Update display
            value = f"{msg.data:.3f}"
            if msg.data == -1.0:
                value = "ERROR"
            self.root.after(0, lambda: self.plc_vars[address].set(value))
        return callback

    def _publish_write_command(self, address, data):
        """Publishes a write command to the ROS topic."""
        msg = f"{address},{data}"
        rospy.loginfo(f"[GUI] Publishing write command: {msg} to topic /write_command")
        self._write_pub.publish(msg)
        rospy.loginfo(f"[GUI] Command published successfully")

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
