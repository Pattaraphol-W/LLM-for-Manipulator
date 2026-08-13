#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, String
import tkinter as tk

class PLCMonitorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Keyence PLC Monitor")

        # --- Create StringVars to hold the dynamic data ---
        self.r_vars = {}
        read_addresses = ['R000', 'R001', 'R008', 'R009', 'R010', 'R011', 'R014',
                          'R207', 'R208', 'R209', 'R210', 'R211', 'R214']
        for addr in read_addresses:
            self.r_vars[addr] = tk.StringVar(value=f"{addr} Status: --")

        self.mr_vars = {}
        mr_read_addresses = ['MR001', 'MR012', 'MR013', 'MR400', 'MR401', 'MR402', 'MR403', 'MR404', 'MR405', 
                             'MR602', 'MR603', 'MR604', 'MR605']
        for addr in mr_read_addresses:
            self.mr_vars[addr] = tk.StringVar(value=f"{addr} Value: --")

        # --- GUI Layout for Read Values ---
        read_frame = tk.Frame(self.root, borderwidth=2, relief="groove")
        read_frame.pack(padx=10, pady=10, fill="x")
        tk.Label(read_frame, text="PLC Read Values", font=("Courier", 16, "bold")).pack(pady=5)

        for addr in read_addresses:
            tk.Label(read_frame, textvariable=self.r_vars[addr], font=("Courier", 14)).pack(anchor="w", padx=10, pady=2)
        for addr in mr_read_addresses:
            tk.Label(read_frame, textvariable=self.mr_vars[addr], font=("Courier", 14)).pack(anchor="w", padx=10, pady=2)

        # --- GUI Layout for Write Switches ---
        write_frame = tk.Frame(self.root, borderwidth=2, relief="groove")
        write_frame.pack(padx=10, pady=10, fill="x")
        tk.Label(write_frame, text="PLC Write Switches", font=("Courier", 16, "bold")).pack(pady=5)

        self._write_pub = rospy.Publisher('/write_command', String, queue_size=10)
        write_addresses = ['R206', 'MR207', 'MR208', 'MR209', 'MR210', 'MR211', 'MR214', 
                           'MR400', 'MR401', 'MR402', 'MR403', 'MR404', 'MR405',        
                           'MR602', 'MR603', 'MR604', 'MR605']
        
        for addr in write_addresses:
            button_frame = tk.Frame(write_frame)
            button_frame.pack(fill="x", pady=2)
            tk.Label(button_frame, text=f"{addr}:", font=("Courier", 14)).pack(side="left", padx=(10, 5))
            tk.Button(button_frame, text="ON(1)", command=lambda a=addr: self._publish_write_command(a, 1)).pack(side="left", padx=5)
            tk.Button(button_frame, text="OFF(0)", command=lambda a=addr: self._publish_write_command(a, 0)).pack(side="left", padx=5)

        # --- ROS Setup ---
        rospy.init_node('plc_monitor_gui', anonymous=True)
        self._rate = rospy.Rate(10)

        # Subscribers for read topics
        for addr in read_addresses:
            rospy.Subscriber(f"/plc_protocol_driver/read/{addr}", Float32, self._create_callback(self.r_vars[addr]))
        for addr in mr_read_addresses:
            rospy.Subscriber(f"/plc_protocol_driver/read/{addr}", Float32, self._create_callback(self.mr_vars[addr], is_mr=True))

    def _create_callback(self, tk_var, is_mr=False):
        """Creates a callback function to update a specific StringVar."""
        def callback(msg):
            value = f"{int(msg.data)}" if not is_mr else f"{msg.data:.3f}"
            if msg.data == -1.0:
                value = "ERROR"
            self.root.after(0, lambda: tk_var.set(f"{tk_var.get().split(':')[0]}: {value}"))
        return callback

    def _publish_write_command(self, address, data):
        """Publishes a write command to the ROS topic."""
        msg = f"{address},{data}"
        rospy.loginfo(f"Publishing write command: {msg}")
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
    app = PLCMonitorGUI(root)
    app.spin()

if __name__ == '__main__':
    main()

