#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import tkinter as tk
import math

class ControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PS4 Controller Monitor")

        # Create StringVars to hold the text for our labels
        self.left_stick_var = tk.StringVar(value="Left Stick (Y): (0.00)")        
        self.right_stick_var = tk.StringVar(value="Right Stick (Y): (0.00)")
        
        self.triggers3_var = tk.StringVar(value="L3/R3 Triggers: (0.00, 0.00)")
        self.triggers2_var = tk.StringVar(value="L2/R2 Triggers: (0.00, 0.00)")
        self.buttons_var = tk.StringVar(value="Buttons (X, O, ^, []): (0, 0, 0, 0)")

        # Create labels and arrange them in the window
        tk.Label(self.root, textvariable=self.left_stick_var, font=("Courier", 14)).pack(anchor="w", padx=10, pady=5)
        tk.Label(self.root, textvariable=self.right_stick_var, font=("Courier", 14)).pack(anchor="w", padx=10, pady=5)
        
        tk.Label(self.root, textvariable=self.triggers3_var, font=("Courier", 14)).pack(anchor="w", padx=10, pady=5)
        tk.Label(self.root, textvariable=self.triggers2_var, font=("Courier", 14)).pack(anchor="w", padx=10, pady=5)
        tk.Label(self.root, textvariable=self.buttons_var, font=("Courier", 14)).pack(anchor="w", padx=10, pady=5)

        # Initialize the ROS node
        rospy.init_node('ps4_reader', anonymous=True)

        # Subscribe to the /joy topic
        rospy.Subscriber('/joy', Joy, self.joy_callback)

    def joy_callback(self, msg):
        """This function is called every time a message is received from the /joy topic."""

        left_trig = msg.buttons[11]
        
        if left_trig == 0:
            left_y = 0
        else:
            left_y = msg.axes[1]
        
        right_trig = msg.buttons[12]
        
        if right_trig == 0:
            right_y = 0
        else:
            right_y = msg.axes[4]

        self.left_stick_var.set(f"Left Stick (Y): ({left_y:.2f})")       
        self.right_stick_var.set(f"Right Stick (Y): ({right_y:.2f})")
        
        self.triggers3_var.set(f"L3/R3 Triggers:     ({left_trig:.2f}, {right_trig:.2f})")
        self.triggers2_var.set(f"L2/R2 Triggers:     ({msg.axes[2]:.2f}, {msg.axes[5]:.2f})")
        self.buttons_var.set(f"Buttons (X, O, ^, []): ({msg.buttons[0]}, {msg.buttons[1]}, {msg.buttons[2]}, {msg.buttons[3]})")

def main():
    # Create the main Tkinter window
    root = tk.Tk()
    # Create an instance of our GUI class
    app = ControllerGUI(root)
    # Start the Tkinter event loop (this keeps the window open)
    root.mainloop()

if __name__ == '__main__':
    main()

