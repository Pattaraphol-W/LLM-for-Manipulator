#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
import time

class JointControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Joint Controller")

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # --- STATE VARIABLES ---
        self.joint_positions = [0.0] * len(self.joint_names)
        self.current_joint_index = 0
        self.last_button_press_time = 0
        self.debounce_duration = 0.2 #prevent multiple presses

        # --- ROS PUBLISHERS & SUBSCRIBERS ---
        self.command_pub = rospy.Publisher(
            "/joint_trajectory_controller/command",
            JointTrajectory, 
            queue_size=10
        )
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # --- GUI SETUP ---
        self.labels = {}
        main_frame = tk.Frame(self.root, padx=10, pady=10)
        main_frame.pack()

        for i, name in enumerate(self.joint_names):
            tk.Label(main_frame, text=name, font=("Arial", 12, "bold")).grid(row=i, column=0, sticky="w", padx=5, pady=2)
            self.labels[name] = tk.Label(main_frame, text=f"{self.joint_positions[i]:.4f}", font=("Arial", 12))
            self.labels[name].grid(row=i, column=1, sticky="e", padx=5, pady=2)

        self.selected_label = tk.Label(self.root, text="", font=("Arial", 14, "bold"), fg="blue")
        self.selected_label.pack(pady=10)
        
        rospy.loginfo("Joint Controller GUI started.")
        rospy.loginfo("Triangle/Cross to switch joint, Circle/Square to move joint.")

    def joint_states_callback(self, msg):
        """Updates our internal list of joint positions from the /joint_states topic."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]

    def joy_callback(self, msg):
        """Handles input from the PS4 controller."""
        # PS4 button mapping
        button_cross = msg.buttons[0]
        button_circle = msg.buttons[1]
        button_triangle = msg.buttons[2]
        button_square = msg.buttons[3]
        
        current_time = time.time()
        
        # Debounce button presses for switching joints
        if current_time - self.last_button_press_time > self.debounce_duration:
            # Switch selected joint
            if button_triangle == 1: # v joint
                self.current_joint_index = (self.current_joint_index - 1) % len(self.joint_names)
                self.last_button_press_time = current_time
            elif button_cross == 1: # ^ joint
                self.current_joint_index = (self.current_joint_index + 1) % len(self.joint_names)
                self.last_button_press_time = current_time

        # Move selected joint
        step = 0.010 # Amount to move the joint per message
        moved = False
        if button_circle == 1: # Increase angle
            self.joint_positions[self.current_joint_index] += step
            moved = True
        elif button_square == 1: # Decrease angle
            self.joint_positions[self.current_joint_index] -= step
            moved = True
            
        if moved:
            self.send_command()

    def send_command(self):
        """Creates and publishes a JointTrajectory message to move the robot."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(0.1) # Move to the position in 0.1 seconds
        
        msg.points.append(point)
        self.command_pub.publish(msg)

    def update_gui(self):
        """Periodically refreshes the GUI with the latest data."""
        for i, name in enumerate(self.joint_names):
            self.labels[name].config(text=f"{self.joint_positions[i]:.4f}")
        
        selected_joint_name = self.joint_names[self.current_joint_index]
        self.selected_label.config(text=f"Selected: {selected_joint_name}")
        
        # Highlight the selected joint
        for name, label in self.labels.items():
            if name == selected_joint_name:
                label.config(fg="blue")
            else:
                label.config(fg="black")
        
        # Reschedule this function to run again
        self.root.after(100, self.update_gui)

def main():
    rospy.init_node("joint_controller_gui")
    root = tk.Tk()
    gui = JointControllerGUI(root)

    # Start the periodic GUI update
    gui.update_gui()
    
    # Start the Tkinter main event loop.
    root.mainloop()

if __name__ == "__main__":
    main()

