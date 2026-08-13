#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
import time

class JointControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Joint Controller GUI")

        # Joint setup (must match your robot)
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.joint_positions = [0.0] * len(self.joint_names)
        self.current_joint = 0

        # ROS publisher
        self.pub = rospy.Publisher("/joint_trajectory_controller/command",
                                   JointTrajectory, queue_size=10)

        # ROS subscribers
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Tkinter labels for joint positions
        self.labels = {}
        for i, name in enumerate(self.joint_names):
            tk.Label(root, text=name, font=("Arial", 12, "bold")).grid(row=i, column=0, padx=10, pady=5)
            self.labels[name] = tk.Label(root, text=f"{self.joint_positions[i]:.4f}", font=("Arial", 12))
            self.labels[name].grid(row=i, column=1, padx=10, pady=5)

        self.selected_label = tk.Label(root, text=f"Selected Joint: {self.joint_names[self.current_joint]}",
                                       font=("Arial", 14, "bold"), fg="blue")
        self.selected_label.grid(row=len(self.joint_names), column=0, columnspan=2, pady=10)

        # Periodically update GUI
        self.root.after(100, self.update_gui)

        rospy.loginfo("Joint Controller GUI started. Triangle/Cross to switch joint, Circle/Square to move slowly.")

    def joint_states_callback(self, msg):
        # Update joint positions from /joint_states
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]

    def joy_callback(self, msg):
        # PS4 button mapping (check with `rostopic echo /joy`)
        button_triangle = msg.buttons[4]  # ^
        button_cross    = msg.buttons[5]  # X
        button_circle   = msg.buttons[6]  # O
        button_square   = msg.buttons[7]  # []

        step = 0.001  # very slow increment

        # Switch selected joint
        if button_triangle:
            self.current_joint = (self.current_joint + 1) % len(self.joint_names)
            sleep(10)
        elif button_cross:
            self.current_joint = (self.current_joint - 1) % len(self.joint_names)
            sleep(10)

        # Move selected joint slowly
        updated = False
        if button_circle:
            self.joint_positions[self.current_joint] += step
            updated = True
        elif button_square:
            self.joint_positions[self.current_joint] -= step
            updated = True

        if updated:
            self.send_command()
            sleep(5)  # slow down high-frequency callbacks

    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = rospy.Duration(0.5)
        msg.points.append(point)
        self.pub.publish(msg)

    def update_gui(self):
        # Refresh GUI with current joint positions
        for i, name in enumerate(self.joint_names):
            self.labels[name].config(text=f"{self.joint_positions[i]:.4f}")
        self.selected_label.config(text=f"Selected Joint: {self.joint_names[self.current_joint]}")
        self.root.after(100, self.update_gui)

def main():
    rospy.init_node("joint_controller_gui", anonymous=True)
    root = tk.Tk()
    gui = JointControllerGUI(root)

    def tk_update():
        if not rospy.is_shutdown():
            root.update()
            root.after(50, tk_update)
    root.after(50, tk_update)
    root.mainloop()

if __name__ == "__main__":
    main()

