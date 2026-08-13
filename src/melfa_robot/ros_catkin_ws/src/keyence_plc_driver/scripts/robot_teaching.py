#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
import os
import threading
import time

class RobotTeachingPendant:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Teaching Pendant")
        self.root.geometry("900x700")
        self.root.configure(bg='#2c3e50')
        
        # Robot state
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.current_joint_positions = [0.0] * 6
        self.taught_positions = []
        self.is_playing = False
        self.is_recording = False
        self.current_filename = None
        
        # Playback settings
        self.playback_speed = 1.0
        self.loop_playback = False
        
        self.setup_gui()
        self.setup_ros()
        
        rospy.loginfo("Teaching Pendant started")
    
    def setup_gui(self):
        # ========== TOP: Current Position Display ==========
        top_frame = tk.Frame(self.root, bg='#34495e', relief="raised", bd=3)
        top_frame.pack(fill="x", padx=10, pady=10)
        
        tk.Label(top_frame, text="CURRENT ROBOT POSITION", 
                font=("Helvetica", 14), bg='#34495e', fg='#ecf0f1').pack(pady=5)
        
        position_frame = tk.Frame(top_frame, bg='#2c3e50')
        position_frame.pack(fill="x", padx=10, pady=5)
        
        self.position_labels = {}
        for i, name in enumerate(self.joint_names):
            row = tk.Frame(position_frame, bg='#2c3e50')
            row.pack(fill='x', pady=2)
            
            tk.Label(row, text=name, font=("Courier", 10), 
                    width=8, anchor='w', bg='#2c3e50', fg='#3498db').pack(side=tk.LEFT, padx=5)
            
            self.position_labels[name] = tk.Label(row, text="0.000", 
                                                  font=("Courier", 10), width=10, 
                                                  anchor='e', bg='#2c3e50', fg='#f39c12')
            self.position_labels[name].pack(side=tk.LEFT)
        
        # ========== MIDDLE: Teaching Controls ==========
        middle_frame = tk.Frame(self.root, bg='#34495e', relief="raised", bd=3)
        middle_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Teach button
        teach_btn_frame = tk.Frame(middle_frame, bg='#34495e')
        teach_btn_frame.pack(pady=10)
        
        self.teach_button = tk.Button(teach_btn_frame, text="TEACH POSITION", 
                                      font=("Helvetica", 12), bg='#27ae60', 
                                      fg='white', width=20, height=2,
                                      command=self.teach_current_position)
        self.teach_button.pack(side=tk.LEFT, padx=5)
        
        self.record_button = tk.Button(teach_btn_frame, text="START RECORDING", 
                                       font=("Helvetica", 12), bg='#e74c3c', 
                                       fg='white', width=20, height=2,
                                       command=self.toggle_recording)
        self.record_button.pack(side=tk.LEFT, padx=5)
        
        # Position list
        list_frame = tk.LabelFrame(middle_frame, text="Taught Positions", 
                                   bg='#2c3e50', fg='#ecf0f1', font=("Helvetica", 11))
        list_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Scrollbar for listbox
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.position_listbox = tk.Listbox(list_frame, font=("Courier", 9), 
                                          bg='#34495e', fg='#ecf0f1',
                                          selectmode=tk.SINGLE,
                                          yscrollcommand=scrollbar.set)
        self.position_listbox.pack(fill="both", expand=True, padx=5, pady=5)
        scrollbar.config(command=self.position_listbox.yview)
        
        # Position management buttons
        btn_frame = tk.Frame(middle_frame, bg='#34495e')
        btn_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Button(btn_frame, text="Move Up", font=("Helvetica", 9), 
                 bg='#3498db', fg='white', width=12,
                 command=self.move_position_up).pack(side=tk.LEFT, padx=2)
        
        tk.Button(btn_frame, text="Move Down", font=("Helvetica", 9), 
                 bg='#3498db', fg='white', width=12,
                 command=self.move_position_down).pack(side=tk.LEFT, padx=2)
        
        tk.Button(btn_frame, text="Delete", font=("Helvetica", 9), 
                 bg='#e74c3c', fg='white', width=12,
                 command=self.delete_position).pack(side=tk.LEFT, padx=2)
        
        tk.Button(btn_frame, text="Clear All", font=("Helvetica", 9), 
                 bg='#c0392b', fg='white', width=12,
                 command=self.clear_all_positions).pack(side=tk.LEFT, padx=2)
        
        # ========== BOTTOM: File & Playback Controls ==========
        bottom_frame = tk.Frame(self.root, bg='#34495e', relief="raised", bd=3)
        bottom_frame.pack(fill="x", padx=10, pady=10)
        
        # File controls
        file_frame = tk.LabelFrame(bottom_frame, text="File Operations", 
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
        playback_frame = tk.LabelFrame(bottom_frame, text="Playback Control", 
                                      bg='#2c3e50', fg='#ecf0f1', font=("Helvetica", 10))
        playback_frame.pack(fill="x", padx=10, pady=5)
        
        # Speed control
        speed_row = tk.Frame(playback_frame, bg='#2c3e50')
        speed_row.pack(pady=5)
        
        tk.Label(speed_row, text="Speed:", font=("Helvetica", 9), 
                bg='#2c3e50', fg='#ecf0f1').pack(side=tk.LEFT, padx=5)
        
        self.speed_slider = tk.Scale(speed_row, from_=0.1, to=2.0, resolution=0.1,
                                     orient=tk.HORIZONTAL, length=200,
                                     bg='#34495e', fg='#ecf0f1', troughcolor='#7f8c8d',
                                     command=self.update_speed)
        self.speed_slider.set(1.0)
        self.speed_slider.pack(side=tk.LEFT, padx=5)
        
        self.speed_label = tk.Label(speed_row, text="1.0x", font=("Helvetica", 9), 
                                    bg='#2c3e50', fg='#f39c12', width=6)
        self.speed_label.pack(side=tk.LEFT, padx=5)
        
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
        self.status_label = tk.Label(bottom_frame, text="Ready", 
                                     font=("Helvetica", 10), bg='#2c3e50', 
                                     fg='#2ecc71', relief=tk.SUNKEN, anchor='w')
        self.status_label.pack(fill="x", padx=10, pady=5)
    
    def setup_ros(self):
        self._rate = rospy.Rate(10)
        
        # Publisher for robot commands
        self.robot_command_pub = rospy.Publisher(
            "/joint_trajectory_controller/command", 
            JointTrajectory, 
            queue_size=10
        )
        
        # Subscriber for joint states
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
    
    def joint_states_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[idx]
    
    def teach_current_position(self):
        position = {
            'joints': self.current_joint_positions.copy(),
            'time': 2.0  # Default time to reach this position
        }
        self.taught_positions.append(position)
        self.update_position_listbox()
        self.update_status(f"Position {len(self.taught_positions)} taught", '#2ecc71')
    
    def toggle_recording(self):
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
        rate = rospy.Rate(1)  # Record 1 position per second
        while self.is_recording and not rospy.is_shutdown():
            self.teach_current_position()
            rate.sleep()
    
    def update_position_listbox(self):
        self.position_listbox.delete(0, tk.END)
        for i, pos in enumerate(self.taught_positions):
            joints_str = ", ".join([f"{j:.3f}" for j in pos['joints']])
            self.position_listbox.insert(tk.END, f"#{i+1:02d} | {joints_str} | t={pos['time']:.1f}s")
    
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
                    'positions': self.taught_positions
                }
                
                with open(filename, 'w') as f:
                    json.dump(data, f, indent=2)
                
                self.current_filename = filename
                self.filename_label.config(text=f"File: {os.path.basename(filename)}")
                self.update_status(f"Program saved: {os.path.basename(filename)}", '#27ae60')
                messagebox.showinfo("Success", f"Program saved successfully!\n{len(self.taught_positions)} positions")
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
                messagebox.showinfo("Success", f"Program loaded successfully!\n{len(self.taught_positions)} positions")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load: {str(e)}")
    
    def update_speed(self, value):
        self.playback_speed = float(value)
        self.speed_label.config(text=f"{self.playback_speed:.1f}x")
    
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
            while self.is_playing and not rospy.is_shutdown():
                for i, position in enumerate(self.taught_positions):
                    if not self.is_playing:
                        break
                    
                    self.update_status(f"Playing position {i+1}/{len(self.taught_positions)}", '#3498db')
                    
                    # Send command to robot
                    msg = JointTrajectory()
                    msg.joint_names = self.joint_names
                    
                    point = JointTrajectoryPoint()
                    point.positions = position['joints']
                    point.time_from_start = rospy.Duration(position['time'] / self.playback_speed)
                    
                    msg.points.append(point)
                    self.robot_command_pub.publish(msg)
                    
                    # Wait for movement to complete
                    time.sleep(position['time'] / self.playback_speed)
                
                if not self.loop_playback:
                    break
            
            if self.is_playing:
                self.update_status("Playback completed", '#27ae60')
        except Exception as e:
            rospy.logerr(f"Playback error: {e}")
            self.update_status(f"Playback error: {e}", '#e74c3c')
        finally:
            self.root.after(100, self.stop_playback)
    
    def update_status(self, text, color='#ecf0f1'):
        self.status_label.config(text=text, fg=color)
    
    def update_gui(self):
        # Update position displays
        for i, name in enumerate(self.joint_names):
            self.position_labels[name].config(
                text=f"{self.current_joint_positions[i]:.3f}"
            )
        
        self.root.after(50, self.update_gui)
    
    def spin(self):
        def check_ros():
            if rospy.is_shutdown():
                self.root.destroy()
            else:
                self.root.after(100, check_ros)
        
        self.root.after(100, check_ros)
        self.root.mainloop()


def main():
    rospy.init_node('robot_teaching_pendant', anonymous=True)
    
    root = tk.Tk()
    pendant = RobotTeachingPendant(root)
    pendant.update_gui()
    pendant.spin()


if __name__ == '__main__':
    main()
