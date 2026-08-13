#!/usr/bin/env python3
"""
Tkinter viewer optimized for Docker environments
Requires: apt-get install python3-tk python3-opencv python3-pil python3-pil.imagetk
"""
import os
import sys

# Verify display is available
if not os.environ.get('DISPLAY'):
    print("ERROR: DISPLAY not set. Run container with:")
    print("  docker run --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ...")
    sys.exit(1)

import tkinter as tk
from PIL import Image, ImageTk
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

class DockerViewer:
    def __init__(self):
        print("Initializing viewer for Docker...")
        
        # Create window
        self.root = tk.Tk()
        self.root.title("ROS Camera (Docker)")
        self.root.geometry("800x600")
        
        # Canvas for image
        self.canvas = tk.Canvas(self.root, bg="black")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Status bar
        self.status = tk.Label(self.root, text="Starting ROS...", 
                              bg="lightgray", anchor=tk.W)
        self.status.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.latest_data = None
        self.photo = None
        self.frame_count = 0
        
        # Init ROS after window is ready
        self.root.after(100, self.init_ros)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
    def init_ros(self):
        try:
            print("Initializing ROS node...")
            rospy.init_node("docker_tk_viewer", disable_signals=True)
            
            print("Subscribing to /udp/compressed...")
            self.sub = rospy.Subscriber("/udp/compressed",
                                       CompressedImage,
                                       self.callback,
                                       queue_size=1,
                                       buff_size=2**24)
            
            self.status.config(text="Ready - waiting for images...")
            print("Ready!")
            
            # Start update loop
            self.update()
            
        except Exception as e:
            print(f"ROS init error: {e}")
            self.status.config(text=f"Error: {e}")
    
    def callback(self, msg):
        self.latest_data = bytes(msg.data)
        self.frame_count += 1
    
    def update(self):
        try:
            if self.latest_data:
                data = self.latest_data
                self.latest_data = None
                
                # Decode
                np_arr = np.frombuffer(data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if cv_img is not None:
                    # BGR to RGB
                    rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                    
                    # Resize to canvas
                    cw = self.canvas.winfo_width()
                    ch = self.canvas.winfo_height()
                    
                    if cw > 1 and ch > 1:
                        h, w = rgb.shape[:2]
                        ratio = min(cw/w, ch/h)
                        new_size = (int(w*ratio), int(h*ratio))
                        rgb = cv2.resize(rgb, new_size, interpolation=cv2.INTER_AREA)
                    
                    # Convert to PhotoImage
                    pil_img = Image.fromarray(rgb)
                    self.photo = ImageTk.PhotoImage(pil_img)
                    
                    # Display
                    self.canvas.delete("all")
                    self.canvas.create_image(cw//2, ch//2, 
                                           image=self.photo, anchor=tk.CENTER)
                    
                    h, w = rgb.shape[:2]
                    self.status.config(text=f"Frame {self.frame_count} | {w}x{h}")
                    
        except Exception as e:
            print(f"Display error: {e}")
        
        # Continue updating
        if not rospy.is_shutdown():
            self.root.after(30, self.update)
    
    def on_close(self):
        rospy.signal_shutdown("Window closed")
        self.root.destroy()
    
    def run(self):
        print("Starting Tkinter mainloop...")
        self.root.mainloop()

if __name__ == "__main__":
    print("=" * 60)
    print("ROS Camera Viewer for Docker")
    print("=" * 60)
    print(f"DISPLAY = {os.environ.get('DISPLAY', 'NOT SET')}")
    print(f"Python = {sys.version}")
    
    try:
        viewer = DockerViewer()
        viewer.run()
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
