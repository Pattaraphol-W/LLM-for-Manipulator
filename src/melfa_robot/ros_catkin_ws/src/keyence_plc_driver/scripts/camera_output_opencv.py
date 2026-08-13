#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

#sudo apt-get install python3-opencv

class Viewer:
    def __init__(self):
        rospy.init_node("opencv_camera_viewer", anonymous=True)
        
        self.sub = rospy.Subscriber("/udp/compressed",
                                     CompressedImage,
                                     self.callback,
                                     queue_size=1)
        
        cv2.namedWindow("Camera Output", cv2.WINDOW_NORMAL)
        print("Waiting for images on /udp/compressed...")
        
    def callback(self, msg):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is not None:
                cv2.imshow("Camera Output", img)
                cv2.waitKey(1)
            else:
                print("Failed to decode image")
                
        except Exception as e:
            print(f"Error: {e}")
    
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    viewer = Viewer()
    viewer.run()
