#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

#sudo apt-get install python3-opencv

class Viewer(Node):
    def __init__(self):
        super().__init__('opencv_camera_viewer')
        
        self.sub = self.create_subscription(
            CompressedImage,
            '/udp/compressed',
            self.callback,
            1
        )
        
        cv2.namedWindow("Camera Output", cv2.WINDOW_NORMAL)
        self.get_logger().info("Waiting for images on /udp/compressed...")
        
    def callback(self, msg):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is not None:
                cv2.imshow("Camera Output", img)
                cv2.waitKey(1)
            else:
                self.get_logger().error("Failed to decode image")
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
    
    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")
        finally:
            cv2.destroyAllWindows()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    viewer = Viewer()
    viewer.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
