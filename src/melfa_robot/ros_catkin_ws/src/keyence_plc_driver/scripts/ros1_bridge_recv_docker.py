#!/usr/bin/env python3
"""
ROS1 UDP Bridge Receiver
========================
Receives a UDP stream from a remote sender and publishes it to:
  - Port 9004  ->  /joint_trajectory_controller/command  (JointTrajectory, no decode)

Target IP : 192.168.1.2  (Docker container on the same host, different port)
"""

import rospy
import socket
import struct
from trajectory_msgs.msg import JointTrajectory


def main():
    rospy.init_node("ros1_joint_trajectory_receiver")

    pub = rospy.Publisher(
        "/joint_trajectory_controller/command",
        JointTrajectory,
        queue_size=10
    )

    # Bind to the host IP on a different port from the ROS2 receiver
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("192.168.1.3", 9004))
    rospy.loginfo("Listening on 192.168.1.3:9004  ->  /joint_trajectory_controller/command")

    while not rospy.is_shutdown():
        try:
            # Receive the raw serialized JointTrajectory packet
            packet, addr = sock.recvfrom(4096)  # Increase if trajectories are large

            # Deserialize using ROS1 built-in deserialization
            msg = JointTrajectory()
            msg.deserialize(packet)

            # Publish directly - no additional decoding
            pub.publish(msg)

        except Exception as e:
            rospy.logerr(f"Error receiving or publishing packet: {e}")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
