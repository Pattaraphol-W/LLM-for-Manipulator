#!/usr/bin/env python
"""
ROS1 Bridge Sender (Docker)
===========================
Sends ROS1 /joint_states from a Docker container (ROS1) to My PC (ROS2).
  - ROS1 /joint_states  ->  My PC (ROS2) /joint_states (Port 9003)
This script runs inside the Docker container (ROS1).
It forwards raw message bytes as-is over UDP without any parsing or conversion.
"""
import rospy
import socket


def joint_state_callback(ros1_msg, args):
    sock, target_address = args

    # ros1_msg._buff contains raw bytes when using AnyMsg
    serialized_msg = ros1_msg._buff

    try:
        sock.sendto(serialized_msg, target_address)
        rospy.loginfo("[JOINT_STATES] Sent %d bytes to %s", len(serialized_msg), str(target_address))
    except Exception as e:
        rospy.logerr("Error sending JointState: %s", str(e))


def main():
    rospy.init_node("ros1_joint_state_sender")

    target_ip = "192.168.1.3"
    target_port = 9003
    target_address = (target_ip, target_port)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # AnyMsg captures the raw serialized bytes without deserializing
    rospy.Subscriber(
        "/joint_states",
        rospy.AnyMsg,
        joint_state_callback,
        (sock, target_address)
    )

    rospy.loginfo("ROS1 JointState sender started. Sending to %s:%d", target_ip, target_port)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
