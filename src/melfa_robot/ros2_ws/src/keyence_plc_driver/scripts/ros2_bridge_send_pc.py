#!/usr/bin/env python3
"""
ROS2 Bridge Sender (My PC)
==========================
Sends ROS2 /joint_trajectory_controller/command from My PC (ROS2) to Docker (ROS1).
  - ROS2 /joint_trajectory_controller/command  ->  Docker (ROS1) Port 9004
Serializes into ROS1 binary wire format so the ROS1 receiver can deserialize directly.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory as JointTrajectoryRos2
import socket
import struct
import io


def serialize_ros1_joint_trajectory(msg: JointTrajectoryRos2) -> bytes:
    """
    Serialize ROS2 JointTrajectory into ROS1 binary wire format.
    ROS1 JointTrajectory layout:
        Header
            uint32 seq
            time   stamp (int32 secs + int32 nsecs)
            string frame_id
        string[]  joint_names
        JointTrajectoryPoint[] points
            each point:
                float64[] positions
                float64[] velocities
                float64[] accelerations
                float64[] effort
                duration  time_from_start (int32 secs + int32 nsecs)
    """
    buf = io.BytesIO()

    # --- Header ---
    buf.write(struct.pack('<I', 0))   # seq (unused, set to 0)
    buf.write(struct.pack('<i', 0))   # stamp secs
    buf.write(struct.pack('<i', 0))   # stamp nsecs
    frame_id = msg.header.frame_id.encode('utf-8')
    buf.write(struct.pack('<I', len(frame_id)))
    buf.write(frame_id)

    # --- joint_names (string[]) ---
    buf.write(struct.pack('<I', len(msg.joint_names)))
    for name in msg.joint_names:
        name_bytes = name.encode('utf-8')
        buf.write(struct.pack('<I', len(name_bytes)))
        buf.write(name_bytes)

    # --- points (JointTrajectoryPoint[]) ---
    buf.write(struct.pack('<I', len(msg.points)))
    for point in msg.points:

        # positions (float64[])
        buf.write(struct.pack('<I', len(point.positions)))
        for v in point.positions:
            buf.write(struct.pack('<d', v))

        # velocities (float64[])
        buf.write(struct.pack('<I', len(point.velocities)))
        for v in point.velocities:
            buf.write(struct.pack('<d', v))

        # accelerations (float64[])
        buf.write(struct.pack('<I', len(point.accelerations)))
        for v in point.accelerations:
            buf.write(struct.pack('<d', v))

        # effort (float64[])  <-- correct field name
        buf.write(struct.pack('<I', len(point.effort)))
        for v in point.effort:
            buf.write(struct.pack('<d', v))

        # time_from_start (duration: int32 secs + int32 nsecs)
        buf.write(struct.pack('<i', point.time_from_start.sec))
        buf.write(struct.pack('<i', point.time_from_start.nanosec))

    return buf.getvalue()


class JointTrajectorySender(Node):
    def __init__(self, target_ip, target_port):
        super().__init__("ros2_joint_trajectory_sender")
        self.target = (target_ip, target_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.subscription = self.create_subscription(
            JointTrajectoryRos2,
            "/joint_trajectory_controller/command",
            self.joint_trajectory_callback,
            10
        )
        self.get_logger().info(f"Sending JointTrajectory to {self.target}")

    def joint_trajectory_callback(self, msg: JointTrajectoryRos2):
        try:
            serialized_msg = serialize_ros1_joint_trajectory(msg)
            self.sock.sendto(serialized_msg, self.target)
            self.get_logger().debug(
                f"[JOINT_TRAJECTORY] Sent {len(serialized_msg)} bytes to {self.target}"
            )
        except Exception as e:
            self.get_logger().error(f"Error sending JointTrajectory: {e}")


def main(args=None):
    rclpy.init(args=args)
    sender = JointTrajectorySender("192.168.1.3", 9004)
    try:
        rclpy.spin(sender)
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
