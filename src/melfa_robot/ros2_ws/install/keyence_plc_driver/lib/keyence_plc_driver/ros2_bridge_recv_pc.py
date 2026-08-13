#!/usr/bin/env python3
"""
ROS2 UDP Bridge Receiver
========================
Receives two UDP streams from a remote PC (192.168.1.2):
  - Port 9002  ->  /udp/compressed   (CompressedImage, with JPEG decode via OpenCV)
  - Port 9003  ->  /joint_states     (JointState, parsed from plain text)

Requirements:
  pip install opencv-python
"""

import rclpy
from rclpy.node import Node
import socket
import struct
import threading
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, JointState
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Frame reassembly buffer (shared across threads, protected by a lock)
frames = {}
frames_lock = threading.Lock()


# ---------------------------------------------------------------------------
# /udp/compressed listener
# ---------------------------------------------------------------------------
def udp_listen_compressed_image(node: Node, sock: socket.socket, pub):
    """
    Reassembles chunked UDP packets into a full JPEG frame, decodes it with
    OpenCV for validation / optional processing, then publishes a
    CompressedImage message to /udp/compressed.

    Packet layout (8-byte header, same as the ROS1 sender):
        [0:4]  frame_id   - uint32, big-endian
        [4:6]  chunk_id   - uint16, big-endian  (0-indexed)
        [6:8]  total      - uint16, big-endian  (total chunks in this frame)
        [8:]   payload    - raw JPEG chunk bytes
    """
    while rclpy.ok():
        try:
            packet, _ = sock.recvfrom(65535)
            if len(packet) < 8:
                continue

            fid, chunk_id, total = struct.unpack("!IHH", packet[:8])
            chunk = packet[8:]

            with frames_lock:
                if fid not in frames:
                    frames[fid] = {"chunks": {}, "total": total}
                frames[fid]["chunks"][chunk_id] = chunk

                if len(frames[fid]["chunks"]) < total:
                    continue  # frame not yet complete

                # All chunks received - reassemble
                data = b"".join(frames[fid]["chunks"][i] for i in range(total))
                del frames[fid]

            # -- Image decode (OpenCV) ----------------------------------------
            np_arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                node.get_logger().debug(
                    f"Frame {fid} decoded OK - shape {img.shape}"
                )
            else:
                node.get_logger().warn(f"Frame {fid}: cv2.imdecode returned None")

            # -- Publish CompressedImage ---------------------------------------
            msg = CompressedImage()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = list(data)
            pub.publish(msg)

        except Exception as exc:
            node.get_logger().error(f"[compressed_image] {exc}")


# ---------------------------------------------------------------------------
# /joint_states listener
# ---------------------------------------------------------------------------
def udp_listen_joint_states(node: Node, sock: socket.socket, pub):
    """
    Receives a raw ROS1-serialized JointState message via UDP and publishes
    it to /joint_states.
    """
    while rclpy.ok():
        try:
            packet, _ = sock.recvfrom(4096)

            # Parse ROS1 wire format manually
            offset = 0

            # --- Header ---
            # seq (uint32)
            seq = struct.unpack_from('<I', packet, offset)[0]; offset += 4
            # stamp secs + nsecs (int32 each)
            secs = struct.unpack_from('<i', packet, offset)[0]; offset += 4
            nsecs = struct.unpack_from('<i', packet, offset)[0]; offset += 4
            # frame_id (uint32 length + bytes)
            fid_len = struct.unpack_from('<I', packet, offset)[0]; offset += 4
            frame_id = packet[offset:offset+fid_len].decode('utf-8'); offset += fid_len

            # --- name (string[]) ---
            name_count = struct.unpack_from('<I', packet, offset)[0]; offset += 4
            names = []
            for _ in range(name_count):
                n_len = struct.unpack_from('<I', packet, offset)[0]; offset += 4
                names.append(packet[offset:offset+n_len].decode('utf-8')); offset += n_len

            # --- position (float64[]) ---
            pos_count = struct.unpack_from('<I', packet, offset)[0]; offset += 4
            positions = list(struct.unpack_from(f'<{pos_count}d', packet, offset))
            offset += pos_count * 8

            # --- velocity (float64[]) ---
            vel_count = struct.unpack_from('<I', packet, offset)[0]; offset += 4
            velocities = list(struct.unpack_from(f'<{vel_count}d', packet, offset))
            offset += vel_count * 8

            # --- effort (float64[]) ---
            eff_count = struct.unpack_from('<I', packet, offset)[0]; offset += 4
            efforts = list(struct.unpack_from(f'<{eff_count}d', packet, offset))

            # Build and publish ROS2 JointState
            msg = JointState()
            msg.header.stamp.sec  = secs
            msg.header.stamp.nanosec = nsecs
            msg.header.frame_id   = frame_id
            msg.name              = names
            msg.position          = positions
            msg.velocity          = velocities
            msg.effort            = efforts

            pub.publish(msg)
            node.get_logger().debug(f"[joint_states] Published {names}")

        except Exception as exc:
            node.get_logger().error(f"[joint_states] {exc}")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Node("ros2_bridge_receiver")

    # Publishers
    pub_img   = node.create_publisher(CompressedImage, "/udp/compressed", 10)
    pub_joint = node.create_publisher(JointState,      "/joint_states",   10)

    # UDP sockets - bind to this PC's IP (192.168.1.2)
    sock_img = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_img.bind(("192.168.1.3", 9002))
    node.get_logger().info("Listening on 192.168.1.3:9002  ->  /udp/compressed")

    sock_joint = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_joint.bind(("192.168.1.3", 9003))
    node.get_logger().info("Listening on 192.168.1.3:9003  ->  /joint_states")

    # Background listener threads
    threading.Thread(
        target=udp_listen_compressed_image,
        args=(node, sock_img, pub_img),
        daemon=True
    ).start()

    threading.Thread(
        target=udp_listen_joint_states,
        args=(node, sock_joint, pub_joint),
        daemon=True
    ).start()

    node.get_logger().info("ROS2 Bridge Receiver running - press Ctrl+C to stop.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sock_img.close()
        sock_joint.close()


if __name__ == "__main__":
    main()
