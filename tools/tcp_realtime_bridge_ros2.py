#!/usr/bin/env python3
import json
import socket
import threading
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TcpRealtimeBridgeRos2(Node):
    def __init__(self):
        super().__init__("tcp_realtime_bridge_ros2")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5000)
        self.declare_parameter("trajectory_topic", "/revo2_hand_controller/joint_trajectory")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        topic = str(self.get_parameter("trajectory_topic").value)

        self.pub = self.create_publisher(JointTrajectory, topic, 10)

        self.thread = threading.Thread(target=self._server_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f"TCP bridge listening on {self.host}:{self.port}, pub -> {topic}")

    def _publish_hand(self, msg: Dict[str, Any]):
        positions = msg.get("hand_positions", [])
        sec = float(msg.get("hand_sec", 0.2))
        if not isinstance(positions, list) or len(positions) != 6:
            return

        traj = JointTrajectory()
        traj.joint_names = [
            "right_thumb_metacarpal_joint",
            "right_thumb_proximal_joint",
            "right_index_proximal_joint",
            "right_middle_proximal_joint",
            "right_ring_proximal_joint",
            "right_pinky_proximal_joint",
        ]
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in positions]
        p.time_from_start.sec = int(sec)
        p.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
        traj.points = [p]
        self.pub.publish(traj)

    def _server_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.listen(1)
        while rclpy.ok():
            conn, addr = s.accept()
            self.get_logger().info(f"TCP client connected: {addr}")
            with conn:
                buf = b""
                while rclpy.ok():
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            data = json.loads(line.decode("utf-8"))
                            self._publish_hand(data)
                        except Exception:
                            pass
            self.get_logger().info("TCP client disconnected")


def main(args=None):
    rclpy.init(args=args)
    node = TcpRealtimeBridgeRos2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
