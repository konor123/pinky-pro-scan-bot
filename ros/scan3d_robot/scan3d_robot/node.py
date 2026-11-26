"""ROS 2 Jazzy bus 연결과 로봇 에이전트 실행 진입점(docs/specs/robot_agent.md)."""
import argparse
import math
import queue
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from scan3d import protocol
from scan3d_robot.agent import Agent
from scan3d_robot.camera import RealSenseCamera


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class RosBus(Node):
    """Agent가 기대하는 bus 인터페이스를 rclpy 토픽으로 구현한다."""

    def __init__(self, clock=time.monotonic):
        super().__init__("scan3d_robot_agent")
        self._clock = clock
        self._lock = threading.Lock()
        self._odom = (0.0, 0.0, 0.0, 0.0)
        self._odom_cbs = []
        self._scan_cbs = []
        self._status_cbs = []
        self._status_q: queue.Queue[str] = queue.Queue()

        self._cmd_pub = self.create_publisher(String, "/pinky_command", 10)
        self._twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "/command_status", self._on_status, 10)

    def publish_command(self, text: str) -> None:
        self._cmd_pub.publish(String(data=str(text)))

    def publish_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._twist_pub.publish(msg)

    def command_subscribers(self) -> int:
        return int(self._cmd_pub.get_subscription_count())

    def next_status(self, timeout: float) -> str | None:
        try:
            return self._status_q.get(timeout=max(0.0, float(timeout)))
        except queue.Empty:
            return None

    def odom(self) -> tuple[float, float, float, float]:
        with self._lock:
            return self._odom

    def on_odom(self, fn) -> None:
        with self._lock:
            self._odom_cbs.append(fn)

    def on_scan(self, fn) -> None:
        with self._lock:
            self._scan_cbs.append(fn)

    def on_status(self, fn) -> None:
        with self._lock:
            self._status_cbs.append(fn)

    def _on_odom(self, msg: Odometry) -> None:
        t = float(self._clock())
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        sample = (t, float(p.x), float(p.y), yaw)
        with self._lock:
            self._odom = sample
            cbs = list(self._odom_cbs)
        for fn in cbs:
            fn(*sample)

    def _on_scan(self, msg: LaserScan) -> None:
        t = float(self._clock())
        with self._lock:
            cbs = list(self._scan_cbs)
        for fn in cbs:
            fn(t, float(msg.angle_min), float(msg.angle_increment),
               float(msg.range_min), float(msg.range_max), msg.ranges)

    def _on_status(self, msg: String) -> None:
        t = float(self._clock())
        text = str(msg.data)
        self._status_q.put(text)
        with self._lock:
            cbs = list(self._status_cbs)
        for fn in cbs:
            fn(t, text)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="scan3d robot TCP agent")
    parser.add_argument("--role", required=True, choices=["L", "H"])
    parser.add_argument("--port", type=int, default=protocol.CTRL_PORT)
    parser.add_argument("--stream-port", type=int, default=protocol.STREAM_PORT)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    bus = RosBus()
    camera = RealSenseCamera()
    agent = Agent(args.role, camera, bus, port=args.port, stream_port=args.stream_port)

    def _handle_signal(_signum, _frame):
        agent.shutdown()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    spin_thread = threading.Thread(target=rclpy.spin, args=(bus,), daemon=True)
    spin_thread.start()
    try:
        agent.serve_forever()
    finally:
        agent.shutdown()
        bus.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
