"""Gazebo RGB-D 토픽을 받아 scan3d_robot.agent.Agent로 로봇과 같은 TCP 서비스를 띄운다."""
import argparse
import signal
import threading

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from scan3d import protocol
from scan3d_robot.agent import Agent
from scan3d_robot.node import RosBus

from scan3d_sim.sim_camera import GazeboCamera


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Gazebo scan3d simulation TCP agent")
    ap.add_argument("--role", required=True, choices=["L", "H"])
    ap.add_argument("--port", type=int, default=protocol.CTRL_PORT)
    ap.add_argument("--stream-port", type=int, default=protocol.STREAM_PORT)
    ap.add_argument("--topic-prefix", default="/d455")
    ap.add_argument("--timeout", type=float, default=20.0)
    args, ros_args = ap.parse_known_args(argv)

    prefix = args.topic_prefix.rstrip("/")
    rclpy.init(args=ros_args)
    bus = RosBus()
    camera = GazeboCamera(serial=f"SIM-{args.role}", timeout_s=args.timeout)
    bus.create_subscription(CameraInfo, f"{prefix}/camera_info", camera.on_camera_info_msg, qos_profile_sensor_data)
    bus.create_subscription(Image, f"{prefix}/image", camera.on_color_msg, qos_profile_sensor_data)
    bus.create_subscription(Image, f"{prefix}/depth_image", camera.on_depth_msg, qos_profile_sensor_data)

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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
