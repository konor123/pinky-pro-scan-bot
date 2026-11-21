"""앞 로봇을 초음파로 gap(기본 15 cm) 간격을 두고 따라가며 원을 laps 바퀴 돈다.

입력: /us_sensor/range(pinky_sensor_adc, 약 20 Hz), /odom
출력: /cmd_vel(pinky_auto motor_driver. 0.5 s 동안 명령이 없으면 모터가 선다)
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from pinky_auto.follow_core import (DONE, FollowParams, Follower, LapCounter, MedianFilter,
                                    yaw_from_quat)

RATE_HZ = 20.0
RANGE_TIMEOUT_S = 0.5


class FollowUltra(Node):
    def __init__(self):
        super().__init__('follow_ultra')
        d = FollowParams()
        decl = {
            'gap': d.gap, 'kp': d.kp, 'v_max': d.v_max, 'accel': d.accel,
            'radius': d.radius, 'direction': d.direction, 'stop_dist': d.stop_dist,
            'lost_dist': d.lost_dist, 'deadband': d.deadband, 'window': d.window,
        }
        for k, v in decl.items():
            self.declare_parameter(k, v)
        self.declare_parameter('laps', 1)
        self.params = FollowParams(**{k: type(v)(self.get_parameter(k).value) for k, v in decl.items()})
        self.laps = int(self.get_parameter('laps').value)

        self.filter = MedianFilter(self.params.window)
        self.follower = Follower(self.params)
        self.laps_counter = LapCounter(self.params.direction)
        self.dist = None
        self.range_t = 0.0
        self.have_odom = False
        self.state = None
        self.finished = False
        self.last_log = 0.0
        self.last_tick = time.monotonic()

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Range, 'us_sensor/range', self._on_range, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(1.0 / RATE_HZ, self._tick)
        p = self.params
        self.get_logger().info(
            f'follow: gap={p.gap:.2f} m, radius={p.radius:.2f} m, '
            f"{'left(CCW)' if p.direction > 0 else 'right(CW)'}, laps={self.laps}, v_max={p.v_max:.2f} m/s")

    def _on_range(self, msg):
        r = msg.range
        if not (msg.min_range <= r <= msg.max_range):
            r = None
        self.dist = self.filter.push(r)
        self.range_t = time.monotonic()

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.laps_counter.update(yaw_from_quat(q.z, q.w, q.x, q.y))
        self.have_odom = True

    def _publish(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        if self.finished:
            self._publish(0.0, 0.0)
            return
        if not self.have_odom or now - self.range_t > RANGE_TIMEOUT_S:
            self.follower.stop()
            self._publish(0.0, 0.0)
            self._set_state('waiting(no range/odom)')
            return
        if self.laps_counter.done(self.laps):
            self.finished = True
            self.follower.stop()
            self._publish(0.0, 0.0)
            self._set_state(DONE)
            self.get_logger().info(f'{self.laps} lap(s) done: turned {math.degrees(self.laps_counter.turned):.0f} deg')
            return
        v, w, state = self.follower.step(self.dist, dt)
        self._publish(v, w)
        self._set_state(state)
        if now - self.last_log > 1.0:
            self.last_log = now
            dist = 'none' if self.dist is None else f'{self.dist:.3f}'
            self.get_logger().info(
                f'dist={dist} m v={v:.3f} m/s w={w:.2f} rad/s turned={math.degrees(self.laps_counter.turned):.0f} deg')

    def _set_state(self, state):
        if state != self.state:
            self.state = state
            self.get_logger().info(f'state: {state}')

    def stop(self):
        # 종료 신호로 컨텍스트가 먼저 닫혔으면 보낼 수 없다. 그때는 motor_driver의 cmd_vel 시간 초과(0.5 s)가 세운다.
        try:
            for _ in range(3):
                self._publish(0.0, 0.0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FollowUltra()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
