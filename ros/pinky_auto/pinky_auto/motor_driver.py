import time

import rclpy
from pinky_bringup.bringup import Pinky

from .cmd_watchdog import CmdWatchdog


class GuardedPinky(Pinky):
    """Vendor motor driver that stops the wheels when cmd_vel goes silent."""

    def __init__(self):
        super().__init__()
        self.declare_parameter('cmd_timeout', 0.5)
        timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self._watchdog = CmdWatchdog(timeout)
        if self.is_initialized:
            self.create_timer(0.05, self._check_cmd_timeout)
            self.get_logger().info(f'cmd_vel timeout: {timeout:.2f} s')

    def twist_callback(self, msg):
        self._watchdog.feed(time.monotonic())
        super().twist_callback(msg)

    def _check_cmd_timeout(self):
        if not self._watchdog.should_stop(time.monotonic()):
            return
        if self.driver.set_double_rpm(0, 0):
            self._watchdog.mark_stopped()
            self.get_logger().warn('cmd_vel timed out, motors stopped')
        else:
            self.get_logger().error('cmd_vel timed out, stop command failed')


def main(args=None):
    rclpy.init(args=args)
    node = GuardedPinky()
    try:
        if node.is_initialized:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.is_initialized:
            node.driver.set_double_rpm(0, 0)
            node.driver.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
