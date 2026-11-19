import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .patrol_core import PatrolSequence, load_plan


def to_pose_stamped(navigator, frame_id, pose):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = navigator.get_clock().now().to_msg()
    msg.pose.position.x = pose.x
    msg.pose.position.y = pose.y
    msg.pose.orientation.z, msg.pose.orientation.w = pose.quaternion_zw()
    return msg


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator(node_name='patrol')
    navigator.declare_parameter('waypoints_file', '')
    path = navigator.get_parameter('waypoints_file').get_parameter_value().string_value
    log = navigator.get_logger()
    try:
        plan = load_plan(path)
        if plan.initial_pose is not None:
            navigator.setInitialPose(to_pose_stamped(navigator, plan.frame_id, plan.initial_pose))
        navigator.waitUntilNav2Active()

        sequence = PatrolSequence(plan)
        while rclpy.ok():
            waypoint = sequence.next()
            if waypoint is None:
                break
            log.info(f'lap {sequence.lap + 1}: going to {waypoint.name} ({waypoint.x:.2f}, {waypoint.y:.2f})')
            navigator.goToPose(to_pose_stamped(navigator, plan.frame_id, waypoint))
            while not navigator.isTaskComplete():
                pass
            succeeded = navigator.getResult() == TaskResult.SUCCEEDED
            if not succeeded:
                log.warn(f'{waypoint.name}: not reached')
            sequence.report(succeeded)
        log.info('patrol finished')
    except KeyboardInterrupt:
        navigator.cancelTask()
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
