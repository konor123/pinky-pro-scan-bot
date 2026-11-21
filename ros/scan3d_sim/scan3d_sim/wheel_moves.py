"""Gazebo Pinky의 거리·회전 명령을 실행하는 CLI."""

import argparse
import math
import time
from collections import deque

COMMAND_TOPIC = '/scan3d_sim/pinky_command'
STATUS_TOPIC = '/scan3d_sim/command_status'


def positive_number(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('유한한 양수를 입력하세요.')
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=positive_number, default=60.0,
                        help='명령 완료 대기 시간(초)')
    parser.add_argument('--speed', type=positive_number, help='직진 속도(m/s, 0.02~0.25)')
    moves = parser.add_subparsers(dest='move', required=True)
    for name in ('forward', 'back', 'square'):
        moves.add_parser(name).add_argument('distance', type=positive_number, help='거리(m)')
    for name in ('left', 'right'):
        moves.add_parser(name).add_argument('degrees', type=positive_number, help='각도(도)')
    moves.add_parser('demo', help='10cm 전진·후진, 좌·우 90도 회전')
    moves.add_parser('stop', help='현재 이동과 대기 동작 취소')
    args = parser.parse_args(argv)

    if args.move in ('forward', 'back', 'square'):
        if args.distance > 5:
            parser.error('거리는 5m 이하여야 합니다.')
        command = f'{args.move} {args.distance}'
    elif args.move in ('left', 'right'):
        if args.degrees > 720:
            parser.error('각도는 720도 이하여야 합니다.')
        degrees = args.degrees if args.move == 'left' else -args.degrees
        command = f'turn {degrees}'
    elif args.move == 'demo':
        command = 'forward 0.1; back 0.1; turn 90; turn -90'
    else:
        command = 'stop'

    if args.speed is not None:
        if command == 'stop' or not 0.02 <= args.speed <= 0.25:
            parser.error('--speed는 이동 명령에서 0.02~0.25 m/s로 지정하세요.')
        command = f'speed {args.speed}; {command}'
    return run_command(command, args.timeout)


def run_command(command, timeout):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('scan3d_wheel_moves')
    statuses = deque()
    publisher = node.create_publisher(String, COMMAND_TOPIC, 10)
    subscription = node.create_subscription(String, STATUS_TOPIC,
                                            lambda msg: statuses.append(msg.data), 10)
    pending = False
    try:
        deadline = time.monotonic() + 5
        while not (publisher.get_subscription_count() and subscription.get_publisher_count()):
            if time.monotonic() >= deadline:
                print('Gazebo 명령 드라이버를 찾지 못했습니다.')
                return 1
            rclpy.spin_once(node, timeout_sec=0.1)

        pending = command != 'stop'
        publisher.publish(String(data=command))
        accepted = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            while statuses:
                status = statuses.popleft()
                if status.startswith(f'accepted: {command} ('):
                    accepted = True
                if status.startswith('stopped:') and (accepted or command == 'stop'):
                    pending = False
                    print(status)
                    return 0 if command == 'stop' else 1
                if accepted and status.startswith('blocked:'):
                    pending = False
                    print(status)
                    return 1
                if accepted and status.startswith('done:') and status.endswith('(모든 명령 완료)'):
                    pending = False
                    print(status)
                    return 0
        print('Gazebo 명령 완료 대기 시간이 초과됐습니다.')
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if pending:
            publisher.publish(String(data='stop'))
            publisher.wait_for_all_acked(timeout=Duration(seconds=0.5))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
