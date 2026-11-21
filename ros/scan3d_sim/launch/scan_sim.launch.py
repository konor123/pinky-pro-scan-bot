"""Gazebo 스캔 시험: 서버(창 없음) + D455 옆 장착 로봇 + 센서 브리지 + command_driver + 시뮬레이션 에이전트.

    ros2 launch scan3d_sim scan_sim.launch.py role:=L world:=<scan3d_room.world> \
        cam_z:=0.20 pitch_deg:=10 x:=... y:=... yaw:=... scan3d_root:=<저장소>

world 파일과 스폰 자세는 scripts/sim_scan.sh가 만들어 넘긴다.
pinky_sim_examples launch/twin_arena.launch.py(1d14183)의 서버·로봇·브리지 구성을 따른다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from scan3d_sim.wheel_moves import COMMAND_TOPIC, STATUS_TOPIC


def generate_launch_description():
    share = get_package_share_directory('scan3d_sim')
    examples = get_package_share_directory('pinky_sim_examples')
    gz_share = get_package_share_directory('pinky_gz_sim')
    desc_share = get_package_share_directory('pinky_description')
    ros_gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    arg = LaunchConfiguration
    world_dir = PythonExpression(["__import__('os').path.dirname('", arg('world'), "')"])

    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'scan3d_robot.urdf.xacro'),
        ' d455_z:=', arg('cam_z'), ' d455_pitch_deg:=', arg('pitch_deg')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('role', description='L 또는 H'),
        DeclareLaunchArgument('world', description='scripts/sim_scan.sh가 만든 world 파일(절대 경로)'),
        DeclareLaunchArgument('world_name', default_value='scan3d_room'),
        DeclareLaunchArgument('cam_z', default_value='0.20', description='카메라 높이(바닥 기준, m)'),
        DeclareLaunchArgument('pitch_deg', default_value='10', description='카메라를 아래로 숙인 각(도)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='rad'),
        DeclareLaunchArgument('port', default_value='7010'),
        DeclareLaunchArgument('stream_port', default_value='7011'),
        DeclareLaunchArgument('scan3d_root', description='scan3d 패키지(protocol)가 있는 저장소 폴더'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [
            f'{desc_share}/../:{gz_share}/models:', world_dir,
            f':{os.path.expanduser("~")}/.gazebo/models']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ros_gz_launch), launch_arguments={
            'gz_args': ['-r -s --headless-rendering -v2 ', arg('world')],
            'on_exit_shutdown': 'true'}.items()),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True,
                          'ignore_timestamp': False}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'rate': 20.0, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', arg('world_name'), '-name', 'pinky', '-topic', 'robot_description',
                        '-x', arg('x'), '-y', arg('y'), '-z', '0.02', '-Y', arg('yaw')],
             parameters=[{'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='parameter_bridge',
             parameters=[{'config_file': os.path.join(examples, 'config', 'sim_bridge.yaml')}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='d455_bridge',
             parameters=[{'config_file': os.path.join(examples, 'config', 'd455_bridge.yaml')}]),

        Node(package='pinky_sim_examples', executable='command_driver', name='command_driver',
             output='screen', parameters=[{'use_sim_time': True, 'stop_distance': 0.0}],
             remappings=[('/pinky_command', COMMAND_TOPIC), ('/command_status', STATUS_TOPIC)]),
        Node(package='scan3d_sim', executable='sim_agent', name='sim_agent', output='screen',
             arguments=['--role', arg('role'), '--port', arg('port'), '--stream-port', arg('stream_port')],
             remappings=[('/pinky_command', COMMAND_TOPIC), ('/command_status', STATUS_TOPIC)],
             additional_env={'PYTHONPATH': [arg('scan3d_root'), ':',
                                            EnvironmentVariable('PYTHONPATH', default_value='')]}),
    ])
