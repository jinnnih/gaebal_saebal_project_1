# -*- coding: utf-8 -*-
"""실행 중인 Gazebo Harmonic 에 valet_car 를 스폰하고 제어 스택을 올린다.

전제: parking_lot_world/launch/parking_world.launch.py 로 월드가 이미 떠 있을 것.

  ros2 launch valet_robot spawn_valet_car.launch.py

주요 인자
  x, y, yaw          스폰 포즈. 기본값은 parking_lot_world README 의
                     입구 진입 직후 지점 (-24.00, -19.30, 0.0)
                     = parking_spots.json 의 entry_pose
  cmd_vel_topic      twist_to_ackermann 이 구독할 토픽 (기본 /cmd_vel).
                     Nav2 와 함께 쓸 때는 /cmd_vel_smoothed
  cmd_vel_stamped    위 토픽이 TwistStamped 면 true
  rviz               RViz 동시 실행

올라오는 노드
  robot_state_publisher            URDF -> TF
  ros_gz_sim create                모델 스폰
  ros_gz_bridge                    /scan /imu /odom /tf
  joint_state_broadcaster          /joint_states
  ackermann_steering_controller    조향/구동
  twist_to_ackermann               cmd_vel -> 컨트롤러 reference (기구학 제한)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('valet_robot')
    xacro_file = os.path.join(pkg, 'urdf', 'valet_car.urdf.xacro')
    controllers = os.path.join(pkg, 'config', 'controllers.yaml')
    bridge_cfg = os.path.join(pkg, 'config', 'gz_bridge.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'valet_robot.rviz')

    name = LaunchConfiguration('name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_stamped = LaunchConfiguration('cmd_vel_stamped')
    use_rviz = LaunchConfiguration('rviz')
    use_lidar = LaunchConfiguration('lidar')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' sim:=true',
                 ' lidar:=', use_lidar,
                 ' controllers_file:=', controllers]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', name,
                   '-x', x, '-y', y, '-z', z, '-Y', yaw])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='valet_gz_bridge', output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}])

    jsb = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60',
                   # 헤드리스 기동 직후엔 gz 가 렌더러 초기화로 바빠서
                   # 기본 5 s 안에 상태전환이 안 끝난다 (실제로 타임아웃 발생함).
                   '--switch-timeout', '60'])

    ackermann = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['ackermann_steering_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60',
                   # 헤드리스 기동 직후엔 gz 가 렌더러 초기화로 바빠서
                   # 기본 5 s 안에 상태전환이 안 끝난다 (실제로 타임아웃 발생함).
                   '--switch-timeout', '60'])

    relay = Node(
        package='valet_robot', executable='twist_to_ackermann.py',
        name='twist_to_ackermann', output='screen',
        parameters=[{'use_sim_time': True,
                     'input_topic': cmd_vel_topic,
                     'input_stamped': cmd_vel_stamped,
                     'output_topic': '/ackermann_steering_controller/reference',
                     'max_speed_forward': 1.60,
                     'max_speed_reverse': 0.60,
                     'min_turning_radius': 3.5704}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_cfg])

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='valet_car'),
        DeclareLaunchArgument('x', default_value='-24.00'),
        DeclareLaunchArgument('y', default_value='-19.30'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_stamped', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # lidar:=false 로 두면 라이다 없이 뜬다. 렌더링이 막힌 환경에서
        # 물리/제어만 검증할 때 쓴다 (/dev/dri 권한 없을 때 등).
        DeclareLaunchArgument('lidar', default_value='true'),

        rsp,
        bridge,
        spawn,
        relay,
        rviz,

        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[ackermann])),
    ])
