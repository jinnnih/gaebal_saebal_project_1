# -*- coding: utf-8 -*-
"""Gazebo Harmonic 주차장 월드 실행 (ROS2 Jazzy).

  ros2 launch parking_lot_world parking_world.launch.py
  ros2 launch parking_lot_world parking_world.launch.py gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('parking_lot_world')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg, 'worlds', 'parking_lot.sdf'),
            description='실행할 SDF 월드 경로'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),

        # 월드에서 model:// URI 로 모델을 찾을 수 있게
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.path.join(pkg, 'models') + os.pathsep +
                               os.path.join(pkg, 'worlds')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-v4 ', world],
                'on_exit_shutdown': 'true',
            }.items(),
            condition=IfCondition(gui)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-v4 -s -r ', world],
                'on_exit_shutdown': 'true',
            }.items(),
            condition=IfCondition(['not ', gui])),

        # /clock 브리지 — use_sim_time 을 쓰는 모든 노드에 필수
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='gz_clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'),
    ])
