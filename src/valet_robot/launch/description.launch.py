# -*- coding: utf-8 -*-
"""URDF 모델만 확인 (Gazebo 없이 RViz + joint_state_publisher_gui).

  ros2 launch valet_robot description.launch.py

조향 조인트 슬라이더를 움직여 좌/우 조향각과 바퀴 위치가 맞는지 눈으로 본다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('valet_robot')
    xacro_file = os.path.join(pkg, 'urdf', 'valet_car.urdf.xacro')
    rviz_cfg = os.path.join(pkg, 'rviz', 'valet_robot.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=false']), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': False}]),

        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui', output='screen'),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rviz_cfg]),
    ])
