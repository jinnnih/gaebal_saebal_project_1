# -*- coding: utf-8 -*-
"""Gazebo Harmonic 주차장 월드 실행 (ROS2 Jazzy).

  ros2 launch parking_lot_world parking_world.launch.py
  ros2 launch parking_lot_world parking_world.launch.py gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('parking_lot_world')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')

    # paused:=false 면 -r 을 붙여 시뮬을 바로 돌린다.
    # ! 이게 없으면 GUI 모드가 "일시정지" 상태로 떠서 /clock 이 흐르지 않고,
    #   use_sim_time 을 쓰는 controller_manager 가 영영 활성화되지 않는다.
    #   (사람이 GUI 재생 버튼을 눌러야만 진행됨)
    run_flag = PythonExpression(
        ["'' if '", paused, "' == 'true' else '-r '"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg, 'worlds', 'parking_lot.sdf'),
            description='실행할 SDF 월드 경로'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),

        # 월드에서 model:// URI 로 모델을 찾을 수 있게.
        # ! Set 이 아니라 Append 여야 한다. Set 으로 덮어쓰면 다른 패키지가
        #   ament 환경 훅으로 등록해 둔 경로가 지워져서, 로봇 패키지의
        #   package://.../meshes/*.obj 를 Gazebo 가 못 찾는다.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(pkg, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(pkg, 'worlds')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-v4 ', run_flag, world],
                'on_exit_shutdown': 'true',
            }.items(),
            condition=IfCondition(gui)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-v4 -s ', run_flag, world],
                'on_exit_shutdown': 'true',
            }.items(),
            condition=UnlessCondition(gui)),

        # /clock 브리지 — use_sim_time 을 쓰는 모든 노드에 필수
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='gz_clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'),
    ])
