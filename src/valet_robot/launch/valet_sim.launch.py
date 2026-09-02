# -*- coding: utf-8 -*-
"""주차장 월드 + valet_car + (선택) Nav2 를 한 번에 띄운다.

  # 월드 + 로봇 + RViz   (계획서 1주차: 수동 조향 주행 확인)
  ros2 launch valet_robot valet_sim.launch.py rviz:=true
  ros2 run valet_robot ackermann_teleop_key.py

  # 월드 + 로봇 + Nav2   (2주차 이후)
  ros2 launch valet_robot valet_sim.launch.py nav2:=true rviz:=true

nav2:=true 이면 twist_to_ackermann 의 입력이 자동으로 /cmd_vel_smoothed 가 된다.
parking_lot_world/launch/nav2_valet.launch.py 는 velocity_smoother 까지만 띄우고
collision_monitor 를 띄우지 않아 최종 속도 토픽이 /cmd_vel_smoothed 이기 때문이다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    world_pkg = get_package_share_directory('parking_lot_world')
    robot_pkg = get_package_share_directory('valet_robot')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    use_nav2 = LaunchConfiguration('nav2')
    use_lidar = LaunchConfiguration('lidar')
    use_filters = LaunchConfiguration('use_costmap_filters')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    cmd_vel_topic = PythonExpression(
        ['"/cmd_vel_smoothed" if "', use_nav2, '" == "true" else "/cmd_vel"'])

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(world_pkg, 'launch', 'parking_world.launch.py')),
        launch_arguments={'gui': gui, 'world': world}.items())

    # 월드/센서 시스템이 뜬 뒤에 스폰해야 gz_ros2_control 이 안정적으로 붙는다.
    robot = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_pkg, 'launch', 'spawn_valet_car.launch.py')),
            launch_arguments={'x': x, 'y': y, 'yaw': yaw,
                              'rviz': use_rviz, 'lidar': use_lidar,
                              'cmd_vel_topic': cmd_vel_topic}.items())])

    nav2 = TimerAction(period=12.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(world_pkg, 'launch', 'nav2_valet.launch.py')),
            launch_arguments={'use_costmap_filters': use_filters}.items(),
            condition=IfCondition(use_nav2))])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(world_pkg, 'worlds', 'parking_lot.sdf'),
            description='SDF 월드 경로'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('nav2', default_value='false'),
        DeclareLaunchArgument('lidar', default_value='true'),
        # keepout 필터: 주차면 내부를 진입금지로 만들어 통로 주행 중
        # 플래너가 주차면을 가로지르는 것을 막는다. 주차할 때는 런타임으로
        # 꺼야 한다 (ParkManeuver 진입 직전) — 이슈 #7.
        DeclareLaunchArgument('use_costmap_filters', default_value='false'),
        DeclareLaunchArgument('x', default_value='-23.00'),
        DeclareLaunchArgument('y', default_value='-19.30'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        world,
        robot,
        nav2,
    ])
