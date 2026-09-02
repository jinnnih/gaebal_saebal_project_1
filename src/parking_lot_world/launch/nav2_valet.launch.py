# -*- coding: utf-8 -*-
"""주차장 맵 + Nav2 (Smac Hybrid-A* / MPPI) 스택 실행.

  # 1) 월드
  ros2 launch parking_lot_world parking_world.launch.py
  # 2) 로봇 스폰 (팀원 A 의 로봇 패키지)
  # 3) Nav2
  ros2 launch parking_lot_world nav2_valet.launch.py

옵션
  use_costmap_filters:=true   keepout(주차면 진입금지) + speed(감속) 필터 활성화
  map:=<경로>                 사용할 맵 yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NAV2_NODES = [
    ('nav2_controller',      'controller_server',   'controller_server'),
    ('nav2_smoother',        'smoother_server',     'smoother_server'),
    ('nav2_planner',         'planner_server',      'planner_server'),
    ('nav2_behaviors',       'behavior_server',     'behavior_server'),
    ('nav2_bt_navigator',    'bt_navigator',        'bt_navigator'),
    ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
]


def generate_launch_description():
    pkg = get_package_share_directory('parking_lot_world')
    default_map = os.path.join(pkg, 'maps', 'parking_lot.yaml')
    params = os.path.join(pkg, 'config', 'nav2_ackermann.yaml')
    filters = os.path.join(pkg, 'config', 'costmap_filters.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    use_filters = LaunchConfiguration('use_costmap_filters')

    common = {'use_sim_time': use_sim_time}

    nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[params, common, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[params, common]),
    ]
    # Ackermann 전용 BT. 기본 트리는 복구행동에 Spin 을 쓰는데 차량형은
    # 제자리 회전이 안 돼 behavior_plugins 에서 Spin 을 뺐고, 그러면 기본
    # 트리가 로드 단계에서 죽는다. 경로는 여기서 절대경로로 넘긴다
    # (YAML 파라미터 파일에서는 $(find-pkg-share ...) 가 치환되지 않는다).
    bt_params = {
        'default_nav_to_pose_bt_xml': os.path.join(
            pkg, 'behavior_trees', 'navigate_to_pose_ackermann.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(
            pkg, 'behavior_trees', 'navigate_through_poses_ackermann.xml'),
    }

    for pkg_name, exe, name in NAV2_NODES:
        extra = [bt_params] if name == 'bt_navigator' else []
        nodes.append(Node(package=pkg_name, executable=exe, name=name,
                          output='screen', parameters=[params, common] + extra,
                          remappings=[('cmd_vel', 'cmd_vel_nav')]))

    lifecycle = ['map_server', 'amcl'] + [n for _, _, n in NAV2_NODES]

    filter_group = GroupAction(
        condition=IfCondition(use_filters),
        actions=[
            Node(package='nav2_map_server', executable='map_server',
                 name='filter_mask_server', output='screen',
                 parameters=[filters, common,
                             {'yaml_filename': os.path.join(pkg, 'maps',
                                                            'keepout_mask.yaml')}]),
            Node(package='nav2_map_server', executable='costmap_filter_info_server',
                 name='costmap_filter_info_server', output='screen',
                 parameters=[filters, common]),
            Node(package='nav2_map_server', executable='map_server',
                 name='speed_mask_server', output='screen',
                 parameters=[filters, common,
                             {'yaml_filename': os.path.join(pkg, 'maps',
                                                            'speed_mask.yaml')}]),
            Node(package='nav2_map_server', executable='costmap_filter_info_server',
                 name='speed_filter_info_server', output='screen',
                 parameters=[filters, common]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_filters', output='screen',
                 parameters=[common, {'autostart': True},
                             {'node_names': ['filter_mask_server',
                                             'costmap_filter_info_server',
                                             'speed_mask_server',
                                             'speed_filter_info_server']}]),
        ])

    # 주차면 관리 노드 파라미터 (노드 구현은 팀원 A 패키지)
    spot_params = os.path.join(pkg, 'config', 'parking_spots.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_costmap_filters', default_value='false',
                              description='주차면 keepout / 감속 필터 사용'),
        DeclareLaunchArgument('spot_params', default_value=spot_params),
        *nodes,
        filter_group,
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[common, {'autostart': True}, {'node_names': lifecycle}]),
    ])
