#!/usr/bin/env python3
"""
TurtleBot3 Nav2 bring-up + 자동 초기위치 설정
 - map:  ~/turtlebot3_ws/maps/map.yaml (기본값)
 - use_sim_time: false 기본
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # ── 기본 Nav2 bring-up 포함 ───────────────────────────
    tb3_nav2_pkg = get_package_share_directory('turtlebot3_navigation2')
    nav2_launch  = PathJoinSubstitution([tb3_nav2_pkg, 'launch', 'navigation2.launch.py'])

    map_arg  = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/turtlebot3_ws/map/map.yaml'),
        description='absolute path to map.yaml')
    use_sim_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true'          # Lifecycle 자동 active
        }.items())

    # ── 자동 초기위치 노드 ────────────────────────────────
    auto_init = Node(
        package='hearbear',
        executable='auto_localize_spin.py',
        name='auto_localize_spin',
        output='screen',
        parameters=[{
            'service_name': '/reinitialize_global_localization',  # 필요 시 변경
            'angular_speed': 0.22,
            'spin_angle': 12.566,
            'cmd_vel_topic': '/cmd_vel'
        }])

    return LaunchDescription([map_arg, use_sim_arg, nav2, auto_init])