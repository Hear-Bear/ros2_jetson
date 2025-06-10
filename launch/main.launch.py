from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TurtleBot3 bringup 먼저 실행
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'turtlebot3_bringup', 'robot.launch.py',
                'use_sim_time:=False'
            ],
            output='screen'
        ),

        # 약 10초 후에 map_monitor 노드 실행
        TimerAction(
            period=10.0,  # 지연 시간 (초)
            actions=[
                Node(
                    package='hearbear',
                    executable='main_monitor.py',
                    name='main_monitor',
                    output='screen',
                )
            ]
        )
    ])