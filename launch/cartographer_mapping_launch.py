from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
import os

def generate_launch_description():
    map_save_path = '/home/nvidia/turtlebot3_ws/map'
    map_name = 'map'

    # Cartographer SLAM 실행
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('turtlebot3_cartographer').find('turtlebot3_cartographer'),
                'launch',
                'cartographer.launch.py'
            )
        ])
    )

    rosbridge =TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[Launch] ROS BRIDGE'),
            ExecuteProcess(
                cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
                output='screen'
            )
        ]
    )

    # 사각형 경로 이동 노드 정의
    move_rectangle_node = Node(
        package='hearbear',
        executable='move_rectangle.py',
        name='move_rectangle',
        output='screen',
        parameters=[
            {'linear_speed': 0.05},
            {'angular_speed': 0.2}
        ]
    )

    # move_rectangle 종료 후 맵 저장 (10초 후)
    map_save = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-f', os.path.join(map_save_path, map_name)
                ],
                output='screen'
            )
        ]
    )

    # map 저장 후 10초 뒤 .png 변환
    convert_to_png = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[Launch] Converting PGM to PNG...'),
            ExecuteProcess(
                cmd=[
                    'convert',
                    os.path.join(map_save_path, map_name + '.pgm'),
                    os.path.join(map_save_path, map_name + '.png')
                ],
                output='screen'
            )
        ]
    )

    send_map_node = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg='[Launch] Publishing map over rosbridge...'),
            Node(
                package='hearbear',
                executable='map_sender_node.py',
                name='map_sender_node',
                output='screen'
            )
        ]
    )

    # png 저장 후 10초 뒤 전체 프로그램 종료
    shutdown = TimerAction(
        period=50.0,
        actions=[
            LogInfo(msg='[Launch] All tasks completed. Shutting down...'),
            EmitEvent(event=Shutdown(reason='Mapping and PNG conversion complete'))
        ]
    )

    # move_rectangle 노드 종료 이벤트 핸들러로 후속 단계 연결
    move_rectangle_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=move_rectangle_node,
            on_exit=[map_save, convert_to_png, shutdown]
        )
    )

    return LaunchDescription([
        cartographer,
        TimerAction(period=10.0, actions=[move_rectangle_node]),
        move_rectangle_handler
    ]) 