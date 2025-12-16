from launch import LaunchDescription
from launch.actions import (
    TimerAction, ExecuteProcess, IncludeLaunchDescription, 
    RegisterEventHandler, EmitEvent, LogInfo
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
import os

def generate_launch_description():
    map_save_path = '/home/nvidia/turtlebot3_ws/map'
    map_name = 'map'
    map_file_base = os.path.join(map_save_path, map_name) # 경로 중복 제거

    # --- 1. Cartographer SLAM 실행 ---
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('turtlebot3_cartographer').find('turtlebot3_cartographer'),
                'launch',
                'cartographer.launch.py'
            )
        ])
    )

    # --- 2. Rosbridge (10초 후 실행) ---
    # (참고: 이 노드는 원본 코드에서 LD 리스트에 포함되지 않았지만, 
    #  필요하다면 포함시킵니다.)
    rosbridge = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[Launch] ROS BRIDGE'),
            ExecuteProcess(
                cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
                output='screen'
            )
        ]
    )

    # --- 3. 사각형 경로 이동 노드 (10초 후 실행) ---
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

    # === 작업 단계 정의 (TimerAction이 아닌 실제 작업) ===

    # 4. 맵 저장 (ExecuteProcess)
    map_save_action = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_file_base
        ],
        output='screen'
    )

    # 5. PNG 변환 (ExecuteProcess)
    convert_to_png_action = ExecuteProcess(
        cmd=[
            'convert',
            map_file_base + '.pgm',
            map_file_base + '.png'
        ],
        output='screen'
    )

    # 6. 맵 전송 (Node)
    send_map_node_action = Node(
        package='hearbear',
        executable='map_sender_node.py',
        name='map_sender_node',
        output='screen'
    )

    # 7. 종료 이벤트
    shutdown_action = EmitEvent(
        event=Shutdown(reason='Mapping and PNG conversion complete')
    )


    # === 이벤트 핸들러 체인 ===

    # Handler 1: move_rectangle이 끝나면 -> (5초 대기) -> map_save 실행
    handler_1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=move_rectangle_node,
            on_exit=[
                LogInfo(msg='[Handler 1] Move complete. Waiting 5s...'),
                # Cartographer가 맵을 완성할 시간을 5초 정도 줍니다.
                TimerAction(period=5.0, actions=[
                    LogInfo(msg='[Handler 1] Saving map...'),
                    map_save_action
                ])
            ]
        )
    )

    # Handler 2: map_save가 끝나면 -> convert_to_png 실행
    handler_2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=map_save_action,
            on_exit=[
                LogInfo(msg='[Handler 2] Map saved. Converting to PNG...'),
                convert_to_png_action
            ]
        )
    )

    # Handler 3: convert_to_png가 끝나면 -> send_map_node 실행
    handler_3 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=convert_to_png_action,
                on_exit=[
                    LogInfo(msg='[Handler 3] PNG converted. Sending map...'),
                    send_map_node_action, # 맵 전송 노드 시작
    
                    # 맵 전송 노드가 시작하고 (종료를 기다리지 않고) 5초 뒤에 전체 종료
                    LogInfo(msg='[Handler 3] Starting 5s shutdown timer...'),
                    TimerAction(period=5.0, actions=[shutdown_action]) 
                ]
          )
    )


    # --- 최종 런치 리스트 반환 ---
    return LaunchDescription([
        cartographer,
        rosbridge,  # Rosbridge를 사용한다면 포함
        
        # 10초 후 사각형 이동 시작
        TimerAction(period=10.0, actions=[move_rectangle_node]),

        # 모든 이벤트 핸들러 등록
        handler_1,
        handler_2,
        handler_3
    ])