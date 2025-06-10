from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = 'hearbear'
    node_exe = os.path.join(
        get_package_share_directory(pkg), '..',
        'lib', pkg, 'audio_doa_rotate_node.py'
    )

    audio_node = Node(
        package=pkg,
        executable='audio_doa_rotate_node.py',  # 실행 파일 이름
        name='audio_doa_rotate',
        output='screen',
        parameters=[
            {'threshold_db': -20.0},
            {'record_sec':   5.0},
            {'ang_vel':      0.5},
        ],
    )

    return LaunchDescription([audio_node])
