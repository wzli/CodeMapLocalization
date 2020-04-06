from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        'capture_device': '0',
        'frame_id': 'map',
    }
    node = Node(package='code_map_localization',
                node_executable='webcam',
                output='screen',
                parameters=[params])
    return LaunchDescription([node])
