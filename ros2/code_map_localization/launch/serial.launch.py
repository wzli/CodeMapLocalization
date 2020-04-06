from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        'serial_device': '/dev/ttyUSB0',
        'csv_file': 'None',
        'frame_id': 'map',
    }
    node = Node(package='code_map_localization',
                node_executable='serial',
                output='screen',
                parameters=[params])
    return LaunchDescription([node])
