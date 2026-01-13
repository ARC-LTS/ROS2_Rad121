from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('ros2_rad_map'),
        'params',
        'zones.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_rad_map',
            executable='zone_selector_node',
            name='zone_selector_node',
            parameters=[param_file],
            output='screen'
        )
    ])
