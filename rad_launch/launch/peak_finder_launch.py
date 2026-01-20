from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_rad_map')

    # Path to your parameter file
    param_file = os.path.join(pkg_share, 'params', 'heatmap.yaml')

    # The C++ node that calls the peak finding service
    peak_finder_client_node = Node(
        package='ros2_rad_map',
        executable='peak_finder_client_node',
        name='peak_finder_client_node',
        parameters=[param_file],
        output='screen'
    )

    return LaunchDescription([
        peak_finder_client_node
    ])
