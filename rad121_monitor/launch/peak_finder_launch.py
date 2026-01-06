from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rad121_monitor')

    param_file = os.path.join(pkg_share, 'params', 'heatmap.yaml')

    gp_service_node = Node(
        package='rad121_monitor',
        executable='gp_predictor_node.py',
        name='gp_predictor_node',
        output='screen',
        parameters=[param_file]
    )

    peak_finder_client_node = Node(
        package='rad121_monitor',
        executable='peak_finder_client_node',
        name='peak_finder_client_node',
        parameters=[param_file],
        output='screen'
    )

    return LaunchDescription([
        gp_service_node,
        peak_finder_client_node
    ])
