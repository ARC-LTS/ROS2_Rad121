from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_rad_map')

    # Parameter file for the heatmap node
    heatmap_param_file = os.path.join(pkg_share, 'params', 'heatmap.yaml')

    # Parameter file for the new zone planner node
    planner_param_file = os.path.join(pkg_share, 'params', 'zone_planner.yaml')

    # The new C++ node that finds and visualizes zones
    rad_zone_planner_node = Node(
        package='ros2_rad_map',
        executable='rad_zone_planner_node',
        name='rad_zone_planner_node',
        parameters=[planner_param_file],
        output='screen'
    )

    peak_finder_client_node = Node(
        package='ros2_rad_map',
        executable='peak_finder_client_node',
        name='peak_finder_client_node',
        parameters=[heatmap_param_file],
        output='screen'
    )

    return LaunchDescription([
        rad_zone_planner_node,
        peak_finder_client_node
    ])
