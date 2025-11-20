from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rad121_monitor')

    # Parameter file for the heatmap node
    heatmap_param_file = os.path.join(pkg_share, 'params', 'heatmap.yaml')

    # Parameter file for the new zone planner node
    planner_param_file = os.path.join(pkg_share, 'params', 'zone_planner.yaml')

    # The GP prediction server
    gp_prediction_server_node = Node(
        package='rad121_monitor',
        executable='gp_predictor_node.py',
        name='gp_predictor_node',
        output='screen'
    )

    # The C++ node that generates the heatmaps
    rad_heatmap_node = Node(
        package='rad121_monitor',
        executable='rad_heatmap_node',
        name='rad_heatmap_node',
        parameters=[heatmap_param_file],
        output='screen'
    )

    # The new C++ node that finds and visualizes zones
    rad_zone_planner_node = Node(
        package='rad121_monitor',
        executable='rad_zone_planner_node',
        name='rad_zone_planner_node',
        parameters=[planner_param_file],
        output='screen'
    )

    return LaunchDescription([
        gp_prediction_server_node,
        rad_heatmap_node,
        rad_zone_planner_node
    ])
