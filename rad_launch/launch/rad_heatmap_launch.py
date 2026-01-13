# offline_rad_heatmap_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_rad_map')

    # Path to your parameter file for the C++ node
    param_file = os.path.join(pkg_share, 'params', 'heatmap.yaml')

    # Path to your trained model for the Python node
    model_file_path = os.path.join(pkg_share, 'models', 'gp_model.pkl')

    # 1. The C++ node that requests predictions (the client)
    rad_heatmap_client_node = Node(
        package='ros2_rad_map',
        executable='rad_heatmap_node',
        name='rad_heatmap_node',
        parameters=[param_file],
        output='screen'
    )

    # 2. The Python node that provides predictions (the server)
    gp_prediction_server_node = Node(
        package='ros2_rad_map',
        executable='gp_predictor_node.py', # The executable Python script
        name='gp_predictor_node',
        output='screen',
        parameters=[{'model_path': model_file_path}, {param_file}] # Set the model_path parameter
    )

    # The launch description now returns BOTH nodes
    return LaunchDescription([
        gp_prediction_server_node,
        rad_heatmap_client_node
    ])