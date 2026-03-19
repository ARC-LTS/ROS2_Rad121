from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Launch the GP predictor node as a standalone service."""
    
    ros2_rad_map_share = get_package_share_directory('ros2_rad_map')

    param_file = os.path.join(ros2_rad_map_share, 'params', 'heatmap.yaml')
    model_file_path = os.path.join(ros2_rad_map_share, 'models', 'gp_model.pkl')

    gp_prediction_server_node = Node(
        package='ros2_rad_map',
        executable='gp_predictor_node.py',
        name='gp_predictor_node',
        output='screen',
        parameters=[{'model_path': model_file_path}, param_file]
    )

    return LaunchDescription([
        gp_prediction_server_node
    ])
