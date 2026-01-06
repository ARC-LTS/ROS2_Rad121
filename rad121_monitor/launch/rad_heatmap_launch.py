
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rad121_monitor')

    param_file = os.path.join(pkg_share, 'params', 'heatmap.yaml')

    model_file_path = os.path.join(pkg_share, 'models', 'gp_model.pkl')

    rad_heatmap_client_node = Node(
        package='rad121_monitor',
        executable='rad_heatmap_node',
        name='rad_heatmap_node',
        parameters=[param_file],
        output='screen'
    )

    gp_prediction_server_node = Node(
        package='rad121_monitor',
        executable='gp_predictor_node.py',  The executable Python script
        name='gp_predictor_node',
        output='screen',
        parameters=[{'model_path': model_file_path}, {param_file}]  Set the model_path parameter
    )

    return LaunchDescription([
        gp_prediction_server_node,
        rad_heatmap_client_node
    ])