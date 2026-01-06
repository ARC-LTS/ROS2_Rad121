from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rad121_monitor')

    param_file = os.path.join(pkg_share, 'params', 'heatmap.yaml')
    
    # IMPORTANT: Update this to the correct path for your .pkl file!
    model_file_path = os.path.join(pkg_share, 'models', 'gp_model.pkl')

    static_predictor_node = Node(
        package='rad121_monitor',
        executable='static_gp_predictor_node.py',
        name='static_gp_predictor_node',
        parameters=[{'model_path': model_file_path}, {'use_sim_time': True}]
    )

    static_heatmap_node = Node(
        package='rad121_monitor',
        executable='static_rad_heatmap_node',
        name='rad_heatmap_node',
        parameters=[{'use_sim_time': True}, param_file],
        output='screen'
    )

    return LaunchDescription([
        static_predictor_node,
        static_heatmap_node
    ])