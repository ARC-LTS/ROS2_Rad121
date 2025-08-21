import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your package's share directory
    rad121_monitor_share_dir = get_package_share_directory('rad121_monitor')

    # --- Parameters ---
    # Define the path to your GP model pickle file.
    # It's good practice to place model files in a 'models' or 'config' directory
    # within your package, but you can specify any absolute path.
    # For this example, let's assume it's in a 'models' folder in your package.
    gp_model_path = os.path.join(rad121_monitor_share_dir, 'models', 'gp_model.pkl')

    # Define the path to your parameters file if you use one for the C++ node
    params_file = os.path.join(rad121_monitor_share_dir, 'params', 'heatmap.yaml')


    # --- Nodes ---
    # 1. The Python node that serves GP predictions
    gp_predictor_node = Node(
        package='rad121_monitor',
        executable='gp_predictor_node.py',
        name='gp_predictor_node',
        output='screen',
        parameters=[
            {'model_path': gp_model_path}
        ]
    )

    # 2. The refactored C++ node that generates heatmaps
    radiation_heatmap_node = Node(
        package='rad121_monitor',
        executable='radiation_heatmap_node',
        name='radiation_heatmap_node',
        output='screen',
        # Uncomment the line below if you have a params file
        # parameters=[params_file]
    )

    return LaunchDescription([
        gp_predictor_node,
        radiation_heatmap_node
    ])