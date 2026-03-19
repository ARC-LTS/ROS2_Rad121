from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # The C++ node that generates the visualization.
    # It relies on an external 'gp_predictor_node' to be running.
    gp_visualizer_node = Node(
        package='ros2_rad_map',
        executable='gp_visualizer_node',
        name='gp_visualizer_node',
        output='screen',
        parameters=[
            {
                'update_period_s': 10.0,
                'cloud_resolution_m': 0.25,
            }
        ]
    )

    return LaunchDescription([
        gp_visualizer_node
    ])
