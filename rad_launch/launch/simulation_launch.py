from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    num_random_sources = LaunchConfiguration('num_random_sources_to_spawn', default='3')

    # Path to the unified parameter file
    param_file = os.path.join(
        get_package_share_directory('rad_simulation'),
        'params',
        'sensor_simulator.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_random_sources_to_spawn',
            default_value='3',
            description='Number of random radiation sources to spawn via service call.'
        ),
        Node(
            package='rad_simulation',
            executable='rad_field_server',
            name='rad_field_server',
            output='screen',
            emulate_tty=True,
            parameters=[param_file] # Load all parameters from the YAML file
        ),
        Node(
            package='rad_simulation',
            executable='rad_sensor_simulator',
            name='rad_sensor_simulator',
            output='screen',
            emulate_tty=True,
            parameters=[param_file] # Load all parameters from the YAML file
        ),
        Node(
            package='rad_simulation',
            executable='source_selector_node',
            name='source_selector_node',
            output='screen',
            parameters=[param_file]
        ),
    ])
