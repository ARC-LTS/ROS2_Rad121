from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import os

def generate_launch_description():
    num_random_sources = LaunchConfiguration('num_random_sources_to_spawn', default='3')

    rad_simulation_share = get_package_share_directory('rad_simulation')
    # clearpath_gz_share = get_package_share_directory('clearpath_gz')

    # Path to the unified parameter file
    param_file = os.path.join(
        rad_simulation_share,
        'params',
        'sensor_simulator.yaml'
    )

    # Where rad_field_server's save_sources/load_sources write/read the layout.
    # Empty -> the node's compiled default (<share>/params/saved_sources.yaml).
    default_saved_sources = os.path.join(rad_simulation_share, 'params', 'saved_sources.yaml')
    saved_sources_file = PythonExpression([
        "'", LaunchConfiguration('saved_sources_file'), "' or '", default_saved_sources, "'"])

    #battery_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(clearpath_gz_share, 'launch', 'battery_launch.py')
    #    ),
    #    launch_arguments={'battery_topic': '/battery_state'}.items() # Add any needed overrides
    #)

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_random_sources_to_spawn',
            default_value='3',
            description='Number of random radiation sources to spawn via service call.'
        ),
        DeclareLaunchArgument(
            'saved_sources_file',
            default_value='',
            description='Layout YAML for save_sources/load_sources (empty = share default).'
        ),
        Node(
            package='rad_simulation',
            executable='rad_field_server',
            name='rad_field_server',
            output='screen',
            emulate_tty=True,
            parameters=[param_file, {'output_yaml_file': saved_sources_file}]
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
        # battery_launch,
    ])
