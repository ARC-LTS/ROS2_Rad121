from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the new parameter file
    param_file = os.path.join(
        get_package_share_directory('rad_simulation'),
        'params',
        'sensor_simulator.yaml'
    )

    return LaunchDescription([
        Node(
            package='rad_simulation',
            executable='rad_field_server',
            name='rad_field_server',
            output='screen',
            emulate_tty=True,
            parameters=[
                # These could also be moved to a yaml file if desired
                {'map_frame': 'map'},
                {'robot_frame': 'base_link'}
            ]
        ),
        Node(
            package='rad_simulation',
            executable='rad_sensor_simulator',
            name='rad_sensor_simulator',
            output='screen',
            emulate_tty=True,
            parameters=[param_file] # Load all parameters from the YAML file
        ),
    ])
