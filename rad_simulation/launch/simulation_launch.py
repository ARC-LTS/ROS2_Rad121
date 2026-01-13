from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rad_simulation',
            executable='rad_field_server',
            name='rad_field_server',
            output='screen',
            emulate_tty=True,
            parameters=[
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
            parameters=[
                {'map_frame': 'map'},
                {'robot_frame': 'base_link'},
                {'publish_rate_hz': 10.0},
                {'min_sensing_distance_m': 0.58}
            ]
        ),
    ])
