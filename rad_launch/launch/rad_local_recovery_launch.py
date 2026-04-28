from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('rad_local_recovery'),
        'params',
        'local_recovery.yaml'
    )

    return LaunchDescription([
        Node(
            package='rad_local_recovery',
            executable='local_recovery_node',
            name='local_recovery_node',
            parameters=[param_file],
            output='screen'
        ),
        Node(
            package='rad_local_recovery',
            executable='stuck_escape_node',
            name='stuck_escape_node',
            output='screen'
        ),
    ])
