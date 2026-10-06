from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_rad_map')

    # Path to your parameter file for the C++ node
    param_file = os.path.join(pkg_share, 'params', 'heatmap.yaml')

    known_field_mode_arg = DeclareLaunchArgument(
        'known_field_mode',
        default_value='false',
        description='Known-field oracle: publish the analytic true radiation field '
                    'instead of the GP estimate, giving the robot perfect radiation '
                    'knowledge over the mapped extent. false (default) reproduces the '
                    'GP-estimated pipeline exactly. Diagnostic ablation for the H1/H2 '
                    'dose/safety null, not an operational mode.',
    )

    # The C++ node that requests predictions (the client)
    rad_heatmap_client_node = Node(
        package='ros2_rad_map',
        executable='rad_heatmap_node',
        name='rad_heatmap_node',
        parameters=[param_file, {
            'known_field_mode': ParameterValue(
                LaunchConfiguration('known_field_mode'), value_type=bool),
        }],
        output='screen'
    )

    return LaunchDescription([
        known_field_mode_arg,
        rad_heatmap_client_node
    ])