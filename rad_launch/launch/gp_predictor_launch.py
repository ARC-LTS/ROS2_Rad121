from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    """Launch the GP predictor node as a standalone service."""

    ros2_rad_map_share = get_package_share_directory('ros2_rad_map')

    param_file = os.path.join(ros2_rad_map_share, 'params', 'heatmap.yaml')

    # gp_mode selects the GP Fidelity Mode factor of the test matrix (chap5
    # sec:ch5_test_matrix): 'argp' loads the UFTR-trained low-fidelity prior
    # pickle as-is; 'single_fidelity' passes an empty model_path so the node
    # falls back to a flat background prior, per Chapter 3's single-fidelity
    # fallback path.
    gp_mode = LaunchConfiguration('gp_mode').perform(context)
    if gp_mode == 'single_fidelity':
        model_file_path = ''
    else:
        model_file_path = os.path.join(ros2_rad_map_share, 'models', 'gp_model.pkl')

    gp_prediction_server_node = Node(
        package='ros2_rad_map',
        executable='gp_predictor_node.py',
        name='gp_predictor_node',
        output='screen',
        # param_file first: heatmap.yaml's own 'model_path: ""' placeholder must be
        # overridden by the resolved path below, and ROS2 applies later entries
        # in this list on top of earlier ones for overlapping keys.
        parameters=[param_file, {'model_path': model_file_path}]
    )

    return [gp_prediction_server_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gp_mode',
            default_value='argp',
            description="'argp' (default, UFTR-trained low-fidelity prior) or "
                        "'single_fidelity' (flat background prior, no pickle loaded).",
        ),
        OpaqueFunction(function=launch_setup),
    ])
