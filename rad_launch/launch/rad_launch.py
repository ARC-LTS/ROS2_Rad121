import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Master launch file that starts the radiation simulation and the heatmap generation.
    """
    # Get the share directory of the rad_launch package where the launch files were moved.
    rad_launch_share = get_package_share_directory('rad_launch')

    # --- 1. Include Simulation Launch ---
    # This will launch rad_field_server and rad_sensor_simulator from the rad_simulation package.
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rad_launch_share, 'launch', 'simulation_launch.py')
        )
    )

    # --- 2. Include Heatmap Launch ---
    # This will launch rad_heatmap_node and gp_predictor_node from the ros2_rad_map package.
    rad_heatmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rad_launch_share, 'launch', 'rad_heatmap_launch.py')
        )
    )

    # --- Return the master launch description ---
    return LaunchDescription([
        simulation_launch,
        rad_heatmap_launch,
    ])
