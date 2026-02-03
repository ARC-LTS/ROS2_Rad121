import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Master launch file that starts the radiation simulation and the mapping/analysis pipeline.
    """
    rad_launch_share = get_package_share_directory('rad_launch')

    # --- 1. Include Simulation Launch ---
    # This launches rad_field_server and rad_sensor_simulator.
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rad_launch_share, 'launch', 'simulation_launch.py')
        )
    )

    # --- 2. Include GP Predictor Launch ---
    # This launches the core GP service node.
    gp_predictor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rad_launch_share, 'launch', 'gp_predictor_launch.py')
        )
    )

    # --- 3. Include Heatmap Launch ---
    # This launches the client node that generates heatmap images.
    rad_heatmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rad_launch_share, 'launch', 'rad_heatmap_launch.py')
        )
    )

    rad_zoning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rad_launch_share, 'launch', 'rad_zoning_launch.py')
        )
    )

    rad_safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rad_launch_share, 'launch', 'rad_safety_launch.py')
        )
    )

    # --- Return the master launch description ---
    return LaunchDescription([
        simulation_launch,
        gp_predictor_launch,
        rad_heatmap_launch,
        rad_zoning_launch,
        rad_safety_launch,
    ])
