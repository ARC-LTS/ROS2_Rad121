import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rad_launch_share = get_package_share_directory('rad_launch')
    gz_share = get_package_share_directory('clearpath_gz')
    nav_share = get_package_share_directory('clearpath_nav2_demos')
    wfd_share = get_package_share_directory('explore_lite')

    # --- 1. Core Gazebo Simulation (Husky robot) ---
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'simulation.launch.py')
        )
    )

    # --- 2. Radiation & Battery Simulation (5s delay) ---
    simulation_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rad_launch_share, 'launch', 'simulation_launch.py')
            )
        )]
    )

    # --- 3. SLAM (10s delay - wait for robot sensors) ---
    slam_launch = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'slam.launch.py')
            )
        )]
    )

    # --- 4. Navigation (15s delay - wait for SLAM map) ---
    nav_launch = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'nav2.launch.py')
            )
        )]
    )

    # --- 5. Radiation Analysis Stack (20s delay) ---
    rad_stack = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, 'launch', 'gp_predictor_launch.py')
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, 'launch', 'rad_heatmap_launch.py')
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, 'launch', 'rad_costmap_visualizer_launch.py')
                )
            ),
        ]
    )

    # --- 6. Exploration & Recovery (25s delay - wait for Nav2/Costmaps) ---
    explore_launch = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(wfd_share, 'launch', 'explore.launch.py')
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, 'launch', 'rad_local_recovery_launch.py')
                )
            ),
        ]
    )

    return LaunchDescription([
        gz_launch,
        simulation_launch,
        slam_launch,
        nav_launch,
        rad_stack,
        explore_launch,
    ])
