import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/',
        description='Clearpath robot.yaml directory (must end with /)'),
]

def generate_launch_description():
    rad_launch_share = get_package_share_directory('rad_launch')
    nav_share = get_package_share_directory('clearpath_nav2_demos')
    wfd_share = get_package_share_directory('explore_lite')
    setup_path = LaunchConfiguration('setup_path')

    # --- 1. Real sensor (immediate) ---
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rad_launch_share, 'launch', 'rad121_monitor_launch.py')
        )
    )

    # --- 2. Safety monitor (immediate) ---
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rad_launch_share, 'launch', 'rad_safety_launch.py')
        )
    )

    # --- 3. SLAM (5s delay - wait for sensor topics) ---
    slam_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'slam.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'output': 'log',
            }.items()
        )]
    )

    # --- 4. Navigation (10s delay - wait for SLAM map) ---
    nav_launch = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'setup_path': setup_path,
            }.items()
        )]
    )

    # --- 5. Radiation analysis stack (15s delay - wait for map/costmaps) ---
    rad_stack = TimerAction(
        period=15.0,
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

    # --- 6. Exploration + recovery (20s delay - wait for Nav2/costmaps) ---
    explore_launch = TimerAction(
        period=20.0,
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

    # --- 7. System reset node (25s delay - after all subsystems are up) ---
    system_reset = TimerAction(
        period=25.0,
        actions=[Node(
            package='rad_launch',
            executable='system_reset_node',
            name='system_reset_node',
            output='log',
        )]
    )

    return LaunchDescription(ARGUMENTS + [
        sensor_launch,
        safety_launch,
        slam_launch,
        nav_launch,
        rad_stack,
        explore_launch,
        system_reset,
    ])
