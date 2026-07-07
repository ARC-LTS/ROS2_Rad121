import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rad_launch_share = get_package_share_directory('rad_launch')
    gz_share = get_package_share_directory('clearpath_gz')
    nav_share = get_package_share_directory('clearpath_nav2_demos')
    wfd_share = get_package_share_directory('explore_lite')
    orbit_surveys_arg = DeclareLaunchArgument(
        'orbit_surveys',
        default_value='false',
        description='Enable orbital survey of discovered radiation zones. '
                    'When true, the zone_explorer_node manages explore_lite '
                    'and circles each discovered source before resuming exploration.',
    )
    orbit_surveys = LaunchConfiguration('orbit_surveys')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo server-only (no GUI process), for batch/automated trials '
                    'where no display is available.',
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse_barricaded',
        description='Gazebo world to load (e.g. rectangle, warehouse_barricaded). '
                    'Passed through to clearpath_gz.',
    )

    radiation_agnostic_arg = DeclareLaunchArgument(
        'radiation_agnostic',
        default_value='false',
        description='Realize the radiation-agnostic baseline arm: zero out '
                    'explore_node\'s radiation-informed cost weights and make '
                    'local_recovery_node\'s dose thresholds unreachable, both at '
                    'construction time. Must be set here (not via a later '
                    '`ros2 param set`) -- these nodes only read these parameters '
                    'once, at startup.',
    )

    # --- 1. Core Gazebo Simulation (Husky robot) ---
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'simulation.launch.py')
        ),
        launch_arguments=[
            ('headless', LaunchConfiguration('headless')),
            ('world', LaunchConfiguration('world')),
        ]
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
            ),
            launch_arguments={'output': 'log'}.items()
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, 'launch', 'peak_finder_launch.py')
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
                ),
                launch_arguments=[
                    ('radiation_agnostic', LaunchConfiguration('radiation_agnostic')),
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, 'launch', 'rad_local_recovery_launch.py')
                ),
                launch_arguments=[
                    ('radiation_agnostic', LaunchConfiguration('radiation_agnostic')),
                ]
            ),
            # Optional: orbital survey of discovered radiation zones.
            # Disabled by default; enable with orbit_surveys:=true.
            Node(
                package='rad_nav_behavior',
                executable='zone_explorer_node',
                name='zone_explorer_node',
                output='screen',
                parameters=[{
                    'mode': 'exploration',
                    'zones_topic': '/radiation_zones',
                    'survey_radius_m': 1.5,
                    'survey_waypoints': 6,
                    'dwell_per_waypoint_s': 2,
                    'explore_start_service': 'explore_node/start',
                    'explore_stop_service': 'explore_node/stop',
                }],
                condition=IfCondition(orbit_surveys),
            ),
            # zone_publisher_node always launches with zone_explorer_node so
            # sync_from_field_server is available regardless of operating mode.
            Node(
                package='rad_nav_behavior',
                executable='zone_publisher_node.py',
                name='zone_publisher_node',
                output='screen',
                parameters=[{
                    'zones_file': 'none',
                    'zones_topic': '/radiation_zones',
                    'publish_rate_hz': 1.0,
                    'saved_sources_file': os.path.join(
                        get_package_share_directory('rad_simulation'),
                        'params', 'saved_sources.yaml'),
                }],
                condition=IfCondition(orbit_surveys),
            ),
        ]
    )

    # --- 7. System Reset Node (30s delay - after all subsystems are up) ---
    system_reset = TimerAction(
        period=30.0,
        actions=[Node(
            package='rad_launch',
            executable='system_reset_node',
            name='system_reset_node',
            output='log',
        )]
    )

    return LaunchDescription([
        orbit_surveys_arg,
        headless_arg,
        world_arg,
        radiation_agnostic_arg,
        gz_launch,
        simulation_launch,
        slam_launch,
        nav_launch,
        rad_stack,
        explore_launch,
        system_reset,
    ])
