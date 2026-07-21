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

    gp_mode_arg = DeclareLaunchArgument(
        'gp_mode',
        default_value='argp',
        description="GP Fidelity Mode factor: 'argp' (default) or 'single_fidelity'. "
                    'Passed through to gp_predictor_launch.py.',
    )

    dose_relaxation_k_arg = DeclareLaunchArgument(
        'dose_relaxation_k',
        default_value='0.0',
        description='Dose-budget-aware relaxation coefficient for local_recovery_node\'s '
                    'acute_dose_rate_mrem_hr and rad_safety_monitor_node\'s '
                    'emergency_threshold_mrem_hr. 0.0 (default) reproduces the fixed '
                    'thresholds exactly. Must be set here, at launch time, for the same '
                    'reason radiation_agnostic must be.',
    )

    frontier_spend_fraction_arg = DeclareLaunchArgument(
        'frontier_spend_fraction',
        default_value='0.05',
        description='Base fraction of the remaining dose budget a single frontier '
                    'round trip may spend (explore_node target-gate transit cap); '
                    'varied by the stage-5 sweep.',
    )

    radiation_seek_scale_arg = DeclareLaunchArgument(
        'radiation_seek_scale',
        default_value='0.0',
        description='Scale of the information-gain-style attraction bonus toward '
                    'elevated-but-unconfirmed regions in explore_node frontier '
                    'costing. 0.0 (default) reproduces the prior avoidance-only '
                    'behavior exactly.',
    )

    radiation_seek_adaptive_arg = DeclareLaunchArgument(
        'radiation_seek_adaptive',
        default_value='false',
        description='Ramp radiation_seek_scale by progress deficit instead of '
                    'applying it as a flat constant. false (default) reproduces '
                    'the constant-scale behavior exactly.',
    )

    radiation_survey_dirs_arg = DeclareLaunchArgument(
        'radiation_survey_dirs',
        default_value='8',
        description='Boundary-survey ray count for RADIATION-mode source '
                    'characterization. 8 (default) reproduces the original behavior.',
    )

    radiation_survey_time_cap_arg = DeclareLaunchArgument(
        'radiation_survey_time_cap_s',
        default_value='0.0',
        description='Cap total boundary-survey wall-clock time directly, '
                    'regardless of ray count visited. 0.0 (default) disables '
                    'the cap and reproduces the original behavior exactly.',
    )

    safety_agnostic_arg = DeclareLaunchArgument(
        'safety_agnostic',
        default_value='false',
        description='Exploratory ablation only: also disable rad_safety_monitor_node\'s '
                    'hard emergency brake for a fully radiation-oblivious Baseline arm. '
                    'Distinct from radiation_agnostic; must stay false for every '
                    'confirmatory H1-H4 trial.',
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
                ),
                launch_arguments=[
                    ('gp_mode', LaunchConfiguration('gp_mode')),
                ]
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
                    # explore_node's acute-dose yield threshold must relax in
                    # lockstep with the recovery nodes' triggers, or a dose
                    # band opens where explore has stopped itself and no
                    # recovery node ever fires to restart it.
                    ('dose_relaxation_k', LaunchConfiguration('dose_relaxation_k')),
                    ('frontier_spend_fraction', LaunchConfiguration('frontier_spend_fraction')),
                    ('radiation_seek_scale', LaunchConfiguration('radiation_seek_scale')),
                    ('radiation_seek_adaptive', LaunchConfiguration('radiation_seek_adaptive')),
                    ('radiation_survey_dirs', LaunchConfiguration('radiation_survey_dirs')),
                    ('radiation_survey_time_cap_s', LaunchConfiguration('radiation_survey_time_cap_s')),
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, 'launch', 'rad_local_recovery_launch.py')
                ),
                launch_arguments=[
                    ('radiation_agnostic', LaunchConfiguration('radiation_agnostic')),
                    ('dose_relaxation_k', LaunchConfiguration('dose_relaxation_k')),
                ]
            ),
            # Hard emergency-stop backstop. Deliberately NOT gated on
            # radiation_agnostic -- the test plan's design requires this to
            # stay active in both the RaMonES and Baseline arms, since it is a
            # physical brake rather than a navigational strategy, and
            # disabling it for Baseline would confound the confirmatory
            # comparison with unmodeled acute-overexposure risk. Was
            # previously defined (rad_safety_launch.py) but never included
            # here, so no simulation trial actually exercised it; added to
            # close that gap. safety_agnostic is a SEPARATE, exploratory-only
            # override (default false) for the unsafe-baseline ablation --
            # see rad_safety_launch.py.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, 'launch', 'rad_safety_launch.py')
                ),
                launch_arguments=[
                    ('dose_relaxation_k', LaunchConfiguration('dose_relaxation_k')),
                    ('safety_agnostic', LaunchConfiguration('safety_agnostic')),
                ]
            ),
            # General-purpose, radiation-independent stuck watchdog: if the
            # robot is executing a nav goal but hasn't physically moved (per
            # odometry, not just goal status) for stuck_timeout_s, cancels
            # the goal and backs the robot away directly via cmd_vel. Runs
            # regardless of arm/radiation state -- unlike local_recovery_node
            # (radiation-triggered) or explore_lite's own replanning (which
            # only changes the target, not the robot's physical position),
            # this is the backstop for a robot geometrically wedged (e.g. a
            # corner) with nothing forcing it to actually move.
            Node(
                package='rad_local_recovery',
                executable='stuck_escape_node',
                name='stuck_escape_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    # Default is /odometry, which doesn't exist in this stack
                    # (confirmed directly: node was reading a perpetually-zero
                    # position and firing false-positive "stuck" triggers
                    # every ~16s regardless of the robot's real state).
                    'odom_topic': '/odom',
                }],
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
        gp_mode_arg,
        dose_relaxation_k_arg,
        frontier_spend_fraction_arg,
        radiation_seek_scale_arg,
        radiation_seek_adaptive_arg,
        radiation_survey_dirs_arg,
        radiation_survey_time_cap_arg,
        safety_agnostic_arg,
        gz_launch,
        simulation_launch,
        slam_launch,
        nav_launch,
        rad_stack,
        explore_launch,
        system_reset,
    ])
