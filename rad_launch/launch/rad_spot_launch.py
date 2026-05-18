import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Phase delays (seconds from launch start).
#
# The original 5-second gaps were too short: DDS discovery traffic from the
# simulation nodes and CPU contention from rad_field_server starved the
# Velodyne UDP socket, so rtabmap never got consistent lidar data.
#
# Critical gap: slam -> nav2 must be ≥30 s so rtabmap can establish stable
# odometry before nav2 adds another burst of node startups and DDS traffic.
_T_SIM   =  5   # simulation nodes: lightweight, start early
_T_SLAM  = 20   # slam/velodyne: wait for simulation DDS storm to settle
_T_NAV2  = 70   # nav2: 50 s after SLAM — extra time for RTAB-Map CPU spike to settle
_T_RAD   = 90   # radiation stack: 20 s after nav2
_T_EXPL  = 105  # exploration: 15 s after rad stack
_T_RESET = 140  # system reset: only after the full stack is stable

def generate_launch_description():
    rad_launch_share = get_package_share_directory('rad_launch')
    spot_robot_share = get_package_share_directory('spot_robot')
    wfd_share = get_package_share_directory('explore_lite')

    simulation_launch = TimerAction(
        period=float(_T_SIM),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rad_launch_share, 'launch', 'simulation_launch.py')
            )
        )]
    )

    slam_launch = TimerAction(
        period=float(_T_SLAM),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(spot_robot_share, 'launch', 'slam.launch.py')
                ),
                launch_arguments={'output': 'log'}.items()
            ),
            # Spot terrain grid → voxelized point cloud for small-obstacle detection.
            # Started with SLAM so the robot connection is already established and
            # the odom frame exists before nav2 tries to subscribe to the topic.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(spot_robot_share, 'launch', 'publish_local_grid.launch.py')
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(spot_robot_share, 'launch', 'voxelize.launch.py')
                )
            ),
            # Merge all 6 depth-camera point clouds into /cloud_full (body frame,
            # 0–3 m range). Covers the Velodyne's near-range dead zone and feeds
            # the Nav2 obstacle_layer sources configured in nav2_params.yaml.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(spot_robot_share, 'launch', 'publish_full_pointcloud.launch.py')
                )
            ),
        ]
    )

    nav_launch = TimerAction(
        period=float(_T_NAV2),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(spot_robot_share, 'launch', 'nav2.launch.py')
            )
        )]
    )

    rad_stack = TimerAction(
        period=float(_T_RAD),
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
            #IncludeLaunchDescription(
            #    PythonLaunchDescriptionSource(
            #        os.path.join(rad_launch_share, 'launch', 'rad_costmap_visualizer_launch.py')
            #    )
            #),
        ]
    )

    explore_launch = TimerAction(
        period=float(_T_EXPL),
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

    system_reset = TimerAction(
        period=float(_T_RESET),
        actions=[Node(
            package='rad_launch',
            executable='system_reset_node',
            name='system_reset_node',
            output='log',
        )]
    )

    return LaunchDescription([
        simulation_launch,
        slam_launch,
        nav_launch,
        rad_stack,
        explore_launch,
        system_reset,
    ])