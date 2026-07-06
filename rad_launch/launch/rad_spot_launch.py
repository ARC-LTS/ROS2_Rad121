import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

_T_SIM   =  5
_T_RAD   = 10
_T_NAV2  = 70
_T_HEAT  = 90   # 80 s after gp_predictor — discovery server needs time to relay service endpoint
_T_EXPL  = 105
_T_RESET = 140

def generate_launch_description():
    rad_launch_share = get_package_share_directory("rad_launch")
    spot_robot_share = get_package_share_directory("spot_robot")
    wfd_share = get_package_share_directory("explore_lite")

    # NOTE: gp_predictor_node (GPy) and rad_heatmap_node (OpenCV CPU Mat) do not
    # use CUDA/CuPy/cv::cuda — both run entirely on CPU. There is no GPU codepath
    # in this bringup today, so no CUDA env/probe is set up here.

    simulation_launch = TimerAction(
        period=float(_T_SIM),
        actions=[
            LogInfo(msg="rad_field_server, rad_sensor_simulator, source_selector_node launching"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, "launch", "simulation_launch.py")
                )
            ),
        ]
    )

    nav_launch = TimerAction(
        period=float(_T_NAV2),
        actions=[
            LogInfo(msg="nav2 stack launching"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(spot_robot_share, "launch", "nav2.launch.py")
                )
            ),
        ]
    )

    gp_stack = TimerAction(
        period=float(_T_RAD),
        actions=[
            LogInfo(msg="gp_predictor_node launching (CPU GPy, warm-started hyperparameters)"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, "launch", "gp_predictor_launch.py")
                )
            ),
        ]
    )

    heatmap_stack = TimerAction(
        period=float(_T_HEAT),
        actions=[
            LogInfo(msg="rad_heatmap_node launching (CPU OpenCV)"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, "launch", "rad_heatmap_launch.py")
                )
            ),
        ]
    )

    explore_launch = TimerAction(
        period=float(_T_EXPL),
        actions=[
            LogInfo(msg="explore_lite + rad_local_recovery launching"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(wfd_share, "launch", "explore.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rad_launch_share, "launch", "rad_local_recovery_launch.py")
                )
            ),
        ]
    )

    system_reset = TimerAction(
        period=float(_T_RESET),
        actions=[Node(
            package="rad_launch",
            executable="system_reset_node",
            name="system_reset_node",
            output="log",
        )]
    )

    return LaunchDescription([
        #simulation_launch,
        #nav_launch,
        gp_stack,
        heatmap_stack,
        #explore_launch,
        #system_reset,
    ])
