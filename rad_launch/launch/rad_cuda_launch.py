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
_T_HEAT  = 90   # 80 s after gp_predictor — discovery server needs time to relay service endpoint
_T_EXPL  = 105
_T_RESET = 140

def generate_launch_description():
    rad_launch_share = get_package_share_directory("rad_launch")
    
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
        simulation_launch,
        #nav_launch,
        gp_stack,
        heatmap_stack,
        #explore_launch,
        #system_reset,
    ])
