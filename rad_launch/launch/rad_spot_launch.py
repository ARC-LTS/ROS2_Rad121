import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription, TimerAction,
    SetEnvironmentVariable, LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
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

    cuda_env = [
        SetEnvironmentVariable("CUDA_VISIBLE_DEVICES", "0"),
        SetEnvironmentVariable("CUDA_HOME", "/usr/local/cuda"),
        SetEnvironmentVariable(
            "LD_LIBRARY_PATH",
            ["/usr/local/cuda/lib64:", EnvironmentVariable("LD_LIBRARY_PATH")],
        ),
    ]

    # Probe the GPU at launch time and print device info once.
    cuda_probe = ExecuteProcess(
        cmd=[
            "python3", "-c",
            (
                "import cupy as cp; "
                "d = cp.cuda.Device(0); "
                "props = cp.cuda.runtime.getDeviceProperties(0); "
                "name = props['name'].decode(); "
                "mem = props['totalGlobalMem'] // (1024**2); "
                "print(f'[CUDA] Device 0: {name} | {mem} MB | CUDA_VISIBLE_DEVICES=0')"
            ),
        ],
        output="screen",
    )

    simulation_launch = TimerAction(
        period=float(_T_SIM),
        actions=[
            LogInfo(msg="[CUDA] rad_field_server, rad_sensor_simulator, source_selector_node launching (env: CUDA_VISIBLE_DEVICES=0)"),
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
            LogInfo(msg="[CUDA] nav2 stack launching (env: CUDA_VISIBLE_DEVICES=0)"),
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
            LogInfo(msg="[CUDA] gp_predictor_node launching — CuPy GP acceleration enabled"),
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
            LogInfo(msg="[CUDA] rad_heatmap_node launching — cv::cuda::GpuMat kernel enabled"),
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
            LogInfo(msg="[CUDA] explore_lite + rad_local_recovery launching (env: CUDA_VISIBLE_DEVICES=0)"),
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

    return LaunchDescription(cuda_env + [
        cuda_probe,
        simulation_launch,
        nav_launch,
        gp_stack,
        heatmap_stack,
        explore_launch,
        system_reset,
    ])
