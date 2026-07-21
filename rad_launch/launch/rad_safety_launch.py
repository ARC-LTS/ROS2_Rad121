from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    param_file = os.path.join(
        get_package_share_directory('rad_safety'),
        'params',
        'rad_safety.yaml'
    )

    parameters = [param_file, {
        'dose_budget_relaxation_k': float(LaunchConfiguration('dose_relaxation_k').perform(context)),
        # The node integrates cumulative dose from /rad timestamps' deltas via
        # its own clock; without sim time it counts wall seconds against a
        # sim-time dose rate, overstating spent budget by 1/RTF relative to
        # explore_node and rad_evaluator_node (both sim-time).
        'use_sim_time': True,
    }]

    # Deliberately a SEPARATE flag from radiation_agnostic, which already
    # gates explore_node's frontier costing and local_recovery_node's
    # proactive escape. This node is the one radiation-reactive component
    # the test plan holds active in BOTH arms by design (see the
    # unmodeled-acute-overexposure-risk comment on its include in
    # rad_launch_sim_launch.py); disabling it here is an explicit,
    # separately-tagged exploratory ablation, not a change to what
    # radiation_agnostic means for the confirmatory H1-H4 arms.
    if LaunchConfiguration('safety_agnostic').perform(context) == 'true':
        parameters.append({'emergency_threshold_mrem_hr': 1000000.0})

    return [Node(
        package='rad_safety',
        executable='rad_safety_monitor_node',
        name='rad_safety_monitor_node',
        parameters=parameters,
        output='screen'
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dose_relaxation_k',
            default_value='0.0',
            description='Dose-budget-aware relaxation coefficient. 0.0 (default) reproduces '
                        'the fixed emergency_threshold_mrem_hr exactly.',
        ),
        DeclareLaunchArgument(
            'safety_agnostic',
            default_value='false',
            description='Exploratory ablation only: set the emergency threshold unreachable '
                        'at construction time, disabling even the hard brake. Distinct from '
                        'radiation_agnostic; must default to false so no confirmatory arm '
                        "changes behavior.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
