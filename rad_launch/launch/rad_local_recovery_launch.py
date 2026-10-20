from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    param_file = os.path.join(
        get_package_share_directory('rad_local_recovery'),
        'params',
        'local_recovery.yaml'
    )

    parameters = [param_file]
    # acute_dose_rate_mrem_hr/safe_dose_threshold_mrem_hr
    # are only ever read once, at construction -- there is no on_set_parameters
    # callback, so a runtime `ros2 param set` on a running node has zero
    # effect on behavior even though it updates the parameter server's stored
    # value. This must be set here, at launch time, to actually disable
    # local recovery for a radiation-agnostic baseline arm.
    if LaunchConfiguration('radiation_agnostic').perform(context) == 'true':
        parameters.append({
            'acute_dose_rate_mrem_hr': 1000000.0,
            'safe_dose_threshold_mrem_hr': 1000000.0,
        })
    parameters.append({
        'max_recovery_episodes': int(LaunchConfiguration('max_recovery_episodes').perform(context)),
        'dose_budget_relaxation_k': float(LaunchConfiguration('dose_relaxation_k').perform(context)),
        # Sim time, matching explore_node and rad_evaluator_node: the node
        # integrates cumulative dose via its own clock, and counting wall
        # seconds against a sim-time dose rate overstates spent budget by
        # 1/RTF, making its relaxation decay faster than the other layers'.
        'use_sim_time': True,
    })

    return [Node(
        package='rad_local_recovery',
        executable='local_recovery_node',
        name='local_recovery_node',
        parameters=parameters,
        output='screen'
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'radiation_agnostic',
            default_value='false',
            description='Set dose thresholds unreachable at construction time, '
                        'disabling local recovery for a radiation-agnostic baseline arm. '
                        'Must be set at launch time (see comment at the parameters list above).',
        ),
        DeclareLaunchArgument(
            'dose_relaxation_k',
            default_value='0.0',
            description='Dose-budget-aware relaxation coefficient. 0.0 (default) reproduces '
                        'the fixed acute_dose_rate_mrem_hr exactly.',
        ),
        DeclareLaunchArgument(
            'max_recovery_episodes',
            default_value='0',
            description='Cap on distinct dose-triggered recovery episodes per mission. '
                        '0 (default) is unbounded, reproducing every trial to date. '
                        'consecutive_recovery_failures_ bounds a losing streak but not '
                        'the total, so an oscillating mission can otherwise consume the '
                        'whole wall clock in recover/resume cycles.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
