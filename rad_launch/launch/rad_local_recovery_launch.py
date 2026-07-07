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
    # critical_dose_mrem_hr/acute_dose_rate_mrem_hr/safe_dose_threshold_mrem_hr
    # are only ever read once, at construction -- there is no on_set_parameters
    # callback, so a runtime `ros2 param set` on a running node has zero
    # effect on behavior even though it updates the parameter server's stored
    # value. This must be set here, at launch time, to actually disable
    # local recovery for a radiation-agnostic baseline arm.
    if LaunchConfiguration('radiation_agnostic').perform(context) == 'true':
        parameters.append({
            'critical_dose_mrem_hr': 1000000.0,
            'acute_dose_rate_mrem_hr': 1000000.0,
            'safe_dose_threshold_mrem_hr': 1000000.0,
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
        OpaqueFunction(function=launch_setup),
    ])
