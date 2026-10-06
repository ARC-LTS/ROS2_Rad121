from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    test_case_arg = DeclareLaunchArgument(
        'test_case',
        default_value='baseline',
        description='Name of the test case for the report'
    )

    critical_dose_mrem_hr_arg = DeclareLaunchArgument(
        'critical_dose_mrem_hr',
        default_value='2.0',
        description='Dose-rate ceiling that violations/max_penetration are reported '
                    'against. 2.0 (default) is the original, deliberately conservative '
                    'test threshold used in every confirmatory trial to date -- it was '
                    'never derived from actual hardware tolerance. Raise this to report '
                    'against a less conservative ceiling; does not change any robot '
                    'behavior (local_recovery_node\'s actual retreat trigger is a '
                    'separate parameter, see rad_local_recovery_launch.py).',
    )

    output_dir = os.path.join(
        get_package_share_directory('rad_evaluation'),
        'eval_reports'
    )

    return LaunchDescription([
        test_case_arg,
        critical_dose_mrem_hr_arg,
        Node(
            package='rad_evaluation',
            executable='rad_evaluator_node',
            name='rad_evaluator_node',
            output='screen',
            parameters=[{
                'test_case_name': LaunchConfiguration('test_case'),
                'critical_dose_mrem_hr': ParameterValue(
                    LaunchConfiguration('critical_dose_mrem_hr'), value_type=float),
                'output_dir': output_dir
            }]
        )
    ])
