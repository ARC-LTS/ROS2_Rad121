from launch import LaunchDescription
from launch_ros.actions import Node
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

    output_dir = os.path.join(
        get_package_share_directory('rad_evaluation'),
        'eval_reports'
    )

    return LaunchDescription([
        test_case_arg,
        Node(
            package='rad_evaluation',
            executable='rad_evaluator_node',
            name='rad_evaluator_node',
            output='screen',
            parameters=[{
                'test_case_name': LaunchConfiguration('test_case'),
                'critical_dose_mrem_hr': 2.0,
                'output_dir': output_dir
            }]
        )
    ])
