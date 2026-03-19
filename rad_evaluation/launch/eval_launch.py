from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    test_case_arg = DeclareLaunchArgument(
        'test_case',
        default_value='baseline',
        description='Name of the test case for the report'
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
                'output_dir': '/home/brendoncintas/radmon_ws/data/eval_reports'
            }]
        )
    ])
