from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for the ZoneExplorerNode.
    """
    zone_explorer_node = Node(
        package='rad_nav_behavior',
        executable='zone_explorer_node',
        name='zone_explorer_node',
        output='screen',
        parameters=[{
            'zones_topic': '/radiation_zones',
            'secondary_scan_duration_s': 5
        }]
    )

    return LaunchDescription([
        zone_explorer_node
    ])
