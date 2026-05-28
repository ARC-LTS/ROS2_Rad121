import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rad_nav_behavior')
    default_zones_file = os.path.join(pkg_share, 'params', 'known_sources.yaml')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='exploration',
        description='"exploration" uses frontier navigation to find sources; '
                    '"surveillance" patrols known source locations read from zones_file.',
    )
    zones_file_arg = DeclareLaunchArgument(
        'zones_file',
        default_value=default_zones_file,
        description='Path to a YAML file listing known radiation source locations '
                    '(surveillance mode only). Format: zones: [{label, x, y}, ...]',
    )

    mode       = LaunchConfiguration('mode')
    zones_file = LaunchConfiguration('zones_file')

    is_surveillance = PythonExpression(["'", mode, "' == 'surveillance'"])

    # --- Exploration mode ---
    # Frontier navigation finds sources; zone_explorer pauses/resumes explore_lite
    # around each orbital survey automatically.
    exploration_node = Node(
        package='rad_nav_behavior',
        executable='zone_explorer_node',
        name='zone_explorer_node',
        output='screen',
        parameters=[{
            'mode': 'exploration',
            'zones_topic': '/radiation_zones',
            'survey_radius_m': 1.5,
            'survey_waypoints': 6,
            'dwell_per_waypoint_s': 2,
            'explore_start_service': 'explore_node/start',
            'explore_stop_service': 'explore_node/stop',
        }],
        condition=UnlessCondition(is_surveillance),
    )

    # --- Surveillance mode ---
    # zone_publisher_node reads the YAML and publishes source locations as a
    # RadZoneArray topic.  zone_explorer_node subscribes to that same topic and
    # orbits each source without entering the hotspot center.
    zone_publisher_node = Node(
        package='rad_nav_behavior',
        executable='zone_publisher_node.py',
        name='zone_publisher_node',
        output='screen',
        parameters=[{
            'zones_file': zones_file,
            'zones_topic': '/radiation_zones',
            'publish_rate_hz': 1.0,
        }],
        condition=IfCondition(is_surveillance),
    )

    surveillance_node = Node(
        package='rad_nav_behavior',
        executable='zone_explorer_node',
        name='zone_explorer_node',
        output='screen',
        parameters=[{
            'mode': 'surveillance',
            'zones_topic': '/radiation_zones',
            'survey_radius_m': 1.5,
            'survey_waypoints': 6,
            'dwell_per_waypoint_s': 2,
        }],
        condition=IfCondition(is_surveillance),
    )

    return LaunchDescription([
        mode_arg,
        zones_file_arg,
        exploration_node,
        zone_publisher_node,
        surveillance_node,
    ])
