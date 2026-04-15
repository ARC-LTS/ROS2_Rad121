import os
import re
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def build_exclude_regex():
    """Build a single regex from rosbag_excluded_topics.yaml plus image_transport variants."""
    config_path = os.path.join(
        get_package_share_directory("rad_evaluation"),
        "config",
        "rosbag_excluded_topics.yaml"
    )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    excluded = config.get("excluded_topics") or []

    # Exact-match each excluded topic, plus catch all image_transport compressed variants
    patterns = [re.escape(t) for t in excluded]
    patterns.append(r".+/(compressed|compressedDepth|theora)$")

    return "(" + "|".join(f"^{p}$" for p in patterns) + ")"


def generate_launch_description():
    exclude_regex = build_exclude_regex()

    # The stopper node owns the recorder subprocess so it holds the PID directly.
    # It starts the recorder on init and stops it cleanly via the /rosbag/stop service.
    stopper = Node(
        package='rad_evaluation',
        executable='rosbag_stopper.py',
        name='rosbag_stopper',
        output='screen',
        parameters=[{'exclude_regex': exclude_regex}],
    )

    return LaunchDescription([stopper])
