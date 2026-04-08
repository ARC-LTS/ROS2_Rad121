import os
import yaml
import subprocess
import launch_ros
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def get_excluded_topics():
    """Load excluded topics from the package config file."""
    config_path = os.path.join(
        get_package_share_directory("rad_evaluation"),
        "config",
        "rosbag_excluded_topics.yaml"
    )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return set(config.get("excluded_topics") or [])

def get_included_topics():
    """Fetch all ROS 2 topics and exclude specific ones."""
    excluded_topics = get_excluded_topics()

    try:
        # Get list of all available topics
        result = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, check=True)
        all_topics = result.stdout.splitlines()

        included_topics = []
        for topic in all_topics:
            # 1. Skip explicitly excluded topics
            if topic in excluded_topics:
                continue

            # 2. Safest approach: filter out ALL auto-generated image_transport topics.
            # This completely prevents format mismatch errors and saves CPU/disk space.
            if topic.endswith(('/compressed', '/compressedDepth', '/theora')):
                continue

            included_topics.append(topic)

        if not included_topics:
            raise RuntimeError("No topics left after exclusion!")

        return included_topics
    except subprocess.CalledProcessError as e:
        print(f"Error fetching topics: {e}")
        return []

def generate_launch_description():
    topics_to_record = get_included_topics()

    if not topics_to_record:
        print("No topics available to record.")
        return LaunchDescription()

    return LaunchDescription([
        ExecuteProcess(
            cmd=["ros2", "bag", "record"] + topics_to_record,
            output="screen"
        )
    ])
