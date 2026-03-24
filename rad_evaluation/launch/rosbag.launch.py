import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import subprocess

def get_included_topics():
    """Fetch all ROS 2 topics and exclude specific ones."""
    excluded_topics = {
#        "/camera/back/camera_info",
#        "/camera/back/image",
#        "/camera/frontleft/camera_info",
#        "/camera/frontleft/image",
#        "/camera/frontright/camera_info",
#        "/camera/frontright/image",
#        "/camera/left/camera_info",
#        "/camera/left/image",
#        "/camera/right/camera_info",
#        "/camera/right/image",
#        "/depth/back/camera_info",
#        "/depth/back/image",
#        "/depth/frontleft/camera_info",
#        "/depth/frontleft/image",
#        "/depth/frontright/camera_info",
#        "/depth/frontright/image",
#        "/depth/left/camera_info",
#        "/depth/left/image",
#        "/depth/right/camera_info",
#        "/depth/right/image",
#        "/depth_registered/frontleft/image",
#        "/depth_registered/image_rect/compressed",
    }

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