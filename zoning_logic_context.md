# Project: Autonomous Zone-Based Swabbing System

## High-Level Goal

Create an autonomous robotic system for radiological surveillance and swabbing in a nuclear environment.

## Project Phases

(a) **Preliminary Survey:** Conduct a survey of the environment using frontier-based exploration to map radiation levels. (Completed)
(b) **Zone Definition:** Autonomously identify and define "zones" of interest based on the survey data. (Current Task)
(c) **In-Zone Scanning:** Navigate to each zone and use a secondary sensor to find the optimal swabbing spot.
(d) **Swabbing:** Perform the swabbing action.
(e) **Repeat:** Continue until all zones are processed.
(f) **Docking:** Autonomously dock after the mission.

## Current Task: Implement Zoning Logic (Phase b)

The goal is to create a ROS2 node that processes the radiation data from the preliminary survey to autonomously define zones for swabbing.

### Existing Relevant Components

*   **`rad121_monitor` package:** The main ROS2 package for this project.
*   **`rad_mapper_node`:** Subscribes to `/rad` and `/map` to record the robot's path and where radiation was detected. This provides the raw survey data.
*   **`rad_heatmap_node` :** These nodes generate a radiation heatmap from the survey data using Gaussian Process (GP) regression. They publish the heatmap as a `sensor_msgs/Image` on `/rad_heatmap/mean_map`. This heatmap is a good input for the zoning logic.
*   **`zone_selector_node`:** Allows for manual definition of zones via RViz, which are saved to `predefined_zones.yaml`. The output format (center, radius) is a good target for the autonomous zone planner.

### Proposed Plan for Zoning Logic

1.  **Create a new ROS2 node:** `rad_zone_planner_node`.
2.  **Input:** The node will subscribe to the `/rad_heatmap/mean_map` topic (a `sensor_msgs/Image`) published by the `rad_heatmap_node`. It will also need map metadata, so subscribing to `/map` (`nav_msgs/OccupancyGrid`) is also necessary to convert pixel coordinates to world coordinates.
3.  **Processing (inside the node):**
    a.  Convert the received heatmap image to an OpenCV `cv::Mat`.
    b.  Apply a threshold to the heatmap to isolate areas with high radiation intensity. The threshold value should be a configurable parameter.
    c.  Use OpenCV's `findContours` on the thresholded image to find distinct high-radiation regions.
    d.  Filter out very small, insignificant contours.
    e.  For each valid contour, calculate its properties:
        *   Centroid (to define the zone's center).
        *   Bounding circle (to define the zone's radius).
    f.  Store these as `Zone` objects (e.g., a struct with center, radius, and an ID).
4.  **Output:**
    a.  Publish a `visualization_msgs/MarkerArray` to display the zones in RViz for visualization. Each zone will be a cylinder or a circle.
    b.  Provide the zones to other nodes. A service that returns the list of zones would be a good approach, similar to the existing services in the package. A custom service definition `GetZones.srv` might be needed. Or, it could publish to a topic.
5.  **Integration:**
    a.  Create a new source file `src/rad_zone_planner_node.cpp`.
    b.  Update `CMakeLists.txt` to build the new executable.
    c.  Create a new launch file `launch/rad_zone_planner_launch.py` to run the node.

This approach leverages the existing heatmap generation and provides a clear path to autonomously defining the zones as requested.
