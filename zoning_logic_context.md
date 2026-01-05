# Project: Autonomous Zone-Based Swabbing System

## High-Level Goal

Create an autonomous robotic system for radiological surveillance and swabbing in a nuclear environment.

## Project Phases

(a) **Preliminary Survey:** Conduct a survey of the environment using frontier-based exploration to map radiation levels. (Completed)
(b) **Zone Definition:** Autonomously identify and define "zones" of interest based on the survey data. (Completed)
(c) **In-Zone Scanning:** Navigate to each zone and use a secondary sensor to find the optimal swabbing spot.
(d) **Swabbing:** Perform the swabbing action.
(e) **Repeat:** Continue until all zones are processed.
(f) **Docking:** Autonomously dock after the mission.

## Phase (b) Implementation: Integrated Zone Detection

The zoning logic has been implemented and integrated directly into the `rad_heatmap_node`. This approach was chosen to ensure data consistency, simplify the overall system architecture, and provide the desired annotated image visualization. The initially proposed `rad_zone_planner_node` was removed in favor of this more robust, consolidated design.

### Final Architecture Details

*   **Consolidated Node:** All logic for heatmap generation, zone detection, and visualization now resides within `rad_heatmap_node.cpp`.

*   **Input:** The logic uses the raw historical mean heatmap (`hist_mean_raw`) that is generated internally by the node after the Gaussian Process (GP) regression step.

*   **Processing (within `rad_heatmap_node`):**
    1.  A configurable `radiation_threshold` is applied to the raw heatmap to create a binary image of high-radiation areas.
    2.  To separate large, contiguous zones into smaller, more meaningful regions, a **Watershed Segmentation** algorithm is applied. The sensitivity of this separation can be adjusted via the `zone_separation_aggressiveness` ROS parameter.
    3.  `cv::findContours` is used on the resulting watershed segments to extract the precise boundaries of each new zone.
    4.  For each valid contour, its centroid is calculated to serve as a navigation waypoint.

*   **Output:**
    1.  **Annotated Heatmap Image:** The primary visualization is the `/rad_heatmap/historical_mean` topic. The node draws the found zone contours and their labels (A, B, C...) directly onto the blended color heatmap before publishing. This provides a single, intuitive visual output.
    2.  **Zone Waypoints:** The node publishes a `rad121_monitor/msg/RadZoneArray` message on the `/radiation_zones` topic. This message contains the label and the calculated center point (waypoint) for each detected zone, which can be consumed by a downstream navigation or task planning system.

*   **Integration:**
    *   All new logic is contained within `rad121_monitor/src/rad_heatmap_node.cpp`.
    *   Custom message types `RadZone.msg` and `RadZoneArray.msg` were created to support the waypoint publisher.
    *   The `CMakeLists.txt` has been updated to build these messages and the modified node.
    *   The system is launched via `rad_heatmap_launch.py`, which runs the `rad_heatmap_node` and the required `gp_predictor_node`.