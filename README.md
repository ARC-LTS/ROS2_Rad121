# Radiation Monitoring and Exploration System (RaMonES)

This repository contains a ROS 2 based system for autonomous robot exploration in radiation-contaminated environments. The system integrates radiation sensing, spatial modeling using Gaussian Process (GP) regression, and radiation-aware navigation and exploration.

## 🚀 Overview

RaMonES is a modular, comprehensive radiation monitoring system that enables any mobile robot (e.g., Clearpath Husky, Boston Dynamics Spot) to autonomously map an unknown area while minimizing their radiation dose. It uses a combination of Frontier Exploration and a custom Radiation Costmap Layer to guide the robot through safe paths and avoid high-intensity radiation zones.

## 📂 Project Structure

| Package | Description |
| --- | --- |
| `rad_simulation` | Simulates radiation sources (point sources and spatial background) and sensor readings, published to the `\rad` topic. |
| `ros2_rad_map` | Generates radiation heatmaps using Gaussian Process (GP) regression for smooth spatial modeling. |
| `rad_costmap_plugin` | A Nav2 costmap layer that translates radiation intensity into navigation costs. |
| `rad_launch` | Centralized launch files for the entire simulation and analysis stack. |
| `rad_nav_behavior` | Zone explorer node with exploration and surveillance modes, orbital survey, and a YAML-driven zone publisher. |
| `rad_local_recovery` | Acute recovery behaviours: detects high-dose events, stops frontier navigation, navigates the robot to a safe zone, and resumes exploration once clear. |
| `rad_safety` | Monitoring nodes for cumulative radiation dose and robot safety. |
| `rad121_monitor` | Drivers and monitors for the RAD121 radiation sensor that we used for this project. Not needed if you intend on using another sensor instead. |

## ✨ Key Features

### 1. Radiation Simulation
*   **Spatial Background Field:** Implements a spatially-fixed, randomized background radiation field (0.05 - 1.0 mrem/hr) to replace high-noise temporal fluctuations.
*   **Point Source Modeling:** Accurately simulates multiple radiation sources with a point source (e.g. $1/r^2$) intensity decay.

### 2. Gaussian Process (GP) Heatmapping
*   **Smooth Interpolation:** Uses GP regression to predict radiation levels in unvisited areas, filling gaps in the sensor data.
*   **Uncertainty-Aware Masking:** Dynamically masks the radiation field based on model confidence, allowing the robot to "trust" the model in reachable free space.

### 3. Autoregressive Gaussian Process (ARGP) System
The system features an **Autoregressive Gaussian Process (ARGP)** node that manages multi-fidelity spatial modeling:
*   **Linear Autoregressive Fusion:** Implements an autoregressive formulation where high-fidelity predictions are modeled as a scaled version of the low-fidelity prior plus a residual Gaussian Process: $f_h(x) = \rho \cdot f_l(x) + \delta(x)$.
*   **Dynamic Scaling ($\rho$):** The node learns the optimal scaling factor ($\rho$) in real-time to align the low-fidelity prior with high-fidelity sensor observations.
*   **Sparse GP Transition:** Automatically switches from standard GP to Sparse GP (Inducing Points) when the sample count exceeds a threshold (default 300), maintaining $O(NM^2)$ complexity for real-time performance.
*   **Landmark Preservation:** Uses a specialized spatial buffer to protect high-intensity "landmarks" (spikes) from being smoothed out by downsampling or GP averaging.
*   **Active Peak Finding:** Provides a service to find radiation "hotspots" by performing non-linear optimization (L-BFGS-B) on the ARGP mean surface.

### 4. Radiation-Aware Costmap
*   **Bilinear Interpolation:** Smoothly maps the radiation heatmap onto the Nav2 costmap, avoiding "voxelized" navigation paths.
*   **Performance Optimized:** Uses localized update bounds (10m radius around the robot) to maintain a high control rate (10-20 Hz) without CPU bottlenecks.

### 5. Autonomous Exploration
*   **Frontier-Based Navigation:** Automatically identifies and explores unknown regions.
*   **Loop-Closure Stability:** Features a proximity-based goal validation system to prevent "target switching" loops as SLAM clears the robot's immediate path.
*   **Dose-Aware Planning:** Prioritizes lower-radiation paths and blacklists "lethal" radiation zones.

### 6. Zone Explorer & Orbital Survey (`rad_nav_behavior`)

The `zone_explorer_node` provides two distinct operating modes and can be switched at runtime via service calls.

#### Exploration Mode
The robot uses frontier navigation to map the environment. When the GP pipeline identifies a radiation source and publishes it to `/radiation_zones`, the zone explorer pauses frontier navigation, navigates to the source location, performs a configurable orbital survey around it (facing inward at each waypoint), then resumes frontier exploration when the queue is empty.

#### Surveillance Mode
The robot patrols a set of known source locations loaded from a YAML file. A companion `zone_publisher_node` reads the file and continuously publishes the locations as a `RadZoneArray` on `/radiation_zones`. The zone explorer subscribes to the same topic and orbits each source **without navigating to the hotspot center**, relying on the radiation costmap layer to route between waypoints safely.

The YAML format for known sources (`rad_nav_behavior/params/known_sources.yaml`):
```yaml
zones:
  - label: "Source_A"
    x: -4.17
    y: -12.87
  - label: "Source_B"
    x: -10.93
    y: -15.54
```

## 🛠️ Installation

1.  **Install vcstool** if not already present:
    ```bash
    sudo apt install python3-vcstool
    ```

2.  **Clone the repository** into your ROS 2 workspace:
    ```bash
    mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
    git clone https://git.turingnet.work/doobs/ROS2_Rad121.git
    ```

3.  **Pull required source dependencies** (including `explore-ros2`):
    ```bash
    vcs import --input ROS2_Rad121/radmon.repos
    ```
    This clones all declared source dependencies (e.g. `explore-ros2`) into the workspace alongside `ROS2_Rad121`.

4.  **Install ROS dependencies**:
    ```bash
    cd ~/ros_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

5.  **Build and source the workspace**:
    ```bash
    colcon build
    source ~/ros_ws/install/setup.bash
    ```

## 🏃 Usage

### Running the Full Simulation
To launch the complete Gazebo simulation with SLAM, Nav2, and the Radiation Stack:
```bash
ros2 launch rad_launch rad_launch_sim_launch.py
```
This launch script handles the sequential startup of Gazebo, SLAM, Navigation, and the Radiation Analysis nodes with appropriate delays.

#### Enabling Orbital Surveys
To additionally launch the zone explorer in exploration mode (pauses/resumes frontier nav around each orbital survey):
```bash
ros2 launch rad_launch rad_launch_sim_launch.py orbit_surveys:=true
```

### Launching the Zone Explorer Standalone

**Exploration mode** (frontier navigation + orbital survey of discovered zones):
```bash
ros2 launch rad_nav_behavior zone_explorer.launch.py mode:=exploration
```

**Surveillance mode** (patrol known source locations from a YAML file):
```bash
ros2 launch rad_nav_behavior zone_explorer.launch.py \
  mode:=surveillance \
  zones_file:=/path/to/your/sources.yaml
```
The `zone_publisher_node` is launched automatically alongside the zone explorer in surveillance mode.

### Manual Source Placement
You can manually place radiation sources in the environment using RViz2:
```bash
ros2 run rad_simulation source_selector_node
```
Use the **"Publish Point"** tool in RViz to click on the map and spawn sources in real-time.

## 🎛️ Runtime Service Calls

While the zone explorer node is running, its behavior can be changed at runtime without restarting.

### Switch Operating Mode
```bash
# Switch to exploration mode (starts frontier navigation)
ros2 service call /zone_explorer_node/exploration_mode std_srvs/srv/Trigger "{}"

# Switch to surveillance mode (stops frontier navigation)
ros2 service call /zone_explorer_node/surveillance_mode std_srvs/srv/Trigger "{}"
```
Any survey already in progress completes before the new mode's behavior takes effect.

### Trigger an Orbital Survey On Demand
Queue an orbital survey of any map-frame position immediately, regardless of the current mode:
```bash
ros2 service call /zone_explorer_node/orbit_zone ros2_rad_map/srv/OrbitZone \
  "{label: 'Source_A', x: -4.17, y: -12.87}"
```
The `label` field is optional — if omitted, a timestamped name is generated automatically. The requested zone is placed at the front of the queue and surveyed next.

## 📝 Configuration

Key parameters (thresholds, scales, topics) can be found in the following locations:
*   **Radiation Source Config:** `rad_simulation/params/sensor_simulator.yaml`
*   **Heatmap/GP Params:** `ros2_rad_map/params/heatmap.yaml`
*   **Exploration Config:** `src/explore-ros2/explore/config/params.yaml`
*   **Known Sources (Surveillance):** `rad_nav_behavior/params/known_sources.yaml`
*   **Zone Explorer Params:** configured inline in `rad_nav_behavior/launch/zone_explorer.launch.py`

| Parameter | Default | Description |
| --- | --- | --- |
| `mode` | `exploration` | Starting mode: `exploration` or `surveillance` |
| `survey_radius_m` | `1.5` | Orbit radius around each source (metres) |
| `survey_waypoints` | `6` | Number of equally-spaced orbit points |
| `dwell_per_waypoint_s` | `2` | Seconds to pause at each orbit waypoint |
| `zones_topic` | `/radiation_zones` | Topic to receive `RadZoneArray` zone updates |
| `explore_start_service` | `explore_node/start` | Service to resume frontier exploration |
| `explore_stop_service` | `explore_node/stop` | Service to pause frontier exploration |

---
*Made with <3 by Brendon (Doobs) in Miami, FL*
