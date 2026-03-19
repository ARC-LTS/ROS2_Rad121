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
| `rad_nav_behavior` | Custom navigation behaviors for radiation recovery and avoidance. |
| `rad_safety` | Monitoring nodes for cumulative radiation dose and robot safety. |
| `rad121_monitor` | Drivers and monitors for the RAD121 radiation sensor. |

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

## 🛠️ Installation

1.  **Clone the repository** into your ROS 2 workspace:
    ```bash
    mkdir -p /ros_ws/src && cd ~/ros_ws/src
    git clone https://git.turingnet.work/doobs/ROS2_Rad121.git
    ```

2.  **Install dependencies**:
    ```bash
    cd ~/your_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build and source the workspace**:
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

### Manual Source Placement
You can manually place radiation sources in the environment using RViz2:
```bash
ros2 run rad_simulation source_selector_node
```
Use the **"Publish Point"** tool in RViz to click on the map and spawn sources in real-time.

## 📝 Configuration

Key parameters (thresholds, scales, topics) can be found in the following locations:
*   **Radiation Source Config:** `src/ROS2_Rad121/rad_simulation/params/sensor_simulator.yaml`
*   **Heatmap/GP Params:** `src/ROS2_Rad121/ros2_rad_map/params/heatmap.yaml`
*   **Exploration Config:** `src/explore-ros2/explore/config/params.yaml`

---
*Made with <3 by Brendon (Doobs) in Miami, FL*
