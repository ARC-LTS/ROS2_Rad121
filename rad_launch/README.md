# rad_launch

Top-level launch orchestration for the RaMonES stack, plus a system-reset helper node. This is the package you launch to bring up a full simulation trial.

## Overview

`rad_launch` wires together every other package into runnable bring-ups. The main
entry point, `rad_launch_sim_launch.py`, sequences a full closed-loop simulation
trial: Gazebo → radiation simulation → SLAM → Nav2 → the GP mapping/analysis stack
→ frontier exploration and the recovery/safety layers, each on a staggered timer so
dependencies are up before dependents start. It also exposes the experiment knobs —
the **arm selector** (`radiation_agnostic`), the GP fidelity mode (`gp_mode`), the
stuck-escape fix (`escape_distance_m`), the dose-budget relaxation sweep
(`dose_relaxation_k`), and the frontier-costing sweep arguments — as launch
arguments.

The package is `ament_python`. It installs all `launch/*.py` files and one console
script, `system_reset_node`.

> **Arm selection in one line.** `radiation_agnostic:=false` (default) = **RaMonES**
> arm (radiation costmap layer + radiation-informed frontier costing + local
> recovery active). `radiation_agnostic:=true` = **Baseline** arm (frontier
> radiation weights zeroed and local-recovery thresholds set unreachable at
> construction). The hard safety brake (`rad_safety`) stays active in **both** arms
> unless the separate `safety_agnostic:=true` ablation is set.

---

## Launch files

### `rad_launch_sim_launch.py` — main simulation bring-up
Sequences the whole stack on `TimerAction` delays:

| t (s) | Starts | Notes |
| --- | --- | --- |
| 0 | `clearpath_gz/simulation.launch.py` | Gazebo + Husky; passes `headless`, `world`. |
| +5 | `simulation_launch.py` | `rad_field_server`, `rad_sensor_simulator`, `source_selector_node`. |
| +10 | `clearpath_nav2_demos/slam.launch.py` | SLAM (`output:=log`). |
| +15 | `clearpath_nav2_demos/nav2.launch.py` | Navigation. |
| +20 | `gp_predictor_launch.py` (passes `gp_mode`), `rad_heatmap_launch.py` (passes `known_field_mode`), `rad_costmap_visualizer_launch.py`, `peak_finder_launch.py` | GP + heatmap + analysis. |
| +25 | `explore_lite/explore.launch.py` (+ frontier-costing args), `rad_local_recovery_launch.py`, `rad_safety_launch.py`, `stuck_escape_node` (inline), optional `zone_explorer_node`/`zone_publisher_node` | Exploration + recovery + safety. |
| +30 | `system_reset_node` | Resets SLAM/costmaps/heatmap/battery/explore once everything is up. |

`stuck_escape_node` is launched inline here (not via `rad_local_recovery`'s launch)
with `odom_topic:=/odom`, `use_sim_time:=true`, and the escape/idle/spin args wired
through. The ground-truth pose bridge (`ros_gz_bridge parameter_bridge`) is launched
only when `record_ground_truth:=true`, bridging `/model/robot/pose` → `/ground_truth/tf`.

**Arguments** (all with their defaults):
| Argument | Default | Description |
| --- | --- | --- |
| `orbit_surveys` | `false` | Launch `zone_explorer_node` + `zone_publisher_node` so discovered zones get an orbital survey. |
| `headless` | `false` | Gazebo server-only (no GUI) for batch/headless trials. |
| `world` | `warehouse_barricaded` | Gazebo world (e.g. `rectangle`, `warehouse_barricaded`); passed to `clearpath_gz`. |
| `record_ground_truth` | `false` | Bridge the robot's true PosePublisher to `/ground_truth/tf` for the H3 source-approach metric. |
| `radiation_agnostic` | `false` | **Arm selector.** `true` = Baseline: zero `explore_node`'s radiation cost weights and set `local_recovery_node`'s dose thresholds unreachable (both at construction — must be set here, not via runtime `ros2 param set`). |
| `gp_mode` | `argp` | GP fidelity: `argp` (UFTR-trained low-fidelity prior pickle) or `single_fidelity` (flat background prior, no pickle). Passed to `gp_predictor_launch.py`. |
| `dose_relaxation_k` | `0.0` | Dose-budget-aware relaxation coefficient shared by `explore_node`, `local_recovery_node`, and `rad_safety_monitor_node`. 0 = fixed thresholds. |
| `frontier_spend_fraction` | `0.05` | Fraction of remaining dose budget one frontier round-trip may spend (explore target-gate transit cap). |
| `radiation_seek_scale` | `0.0` | Information-gain attraction bonus toward elevated-but-unconfirmed regions in frontier costing. 0 = avoidance-only. |
| `radiation_seek_adaptive` | `false` | Ramp `radiation_seek_scale` by progress deficit instead of a flat constant. |
| `radiation_survey_dirs` | `8` | Boundary-survey ray count for RADIATION-mode source characterization. |
| `radiation_survey_time_cap_s` | `0.0` | Cap total boundary-survey wall-clock time. 0 = no cap. |
| `radiation_seek_trap_avoid_radius_m` | `0.0` | Deny the seek bonus within this radius of a recent stuck-escape location. 0 = disabled. |
| `blacklist_ttl_s` | `0.0` | Seconds after which a blacklisted frontier becomes eligible again. 0 = permanent. |
| `safety_agnostic` | `false` | Exploratory ablation only: also disable `rad_safety_monitor_node`'s hard brake. Distinct from `radiation_agnostic`; keep `false` for confirmatory H1–H4 trials. |
| `retreat_dose_mrem_hr` | `2.0` | **Deprecated, no effect.** Kept only so existing callers (`run_trial.py`) don't break. The hard-brake trigger is `acute_dose_rate_mrem_hr` (10.0) in `rad_local_recovery/params/local_recovery.yaml`. |
| `escape_distance_m` | `0.0` | `stuck_escape_node` escape-distance fix. 0 = legacy time-based 0.25 m; positive = distance-based backup that clears the inflation band. |
| `idle_stall_timeout_s` | `0.0` | Report a robot stationary with no nav goal executing for this long on `/stuck_escape/idle_stall`. 0 = disabled. |
| `spin_target_yaw_rad` | `3.14159` | Reorientation angle at the end of a stuck escape (couples to `explore_node`'s rear-hemisphere frontier penalty). |
| `max_recovery_episodes` | `0` | Cap on dose-triggered recovery episodes per mission. 0 = unbounded. |
| `known_field_mode` | `false` | Known-field oracle: heatmap publishes the analytic true field instead of the GP estimate. Diagnostic ablation for the H1/H2 null. |

```bash
# RaMonES arm, default world, with GUI
ros2 launch rad_launch rad_launch_sim_launch.py
# Baseline arm, headless, rectangle world, with the stuck-escape fix enabled
ros2 launch rad_launch rad_launch_sim_launch.py radiation_agnostic:=true headless:=true world:=rectangle escape_distance_m:=0.8
```

### `simulation_launch.py` — radiation simulation only
Launches `rad_field_server`, `rad_sensor_simulator`, and `source_selector_node`
(all from `rad_simulation`), each loading `rad_simulation/params/sensor_simulator.yaml`.
| Argument | Default | Description |
| --- | --- | --- |
| `num_random_sources_to_spawn` | `3` | Number of random sources to spawn via service (declared; a battery launch is commented out). |

### `gp_predictor_launch.py` — GP prediction service
Launches `ros2_rad_map/gp_predictor_node.py` with `ros2_rad_map/params/heatmap.yaml`;
resolves the model pickle based on `gp_mode`.
| Argument | Default | Description |
| --- | --- | --- |
| `gp_mode` | `argp` | `argp` loads `models/gp_model.pkl`; `single_fidelity` passes an empty `model_path` (flat background prior). |

### `rad_heatmap_launch.py` — heatmap generator
Launches `ros2_rad_map/rad_heatmap_node` with `heatmap.yaml`.
| Argument | Default | Description |
| --- | --- | --- |
| `known_field_mode` | `false` | Publish the analytic true field instead of the GP estimate (bool, passed as `known_field_mode` param). |

### `rad_local_recovery_launch.py` — dose-triggered recovery node
Launches `rad_local_recovery/local_recovery_node` with
`rad_local_recovery/params/local_recovery.yaml`, `use_sim_time:=true`.
| Argument | Default | Description |
| --- | --- | --- |
| `radiation_agnostic` | `false` | `true` sets `acute_dose_rate_mrem_hr` and `safe_dose_threshold_mrem_hr` to 1e6 (disables recovery). |
| `dose_relaxation_k` | `0.0` | → `dose_budget_relaxation_k`. |
| `max_recovery_episodes` | `0` | → `max_recovery_episodes` (0 = unbounded). |

### `rad_safety_launch.py` — hard emergency brake
Launches `rad_safety/rad_safety_monitor_node` with `rad_safety/params/rad_safety.yaml`,
`use_sim_time:=true`.
| Argument | Default | Description |
| --- | --- | --- |
| `dose_relaxation_k` | `0.0` | → `dose_budget_relaxation_k`. |
| `safety_agnostic` | `false` | `true` sets `emergency_threshold_mrem_hr` to 1e6 (disables the brake). |

### `rad121_monitor_launch.py` — hardware sensor driver
Launches `rad121_monitor/rad121_monitor_node` with `sensor.yaml`. No arguments.

### `peak_finder_launch.py` — hotspot finder client
Launches `ros2_rad_map/peak_finder_client_node` with `heatmap.yaml`. No arguments.

### `rad_costmap_visualizer_launch.py` — radiation costmap visualizer
Launches `ros2_rad_map/rad_costmap_visualizer_node` with `heatmap.yaml`. No arguments.

### `gp_visualizer_launch.py` — GP surface visualizer
Launches `ros2_rad_map/gp_visualizer_node` with `update_period_s:=10.0`,
`cloud_resolution_m:=0.25`. No arguments.

### `rad_points_launch.py` — radiation point cloud
Launches `ros2_rad_map/rad_pointcloud_node` with `pointcloud.yaml`. No arguments.

### `rad_launch.py` — hardware/Spot master (legacy)
Includes `simulation_launch.py`, `gp_predictor_launch.py`, `rad_heatmap_launch.py`,
`rad_local_recovery_launch.py`, `rad_costmap_visualizer_launch.py`, and
`spot_robot/nav2.launch.py`. No arguments. Oriented at a Spot/hardware bring-up
rather than the Gazebo Husky trial; superseded by `rad_launch_sim_launch.py`.

### `rad_zoning_launch.py` — peak finder (duplicate)
Launches `ros2_rad_map/peak_finder_client_node` with `heatmap.yaml` (same as
`peak_finder_launch.py`). No arguments.

### Stale / disabled launch files
These reference executables that are **not built in the current `ros2_rad_map`
CMakeLists**, or have their main actions commented out; treat as non-functional
until repaired:
- `rad_zones_selector_launch.py` — references `zone_selector_node` (not built).
- `static_rad_heatmap_launch.py` — references `static_gp_predictor_node.py` and `static_rad_heatmap_node` (not built).
- `rad_cuda_launch.py` — a timed bring-up whose returned `LaunchDescription` only enables `gp_stack` + `heatmap_stack`; sim/nav/explore/reset actions are commented out. (Comment notes there is no CUDA code path — GPy and OpenCV both run on CPU.)
- `rad_spot_launch.py` — Spot variant; same pattern, only `gp_stack` + `heatmap_stack` active in the returned description.

---

## Nodes (in this package)

### `system_reset_node`
Source `rad_launch/system_reset_node.py`; entry point `system_reset_node`.
A `MultiThreadedExecutor` helper that resets mission state between trials by calling
out to other subsystems' reset services. Uses `threading.Event` (not nested spins)
to block safely inside callbacks.

**Service servers** (all `std_srvs/Trigger`): `~/reset_slam`, `~/reset_costmaps`,
`~/reset_battery`, `~/reset_heatmap`, `~/reset_explore`, `~/reset_all`.

**Service clients it calls**:
- `/rtabmap/reset` (`std_srvs/Empty`) — SLAM.
- `/global_costmap/clear_entirely_global_costmap`, `/local_costmap/clear_entirely_local_costmap` (`nav2_msgs/ClearEntireCostmap`).
- `/battery_simulator/reset` (`std_srvs/Trigger`).
- `/reset_rad_heatmap` (`std_srvs/Trigger`) — the heatmap node's reset.
- `/explore_node/reset` (`std_srvs/Trigger`).

`~/reset_all` runs explore → slam → costmaps → heatmap → battery in order and
reports a per-subsystem ok/FAIL summary.

## Config files
None of its own. Each launch file pulls params from the owning package's `share`
directory (`heatmap.yaml`, `sensor_simulator.yaml`, `local_recovery.yaml`,
`rad_safety.yaml`, `sensor.yaml`, `pointcloud.yaml`).

## Dependencies
From `package.xml`: `launch`, `launch_ros`, `ament_index_python`, `rclpy`,
`std_srvs`, `nav2_msgs`, `rad121_monitor`, `ros2_rad_map`, `rad_simulation`,
`rad_local_recovery`, `explore_lite`, `spot_robot`. Build type `ament_python`.
`explore_lite`, `spot_robot`, `clearpath_gz`, and `clearpath_nav2_demos` are
external packages the launches include; `explore_lite` (the radiation-aware fork)
comes in via `radmon.repos`. (`package.xml` `description`/`license` are still `TODO`
placeholders — harmless for build.)

## Build & run
```bash
colcon build --packages-select rad_launch
source install/setup.bash
ros2 launch rad_launch rad_launch_sim_launch.py            # full RaMonES simulation trial
ros2 run rad_launch system_reset_node                      # reset helper (usually auto-started)
ros2 service call /system_reset_node/reset_all std_srvs/srv/Trigger "{}"
```
