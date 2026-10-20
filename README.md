# ROS2_Rad121 — RaMonES Radiation Monitoring and Exploration System

A ROS 2 Humble, closed-loop system for autonomous robot exploration of
radiation-contaminated environments. A mobile robot explores an unknown area while
a Gaussian-Process (GP) radiation field map is built in real time, sources are
localized, and the live predicted field is fed back into navigation as a cost — so
the robot maps the space while actively minimizing dose.

This README is the architecture map for the whole stack. Each package has its own
README with node-, topic-, parameter-, and launch-level detail; links are below.

---

## The two arms

Every trial runs in one of two arms, selected by a single launch argument on
`rad_launch_sim_launch.py`:

| Arm | `radiation_agnostic` | Radiation costmap layer | Frontier radiation costing | Local Recovery Node | Hard safety brake |
| --- | --- | --- | --- | --- | --- |
| **RaMonES** | `false` (default) | loaded | active (avoid + optional seek) | active | active |
| **Baseline** | `true` | omitted from Nav2 config | weights zeroed | disabled (thresholds set to 1e6 at construction) | **active** |

The hard safety brake (`rad_safety_monitor_node`) is deliberately held active in
**both** arms — it is a physical overexposure backstop, not a navigation strategy.
A separate, exploratory-only `safety_agnostic:=true` flag can disable even that for
an unsafe-baseline ablation.

### Layered safety / recovery thresholds
Three dose-rate levels, enforced by different components:

- **~10 mrem/hr — acute.** Shared by the frontier hard-exclusion (in `explore_node`) and `local_recovery_node`'s escape trigger (`acute_dose_rate_mrem_hr = 10.0`). This is where the proactive escape search fires.
- **~12 mrem/hr — emergency brake.** `rad_safety_monitor_node`'s `emergency_threshold_mrem_hr` (YAML value), intentionally *above* the acute level so it only engages as a true last resort, after yielding to `local_recovery_node` for a patience window.
- **~2 mrem/hr — scoring only.** Used by `rad_evaluator_node` to score dose violations and max penetration; it does **not** drive robot behaviour.

Two physically-distinct recovery mechanisms live in `rad_local_recovery`:
- `local_recovery_node` — **dose-triggered** escape search on a costmap/heatmap-scored polar grid.
- `stuck_escape_node` — **geometry watchdog** for a wedged robot (no displacement for a stuck timeout); cancels the goal and backs out. The `escape_distance_m` argument is the subject of a documented fix: `0.0` keeps the legacy time-based ~0.25 m escape, a positive value gives a distance-based escape that clears the Nav2 inflation band.

---

## Packages

| Package | Role | README |
| --- | --- | --- |
| `rad_simulation` | Simulated radiation field + sensor (stand-in for hardware) | — (brief below) |
| `rad121_monitor` | Real RAD121 Geiger-counter driver | [README](rad121_monitor/README.md) |
| `ros2_rad_map` | GP radiation mapping, hotspot finding, visualizers, custom msgs/srvs | — (brief below) |
| `rad_costmap_plugin` | Nav2 costmap layer injecting the predicted field as nav cost | [README](rad_costmap_plugin/README.md) |
| `rad_nav_behavior` | Zone explorer (exploration/surveillance) + orbital survey | [README](rad_nav_behavior/README.md) |
| `rad_local_recovery` | Dose-triggered escape + geometry stuck-recovery | [README](rad_local_recovery/README.md) |
| `rad_safety` | Hard dose backstop / emergency brake | [README](rad_safety/README.md) |
| `rad_evaluation` | Per-trial metrics + the dose-budget authority | — (brief below) |
| `rad_launch` | Top-level launch orchestration + system reset | [README](rad_launch/README.md) |

### The three packages without a dedicated README (covered here)

**`rad_simulation`** — the simulated sensing layer that stands in for `rad121_monitor`
during trials. `rad_field_server` holds the ground-truth field (point sources with
~1/r² decay plus a fixed randomized spatial background), exposes services to spawn,
clear, and save sources (`SpawnSource`, `SpawnRandomSources`,
`SetFloat64`, plus `std_srvs` toggles), and publishes source markers / source data /
background level under `~/`. `rad_sensor_simulator` samples that field at the robot's
TF pose and publishes the reading on **`/rad`** (`std_msgs/Float64MultiArray`,
`[compensated_cpm, mrem_hr]`) — the exact topic and format `rad121_monitor` produces,
so the rest of the stack is identical in sim and on hardware. `source_selector_node`
lets you place sources interactively from RViz's "Publish Point". Params in
`params/sensor_simulator.yaml` and the `*_saved_sources.yaml` world presets.

**`ros2_rad_map`** — the mapping and source-localization brain, and the package that
defines the shared custom interfaces (`GPPredict`, `FindPeaks`,
`OrbitZone`, `GoToZone` services; `RadZone`, `RadZoneArray` messages).
`gp_predictor_node.py` (GPy, CPU) consumes `/rad` + robot pose and serves the
multi-fidelity Autoregressive GP (`argp`) or single-fidelity prediction via the
`predict_gp` service, plus hotspot optimization via `find_rad_peaks`.
`rad_heatmap_node` calls `predict_gp` on a timer and publishes the field images —
notably **`/rad_heatmap/gp_mean_map`** and **`/rad_heatmap/raw_mean_map`**
(`sensor_msgs/Image`, `32FC1`), plus std-dev/overlay maps — and offers
`reset_rad_heatmap` / `save_rad_maps`. `peak_finder_client_node` turns GP peaks into
**`/radiation_zones`** (`RadZoneArray`) and `/confirmed_rad_sources`. The rest
(`gp_visualizer_node`, `rad_costmap_visualizer_node`, `rad_pointcloud_node`,
`rad_mapper_node`) are visualization/auxiliary nodes. The
trained low-fidelity prior lives in `models/gp_model.pkl`.

**`rad_evaluation`** — the scoring and dose-budget layer. `rad_evaluator_node`
subscribes to `/rad`, `/map`, the ground-truth source markers
(`/rad_field_server/sources`), the detected peaks (`/radiation_zones`), and
`/stuck_escape/event`; it integrates cumulative dose (the campaign's reported ground
truth) and is the **dose-budget authority**, publishing
**`/dose_budget/remaining_fraction`** (`std_msgs/Float64`) that `explore_node`,
`local_recovery_node`, and `rad_safety_monitor_node` all consume so their effective
thresholds scale by one shared multiplier. It also subscribes to each of those
layers' `~/dose_budget_multiplier` for a per-trial contract-coherence check. Ships a
`rosbag`-recording launch and `rosbag_stopper.py` for batch trials.

> Note: frontier exploration itself is provided by **`explore_lite`**, a
> radiation-aware fork (`explore-ros2`, pinned by `ramones_ws`; not in this repo). It owns the
> `/explore_node/start|stop|reset` services and the radiation-informed frontier
> costing that the arm selector and the frontier-costing launch args configure.

---

## Data flow (sensing → mapping → localization → exploration → recovery)

```
                    radiation reading  /rad  [cpm, mrem/hr]
 rad_sensor_simulator ─────────────────┬───────────────────────────────┐
 (or rad121_monitor)                   │                               │
                                       ▼                               ▼
                              gp_predictor_node  ──predict_gp──▶  rad_heatmap_node
                              (GP / ARGP map)     (service)       publishes field images:
                                       │                          /rad_heatmap/gp_mean_map
                                       │ find_rad_peaks            /rad_heatmap/raw_mean_map
                                       ▼                                   │
                              peak_finder_client_node                      │
                              publishes /radiation_zones                   │
                                       │                                   │
         ┌─────────────────────────────┤                                   │
         ▼                             ▼                                   ▼
 zone_explorer_node            explore_lite (explore_node)        rad_costmap_plugin
 (orbital survey of zones)     frontier exploration,             RadiationLayer reads
                               radiation-informed costing        /rad_heatmap/raw_mean_map
                                       │                          → Nav2 global costmap cost
                                       ▼                                   │
                                 Nav2 navigate_to_pose ◀────────────────────┘
                                       ▲        ▲
                 escape goals          │        │  retreat goals / geometry backup
   local_recovery_node ────────────────┘        └──── rad_safety_monitor_node
   (dose-triggered, reads /rad,                        (emergency brake, reads /rad)
    costmap, gp_mean_map)              stuck_escape_node (odometry watchdog → /cmd_vel + spin)

 rad_evaluator_node integrates /rad → /dose_budget/remaining_fraction ──▶ (explore_node,
                                                      local_recovery_node, rad_safety_monitor_node)
```

1. **Sense.** `rad_sensor_simulator` (sim) or `rad121_monitor` (hardware) publishes dose on `/rad`.
2. **Map.** `gp_predictor_node` fits a (multi-fidelity) GP; `rad_heatmap_node` turns predictions into field images.
3. **Localize sources.** `peak_finder_client_node` optimizes the GP mean into hotspots on `/radiation_zones`.
4. **Explore.** `explore_lite` drives frontier exploration with radiation-informed costing; `rad_costmap_plugin` bakes the predicted field into the Nav2 global costmap; `zone_explorer_node` optionally orbits discovered sources.
5. **Recover.** `local_recovery_node` (dose) and `stuck_escape_node` (geometry) handle escapes; `rad_safety_monitor_node` is the hard brake. All three dose layers share the budget multiplier from `rad_evaluator_node`.

---

## Key tunables

Set on `ros2 launch rad_launch rad_launch_sim_launch.py`:

| Argument | Default | What it controls |
| --- | --- | --- |
| `radiation_agnostic` | `false` | **Arm selector** (RaMonES vs Baseline). |
| `world` | `warehouse_barricaded` | Gazebo world. |
| `headless` | `false` | Server-only Gazebo for batch runs. |
| `setup_path` | `$RAMONES_SETUP_PATH` / `~/clearpath/` | Clearpath robot setup directory shared by Gazebo, SLAM and Nav2 (see Build). |
| `gp_mode` | `argp` | GP fidelity: `argp` (trained prior) or `single_fidelity` (flat prior). |
| `known_field_mode` | `false` | Oracle field (analytic true field instead of GP) — diagnostic ablation. |
| `escape_distance_m` | `0.0` | Stuck-escape fix: `0` = legacy ~0.25 m time-based; `>0` = distance-based, clears inflation band. |
| `dose_relaxation_k` | `0.0` | Budget-aware threshold relaxation shared across the three dose layers. |
| `frontier_spend_fraction` | `0.05` | Dose-budget fraction one frontier round-trip may spend. |
| `radiation_seek_scale` / `radiation_seek_adaptive` | `0.0` / `false` | Attraction toward elevated-but-unconfirmed regions. |
| `blacklist_ttl_s` | `0.0` | Frontier-blacklist expiry (`0` = permanent). |
| `max_recovery_episodes` | `0` | Cap on dose-triggered recovery episodes (`0` = unbounded). |
| `safety_agnostic` | `false` | Ablation: disable even the hard brake. |
| `orbit_surveys` | `false` | Launch the zone explorer to orbit discovered sources. |
| `record_ground_truth` | `false` | Bridge true robot pose to `/ground_truth/tf` (H3 metric). |

See the [`rad_launch` README](rad_launch/README.md) for the full argument list and
every secondary launch file.

Per-component thresholds live in each package's params: dose thresholds in
`rad_local_recovery/params/local_recovery.yaml` and `rad_safety/params/rad_safety.yaml`;
costmap cost ramp in the Nav2 config that wires `rad_costmap_plugin::RadiationLayer`;
GP/heatmap settings in `ros2_rad_map/params/heatmap.yaml`; field/source presets in
`rad_simulation/params/*.yaml`.

---

## Build

**Use the meta-repo.** [`ARC-LTS/ramones_ws`](https://github.com/ARC-LTS/ramones_ws)
pins every component (this repo, `explore-ros2`, `navigation`, `gazebo_sim`, the
Clearpath platform packages), ships the frozen Clearpath robot configuration the
campaign ran on, and bootstraps the workspace with one script:

```bash
git clone https://github.com/ARC-LTS/ramones_ws.git
cd ramones_ws && ./setup.sh
source install/setup.bash
```

Developing on this repo alone? `radmon.repos` pulls the sibling repositories on
their `maintenance` branches. (Their `main` branches predate the campaign launch
arguments and won't work with this branch.)

```bash
sudo apt install python3-vcstool
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone -b maintenance https://github.com/ARC-LTS/ROS2_Rad121.git
vcs import --input ROS2_Rad121/radmon.repos
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install GPy            # gp_predictor_node; GPy has no rosdep rule
colcon build --symlink-install
source install/setup.bash
```

Don't overlay another `clearpath_simulator` checkout (for example an older
`~/clearpath_ws`) on top of this workspace. A 0.2.x `clearpath_generator_gz` is
incompatible with the Humble 1.3.x `clearpath_generator_common` and crashes
robot bring-up.

### Robot configuration (`setup_path`)

The Gazebo, SLAM and Nav2 launches all read a Clearpath setup directory
(`robot.yaml` plus generated `platform/`, `sensors/`, `manipulators/` files).
`rad_launch_sim_launch.py` takes it from the `setup_path` argument, defaulting to
`$RAMONES_SETUP_PATH` and then `~/clearpath/`, and passes it to all three.
`gazebo_sim` does not regenerate these files (`generate:=false`), so the directory
must already contain them. The campaign used a frozen configuration with the
**EKF disabled** and the **VLP-16 at 10 Hz**. `ramones_ws` ships it. Regenerating
it with current Clearpath packages turns the EKF on and raises the lidar to 50 Hz,
and the EKF's skid-steer wheel odometry then fights rtabmap's ICP odometry and
smears the map whenever the robot turns.

## Run a trial
```bash
export RAMONES_SETUP_PATH=/path/to/ramones_ws/config/clearpath/   # frozen campaign config
export RAMONES_GP_THREADS=2      # optional: keep the GP fit from starving SLAM on <=8 cores

# RaMonES arm, default world (warehouse_barricaded), with the Gazebo GUI
ros2 launch rad_launch rad_launch_sim_launch.py

# Baseline arm, headless, rectangle world, with the stuck-escape fix on
ros2 launch rad_launch rad_launch_sim_launch.py \
  radiation_agnostic:=true headless:=true world:=rectangle escape_distance_m:=0.8
```

For scored, reproducible trials (pinned source layouts, evaluator, outcome
detection), use `experiment_runner/run_trial.py` from `ramones_ws`.

## License
BSD-3-Clause (see `LICENSE`). Maintainer: Brendon Cintas.
