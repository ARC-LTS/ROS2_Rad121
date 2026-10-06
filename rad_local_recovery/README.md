# rad_local_recovery

Acute-dose escape and geometric stuck-recovery behaviours for the RaMonES exploration robot.

## Overview

This package provides the two independent recovery layers that sit beneath Nav2
and the frontier explorer:

| Node | Trigger | What it does |
| --- | --- | --- |
| `local_recovery_node` | **Radiation** — rolling-average dose rate crosses an acute threshold | Stops frontier exploration, searches a costmap/heatmap-scored polar grid for a low-dose navigable escape point, drives there via Nav2, and resumes exploration once dose falls below the safe threshold. |
| `stuck_escape_node` | **Geometry** — robot has a nav goal executing but is not physically moving | Cancels the goal, backs the robot out of the inflation band via direct `cmd_vel`, then runs a Nav2 `Spin` to reorient. Radiation-agnostic; runs in both arms. |

`local_recovery_node` is the **RaMonES-arm** proactive escape layer (disabled in
the Baseline arm by making its thresholds unreachable). `stuck_escape_node` is a
general navigational watchdog that runs regardless of arm.

Both nodes coordinate with `rad_safety`'s `rad_safety_monitor_node` (the hard
emergency brake) through a pair of `~/active` Bool topics so that only one node
owns a `navigate_to_pose` goal at a time — see the behaviour notes below.

---

## Nodes

### `local_recovery_node`
Executable `local_recovery_node` — source `src/local_recovery_node.cpp`.

Dose-triggered escape-search recovery. On every `/rad` message it keeps a rolling
average of the dose rate over the last `dose_confirmation_count` samples; a 2 s
timer (`recoveryTick`) drives the state machine.

**Subscribed topics**
- `/rad` (`std_msgs/Float64MultiArray`) — radiation reading; element `[0]` is compensated CPM, element `[1]` is dose rate in mrem/hr (the value used here). Requires `size >= 2`.
- `/global_costmap/costmap` (`nav_msgs/OccupancyGrid`) — Nav2 global costmap; QoS KeepLast(1) / transient-local / reliable.
- `/rad_heatmap/gp_mean_map` (`sensor_msgs/Image`, `32FC1`) — GP mean radiation field, used to score escape candidates by predicted dose.
- `explore/is_finished` (`std_msgs/Bool`) — transient-local/reliable; when `true`, recovery logic is disabled for the rest of the mission.
- `/rad_safety_monitor_node/active` (`std_msgs/Bool`) — `rad_safety_monitor_node`'s retreat state; this node stands down while that is `true`.
- `/dose_budget/remaining_fraction` (`std_msgs/Float64`) — remaining dose-budget fraction published by `rad_evaluator_node` (the "dose-budget authority").

**Published topics**
- `~/active` → `/local_recovery_node/active` (`std_msgs/Bool`) — `true` while a recovery is in progress. Published every `recoveryTick` (2 s), not just on transitions, so a late subscriber always gets a fresh value.
- `~/dose_budget_multiplier` → `/local_recovery_node/dose_budget_multiplier` (`std_msgs/Float64`) — the threshold multiplier currently in effect; published every 1 s for `rad_evaluator_node`'s cross-layer contract-coherence check.

**Action client**
- `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) — sends escape goals; cancels all goals on self-heal resume.

**Service clients**
- `stop_explore_service` (default `/explore_node/stop`, `std_srvs/Trigger`) — pause frontier exploration.
- `start_explore_service` (default `/explore_node/start`, `std_srvs/Trigger`) — resume frontier exploration.

(A `/cmd_vel` publisher and a "physical escape" back-up routine are present but commented out — see Dead code.)

**Parameters** (declared defaults; `params/local_recovery.yaml` overrides a subset)
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `dose_confirmation_count` | int | 3 | Rolling-average window length over `/rad[1]`; smooths transient spikes before recovery is considered. |
| `critical_dose_mrem_hr` | double | 2.0 | **Unused by this node's logic** (declared, read, stored, never referenced in any branch). Labelled "cost-function / safe-zone threshold". See Known mismatches. |
| `acute_dose_rate_mrem_hr` | double | 10.0 | **The real hard-evacuation trigger.** Recovery fires when the rolling average exceeds `acute_dose_rate_mrem_hr * budget_multiplier`. |
| `safe_dose_threshold_mrem_hr` | double | 1.0 | Resume threshold; exploration resumes once the rolling average drops below `safe_dose_threshold_mrem_hr * budget_multiplier`. |
| `recovery_search_distance_m` | double | 15.0 | Max radius of the polar escape-point search. |
| `robot_base_frame` | string | `base_link` | TF frame for the robot pose lookup. |
| `map_frame` | string | `map` | TF/goal frame. |
| `fallback_retreat_distance_m` | double | 3.0 | Distance used by the 8-direction and last-resort fallbacks when the primary search finds nothing. |
| `return_to_origin` | bool | false | If true, evacuate toward the map origin when in bounds (auto-disabled if origin is out of map bounds). |
| `origin_x` / `origin_y` | double | 0.0 / 0.0 | Map-origin evacuation point; also the "last resort" destination after 4 consecutive failures. |
| `stop_explore_service` | string | `/explore_node/stop` | Trigger service to pause exploration. |
| `start_explore_service` | string | `/explore_node/start` | Trigger service to resume exploration. |
| `max_recovery_episodes` | int | 0 | Hard cap on distinct recovery episodes per mission. 0 = unbounded. When positive and the cap is hit, exploration is left running (deferred to the dose-budget and brake layers) rather than being stopped again. `consecutive_recovery_failures_` resets on every resume, so it bounds a losing streak but not the total. |
| `dose_budget_relaxation_k` | double | 0.0 | Dose-budget-aware threshold relaxation coefficient. 0 reproduces the fixed thresholds exactly. |
| `dose_budget_relaxation_ceiling_mult` | double | 2.0 | Upper bound on the relaxation multiplier. |
| `max_radiation_dose_mrem` | double | 100.0 | Mission dose budget, used only by the internal integrator fallback when the authority fraction is stale. |

**Behaviour notes**
- **Budget multiplier**: `min(ceiling, 1 + k * f_remaining)`, where `f_remaining` is the authority's `/dose_budget/remaining_fraction` when fresh (received within 5 s), otherwise `(max_radiation_dose_mrem − cumulative_dose) / max_radiation_dose_mrem` from this node's own `/rad` integration. Both the acute trigger and the safe/resume threshold scale by the same multiplier so the entry/exit hysteresis ratio is invariant in `k`.
- **Escape-point search** (`findSafePoint`): sweeps radii 5..`search_radius` px (step 5) × angles 0..360° (step 15°); accepts cells with costmap cost in `[0, 70)`; scores `utility = −rad*10 + dist/search_radius − cost/50`; early-exits on a cell with `rad < safe_dose_threshold` and distance > 5 m. Search radius expands ×1.5 after ≥2 consecutive failures.
- **Fallback chain** when no safe point is found: (1) best of 8 compass directions at `fallback_retreat_distance_m` by lowest cost; (2) `findAnyNavigablePoint` rear-hemisphere sweep that ignores the blacklist; (3) raw unchecked backward point.
- **Origin last resort**: after ≥4 consecutive failures, force a goal to the map origin — but only if the origin has not itself already failed (otherwise fall through to the normal search).
- **Blacklist**: failed escape points are blacklisted within a 2 m radius, capped at 8 entries (oldest dropped). Failures that occur while `rad_safety_monitor_node` is active, or that return `CANCELED`, are **not** blacklisted (they are treated as preemptions, not genuinely-unsafe points).
- **Self-heal**: if marked recovering but the dose has already dropped below the safe threshold, cancel goals and resume immediately rather than waiting on a specific `navigate_to_pose` result (fixes a dead-zone hang observed between the safe and acute thresholds).
- **Mutual exclusion**: stands down entirely while `/rad_safety_monitor_node/active` is `true`; publishes its own `~/active` so the safety node can symmetrically defer.
- **Cooldown**: will not preempt an in-flight recovery goal within a 10 s window.

### `stuck_escape_node`
Executable `stuck_escape_node` — source `src/stuck_escape_node.cpp`.

Radiation-independent geometry watchdog. A 500 ms `monitorTick` watches odometry
while a `navigate_to_pose` goal is executing; if the robot has not moved
(`< min_movement_m` translation **and** `< min_rotation_rad` rotation) for
`stuck_timeout_s`, it runs a cancel → back-up → spin escape.

**Subscribed topics**
- `odom_topic` (default `/odometry`; `nav_msgs/Odometry`) — robot pose. **The stack launch overrides this to `/odom`** because `/odometry` does not exist here (the default produced a perpetually-zero pose and false-positive stuck triggers).
- `/navigate_to_pose/_action/status` (`action_msgs/GoalStatusArray`) — tracks whether a nav goal is `STATUS_EXECUTING`; the wedge watchdog arms only while one is.

**Published topics**
- `/cmd_vel` (`geometry_msgs/Twist`) — direct backward drive during the escape.
- `/stuck_escape/event` (`std_msgs/Float64`) — one message per completed wedge-and-recover cycle; payload is the wall-clock recovery duration (consumed by `rad_evaluator_node`).
- `/stuck_escape/idle_stall` (`std_msgs/Float64`) — one message when an idle-stall episode is detected; payload is the stalled duration. Deliberately a separate topic so an idle stall is never folded into the `stuck_count` navigational-failure metric.

**Action clients**
- `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) — cancels the active goal before backing up.
- `spin` (`nav2_msgs/action/Spin`) — Nav2 costmap-aware reorientation after backing up.

**Parameters** (all via launch; no YAML in this package)
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `stuck_timeout_s` | double | 12.0 | No-progress duration (while a goal executes) before an escape triggers. |
| `escape_duration_s` | double | 2.5 | Legacy time-based backup duration. Used only when `escape_distance_m == 0`. |
| `escape_speed_m_s` | double | 0.10 | Backward drive speed. Time-based escape therefore covers `0.10 × 2.5 = 0.25 m`. |
| `min_movement_m` | double | 0.1 | Translation below this (over the measurement window) counts as "not moving". |
| `min_rotation_rad` | double | 0.08 | Yaw change below this counts as "not moving"; prevents cancelling a legitimate final in-place RotateToGoal. |
| `odom_topic` | string | `/odometry` | Odometry source (overridden to `/odom` by the stack launch). |
| `spin_target_yaw_rad` | double | 3.14159 | Reorientation angle after the backup (π = 180°). Couples to `explore_node`'s rear-hemisphere frontier penalty — see note. |
| `spin_timeout_s` | double | 8.0 | Force escape completion if the Spin result never arrives (prevents a permanent `is_escaping_` lock). |
| `escape_distance_m` | double | 0.0 | **The escape-distance fix.** 0 keeps the legacy time-based 0.25 m backup. A positive value ends the drive on measured odometry displacement instead, clearing the ~0.4–0.5 m costmap inflation band the robot was wedged in and immune to real-time-factor drift. |
| `escape_max_duration_s` | double | 12.0 | Upper bound on the distance-based drive (bounds the case where the robot genuinely cannot move). |
| `idle_stall_timeout_s` | double | 0.0 | 0 disables. When positive, arms a second detector for a robot stationary with **no** nav goal executing (a state the wedge watchdog is structurally blind to), reporting on `/stuck_escape/idle_stall`. |
| `cancel_settle_max_ms` | int | 400 | Max wait for Nav2 to release `cmd_vel` after the goal cancel, before the backup starts. |
| `cancel_wait_for_confirm` | bool | false | false waits the full `cancel_settle_max_ms`; true starts the backup as soon as the goal-status feed confirms no goal is executing (a behaviour change, hence opt-in). |

**Behaviour notes**
- Escape is a two-phase timer (`DELAY` then `DRIVING`) on a 50 ms wall timer; `cmd_vel` is held low during the settle because Nav2's `velocity_smoother` remaps `cmd_vel_smoothed → cmd_vel` (two publishers on one topic would blend the escape twist away).
- After backing up, a `Spin` (default π) reorients so the robot does not end up back in the same trap facing the same wall. **Coupling caveat**: `explore_node` applies a 1e6 penalty to rear-hemisphere frontiers, so a π spin inverts which frontiers are admissible — smaller `spin_target_yaw_rad` values break the face-the-same-wall failure without inverting the frontier ordering.
- Runs with `use_sim_time:=true` in the stack; the `escape_distance_m` fix exists partly because the time-based escape shrinks in sim-space under real-time-factor drops.

---

## Launch files

This package ships **no launch files of its own**. Launch orchestration lives in
`rad_launch`:
- `rad_launch/launch/rad_local_recovery_launch.py` launches `local_recovery_node` (loads `params/local_recovery.yaml`, applies the `radiation_agnostic`, `dose_relaxation_k`, `retreat_dose_mrem_hr`, and `max_recovery_episodes` overrides, and sets `use_sim_time:=true`).
- `rad_launch/launch/rad_launch_sim_launch.py` launches `stuck_escape_node` inline (with `odom_topic:=/odom`, `use_sim_time:=true`, and the `escape_distance_m`, `idle_stall_timeout_s`, `spin_target_yaw_rad` args wired through).

See the `rad_launch` README for every argument and default.

## Config files
- `params/local_recovery.yaml` — defaults for `local_recovery_node`: `critical_dose_mrem_hr: 2.0`, `acute_dose_rate_mrem_hr: 10.0`, `safe_dose_threshold_mrem_hr: 1.0`, `recovery_search_distance_m: 15.0`, `robot_base_frame: base_link`, `map_frame: map`, `fallback_retreat_distance_m: 3.0`, `return_to_origin: false`, `origin_x/y: 0.0`, explore stop/start service names. `stuck_escape_node` has no params file.

## Known mismatches / dead code
- **`critical_dose_mrem_hr` is dead in the node.** It is declared, read, and stored, but never used in any conditional (the real trigger is `acute_dose_rate_mrem_hr`, default 10.0). However, `rad_local_recovery_launch.py` sets `critical_dose_mrem_hr` from the `retreat_dose_mrem_hr` launch arg and comments it as "the actual hard-brake retreat trigger". Net effect: **varying `retreat_dose_mrem_hr` in the non-agnostic arm does not change recovery behaviour** — the acute trigger stays at 10.0. (The `radiation_agnostic` arm still disables recovery correctly, because it sets all three thresholds — including `acute_dose_rate_mrem_hr` — to 1e6.)
- **Node vs. YAML default for `safe_dose_threshold_mrem_hr`**: now aligned at 1.0 (an earlier node default of 0.25 against a YAML of 1.0 is called out in the source comments).
- **Commented-out physical escape**: `startPhysicalEscape()` / the `/cmd_vel` publisher in `local_recovery_node` are disabled; the physical back-up role is served by `stuck_escape_node` instead.

## Dependencies
From `package.xml`: `rclcpp`, `rclcpp_action`, `std_msgs`, `std_srvs`, `nav_msgs`,
`nav2_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `sensor_msgs`,
`cv_bridge`, `ros2_rad_map`, `action_msgs`. Build type `ament_cmake`.

## Build & run
```bash
# from the workspace root
colcon build --packages-select rad_local_recovery
source install/setup.bash

# standalone (defaults from params/local_recovery.yaml)
ros2 launch rad_launch rad_local_recovery_launch.py

# run a single node directly
ros2 run rad_local_recovery local_recovery_node --ros-args --params-file \
  $(ros2 pkg prefix rad_local_recovery)/share/rad_local_recovery/params/local_recovery.yaml
ros2 run rad_local_recovery stuck_escape_node --ros-args -p odom_topic:=/odom -p escape_distance_m:=0.8
```
In practice both nodes are brought up by `rad_launch rad_launch_sim_launch.py`.
