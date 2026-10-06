# rad_safety

The hard dose backstop: a last-resort emergency brake that retreats the robot when the averaged dose rate exceeds an emergency threshold.

## Overview

`rad_safety` is the bottom layer of the RaMonES recovery stack. Where
`local_recovery_node` (in `rad_local_recovery`) runs a sophisticated,
utility-scored, escalating-radius escape search, `rad_safety_monitor_node` is a
blunt physical brake: average the dose over a short window, and if it crosses the
emergency threshold, send a single `navigate_to_pose` goal straight backward to
the nearest free cell.

It is deliberately the **last resort**. Its threshold (12.0 mrem/hr in the YAML)
sits *above* `local_recovery_node`'s acute trigger (10.0 mrem/hr) so that the
proactive escape layer always engages first, and it yields to that node for a
configurable patience window before ever stepping in. Crucially, unlike
`local_recovery_node`, this node is **kept active in both the RaMonES and
Baseline arms** — it is a physical safety brake, not a navigational strategy, so
disabling it for Baseline would confound the comparison with unmodelled
acute-overexposure risk. A separate `safety_agnostic` ablation flag can disable
it for an explicitly-tagged unsafe-baseline experiment only.

---

## Nodes

### `rad_safety_monitor_node`
Executable `rad_safety_monitor_node` — source `src/rad_safety_monitor_node.cpp`.

**Subscribed topics**
- `/rad` (`std_msgs/Float64MultiArray`) — radiation reading; element `[1]` is dose rate in mrem/hr. Averaged over `dose_smoothing_window` samples; a retreat triggers when the average exceeds `effectiveThreshold()`.
- `/global_costmap/costmap` (`nav_msgs/OccupancyGrid`) — Nav2 global costmap; QoS KeepLast(1) / transient-local / reliable. Used to find a free retreat cell.
- `/local_recovery_node/active` (`std_msgs/Bool`) — `local_recovery_node`'s recovery state; this node defers while it is `true` (within the patience window).
- `/dose_budget/remaining_fraction` (`std_msgs/Float64`) — remaining dose-budget fraction from `rad_evaluator_node` (the dose-budget authority).

**Published topics**
- `~/active` → `/rad_safety_monitor_node/active` (`std_msgs/Bool`) — `true` while this node owns an in-flight retreat goal; lets `local_recovery_node` stand down symmetrically. Published on every state change.
- `~/dose_budget_multiplier` → `/rad_safety_monitor_node/dose_budget_multiplier` (`std_msgs/Float64`) — the threshold multiplier in effect; published every 1 s for `rad_evaluator_node`'s contract-coherence check.

**Action client**
- `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) — the single backward retreat goal.

**Parameters** (declared defaults; `params/rad_safety.yaml` overrides a subset)
| Parameter | Type | Default (declared) | Description |
| --- | --- | --- | --- |
| `emergency_threshold_mrem_hr` | double | 10.0 | Emergency brake threshold. **The YAML raises this to 12.0** so it sits above `local_recovery_node`'s 10.0 acute trigger (see Behaviour notes). Scaled by the budget multiplier. |
| `retreat_distance_m` | double | 1.5 | Nominal retreat distance directly behind the robot. **YAML sets 2.5.** |
| `robot_base_frame` | string | `base_link` | TF frame for the robot-pose lookup. |
| `map_frame` | string | `map` | TF/goal frame. |
| `dose_smoothing_window` | int | 5 | Number of consecutive `/rad` readings averaged before triggering; prevents false retreats from transient spikes. |
| `retreat_cooldown_s` | double | 10.0 | Minimum interval between retreat goals; mirrors `local_recovery_node`'s 10 s cooldown so neither node preempts the other on every tick. |
| `local_recovery_patience_s` | double | 30.0 | How long to let `local_recovery_node`'s own escape search run before this node takes over as last resort. |
| `dose_budget_relaxation_k` | double | 0.0 | Dose-budget-aware threshold relaxation coefficient. 0 reproduces the fixed threshold exactly. |
| `dose_budget_relaxation_ceiling_mult` | double | 2.0 | Upper bound on the relaxation multiplier. |
| `max_radiation_dose_mrem` | double | 100.0 | Mission dose budget for the internal integrator fallback when the authority fraction is stale. |

**Behaviour notes**
- **Effective threshold** = `emergency_threshold_mrem_hr * min(ceiling, 1 + k * f_remaining)`, using the authority's `/dose_budget/remaining_fraction` when fresh (< 5 s old), else this node's own `/rad` integration. `k = 0` (default) reproduces the fixed threshold exactly.
- **Ordering** (from `params/rad_safety.yaml`): the emergency threshold (12.0) is intentionally higher than `local_recovery_node`'s acute trigger (10.0) so only one node drives navigation at a time and the hard brake genuinely is the last resort. Both thresholds scale by the same multiplier, preserving the ordering for every `k`. A comment notes that an earlier value of 8.0 inverted the ordering (brake fired before the rescue layer).
- **Yield / takeover**: while `/local_recovery_node/active` is `true`, this node yields until that node has been active longer than `local_recovery_patience_s` without resolving, then takes over once as a last resort and restarts the patience clock — producing a fair alternation rather than a live-lock. A source comment records a case where the live-lock drove cumulative dose to 16,678 mrem (166× the 100 mrem budget) before the patience/cooldown guards were added.
- **Retreat point** (`findFreeRetreatPoint`): tries the backward direction at angular offsets ±15°, ±30° and distance fractions 1.0 / 0.75 / 0.5; accepts the first costmap cell with cost in `[0, 70)`. Falls back to the raw backward point if none is free. The goal is oriented to face the chosen point.
- `is_retreating_` clears on any goal result (success, abort, or preempt); the cooldown is what prevents an immediate competing re-fire.

---

## Launch files

No launch files in this package. Launch lives in `rad_launch`:
- `rad_launch/launch/rad_safety_launch.py` launches `rad_safety_monitor_node`, loads `params/rad_safety.yaml`, sets `use_sim_time:=true`, applies `dose_relaxation_k`, and (if `safety_agnostic:=true`) sets `emergency_threshold_mrem_hr` to 1e6 to disable the brake.

Its arguments:
| Arg | Default | Description |
| --- | --- | --- |
| `dose_relaxation_k` | `0.0` | Budget-aware relaxation coefficient (→ `dose_budget_relaxation_k`). |
| `safety_agnostic` | `false` | Exploratory ablation only — disables the hard brake. Distinct from `radiation_agnostic`; must stay `false` for every confirmatory H1–H4 trial. |

The stack bring-up (`rad_launch_sim_launch.py`) includes this launch in **both**
arms (it is not gated on `radiation_agnostic`).

## Config files
- `params/rad_safety.yaml` — `emergency_threshold_mrem_hr: 12.0`, `retreat_distance_m: 2.5`, `dose_smoothing_window: 5`, `robot_base_frame: base_link`, `map_frame: map`. (Note the YAML overrides the node's declared defaults of 10.0 and 1.5.)

## Dependencies
From `package.xml`: `rclcpp`, `rclcpp_action`, `std_msgs`, `nav_msgs`,
`nav2_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `ros2_rad_map`.
Build type `ament_cmake`.

## Build & run
```bash
colcon build --packages-select rad_safety
source install/setup.bash

ros2 launch rad_launch rad_safety_launch.py            # defaults
ros2 launch rad_launch rad_safety_launch.py safety_agnostic:=true   # ablation: brake disabled

# direct
ros2 run rad_safety rad_safety_monitor_node --ros-args --params-file \
  $(ros2 pkg prefix rad_safety)/share/rad_safety/params/rad_safety.yaml
```
