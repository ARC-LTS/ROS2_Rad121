# rad_evaluation

Per-trial scoring and metrics recording for RaMonES experiments.

## Overview

`rad_evaluation` is the scorekeeper. It runs alongside a trial, passively
observes the live topics (dose stream, map, ground-truth sources, predicted
zones, robot pose via TF, stuck/recovery events), and on request writes a single
appended CSV row summarizing that trial. It computes the campaign's primary
outcome metrics:

- **source-localization error** — greedy one-to-one nearest matching of
  predicted zones to ground-truth sources within a match gate, reported as the
  mean error over matched pairs plus the matched/total counts;
- **cumulative / per-meter / average dose rate** — integrated from the `/rad`
  stream;
- **critical-dose violations, recovery time, max penetration** — time spent above
  the critical-dose ceiling;
- **area explored, distance travelled, stuck count / recovery** — coverage and
  locomotion;
- **field-estimate convergence** — when (if ever) the matched localization
  settled onto a stable plateau.

It is also the **dose-budget authority**: it integrates cumulative dose and
publishes the single authoritative remaining-budget fraction that the behavior
layers (explorer, local recovery, safety monitor) scale their thresholds by, and
it audits those layers for cross-layer threshold incoherence.

The companion `rosbag.launch.py` / `rosbag_stopper.py` record a filtered rosbag
of the trial for offline analysis.

---

## Nodes

### rad_evaluator_node (`src/rad_evaluator_node.cpp`, node name `rad_evaluator_node`)

Single node. A 100 ms timer samples the robot pose (for distance), a 1 s timer
publishes the dose budget and audits contract coherence, and the metrics are
accumulated callback-by-callback until `~/save_report` is called.

**Subscribed topics**

- `/rad` (`std_msgs/Float64MultiArray`) — dose stream (element `[1]`, mrem/hr).
  Integrated into cumulative dose (converting mrem/hr × seconds → mrem via
  `dt/3600`), tracked for peak dose, and used for violation / recovery /
  penetration bookkeeping against `critical_dose_mrem_hr`. `dt` is gated to
  `(0, 1) s` to ignore gaps.
- `/map` (`nav_msgs/OccupancyGrid`, transient-local reliable) — area explored =
  count of non-unknown cells × resolution².
- `/rad_field_server/sources` (`visualization_msgs/MarkerArray`) — ground truth.
  Only `ADD` + `SPHERE` markers are read; intensity is recovered from marker
  scale by inverting the simulator's normalization (`norm = (scale.x − 0.2)/0.8`,
  `intensity = norm × 7.5 + 2.5`).
- `/radiation_zones` (`ros2_rad_map/RadZoneArray`) — predicted peaks/zones from
  `peak_finder_client_node`. Stored for the end-of-trial loc-error, and drives
  live convergence detection.
- `/stuck_escape/event` (`std_msgs/Float64`) — one message per completed
  stuck-and-recover cycle from `rad_local_recovery` (physically wedged, not
  radiation-triggered). Payload is the recovery duration; tracked separately from
  the radiation violation/recovery metrics.
- `/explore_node/dose_budget_multiplier`, `/local_recovery_node/dose_budget_multiplier`,
  `/rad_safety_monitor_node/dose_budget_multiplier` (all `std_msgs/Float64`) —
  each behavior layer reports the relaxation multiplier it is actually using, for
  the contract-coherence audit.

**Published topics**

- `/dose_budget/remaining_fraction` (`std_msgs/Float64`, 1 Hz) — authoritative
  remaining budget = `max(0, (max_radiation_dose − cumulative_dose) / max_radiation_dose)`.

**Services (provided)**

- `~/save_report` (`std_srvs/Trigger`) → `/rad_evaluator_node/save_report` —
  compute the final metrics and append one row to
  `<output_dir>/<test_case_name>_report.csv` (writing the header first if the
  file is new).

**Parameters**

- `map_frame` (string, `map`) / `robot_frame` (string, `base_link`) — TF frames
  for the pose/distance integration.
- `critical_dose_mrem_hr` (double, `2.0`) — dose-rate ceiling that violations and
  `max_penetration` are reported against. Reporting-only; a deliberately
  conservative test threshold, **not** a robot behavior trigger (local recovery's
  actual retreat trigger is a separate parameter in its own launch). Exposed as
  the `critical_dose_mrem_hr` launch arg.
- `match_gate_m` (double, `10.0`) — max distance for a predicted peak to match a
  ground-truth source.
- `output_dir` (string, `<share>/rad_evaluation/eval_reports`) — CSV output dir.
- `test_case_name` (string, `unnamed_trial`) — CSV filename stem and `test_case`
  column (set by the `test_case` launch arg).
- `max_radiation_dose` (double, `100.0`) — denominator for the remaining-budget
  fraction (total dose budget, mrem).
- `convergence_locerr_tol_m` (double, `0.15`) — max matched-loc-error change
  between consecutive publishes to count as "stable".
- `convergence_stable_updates` (int, `3`) — consecutive stable publishes required
  to declare convergence.
- `convergence_min_sources` (int, `1`) — minimum matched sources required for the
  convergence test to apply.

**Key algorithm / behavior notes**

- *Localization error / matching* (`computeLocError`): all predicted-peak ↔
  ground-truth pairs within `match_gate_m` are enumerated, sorted by distance,
  and claimed greedily nearest-first so each predicted peak matches **at most
  one** ground-truth source and vice versa. The mean is taken only over matched
  pairs (`-1` if none matched), so `avg_loc_error_m` is conditional on
  `sources_matched` and must be read together with `sources_total`/`sources_matched`
  — a trial that localizes 1 of 5 sources well will out-score one that localizes
  5 of 5 adequately. The one-to-one rule fixes an earlier bug where one smeared
  prediction between two real sources counted as localizing both.
- *Convergence detection* is an oracle (it reads the ground truth) and is defined
  on the matched localization, because the raw `/radiation_zones` set
  over-produces and never settles in count. Convergence is declared when the
  matched count is constant and the matched mean loc-error changes <
  `convergence_locerr_tol_m` for `convergence_stable_updates` consecutive
  `/radiation_zones` publishes, with ≥ `convergence_min_sources` matched. It is
  **re-openable**: a later refinement that moves the plateau resets it, so the
  reported `time_to_converge_s` pins to the start of the *final* plateau and
  `converged` reflects the state at termination. This is the live twin of
  `experiment_runner/compute_convergence.py` — keep the two criteria in sync.
- *Dose integration correctness*: `cumulative_dose_` is in mrem (the `dt/3600`
  conversion is load-bearing; the in-code comment records that omitting it
  silently overcounts by 3600×). `avg_dose_rate` divides by `total_time/3600`
  and `dose_per_meter` divides by `total_distance_` for consistency.
- *Distance integration* filters per-tick deltas to `(0.01, 1.0) m` to reject TF
  jitter and teleport jumps.
- *Contract-coherence audit* (`contractTick`, 1 Hz): over the behavior layers
  heard from in the last 10 s, it records `max_multiplier_divergence` (max
  pairwise spread of the reported multipliers) and `contract_misorder_s`
  (seconds spent with the explorer's multiplier below local-recovery's — the
  mis-ordering that reopens the "yield-without-rescue" dead band).

**CSV report columns** (written by `~/save_report`): `timestamp, test_case,
total_time_s, distance_m, area_explored_m2, cum_dose_mrem, avg_dose_rate,
dose_per_meter, max_dose, violations, avg_recovery_s, max_penetration,
avg_loc_error_m, stuck_count, avg_stuck_recovery_s, max_multiplier_divergence,
contract_misorder_s, sources_total, sources_matched, converged,
time_to_converge_s, converged_loc_error_m, converged_sources_matched`.

### rosbag_stopper (`scripts/rosbag_stopper.py`, node name `rosbag_stopper`)

Owns a `ros2 bag record --all --exclude <regex>` subprocess so it holds the PID
directly, and exposes `/rosbag/stop` (`std_srvs/Trigger`) to stop it gracefully:
on stop it sends `SIGTERM` to the recorder only (not the process group) and
blocks indefinitely for a clean flush/finalize — no timeout, no `SIGKILL` —
before shutting down. The service response returns first; the wait happens on a
background thread.

**Parameter**: `exclude_regex` (string, `''`) — passed to `--exclude`; built by
`rosbag.launch.py` from `config/rosbag_excluded_topics.yaml`.

---

## Launch files

- `launch/eval_launch.py` — launches `rad_evaluator_node`. Args: `test_case`
  (default `baseline`) → `test_case_name`; `critical_dose_mrem_hr` (default `2.0`,
  cast to float) → `critical_dose_mrem_hr`. `output_dir` is fixed to
  `<share>/rad_evaluation/eval_reports`.
- `launch/rosbag.launch.py` — launches `rosbag_stopper` after building a single
  exclude regex from `rosbag_excluded_topics.yaml` (exact-match each listed topic
  plus a catch-all for `.../compressed|compressedDepth|theora` image-transport
  variants).

## Config / param files

- `config/rosbag_excluded_topics.yaml` — `excluded_topics:` list of topics kept
  out of the recorded bag (raw camera/LiDAR, RTAB-Map internals, octomap/point
  clouds, redundant costmap/plan copies, visualization/teleop, and ROS
  lifecycle/internal topics) to keep bags small and reconstructible.

## Dependencies

From `package.xml` / `CMakeLists.txt`: build tool `ament_cmake`; `rclcpp`,
`std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`,
`tf2_ros`, `tf2_geometry_msgs`, `std_srvs`, `ament_index_cpp`, and
`ros2_rad_map` (for the `RadZoneArray`/`RadZone` messages). `rosbag_stopper.py`
needs `rclpy`, `std_srvs`, and the `ros2bag` CLI at runtime.

> Note: `package.xml` carries placeholder `description`/`license`/`version`
> (`TODO`/`0.0.0`) — worth filling in for the handoff. `std_srvs` and
> `ament_index_cpp` are `find_package`d in CMake but not declared in
> `package.xml`.

## Build & run

```bash
colcon build --packages-up-to rad_evaluation
source install/setup.bash

# Launch the evaluator for a named trial
ros2 launch rad_evaluation eval_launch.py test_case:=informed_run_01 critical_dose_mrem_hr:=2.0

# (optional) record a filtered bag of the trial
ros2 launch rad_evaluation rosbag.launch.py

# At end of trial, flush the report row:
ros2 service call /rad_evaluator_node/save_report std_srvs/srv/Trigger '{}'

# Stop the bag cleanly:
ros2 service call /rosbag/stop std_srvs/srv/Trigger '{}'
```

Report rows land in `<install>/rad_evaluation/share/rad_evaluation/eval_reports/<test_case>_report.csv`.
