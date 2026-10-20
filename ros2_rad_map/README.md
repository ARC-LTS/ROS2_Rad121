# ros2_rad_map

Gaussian-Process radiation mapping and source peak-finding core for the RaMonES stack.

## Overview

`ros2_rad_map` is the estimation heart of RaMonES. It turns the sparse,
position-tagged GM-counter stream (`/rad`) into a dense, continuous estimate of
the radiation field and extracts source-location estimates from that field.

The pipeline splits across two cooperating nodes plus two consumers:

1. **`gp_predictor_node`** (Python) owns the statistical model. It buffers the
   `/rad` samples against TF, fits a Gaussian Process (optionally a
   multi-fidelity / ARGP model that uses an offline prior as its mean function),
   and answers two services: `predict_gp` (field mean + std-dev at a set of
   points) and `find_rad_peaks` (argmax search for source candidates).
2. **`rad_heatmap_node`** (C++) drives the model. On a timer it collects its own
   elevation-thresholded samples, calls `predict_gp` over every known map cell,
   and publishes the fused field as images (`/rad_heatmap/gp_mean_map`,
   `/rad_heatmap/gp_std_dev`) plus zone boundaries and source-estimate markers.
3. **`peak_finder_client_node`** (C++) periodically calls `find_rad_peaks`,
   tracks peaks across calls, and publishes `/radiation_zones` (consumed by the
   evaluator and the explorer) and `/confirmed_rad_sources`.
4. **`rad_costmap_visualizer_node`** (C++) converts the GP mean map into a
   Nav2-style `OccupancyGrid` cost layer (`/radiation_costmap`) so the field can
   be fed back into navigation.

The single most important tunable for localization sharpness is the GP
likelihood noise floor (`gp_noise_variance_min`, overridable at runtime by the
`RAMONES_GP_NOISE_MIN` environment variable): lower noise → tighter
interpolation → sharper posterior-mean peaks → peak argmax closer to the true
source.

This package also builds several auxiliary executables that are **not** part of
the core estimation path documented here (`rad_mapper_node`,
`rad_pointcloud_node`, `gp_visualizer_node`) and ships extra param files
(`pointcloud.yaml`, `zones.yaml`, `zone_planner.yaml`, `predefined_zones.yaml`)
for them. `src/rad_zone_planner_node.cpp` is present in the tree but is **not
compiled** (it has no `add_executable` entry in `CMakeLists.txt`) — treat it as
dead/legacy code. Likewise the generated services `GetHeatmaps.srv`,
`SaveMaps.srv`, `OrbitZone.srv`, `GoToZone.srv` are declared for those auxiliary
nodes and are not used by the four core nodes below.

---

## Nodes

### gp_predictor_node (`scripts/gp_predictor_node.py`, node name `gp_predictor_node`)

The GP model server. Uses [GPy](https://github.com/SheffieldML/GPy) for the GP
and `scipy.optimize.minimize` (L-BFGS-B) for peak search. Thread-safe around a
single `data_lock`.

**Subscribed topics**

- `/rad` (`std_msgs/Float64MultiArray`) — GM stream. Element `[0]` is a dummy
  CPM, element `[1]` is the dose rate in mrem/hr (the value actually used).
  Each reading is rolling-averaged (`rad_smoothing_window`), TF-transformed
  `sensor_frame`→`map_frame`, spatially downsampled (0.2 m), and stored in the
  internal `historical_samples` buffer (capped at 1000) plus a separate
  `landmark_samples` buffer for readings above `1.5 × max(background, landmark_background_min)`.
- `/rad_field_server/background_radiation` (`std_msgs/Float64`) — prior-mean /
  background level. Also accepts a `Float64MultiArray`-shaped payload defensively.

**Services (provided)**

- `predict_gp` (`ros2_rad_map/GPPredict`) — build/optimize a GP from the request
  samples (or the internal buffer if the request is empty), predict mean+std at
  `points_to_predict`, and return GP diagnostics (LML, lengthscale, variance,
  noise, training/prior MSE, rho). This is the only call that actually runs
  `optimize()`, so it is also what warm-starts the cached hyperparameters.
- `find_rad_peaks` (`ros2_rad_map/FindPeaks`) — build a GP from the internal
  `historical_samples`, run `num_starts` random-restart L-BFGS-B maximizations of
  the posterior mean, reject candidates whose predictive std exceeds
  `peak_confidence_threshold`, de-duplicate within one `kernel.lengthscale`, and
  return the surviving `(x, y, value)` peaks as `geometry_msgs/Point[]`.

**Parameters** (name — type, default — description)

- `kernel.type` (string, `RBF`) — `RBF` | `Matern52` | `Matern32` | `Exponential`/`Matern12`.
- `kernel.lengthscale` (double, `5.0`) — kernel lengthscale; also the peak
  de-duplication radius.
- `fix_lengthscale` (bool, `False`) — if true, the lengthscale is fixed and not
  optimized (and warm-starting of the lengthscale is skipped).
- `sensor_frame` (string, `base_link`) / `map_frame` (string, `map`) — TF frames.
- `sparse_threshold` (int, `300`) — above this sample count, switch from exact
  `GPRegression` (O(N³)) to `SparseGPRegression` with 50 inducing points.
- `use_multi_fidelity` (bool, `True`) — use the offline prior as the GP mean
  function (ARGP mode) instead of a constant background mean.
- `model_path` (string, `''`) — path to the offline prior pickle. Resolved at
  launch to `<share>/ros2_rad_map/models/gp_model.pkl`. Empty ⇒ multi-fidelity
  silently disabled (falls back to constant-mean GP).
- `initial_rho` (double, `1.0`) — initial multi-fidelity scaling `rho`.
- `fix_rho` (bool, `False`) — fix `rho` (and the constant-mean `C` fallback).
- `low_fi_max_prior` (double, `100.0`) — cap on the prior contribution, in
  mrem/hr, applied **after** the inverse transform.
- `prior_transform` (string, `log1p`) — `log1p` | `none`. The offline pipeline
  trains on `log(1+y)`, so `log1p` applies `expm1` to bring the prior into
  mrem/hr. Use `none` only for a prior fitted directly in linear units.
- `use_discrepancy` (bool, `True`) — include the offline discrepancy GP
  (`gp_delta`, the A-11 spatial correction) in the prior mean.
- `landmark_buffer_size` (int, `500`) — cap on the high-radiation landmark buffer.
- `landmark_spatial_dist` (double, `0.2`) — merge radius (m) for landmarks.
- `peak_confidence_threshold` (double, `1.0`) — max predictive std for an
  accepted peak (overridden to `0.8` in `heatmap.yaml`).
- `rad_smoothing_window` (int, `5`) — rolling-average window on raw readings.
- `gp_noise_variance_min` (double, `0.5`) — floor on GP likelihood variance.
- `landmark_background_min` (double, `0.3`) — minimum effective background used
  for the landmark elevation test.

**Key algorithm / behavior notes**

- *Multi-fidelity / ARGP prior* (`LowFidelityPrior`, `LowFiMapping`): the
  serialized pickle holds `gp_lo` (low-fidelity field) and optionally `gp_delta`
  (discrepancy). The fused log-space mean is `mu_l + mu_delta`, back-transformed
  as `expm1(...)` with a delta-method variance correction, then floored at 0
  (a dose rate can't be negative). It is wrapped as a GPy mean function
  `rho * min(prior(x), low_fi_max_prior)` with a trainable `rho`. The class
  docstring documents two historical bugs that are now fixed: (1) the inverse
  log transform was missing, so `rho` was absorbing a unit mismatch and the
  `low_fi_max_prior` cap was comparing against log-space values; (2) `gp_delta`
  was loaded but never applied. `load_low_fi_model` warns loudly if the
  configured `prior_transform` looks inconsistent with the model's stored target
  range.
- *Noise floor & kernel-variance bound*: `optimize()` is constrained so the
  likelihood variance stays in `[gp_noise_variance_min, 1e4]` (prevents fitting
  exactly through transient spikes) and the kernel/signal variance stays in
  `[1e-3, 1e4]`. The kernel-variance bound is critical: with `fix_lengthscale`
  and `fix_rho` both true (the operational config), kernel variance is the only
  free hyperparameter, and an unbounded fit was observed to collapse it toward
  zero, exploding the predictive std (~100) so that `find_peaks` rejected every
  candidate until a later `predict_gp` call recovered the fit.
- *Warm-start cache*: `predict_gp` caches the last optimized lengthscale,
  variance, noise variance, and rho; subsequent model builds (including every
  `find_peaks` call, which never optimizes) seed from these instead of cold
  defaults.
- *Auto-fix of rho*: if the max training value is below `mean_value × 1.5`
  (i.e. no significant signal), `rho` is fixed to avoid fitting noise.
- *Cold-start fallback*: if no samples exist yet but multi-fidelity is on,
  `predict_gp`/`find_rad_peaks` fall back to predicting directly from the offline
  prior over a ±50 m default search box.
- *Environment-variable overrides* (used by the chapter-5 experiment batches;
  all default to the in-code behavior when unset):
  - `RAMONES_GP_NOISE_MIN` — overrides `gp_noise_variance_min` at runtime
    (sharpness-recovery pilot; stays within the stable regime).
  - `RAMONES_GP_CONF_THRESH` — overrides `peak_confidence_threshold` in
    `find_peaks` (co-tune the accept gate when the noise floor changes).
  - `RAMONES_GP_KERN_UNBOUNDED=1` — **disables** the kernel-variance bound to
    reproduce pre-2026-07-20 behavior (isolation experiment only; known to cause
    the std-collapse pathology above).

### rad_heatmap_node (`src/rad_heatmap_node.cpp`, node name `rad_heatmap_node`)

> Note: the node's default name used to be `dynamic_rad_heatmap_node`, so
> `params/heatmap.yaml` (keyed under `rad_heatmap_node`) only applied because
> `rad_heatmap_launch.py` renames it. The default now matches the YAML, so a bare
> `ros2 run` picks up the same parameters as the launch. Six keys the node never
> declared (`min_valid_mrem`, `min_sample_mrem`, `recent_samples_window_size`,
> `gaussian_sigma`, `gaussian_blur_size`, `zone_separation_aggressiveness`) were
> removed from that YAML section. Neither change affects the campaign stack.

Multi-threaded executor. Maintains its own background/elevation model and sample
buffer (independent of `gp_predictor_node`'s), drives the GP on a timer, and
renders all the published maps and markers.

**Subscribed topics**

- `/map` (`nav_msgs/OccupancyGrid`, transient-local reliable) — base map; defines
  the prediction extent.
- `/rad` (`std_msgs/Float64MultiArray`) — GM stream (uses element `[1]`).
- `/rad_field_server/background_radiation` (`std_msgs/Float64`) — background level.
- `/radiation_zone_markers` (`visualization_msgs/MarkerArray`) — external zone
  boundary `LINE_STRIP`s rendered onto the overlay.
- `/rad_field_server/sources` (`visualization_msgs/MarkerArray`) — **only** when
  `known_field_mode` is true; the ground-truth source markers for the oracle.

**Published topics**

- `/rad_heatmap/gp_mean_map` (`sensor_msgs/Image`, 32FC1) — the fused GP mean
  field in mrem/hr. **The primary field output of the system.**
- `/rad_heatmap/gp_std_dev` (`sensor_msgs/Image`, 32FC1) — predictive std-dev.
- `/rad_heatmap/raw_mean_map` (`sensor_msgs/Image`, 32FC1) — same data as
  `gp_mean_map` (both published from `hist_mean_occluded`).
- `/rad_heatmap/historical_mean` (Image, bgr8) — colorized mean with legend +
  zone contours.
- `/rad_heatmap/historical_std_dev` (Image, bgr8) — colorized std-dev with legend.
- `/rad_heatmap/historical_mean_overlay` (Image, bgr8) — colorized mean, no
  legend, 1:1 with the map.
- `/rad_heatmap/source_estimates` (`visualization_msgs/MarkerArray`,
  transient-local) — SPHERE + TEXT markers at connected-component peak cells.
- `/rad_heatmap/zone_boundaries` (`visualization_msgs/MarkerArray`,
  transient-local) — `LINE_STRIP` contours of elevated regions (RViz).
- `/rad_heatmap/stable_source_captured` (`geometry_msgs/PointStamped`) — emitted
  each time an elevated sample cluster is committed; the explorer uses this to
  leave radiation mode.

**Services (provided)**

- `save_rad_maps` (`std_srvs/Trigger`) — write `historical_mean.png`,
  `historical_std_dev.png`, and append a row to `gp_diagnostics.csv` in
  `output_directory`.
- `reset_rad_heatmap` (`std_srvs/Trigger`) — clear all samples, background,
  masks, markers, and publish blank maps.

**Service client**

- `predict_gp` (`ros2_rad_map/GPPredict`) — called every `1/gp_update_frequency`
  seconds over all known map cells.

**Parameters**

- `sensor_frame` (string, `base_link`) / `map_frame` (string, `map`).
- `background_threshold_mrem_hr` (double, `0.1`) — display/paint threshold below
  which cells are left transparent (`heatmap.yaml` sets `0.0`).
- `output_directory` (string, `/tmp`) — PNG/CSV output path.
- `raycasting_max_sources` (int, `5`) — declared; the visibility flood-fill uses
  at most the last 20 samples (not this value) in the current code.
- `raycasting_max_distance_m` (double, `10.0`) — max flood-fill reach from a
  sample for the visibility mask (`heatmap.yaml` sets `20.0`).
- `gp_update_frequency` (double, `0.2`) — Hz of the GP update timer (every 5 s).
- `known_field_mode` (bool, `false`) — **oracle/ablation.** When true, the
  published field is replaced by the analytic true inverse-square-capped field
  over the whole mapped extent with near-zero std, bypassing the visibility mask.
  Not an operational mode — an upper-bound diagnostic for the H1/H2 null.
- `known_field_effective_range_m` (double, `2.0`) — effective range used by the
  oracle field (should match the sensor simulator).

**Key algorithm / behavior notes**

- *Elevation model*: background mean/std are tracked over a 50-sample rolling
  buffer. The elevation threshold is `bg_mean + 2.5 × max(empirical σ, Poisson σ)`
  where Poisson σ = `sqrt(bg_mean)` (GM counts are Poisson). Readings at/below
  threshold are background anchors (stored at 1 m spacing so the GP knows where
  the field is low); readings above threshold accumulate into a pending cluster
  that is committed as one averaged sample after 5 stable readings or when the
  robot leaves a 1 m radius. Buffer capped at 1000; eviction only ever removes
  background samples, never committed elevated ones.
- *Visibility mask*: a BFS flood-fill from recent sample positions through free
  cells (bounded by `raycasting_max_distance_m`) defines where GP values are
  trusted. Free cells only get GP values if reachable; occupied (shelf) cells get
  them when predictive std < 2.0. This stops the GP from painting unexplored
  frontier hot.
- *Zone boundaries / source estimates*: threshold → `MORPH_CLOSE`/`MORPH_OPEN`
  → `findContours`/`approxPolyDP` for boundaries; `connectedComponentsWithStats`
  + per-component argmax (area ≥ 9 cells) for source estimates.
- The GP call runs on a detached worker thread guarded by `is_updating_` so
  updates never overlap.

### peak_finder_client_node (`src/peak_finder_client_node.cpp`, node name `peak_finder_client_node`)

Polls the peak-finder service, tracks peaks across calls into persistent zones,
and promotes stable ones to confirmed sources.

**Subscribed topics**

- `/rad` (`std_msgs/Float64MultiArray`) — buffers TF-tagged samples (deque capped
  at 1000) to feed the service request.
- `/rad_field_server/background_radiation` (`std_msgs/Float64`) — background level
  passed as the request `mean_value`.

**Published topics**

- `/radiation_zones` (`ros2_rad_map/RadZoneArray`) — all tracked candidate zones
  with peak value ≥ `min_peak_mrem_hr`. **Consumed by the evaluator and explorer.**
- `/radiation_peak_markers` (`visualization_msgs/MarkerArray`) — candidate SPHERE +
  TEXT markers.
- `/confirmed_rad_sources` (`ros2_rad_map/RadZoneArray`) — zones detected in the
  same place ≥ `source_confirmation_count` times.
- `/confirmed_rad_source_markers` (`visualization_msgs/MarkerArray`) — solid-red
  cylinder + annotated markers for confirmed sources.

**Service client**

- `/find_rad_peaks` (`ros2_rad_map/FindPeaks`) — called every 15 s (fixed timer).

**Parameters**

- `sensor_frame` (string, `base_link`) / `map_frame` (string, `map`).
- `num_starts` (int, `20`) — random restarts passed to the peak search
  (`heatmap.yaml` sets `30`).
- `marker_scale` (double, `0.5`) — RViz marker size.
- `matching_threshold_m` (double, `1.5`) — max distance to associate a new peak
  with an existing tracked zone.
- `min_peak_mrem_hr` (double, `0.4`) — minimum peak value to publish a zone
  (`heatmap.yaml` sets `1.5`, above the simulator's background max so noise can't
  pass).
- `source_confirmation_count` (int, `3`) — detections in the same location before
  a zone is promoted to a confirmed source (≈45 s at the 15 s interval).

**Behavior notes**: matched zones update their center by running average
(converges toward the true location) but keep the maximum value seen (never
average a confirmed source down). Zone labels are single letters `A`, `B`, ....

### rad_costmap_visualizer_node (`src/rad_costmap_visualizer_node.cpp`, node name `rad_costmap_visualizer_node`)

Converts the GP mean field into a Nav2-style radiation cost layer.

**Subscribed topics**

- `heatmap_topic` (default `/rad_heatmap/raw_mean_map`) (`sensor_msgs/Image`, 32FC1)
  — the GP mean field.
- `map_topic` (default `/map`) (`nav_msgs/OccupancyGrid`, transient-local) — for
  grid geometry.
- `/rad_heatmap/gp_std_dev` (`sensor_msgs/Image`, 32FC1) — predictive std; cells
  with std > 0.5 are treated as unknown and left `NO_INFORMATION`.

**Published topics**

- `radiation_costmap_topic` (default `/radiation_costmap`)
  (`nav_msgs/OccupancyGrid`, transient-local) — per-cell cost:
  `NO_INFORMATION (255)` where uncertain/out-of-overlap; `LETHAL_OBSTACLE (254)`
  at/above `evacuation_threshold_mrem_hr`; a graded `1..INSCRIBED (253)` value in
  the warning band between the two thresholds.
- `/radiation_cost_zones` (`visualization_msgs/MarkerArray`) — **computed but the
  publish call is commented out** (`marker_pub_->publish(...)` is disabled). The
  warning/evac CUBE_LIST markers are built but never sent; only the costmap is
  published. Dead output — noted for the handoff.

**Parameters**

- `heatmap_topic` (string, `/rad_heatmap/raw_mean_map`).
- `map_topic` (string, `/map`).
- `radiation_costmap_topic` (string, `/radiation_costmap`).
- `warning_threshold_mrem_hr` (double, `0.25`) — start of the graded warning band
  (`heatmap.yaml` sets `1.0`).
- `evacuation_threshold_mrem_hr` (double, `2.0`) — lethal-cost threshold.

**Behavior notes**: cost classification is vectorized with `cv::compare`/
`cv::bitwise_*` masks; the warning-band gradient is still a per-pixel loop because
it must truncate (not round) the `unsigned char` cast to match the original
published cost values exactly.

---

## Interfaces defined by this package

Services (`srv/`): `GPPredict`, `FindPeaks` (core); `GetHeatmaps`, `SaveMaps`,
`OrbitZone`, `GoToZone` (auxiliary nodes). Messages (`msg/`): `RadZone`
(`string label`, `geometry_msgs/Point center` — note `center.z` carries the peak
dose rate), `RadZoneArray` (`Header header`, `RadZone[] zones`).

`GPPredict` request: `geometry_msgs/Point[] points_to_predict`, `float64[]`
observed x/y/mean/std arrays, `float64 mean_value`. Response: `float64[]`
`predicted_mean`/`predicted_std_dev` plus scalar diagnostics (LML, lengthscale,
kernel variance, noise variance, training MSE, prior MSE, rho).

`FindPeaks` request: observed x/y/mean arrays, `float64 mean_value`, `int32
num_starts`. Response: `geometry_msgs/Point[] peaks` (`z` = peak value).

---

## Launch files

This package ships **no launch files**; the nodes are launched from the
system-level bringup elsewhere in the RaMonES workspace, which also resolves
`gp_predictor_node`'s `model_path` to `<share>/ros2_rad_map/models/gp_model.pkl`
and applies `params/heatmap.yaml`.

## Config / param files

- `params/heatmap.yaml` — the operational config for the four core nodes.
  Key overrides vs. in-code defaults: `gp_predictor_node` uses `Matern52`
  lengthscale `2.0` with `fix_lengthscale: True` and `fix_rho: True` (so kernel
  variance is the only free hyperparameter), `peak_confidence_threshold: 0.8`,
  `gp_noise_variance_min: 0.5`; `peak_finder_client_node` uses `num_starts: 30`
  and `min_peak_mrem_hr: 1.5`; `rad_costmap_visualizer_node` uses
  `warning_threshold_mrem_hr: 1.0`.
- `params/pointcloud.yaml`, `params/zones.yaml`, `params/zone_planner.yaml`,
  `params/predefined_zones.yaml` — for the auxiliary (non-core) nodes.
- `models/gp_model.pkl` — the offline multi-fidelity prior (`gp_lo` + `gp_delta`).

## Dependencies

From `package.xml`: build tool `ament_cmake`; interface generation via
`rosidl_default_generators`/`rosidl_default_runtime`. C++: `rclcpp`, `std_msgs`,
`nav_msgs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `std_srvs`,
`tf2`/`tf2_ros`/`tf2_geometry_msgs`, `opencv`/`cv_bridge`, `image_transport`,
`message_filters`, `pcl_conversions`/`libpcl-dev`, `octomap`/`octomap_msgs`,
`yaml-cpp`. Python runtime: `rclpy`, `numpy`, `python3-scipy`, `python3-gpy`.

## Build & run

```bash
# from the workspace root
colcon build --packages-up-to ros2_rad_map
source install/setup.bash

# GP model server (Python) — give it the offline prior
ros2 run ros2_rad_map gp_predictor_node.py \
  --ros-args --params-file src/ROS2_Rad121/ros2_rad_map/params/heatmap.yaml \
  -p model_path:=$(ros2 pkg prefix ros2_rad_map)/share/ros2_rad_map/models/gp_model.pkl

# Heatmap driver (C++) — launch named rad_heatmap_node so heatmap.yaml applies
ros2 run ros2_rad_map rad_heatmap_node \
  --ros-args -r __node:=rad_heatmap_node \
  --params-file src/ROS2_Rad121/ros2_rad_map/params/heatmap.yaml

# Peak finder client (C++)
ros2 run ros2_rad_map peak_finder_client_node \
  --ros-args --params-file src/ROS2_Rad121/ros2_rad_map/params/heatmap.yaml

# Radiation cost layer (C++)
ros2 run ros2_rad_map rad_costmap_visualizer_node \
  --ros-args --params-file src/ROS2_Rad121/ros2_rad_map/params/heatmap.yaml

# Sharper peaks (experiment): lower the noise floor via env var before launch
RAMONES_GP_NOISE_MIN=0.1 ros2 run ros2_rad_map gp_predictor_node.py ...
```

The pipeline needs live TF (`map`→`base_link`), a `/map` publisher, and the
`rad_simulation` nodes (or real hardware) producing `/rad` and
`/rad_field_server/background_radiation`.
