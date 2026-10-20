# rad_simulation

Simulated ground-truth radiation field and GM-sensor stream for RaMonES.

## Overview

`rad_simulation` provides the ground truth that the rest of the stack estimates
and is scored against. It has two cooperating nodes:

- **`rad_field_server`** owns the set of point sources (the ground truth). It
  spawns them (from a known list, randomly on the map, at the robot, or by RViz
  click), publishes them as RViz markers on `/rad_field_server/sources` (the
  topic the evaluator and the heatmap oracle read for ground truth), publishes
  the raw source data on `/rad_field_server/sources_data`, publishes a background
  level on `/rad_field_server/background_radiation`, and can save/load source
  layouts to/from YAML. It does **not** itself produce the `/rad` stream.
- **`rad_sensor_simulator`** renders what a GM counter at the robot's pose would
  read. It subscribes to the source data, evaluates the inverse-square-capped
  point-source model at the robot position, adds a deterministic spatial
  background, and publishes the result on `/rad`.

A third helper, **`source_selector_node`**, lets an operator define sources by
clicking "Publish Point" in RViz.

> Architecture note for the handoff: the `/rad` sensor stream is published by
> `rad_sensor_simulator`, not by `rad_field_server`. The field model (the actual
> `I(r) = I₀ · effective_range²/r²` with capping) is evaluated in the sensor
> simulator. The field server only holds and broadcasts the source set and a
> background scalar.

### Field and background models

- **Source dose** (sensor simulator): for each source,
  `reading += intensity · effective_range² / r²`, where `r²` is clamped to
  `max(effective_range², min_sensing_distance²)` so that within `effective_range`
  the reading is capped at the source intensity and beyond it falls off as
  inverse-square. Summed over all sources. Published as
  `data = [total·450 (dummy CPM), total (mrem/hr)]`.
- **Background** (sensor simulator, `get_spatial_background`): a *deterministic*
  multi-scale sine/cosine sum of `(x, y)` normalized into
  `[background_radiation_min, background_radiation_max]`. This is spatial (no
  temporal noise), so the same location always reads the same background.
- **Background scalar** (field server, `publish_background`): a *random* draw in
  `[min, max]` published once per second on `/rad_field_server/background_radiation`.
  This scalar is consumed by the GP/heatmap nodes as their prior-mean background
  — it is a **different quantity** from the sensor simulator's spatial background
  that actually shapes `/rad`.

> Note: `rad_sensor_simulator` does **not** consume
> `/rad_field_server/background_radiation`. `/rad` uses the sensor simulator's own
> deterministic `get_spatial_background`, which is what every campaign trial
> measured. (It used to subscribe to and store the field server's scalar without
> ever reading it; that dead subscription has been removed with no change to `/rad`.)
> The two nodes' background settings are independent and must be kept consistent
> by hand if the GP prior mean should match the simulated field.

---

## Nodes

### rad_field_server (`src/rad_field_server.cpp`, node name `rad_field_server`)

**Subscribed topics**

- `/map` (`nav_msgs/OccupancyGrid`, transient-local reliable) — on first receipt
  it spawns the initial sources (known + random). Random placement rejects
  non-free cells and anything within `min_source_distance_from_robot_m` of the
  robot's current TF pose.

**Published topics** (all namespaced under the node, i.e. `/rad_field_server/...`)

- `~/sources` (`visualization_msgs/MarkerArray`, 1 Hz) — SPHERE markers, one per
  source. Marker scale encodes intensity: `scale = 0.2 + norm·0.8` where
  `norm = (intensity − low_intensity_min)/(high_intensity_threshold − low_intensity_min)`
  clamped to `[0,1]`; color ramps green→red. **The evaluator and the heatmap
  oracle invert this scale to recover intensity**, so the normalization constants
  are a cross-package contract.
- `~/background_radiation` (`std_msgs/Float64`, 1 Hz) — background scalar (random
  in range, or `0.0` when disabled).
- `~/sources_data` (`rad_simulation/RadSourceArray`, 1 Hz) — raw id/position/
  intensity for every source (empty when `enable_sources` is false); consumed by
  the sensor simulator.

**Services (provided)** (all namespaced `/rad_field_server/...`)

- `~/spawn_source` (`rad_simulation/SpawnSource`) — add a source at `(x, y)` with
  `intensity`.
- `~/spawn_random_sources` (`rad_simulation/SpawnRandomSources`) — add
  `num_sources` random sources (high-intensity count from the parameter).
- `~/spawn_at_robot` (`std_srvs/Trigger`) — add a source at the robot pose with a
  random intensity in `[2.5, 10.0]`.
- `~/enable_sources` (`std_srvs/SetBool`) — toggle source emission.
- `~/enable_background` (`std_srvs/SetBool`) — toggle background emission.
- `~/set_background_level` (`rad_simulation/SetFloat64`) — pin background to a
  fixed value (sets both min and max).
- `~/clear_sources` (`std_srvs/Trigger`) — remove all sources.
- `~/save_sources` (`std_srvs/Trigger`) — write current sources as a flat
  `known_sources: [x,y,intensity,...]` list to `output_yaml_file`.
- `~/load_sources` (`std_srvs/Trigger`) — replace sources from `output_yaml_file`
  and enable them.

**Parameters**

- `map_frame` (string, `map`) / `robot_frame` (string, `base_link`).
- `enable_sources` (bool, `true`).
- `known_sources` (double[], `[]`) — flat `[x, y, intensity, ...]` triples spawned
  at startup.
- `num_unknown_sources` (int, `0`) — random sources spawned at startup.
- `num_high_intensity_sources` (int, `0`) — how many of those are high-intensity.
- `high_intensity_threshold` (double, `10.0`) — high-intensity sources drawn
  uniformly in `[threshold, 2·threshold]`.
- `low_intensity_min` (double, `1.0`) / `low_intensity_max` (double, `3.0`) —
  range for low-intensity random sources.
- `background_radiation_enabled` (bool, `true`).
- `background_radiation_min` (double, `0.05`) / `background_radiation_max`
  (double, `1.0`).
- `min_source_distance_from_robot_m` (double, `2.0`) — clearance from the robot's
  spawn pose for random placement (up to 100 placement attempts per source).
- `output_yaml_file` (string, `<share>/rad_simulation/params/saved_sources.yaml`)
  — save/load target.

### rad_sensor_simulator (`src/rad_sensor_simulator.cpp`, node name `rad_sensor_simulator`)

**Subscribed topics**

- `/rad_field_server/sources_data` (`rad_simulation/RadSourceArray`) — the source
  set to render.

**Published topics**

- `/rad` (`std_msgs/Float64MultiArray`, at `publish_rate_hz`) —
  `[dummy_cpm, dose_mrem_hr]`. **The GM stream the whole stack consumes.**

**Parameters**

- `map_frame` (string, `map`) / `robot_frame` (string, `base_link`).
- `publish_rate_hz` (double, `10.0`) — sensor update / publish rate.
- `min_sensing_distance_m` (double, `0.58`) — fallback near-field clamp if
  `effective_range_m` is 0.
- `effective_range_m` (double, `2.0`) — distance at which full source intensity is
  read; the inverse-square cap radius. (`params/sensor_simulator.yaml` sets `2.0`.)
- `background_radiation_enabled` (bool, `true`).
- `background_radiation_min` (double, `0.05`) / `background_radiation_max`
  (double, `1.0`) — range of the deterministic spatial background. Live-tunable
  via a parameter callback (values are cached off the 10 Hz hot path).

### source_selector_node (`src/source_selector_node.cpp`, node name `source_selector_node`)

Interactive source authoring from RViz.

**Subscribed topics**

- `clicked_point_topic` (default `/clicked_point`) (`geometry_msgs/PointStamped`)
  — each clicked point becomes a source at `default_intensity`; if the field
  server's `/rad_field_server/spawn_source` service is ready, it is pushed live.

**Services (provided)**

- `~/save_sources` (`std_srvs/Trigger`) — write the locally collected points as a
  `known_sources` list to `output_yaml_file`.
- `~/clear_sources` (`std_srvs/Trigger`) — clear the local list.

**Service client**

- `/rad_field_server/spawn_source` (`rad_simulation/SpawnSource`).

**Parameters**

- `clicked_point_topic` (string, `/clicked_point`).
- `output_yaml_file` (string, `<share>/rad_simulation/params/manual_sources.yaml`).
- `default_intensity` (double, `5.0`).

---

## Interfaces defined by this package

Messages (`msg/`): `RadSource` (`int32 id`, `geometry_msgs/Point position`,
`float64 intensity`), `RadSourceArray` (`RadSource[] sources`).

Services (`srv/`): `SpawnSource` (`x, y, intensity` → `success, message`),
`SpawnRandomSources` (`int32 num_sources` → `success, message`), `SetFloat64`
(`float64 data` → `success, message`).

---

## Launch files

This package ships **no launch files**; `rad_field_server` and
`rad_sensor_simulator` are started by the system-level bringup elsewhere in the
RaMonES workspace, with `params/sensor_simulator.yaml`.

## Config / param files

- `params/sensor_simulator.yaml` — the operational config. Sets
  `rad_sensor_simulator.effective_range_m: 2.0` and configures `rad_field_server`
  for the default experiment (`num_unknown_sources: 0`,
  `num_high_intensity_sources: 0`, `low_intensity_min/max: 1.0/3.0`,
  `background_radiation_enabled: true`). Also carries a `battery_simulator`
  section for a node that lives in another package.
- `params/saved_sources.yaml` — default save/load target; a flat `known_sources`
  list (the checked-in copy is a 5-source layout).
- `params/hb_saved_sources.yaml`, `params/rect_saved_sources.yaml`,
  `params/wh_saved_sources.yaml` — alternative source layouts for different map
  environments, loadable by pointing `output_yaml_file` at them (or copying over
  `saved_sources.yaml`).

## Dependencies

From `package.xml` / `CMakeLists.txt`: build tool `ament_cmake`; interfaces via
`rosidl_default_generators`/`rosidl_default_runtime`; `rclcpp`, `std_srvs`,
`geometry_msgs`, `visualization_msgs`, `std_msgs`, `tf2_ros`,
`tf2_geometry_msgs`, `nav_msgs`, `ament_index_cpp`, and `yaml-cpp` (linked for
the save/load services).

> Note: `package.xml` carries placeholder `description`/`license`/`version`
> (`TODO`/`0.0.0`). `nav_msgs`, `ament_index_cpp`, and `yaml-cpp` are used in
> CMake but not declared as `<depend>`s in `package.xml` — worth reconciling for
> the handoff.

## Build & run

```bash
colcon build --packages-up-to rad_simulation
source install/setup.bash

# Ground-truth field server + GM sensor simulator
ros2 run rad_simulation rad_field_server \
  --ros-args --params-file src/ROS2_Rad121/rad_simulation/params/sensor_simulator.yaml
ros2 run rad_simulation rad_sensor_simulator \
  --ros-args --params-file src/ROS2_Rad121/rad_simulation/params/sensor_simulator.yaml

# (optional) author sources by clicking in RViz
ros2 run rad_simulation source_selector_node

# Load a saved layout at runtime
ros2 service call /rad_field_server/load_sources std_srvs/srv/Trigger '{}'

# Spawn one source manually
ros2 service call /rad_field_server/spawn_source rad_simulation/srv/SpawnSource \
  '{x: 2.0, y: -1.5, intensity: 8.0}'
```

Both nodes require live TF (`map`→`base_link`) and a `/map` publisher; the field
server spawns its initial sources on the first `/map` message.
