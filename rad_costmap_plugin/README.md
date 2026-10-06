# rad_costmap_plugin

A Nav2 costmap layer that injects the predicted radiation field into the navigation costmap, so the planner routes around high-dose areas.

## Overview

This package provides a single `nav2_costmap_2d::Layer` plugin,
`rad_costmap_plugin::RadiationLayer`. It subscribes to a radiation heatmap image
(the GP mean field from `ros2_rad_map`), bilinearly samples it at each costmap
cell within a radius of the robot, and raises the master costmap's cost in
proportion to predicted dose. This is the mechanism that makes paths through
elevated radiation *expensive* (RaMonES arm); in the Baseline arm the layer is
simply left out of the Nav2 costmap plugin list so radiation never enters
navigation cost.

It is loaded by Nav2 (pluginlib), not run as a standalone node — it has no
`main()` and no executable. Configuration is per-instance, under the layer's name
in the Nav2 costmap parameter tree.

---

## Plugin

### `rad_costmap_plugin::RadiationLayer`
Source `src/RadiationLayer.cpp`, header `include/rad_costmap_plugin/RadiationLayer.h`.
Exported via `plugins.xml` with base class `nav2_costmap_2d::Layer`.

**Subscribed topics**
- `heatmap_topic` (default `/rad_heatmap/raw_mean_map`; `sensor_msgs/Image`, `32FC1`) — the predicted radiation field in mrem/hr. QoS KeepLast(1). Converted to `cv::Mat` via `cv_bridge`.
- `/map` (`nav_msgs/OccupancyGrid`) — SLAM map; QoS KeepLast(1) / transient-local / reliable. Only its `MapMetaData` (resolution/origin/size) is used, to map costmap cells to heatmap pixels.

**Parameters** (declared under `<layer_name>.`; defaults as in `onInitialize`)
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `heatmap_topic` | string | `/rad_heatmap/raw_mean_map` | Radiation-field image topic to subscribe to. |
| `warning_threshold_mrem_hr` | double | 0.25 | Dose below this adds **no** cost; the lower end of the cost ramp. |
| `evacuation_threshold_mrem_hr` | double | 2.0 | Dose at or above this is clamped to `max_radiation_cost`; the upper end of the ramp. |
| `max_radiation_cost` | int | 250 | Maximum injected cost (stored as `unsigned char`; 254 = lethal in Nav2 terms). |

**Cost model** (`updateCosts`)
- For each cell in the update window, world → heatmap-pixel transform, then **bilinear interpolation** of the four surrounding heatmap pixels (avoids voxelised, blocky paths).
- `radiation <= warning_threshold` → no change.
- `warning_threshold < radiation < evacuation_threshold` → cost ramps linearly: `1 + scale * (max_cost - 2)` where `scale = (radiation - warning) / (evacuation - warning)`.
- `radiation >= evacuation_threshold` → cost = `max_radiation_cost`.
- The injected cost is combined with the existing cell cost via `max(current, radiation_cost)`, so the layer only ever *raises* cost.
- Existing `LETHAL_OBSTACLE` cells are left untouched; negative (unknown) radiation leaves the cell traversable.

**Performance / behaviour notes**
- `updateBounds` restricts updates to a **10 m radius around the robot** (hard-coded `update_radius = 10.0`), clamped to map bounds — this is the key optimisation that keeps the control loop at 10–20 Hz instead of recomputing the whole map every cycle.
- `reset()` clears the cached heatmap to force a refresh; `isClearable()` returns `false` (the layer is not a clearable obstacle source).
- Heatmap and map are cached under separate mutexes; `updateCosts` no-ops until both a heatmap and a map have arrived.

---

## Launch files
None. The layer is loaded by Nav2 via its costmap configuration (it is not
launched directly). To enable it, add `rad_layer` (or any instance name) to a
costmap's `plugins` list and set the parameters under that name, e.g.:
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "rad_layer"]
      rad_layer:
        plugin: "rad_costmap_plugin::RadiationLayer"
        heatmap_topic: "/rad_heatmap/raw_mean_map"
        warning_threshold_mrem_hr: 0.25
        evacuation_threshold_mrem_hr: 2.0
        max_radiation_cost: 250
```
(The Nav2 parameter files that actually wire this in live outside this
repository, in the robot bring-up package — e.g. `clearpath_nav2_demos`.)

## Config files
None in this package (parameters are declared in code and set via the host Nav2
config). `plugins.xml` is the pluginlib export descriptor, not a tunable config.

## Dependencies
From `package.xml`: `rclcpp`, `nav2_costmap_2d`, `pluginlib`, `sensor_msgs`,
`cv_bridge`, `nav_msgs`. Build type `ament_cmake`; built as a `SHARED` library and
exported with `pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)`.

## Build & run
```bash
colcon build --packages-select rad_costmap_plugin
source install/setup.bash
# Verify the plugin is discoverable:
ros2 run nav2_costmap_2d ... # (loaded automatically when a costmap lists it)
```
The layer is exercised whenever Nav2 brings up a costmap whose `plugins` list
includes `rad_costmap_plugin::RadiationLayer`. In the RaMonES arm this is present;
in the Baseline arm it is omitted.
