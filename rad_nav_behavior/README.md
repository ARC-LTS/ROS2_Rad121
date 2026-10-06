# rad_nav_behavior

Radiation-zone navigation behaviours: a zone explorer with exploration and surveillance modes plus orbital survey, and a YAML-driven zone publisher.

## Overview

This package decides *what to do about radiation sources once they are found*.
The GP mapping pipeline (`ros2_rad_map`) detects hotspots and publishes them as a
`RadZoneArray` on `/radiation_zones`; `zone_explorer_node` consumes that queue
and runs an orbital survey around each source. It operates in two modes:

- **Exploration** — frontier navigation (via `explore_lite`) runs normally. When a zone arrives, frontier nav is paused, the robot drives to the source centre, orbits it (facing inward at each waypoint), then resumes frontier exploration when the queue empties.
- **Surveillance** — no frontier navigation. The robot patrols a fixed set of known source locations, orbiting each **without** driving to the hotspot centre, relying on the radiation costmap layer to route safely between waypoints.

`zone_publisher_node` is a companion that reads source locations from YAML (or
from `rad_field_server`'s saved sources) and republishes them on
`/radiation_zones`, plus offers runtime services to jump between known zones.

Mode and the active survey can be changed at runtime via services without a
restart.

---

## Nodes

### `zone_explorer_node`
Executable `zone_explorer_node` (C++) — source `src/zone_explorer_node.cpp`.

Maintains a deduplicated queue of radiation zones and surveys them one at a time.
A 1 s timer (`processZoneQueue`) pulls the next zone when idle.

**Subscribed topics**
- `zones_topic` (default `/radiation_zones`; `ros2_rad_map/msg/RadZoneArray`) — incoming zones; deduplicated by `label` against the queue and the in-progress goal.

**Action client**
- `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) — centre goal (exploration) and each orbit waypoint. Waits for the server at startup.

**Service servers**
- `~/exploration_mode` (`std_srvs/Trigger`) — switch to exploration mode; resumes frontier nav.
- `~/surveillance_mode` (`std_srvs/Trigger`) — switch to surveillance mode; stops frontier nav.
- `~/orbit_zone` (`ros2_rad_map/srv/OrbitZone`) — queue an orbital survey of any map-frame `(x, y)` at the **front** of the queue (surveyed next); `label` optional (auto-generated `manual_<t>` if blank).
- `~/clear_queue` (`std_srvs/Trigger`) — drop all queued zones.

**Service clients**
- `explore_start_service` (default `explore_node/start`, `std_srvs/Trigger`) — resume frontier exploration.
- `explore_stop_service` (default `explore_node/stop`, `std_srvs/Trigger`) — pause frontier exploration.

**Parameters**
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `mode` | string | `exploration` | Starting mode: `exploration` or `surveillance` (any other value → exploration). |
| `zones_topic` | string | `/radiation_zones` | Topic for incoming `RadZoneArray`. |
| `survey_radius_m` | double | 1.5 | Orbit radius around each source. |
| `survey_waypoints` | int | 6 | Number of equally-spaced orbit waypoints. |
| `dwell_per_waypoint_s` | int | 2 | Pause at each waypoint before moving on (0 = no dwell). |
| `orbit_waypoint_timeout_s` | int | 60 | Per-waypoint nav timeout; on timeout the goal is cancelled and the waypoint skipped. |
| `explore_start_service` | string | `explore_node/start` | Trigger service to resume frontier nav. |
| `explore_stop_service` | string | `explore_node/stop` | Trigger service to pause frontier nav. |

**Behaviour notes**
- **Exploration flow**: on pulling a zone it calls `explore_stop` first, and only sends the centre goal once the stop confirms. On reaching the centre it runs the orbital survey; when the queue is empty afterward it calls `explore_start`. A rejected/aborted/cancelled centre goal also resumes exploration so the robot never strands.
- **Surveillance flow**: skips the centre goal and goes straight to the orbital survey; never calls `explore_start`/`explore_stop` as part of the survey.
- **Orbital survey**: waypoints are placed on a circle of `survey_radius_m`; each goal's orientation faces the source centre. A Nav2-rejected waypoint (`handle == nullptr`, e.g. goal inside a lethal obstacle) is skipped immediately. A per-waypoint timeout cancels and skips. An **external** preempt (CANCELED that is not a self-timeout, e.g. `rad_safety` cancelling the goal) aborts the whole survey and resumes exploration.
- Service-driven mode switches take effect after any in-progress survey completes.

### `zone_publisher_node`
Executable `zone_publisher_node.py` (Python) — source `scripts/zone_publisher_node.py`.

Reads known source locations from YAML and republishes them periodically (so late
subscribers still receive them), and provides services to drive the zone explorer
to specific known zones. Launched in all modes so `sync_from_field_server` is
always available.

**Published topics**
- `zones_topic` (default `/radiation_zones`; `ros2_rad_map/msg/RadZoneArray`) — the loaded zones, republished at `publish_rate_hz`. QoS depth 1 / reliable / transient-local.

**Service servers**
- `~/sync_from_field_server` (`std_srvs/Trigger`) — reload zones from `saved_sources_file` at runtime (also calls `zone_explorer_node/clear_queue` so the fresh zones are not dropped by dedup, then publishes immediately).
- `~/go_to_zone` (`ros2_rad_map/srv/GoToZone`) — orbit the known zone with the matching `label`.
- `~/next_zone` (`std_srvs/Trigger`) — orbit the next known zone (wraps around).
- `~/prev_zone` (`std_srvs/Trigger`) — orbit the previous known zone (wraps around).

**Service clients**
- `zone_explorer_node/clear_queue` (`std_srvs/Trigger`).
- `zone_explorer_node/orbit_zone` (`ros2_rad_map/srv/OrbitZone`).

**Parameters**
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `zones_file` | string | `none` | Path to a YAML file of known sources. `none`/empty → start empty and wait for a sync call. |
| `zones_topic` | string | `/radiation_zones` | Publish topic. |
| `publish_rate_hz` | double | 1.0 | Republish rate. |
| `saved_sources_file` | string | `''` | Path used by `~/sync_from_field_server`; if unset, that service has nothing to load. |

**Supported YAML formats** (auto-detected by top-level key)
- **zones list** (`params/known_sources.yaml`): `zones: [{label, x, y}, ...]`.
- **rad_field_server saved sources** (`rad_simulation/params/saved_sources.yaml`): `rad_field_server.ros__parameters.known_sources: [x, y, intensity, ...]` (flat triplets; intensity ignored, labels auto-generated `Source_N`).

---

## Launch files

### `launch/zone_explorer.launch.py`
Brings up `zone_explorer_node` (exploration or surveillance variant, selected by
condition) plus `zone_publisher_node` (always). Hard-codes
`survey_radius_m=1.5`, `survey_waypoints=6`, `dwell_per_waypoint_s=2`,
`zones_topic=/radiation_zones`; exploration variant also sets the explore
start/stop service names. The publisher's `saved_sources_file` defaults to
`rad_simulation/params/saved_sources.yaml`.

| Arg | Default | Description |
| --- | --- | --- |
| `mode` | `exploration` | `exploration` = frontier nav + survey of discovered zones; `surveillance` = patrol known locations from `zones_file`. |
| `zones_file` | `none` | Path to a known-sources YAML (surveillance); `none` starts empty. |

```bash
ros2 launch rad_nav_behavior zone_explorer.launch.py mode:=exploration
ros2 launch rad_nav_behavior zone_explorer.launch.py mode:=surveillance zones_file:=/path/to/sources.yaml
```

(The main stack launch, `rad_launch rad_launch_sim_launch.py`, instead launches
`zone_explorer_node` + `zone_publisher_node` inline, gated on
`orbit_surveys:=true`.)

### Runtime service examples
```bash
ros2 service call /zone_explorer_node/surveillance_mode std_srvs/srv/Trigger "{}"
ros2 service call /zone_explorer_node/orbit_zone ros2_rad_map/srv/OrbitZone "{label: 'A', x: -4.17, y: -12.87}"
ros2 service call /zone_publisher_node/sync_from_field_server std_srvs/srv/Trigger "{}"
ros2 service call /zone_publisher_node/next_zone std_srvs/srv/Trigger "{}"
```

## Config files
- `params/known_sources.yaml` — example zones-list file (3 labelled sources) for surveillance mode.

## Dependencies
From `package.xml`: `rclcpp`, `rclcpp_action`, `ros2_rad_map`, `geometry_msgs`,
`nav2_msgs`, `std_srvs`, `rclpy`. Build type `ament_cmake` (C++ node built with
CMake; the Python script installed via `install(PROGRAMS ...)`). The C++ node
links `ros2_rad_map::ros2_rad_map__rosidl_typesupport_cpp` for the custom
messages/services.

Note: `package.xml` still has placeholder `description`/`license`/`maintainer`
fields (`TODO`), harmless for build.

## Build & run
```bash
colcon build --packages-select rad_nav_behavior
source install/setup.bash
ros2 launch rad_nav_behavior zone_explorer.launch.py mode:=exploration
```
