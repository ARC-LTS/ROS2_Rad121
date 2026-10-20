# rad121_monitor

ROS 2 driver for the Teviso/Images SI C-USB-RAD121 Geiger counter: reads pulse counts over FTDI USB and publishes a compensated dose rate.

## Overview

This is the **real-hardware sensing driver**. It talks to a C-USB-RAD121 tube
over an FTDI serial link (libftdi1), counts pulses in a sliding one-second
window, applies dead-time and sensitivity corrections, and publishes the result
on `/rad` — the same topic the simulated sensor (`rad_simulation`) publishes, so
the downstream mapping/recovery/evaluation stack is agnostic to whether readings
come from hardware or simulation.

It is only needed when running against the physical RAD121 sensor; for simulation
trials it is replaced by `rad_simulation`'s `rad_sensor_simulator`.

On startup it can also perform an auto-calibration pass that measures the ambient
background level and publishes it once as a latched value for the GP pipeline.

---

## Nodes

### `rad121_monitor_node`
Executable `rad121_monitor_node` — sources `src/rad121_monitor_node.cpp` (ROS
node) and `src/CUSB_RAD121.cpp` (FTDI driver + dose math), header
`include/CUSB_RAD121.h`.

Opens the FTDI device (VID `0x0403`, PID `0x6001`, 921600 baud, 8N1) and samples
at 1 Hz. `init()` must run after construction (it calls `shared_from_this()`), as
it does in `main()`.

**Published topics**
- `rad` → `/rad` (`std_msgs/Float64MultiArray`) — two elements: `[0]` = dead-time-compensated CPM, `[1]` = dose rate in mR/hr. Published every second. (Downstream nodes read element `[1]`.)
- `/rad_field_server/background_radiation` (`std_msgs/Float64`) — the measured background level, published **once** when auto-calibration completes. QoS depth 1 / transient-local (latched) so late subscribers still get it.

**Parameters**
| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `auto_calibrate` | bool | true | If true, average the first `calibration_duration_s` of readings and publish the mean as the background level. |
| `calibration_duration_s` | int | 30 | Auto-calibration window length (seconds). |
| `usb_rad121.dead_time` | double | 0.00015 | Detector dead time (seconds); used for the non-paralysable CPM correction `cpm / (1 − cps·dead_time)`. Declared by the driver via `ConfigureFromNode`. |
| `usb_rad121.sensitivity` | double | 450.0 | Tube sensitivity in CPM per mR/hr; `mR/hr = compensated_cpm / sensitivity`. |

`params/sensor.yaml` sets `usb_rad121.dead_time` and `usb_rad121.sensitivity`
under `rad121_monitor_node`, matching the names the driver declares. Its values
equal the compiled defaults (0.00015 s, 450.0 CPM/mR·hr⁻¹); edit it to recalibrate.

**Behaviour notes**
- Pulse timestamps are kept in a deque; CPS = count of timestamps within the last second, CPM = CPS × 60.
- The node has an unused `decayRate (0.99)` member and `updateInterval (1)` (the latter only sets the 1 s timer period).
- A read error or failed device open calls `rclcpp::shutdown()`.

---

## Launch files
No launch files in this package. Launch lives in `rad_launch`:
- `rad_launch/launch/rad121_monitor_launch.py` launches `rad121_monitor_node` with `params/sensor.yaml`. It takes no launch arguments.

## Config files
- `params/sensor.yaml` — `usb_rad121.dead_time: 0.00015`, `usb_rad121.sensitivity: 450.0`.

## Known mismatches
- **Parameter-prefix mismatch (fixed)**: `params/sensor.yaml` used unprefixed keys (`dead_time`, `sensitivity`) that the driver never read. The keys are now prefixed (`usb_rad121.*`); because the values equal the compiled defaults, the sensor's dose math is unchanged.

## Dependencies
From `package.xml`: `rclcpp`, `std_msgs`. Plus a system dependency on
**libftdi1** (found via `pkg_check_modules(LIBFTDI REQUIRED libftdi1)` in
CMake and linked directly — not declared in `package.xml`, so install it
manually: `sudo apt install libftdi1-dev`). Build type `ament_cmake`, C++17.

## Build & run
```bash
sudo apt install libftdi1-dev        # system dependency, not in rosdep
colcon build --packages-select rad121_monitor
source install/setup.bash

# requires the RAD121 plugged in and FTDI USB permissions (dialout group / udev rule)
ros2 launch rad_launch rad121_monitor_launch.py
# or
ros2 run rad121_monitor rad121_monitor_node --ros-args --params-file \
  $(ros2 pkg prefix rad121_monitor)/share/rad121_monitor/params/sensor.yaml
```
