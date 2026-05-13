# arch_nav_mavsdk_px4_driver

![License](https://img.shields.io/github/license/mdominmo/arch_nav_mavsdk_px4_driver) ![Version](https://img.shields.io/github/v/release/mdominmo/arch_nav_mavsdk_px4_driver)

[arch-nav](https://github.com/mdominmo/arch-nav) driver that connects to PX4-based autopilots using [MAVSDK](https://mavsdk.mavlink.io).

## Features

- **Takeoff / Land** via MAVSDK Action plugin
- **Waypoint following** via MAVSDK Mission plugin (or MissionRaw when ROI is active)
- **ROI (Region of Interest)** — vehicle nose tracks a fixed geographic point during waypoint missions
- **Yaw control** (absolute LOCAL_NED/LOCAL_ENU and relative BODY_FCS)
- **Arm / Disarm** control
- **Telemetry** feedback (position, kinematics, vehicle status) streamed to the arch-nav kernel
- Pure C++17, no ROS dependency

## PX4-specific behaviour

- Takeoff completion is detected by the flight mode transition `Takeoff → Hold`.
- Yaw changes use `goto_location` at the current position and monitor heading with a 3° tolerance.
- Large body-relative rotations are broken into 80° steps to ensure deterministic direction.
- A 45-second timeout is applied to yaw operations.
- **ROI + waypoints**: when a ROI is active the driver switches from the high-level `Mission` plugin to `MissionRaw` and uploads `MAV_CMD_DO_SET_ROI_LOCATION` (cmd 195) as the first mission item, followed by the waypoints. This is the correct mechanism to make PX4 maintain vehicle yaw toward the ROI in Mission mode — standalone `DO_SET_ROI_LOCATION` commands are ignored by PX4 once a mission starts.
- `execute_set_roi` sends `MAV_CMD_DO_SET_ROI_LOCATION` via `CommandInt` with `MAV_FRAME_GLOBAL_INT` and int32 lat/lon (×1e7).
- `execute_clear_roi` sends `MAV_CMD_DO_SET_ROI_NONE` (cmd 197) via `CommandLong`.
- ROI state is written optimistically to `VehicleContext` immediately after the MAVLink command succeeds (no autopilot ACK wait).

## Supported reference frames

| Command | Frames |
|---------|--------|
| Takeoff | `LOCAL_NED` |
| Change yaw | `LOCAL_NED`, `LOCAL_ENU`, `BODY_FCS` |
| Waypoint following | `GLOBAL_WGS84` |
| Set ROI | `GLOBAL_WGS84` |

## Configuration

The driver reads an optional YAML config file:

```yaml
# mavsdk_config.yaml
connection_url: "udpin://0.0.0.0:14541"
discover_timeout_s: 5.0
default_speed_m_s: 5.0
mission_upload_delay_ms: 1000
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `connection_url` | string | `"udp://localhost:14581"` | MAVSDK connection string to the autopilot |
| `discover_timeout_s` | double | `15.0` | Seconds to wait for autopilot discovery after connection |
| `default_speed_m_s` | float | `5.0` | Cruise speed used for waypoint missions (m/s) |
| `mission_upload_delay_ms` | int | `1000` | Delay (ms) between mission upload and mission start |

Set the config path in `arch_nav_config.yaml`:

```yaml
driver: "mavsdk_px4"
driver_config_path: "/path/to/mavsdk_config.yaml"
```

If `driver_config_path` is empty, the driver uses its built-in defaults.

### Connection URL examples

| Scenario | `connection_url` |
|----------|-----------------|
| PX4 SITL (listen, default) | `udpin://0.0.0.0:14541` |
| PX4 SITL (connect) | `udp://localhost:14540` |
| Serial link (hardware) | `serial:///dev/ttyUSB0:921600` |

## Prerequisites

- [MAVSDK](https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html) C++ library
- [arch-nav](https://github.com/mdominmo/arch-nav) kernel installed

## Build

This driver is compiled together with the arch-nav kernel. See the [arch-nav build instructions](https://github.com/mdominmo/arch-nav#build-and-install).

The build produces a shared library (`libarch_nav_mavsdk_px4.so`) installed to `<prefix>/lib/arch_nav/drivers/`, which arch-nav loads dynamically at startup.

## Writing your own driver

See the [arch-nav driver guide](https://github.com/mdominmo/arch-nav#writing-a-driver) for how to implement and register a custom driver.
