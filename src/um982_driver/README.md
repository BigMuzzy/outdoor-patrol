# um982_driver

ROS 2 driver for the **Unicore UM982** dual-antenna RTK GNSS module
(also shipped as the **WitMotion WTRTK-982**).

## Status — implemented

Managed `LifecycleNode` with a background serial I/O thread, NMEA +
Unicore/KSXT parsing, RTCM injection, and a `diagnostic_updater` health
channel. On `configure` it opens the port and (optionally) clears the
receiver log set; on `activate` it sends the mode-specific init commands
and starts publishing.

## Topics

All outputs are in the node's private namespace (default node name
`um982_driver`, e.g. `/um982_driver/fix`).

| Topic              | Type                                          | Notes |
|--------------------|-----------------------------------------------|-------|
| `~/fix`            | `sensor_msgs/NavSatFix`                       | Covariance seeded from fix quality / HDOP. |
| `~/fix_velocity`   | `geometry_msgs/TwistWithCovarianceStamped`    | From RMC/VTG ground speed + course. |
| `~/heading`        | `geometry_msgs/QuaternionStamped`             | Dual-antenna baseline heading (KSXT). |
| `~/nmea_sentence`  | `nmea_msgs/Sentence`                          | Raw GGA, consumed by `ntrip_client` for VRS upload. |
| `~/time_reference` | `sensor_msgs/TimeReference`                   | GNSS time vs. receive time. |
| `rtcm/in` (sub)    | `rtcm_msgs/Message`                           | Correction stream injected to the receiver. |

## Key parameters

See [`config/um982_rover.yaml`](config/um982_rover.yaml). Highlights:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `port` / `baudrate` | `/dev/ttyUSB0` / `115200` | Serial connection. |
| `frame_id` / `heading_frame_id` | `gnss_link` | TF frames stamped on outputs. |
| `mode` | `rover` | `rover` \| `base_fixed` \| `base_survey` \| `heading2`. |
| `base_lat/lon/height`, `survey_seconds`, `survey_dist_m` | — | Base-station setup (`base_fixed` / `base_survey`). |
| `rtcm_ids`, `rtcm_period_s`, `rtcm_out_com` | — | RTCM message set the base emits. |
| `output_messages.names`, `output_messages.period_s`, `output_messages.com` | `[GPGGA,GPRMC,GPVTG,KSXT]`, `0.2`, `""` | Rover output set. NMEA names are **GP-prefixed** (`GPGGA`…); empty `com` = current port (the USB link). |
| `antenna_h/e/n` | `0` | Antenna lever-arm offsets. |
| `unlogall_on_configure` | `true` | Clear the receiver log set on configure. |
| `save_config` | `false` | Persist config to receiver NVM (`SAVECONFIG`). |

## Reference docs

See [docs/README.md](docs/README.md). The WTRTK-982 and Unicore N4
manuals live in that folder.

## Run (driver only)

```bash
ros2 launch um982_driver um982.launch.py            # does not auto-activate
ros2 lifecycle set /um982_driver configure
ros2 lifecycle set /um982_driver activate
```

## Run (rover + RTK corrections)

To bring up the rover together with an NTRIP correction source and the
VRS GGA-upload loop pre-wired, use the combined launch:

```bash
ros2 launch um982_driver gnss_rtk.launch.py \
  ntrip_params_file:=/path/to/ntrip.yaml \
  um982_params_file:=/path/to/um982_rover.yaml
```

See [`launch/gnss_rtk.launch.py`](launch/gnss_rtk.launch.py) for the topic
remaps and the `auto_activate` argument.

## RTK integration test (dev container)

End-to-end check that the rover reaches an RTK fix using live
corrections. Runs in the ROS 2 Jazzy dev container (no host ROS install).

1. Copy `config/ntrip.yaml.example` → `ntrip.yaml`, fill in caster
   credentials + mountpoint, set the UM982 `port` in `um982_rover.yaml`.
2. Launch the combined stack:

   ```bash
   ros2 launch um982_driver gnss_rtk.launch.py \
     ntrip_params_file:=/abs/ntrip.yaml \
     um982_params_file:=/abs/um982_rover.yaml
   ```
3. Acceptance:
   - `ros2 topic echo /rtcm` shows correction frames arriving (RTCM in).
   - `ros2 topic echo /um982_driver/nmea_sentence` shows GGA going out
     (VRS upload loop closed).
   - `ros2 topic echo /um982_driver/fix` → `status.status == 2`
     (RTK fix) within a couple of minutes under open sky.
   - `ros2 topic echo /um982_driver/heading` yields a stable quaternion
     (dual-antenna baseline locked).
   - `ros2 topic echo /diagnostics` reports OK with low correction age.

## Tests

C++ unit tests cover the NMEA, Unicore, command-builder and
frame-splitter logic. `test/smoke_pty.py` feeds synthetic NMEA through a
PTY and asserts `~/fix` is published. Build and run with
`colcon test --packages-select um982_driver` inside the dev container.
