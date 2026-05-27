# um982_driver

ROS 2 driver for the **Unicore UM982** dual-antenna RTK GNSS module
(also shipped as the **WitMotion WTRTK-982**).

## Status — Phase A scaffold

The driver is a stub `LifecycleNode` that declares parameters, publishers
and an `rtcm/in` subscription. Serial I/O and Unicore/NMEA parsing land
in Phase C.

## Topics (planned)

| Topic              | Type                                          |
|--------------------|-----------------------------------------------|
| `~/fix`            | `sensor_msgs/NavSatFix`                       |
| `~/fix_velocity`   | `geometry_msgs/TwistWithCovarianceStamped`    |
| `~/heading`        | `geometry_msgs/QuaternionStamped`             |
| `~/nmea_sentence`  | `nmea_msgs/Sentence` (raw GGA for NTRIP VRS)  |
| `~/time_reference` | `sensor_msgs/TimeReference`                   |
| `rtcm/in` (sub)    | `rtcm_msgs/Message`                           |

## Reference docs

See [docs/README.md](docs/README.md). The WTRTK-982 and Unicore N4
manuals live in that folder.

## Run

```bash
ros2 launch um982_driver um982.launch.py
ros2 lifecycle set /um982_driver configure
ros2 lifecycle set /um982_driver activate
```
