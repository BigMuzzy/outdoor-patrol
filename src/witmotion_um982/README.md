# witmotion_um982

ROS 2 driver for the WitMotion RTK GNSS/INS module based on the Unicore
**UM982** dual-antenna receiver.

## Status

Boilerplate scaffold. The driver node currently opens no serial port and
publishes no data; protocol parsing will be added after the datasheet
and manual are imported under [docs/](docs/README.md).

## Planned topics

| Topic               | Type                          | Notes                     |
|---------------------|-------------------------------|---------------------------|
| `gnss/fix`          | `sensor_msgs/NavSatFix`       | Position + covariance     |
| `gnss/heading_deg`  | `std_msgs/Float64`            | Dual-antenna heading      |
| `imu/data`          | `sensor_msgs/Imu`             | Onboard IMU / INS output  |

## Parameters

See [config/um982.yaml](config/um982.yaml).

## Run

```bash
ros2 launch witmotion_um982 um982.launch.py
```
