# ntrip_client

Generic RTCM3 source nodes for ROS 2. Each node publishes validated
RTCM3 frames on a common topic (`rtcm/out`, `rtcm_msgs/Message`) so any
GNSS driver can consume them.

## Status — Phase A scaffold

Three executables and a shared framing library are wired up but contain
no I/O yet. Implementations land in Phase B.

| Executable                | Purpose                                            |
|---------------------------|----------------------------------------------------|
| `ntrip_client_node`       | NTRIP v1/v2 caster client (GGA upload, reconnect). |
| `tcp_rtcm_relay_node`     | Raw RTCM3 over TCP (client **and** server roles).  |
| `serial_rtcm_relay_node`  | UART → RTCM (for radio modems on the rover).       |

## Configuration

- [config/ntrip.yaml.example](config/ntrip.yaml.example)
- [config/local_base_tcp.yaml](config/local_base_tcp.yaml)
- [config/local_base_serial.yaml](config/local_base_serial.yaml)

Copy `ntrip.yaml.example` to a file kept out of version control before
filling in credentials.

## Run

```bash
ros2 launch ntrip_client ntrip_client.launch.py params_file:=/path/to/ntrip.yaml
ros2 launch ntrip_client tcp_rtcm_relay.launch.py
ros2 launch ntrip_client serial_rtcm_relay.launch.py
```
