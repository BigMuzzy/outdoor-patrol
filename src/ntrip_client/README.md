# ntrip_client

Generic RTCM3 source nodes for ROS 2. Each node publishes validated
RTCM3 frames on a common topic (`rtcm/out`, `rtcm_msgs/Message`) so any
GNSS driver can consume them.

## Status — implemented

Three executables share a common RTCM3 framing library
([`rtcm_framer`](src/rtcm_framer.cpp), unit-tested) and publish validated
frames on `rtcm/out`. Each runs its own session thread with bounded
exponential reconnect/backoff.

| Executable                | Purpose                                            |
|---------------------------|----------------------------------------------------|
| `ntrip_client_node`       | NTRIP v1/v2 caster client (GGA upload, reconnect). |
| `tcp_rtcm_relay_node`     | Raw RTCM3 over TCP (client **and** server roles).  |
| `serial_rtcm_relay_node`  | UART → RTCM (for radio modems on the rover).       |

## Topics

| Topic                | Type                | Direction | Notes |
|----------------------|---------------------|-----------|-------|
| `rtcm/out`           | `rtcm_msgs/Message` | publish   | Validated RTCM3 frames for any GNSS driver. |
| `nmea_sentence`      | `nmea_msgs/Sentence`| subscribe | `ntrip_client_node` only, when `send_gga: true` — latest GGA is uploaded to the caster for VRS / nearest-base mountpoints. |

Wire `rtcm/out` to a driver's `rtcm/in` (see `um982_driver`).

## Parameters

`ntrip_client_node` (see [config/ntrip.yaml.example](config/ntrip.yaml.example)):
`host`, `port`, `mountpoint`, `username`, `password`, `ntrip_version`
(`1`|`2`|`auto`), `user_agent`, `send_gga`, `gga_period_s`,
`reconnect_backoff_s_min/max`, `frame_id`.

`tcp_rtcm_relay_node` ([local_base_tcp.yaml](config/local_base_tcp.yaml)):
`role` (`client`|`server`), `host`, `bind_address`, `port`,
`reconnect_backoff_s_min/max`, `frame_id`.

`serial_rtcm_relay_node` ([local_base_serial.yaml](config/local_base_serial.yaml)):
`port`, `baudrate`, `frame_id`.

## Configuration

Copy `ntrip.yaml.example` to `ntrip.yaml` and fill in credentials.
`**/ntrip.yaml` is gitignored — keep real credentials out of version
control.

## Run

```bash
ros2 launch ntrip_client ntrip_client.launch.py params_file:=/path/to/ntrip.yaml
ros2 launch ntrip_client tcp_rtcm_relay.launch.py
ros2 launch ntrip_client serial_rtcm_relay.launch.py
```

For the rover + RTK + VRS-GGA loop pre-wired against `um982_driver`, use
`ros2 launch um982_driver gnss_rtk.launch.py`.
