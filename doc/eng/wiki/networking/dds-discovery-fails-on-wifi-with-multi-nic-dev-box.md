# DDS discovery fails on WiFi with a multi-NIC dev box

- **Date:** 2026-05-16
- **Affects:** ROS 2 Jazzy, Fast DDS (default RMW), Orange Pi 5 on WiFi,
  x86_64 dev container with both wired and WiFi on the same `/24`
- **Severity:** blocker (no topics visible cross-host)

## Symptom

After switching the Pi from wired Ethernet to WiFi (same `/24` subnet,
DHCP from the same router), `ros2 topic list` on the dev box dropped
back to defaults:

```text
ros@max-lnx-dev:/workspaces/outdoor-patrol$ ros2 topic list
/parameter_events
/rosout
```

Inside the Pi container the full topic list (including `/cmd_vel`,
`/odom`, `/failsafe/active`, …) was visible, so the robot side was
healthy. `ping 192.168.55.249` worked from the dev box. Small multicast
test packets also got through:

```text
ros2 multicast receive
Received from 192.168.55.249:50755: 'Hello World!'
```

…yet DDS discovery still found nothing.

## Root cause

Two things combined:

1. **WiFi client isolation on the AP.** Pinning the source NIC proved
   it:
   ```text
   ping -I enx806d971f1582 192.168.55.249    → 0% loss   (wired → AP → WiFi: OK)
   ping -I wlp2s0          192.168.55.249    → 100% loss (WiFi → WiFi: blocked)
   ```
   The AP bridges wired↔WiFi but isolates WiFi↔WiFi clients.

2. **Fast DDS multi-interface bind behavior.** The dev box has two NICs
   on the same subnet (wired `enx…/.124` metric 100, WiFi
   `wlp2s0/.113` metric 600). Fast DDS handled this combo poorly —
   small ICMP/UDP multicast worked, but DDS participant discovery did
   not converge. `ROS_STATIC_PEERS=<pi-ip>` +
   `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` did not help.

## Fix

Switch the RMW to **Cyclone DDS** on both ends. Cyclone copes with
multi-NIC + mixed wired/WiFi setups much better and "just worked" with
default (multicast) discovery — no hardcoded peers, no per-host IPs.

Changes committed:

- `deploy/Dockerfile` (runtime stage) — install
  `ros-${ROS_DISTRO}-rmw-cyclonedds-cpp`.
- `deploy/docker-compose.yaml` — set
  `RMW_IMPLEMENTATION: rmw_cyclonedds_cpp`.
- `.devcontainer/Dockerfile` — install
  `ros-${ROS_DISTRO}-rmw-cyclonedds-cpp` (the `althack/ros2:jazzy-full`
  base does not ship it).
- `.devcontainer/devcontainer.json` — set `RMW_IMPLEMENTATION` in
  `containerEnv`.

## How to verify

From the dev container, with the robot container running on the Pi:

```bash
echo "$RMW_IMPLEMENTATION"        # rmw_cyclonedds_cpp
ros2 daemon stop && ros2 daemon start
ros2 topic list                   # full list, including /cmd_vel /odom
ros2 topic echo /odom --once
```

And on the Pi:

```bash
docker exec outdoor-patrol bash -lc 'env | grep RMW_IMPLEMENTATION'
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Notes / future work

- If a future AP/network forbids multicast entirely, fall back to
  Cyclone DDS with a `<Peers>` block. Prefer mDNS hostnames
  (`*.local`) over hardcoded IPs.
- The dual-NIC dev box stays workable as long as the AP keeps bridging
  wired↔WiFi. If that changes, take the wired NIC down or pin DDS to
  one interface via `CYCLONEDDS_URI` (`<NetworkInterfaceAddress>`).
- A small reproducer (`ros2 multicast send` / `receive`) is the fastest
  way to separate "AP drops all multicast" from "DDS-specific bind
  weirdness." Always run that first next time topics go missing.

## Related

- Commit introducing the Cyclone switch: see git log around
  `deploy/Dockerfile`, `deploy/docker-compose.yaml`,
  `.devcontainer/`.
- Upstream: <https://docs.ros.org/en/jazzy/Installation/DDS-Implementations.html>
- Cyclone DDS config reference:
  <https://cyclonedds.io/docs/cyclonedds/latest/config/>
