# Cyclone DDS: "Failed to parse type hash" warnings from micro-ROS topics

- **Date:** 2026-05-16
- **Affects:** ROS 2 Jazzy, `rmw_cyclonedds_cpp`, micro-ROS (XRCE-DDS)
  clients via `micro_ros_agent`
- **Severity:** gotcha (benign — topics work)

## Symptom

After switching the RMW to Cyclone DDS (see
[dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md](../networking/dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md)),
every node that discovers a micro-ROS-side topic logs one warning per
topic, on both the robot and the dev box:

```text
[robot_state_publisher-1] [WARN] [rmw_cyclonedds_cpp]:
  Failed to parse type hash for topic 'rt/odom'
  with type 'nav_msgs::msg::dds_::Odometry_' from USER_DATA '(null)'.
[teleop_twist_keyboard]   [WARN] [rmw_cyclonedds_cpp]:
  Failed to parse type hash for topic 'rt/cmd_vel'
  with type 'geometry_msgs::msg::dds_::Twist_' from USER_DATA '(null)'.
...
```

Topics still publish and subscribe normally; teleop works end-to-end.

## Root cause

ROS 2 Jazzy added a **type hash** field that participants advertise via
DDS `USER_DATA` so peers can verify type compatibility without
matching only on type name. Cyclone DDS surfaces a warning when a
discovered remote endpoint has no parseable type hash, then falls back
to name-based matching.

The micro-ROS XRCE-DDS client (the firmware side bridged by
`micro_ros_agent`) does not yet populate this field. So every topic
created by the agent on behalf of the firmware shows up with empty
USER_DATA and triggers the warning exactly once per discovering node.

Fast DDS does not emit this warning today — that's why we only started
seeing it after the Jazzy + Cyclone switch.

## Fix

None on our side — this is an upstream gap in micro-ROS. Leave the
warnings in place; suppressing them risks hiding a real type mismatch
later (e.g. if we ever change a message definition out from under the
firmware without rebuilding it).

If a launch becomes too noisy to read, the cheap escape hatch is
per-node:

```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    arguments=['--ros-args', '--log-level', 'rmw_cyclonedds_cpp:=ERROR'],
    ...
)
```

or at the CLI:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --log-level rmw_cyclonedds_cpp:=ERROR
```

Do **not** apply this globally via env — it'd silence legitimate RMW
errors too.

## How to verify (that it's benign)

With the warnings present:

```bash
ros2 topic echo /odom --once          # data flows
ros2 topic echo /failsafe/active --once
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'   # firmware acts on it
```

Warning count should equal (number of micro-ROS-side topics) ×
(number of discovering nodes), and only on first discovery — not
continuously.

## Related

- [networking/dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md](../networking/dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md)
- Upstream: <https://github.com/ros2/rmw_cyclonedds>
- micro-ROS: <https://github.com/micro-ROS/micro_ros_setup>
- ROS 2 Jazzy type hash design: <https://design.ros2.org/articles/typesupport.html>
