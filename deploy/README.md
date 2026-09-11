# Deployment Image

Production container image for the `outdoor-patrol` robot. Unlike the
dev containers in [`../.devcontainer/`](../.devcontainer/), this image:

- targets `linux/arm64` (e.g. Orange Pi 5 / RK3588) by default,
- is built from `ros:${ROS_DISTRO}-ros-base` — no RViz, no Gazebo, no GUI,
- uses a multi-stage build so the runtime image carries only the compiled
  `install/` tree and its runtime dependencies,
- is intended to be started by `systemd` or `docker compose up -d` on the
  robot and left running.

## Files

| File | Purpose |
|---|---|
| [`Dockerfile`](Dockerfile) | Multi-stage build: `builder` (colcon) → `runtime` (slim). |
| [`entrypoint.sh`](entrypoint.sh) | Sources `/opt/ros/$ROS_DISTRO/setup.bash` and the workspace overlay, then `exec "$@"`. |
| [`docker-compose.yaml`](docker-compose.yaml) | Sensor/localization stack, fixed container device roles, host networking/IPC, and persistent `./data`. |
| [`docker-compose.nav2.yaml`](docker-compose.nav2.yaml) | Alternate field stack with the same device contract plus recorder/Nav2/mission services. |
| [`.env.example`](.env.example) | Opt-in host-role paths; merge into local, ignored `.env` after host setup. |
| [`udev/99-outdoor-patrol.rules`](udev/99-outdoor-patrol.rules) | Robot-specific identity template; GNSS needs an explicit USB topology pin. |

The build context is the repo root; [`../.dockerignore`](../.dockerignore)
excludes `build/`, `install/`, `log/`, `.git/`, `.devcontainer/`, and the
`src/robot-research` docs submodule.

---

## Build

### On the Pi (simplest)

```bash
# from the repo root on the Pi
docker compose -f deploy/docker-compose.yaml build
```

Or without compose:

```bash
docker build -f deploy/Dockerfile -t outdoor-patrol:arm64 .
```

### Cross-build on a workstation and push to a registry

Requires `docker buildx` with a `linux/arm64` builder configured (QEMU
emulation is fine for occasional builds; a native arm64 runner is faster).

```bash
docker buildx create --use --name multiarch          # one-time
docker buildx build \
  --platform linux/arm64 \
  -f deploy/Dockerfile \
  -t ghcr.io/<you>/outdoor-patrol:arm64 \
  --push .
```

Then on the Pi:

```bash
docker pull ghcr.io/<you>/outdoor-patrol:arm64
```

### Build for a different ROS distro

```bash
docker build \
  --build-arg ROS_DISTRO=humble \
  -f deploy/Dockerfile -t outdoor-patrol:humble-arm64 .
```

---

## Run

### With docker compose (recommended)

The stock configuration starts sensor drivers, localization and the scan brake
with RViz disabled. Its four device sources default to the previously used
host paths, so **existing hosts do not need new udev rules to keep working**.
The container paths are fixed roles regardless of which host paths are used.
The Nav2 alternative has the same device contract; do not run both robot
services at once because they would compete for the devices and ROS graph.

```bash
docker compose -f deploy/docker-compose.yaml up -d
docker compose -f deploy/docker-compose.yaml logs -f
docker compose -f deploy/docker-compose.yaml down
```

The compose file uses `restart: unless-stopped`, so the container comes back
up after reboots as long as the Docker daemon does.

### Device overrides

Both Compose entry points accept `SERIAL_DEV`, `GNSS_DEV`, `IMU_DEV` and
`LIDAR_DEV` as **host-side** paths. They are mapped to fixed container paths;
the launch command always passes those container paths to the drivers:

| Host variable | Container path / launch port |
|---|---|
| `SERIAL_DEV` | `/dev/op-chassis` (`serial_dev`) |
| `GNSS_DEV` | `/dev/op-gnss` (`gnss_dev`) |
| `IMU_DEV` | `/dev/op-imu` (`imu_dev`) |
| `LIDAR_DEV` | `/dev/op-lidar` (`lidar_dev`) |

No driver-YAML edit or image rebuild is needed for a port-only change:

```bash
GNSS_DEV=/dev/gnss-rover IMU_DEV=/dev/imu-primary \
  docker compose -f deploy/docker-compose.yaml up -d --force-recreate robot
```

For a launch outside Compose, use `serial_dev`, `gnss_dev`, `imu_dev` and
`lidar_dev` on
[`gnss_localization.launch.py`](../src/outdoor_patrol_bringup/launch/gnss_localization.launch.py).
Empty `gnss_dev`/`imu_dev` retain the driver YAML values. Use
`gnss_params_file` and `imu_params_file` for baud/protocol parameters;
the legacy `um982_params_file` alias is still accepted. Port overrides
take precedence over those files.

Lifecycle controls are now independent: `gnss_auto_activate` and
`imu_auto_activate` both default to `true`. The legacy `auto_activate` argument
controls **GNSS only**; it no longer unintentionally disables the IMU too.
To disable both, set both role-specific controls to `false`.

### Opt in to stable host roles

This is an explicit host setup step, not something the image build performs.
The checked-in serial numbers record the old USB adapters, **not a live
inventory**. Confirm them on the actual robot before installing.

1. Copy [`udev/99-outdoor-patrol.rules`](udev/99-outdoor-patrol.rules) to a local
   file, for example `~/.config/outdoor-patrol/robot.rules`.
2. Identify the actual GNSS tty and inspect it with
   `udevadm info --query=property --name=/dev/ttyUSB0` (replace the example tty).
   Replace the GNSS path placeholder in your copy with its exact `ID_PATH`.
   The current CH340 adapter has no unique unit serial: matching all CH340s
   or choosing the first tty is not safe. A uniquely serialized replacement
   can instead have a unit-specific rule. Recheck topology pins after recabling.
3. Verify/correct the chassis, IMU and lidar serial matches in that same file.
   Keep the role symlinks and `GROUP="dialout", MODE="0660"`.
4. With all four stock devices connected, install and verify:

   ```bash
   ./scripts/install-device-rules.sh \
     --rules-file "$HOME/.config/outdoor-patrol/robot.rules" --verify
   ```

   The installer rejects an unconfigured GNSS template, reports missing or
   shared device targets, and checks the group. It migrates the original
   chassis-only rule; customized legacy rules require explicit reconciliation.
   It requests sudo only for host installation/reload. A failed `--verify`
   means the rules may be installed but the hardware is **not ready**.
5. Merge the four assignments from [`.env.example`](.env.example) into
   `deploy/.env`. Preserve existing NTRIP/data settings; do not overwrite them.
   Then validate and recreate the selected service:

   ```bash
   docker compose --env-file deploy/.env -f deploy/docker-compose.yaml config --quiet
   docker compose --env-file deploy/.env -f deploy/docker-compose.yaml up -d --force-recreate robot
   ```

`config` validates Compose syntax only, not hardware. `--verify` above is for
the stock four-USB-device setup. A role symlink prevents identity strings from
spreading downstream; it **does not make a Docker device binding hotpluggable**.
After a replacement/unplug/ESP32 reset, recreate the robot container.
`docker restart` alone does not resolve the new device node.

Both devcontainers retain their existing live `/dev` mount, so installed host
roles appear automatically. They do **not** read Compose's `.env`. Their
automatic setup remains chassis-only and leaves a full host-role installation
untouched; standalone launches retain legacy ports unless explicitly overridden.

### Ad-hoc `docker run`

```bash
docker run --rm -it \
  --network=host --ipc=host \
  --device=/dev/op-chassis:/dev/op-chassis \
  --group-add dialout \
  -v "$PWD/data:/data" \
  outdoor-patrol:arm64 \
  ros2 launch outdoor_patrol_bringup teleop.launch.py serial_dev:=/dev/op-chassis
```

---

## Talking to the robot from your workstation

Because the container uses `network_mode: host`, ROS 2 nodes inside the
container are reachable from any host on the same network that has the same
`ROS_DOMAIN_ID`. On your workstation:

```bash
export ROS_DOMAIN_ID=42        # match what's set in docker-compose.yaml
ros2 topic list
```

If you switch to Cyclone DDS, set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
**on both ends**.

---

## Updating the image on the robot

```bash
# pulled-from-registry workflow
docker compose -f deploy/docker-compose.yaml pull
docker compose -f deploy/docker-compose.yaml up -d

# built-on-device workflow
git pull
git submodule update --init --recursive
docker compose -f deploy/docker-compose.yaml build
docker compose -f deploy/docker-compose.yaml up -d
```

---

## Troubleshooting

- **`rosdep` fails during build** — usually a network blip or a missing
  `rosdep` key for a third-party package. Run the same command locally with
  `rosdep install --from-paths src --ignore-src -y` to reproduce.
- **Container starts but no topics** — confirm both ends share
  `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`. With `network_mode: host`,
  multicast must work on your LAN; some Wi-Fi APs block it.
- **`Permission denied` on `/dev/tty…`** — add the right `group_add:` entry
  (typically `dialout`), or run with `--privileged` to confirm it's a
  permissions issue.
- **Image is huge** — make sure `.dockerignore` is in effect (the runtime
  stage should not see `build/`, `install/`, or `src/robot-research`).
- **Build OOMs on the Pi** — colcon parallelism: add
  `--build-arg MAKEFLAGS="-j2"` or temporarily add swap.

---

## Going from deployment back to development

The deploy image is intentionally minimal. To edit code on the robot itself
(launch files, params, quick fixes), use the **Orange Pi dev container**
described in [`../.devcontainer/README.md`](../.devcontainer/README.md).

## Device configuration checks (no robot required)

```bash
pytest -q deploy/test
```

These tests render both Compose configurations and run the installer with
fake udev/sudo and temporary filesystem roots. They never install host rules
or start a robot container. Run them explicitly: `colcon test` only discovers
the ROS package tests, not this directory.
