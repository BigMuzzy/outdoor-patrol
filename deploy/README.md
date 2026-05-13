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
| [`docker-compose.yaml`](docker-compose.yaml) | One-service compose file with host networking, IPC, device / group placeholders, and a persistent `./data` volume. |

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

Edit [`docker-compose.yaml`](docker-compose.yaml) and:

1. Uncomment the `devices:` entries for the sensors actually wired to the Pi
   (GPS on `/dev/ttyUSB0`, IMU on `/dev/ttyACM0`, camera on `/dev/video0`,
   etc.).
2. Uncomment the matching `group_add:` entries (`dialout`, `video`,
   `plugdev`, …).
3. Replace `command: ["bash"]` with your actual bringup launch, e.g.:

   ```yaml
   command: ["ros2", "launch", "outdoor_patrol_bringup", "robot.launch.py"]
   ```

Then:

```bash
docker compose -f deploy/docker-compose.yaml up -d
docker compose -f deploy/docker-compose.yaml logs -f
docker compose -f deploy/docker-compose.yaml down
```

The compose file uses `restart: unless-stopped`, so the container comes back
up after reboots as long as the Docker daemon does.

### Ad-hoc `docker run`

```bash
docker run --rm -it \
  --network=host --ipc=host \
  --device=/dev/ttyUSB0 \
  --group-add dialout \
  -v "$PWD/data:/data" \
  outdoor-patrol:arm64 \
  ros2 launch outdoor_patrol_bringup robot.launch.py
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
