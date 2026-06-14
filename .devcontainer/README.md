# Dev Containers

This folder holds the VS Code Dev Container definitions for working on the
`outdoor-patrol` workspace. There are two variants:

| Folder | Target host | GUI | Hardware passthrough | When to use |
|---|---|---|---|---|
| `./` (this folder) | Workstation (x86_64 / WSL2 / arm64 Mac) | X11 / Wayland / WSLg | `/dev` bind-mount (privileged) | Day-to-day development, simulation, RViz |
| [`orangepi/`](orangepi/devcontainer.json) | Orange Pi 5 (RK3588, arm64) | headless | `/dev` bind-mount (privileged) | On-device development & debugging on the robot |

The workstation variant builds from [`Dockerfile`](Dockerfile) (based on
`althack/ros2:${ROS_DISTRO}-${ROS_VARIANT}`, which is published **amd64-only**).
The Orange Pi variant builds from its own [`orangepi/Dockerfile`](orangepi/Dockerfile),
based on the official, multi-arch `ros:${ROS_DISTRO}-ros-base` image — `althack/ros2`
has no arm64 manifest, so it cannot be pulled on the Pi.

---

## Prerequisites

- [Docker](https://docs.docker.com/engine/install/)
- [VS Code](https://code.visualstudio.com/) + the **Dev Containers** extension
  (`ms-vscode-remote.remote-containers`)
- For the Pi variant: VS Code's **Remote – SSH** extension and SSH access to
  the Pi with Docker installed there.

---

## Workstation variant ([`devcontainer.json`](devcontainer.json))

The default container. Wired for desktop development:

- X11 + Wayland (WSLg) volume mounts and `DISPLAY` / `WAYLAND_DISPLAY` env vars
- `--network=host`, `--ipc=host` for ROS 2 / DDS
- Cyclone DDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) to match the robot /
  deploy container
- Serial / device passthrough for bring-up — see
  [Hardware passthrough](#hardware-passthrough-both-variants)
- Optional Intel iGPU and NVIDIA GPU passthrough (commented)
- Software OpenGL fallback (`LIBGL_ALWAYS_SOFTWARE=1`)

### Open it

1. Open the workspace folder in VS Code.
2. Command palette → **Dev Containers: Reopen in Container** → pick the
   default entry.
3. First build takes a few minutes. Subsequent opens are instant.

### Common per-machine tweaks

Edit [`devcontainer.json`](devcontainer.json) and adjust:

- `USER_UID` / `USER_GID` if you hit XAuthority errors (see top-level
  [README.md](../README.md#xauthority)).
- WSL2 users: comment out the Wayland volume and add the `/usr/lib/wsl` /
  `/dev/dxg` lines from the FAQ.
- NVIDIA users: uncomment `--runtime=nvidia` (Linux) or the WSL2 GPU block.

### What you get inside

- ROS 2 (default: **Jazzy**, full desktop) on Ubuntu 24.04
- `colcon`, `rosdep`, `vcs`, `ament_*` linters, formatters
- VS Code extensions for C++, Python, CMake, XML, YAML, ROS

---

## Orange Pi 5 variant ([`orangepi/devcontainer.json`](orangepi/devcontainer.json))

Headless variant for the robot itself. Differences from the workstation:

- Dedicated arm64 [`orangepi/Dockerfile`](orangepi/Dockerfile) on the official
  multi-arch `ros:jazzy-ros-base` image (the workstation's `althack/ros2` base
  is amd64-only)
- Forced `--platform=linux/arm64`
- No X11 / Wayland / WSLg mounts, no `DISPLAY`
- No NVIDIA bits
- Adds `dialout`, `plugdev`, `video` to the container user's groups (plus
  commented `i2c` / `gpio` slots)

It shares the workstation's serial / device passthrough and Cyclone DDS
settings — see [Hardware passthrough](#hardware-passthrough-both-variants). The
commented reference `--device=` lines add the RK3588-specific `/dev/i2c-*`,
`/dev/spidev*`, `/dev/gpiochip*`, Mali (`/dev/dri`) and RKNN NPU (`/dev/rknpu`)
nodes.

### Open it

1. From your workstation, **Remote – SSH** into the Pi (`Remote-SSH: Connect to Host…`).
2. Open the workspace folder on the Pi.
3. Command palette → **Dev Containers: Reopen in Container** → pick the
   **orangepi** entry from the picker.
4. First build pulls the official arm64 `ros:jazzy-ros-base` image and compiles
   inside the Pi. Plan for ~10–20 minutes the first time.

### Enable your sensors

During bring-up there's nothing to do: `--privileged` + `--volume=/dev:/dev`
expose every host device node live, and `initializeCommand` installs the
`/dev/esp32-chassis` udev symlink on the Pi. To lock this down later, comment
out the privileged block and uncomment the matching `--device=` lines in
[`orangepi/devcontainer.json`](orangepi/devcontainer.json). See
[Hardware passthrough](#hardware-passthrough-both-variants) for details,
including the CAN setup note.

### Adding extra ROS packages

The Pi image is already the lean `ros-base` (no RViz / Gazebo — they aren't
useful on a headless robot anyway). If you need more ROS packages inside the
container, add an `apt-get install ros-${ROS_DISTRO}-<pkg>` line to
[`orangepi/Dockerfile`](orangepi/Dockerfile) and rebuild, or pull them in via
your packages' `package.xml` + `rosdep install`.

---

## Hardware passthrough (both variants)

Both variants currently expose hardware the **permissive** way, for bring-up:

- `--privileged` + `--volume=/dev:/dev` map **all** host device nodes into the
  container, live. This is deliberate: Docker's `--device` binds a node at
  container-create time and never tracks USB re-enumeration, so a device reset
  (e.g. the ESP32-S3's native USB-Serial/JTAG, which drops off the bus on every
  chip reset) would otherwise leave a stale node inside the container.
  Bind-mounting `/dev` makes nodes appear and disappear just as they do on the
  host.
- `initializeCommand` runs [`install-udev-rules.sh`](install-udev-rules.sh) on
  the **host** before the container is created. It installs
  [`99-esp32-chassis.rules`](99-esp32-chassis.rules), creating a stable
  `/dev/esp32-chassis` symlink for the ESP32-S3 chassis controller (its
  `by-id` path embeds a colon-laden MAC that `--device` can't parse). The
  script is idempotent and may prompt for `sudo`.
- The per-device `--device=` lines stay **commented** in both
  `devcontainer.json` files as a reference for tightening this back down once
  the stack is stable.

> ⚠️ `--privileged` is broad. It's fine for bring-up on a trusted machine, but
> switch back to explicit `--device=` entries (or a device-cgroup rule plus the
> `/dev` mount) before relying on this beyond development.

For CAN on the Pi, bring the interface up on the host first:

```bash
sudo ip link set can0 up type can bitrate 500000
```

---

## Useful commands inside any dev container

```bash
# Pull repos listed in src/ros2.repos
vcs import src < src/ros2.repos
# Or as git submodules:
python3 .devcontainer/repos_to_submodules.py

# Resolve dependencies
sudo apt-get update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build / test / lint
./build.sh
./test.sh
ament_uncrustify --reformat src/
```

VS Code's task picker (`Terminal → Run Task…`) exposes the same commands as
well as `new ament_cmake package`, `new ament_python package`, and
`add submodules from .repos`.

---

## Going from dev container to deployment

The dev containers are for **development**. For the image that ships to the
robot, see [`../deploy/README.md`](../deploy/README.md).
