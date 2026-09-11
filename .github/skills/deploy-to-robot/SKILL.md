---
name: deploy-to-robot
description: "Use when the user asks to deploy / ship / push the container (image) to the robot, rebuild and restart the robot's deploy stack, or roll out code changes to the Orange Pi 5. Build-on-device workflow: push to origin/main -> ssh robot -> git pull + submodules -> docker compose build -> up -d -> verify. DO NOT USE for plain SSH/one-off commands (use connect-to-robot), VESC tuning, or local dev-container builds (./build.sh / colcon)."
---

# Deploy the container to the robot

The robot (Orange Pi 5, aarch64) runs the compiled stack as a Docker container
defined in [deploy/docker-compose.yaml](../../../deploy/docker-compose.yaml).
The standard rollout is **build-on-device**: the Pi pulls `origin/main`, builds
the image locally, and recreates the container.

- Robot host: `ssh robot` (see the **connect-to-robot** skill).
- Repo on robot: `~/code/outdoor-patrol` (i.e. `/home/ubuntu/code/outdoor-patrol`).
- Image/tag: `outdoor-patrol:arm64`; Compose service: `robot`;
  container name: `outdoor-patrol`.
- A full build takes **~9 min** on the Pi (longer if base layers are cold).

## ⚠️ Read this first (lessons learned)

1. **The Pi builds from `origin/main`, not your working tree.** Any unpushed
   commits or uncommitted edits will **not** deploy. Push first (see Step 1).
   The dev-only files (`.devcontainer/`, `.github/`, `.vscode/`, `build/`,
   `install/`, `log/`, `src/robot-research`) are excluded by
   [.dockerignore](../../../.dockerignore) and don't affect the image anyway.
2. **The image ≠ what's launched.** The compose `command:` currently runs
   `outdoor_patrol_bringup gnss_localization.launch.py` with stock GNSS,
   IMU and lidar profiles plus shared localization/filter/brake. Additional
   drivers/configs may be baked into the image but do **not** run unless
   selected by a profile or explicitly launched/included. After deploying,
   confirm the effective command and selected profiles match the user's intent.
3. **Submodules matter.** `git pull` alone won't move them; always run
   `git submodule update --init --recursive` (esp. `esp32-s3-uros-controller`
   firmware bumps and `robot-research`).
4. **Don't `git push --force` or `reset --hard` on the robot.** The robot repo
   should stay a clean fast-forward of `origin/main`.

## Workflow

### Step 1 — Push the intended code (on the dev box)

```bash
cd /workspaces/outdoor-patrol
git status --short                              # any deploy-relevant WIP?
git rev-list --left-right --count HEAD...@{u}   # "<ahead> <behind>"
```

If there are unpushed commits or relevant uncommitted changes, **confirm scope
with the user**, commit (keep dev-only and source changes in separate commits;
leave throwaway files like `scratchpad.txt` untracked), then `git push origin main`.
Pushing is shared/irreversible — get explicit confirmation.

If a launch references a new config file, verify the package installs it
(`install(DIRECTORY launch config ...)` in the package `CMakeLists.txt`) or the
deployed launch will fail at runtime.

### Step 2 — Inspect the robot

```bash
ssh robot 'hostname; uname -m; docker --version; docker compose version | head -1; \
  docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"'
```

### Step 3 — Sync the repo on the robot (fast-forward only)

```bash
ssh robot 'cd ~/code/outdoor-patrol && git pull --ff-only && \
  git submodule update --init --recursive && git --no-pager log --oneline -1'
```

If `--ff-only` fails, the robot tree diverged or is dirty — stop and inspect
(`git status`), don't force it.

### Step 4 — Build the image (long-running)

```bash
ssh robot 'cd ~/code/outdoor-patrol && \
  time docker compose -f deploy/docker-compose.yaml build 2>&1 | tail -40'
```

Run it as a long/background command and wait for completion. Success ends with
`outdoor-patrol:arm64  Built`. If it OOMs, add `--build-arg MAKEFLAGS="-j2"` or
add swap on the Pi.

### Step 5 — Recreate the container

```bash
ssh robot 'cd ~/code/outdoor-patrol && \
  docker compose -f deploy/docker-compose.yaml up -d --force-recreate robot && docker ps'
```

`restart: unless-stopped` means it also survives Pi reboots.
Recreation, not just restart, is required after USB re-enumeration. Host `*_DEV`
values select sources; container targets stay `/dev/op-*`. Role-based host
aliases require the explicit [host setup](../../../deploy/README.md#opt-in-to-stable-host-roles);
the default host paths remain backward-compatible. Do not install generic
GNSS rules or guess a USB socket while deploying.

### Step 6 — Verify

```bash
ssh robot 'docker inspect -f "Status={{.State.Status}} Restarts={{.RestartCount}}" outdoor-patrol; \
  docker inspect -f "{{json .Config.Cmd}}" outdoor-patrol; \
  docker inspect -f "{{json .HostConfig.Devices}}" outdoor-patrol; \
  docker compose -f ~/code/outdoor-patrol/deploy/docker-compose.yaml logs --tail=40'
```

Healthy = `Status=running`, `Restarts=0`, and the launch reaches its nodes
(e.g. `robot_state_publisher ... Robot initialized`, `micro_ros_agent ... running`).
Check the effective container argv and device bindings above, not a fixed line
range in the source YAML, and confirm the selected launch profiles match what
the user wanted — flag a mismatch (Lesson #2). Sensor ports and optional YAML
paths use environment-backed launch defaults; see the
[profile contract](../../../src/outdoor_patrol_bringup/README.md).

## Troubleshooting

- **No topics on the dev box after deploy** — Pi races WiFi at boot; the
  container can bind a NIC with no IP. See
  [doc/eng/wiki/deployment/pi-container-races-wifi-at-boot.md](../../../doc/eng/wiki/deployment/pi-container-races-wifi-at-boot.md).
  A `docker restart outdoor-patrol` usually fixes a one-off.
- **Container will not start / restart-looping** — check all host device
  sources (`SERIAL_DEV`, `GNSS_DEV`, `IMU_DEV`, `LIDAR_DEV`). A missing source
  prevents container creation, including missing `/dev/op-*` aliases after
  opting in without installing rules. If the container exists, inspect
  `docker compose logs` for a missing profile/config or network-wait timeout.
- **`Permission denied` on `/dev/tty…`** — the device isn't mapped or the group
  is missing; production uses explicit source-to-role mappings and `dialout`.
  The privileged live `/dev` mount is a devcontainer setting, not production
  Compose behavior.
- **Build pulls the wrong ROS distro / arch** — image is `linux/arm64`,
  `ros:${ROS_DISTRO}-ros-base`. Override with `--build-arg ROS_DISTRO=...` only
  deliberately.

## Rollback

The previous image isn't tagged separately, so the cleanest rollback is to
checkout the prior commit and rebuild:

```bash
ssh robot 'cd ~/code/outdoor-patrol && git checkout <prev-sha> && \
  git submodule update --init --recursive && \
  docker compose -f deploy/docker-compose.yaml build && \
  docker compose -f deploy/docker-compose.yaml up -d'
```

Return to `main` with `git checkout main` once a fixed build is pushed.
