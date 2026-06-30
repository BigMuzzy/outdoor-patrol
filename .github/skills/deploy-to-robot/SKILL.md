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
- Image/tag: `outdoor-patrol:arm64`; service/container name: `outdoor-patrol`.
- A full build takes **~9 min** on the Pi (longer if base layers are cold).

## ⚠️ Read this first (lessons learned)

1. **The Pi builds from `origin/main`, not your working tree.** Any unpushed
   commits or uncommitted edits will **not** deploy. Push first (see Step 1).
   The dev-only files (`.devcontainer/`, `.github/`, `.vscode/`, `build/`,
   `install/`, `log/`, `src/robot-research`) are excluded by
   [.dockerignore](../../../.dockerignore) and don't affect the image anyway.
2. **The image ≠ what's launched.** The compose `command:` currently runs
   `outdoor_patrol_bringup teleop.launch.py`. New launches/configs (e.g.
   `global_localization.launch.py`, `heading_to_imu.yaml`) get baked into the
   image but are **not started** unless the compose `command:` (or an
   `IncludeLaunchDescription`) actually invokes them. After deploying, confirm
   the running command launches what the user expects.
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
  docker compose -f deploy/docker-compose.yaml up -d && docker ps'
```

`restart: unless-stopped` means it also survives Pi reboots.

### Step 6 — Verify

```bash
ssh robot 'docker inspect -f "Status={{.State.Status}} Restarts={{.RestartCount}}" outdoor-patrol; \
  sed -n "57,70p" ~/code/outdoor-patrol/deploy/docker-compose.yaml; \
  docker compose -f ~/code/outdoor-patrol/deploy/docker-compose.yaml logs --tail=40'
```

Healthy = `Status=running`, `Restarts=0`, and the launch reaches its nodes
(e.g. `robot_state_publisher ... Robot initialized`, `micro_ros_agent ... running`).
Re-read the compose `command:` (the `sed` line) and confirm it launches what the
user actually wanted — flag it if not (Lesson #2).

## Troubleshooting

- **No topics on the dev box after deploy** — Pi races WiFi at boot; the
  container can bind a NIC with no IP. See
  [doc/eng/wiki/deployment/pi-container-races-wifi-at-boot.md](../../../doc/eng/wiki/deployment/pi-container-races-wifi-at-boot.md).
  A `docker restart outdoor-patrol` usually fixes a one-off.
- **Container restart-looping** — check `docker compose logs`; common causes are
  a missing serial device (`SERIAL_DEV`), a launch referencing an uninstalled
  config, or the entrypoint's network wait timing out.
- **`Permission denied` on `/dev/tty…`** — the device isn't mapped or the group
  is missing; the current compose uses permissive `/dev` passthrough for bring-up.
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
