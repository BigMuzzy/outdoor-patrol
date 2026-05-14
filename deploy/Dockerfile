# syntax=docker/dockerfile:1.7
#
# Deployment image for the outdoor-patrol robot.
# Multi-stage: build with the full ROS 2 dev tooling, then copy the
# resulting `install/` tree into a slim ros-base runtime image.
#
# Build (on the Pi):
#   docker build -f deploy/Dockerfile -t outdoor-patrol:arm64 .
#
# Cross-build from a workstation and push:
#   docker buildx build --platform linux/arm64 \
#     -f deploy/Dockerfile \
#     -t ghcr.io/<you>/outdoor-patrol:arm64 --push .

ARG ROS_DISTRO=jazzy

# ---------- Stage 1: build ----------
FROM ros:${ROS_DISTRO}-ros-base AS builder
ARG ROS_DISTRO

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/ ./src/
COPY scripts/ ./scripts/

# Skip the ESP-IDF firmware subtree — its CMakeLists.txt is not a colcon
# package and would fail the workspace build.
RUN touch src/esp32-s3-uros-controller/firmware/COLCON_IGNORE || true

# Resolve dependencies declared in package.xml files (covers both our
# packages and the vendored src/micro_ros_setup submodule).
RUN apt-get update \
    && rosdep update --rosdistro=${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y --rosdistro=${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

# Build the main workspace (includes micro_ros_setup meta-tooling).
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build the micro-ROS agent in its sibling workspace, mirroring what
# scripts/setup-uros-agent.sh does on a developer machine. Path lands at
# /ws/uros_agent_ws to match the convention used outside the container.
RUN apt-get update \
    && ./scripts/setup-uros-agent.sh \
    && rm -rf /var/lib/apt/lists/* \
               /ws/uros_agent_ws/build \
               /ws/uros_agent_ws/log \
               /ws/uros_agent_ws/src

# ---------- Stage 2: runtime ----------
FROM ros:${ROS_DISTRO}-ros-base AS runtime
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# Runtime-only deps. Add anything your nodes dlopen at runtime here.
# teleop_twist_keyboard is pulled in so `docker exec` can run it
# interactively inside the same container.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-rosdep \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Re-resolve runtime deps based on the built install/ metadata.
WORKDIR /opt/outdoor-patrol
COPY --from=builder /ws/install ./install
COPY --from=builder /ws/uros_agent_ws/install ./uros_agent_ws/install
COPY --from=builder /ws/src ./src
RUN apt-get update \
    && rosdep update --rosdistro=${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y --rosdistro=${ROS_DISTRO} \
        --dependency-types=exec \
    && rm -rf /var/lib/apt/lists/* /opt/outdoor-patrol/src

COPY deploy/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# Override at `docker run` / compose time, e.g.:
#   command: ["ros2", "launch", "outdoor_patrol_bringup", "teleop.launch.py"]
CMD ["bash"]
