# Dev container image for on-device development on the Orange Pi 5 (RK3588, arm64).
#
# The workstation image (../Dockerfile) is based on althack/ros2, which is only
# published for amd64 — pulling it on the Pi fails with
#   "no matching manifest for linux/arm64/v8 in the manifest list entries".
# This image instead builds on the official, multi-arch `ros` image (which ships
# an arm64/v8 manifest, like the deploy image) and re-creates the parts of the
# althack image this workspace relies on: a non-root `ros` user with passwordless
# sudo, the colcon/rosdep/vcs dev tooling (already in ros-base), `envsubst`
# (used by setup.sh) and Cyclone DDS.

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

# --- Non-root user -----------------------------------------------------------
# Recreate the althack/ros2 `ros` user (UID/GID 1000). devcontainer.json sets
# `remoteUser: ros` and adds it to the dialout/plugdev/video groups, so the user
# must exist. Ubuntu 24.04 (noble), which the ROS Jazzy image is based on, ships
# a default `ubuntu` user at UID/GID 1000 — remove whatever occupies 1000 first
# so we can claim it for `ros`.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN set -eux; \
    if getent passwd "${USER_UID}" >/dev/null; then \
        userdel -r "$(getent passwd "${USER_UID}" | cut -d: -f1)" 2>/dev/null || true; \
    fi; \
    if getent group "${USER_GID}" >/dev/null; then \
        groupdel "$(getent group "${USER_GID}" | cut -d: -f1)" 2>/dev/null || true; \
    fi; \
    apt-get update; \
    apt-get install -y --no-install-recommends sudo; \
    groupadd --gid "${USER_GID}" "${USERNAME}"; \
    useradd -s /bin/bash --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}"; \
    echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"; \
    rm -rf /var/lib/apt/lists/*

# --- Extra tooling beyond the official ros-base image ------------------------
#   - gettext-base: provides `envsubst`, used by setup.sh to expand src/ros2.repos
#   - rmw-cyclonedds-cpp: Cyclone DDS RMW, to match the robot/deploy container
#     (see deploy/docker-compose.yaml); not shipped by the base image
# colcon, rosdep (already initialized) and vcstool come with ros-base.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        gettext-base \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Auto-source ROS for the user's interactive shells, like the althack image.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "/home/${USERNAME}/.bashrc"
