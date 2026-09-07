#!/bin/bash
# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
#
# Resolve and install the system dependencies declared in every package.xml
# under src/. Used by setup.sh, the devcontainer postCreateCommand and
# deploy/Dockerfile so all three agree on what "dependencies installed" means.
#
# Why this exists rather than a bare `rosdep install`:
#
# Three of the entries in src/ are submodules / vcs imports
# (sllidar_ros2, robot-research, esp32-s3-uros-controller). They are pulled by
# `./setup.sh` or `git submodule update`, both of which need git SSH
# credentials, so a freshly rebuilt devcontainer routinely has them missing or
# empty. Our own package.xml files still name them -- outdoor_patrol_bringup
# exec_depends on sllidar_ros2 -- and rosdep treats an unresolvable key as a
# fatal error for the WHOLE invocation: it exits non-zero having installed
# nothing at all. The visible symptom is not "sllidar is missing", it is
# "nav2 is missing", because every other key was dropped on the floor with it.
#
# Skipping the keys that live in src/ is safe in both directions. When the
# submodule IS checked out, --ignore-src already excludes it, so the skip is a
# no-op. When it is NOT, the skip costs a package we could not have installed
# anyway and keeps the other ~20 resolvable keys.
#
# Usage:
#   ./scripts/rosdep-install.sh                     # all dependency types
#   ./scripts/rosdep-install.sh --dependency-types=exec
#
# Any extra arguments are forwarded to `rosdep install`.

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

: "${ROS_DISTRO:?ROS_DISTRO is not set; source /opt/ros/<distro>/setup.bash}"

# Packages provided by src/ submodules and vcs imports. Keep in sync with
# .gitmodules and src/ros2.repos.
SKIP_KEYS="sllidar_ros2"

SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"

$SUDO rosdep init > /dev/null 2>&1 || true
$SUDO apt-get update
rosdep update --rosdistro="$ROS_DISTRO"
rosdep install --from-paths src --ignore-src -y \
    --rosdistro="$ROS_DISTRO" \
    --skip-keys "$SKIP_KEYS" \
    "$@"
