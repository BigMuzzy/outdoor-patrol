#!/bin/bash
# Bootstrap the micro-ROS agent sibling workspace.
#
# The `micro_ros_setup` package (vendored at `src/micro_ros_setup`) is a
# meta-build tool: it cannot produce `micro_ros_agent` from a single
# `colcon build` of this workspace. Instead it materialises a *second*
# workspace whose sources include the agent plus pinned XRCE-DDS / vendor
# packages, and builds them there.
#
# This script encapsulates that two-step flow and is safe to re-run; on
# subsequent invocations it only does an incremental rebuild.
#
# Prereqs:
#   * `./setup.sh && ./build.sh` already succeeded (main workspace built).
#   * ROS environment is sourced, or `install/setup.bash` exists.
#
# Usage:
#   ./scripts/setup-uros-agent.sh
#   source uros_agent_ws/install/local_setup.bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AGENT_WS="${REPO_ROOT}/uros_agent_ws"
MAIN_SETUP="${REPO_ROOT}/install/setup.bash"

if [[ ! -f "${MAIN_SETUP}" ]]; then
    echo "error: ${MAIN_SETUP} not found; run ./build.sh first." >&2
    exit 1
fi

# colcon's setup.bash references optional vars like COLCON_TRACE without
# guarding for `set -u`. Relax nounset across the source so the script
# still runs under `set -euo pipefail`.
set +u
# shellcheck disable=SC1090
source "${MAIN_SETUP}"
set -u

mkdir -p "${AGENT_WS}"
cd "${AGENT_WS}"

if [[ ! -d "${AGENT_WS}/src" ]]; then
    echo ">>> create_agent_ws.sh: fetching agent sources into ${AGENT_WS}"
    ros2 run micro_ros_setup create_agent_ws.sh
else
    echo ">>> agent sources already present in ${AGENT_WS}/src; skipping create_agent_ws.sh"
fi

echo ">>> build_agent.sh: building micro_ros_agent (incremental if rerun)"
ros2 run micro_ros_setup build_agent.sh

echo
echo "micro-ROS agent ready. To use it:"
echo "    source ${AGENT_WS}/install/local_setup.bash"
