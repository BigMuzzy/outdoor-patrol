#!/usr/bin/env bash
# Entrypoint for the deployment image: source ROS, the micro-ROS agent
# sibling workspace, and the main workspace overlay, then exec whatever
# command was passed.
set -euo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /opt/outdoor-patrol/uros_agent_ws/install/local_setup.bash ]]; then
    source /opt/outdoor-patrol/uros_agent_ws/install/local_setup.bash
fi
if [[ -f /opt/outdoor-patrol/install/setup.bash ]]; then
    source /opt/outdoor-patrol/install/setup.bash
fi

exec "$@"
