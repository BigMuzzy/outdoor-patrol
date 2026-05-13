#!/usr/bin/env bash
# Entrypoint for the deployment image: source ROS and the workspace overlay,
# then exec whatever command was passed.
set -euo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /opt/outdoor-patrol/install/setup.bash ]]; then
    source /opt/outdoor-patrol/install/setup.bash
fi

exec "$@"
