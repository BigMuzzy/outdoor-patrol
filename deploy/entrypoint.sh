#!/usr/bin/env bash
# Entrypoint for the deployment image: source ROS, the micro-ROS agent
# sibling workspace, and the main workspace overlay, then exec whatever
# command was passed.
set -eo pipefail

# ROS / colcon setup scripts reference optional vars like
# AMENT_TRACE_SETUP_FILES and COLCON_TRACE without `:-` guards, so they
# break under `set -u`. nounset is intentionally not enabled here.

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /opt/outdoor-patrol/uros_agent_ws/install/local_setup.bash ]]; then
    source /opt/outdoor-patrol/uros_agent_ws/install/local_setup.bash
fi
if [[ -f /opt/outdoor-patrol/install/setup.bash ]]; then
    source /opt/outdoor-patrol/install/setup.bash
fi

# Wait for the host network to come up before launching ROS nodes. The
# Pi's WiFi often isn't ready when Docker (re)starts the container at
# boot; without this guard, DDS/Cyclone binds to an incomplete stack
# and discovery silently fails. We wait for a default route (up to
# WAIT_FOR_NETWORK_SECS, default 30s) and otherwise let the container
# fail and be restarted by the compose policy.
WAIT_FOR_NETWORK_SECS="${WAIT_FOR_NETWORK_SECS:-30}"
for _ in $(seq 1 "${WAIT_FOR_NETWORK_SECS}"); do
    if ip route show default 2>/dev/null | grep -q '^default '; then
        break
    fi
    sleep 1
done
if ! ip route show default 2>/dev/null | grep -q '^default '; then
    echo "entrypoint: no default route after ${WAIT_FOR_NETWORK_SECS}s; exiting for restart" >&2
    exit 1
fi

exec "$@"
