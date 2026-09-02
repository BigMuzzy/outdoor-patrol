#!/usr/bin/env bash
# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
#
# End-to-end simulation validation of the teach-and-repeat stack (issue #8).
# See doc/eng/plans/issue-8-teach-and-repeat.md for what each run proves.
#
#   R1  teach pass produces a usable route
#   R2  the base_link correction is measurably doing something
#   R3  clean loop: tracks the centerline, never trips the forward brake
#   R4  three barriers: goes around each on the RIGHT shoulder, then resumes
#   R5  degraded GNSS: slows, then stops
#
# Usage:
#   ./run_validation.sh [outdir] [runs...]
#   ./run_validation.sh /tmp/val            # everything
#   ./run_validation.sh /tmp/val r4         # just R4, reusing an earlier route
#
# Every child is spawned into its OWN process group and torn down by group.
# Signalling `ros2 launch` or `ros2 run` alone does not reliably reach what
# they spawned, and a surviving Gazebo server or bridge silently shares
# gz-transport and ROS topic names with the next run -- two /clock publishers,
# duplicate nodes, and an /odometry/global that tracks nothing.

set -u -o pipefail

OUTDIR="${1:-/tmp/patrol_validation}"
shift || true
RUNS=("$@")
[ ${#RUNS[@]} -eq 0 ] && RUNS=(teach r3 r4 r5)

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SHARE="$WS/install/share/outdoor_patrol_sim"
ROUTE_SHARE="$WS/install/share/outdoor_patrol_route"
CENTERLINE="$SHARE/worlds/patrol_road_centerline.yaml"
SCORE_ROUTE="$WS/src/outdoor_patrol_route/scripts/score_route.py"
SCORE_RUN="$WS/src/outdoor_patrol_route/scripts/score_run.py"

# s = 0 of the generated road, heading east along the south straight.
START_X=-8.573
START_Y=-13.573
START_YAW=0.0

# Node names that mean "a stack is already running".
STACK_NODES='^/(gz_bridge|gnss_sim|odom_sim|ekf_global|ekf_filter_node|navsat_transform|confidence_gate|heading_to_imu|scan_safety)$'

mkdir -p "$OUTDIR"
# colcon's setup.bash reads unset variables; -u must be off while sourcing.
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

FAILED=0
SIM_PGID=""
OWN_PGID="$(ps -o pgid= -p $$ | tr -d ' ')"

log()  { printf '\n=== %s\n' "$*"; }
note() { printf '    %s\n' "$*"; }

# --- process groups --------------------------------------------------------

spawn_group() {     # tag, log_file, command...
  local tag="$1" log_file="$2"; shift 2
  local pgid_file="$OUTDIR/.pgid_$tag"
  rm -f "$pgid_file"
  # The child reports its own group id: `setsid` may fork, so $! in this
  # shell is not reliably the group leader, and guessing wrong risks
  # signalling THIS script's group.
  setsid bash -c 'echo $$ > "$1"; shift; exec "$@"' _ "$pgid_file" "$@" \
      > "$log_file" 2>&1 &
  for _ in $(seq 1 40); do
    [ -s "$pgid_file" ] && break
    sleep 0.25
  done
  cat "$pgid_file" 2>/dev/null
}

stop_group() {      # pgid [first signal]
  local pgid="${1:-}"
  [ -z "$pgid" ] && return 0
  if [ "$pgid" = "$OWN_PGID" ]; then
    echo "    ERROR: refusing to signal our own process group" >&2
    return 1
  fi
  kill -"${2:-TERM}" -"$pgid" 2>/dev/null
  for _ in $(seq 1 40); do
    kill -0 -"$pgid" 2>/dev/null || return 0
    sleep 0.5
  done
  kill -KILL -"$pgid" 2>/dev/null
  return 0
}

signal_group() {    # pgid, signal -- send only, do not escalate
  local pgid="${1:-}"
  [ -z "$pgid" ] && return 0
  if [ "$pgid" = "$OWN_PGID" ]; then
    echo "    ERROR: refusing to signal our own process group" >&2
    return 1
  fi
  kill -"$2" -"$pgid" 2>/dev/null
  return 0
}

stack_nodes() {
  timeout 10 ros2 node list 2>/dev/null | grep -E "$STACK_NODES" | tr '\n' ' '
}

wait_for_quiet() {
  # Both the process table and the ROS graph have to settle: discovery keeps
  # a dead node listed for a few seconds after its process is gone.
  for _ in $(seq 1 45); do
    if ! pgrep -f 'gz sim' > /dev/null 2>&1 && [ -z "$(stack_nodes)" ]; then
      return 0
    fi
    sleep 1
  done
  note "WARNING: the previous stack has not fully gone away"
  return 1
}

stop_sim() {
  stop_group "$SIM_PGID"
  SIM_PGID=""
  wait_for_quiet
}
trap 'stop_group "$SIM_PGID"' EXIT

start_sim() {
  local world="$1" log_file="$2"
  local stale
  stale="$(stack_nodes)"
  if [ -n "$stale" ]; then
    echo "    ERROR: stack nodes already on the ROS graph: $stale" >&2
    return 1
  fi

  log "starting sim: $(basename "$world")"
  SIM_PGID="$(spawn_group sim "$log_file" \
      ros2 launch outdoor_patrol_sim sim.launch.py \
      world:="$SHARE/worlds/$world" \
      x:="$START_X" y:="$START_Y" yaw:="$START_YAW")"
  if [ -z "$SIM_PGID" ]; then
    echo "    ERROR: the sim never reported its process group" >&2
    return 1
  fi

  # Ready means the global EKF is publishing, not merely that gz is up.
  local waited=0
  while [ "$waited" -lt 150 ]; do
    if timeout 3 ros2 topic echo /odometry/global --once > /dev/null 2>&1; then
      note "localization live after ${waited}s"
      return 0
    fi
    sleep 3
    waited=$((waited + 3))
  done
  echo "    ERROR: /odometry/global never appeared; see $log_file" >&2
  return 1
}

wait_for_flag() {   # topic, timeout seconds
  local topic="$1" limit="$2" waited=0
  while [ "$waited" -lt "$limit" ]; do
    if timeout 3 ros2 topic echo "$topic" --once 2>/dev/null \
        | grep -q 'data: true'; then
      return 0
    fi
    sleep 5
    waited=$((waited + 5))
  done
  echo "    ERROR: $topic never went true within ${limit}s" >&2
  return 1
}

# --- runs ------------------------------------------------------------------

run_teach() {
  start_sim patrol_road.sdf "$OUTDIR/sim_teach.log" || { FAILED=1; return 1; }

  local groups=()
  for source in odometry_global fix_lever_arm raw_antenna; do
    groups+=("$(spawn_group "rec_$source" "$OUTDIR/rec_$source.log" \
        ros2 run outdoor_patrol_route route_recorder --ros-args \
        -r __node:="rec_$source" \
        -p use_sim_time:=true \
        -p source:="$source" \
        -p output_path:="$OUTDIR/route_$source.yaml")")
  done
  sleep 5

  log "R1 -- driving the teach pass"
  local driver
  driver="$(spawn_group driver "$OUTDIR/teach_driver.log" \
      ros2 run outdoor_patrol_sim sim_route_driver --ros-args \
      -p use_sim_time:=true \
      -p centerline_path:="$CENTERLINE" \
      -p speed_ms:=0.8 -p laps:=1.0)"

  wait_for_flag /sim_route_driver/finished 400 || FAILED=1
  stop_group "$driver"

  for source in odometry_global fix_lever_arm raw_antenna; do
    timeout 180 ros2 service call "/rec_$source/save" std_srvs/srv/Trigger \
        >> "$OUTDIR/save.log" 2>&1 || FAILED=1
  done
  for pgid in "${groups[@]}"; do stop_group "$pgid"; done
  stop_sim

  log "R1 + R2 -- scoring the recorded routes"
  python3 "$SCORE_ROUTE" --centerline "$CENTERLINE" \
      --route "$OUTDIR/route_odometry_global.yaml" \
      --route "$OUTDIR/route_fix_lever_arm.yaml" \
      --route "$OUTDIR/route_raw_antenna.yaml" \
      --json "$OUTDIR/score_route.json" || FAILED=1
}

follow_run() {      # label, world, bag name, extra scorer args...
  local label="$1" world="$2" bag="$3"; shift 3
  local route="$OUTDIR/route_odometry_global.yaml"

  if [ ! -f "$route" ]; then
    echo "    ERROR: $route missing -- run the teach pass first" >&2
    FAILED=1
    return 1
  fi

  start_sim "$world" "$OUTDIR/sim_$label.log" || { FAILED=1; return 1; }

  rm -rf "${OUTDIR:?}/$bag"
  local recorder
  recorder="$(spawn_group "bag_$label" "$OUTDIR/$bag.log" \
      ros2 bag record -o "$OUTDIR/$bag" \
      /odom_truth /route_follower/status /cmd_vel /cmd_vel_raw \
      /gnss/fix_gated)"
  sleep 3

  log "$label -- following the route"
  # --params-file first: ROS 2 applies overrides in order, and route.yaml
  # carries an empty route_path that would otherwise win.
  local follower
  follower="$(spawn_group "follow_$label" "$OUTDIR/follower_$label.log" \
      ros2 run outdoor_patrol_route route_follower --ros-args \
      --params-file "$ROUTE_SHARE/config/route.yaml" \
      -p use_sim_time:=true \
      -p route_path:="$route")"

  if [ "$label" = "R5" ]; then
    # Let it settle into the lane, then degrade the fix past sigma_stop.
    sleep 45
    log "R5 -- degrading the simulated fix to sigma 0.8 m"
    ros2 param set /gnss_sim horizontal_stddev_m 0.8 \
        >> "$OUTDIR/follower_$label.log" 2>&1
    sleep 45
  else
    wait_for_flag /route_follower/finished 500 || FAILED=1
  fi

  stop_group "$follower"
  sleep 2
  # SIGTERM and then WAIT. rosbag2 writes mcap in chunks and loses the chunk
  # it is filling if it is killed, which silently truncated a lap by its last
  # 6 seconds -- the run then scores as 0.95 laps and fails for no reason the
  # controller is responsible for. SIGINT alone does not make it close the
  # file when it has no controlling terminal; SIGTERM does. metadata.yaml
  # appearing is the signal that the writer has closed cleanly, so only
  # escalate after that or after a generous timeout.
  signal_group "$recorder" TERM
  local flushed=0
  for _ in $(seq 1 60); do
    if [ -f "$OUTDIR/$bag/metadata.yaml" ]; then
      flushed=1
      break
    fi
    sleep 1
  done
  if [ "$flushed" -eq 0 ]; then
    note "WARNING: rosbag2 did not close $bag cleanly; the tail may be lost"
  fi
  stop_group "$recorder"
  stop_sim

  log "$label -- scoring"
  python3 "$SCORE_RUN" --bag "$OUTDIR/$bag" --centerline "$CENTERLINE" \
      --label "$label" --json "$OUTDIR/score_$label.json" "$@" || FAILED=1
}

for run in "${RUNS[@]}"; do
  case "$run" in
    teach) run_teach ;;
    r3) follow_run R3 patrol_road.sdf bag_r3 ;;
    r4) follow_run R4 patrol_road_obstacles.sdf bag_r4 --expect-obstacles ;;
    r5) follow_run R5 patrol_road.sdf bag_r5 --expect-degraded \
            --min-laps 0.0 --max-stop-s 1e9 ;;
    *) echo "unknown run: $run" >&2; FAILED=1 ;;
  esac
done

log "validation complete: $([ "$FAILED" -eq 0 ] && echo PASS || echo FAIL)"
note "artefacts in $OUTDIR"
exit "$FAILED"
