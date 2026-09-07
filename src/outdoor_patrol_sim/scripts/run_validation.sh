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
# The Nav2 variants (doc/eng/plans/nav2-migration/, Phase 1) reuse the SAME
# world, the SAME recorded route and the SAME scorer, and swap only the thing
# under test -- route_follower for Nav2 + patrol_mission:
#
#   r3n  R3-N  clean loop under Nav2. Must stay within 2x the R3 baseline RMS.
#   r5n  R5-N  degraded GNSS under Nav2: patrol_mission cancels the goal.
#
# Usage:
#   ./run_validation.sh [outdir] [runs...]
#   ./run_validation.sh /tmp/val            # everything
#   ./run_validation.sh /tmp/val r4         # just R4, reusing an earlier route
#
#   ./run_validation.sh /tmp/val teach r3 r3n   # baseline and Nav2, same route
#
#   GUI=1 ./run_validation.sh /tmp/val r4   # + Gazebo GUI and RViz
#
#   GNSS_SIGMA=0.05 ./run_validation.sh /tmp/val r3   # re-run at a worse fix
#   GUI=rviz ./run_validation.sh /tmp/val   # RViz only (much lighter)
#   GUI=gz   ./run_validation.sh /tmp/val   # Gazebo GUI only
#
# GUI needs a display ($DISPLAY or a Wayland socket). It slows the sim down,
# so the pass/fail numbers still come from a headless run -- watch with the
# GUI, score without it.
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
NAV_SHARE="$WS/install/share/outdoor_patrol_nav"
# --- world selection -------------------------------------------------------
#
# Everything below defaults to the 100 m patrol_road the Phase 0 baseline and
# the Phase 1 parity numbers were measured on. They are overridable by
# environment variable so a differently-scaled world -- e.g. the 18 ft
# driveway loop -- can be driven through the same harness without editing
# this file or disturbing those numbers:
#
#   WORLD=driveway.sdf \
#   CENTERLINE=$PWD/install/share/outdoor_patrol_sim/worlds/driveway_centerline.yaml \
#   NAV_PARAMS=$PWD/install/share/outdoor_patrol_nav/config/nav2_params_driveway.yaml \
#   MISSION_PARAMS=$PWD/install/share/outdoor_patrol_nav/config/patrol_mission_driveway.yaml \
#   BT_XML=$PWD/install/share/outdoor_patrol_nav/bt/patrol_driveway.xml \
#   START_X=-1.4332 START_Y=-2.4332 START_YAW=0.0 TEACH_SPEED=0.35 \
#   GUI=1 ./run_validation.sh /tmp/driveway teach r3n
#
# A scaled world is NOT a validated configuration: the R3/R3-N pass criteria
# are tuned to the 100 m road, so read the scores as observations rather than
# as a gate.
WORLD="${WORLD:-patrol_road.sdf}"
OBSTACLE_WORLD="${OBSTACLE_WORLD:-patrol_road_obstacles.sdf}"
CENTERLINE="${CENTERLINE:-$SHARE/worlds/patrol_road_centerline.yaml}"
NAV_PARAMS="${NAV_PARAMS:-$NAV_SHARE/config/nav2_params.yaml}"
MISSION_PARAMS="${MISSION_PARAMS:-$NAV_SHARE/config/patrol_mission.yaml}"
BT_XML="${BT_XML:-$NAV_SHARE/bt/patrol.xml}"
TEACH_SPEED="${TEACH_SPEED:-0.8}"
# Pure-pursuit lookahead for the teach-pass driver. A lookahead of L on a
# corner of radius R cuts the corner by roughly L^2/(8R), so the 1.5 m default
# costs 0.056 m on the 100 m road's 5 m corners and 0.281 m on a driveway's
# 1 m corners -- the recorded route, not the follower, is then the error.
# Scale it with the corner radius, not with the speed: this is geometry and
# slowing down does not help.
TEACH_LOOKAHEAD="${TEACH_LOOKAHEAD:-1.5}"
SCORE_ROUTE="$WS/src/outdoor_patrol_route/scripts/score_route.py"
SCORE_RUN="$WS/src/outdoor_patrol_route/scripts/score_run.py"

# Parity bar for R3-N: 2x the mean R3 RMS from the Phase 0 baseline runs.
# 0.129 is MEASURED: three runs of `teach r3 r4 r5` on 2026-09-06 gave R3
# RMS 0.0612 / 0.0642 / 0.0681 m, mean 0.0645 m. See
# doc/eng/plans/nav2-migration/runs/baseline/baseline.md for the machine and
# the full table. Re-measure if the world, the chassis params or the
# follower's gains change.
NAV_MAX_RMS="${NAV_MAX_RMS:-0.129}"

# s = 0 of the generated road, heading east along the south straight.
START_X="${START_X:--8.573}"
START_Y="${START_Y:--13.573}"
START_YAW="${START_YAW:-0.0}"

# Node names that mean "a stack is already running". The nav2 servers are in
# here for the same reason as the EKFs: a leftover controller_server still
# publishes /cmd_vel_nav, and the next run would drive on it.
STACK_NODES='^/(gz_bridge|gnss_sim|odom_sim|ekf_global|ekf_filter_node|navsat_transform|confidence_gate|heading_to_imu|scan_safety'
STACK_NODES="$STACK_NODES"'|controller_server|planner_server|smoother_server'
STACK_NODES="$STACK_NODES"'|behavior_server|bt_navigator|velocity_smoother'
STACK_NODES="$STACK_NODES"'|lifecycle_manager_navigation|patrol_mission)$'

mkdir -p "$OUTDIR"
# colcon's setup.bash reads unset variables; -u must be off while sourcing.
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

FAILED=0
SIM_PGID=""
OWN_PGID="$(ps -o pgid= -p $$ | tr -d ' ')"

# GUI= unset|0  headless (default, and what the numbers are measured on)
#      1|all    Gazebo GUI + RViz
#      gz       Gazebo GUI only
#      rviz     RViz only -- much lighter, and shows the corridor markers,
#               which is usually what you actually want to watch
GUI="${GUI:-0}"
case "$GUI" in
  1|all|true)  GZ_GUI=true;  RVIZ_GUI=true ;;
  gz|gazebo)   GZ_GUI=true;  RVIZ_GUI=false ;;
  rviz)        GZ_GUI=false; RVIZ_GUI=true ;;
  *)           GZ_GUI=false; RVIZ_GUI=false ;;
esac

if [ "$GZ_GUI" = true ] || [ "$RVIZ_GUI" = true ]; then
  if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "GUI requested but neither DISPLAY nor WAYLAND_DISPLAY is set." >&2
    exit 1
  fi
  echo "GUI mode: gazebo=$GZ_GUI rviz=$RVIZ_GUI on ${DISPLAY:-$WAYLAND_DISPLAY}"
  echo "Rendering competes with physics -- treat these runs as a look, not"
  echo "as the measurement."
  # Everything downstream waits in WALL time while the robot advances in SIM
  # time, so a lower real-time factor has to buy proportionally longer
  # patience or the harness gives up on a run that is merely slow.
  READY_TIMEOUT=450
  FLAG_TIMEOUT=1800
  # Each readiness probe is a fresh `ros2 topic echo`, and most of its cost is
  # process start-up plus discovery, not waiting for a message. With Gazebo
  # and RViz competing for the CPU that start-up alone blows past 3 s, so the
  # probe times out forever on a topic that is publishing perfectly well.
  PROBE_TIMEOUT=15
else
  READY_TIMEOUT=150
  FLAG_TIMEOUT=500
  PROBE_TIMEOUT=3
fi

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
  timeout "$((PROBE_TIMEOUT * 2))" ros2 node list 2>/dev/null \
      | grep -E "$STACK_NODES" | tr '\n' ' '
}

reap_stragglers() {
  # `ros2 launch` starts some of its children in their own sessions, so a
  # process-group kill can miss them -- and a surviving bridge or EKF poisons
  # the next run. Find what is left by executable and end it explicitly.
  local pattern pids pid
  pattern='gz sim|parameter_bridge|ekf_node|navsat_transform_node'
  pattern="$pattern"'|install/lib/outdoor_patrol|robot_state_publisher'
  # The nav2 servers come from /opt/ros, not from install/lib/outdoor_patrol,
  # so the line above does not reach them. Matched by executable path to avoid
  # matching this script's own arguments.
  pattern="$pattern"'|lib/nav2_[a-z_]*/(controller_server|planner_server)'
  pattern="$pattern"'|lib/nav2_[a-z_]*/(smoother_server|behavior_server)'
  pattern="$pattern"'|lib/nav2_[a-z_]*/(bt_navigator|velocity_smoother)'
  pattern="$pattern"'|lib/nav2_[a-z_]*/lifecycle_manager'
  pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
  [ -z "$pids" ] && return 0
  note "reaping stragglers: $(echo "$pids" | tr '\n' ' ')"
  for pid in $pids; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  sleep 2
}

wait_for_quiet() {
  # Both the process table and the ROS graph have to settle: discovery keeps
  # a dead node listed for a few seconds after its process is gone.
  for _ in $(seq 1 20); do
    if ! pgrep -f 'gz sim' > /dev/null 2>&1 && [ -z "$(stack_nodes)" ]; then
      return 0
    fi
    sleep 1
  done
  # Still here after 20 s: something escaped the group kill. Take it out
  # rather than starting the next run on top of it.
  reap_stragglers
  for _ in $(seq 1 20); do
    [ -z "$(stack_nodes)" ] && return 0
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
trap 'stop_group "$SIM_PGID"; reap_stragglers' EXIT

start_sim() {
  local world="$1" log_file="$2" nav="${3:-false}"
  local stale
  stale="$(stack_nodes)"
  if [ -n "$stale" ]; then
    echo "    ERROR: stack nodes already on the ROS graph: $stale" >&2
    # These node names are shared between the sim and the REAL robot, so this
    # is not only a leftover-process check. If the robot is powered up on the
    # same DDS domain, starting a sim here would publish /cmd_vel into a graph
    # that the ESP32 drive node is subscribed to -- and the real robot would
    # move. Check before you clear this.
    if timeout "$PROBE_TIMEOUT" ros2 node list 2>/dev/null \
        | grep -qE '^/(um982_driver|esp32_drive|sllidar_node|imu_driver)$'; then
      echo "    *** A REAL ROBOT IS ON THIS ROS GRAPH. ***" >&2
      echo "    Driver nodes are live, so /cmd_vel from a sim would reach" >&2
      echo "    the actual chassis. Power the robot down, or put it on a" >&2
      echo "    different ROS_DOMAIN_ID, before running the sim." >&2
    fi
    return 1
  fi

  log "starting sim: $(basename "$world")$([ "$nav" = true ] && echo ' + nav2')"
  SIM_PGID="$(spawn_group sim "$log_file" \
      ros2 launch outdoor_patrol_sim sim.launch.py \
      world:="$SHARE/worlds/$world" \
      x:="$START_X" y:="$START_Y" yaw:="$START_YAW" \
      gui:="$GZ_GUI" use_rviz:="$RVIZ_GUI" \
      nav:="$nav" \
      nav_params_file:="$NAV_PARAMS" \
      bt_xml:="$BT_XML" \
      rviz_config:="$ROUTE_SHARE/config/route.rviz")"
  if [ -z "$SIM_PGID" ]; then
    echo "    ERROR: the sim never reported its process group" >&2
    return 1
  fi

  # Ready means the global EKF is publishing, not merely that gz is up.
  local waited=0
  while [ "$waited" -lt "$READY_TIMEOUT" ]; do
    if timeout "$PROBE_TIMEOUT" ros2 topic echo /odometry/global --once \
        > /dev/null 2>&1; then
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
    if timeout "$PROBE_TIMEOUT" ros2 topic echo "$topic" --once 2>/dev/null \
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
  start_sim "$WORLD" "$OUTDIR/sim_teach.log" || { FAILED=1; return 1; }

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
      -p speed_ms:="$TEACH_SPEED" -p laps:=1.0 \
      -p lookahead_m:="$TEACH_LOOKAHEAD")"

  wait_for_flag /sim_route_driver/finished "$FLAG_TIMEOUT" || FAILED=1
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

  # A "-N" label runs the same scenario against Nav2 + patrol_mission instead
  # of route_follower. World, route, scorer and thresholds are deliberately
  # identical, so a difference in the score is a difference in the controller
  # and nothing else.
  local nav=false
  local status_topic=/route_follower/status
  local finished_topic=/route_follower/finished
  case "$label" in
    *-N)
      nav=true
      status_topic=/patrol_mission/status
      finished_topic=/patrol_mission/finished
      ;;
  esac

  if [ ! -f "$route" ]; then
    echo "    ERROR: $route missing -- run the teach pass first" >&2
    FAILED=1
    return 1
  fi

  start_sim "$world" "$OUTDIR/sim_$label.log" "$nav" || { FAILED=1; return 1; }

  # GNSS_SIGMA lets a run be repeated at the accuracy a real correction
  # service actually delivers, rather than the 2 cm the sim defaults to.
  # Set it to the top of your provider's spec and see whether the stack still
  # tracks -- that is a measurement, not a datasheet argument.
  if [ -n "${GNSS_SIGMA:-}" ]; then
    note "setting simulated fix sigma to ${GNSS_SIGMA} m"
    ros2 param set /gnss_sim horizontal_stddev_m "$GNSS_SIGMA" > /dev/null 2>&1 \
      || note "WARNING: could not set gnss_sim sigma"
  fi

  # The scorer reads /odom_truth, the status topic and /cmd_vel. The rest is
  # for the human reading the bag afterwards: /cmd_vel_nav shows what the
  # controller asked for before the smoother and the brake got to it, and
  # /plan shows what Hybrid-A* actually handed to MPPI.
  local topics=(/odom_truth "$status_topic" /cmd_vel /cmd_vel_raw
                /gnss/fix_gated)
  if [ "$nav" = true ]; then
    topics+=(/cmd_vel_nav /plan)
  fi

  rm -rf "${OUTDIR:?}/$bag"
  local recorder
  recorder="$(spawn_group "bag_$label" "$OUTDIR/$bag.log" \
      ros2 bag record -o "$OUTDIR/$bag" "${topics[@]}")"
  sleep 3

  log "$label -- following the route"
  # --params-file first: ROS 2 applies overrides in order, and both parameter
  # files carry an empty route_path that would otherwise win.
  local follower
  if [ "$nav" = true ]; then
    # patrol_mission retries until bt_navigator is active, so it is safe to
    # start it before the lifecycle manager has finished bringing Nav2 up.
    follower="$(spawn_group "follow_$label" "$OUTDIR/follower_$label.log" \
        ros2 run outdoor_patrol_nav patrol_mission --ros-args \
        --params-file "$MISSION_PARAMS" \
        -p use_sim_time:=true \
        -p route_path:="$route")"
  else
    follower="$(spawn_group "follow_$label" "$OUTDIR/follower_$label.log" \
        ros2 run outdoor_patrol_route route_follower --ros-args \
        --params-file "$ROUTE_SHARE/config/route.yaml" \
        -p use_sim_time:=true \
        -p route_path:="$route")"
  fi

  case "$label" in
    R5|R5-N)
      # Let it settle into the lane, then degrade the fix past sigma_stop.
      sleep 45
      log "$label -- degrading the simulated fix to sigma 0.8 m"
      ros2 param set /gnss_sim horizontal_stddev_m 0.8 \
          >> "$OUTDIR/follower_$label.log" 2>&1
      sleep 45
      ;;
    *)
      wait_for_flag "$finished_topic" "$FLAG_TIMEOUT" || FAILED=1
      ;;
  esac

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
      --status-topic "$status_topic" \
      --label "$label" --json "$OUTDIR/score_$label.json" "$@" || FAILED=1
}

for run in "${RUNS[@]}"; do
  case "$run" in
    teach) run_teach ;;
    r3) follow_run R3 "$WORLD" bag_r3 ;;
    r4) follow_run R4 "$OBSTACLE_WORLD" bag_r4 --expect-obstacles ;;
    r5) follow_run R5 "$WORLD" bag_r5 --expect-degraded \
            --min-laps 0.0 --max-stop-s 1e9 ;;
    # Nav2 (doc/eng/plans/nav2-migration/phase-1.md). No r4n: R4 scores the
    # retreat from the COMMANDED lateral offset, which is identically zero
    # under Nav2. Phase 2 replaces that check before there is an R4-N.
    r3n) follow_run R3-N "$WORLD" bag_r3n --max-rms "$NAV_MAX_RMS" ;;
    r5n) follow_run R5-N "$WORLD" bag_r5n --expect-degraded \
            --min-laps 0.0 --max-stop-s 1e9 ;;
    *) echo "unknown run: $run" >&2; FAILED=1 ;;
  esac
done

log "validation complete: $([ "$FAILED" -eq 0 ] && echo PASS || echo FAIL)"
note "artefacts in $OUTDIR"
exit "$FAILED"
