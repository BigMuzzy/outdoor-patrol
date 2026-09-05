# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Field validation dashboard: the node behind the RViz panel.

Subscribes to everything ``doc/eng/plans/field-validation-alley.md`` tells the
operator to watch, runs the Phase 0-7 gates over it, and publishes one JSON
document on ``~/state`` that the panel renders. Commands come back on
``~/command``.

Why a node and not a shell script full of ``ros2 topic echo``: the plan's
gates are *temporal*. "sigma <= 5 cm held for ten minutes with no dropouts"
and "d_cmd never positive" and "never stationary more than 3 s" cannot be read
off a scrolling terminal -- they have to be accumulated by something that is
watching continuously. Doing that by eye, outdoors, next to a moving robot, is
how a failed phase gets recorded as a pass.

The node deliberately holds no display code and the panel holds no gate logic.
That split means the dashboard also runs headless on the robot -- start it
before you drive, and the report is written whether or not anyone had RViz
open.

Interfaces
----------

``~/state`` (``std_msgs/String``, JSON, 5 Hz)
    ``{"signals": {...}, "phases": [...], "active": int|null, "log": [...]}``

``~/command`` (``std_msgs/String``, JSON)
    ``{"action": "start"|"stop"|"reset"|"mark"|"report"|"note",
       "phase": int, "verdict": "pass"|"fail", "text": str}``

JSON over ``String`` rather than a custom interface package, matching
``route_follower``'s existing ``~/status``: it keeps the panel free of
generated headers and stays greppable with ``ros2 topic echo`` when the panel
itself is what is broken.
"""

from __future__ import annotations

import datetime
import json
import math
import os
import time
from typing import List, Optional

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robot_localization.srv import FromLL
from rtcm_msgs.msg import Message as RtcmMessage
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from outdoor_patrol_validation import phases as phase_defs
from outdoor_patrol_validation.managed_process import ManagedProcess
from outdoor_patrol_validation.signals import Signals

#: Report template. Phases the operator never ran are recorded as skipped
#: rather than silently dropped -- "a phase that passed and you do not know
#: why is not a result".
_REPORT_HEADER = """# Field validation report

* Site: {site}
* Started: {started}
* Written: {written}
* Stack: {stack}

| Phase | Verdict | Elapsed | Gate |
|---|---|---|---|
"""


class FieldDashboard(Node):
    """Accumulates the stack's signals into per-phase verdicts."""

    def __init__(self) -> None:
        super().__init__('field_dashboard')

        self.declare_parameter('fix_topic', '/um982_driver/fix')
        self.declare_parameter('fix_gated_topic', '/gnss/fix_gated')
        self.declare_parameter('nmea_topic', '/um982_driver/nmea_sentence')
        self.declare_parameter('rtcm_topic', '/rtcm')
        self.declare_parameter('heading_topic', '/gnss/heading')
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('status_topic', '/route_follower/status')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('fromll_service', '/fromLL')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('site', 'alley')
        self.declare_parameter('report_dir', 'runs/field')
        #: Bearing of robot-forward in the RAW scan frame. Like scan_safety,
        #: this node reads raw scan angles instead of transforming through TF,
        #: so it MUST match the lidar_link yaw in chassis.yaml. Wrong here and
        #: the front and rear sectors swap silently.
        self.declare_parameter('scan_forward_offset_deg', 180.0)
        #: Phase 4 only trusts the lever arm while the robot is still: the
        #: fix, the odometry and the service round-trip are not co-stamped,
        #: and at speed that latency shows up as a spurious offset.
        self.declare_parameter('lever_arm_max_speed_ms', 0.1)
        #: Matches route_alley.yaml's toll_timeout_s. A /fromLL that accepts a
        #: request and never answers must not stall Phase 4 for ever.
        self.declare_parameter('fromll_timeout_s', 5.0)

        # -- managed nodes ------------------------------------------------
        #: Phase 3 Start launches a route_recorder; Stop saves and stops it.
        #: On by default: recording only reads topics and writes a file, and
        #: the alternative is a teach pass that silently records nothing.
        self.declare_parameter('manage_recorder', True)
        #: Phase 5/6 Start launches route_follower. OFF by default because it
        #: MOVES THE ROBOT, which in the field must stay a deliberate act by
        #: someone holding a kill switch. The sim profile turns it on.
        self.declare_parameter('manage_follower', False)
        self.declare_parameter('route_dir', 'runs/routes')
        #: Parameter file handed to both recorder and follower. Empty means
        #: their package defaults.
        self.declare_parameter('route_params_file', '')
        #: Route the follower drives. Empty means "the last one recorded",
        #: which is what makes teach-then-repeat work without retyping paths.
        self.declare_parameter('route_path', '')
        self.declare_parameter('follower_speed_ms', 0.4)
        #: Starting values for the panel's live-tuning box. They are the
        #: dashboard's, not the follower's, so they exist before a run does.
        self.declare_parameter('follower_corridor_m', 3.0)
        self.declare_parameter('follower_avoidance', True)
        #: Draw the lane/corridor bands. Purely a display choice -- on a
        #: tight route the folded offsets clutter the scene.
        self.declare_parameter('follower_show_corridor', True)
        #: Children need this to match the stack; the sim runs on /clock.
        self.declare_parameter('child_use_sim_time', False)
        self.declare_parameter('log_dir', '/tmp')

        for name, default in vars(phase_defs.Thresholds()).items():
            self.declare_parameter(f'thresholds.{name}', default)

        self._thresholds = phase_defs.Thresholds(**{
            name: self._typed(f'thresholds.{name}', default)
            for name, default in vars(phase_defs.Thresholds()).items()})

        self._signals = Signals(
            forward_offset_deg=float(
                self.get_parameter('scan_forward_offset_deg').value))
        self._phases = phase_defs.build_phases(self._thresholds)
        self._active: Optional[int] = None
        self._log: List[str] = []
        self._started_wall = datetime.datetime.now()
        self._last_update = time.monotonic()

        self._recorder: Optional[ManagedProcess] = None
        self._follower: Optional[ManagedProcess] = None
        #: Where the running recorder is writing.
        self._pending_route: Optional[str] = None
        #: Last successfully saved route -- what Phase 5/6 will drive.
        self._last_route: Optional[str] = self._newest_route()
        self._apply_pending_timer = None
        self._apply_deadline = 0.0
        #: Follower tuning owned by the DASHBOARD, so it can be set before a
        #: run exists and handed to the follower when one is launched.
        self._desired = {
            'avoidance_enabled': bool(
                self.get_parameter('follower_avoidance').value),
            'nominal_speed_ms': float(
                self.get_parameter('follower_speed_ms').value),
            'corridor_half_width_m': float(
                self.get_parameter('follower_corridor_m').value),
            'show_corridor': bool(
                self.get_parameter('follower_show_corridor').value),
        }

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE

        self._state_pub = self.create_publisher(String, '~/state', reliable)
        # Plain reliable, not latched: a latched command would be replayed to
        # a dashboard that restarts mid-procedure and silently re-start a
        # phase the operator had already stopped.
        self.create_subscription(String, '~/command', self._on_command, reliable)

        p = self.get_parameter
        self.create_subscription(
            NavSatFix, p('fix_topic').value, self._on_fix, reliable)
        self.create_subscription(
            NavSatFix, p('fix_gated_topic').value, self._on_fix_gated, reliable)
        self.create_subscription(
            Sentence, p('nmea_topic').value, self._on_nmea,
            qos_profile_sensor_data)
        self.create_subscription(
            RtcmMessage, p('rtcm_topic').value, self._on_rtcm, QoSProfile(depth=50))
        self.create_subscription(
            Imu, p('heading_topic').value, self._on_heading,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, p('odom_topic').value, self._on_odom, reliable)
        self.create_subscription(
            String, p('status_topic').value, self._on_status, reliable)
        self.create_subscription(
            LaserScan, p('scan_topic').value, self._on_scan,
            qos_profile_sensor_data)
        self.create_subscription(
            Twist, p('cmd_topic').value, self._on_cmd, reliable)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Its own group: the lever-arm round trip must not block the gate
        # timer, or a missing /fromLL freezes the whole dashboard.
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._fromll = self.create_client(
            FromLL, p('fromll_service').value,
            callback_group=self._service_group)
        self._fromll_busy = False
        self._fromll_sent: Optional[float] = None

        period = 1.0 / max(float(p('publish_rate_hz').value), 0.5)
        self.create_timer(period, self._tick)
        self.create_timer(0.5, self._lever_arm_tick,
                          callback_group=self._service_group)

        self._note(f'dashboard up; {len(self._phases)} phases loaded')
        if self._last_route:
            self._note('picked up existing route '
                       f'{os.path.basename(self._last_route)} -- Phase 5 will '
                       'follow it unless you re-teach')
        # Two dashboards publishing the same state topic interleave their
        # messages, and the panel then shows alternating data from both. The
        # symptom is baffling -- values that flicker, settings that "do not
        # take" -- so say it plainly instead of leaving it to be discovered.
        self.create_timer(3.0, self._check_for_duplicates)
        self.get_logger().info(
            'field dashboard ready -- state on %s, commands on %s'
            % (self._state_pub.topic_name, self.get_name() + '/command'))

    def _typed(self, name: str, default):
        value = self.get_parameter(name).value
        return type(default)(value) if value is not None else default

    # -- ingest ------------------------------------------------------------

    def _now(self) -> float:
        return time.monotonic()

    def _on_fix(self, msg: NavSatFix) -> None:
        self._signals.on_fix(msg.latitude, msg.longitude,
                             msg.position_covariance, self._now(),
                             status=int(msg.status.status))

    def _on_fix_gated(self, msg: NavSatFix) -> None:
        self._signals.on_fix_gated(msg.position_covariance, self._now())

    def _on_nmea(self, msg: Sentence) -> None:
        self._signals.on_nmea(msg.sentence, self._now())

    def _on_rtcm(self, msg: RtcmMessage) -> None:
        self._signals.on_rtcm(len(msg.message), self._now())

    def _on_heading(self, msg: Imu) -> None:
        q = msg.orientation
        self._signals.on_heading(q.x, q.y, q.z, q.w, self._now())

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self._signals.on_odometry(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            (q.x, q.y, q.z, q.w), speed, self._now())

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(status, dict):
            self._signals.on_follower_status(status, self._now())

    def _on_scan(self, msg: LaserScan) -> None:
        self._signals.on_scan(list(msg.ranges), msg.angle_min,
                              msg.angle_increment, self._now(),
                              msg.range_min, msg.range_max)

    def _on_cmd(self, msg: Twist) -> None:
        self._signals.on_cmd_vel(msg.linear.x, msg.angular.z, self._now())

    # -- lever arm ---------------------------------------------------------

    def _lever_arm_tick(self) -> None:
        """Convert the raw antenna fix into body-frame offset from base_link.

        This is the live form of the plan's Phase 4 two-recorder comparison.
        Skipped unless the robot is essentially still, so the answer is
        geometry rather than latency.
        """
        now = self._now()
        if self._fromll_busy:
            # A server that accepts a request and never answers would latch
            # this flag for the life of the node. Phase 4 has no other data
            # source, so it would sit in PENDING with nothing to explain it.
            timeout = float(self.get_parameter('fromll_timeout_s').value)
            if (self._fromll_sent is not None
                    and now - self._fromll_sent > timeout):
                self.get_logger().warning(
                    f'/fromLL did not answer within {timeout:.0f} s; retrying')
                self._fromll_busy = False
            else:
                return
        if not self._fromll.service_is_ready():
            return
        snap = self._signals.snapshot(now)
        if not (snap.raw_ok and snap.odom_ok and snap.lat is not None):
            return
        if snap.odom_speed is None:
            return
        if snap.odom_speed > float(
                self.get_parameter('lever_arm_max_speed_ms').value):
            return

        request = FromLL.Request()
        request.ll_point = GeoPoint(latitude=float(snap.lat),
                                    longitude=float(snap.lon),
                                    altitude=0.0)
        origin = (snap.odom_x, snap.odom_y, math.radians(snap.odom_yaw_deg))
        self._fromll_busy = True
        self._fromll_sent = now
        try:
            future = self._fromll.call_async(request)
        except Exception as exc:                      # noqa: BLE001
            self._fromll_busy = False
            self.get_logger().warning(f'/fromLL call failed: {exc}')
            return
        future.add_done_callback(
            lambda f, o=origin: self._on_fromll(f, o))

    def _on_fromll(self, future, origin) -> None:
        self._fromll_busy = False
        try:
            result = future.result()
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warning(f'/fromLL failed: {exc}')
            return
        if result is None:
            return
        x0, y0, yaw = origin
        dx, dy = result.map_point.x - x0, result.map_point.y - y0
        self._signals.on_lever_arm(
            math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
            self._now())

    # -- phase control -----------------------------------------------------

    def _on_command(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warning(f'unparseable command: {msg.data!r}')
            return
        action = command.get('action')
        index = command.get('phase')

        if action == 'start':
            self._start(index)
        elif action == 'stop':
            self._stop()
        elif action == 'reset':
            self._reset(index)
        elif action == 'mark':
            self._mark(index, command.get('verdict'))
        elif action == 'note':
            self._note(str(command.get('text', '')).strip())
        elif action == 'set_param':
            self._set_follower_params(command.get('params') or {})
        elif action == 'report':
            path = self.write_report()
            self._note(f'report written to {path}')
        else:
            self.get_logger().warning(f'unknown action {action!r}')

    def _valid(self, index) -> bool:
        return isinstance(index, int) and 0 <= index < len(self._phases)

    def _start(self, index) -> None:
        if not self._valid(index):
            return
        if self._active is not None and self._active != index:
            self._note(f'phase {self._active} stopped to start {index}')
        # Whatever the previous phase started, stop it: a recorder left
        # running would keep appending to a route the operator has moved on
        # from, and a follower would keep driving.
        self._stop_children()

        phase = self._phases[index]
        phase.reset()
        self._active = index
        self._last_update = self._now()
        self._note(f'phase {index} ({phase.name}) started')

        if index == 3:
            self._start_recorder()
        elif index in (5, 6):
            self._start_follower()

    def _stop(self) -> None:
        if self._active is None:
            return
        phase = self._phases[self._active]
        # Save BEFORE reporting the verdict: saving is what Stop is for on a
        # teach pass, and the message should reflect whether it worked.
        self._stop_children()
        self._note(f'phase {self._active} ({phase.name}) stopped: '
                   f'{phase.verdict()}')
        self._active = None

    def _reset(self, index) -> None:
        if not self._valid(index):
            return
        if self._active == index:
            self._stop_children()
            self._active = None
        self._phases[index].reset()
        self._note(f'phase {index} reset')

    def _mark(self, index, verdict) -> None:
        if not self._valid(index) or verdict not in (phase_defs.PASS,
                                                     phase_defs.FAIL):
            return
        self._phases[index].mark(verdict)
        self._note(f'phase {index} marked {verdict} by the operator')

    def _note(self, text: str) -> None:
        if not text:
            return
        stamp = datetime.datetime.now().strftime('%H:%M:%S')
        self._log.append(f'{stamp}  {text}')
        del self._log[:-200]
        if self._active is not None:
            self._phases[self._active].note(text)
        self.get_logger().info(text)

    # -- managed nodes -----------------------------------------------------

    def _newest_route(self) -> Optional[str]:
        """Most recent route in ``route_dir``, if any.

        So that restarting the dashboard does not lose the teach pass. The
        panel is a thing you close and reopen -- RViz crashes, laptops sleep
        -- and having Phase 5 answer "no route to follow" after a restart,
        with a perfectly good route sitting on disk, is a trap.
        """
        directory = str(self.get_parameter('route_dir').value)
        try:
            files = [os.path.join(directory, f)
                     for f in os.listdir(directory) if f.endswith('.yaml')]
        except OSError:
            return None
        if not files:
            return None
        newest = max(files, key=os.path.getmtime)
        return os.path.abspath(newest)

    def _child_args(self) -> List[str]:
        """Common ros-args for a child: params file, then sim time."""
        args: List[str] = []
        params = self._route_params_file()
        if params:
            args += ['--params-file', params]
        if bool(self.get_parameter('child_use_sim_time').value):
            args += ['-p', 'use_sim_time:=true']
        return args

    def _route_params_file(self) -> str:
        """Resolve `route_params_file`, which may be package-relative.

        Accepts either an absolute path or ``<package>/<file.yaml>``. The
        package form exists so a profile can name the route parameters that
        belong with it -- ``outdoor_patrol_route/route_alley.yaml`` -- without
        hard-coding an install prefix that differs on every machine.

        This matters more than it looks. Left empty, a dashboard-launched
        recorder falls back to route_recorder's own defaults: a 2.0 m lane and
        loop: true. In the alley the plan is explicit that those are the
        numbers that steer a robot into a wall, so the profile has to be able
        to say which file it means.
        """
        spec = str(self.get_parameter('route_params_file').value or '').strip()
        if not spec:
            return ''
        if os.path.isabs(spec):
            return spec if os.path.exists(spec) else self._missing(spec)
        package, _, relative = spec.partition('/')
        if not relative:
            return self._missing(spec)
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory(package),
                                'config', relative)
        except Exception:                             # noqa: BLE001
            return self._missing(spec)
        return path if os.path.exists(path) else self._missing(path)

    def _missing(self, spec: str) -> str:
        self._note(f'route_params_file {spec!r} not found -- children will '
                   'run on their PACKAGE DEFAULTS, which are probably not '
                   'what this site needs')
        return ''

    def _start_recorder(self) -> None:
        """Phase 3: record the teach pass for real."""
        if not bool(self.get_parameter('manage_recorder').value):
            self._note('manage_recorder is off -- start route_recorder '
                       'yourself, or Phase 3 records nothing')
            return
        if self._recorder is not None and self._recorder.running:
            self._recorder.stop()

        directory = str(self.get_parameter('route_dir').value)
        os.makedirs(directory, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        site = str(self.get_parameter('site').value)
        self._pending_route = os.path.abspath(
            os.path.join(directory, f'{site}_{stamp}.yaml'))

        command = ['ros2', 'run', 'outdoor_patrol_route', 'route_recorder',
                   '--ros-args']
        command += self._child_args()
        command += ['-p', f'output_path:={self._pending_route}']
        self._recorder = ManagedProcess(
            'route_recorder', command,
            os.path.join(str(self.get_parameter('log_dir').value),
                         'field_recorder.log'),
            self.get_logger())
        if self._recorder.start():
            self._note(f'recording to {self._pending_route}')
        else:
            self._recorder = None

    def _stop_recorder(self) -> None:
        """Phase 3 Stop: SAVE, then shut the recorder down.

        Saving is the whole point of pressing Stop, and it has to happen
        before the process dies -- the recorder holds the stations in memory
        and only writes them when its save service is called.
        """
        if self._recorder is None or not self._recorder.running:
            return
        saved = False
        # MUST be in a different callback group from the ~/command
        # subscription that is calling this. Otherwise the executor is still
        # inside that callback while we wait here, the response can never be
        # delivered, and the save times out -- the route then only survives
        # because route_recorder happens to write on SIGINT, which is luck
        # rather than design.
        client = self.create_client(Trigger, '/route_recorder/save',
                                    callback_group=self._service_group)
        if client.wait_for_service(timeout_sec=5.0):
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 10.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            result = future.result() if future.done() else None
            if result is not None and result.success:
                saved = True
                self._note(f'route saved: {result.message}')
                self._last_route = self._pending_route
            else:
                self._note('route_recorder/save FAILED -- the teach pass was '
                           'NOT written')
        else:
            self._note('route_recorder/save did not appear; teach pass NOT '
                       'saved')
        self.destroy_client(client)

        self._recorder.stop()
        self._recorder = None
        if not saved:
            self._note('re-run Phase 3: there is no route file to follow')

    def _start_follower(self) -> None:
        """Phase 5/6: drive the recorded route."""
        if not bool(self.get_parameter('manage_follower').value):
            self._note('manage_follower is off -- launch route_follower '
                       'yourself (this is deliberate: it moves the robot)')
            return

        route = (str(self.get_parameter('route_path').value or '')
                 or self._last_route or '')
        if not route:
            self._note('no route to follow -- run Phase 3 first, or set the '
                       'route_path parameter')
            return
        if not os.path.exists(route):
            self._note(f'route file missing: {route}')
            return

        if self._follower is not None and self._follower.running:
            self._follower.stop()

        command = ['ros2', 'launch', 'outdoor_patrol_route',
                   'route_follow.launch.py', f'route_path:={route}',
                   'nominal_speed_ms:=%s' % self._desired['nominal_speed_ms']]
        params = self._route_params_file()
        if params:
            command.append(f'params_file:={params}')
        if bool(self.get_parameter('child_use_sim_time').value):
            command.append('use_sim_time:=true')

        self._follower = ManagedProcess(
            'route_follower', command,
            os.path.join(str(self.get_parameter('log_dir').value),
                         'field_follower.log'),
            self.get_logger())
        if self._follower.start():
            self._note(f'following {os.path.basename(route)}')
            # Push the settings chosen BEFORE the run started. The corridor
            # and the avoidance switch are exactly what you set while
            # deciding what this run is for, and route_follower does not
            # exist to receive them until now.
            #
            # It takes a while to appear: it waits for /fromLL and projects
            # the whole route before it spins. So retry rather than firing
            # once and silently giving up -- that hands the operator a run
            # configured with the values they just changed away from.
            self._apply_deadline = time.monotonic() + 45.0
            self._apply_pending_timer = self.create_timer(
                1.0, self._apply_desired_when_ready,
                callback_group=self._service_group)
        else:
            self._follower = None

    def _cancel_apply_timer(self) -> None:
        timer = getattr(self, '_apply_pending_timer', None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            self._apply_pending_timer = None

    def _apply_desired_when_ready(self) -> None:
        """Push the stored tuning as soon as the follower can receive it."""
        if self._follower is None or not self._follower.running:
            self._cancel_apply_timer()
            return
        client = self.create_client(
            SetParameters, '/route_follower/set_parameters',
            callback_group=self._service_group)
        ready = client.service_is_ready()
        self.destroy_client(client)
        if not ready:
            if time.monotonic() > getattr(self, '_apply_deadline', 0.0):
                self._cancel_apply_timer()
                self._note('follower never accepted parameters -- it is '
                           'running with its profile defaults, NOT the '
                           'settings in the tuning box')
            return
        self._cancel_apply_timer()
        self._set_follower_params(dict(self._desired), announce=False)
        self._note('applied tuning to the new run: '
                   + ', '.join(f'{k}={v}' for k, v in self._desired.items()))

    def _stop_follower(self) -> None:
        self._cancel_apply_timer()
        if self._follower is None:
            return
        self._follower.stop()
        self._follower = None
        self._note('follower stopped')

    def _stop_children(self) -> None:
        self._stop_recorder()
        self._stop_follower()

    def shutdown_children(self) -> None:
        """Teardown on exit. Kills without saving -- a Ctrl-C is not a Stop."""
        for child in (self._recorder, self._follower):
            if child is not None:
                child.stop()
        self._recorder = None
        self._follower = None

    def _check_for_duplicates(self) -> None:
        """Warn if a second dashboard is publishing the same state topic."""
        try:
            count = self.count_publishers(self._state_pub.topic_name)
        except Exception:                             # noqa: BLE001
            return
        if count > 1:
            self._note(
                f'WARNING: {count} dashboards are publishing '
                f'{self._state_pub.topic_name}. The panel will show '
                'alternating data from all of them and settings will appear '
                'not to take. Stop the others: pgrep -af field_dashboard')

    # -- live tuning -------------------------------------------------------

    def _set_follower_params(self, params: dict, announce: bool = True) -> None:
        """Record the wanted values, and push them if a follower is running.

        The tuning box has to work BEFORE a run as well as during one -- the
        corridor width and the avoidance switch are precisely what you decide
        while setting a run up, and `route_follower` does not exist yet to
        receive them. So the dashboard owns them: it stores whatever you set,
        applies it to a running follower immediately, and hands it to the next
        one it launches.
        """
        if not params:
            return

        for name, value in params.items():
            if name in self._desired:
                self._desired[name] = value

        client = self.create_client(
            SetParameters, '/route_follower/set_parameters',
            callback_group=self._service_group)
        if not client.wait_for_service(timeout_sec=3.0):
            self.destroy_client(client)
            if announce:
                self._note('saved for the next run: '
                           + ', '.join(f'{k}={v}' for k, v in params.items()))
            return

        request = SetParameters.Request()
        for name, value in params.items():
            parameter = Parameter()
            parameter.name = str(name)
            parameter.value = self._parameter_value(value)
            if parameter.value is None:
                self._note(f'cannot set {name}: unsupported type '
                           f'{type(value).__name__}')
                continue
            request.parameters.append(parameter)
        if not request.parameters:
            self.destroy_client(client)
            return

        future = client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        response = future.result() if future.done() else None
        self.destroy_client(client)

        if response is None:
            self._note('set_parameters timed out')
            return
        names = [p.name for p in request.parameters]
        for name, result in zip(names, response.results):
            if not result.successful:
                # Always report a rejection: the stored value is now out of
                # step with the follower, and that must not be silent.
                self._note(f'{name} REJECTED: {result.reason}')
            elif announce:
                self._note(f'{name} = {params[name]}')

    @staticmethod
    def _parameter_value(value):
        """Python value -> ParameterValue, or None if unsupported."""
        parameter_value = ParameterValue()
        if isinstance(value, bool):
            parameter_value.type = ParameterType.PARAMETER_BOOL
            parameter_value.bool_value = value
        elif isinstance(value, int):
            parameter_value.type = ParameterType.PARAMETER_INTEGER
            parameter_value.integer_value = value
        elif isinstance(value, float):
            parameter_value.type = ParameterType.PARAMETER_DOUBLE
            parameter_value.double_value = value
        elif isinstance(value, str):
            parameter_value.type = ParameterType.PARAMETER_STRING
            parameter_value.string_value = value
        else:
            return None
        return parameter_value

    # -- main loop ---------------------------------------------------------

    def _tick(self) -> None:
        now = self._now()
        dt = max(0.0, now - self._last_update)
        self._last_update = now

        self._signals.on_tf(self._tf_alive())
        snap = self._signals.snapshot(now)

        if self._active is not None:
            before = self._phases[self._active].verdict()
            self._phases[self._active].update(snap, dt)
            after = self._phases[self._active].verdict()
            if after != before and after != phase_defs.PENDING:
                self._note(f'phase {self._active} -> {after.upper()}')

        self._state_pub.publish(String(data=json.dumps({
            'active': self._active,
            'site': self.get_parameter('site').value,
            'managed': {
                'recorder': bool(self._recorder and self._recorder.running),
                'follower': bool(self._follower and self._follower.running),
                'route': (os.path.basename(self._last_route)
                          if self._last_route else None),
                'manage_recorder': bool(
                    self.get_parameter('manage_recorder').value),
                'manage_follower': bool(
                    self.get_parameter('manage_follower').value),
            },
            'signals': snap.as_dict(),
            'tuning': {
                # What the follower reports it is actually using.
                'avoidance': snap.avoidance,
                'corridor_half_width_m': snap.corridor_half_width_m,
                'nominal_speed_ms': snap.nominal_speed_ms,
                # What the operator has asked for. Equal to the above while a
                # run is going; the pending settings for the next one if not.
                'desired': dict(self._desired),
                'live': bool(snap.follower_ok),
            },
            'phases': [p.as_dict() for p in self._phases],
            'log': self._log[-12:],
        })))

    def _tf_alive(self) -> bool:
        try:
            return self._tf_buffer.can_transform(
                self.get_parameter('map_frame').value,
                self.get_parameter('base_frame').value,
                rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError):
            return False

    # -- report ------------------------------------------------------------

    def write_report(self) -> str:
        """Write the markdown the plan's "Bring home" section asks for."""
        directory = str(self.get_parameter('report_dir').value)
        os.makedirs(directory, exist_ok=True)
        stamp = self._started_wall.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(directory, f'field_validation_{stamp}.md')

        def cell(text) -> str:
            # Gate text contains |cross_track|, which would otherwise split
            # the row into extra columns.
            return str(text).replace('|', r'\|')

        rows = _REPORT_HEADER.format(
            site=self.get_parameter('site').value,
            started=self._started_wall.isoformat(timespec='seconds'),
            written=datetime.datetime.now().isoformat(timespec='seconds'),
            stack=os.environ.get('ROS_DISTRO', 'unknown'))
        for phase in self._phases:
            verdict = (phase.verdict() if phase.samples
                       else 'not run')
            rows += (f'| {phase.index} {phase.name} | **{verdict}** | '
                     f'{phase.elapsed:.0f} s | {cell(phase.gate)} |\n')

        details = '\n'
        for phase in self._phases:
            if not phase.samples and phase.manual_verdict is None:
                continue
            details += f'\n## Phase {phase.index} - {phase.name}\n\n'
            details += f'*Gate: {phase.gate}*\n\n'
            details += '| Check | Value | Result |\n|---|---|---|\n'
            for check in phase.checks():
                details += (f'| {cell(check.label)} | {cell(check.value)} | '
                            f'{check.status} |\n')
            if phase.manual_verdict:
                details += (f'\nOperator marked this phase '
                            f'**{phase.manual_verdict}**.\n')
            if phase.notes:
                details += '\n' + '\n'.join(f'* {n}' for n in phase.notes) + '\n'

        log = '\n## Log\n\n```\n' + '\n'.join(self._log) + '\n```\n'
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(rows + details + log)
        return os.path.abspath(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldDashboard()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave a follower publishing /cmd_vel after the dashboard has
        # gone: nothing else would stop it.
        try:
            node.shutdown_children()
        except Exception as exc:                      # noqa: BLE001
            node.get_logger().error(f'child teardown failed: {exc}')
        try:
            node.get_logger().info(
                f'writing report to {node.write_report()}')
        except Exception as exc:                      # noqa: BLE001
            node.get_logger().error(f'report failed: {exc}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
