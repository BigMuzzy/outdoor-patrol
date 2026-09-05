# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Gate evaluation for the alley field validation, Phase 0 through 7.

One class per phase of
:doc:`doc/eng/plans/field-validation-alley.md`. Each one accumulates from the
:class:`~outdoor_patrol_validation.signals.Snapshot` stream and reports a list
of :class:`Check` rows, which is exactly what the RViz panel draws.

Nothing here touches ROS. That is deliberate: the gates are the part worth
unit-testing, and in the field they are the part you cannot afford to have
wrong. ``test/test_phases.py`` drives every one of them with synthetic
snapshots.

Three rules the whole module follows:

* **Violations latch.** A run that once put the robot 0.6 m off the route did
  not pass because it recovered. Anything phrased "never ..." in the plan is a
  sticky failure, cleared only by an explicit reset.
* **Nothing passes by default.** Checks start ``PENDING`` and go to ``PASS``
  only on evidence. A sensor that never publishes leaves its gate pending, not
  green.
* **The thresholds are the shipped ones.** Defaults here match
  ``route_alley.yaml``/``confidence_gate.yaml``; the node re-declares them as
  parameters so a different site can override without editing code. The plan
  is blunt that relaxing them to make a phase pass defeats the point.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional

from outdoor_patrol_validation.signals import Snapshot

PASS = 'pass'
FAIL = 'fail'
PENDING = 'pending'
INFO = 'info'


@dataclass
class Check:
    """One row of the panel's gate table."""

    label: str
    value: str = '--'
    status: str = PENDING
    #: Shown under the row when it matters -- what to do about a failure.
    hint: str = ''

    def as_dict(self) -> dict:
        return {
            'label': self.label,
            'value': self.value,
            'status': self.status,
            'hint': self.hint,
        }


@dataclass
class Thresholds:
    """Every number a gate compares against, in one place.

    Defaults are the alley values. See the docstrings on each phase for where
    the number comes from.
    """

    # Phase 0
    bringup_hold_s: float = 10.0
    # Phase 1 -- confidence_gate.yaml max_horizontal_sigma_m, and the plan's
    # "held for 10 minutes".
    sigma_gate_m: float = 0.05
    soak_hold_s: float = 600.0
    # Phase 2
    heading_tolerance_deg: float = 10.0
    heading_probe_distance_m: float = 2.0
    # Phase 3 -- recorder sample_dist_m/sample_yaw_deg, loop_closure_tolerance_m
    teach_sample_dist_m: float = 1.0
    teach_sample_yaw_deg: float = 5.0
    teach_min_samples: int = 25
    teach_min_length_m: float = 20.0
    loop_closure_tolerance_m: float = 3.0
    #: What the route is SUPPOSED to be: 'open' for a there-and-back like the
    #: alley, 'loop' for a circuit like a driveway square, 'either' to accept
    #: whichever the recorder decides. The alley default is 'open' because
    #: the plan is explicit that a there-and-back must not be written as a
    #: loop -- but a circuit is a perfectly good route, and demanding
    #: 'open' of one is a spurious failure, not a finding.
    teach_expect_loop: str = 'open'
    # Phase 4 -- chassis.yaml gnss_link xyz "0.28 -0.42 0.18"
    lever_arm_x_m: float = 0.28
    lever_arm_y_m: float = -0.42
    lever_arm_tolerance_m: float = 0.15
    # Phase 5
    max_cross_track_m: float = 0.5
    min_wall_clearance_m: float = 1.0
    lane_keeping_d_cmd_m: float = 0.2
    # Phase 6
    obstacle_full_retreat_m: float = 1.0
    resume_within_m: float = 10.0
    max_stationary_s: float = 3.0
    min_obstacle_clearance_m: float = 0.3
    # Phase 7 -- route_alley.yaml route_follower sigma_slow_m / sigma_stop_m
    sigma_slow_m: float = 0.05
    sigma_stop_m: float = 0.15
    nominal_speed_ms: float = 0.4
    stopped_speed_ms: float = 0.05


def _fmt(value: Optional[float], unit: str = '', digits: int = 2) -> str:
    if value is None:
        return '--'
    return f'{value:.{digits}f}{unit}'


def angle_diff_deg(a: float, b: float) -> float:
    """Signed smallest difference ``a - b``, wrapped to (-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def _combine(checks: List[Check]) -> str:
    """A phase is only as good as its worst gate."""
    scored = [c for c in checks if c.status != INFO]
    if any(c.status == FAIL for c in scored):
        return FAIL
    if scored and all(c.status == PASS for c in scored):
        return PASS
    return PENDING


class Phase:
    """Base class: accumulate snapshots, report gate rows."""

    index = -1
    name = ''
    #: The gate line from the plan, shown as the panel's subtitle.
    gate = ''
    #: What the operator has to physically do. Shown before the phase starts.
    action = ''

    def __init__(self, thresholds: Optional[Thresholds] = None) -> None:
        self.th = thresholds or Thresholds()
        self.manual_verdict: Optional[str] = None
        self.notes: List[str] = []
        self.reset()

    # -- lifecycle ---------------------------------------------------------

    def reset(self) -> None:
        self.elapsed = 0.0
        self.samples = 0
        self.manual_verdict = None
        self.notes = []
        self._reset()

    def _reset(self) -> None:
        """Per-phase accumulator reset."""

    def update(self, snap: Snapshot, dt: float) -> None:
        self.elapsed += dt
        self.samples += 1
        self._update(snap, dt)

    def _update(self, snap: Snapshot, dt: float) -> None:
        """Per-phase accumulation."""

    # -- reporting ---------------------------------------------------------

    def checks(self) -> List[Check]:
        return []

    def verdict(self) -> str:
        if self.manual_verdict is not None:
            return self.manual_verdict
        return _combine(self.checks())

    def mark(self, verdict: str) -> None:
        """Operator override, for the gates no sensor can settle."""
        self.manual_verdict = verdict if verdict in (PASS, FAIL) else None

    def note(self, text: str) -> None:
        if text not in self.notes:
            self.notes.append(text)

    def as_dict(self) -> dict:
        return {
            'index': self.index,
            'name': self.name,
            'gate': self.gate,
            'action': self.action,
            'verdict': self.verdict(),
            'manual': self.manual_verdict is not None,
            'elapsed_s': round(self.elapsed, 1),
            'checks': [c.as_dict() for c in self.checks()],
            'notes': list(self.notes),
        }


class Phase0Bringup(Phase):
    """Stack liveness.

    The plan's Phase 0 gate is "container ``Up``, no restart loop", which is a
    docker fact rather than a ROS one. What matters downstream, and what this
    checks, is the consequence: every topic the later phases read is actually
    publishing, and the localization TF chain is up. Held for
    ``bringup_hold_s`` so a stack still in the middle of starting does not
    flash green on its way past.
    """

    index = 0
    name = 'Bring-up'
    gate = 'every required topic live and map->base_link resolving, held 10 s'
    action = 'Start the stack on the robot, then press Start.'

    def _reset(self) -> None:
        self.hold_s = 0.0
        self.best_hold_s = 0.0
        self.missing: List[str] = []
        self.tf_seen = False

    def _update(self, snap: Snapshot, dt: float) -> None:
        self.missing = [name for name, alive in sorted(snap.topics.items())
                        if not alive]
        if snap.tf_ok:
            self.tf_seen = True
        if not self.missing and snap.tf_ok:
            self.hold_s += dt
            self.best_hold_s = max(self.best_hold_s, self.hold_s)
        else:
            self.hold_s = 0.0

    def checks(self) -> List[Check]:
        held = self.hold_s >= self.th.bringup_hold_s
        return [
            Check('all required topics live',
                  'yes' if not self.missing
                  else f'missing {", ".join(self.missing)}',
                  PASS if not self.missing else FAIL,
                  hint='' if not self.missing
                       else 'docker compose ... logs --tail=40'),
            Check('map -> base_link TF',
                  'resolving' if self.tf_seen else 'absent',
                  PASS if self.tf_seen else PENDING,
                  hint='' if self.tf_seen
                       else 'the dual-EKF has not converged; needs a fix'),
            Check(f'held {self.th.bringup_hold_s:.0f} s',
                  _fmt(self.hold_s, ' s', 1),
                  PASS if held else PENDING),
        ]


class Phase1GnssSoak(Phase):
    """The go/no-go.

    ``sigma_gate_m`` is ``confidence_gate.yaml``'s ``max_horizontal_sigma_m``:
    above it the gate multiplies the covariance by 1000 and the EKF stops
    trusting GNSS. The hold requirement is the plan's 10 minutes.

    A violation *resets the hold clock* rather than latching a failure -- the
    gate is "10 continuous minutes", so an early wobble that settles is not
    disqualifying, it just means starting the clock again. Dropouts are
    counted and reported because a site that needs five attempts to find a
    clean 10 minutes has told you something.
    """

    index = 1
    name = 'GNSS soak'
    gate = 'sigma <= 5 cm held for 10 min, no dropouts'
    action = ('Park the robot in the middle of the alley, halfway along. '
              'Do not touch it.')

    def _reset(self) -> None:
        self.hold_s = 0.0
        self.best_hold_s = 0.0
        self.worst_sigma: Optional[float] = None
        self.dropouts = 0
        self._was_ok = True
        self.rtk_fixed_s = 0.0
        self.worst_corr_age: Optional[float] = None

    def _update(self, snap: Snapshot, dt: float) -> None:
        sigma = snap.sigma_gated
        ok = snap.gated_ok and sigma is not None and sigma <= self.th.sigma_gate_m
        if sigma is not None and snap.gated_ok:
            self.worst_sigma = (sigma if self.worst_sigma is None
                                else max(self.worst_sigma, sigma))
        if snap.gga_corr_age is not None:
            self.worst_corr_age = (snap.gga_corr_age if self.worst_corr_age is None
                                   else max(self.worst_corr_age, snap.gga_corr_age))
        if snap.gga_quality == 4:
            self.rtk_fixed_s += dt

        if ok:
            self.hold_s += dt
            self.best_hold_s = max(self.best_hold_s, self.hold_s)
        else:
            if self._was_ok and self.elapsed > dt:
                self.dropouts += 1
            self.hold_s = 0.0
        self._was_ok = ok

    def checks(self) -> List[Check]:
        sigma_ok = (self.worst_sigma is not None
                    and self.worst_sigma <= self.th.sigma_gate_m)
        held = self.hold_s >= self.th.soak_hold_s
        remaining = max(0.0, self.th.soak_hold_s - self.hold_s)
        # Minutes for a field soak, seconds for a shortened one -- a 20 s
        # rehearsal soak rendered in minutes reads "0.4 / 0 min".
        if self.th.soak_hold_s >= 120.0:
            label = f'held {self.th.soak_hold_s / 60:.0f} min continuously'
            value = (f'{self.hold_s / 60:.1f} / '
                     f'{self.th.soak_hold_s / 60:.0f} min')
            togo = f'  ({remaining / 60:.1f} min to go)'
        else:
            label = f'held {self.th.soak_hold_s:.0f} s continuously'
            value = f'{self.hold_s:.0f} / {self.th.soak_hold_s:.0f} s'
            togo = f'  ({remaining:.0f} s to go)'
        return [
            Check(f'worst sigma <= {self.th.sigma_gate_m * 100:.0f} cm',
                  _fmt(self.worst_sigma, ' m', 3),
                  PASS if sigma_ok else (PENDING if self.worst_sigma is None
                                         else FAIL),
                  hint='' if sigma_ok else
                       'not a threshold to relax -- the site may not suit '
                       'GNSS-only navigation'),
            Check(label, value + ('' if held else togo),
                  PASS if held else PENDING),
            Check('dropouts', str(self.dropouts), INFO,
                  hint='each one restarted the hold clock'),
            Check('RTK fixed', _fmt(self.rtk_fixed_s, ' s', 0), INFO),
            Check('worst correction age', _fmt(self.worst_corr_age, ' s', 1),
                  INFO,
                  hint='tens of seconds means the NTRIP stream stalled'),
        ]


class Phase2Heading(Phase):
    """The silent killer.

    ``yaw_offset`` in ``heading_to_imu.yaml`` is a reasoned guess with a
    "FINALIZE empirically" comment on it. If it is 180 deg out the robot
    believes it faces backwards and drives away from the route without
    complaining.

    The plan asks the operator to eyeball the heading against a phone compass
    and then drive 2 m to confirm. The second half is the objective one and is
    fully automated here: capture the origin at Start, and once the robot has
    moved ``heading_probe_distance_m``, compare the course actually travelled
    against the heading the receiver reported. Both are ENU, so they should
    agree.

    The 180 deg case gets called out by name, because that is the failure the
    phase exists to catch and the fix is a one-line sign flip.
    """

    index = 2
    name = 'Heading'
    gate = 'reported yaw within 10 deg of the course actually driven'
    action = ('Nose along the alley, wheels on the ground, hand on the kill '
              'switch. Press Start, then teleop forward 2 m.')

    def _reset(self) -> None:
        self.origin: Optional[tuple] = None
        self.distance = 0.0
        self.error_deg: Optional[float] = None
        self.course_deg: Optional[float] = None
        self.reported_deg: Optional[float] = None

    def _update(self, snap: Snapshot, dt: float) -> None:
        if self.error_deg is not None:
            # Latched at the first valid comparison. The chord from the origin
            # only represents the direction the robot points while it is still
            # going roughly straight; keep updating and the first bend turns a
            # correct heading into a large error. On a driveway square that
            # reads as a 90 deg failure -- spurious, and the kind an operator
            # would believe and act on.
            return
        if not snap.odom_ok or snap.odom_x is None:
            return
        if self.origin is None:
            self.origin = (snap.odom_x, snap.odom_y)
            return
        dx = snap.odom_x - self.origin[0]
        dy = snap.odom_y - self.origin[1]
        self.distance = math.hypot(dx, dy)
        if (self.distance >= self.th.heading_probe_distance_m
                and snap.heading_yaw_deg is not None and snap.heading_ok):
            self.course_deg = math.degrees(math.atan2(dy, dx))
            self.reported_deg = snap.heading_yaw_deg
            self.error_deg = angle_diff_deg(self.reported_deg, self.course_deg)

    def checks(self) -> List[Check]:
        moved = self.distance >= self.th.heading_probe_distance_m
        tol = self.th.heading_tolerance_deg
        within = self.error_deg is not None and abs(self.error_deg) <= tol
        hint = ''
        if self.error_deg is not None and not within:
            if abs(abs(self.error_deg) - 180.0) <= 20.0:
                hint = ('180 deg out: ANT1/ANT2 are the other way round. Flip '
                        'the sign of yaw_offset in heading_to_imu.yaml, '
                        'redeploy, repeat.')
            else:
                hint = ('yaw_offset is wrong by roughly this much -- correct '
                        'it in heading_to_imu.yaml and repeat.')
        return [
            Check(f'driven >= {self.th.heading_probe_distance_m:.0f} m forward',
                  _fmt(self.distance, ' m'),
                  PASS if moved else PENDING),
            Check('GNSS heading fresh',
                  _fmt(self.reported_deg, ' deg', 1),
                  PASS if self.reported_deg is not None else PENDING),
            Check('course actually driven',
                  _fmt(self.course_deg, ' deg', 1), INFO),
            Check(f'heading error <= {tol:.0f} deg',
                  _fmt(self.error_deg, ' deg', 1),
                  PASS if within else (FAIL if self.error_deg is not None
                                       else PENDING),
                  hint=hint),
        ]


class Phase3Teach(Phase):
    """Teach pass.

    ``route_recorder`` writes the route file but publishes no status, so the
    dashboard cannot read its sample count. Instead it applies the recorder's
    own sampling rule (``sample_dist_m`` / ``sample_yaw_deg``) to
    ``/odometry/global`` and reports what the file *should* contain. Compare
    it against the saved YAML: a disagreement means the recorder was reading a
    different source than you think, which is the silent failure the plan
    warns about when ``--params-file`` and ``-p`` are ordered wrongly.

    Loop closure is predicted the same way, from the start-to-end distance
    against ``loop_closure_tolerance_m``. An alley is there-and-back, so
    ``loop: false`` is the correct answer.
    """

    index = 3
    name = 'Teach pass'
    gate = 'enough samples, the shape you meant, worst fix: fixed'
    action = ('Start both recorders, press Start, then teleop the full alley '
              'down the middle at walking pace. Stop at the far end.')

    _FIX_ORDER = ('none', 'single', 'float', 'fixed')

    #: Position steps below this are treated as noise, not travel.
    #:
    #: Length is accumulated only across accepted stations, which already
    #: makes it immune to per-cycle jitter -- but a yaw-triggered station can
    #: still be accepted while the robot is essentially still, and this stops
    #: those contributing. Measured against the Gazebo sim: a robot parked
    #: nose-to-a-wall reported 3.9 m of "route" in 20 s of standing still,
    #: which would clear a 20 m length gate on a teach pass that never
    #: happened.
    _NOISE_FLOOR_M = 0.03

    def _reset(self) -> None:
        self.origin: Optional[tuple] = None
        self.last_sample: Optional[tuple] = None
        self.predicted_samples = 0
        self.length = 0.0
        self.end_distance = 0.0
        self.worst_fix: Optional[str] = None

    def _classify(self, snap: Snapshot) -> Optional[str]:
        """The class the RECORDER will write.

        ``snap.fix_class`` comes from the NavSatFix status and sigma, which is
        exactly what ``route_recorder`` uses -- so this predicts the file
        rather than a correlated quantity. It also works where there is no
        NMEA at all, such as the Gazebo sim, where a GGA-only version leaves
        this gate stuck on PENDING for ever.
        """
        if snap.fix_class is not None:
            return snap.fix_class
        if snap.gga_quality is None:
            return None
        return {4: 'fixed', 5: 'float', 2: 'single', 1: 'single'}.get(
            snap.gga_quality, 'none')

    def _update(self, snap: Snapshot, dt: float) -> None:
        fix = self._classify(snap)
        if fix is not None:
            if (self.worst_fix is None
                    or self._FIX_ORDER.index(fix)
                    < self._FIX_ORDER.index(self.worst_fix)):
                self.worst_fix = fix

        if not snap.odom_ok or snap.odom_x is None:
            return
        here = (snap.odom_x, snap.odom_y, snap.odom_yaw_deg or 0.0)
        if self.origin is None:
            self.origin = here
            self.last_sample = here
            self.predicted_samples = 1
            return

        self.end_distance = math.hypot(here[0] - self.origin[0],
                                       here[1] - self.origin[1])

        # Length accumulates ONLY across accepted stations, so it is the
        # polyline length the recorder will actually write -- and it cannot
        # be manufactured by a stationary robot's jitter, which integrating
        # every cycle did.
        moved = math.hypot(here[0] - self.last_sample[0],
                           here[1] - self.last_sample[1])
        turned = abs(angle_diff_deg(here[2], self.last_sample[2]))
        if (moved >= self.th.teach_sample_dist_m
                or turned >= self.th.teach_sample_yaw_deg):
            if moved >= self._NOISE_FLOOR_M:
                self.length += moved
            self.predicted_samples += 1
            self.last_sample = here

    def checks(self) -> List[Check]:
        enough = self.predicted_samples >= self.th.teach_min_samples
        long_enough = self.length >= self.th.teach_min_length_m
        closes = self.end_distance <= self.th.loop_closure_tolerance_m
        driven = self.length >= 1.0
        expect = str(self.th.teach_expect_loop).lower()
        fix_ok = self.worst_fix == 'fixed'

        if expect == 'loop':
            loop_status = (PASS if (driven and closes)
                           else (PENDING if not driven else FAIL))
            loop_label = 'route closes into a loop (predicted)'
            loop_value = f'ends {self.end_distance:.1f} m from start'
            loop_hint = ('' if closes else
                         'the circuit did not come back to its start, so the '
                         'recorder will write loop: false -- drive the last '
                         'leg back to where you began')
        elif expect == 'either':
            loop_status = PASS if driven else PENDING
            loop_label = 'route shape (predicted)'
            loop_value = ('closed loop' if closes else 'open') + \
                         f', ends {self.end_distance:.1f} m from start'
            loop_hint = ''
        else:
            loop_status = (PASS if (driven and not closes)
                           else (PENDING if not driven else FAIL))
            loop_label = 'route stays open (predicted)'
            loop_value = f'ends {self.end_distance:.1f} m from start'
            loop_hint = ('' if not closes else
                         'start and end are within the loop closure '
                         'tolerance, so the recorder will write loop: true')

        return [
            Check(f'samples >= {self.th.teach_min_samples} (predicted)',
                  str(self.predicted_samples),
                  PASS if enough else PENDING,
                  hint='compare against the saved YAML; a mismatch means the '
                       'recorder read a different source'),
            Check(f'route length >= {self.th.teach_min_length_m:.0f} m',
                  _fmt(self.length, ' m', 1),
                  PASS if long_enough else PENDING),
            Check(loop_label, loop_value, loop_status, hint=loop_hint),
            Check('worst fix: fixed', self.worst_fix or '--',
                  PASS if fix_ok else (PENDING if self.worst_fix is None
                                       else FAIL),
                  hint='' if fix_ok else
                       'the teach pass dropped off RTK fixed; re-record'),
        ]


class Phase4LeverArm(Phase):
    """Prove the base_link correction is live, outdoors.

    The plan scores this offline by recording two routes on one drive and
    comparing them, because there is no ground truth in an alley. The live
    equivalent, and what this computes, is the same quantity without the
    second recorder: convert the raw antenna fix into the map frame via
    ``/fromLL`` and express its offset from ``/odometry/global`` in body axes.
    That offset *is* the lever arm, and it should reproduce ``gnss_link`` from
    ``chassis.yaml`` -- (+0.28, -0.42) m.

    The sign carries the content. Right magnitude, wrong sign means the lever
    arm is being added where it should be subtracted, and the robot's idea of
    where it is sits 0.84 m off. A magnitude-only comparison misses that, so
    the sign is checked as its own gate.

    Run the plan's ``score_route.py --compare`` as well and mark the phase by
    hand if you want the recorded-route version on the record too.
    """

    index = 4
    name = 'Lever arm'
    gate = 'antenna sits +0.28 / -0.42 m from base_link, in body axes'
    action = ('Anywhere with a fix and a settled heading. Press Start and '
              'wait a few seconds.')

    def _reset(self) -> None:
        self.samples_used = 0
        self.mean_x: Optional[float] = None
        self.mean_y: Optional[float] = None

    def _update(self, snap: Snapshot, dt: float) -> None:
        if snap.lever_x is None or snap.lever_y is None:
            return
        self.samples_used += 1
        n = self.samples_used
        if self.mean_x is None:
            self.mean_x, self.mean_y = snap.lever_x, snap.lever_y
        else:
            self.mean_x += (snap.lever_x - self.mean_x) / n
            self.mean_y += (snap.lever_y - self.mean_y) / n

    def checks(self) -> List[Check]:
        tol = self.th.lever_arm_tolerance_m
        have = self.mean_y is not None and self.samples_used >= 10
        x_ok = have and abs(self.mean_x - self.th.lever_arm_x_m) <= tol
        y_ok = have and abs(self.mean_y - self.th.lever_arm_y_m) <= tol
        sign_ok = have and (self.mean_y < 0) == (self.th.lever_arm_y_m < 0)
        hint = ''
        if have and not sign_ok:
            hint = ('the lever arm is being ADDED where it should be '
                    'subtracted -- this is the failure the sign check exists '
                    'for. Stop; every later phase builds on it.')
        return [
            Check('samples', str(self.samples_used),
                  PASS if have else PENDING,
                  hint='' if have or self.samples_used
                       else 'needs /fromLL, a raw fix and a heading'),
            Check(f'lateral offset {self.th.lever_arm_y_m:+.2f} m +/- '
                  f'{tol:.2f}',
                  _fmt(self.mean_y, ' m', 3),
                  PASS if y_ok else (PENDING if not have else FAIL)),
            Check('offset sign (antenna on the right)',
                  'right' if (have and self.mean_y < 0)
                  else ('left' if have else '--'),
                  PASS if sign_ok else (PENDING if not have else FAIL),
                  hint=hint),
            Check(f'longitudinal offset {self.th.lever_arm_x_m:+.2f} m +/- '
                  f'{tol:.2f}',
                  _fmt(self.mean_x, ' m', 3),
                  PASS if x_ok else (PENDING if not have else FAIL)),
        ]


class _RunPhase(Phase):
    """Shared accumulation for the two autonomous runs (Phases 5 and 6)."""

    def _reset(self) -> None:
        self.max_cross_track = 0.0
        self.min_side_clearance: Optional[float] = None
        self.states: List[str] = []
        self.finished = False
        self.travelled = 0.0
        self.stationary_s = 0.0
        self.max_stationary_s = 0.0
        self.blocked_s = 0.0
        self._moved = False
        self.start_xy: Optional[tuple] = None
        self.closure_m: Optional[float] = None
        #: A healthy lidar was seen at least once. Distinguishes "nothing
        #: within range" from "no lidar", which the clearance gate must not
        #: confuse: an open road has no returns at all, and treating that as
        #: missing data leaves the gate PENDING for the whole run.
        self.scan_seen = False
        #: route_follower has published at least once since Start.
        #:
        #: Phases 5 and 6 MONITOR the follower; they do not launch it. If it
        #: is not running there is no publisher on ~/status, every gate sits
        #: on PENDING, and the panel looks like it is waiting for the robot
        #: when it is really waiting for a node that was never started. That
        #: is precisely the silent no-op this dashboard exists to prevent, so
        #: it gets its own gate rather than being inferred.
        self.follower_seen = False

    def _accumulate(self, snap: Snapshot, dt: float) -> None:
        # Closure error: how far from where it set off. On a circuit driven
        # back to its start that IS the localization drift over a lap, which
        # is the most direct measurement in the whole procedure -- no ground
        # truth needed, just the same patch of ground twice.
        if snap.odom_ok and snap.odom_x is not None:
            if self.start_xy is None:
                self.start_xy = (snap.odom_x, snap.odom_y)
            else:
                self.closure_m = math.hypot(snap.odom_x - self.start_xy[0],
                                            snap.odom_y - self.start_xy[1])

        if snap.follower_ok:
            self.follower_seen = True
            if snap.cross_track is not None:
                self.max_cross_track = max(self.max_cross_track,
                                           abs(snap.cross_track))
            if snap.state and (not self.states or self.states[-1] != snap.state):
                self.states.append(snap.state)
            if snap.state == 'finished':
                self.finished = True
            if snap.state == 'blocked':
                self.blocked_s += dt
            if snap.travelled is not None:
                self.travelled = snap.travelled

            # "Never stationary more than 3 s" is about the run, not about
            # the wait before it or the parked robot after it. Counting
            # either turns every completed run into a failure.
            speed = snap.follower_speed
            if speed is not None:
                if abs(speed) >= self.th.stopped_speed_ms:
                    self._moved = True
                    self.stationary_s = 0.0
                elif self._moved and not self.finished:
                    self.stationary_s += dt
                    self.max_stationary_s = max(self.max_stationary_s,
                                                self.stationary_s)

        if snap.scan_ok:
            self.scan_seen = True
        for side in (snap.scan_left_min, snap.scan_right_min):
            if side is not None:
                self.min_side_clearance = (
                    side if self.min_side_clearance is None
                    else min(self.min_side_clearance, side))

    def _follower_check(self) -> Check:
        """First row of both run phases: is anything actually driving?

        Answers the question an operator asks out loud when the panel does
        nothing -- and it is almost always this, because pressing Start here
        begins WATCHING a run, it does not begin one.
        """
        if self.follower_seen:
            return Check('route_follower publishing', 'yes', PASS)
        return Check(
            'route_follower publishing',
            'NO PUBLISHER on /route_follower/status',
            PENDING if self.elapsed < 5.0 else FAIL,
            hint='this phase WATCHES a run, it does not start one. Launch '
                 'the follower in another terminal:  ros2 launch '
                 'outdoor_patrol_route route_follow.launch.py '
                 'route_path:=<your route.yaml> params_file:=<profile.yaml>'
                 '  -- and record a route first if you have not (Phase 3).')


class Phase5AutonomousRun(_RunPhase):
    """First autonomous run, clear alley.

    The plan's abort list is the gate list: cross-track beyond 0.5 m, a wall
    closer than 1 m, or the robot heading somewhere it should not. Those latch,
    so an aborted run reads FAIL afterwards rather than reverting to green once
    you have carried the robot back.

    ``d_cmd`` is expected to stay at zero: there is no obstacle in this phase,
    so any commanded offset means the follower saw something in the corridor.
    """

    index = 5
    name = 'Autonomous run'
    gate = ('end to end, |cross_track| <= 0.5 m, never within 1 m of a wall')
    action = ('Clear route, no obstacle. Robot at the start, kill switch in '
              'hand. LAUNCH THE FOLLOWER in another terminal (ros2 launch '
              'outdoor_patrol_route route_follow.launch.py route_path:=...) '
              '-- this panel watches the run, it does not start it.')

    def _reset(self) -> None:
        super()._reset()
        self.max_d_cmd = 0.0

    def _update(self, snap: Snapshot, dt: float) -> None:
        self._accumulate(snap, dt)
        if snap.follower_ok and snap.d_cmd is not None:
            self.max_d_cmd = max(self.max_d_cmd, abs(snap.d_cmd))

    def checks(self) -> List[Check]:
        started = bool(self.states)
        ct_ok = self.max_cross_track <= self.th.max_cross_track_m
        wall_ok = (self.min_side_clearance is None
                   or self.min_side_clearance >= self.th.min_wall_clearance_m)
        if self.min_side_clearance is not None:
            wall_value = _fmt(self.min_side_clearance, ' m')
            wall_status = PASS if wall_ok else FAIL
        elif self.scan_seen:
            # Lidar healthy, nothing ever came within range: that is maximum
            # clearance, not missing data.
            wall_value = 'nothing within range'
            wall_status = PASS
        else:
            wall_value = '--'
            wall_status = PENDING
        lane_ok = self.max_d_cmd <= self.th.lane_keeping_d_cmd_m
        return [
            self._follower_check(),
            Check('reached the far end',
                  'finished' if self.finished
                  else (self.states[-1] if self.states else '--'),
                  PASS if self.finished else PENDING),
            Check(f'|cross_track| <= {self.th.max_cross_track_m} m',
                  f'worst {self.max_cross_track:.2f} m',
                  PASS if (started and ct_ok) else (FAIL if not ct_ok
                                                    else PENDING),
                  hint='' if ct_ok else
                       'the teach pass may not have been down the middle'),
            Check(f'wall clearance >= {self.th.min_wall_clearance_m:.1f} m',
                  wall_value, wall_status),
            Check(f'stayed in lane (|d_cmd| <= '
                  f'{self.th.lane_keeping_d_cmd_m} m)',
                  f'worst {self.max_d_cmd:.2f} m',
                  PASS if (started and lane_ok) else (FAIL if not lane_ok
                                                      else PENDING),
                  hint='' if lane_ok else
                       'the follower saw something in an alley you said was '
                       'clear'),
            Check('time blocked', _fmt(self.blocked_s, ' s', 1), INFO),
            Check('distance travelled', _fmt(self.travelled, ' m', 1), INFO),
            Check('distance from start (drift, on a circuit)',
                  _fmt(self.closure_m, ' m'), INFO,
                  hint='on a closed circuit driven back to its start this is '
                       'the localization drift over the lap'),
        ]


class Phase6Obstacle(_RunPhase):
    """The obstacle.

    ``d_cmd`` must never go positive: the obstacle is against the left wall,
    so the only way past is the right shoulder, and a positive offset is the
    robot steering into the blocked side. That gate latches on the first
    positive sample.

    "Back to |d_cmd| < 0.2 within 10 m of clearing" is measured from the
    moment the follower enters ``resuming`` to the moment the offset is
    actually back in the lane, using the follower's own ``travelled``.

    Stopping and staying stopped is the designed fallback when nothing is
    clear, not a crash -- but more than ``max_stationary_s`` of it fails the
    phase, which is what the plan asks for.
    """

    index = 6
    name = 'Obstacle'
    gate = ('d_cmd never positive, full retreat, back in lane within 10 m, '
            'never stalled > 3 s')
    action = ('2.4 m soft obstacle against the LEFT wall, ~18 m in. Robot '
              'back at the start. Launch the follower again as in Phase 5 -- '
              'this panel watches the run, it does not start it.')

    def _reset(self) -> None:
        super()._reset()
        self.went_positive = False
        self.worst_positive_d = 0.0
        self.max_retreat = 0.0
        self.resume_start_m: Optional[float] = None
        self.resume_distance: Optional[float] = None
        self.min_clearance: Optional[float] = None

    def _update(self, snap: Snapshot, dt: float) -> None:
        self._accumulate(snap, dt)
        if snap.scan_front_min is not None:
            self.min_clearance = (snap.scan_front_min
                                  if self.min_clearance is None
                                  else min(self.min_clearance,
                                           snap.scan_front_min))
        if not snap.follower_ok or snap.d_cmd is None:
            return
        d = snap.d_cmd
        if d > 1e-3:
            self.went_positive = True
            self.worst_positive_d = max(self.worst_positive_d, d)
        self.max_retreat = max(self.max_retreat, -d)

        if snap.state == 'resuming' and self.resume_start_m is None:
            self.resume_start_m = self.travelled
        if (self.resume_start_m is not None and self.resume_distance is None
                and abs(d) < self.th.lane_keeping_d_cmd_m
                and self.max_retreat >= self.th.obstacle_full_retreat_m):
            self.resume_distance = max(0.0, self.travelled - self.resume_start_m)

    def checks(self) -> List[Check]:
        retreated = self.max_retreat >= self.th.obstacle_full_retreat_m
        resumed_ok = (self.resume_distance is not None
                      and self.resume_distance <= self.th.resume_within_m)
        stall_ok = self.max_stationary_s <= self.th.max_stationary_s
        touch_ok = (self.min_clearance is None
                    or self.min_clearance >= self.th.min_obstacle_clearance_m)
        return [
            self._follower_check(),
            Check('d_cmd never positive',
                  'clean' if not self.went_positive
                  else f'reached {self.worst_positive_d:+.2f} m',
                  FAIL if self.went_positive else
                  (PASS if self.states else PENDING),
                  hint='' if not self.went_positive else
                       'it tried to pass on the blocked side'),
            Check(f'full retreat >= {self.th.obstacle_full_retreat_m:.1f} m',
                  f'{self.max_retreat:.2f} m',
                  PASS if retreated else PENDING),
            Check(f'back in lane within {self.th.resume_within_m:.0f} m',
                  _fmt(self.resume_distance, ' m', 1),
                  PASS if resumed_ok else
                  (FAIL if (self.resume_distance is not None) else PENDING)),
            Check(f'never stalled > {self.th.max_stationary_s:.0f} s',
                  f'worst {self.max_stationary_s:.1f} s',
                  FAIL if not stall_ok else (PASS if self.states else PENDING),
                  hint='' if stall_ok else
                       'read `blocked` in the status message -- in a narrow '
                       'alley both walls reading blocked is the usual cause'),
            Check(f'never closer than '
                  f'{self.th.min_obstacle_clearance_m:.1f} m ahead',
                  _fmt(self.min_clearance, ' m'),
                  PASS if (touch_ok and self.min_clearance is not None)
                  else (FAIL if not touch_ok else PENDING)),
            Check('state sequence',
                  ' > '.join(self.states) if self.states else '--', INFO),
        ]


class Phase7GnssFault(Phase):
    """The fault path.

    Degrade the fix deliberately and confirm the robot slows and then stops
    rather than carrying on at speed on a position it cannot trust. The
    thresholds are the follower's own ``sigma_slow_m`` / ``sigma_stop_m`` from
    ``route_alley.yaml``.

    The sticky failure is the one that matters: full speed while sigma is past
    the stop threshold means the speed ramp is not wired to the quality signal
    at all.
    """

    index = 7
    name = 'GNSS fault'
    gate = 'slows past sigma_slow, stopped past sigma_stop'
    action = ('Start a run, then block sky view halfway along -- a bucket '
              'over the antenna.')

    def _reset(self) -> None:
        self.max_sigma: Optional[float] = None
        self.degraded_seen = False
        self.fix_lost = False
        self.slowed = False
        self.stopped = False
        self.drove_degraded = False
        self.worst_degraded_speed = 0.0
        self.min_speed_after_degrade: Optional[float] = None

    def _update(self, snap: Snapshot, dt: float) -> None:
        sigma = snap.sigma_h if snap.sigma_h is not None else snap.sigma_raw
        speed = snap.follower_speed

        # A bucket over the antenna usually does not raise sigma -- it takes
        # the fix away entirely. The follower stops on either (its
        # _fix_penalty returns 0 for a stale fix as well as for a large
        # sigma), so the gate has to accept either, or blocking the sky
        # properly leaves this phase pending for ever.
        fix_lost_now = not snap.raw_ok
        if fix_lost_now:
            self.fix_lost = True
            self.degraded_seen = True
        if sigma is not None:
            self.max_sigma = (sigma if self.max_sigma is None
                              else max(self.max_sigma, sigma))
            if sigma >= self.th.sigma_slow_m:
                self.degraded_seen = True

        if speed is None:
            return
        if self.degraded_seen:
            self.min_speed_after_degrade = (
                speed if self.min_speed_after_degrade is None
                else min(self.min_speed_after_degrade, speed))
            if speed < 0.9 * self.th.nominal_speed_ms:
                self.slowed = True

        # Deliberately the CURRENT fix state, not the latched `fix_lost`.
        # Once the sky is unblocked the robot is entitled to drive at full
        # speed again; judging that against a latched flag would fail the
        # phase for recovering, which is the behaviour being tested for.
        refusing = fix_lost_now or (sigma is not None
                                    and sigma >= self.th.sigma_stop_m)
        if refusing:
            if speed <= self.th.stopped_speed_ms:
                self.stopped = True
            if speed >= 0.9 * self.th.nominal_speed_ms:
                self.drove_degraded = True
                self.worst_degraded_speed = max(self.worst_degraded_speed,
                                                speed)

    def checks(self) -> List[Check]:
        return [
            Check('fix degraded past sigma_slow',
                  'fix lost' if self.fix_lost else _fmt(self.max_sigma, ' m', 3),
                  PASS if self.degraded_seen else PENDING,
                  hint='' if self.degraded_seen
                       else 'the stimulus has not taken effect yet'),
            Check('slowed down', 'yes' if self.slowed else 'no',
                  PASS if self.slowed else PENDING),
            Check('came to a stop',
                  _fmt(self.min_speed_after_degrade, ' m/s'),
                  PASS if self.stopped else PENDING,
                  hint='' if self.stopped else
                       'needs the fix past sigma_stop, or lost outright -- '
                       'a partial degrade only proves the slow ramp'),
            Check('never drove fast on a bad fix',
                  'clean' if not self.drove_degraded
                  else f'{self.worst_degraded_speed:.2f} m/s past sigma_stop',
                  FAIL if self.drove_degraded
                  else (PASS if self.stopped else PENDING),
                  hint='' if not self.drove_degraded else
                       'the speed ramp is not reading the fix quality'),
        ]


PHASE_TYPES = (
    Phase0Bringup,
    Phase1GnssSoak,
    Phase2Heading,
    Phase3Teach,
    Phase4LeverArm,
    Phase5AutonomousRun,
    Phase6Obstacle,
    Phase7GnssFault,
)


def build_phases(thresholds: Optional[Thresholds] = None) -> List[Phase]:
    """One instance of every phase, in running order."""
    th = thresholds or Thresholds()
    return [cls(th) for cls in PHASE_TYPES]
