# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Gate logic tests.

These are the tests that matter for this package. The panel is a renderer and
the node is plumbing, but the gates decide whether a field trip counted, and
they are hard to exercise for real -- Phase 1 alone takes ten minutes of
standing in an alley. So they get driven here with synthetic snapshots
instead, including the failure modes the plan calls out by name: the 180 deg
heading flip, the lever arm applied backwards, and a run that recovers from a
violation and must still read FAIL.
"""

import math

from outdoor_patrol_validation import phases as ph
from outdoor_patrol_validation.signals import (
    Signals, Snapshot, classify_fix, parse_gga, sector_min,
    sigma_from_covariance, yaw_deg_from_quaternion)

import pytest


def snap(**kwargs) -> Snapshot:
    """A snapshot with a healthy stack, overridden per test."""
    defaults = dict(
        t=0.0,
        sigma_raw=0.02, raw_ok=True, raw_age=0.1,
        sigma_gated=0.02, gated_ok=True, gated_age=0.1,
        gga_quality=4, gga_quality_name='RTK fixed', gga_sats=30,
        gga_hdop=0.5, gga_corr_age=1.4, gga_station='4053', gga_ok=True,
        fix_class='fixed',
        heading_yaw_deg=0.0, heading_ok=True,
        odom_x=0.0, odom_y=0.0, odom_yaw_deg=0.0, odom_speed=0.0,
        odom_ok=True, tf_ok=True,
        follower_ok=False, scan_ok=True,
        topics={name: True for name in Signals.REQUIRED},
    )
    defaults.update(kwargs)
    return Snapshot(**defaults)


def drive(phase, snapshots, dt=0.2):
    for item in snapshots:
        phase.update(item, dt)
    return phase.verdict()


def status_of(phase, label):
    for check in phase.checks():
        if check.label.startswith(label):
            return check.status
    raise AssertionError(f'no check starting {label!r} in '
                         f'{[c.label for c in phase.checks()]}')


# -- signals -----------------------------------------------------------------

def test_sigma_is_the_square_root_of_the_east_variance():
    # confidence_gate compares 1-sigma, but NavSatFix carries variance.
    assert sigma_from_covariance([0.0025, 0, 0] + [0] * 6) == pytest.approx(0.05)
    assert sigma_from_covariance([]) is None
    assert sigma_from_covariance([-1.0] + [0] * 8) is None


def test_parse_gga_reads_the_fields_the_plan_points_at():
    line = ('$GNGGA,025453.40,4732.58913274,N,12152.82201833,W,5,30,0.5,'
            '250.6193,M,-21.0746,M,1.4,4053*6A')
    parsed = parse_gga(line)
    assert parsed['quality'] == 5
    assert parsed['quality_name'] == 'RTK float'
    assert parsed['sats'] == 30
    assert parsed['hdop'] == pytest.approx(0.5)
    assert parsed['corr_age'] == pytest.approx(1.4)
    assert parsed['station'] == '4053'


def test_parse_gga_ignores_other_sentences():
    assert parse_gga('$KSXT,20260903025453.40,...') is None
    assert parse_gga('') is None


def test_stale_signals_read_as_absent_not_as_their_last_value():
    signals = Signals()
    signals.on_fix_gated([0.0004] + [0] * 8, now=0.0)
    assert signals.snapshot(0.5).sigma_gated == pytest.approx(0.02)
    # Two seconds is the timeout; a publisher that died must not keep
    # reporting a healthy fix.
    late = signals.snapshot(10.0)
    assert late.sigma_gated is None
    assert late.gated_ok is False


def test_yaw_is_enu_degrees():
    assert yaw_deg_from_quaternion(0, 0, 0, 1) == pytest.approx(0.0)
    north = yaw_deg_from_quaternion(0, 0, math.sin(math.pi / 4),
                                    math.cos(math.pi / 4))
    assert north == pytest.approx(90.0)


def test_sector_min_picks_the_sector_and_drops_invalid_returns():
    # 360 beams, 1 deg apart, starting behind the robot.
    ranges = [10.0] * 360
    ranges[180] = 2.0        # straight ahead (angle 0)
    ranges[270] = 1.0        # 90 deg, the left sector
    ranges[181] = float('inf')
    start = -math.pi
    step = math.radians(1.0)
    assert sector_min(ranges, start, step, 0.0, 10.0) == pytest.approx(2.0)
    assert sector_min(ranges, start, step, 90.0, 10.0) == pytest.approx(1.0)
    assert sector_min([], start, step, 0.0, 10.0) is None


def test_scan_sectors_honour_the_yaw_180_lidar_mount():
    """Robot-forward is at 180 deg in the raw scan, not 0.

    The C1's 0 deg points at the robot body (chassis.yaml mounts lidar_link
    yaw-180, and scan_safety carries the same forward_offset_deg: 180). If
    this offset is dropped, front and rear swap silently and Phase 6's
    "never closer than 0.3 m ahead" ends up watching behind the robot.
    """
    ranges = [10.0] * 360
    start = -math.pi
    step = math.radians(1.0)
    # Raw index 0 is scan angle -180, which IS robot-forward.
    ranges[0] = 1.5
    # Raw index 180 is scan angle 0, which points backwards.
    ranges[180] = 0.4

    signals = Signals(forward_offset_deg=180.0)
    signals.on_scan(ranges, start, step, now=0.0)
    snap_ = signals.snapshot(0.0)
    assert snap_.scan_front_min == pytest.approx(1.5)

    # The unconfigured convention would have read the obstacle behind us.
    naive = Signals(forward_offset_deg=0.0)
    naive.on_scan(ranges, start, step, now=0.0)
    assert naive.snapshot(0.0).scan_front_min == pytest.approx(0.4)


def test_scan_left_and_right_are_body_left_and_right():
    ranges = [10.0] * 360
    start = -math.pi
    step = math.radians(1.0)
    # Body-left is +90 deg in body axes -> 270 deg in raw scan -> index 90.
    ranges[90] = 0.8
    # Body-right -> raw 90 deg -> index 270.
    ranges[270] = 1.7
    signals = Signals(forward_offset_deg=180.0)
    signals.on_scan(ranges, start, step, now=0.0)
    snap_ = signals.snapshot(0.0)
    assert snap_.scan_left_min == pytest.approx(0.8)
    assert snap_.scan_right_min == pytest.approx(1.7)


# -- Phase 0 -----------------------------------------------------------------

def test_phase0_needs_every_topic_and_a_hold():
    phase = ph.Phase0Bringup()
    assert drive(phase, [snap()] * 10) == ph.PENDING       # only 2 s so far
    assert drive(phase, [snap()] * 50) == ph.PASS          # past 10 s


def test_phase0_fails_while_a_topic_is_missing():
    phase = ph.Phase0Bringup()
    missing = dict.fromkeys(Signals.REQUIRED, True)
    missing['scan'] = False
    assert drive(phase, [snap(topics=missing)] * 60) == ph.FAIL
    assert status_of(phase, 'all required topics') == ph.FAIL


def test_phase0_hold_restarts_after_a_gap():
    phase = ph.Phase0Bringup()
    drive(phase, [snap()] * 40)                            # 8 s
    drive(phase, [snap(tf_ok=False)] * 2)                  # TF drops
    assert phase.hold_s == 0.0
    assert phase.verdict() == ph.PENDING


# -- Phase 1 -----------------------------------------------------------------

def test_phase1_passes_only_after_the_full_hold():
    th = ph.Thresholds(soak_hold_s=10.0)
    phase = ph.Phase1GnssSoak(th)
    assert drive(phase, [snap(sigma_gated=0.021)] * 25) == ph.PENDING
    assert drive(phase, [snap(sigma_gated=0.021)] * 30) == ph.PASS


def test_phase1_dropout_restarts_the_clock_and_is_counted():
    th = ph.Thresholds(soak_hold_s=10.0)
    phase = ph.Phase1GnssSoak(th)
    drive(phase, [snap(sigma_gated=0.02)] * 40)            # 8 s of hold
    drive(phase, [snap(sigma_gated=0.09)] * 2)             # over the gate
    assert phase.dropouts == 1
    assert phase.hold_s == 0.0
    assert phase.verdict() == ph.FAIL                      # worst sigma latched
    # The site's worst sigma is what disqualifies it, and 9 cm stays on the
    # record even once the fix recovers.
    drive(phase, [snap(sigma_gated=0.02)] * 60)
    assert phase.worst_sigma == pytest.approx(0.09)
    assert phase.verdict() == ph.FAIL


def test_phase1_treats_a_dead_publisher_as_a_dropout_not_a_pass():
    th = ph.Thresholds(soak_hold_s=2.0)
    phase = ph.Phase1GnssSoak(th)
    assert drive(phase, [snap(gated_ok=False, sigma_gated=None)] * 40) \
        == ph.PENDING


# -- Phase 2 -----------------------------------------------------------------

def _heading_run(reported_deg, travel_deg, distance=3.0, steps=30):
    """Drive `distance` along `travel_deg` while the receiver claims another.

    The gap between the two is what Phase 2 is supposed to measure.
    """
    out = []
    for i in range(steps + 1):
        r = distance * i / steps
        out.append(snap(
            odom_x=r * math.cos(math.radians(travel_deg)),
            odom_y=r * math.sin(math.radians(travel_deg)),
            heading_yaw_deg=reported_deg))
    return out


def test_phase2_passes_when_heading_matches_the_course_driven():
    phase = ph.Phase2Heading()
    assert drive(phase, _heading_run(90.0, 90.0)) == ph.PASS
    assert abs(phase.error_deg) < 1.0


def test_phase2_catches_the_180_degree_flip_and_says_so():
    phase = ph.Phase2Heading()
    assert drive(phase, _heading_run(-90.0, 90.0)) == ph.FAIL
    hint = [c.hint for c in phase.checks() if c.status == ph.FAIL][0]
    assert 'yaw_offset' in hint and '180' in hint


def test_phase2_stays_pending_until_the_robot_has_actually_moved():
    phase = ph.Phase2Heading()
    assert drive(phase, _heading_run(90.0, 90.0, distance=0.5)) == ph.PENDING


def test_phase2_latches_at_the_first_valid_comparison():
    """A bend after the probe distance must not overwrite a good result.

    Found on the driveway square: the chord from the origin only tracks the
    robot's heading while it is going straight, so continuing to update turns
    a correct heading into a 90 deg error at the first corner.
    """
    phase = ph.Phase2Heading()
    run = _heading_run(90.0, 90.0, distance=2.5)
    # Then the route turns a corner and carries on due east.
    last = run[-1]
    for i in range(1, 30):
        run.append(snap(odom_x=last.odom_x + 0.2 * i, odom_y=last.odom_y,
                        heading_yaw_deg=0.0))
    assert drive(phase, run) == ph.PASS
    assert abs(phase.error_deg) < 1.0


def test_phase2_tolerates_a_small_offset():
    phase = ph.Phase2Heading()
    assert drive(phase, _heading_run(95.0, 90.0)) == ph.PASS
    assert drive(ph.Phase2Heading(), _heading_run(115.0, 90.0)) == ph.FAIL


# -- Phase 3 -----------------------------------------------------------------

def _teach_run(length=30.0, steps=300, quality=4):
    return [snap(odom_x=length * i / steps, odom_y=0.0, gga_quality=quality)
            for i in range(steps + 1)]


def test_phase3_predicts_the_recorder_sample_count():
    phase = ph.Phase3Teach()
    assert drive(phase, _teach_run()) == ph.PASS
    # 30 m at sample_dist_m 1.0 -> about 30 samples, as the plan expects.
    assert 28 <= phase.predicted_samples <= 32
    assert phase.worst_fix == 'fixed'


def test_phase3_fails_a_teach_pass_that_dropped_off_rtk_fixed():
    phase = ph.Phase3Teach()
    run = _teach_run()
    # A real RTK-float sample degrades both: the GGA digit AND the sigma the
    # driver seeds from it, which is what the recorder actually classifies on.
    run[150] = snap(odom_x=15.0, odom_y=0.0, gga_quality=5,
                    sigma_raw=0.2, fix_class='float')
    assert drive(phase, run) == ph.FAIL
    assert phase.worst_fix == 'float'


def test_phase3_does_not_integrate_noise_into_route_length():
    """A parked robot must not accumulate a route.

    Found against the Gazebo sim: the robot was nose-to-a-wall and not moving,
    yet Phase 3 reported 3.9 m of route length in 20 s. The length is
    integrated from |delta position| each cycle, and RTK jitter on a
    stationary robot is a couple of centimetres per sample -- which at 5 Hz
    would clear a 20 m gate in under two minutes of standing still.
    """
    import random
    rng = random.Random(7)
    parked = [snap(odom_x=rng.gauss(0.0, 0.02), odom_y=rng.gauss(0.0, 0.02))
              for _ in range(600)]          # 2 minutes at 5 Hz
    phase = ph.Phase3Teach()
    drive(phase, parked)
    assert phase.length < 1.0, phase.length
    assert phase.predicted_samples <= 2


def test_phase3_still_measures_real_travel():
    phase = ph.Phase3Teach()
    drive(phase, _teach_run())              # a genuine 30 m straight
    assert phase.length == pytest.approx(30.0, abs=1.0)


def test_phase3_flags_a_route_that_closes_into_a_loop():
    phase = ph.Phase3Teach()
    circle = []
    for i in range(361):
        a = math.radians(i)
        circle.append(snap(odom_x=6.0 * math.cos(a), odom_y=6.0 * math.sin(a),
                           odom_yaw_deg=i))
    assert drive(phase, circle) == ph.FAIL
    assert status_of(phase, 'route stays open') == ph.FAIL


def _square_run(half=1.75, radius=1.0, step=0.1):
    """Drive a rounded square circuit, clockwise, back to the start."""
    import math as m
    pts, side = [], half - radius
    centres = [(side, side, 0.5), (side, -side, 0.0),
               (-side, -side, -0.5), (-side, side, -1.0)]
    for cx, cy, q in centres:
        a0 = q * m.pi
        for k in range(int(radius * m.pi / 2 / step) + 1):
            a = a0 - k * step / radius
            pts.append((cx + radius * m.cos(a), cy + radius * m.sin(a)))
    pts.append(pts[0])
    out = []
    for i, (x, y) in enumerate(pts):
        nx, ny = pts[min(i + 1, len(pts) - 1)]
        px, py = pts[max(i - 1, 0)]
        out.append(snap(odom_x=x, odom_y=y,
                        odom_yaw_deg=m.degrees(m.atan2(ny - py, nx - px))))
    return out


def test_phase3_accepts_a_closed_circuit_when_it_expects_one():
    # A driveway square is a legitimate route. The alley default would fail
    # it twice -- too short, and it closes -- so the profile has to be able
    # to say what shape it is expecting.
    th = ph.Thresholds(teach_expect_loop='loop', teach_min_length_m=10.0,
                       teach_min_samples=40, loop_closure_tolerance_m=1.0)
    phase = ph.Phase3Teach(th)
    assert drive(phase, _square_run()) == ph.PASS


def test_phase3_rejects_a_circuit_that_did_not_close():
    th = ph.Thresholds(teach_expect_loop='loop', teach_min_length_m=10.0,
                       teach_min_samples=40, loop_closure_tolerance_m=1.0)
    phase = ph.Phase3Teach(th)
    # Stop three quarters of the way round.
    run = _square_run()
    assert drive(phase, run[:int(len(run) * 0.75)]) == ph.FAIL
    assert status_of(phase, 'route closes into a loop') == ph.FAIL


def test_phase3_still_rejects_a_closed_route_when_it_expects_an_open_one():
    # The alley behaviour must not regress: a there-and-back written as a
    # loop is the failure the original gate existed to catch.
    th = ph.Thresholds(teach_expect_loop='open', teach_min_length_m=10.0,
                       teach_min_samples=40)
    phase = ph.Phase3Teach(th)
    assert drive(phase, _square_run()) == ph.FAIL
    assert status_of(phase, 'route stays open') == ph.FAIL


def test_phase3_either_accepts_both_shapes():
    th = ph.Thresholds(teach_expect_loop='either', teach_min_length_m=10.0,
                       teach_min_samples=20)
    assert drive(ph.Phase3Teach(th), _square_run()) == ph.PASS
    assert drive(ph.Phase3Teach(th), _teach_run()) == ph.PASS


def test_phase5_reports_closure_error_for_a_circuit():
    # The distance from start after a lap IS the drift, and it is the most
    # valuable number a driveway square produces.
    phase = ph.Phase5AutonomousRun()
    out = [_run_status(travelled=0.1 * i) for i in range(30)]
    for i, s in enumerate(out):
        s.odom_x, s.odom_y = 0.1 * i, 0.0
    back = _run_status(state='finished', travelled=3.0, follower_speed=0.0)
    back.odom_x, back.odom_y = 0.12, 0.0
    drive(phase, out + [back])
    assert phase.closure_m == pytest.approx(0.12, abs=0.01)


def test_classify_fix_mirrors_the_recorders_thresholds():
    # route_file.classify_fix: <=0.05 fixed, <=0.5 float, else single.
    assert classify_fix(0, 0.02) == 'fixed'
    assert classify_fix(0, 0.05) == 'fixed'
    assert classify_fix(0, 0.20) == 'float'
    assert classify_fix(0, 0.90) == 'single'
    assert classify_fix(-1, 0.01) == 'none'
    assert classify_fix(0, None) == 'single'


def test_fix_class_comes_from_the_fix_not_the_nmea():
    """The sim publishes no NMEA, so a GGA-only gate never completes there.

    route_recorder classifies from NavSatFix status + sigma, so the gate that
    predicts what it writes has to do the same. This also happens to be more
    accurate on the real robot: it is the same computation rather than a
    correlated one.
    """
    signals = Signals()
    signals.on_fix(1.0, 2.0, [0.0004] + [0] * 8, now=0.0, status=0)
    assert signals.snapshot(0.0).fix_class == 'fixed'
    signals.on_fix(1.0, 2.0, [0.04] + [0] * 8, now=1.0, status=0)
    assert signals.snapshot(1.0).fix_class == 'float'


def test_phase3_passes_without_any_nmea_as_in_the_gazebo_sim():
    phase = ph.Phase3Teach()
    run = []
    for i in range(301):
        # No GGA at all -- exactly what outdoor_patrol_sim publishes.
        run.append(snap(odom_x=30.0 * i / 300, odom_y=0.0,
                        gga_quality=None, gga_ok=False, fix_class='fixed'))
    assert drive(phase, run) == ph.PASS
    assert phase.worst_fix == 'fixed'


# -- Phase 4 -----------------------------------------------------------------

def test_phase4_passes_the_shipped_lever_arm():
    phase = ph.Phase4LeverArm()
    assert drive(phase, [snap(lever_x=0.28, lever_y=-0.42)] * 20) == ph.PASS


def test_phase4_catches_the_lever_arm_applied_backwards():
    # Right magnitude, wrong sign -- the failure a magnitude-only comparison
    # would wave through.
    phase = ph.Phase4LeverArm()
    assert drive(phase, [snap(lever_x=0.28, lever_y=+0.42)] * 20) == ph.FAIL
    assert status_of(phase, 'offset sign') == ph.FAIL


def test_phase4_needs_samples_before_it_judges():
    phase = ph.Phase4LeverArm()
    assert drive(phase, [snap(lever_x=0.28, lever_y=-0.42)] * 3) == ph.PENDING


# -- Phase 5 -----------------------------------------------------------------

def _run_status(**kwargs):
    base = dict(follower_ok=True, state='driving', cross_track=0.05,
                d_cmd=0.0, follower_speed=0.4, travelled=0.0,
                scan_left_min=1.9, scan_right_min=1.9, scan_front_min=8.0)
    base.update(kwargs)
    return snap(**base)


def test_phase5_says_so_when_the_follower_is_not_running():
    """The commonest "nothing is happening" cause, and it was invisible.

    Phases 5 and 6 WATCH route_follower; they do not launch it. With no
    follower there is no publisher on ~/status, so every gate sat on PENDING
    and the panel looked like it was waiting for the robot rather than for a
    node that was never started.
    """
    phase = ph.Phase5AutonomousRun()
    idle = [snap(follower_ok=False)] * 100      # 20 s at dt 0.2
    assert drive(phase, idle) == ph.FAIL
    assert status_of(phase, 'route_follower publishing') == ph.FAIL
    hint = [c.hint for c in phase.checks()
            if c.label.startswith('route_follower')][0]
    assert 'route_follow.launch.py' in hint


def test_phase5_follower_check_is_patient_for_a_few_seconds():
    # Launching the follower takes a moment; do not cry FAIL instantly.
    phase = ph.Phase5AutonomousRun()
    drive(phase, [snap(follower_ok=False)] * 10)     # 2 s
    assert status_of(phase, 'route_follower publishing') == ph.PENDING


def test_phase6_also_reports_a_missing_follower():
    phase = ph.Phase6Obstacle()
    assert drive(phase, [snap(follower_ok=False)] * 100) == ph.FAIL
    assert status_of(phase, 'route_follower publishing') == ph.FAIL


def test_phase5_passes_a_clean_run():
    phase = ph.Phase5AutonomousRun()
    run = [_run_status(travelled=0.1 * i) for i in range(300)]
    run.append(_run_status(state='finished', travelled=30.0,
                           follower_speed=0.0))
    assert drive(phase, run) == ph.PASS


def test_phase5_cross_track_violation_latches_through_recovery():
    phase = ph.Phase5AutonomousRun()
    drive(phase, [_run_status(cross_track=0.62)] * 5)
    assert status_of(phase, '|cross_track|') == ph.FAIL
    # Recovering to the centreline does not un-fail the run.
    drive(phase, [_run_status(cross_track=0.01)] * 200)
    assert status_of(phase, '|cross_track|') == ph.FAIL
    assert phase.verdict() == ph.FAIL


def test_phase5_passes_on_open_ground_with_a_healthy_lidar():
    """No returns is maximum clearance, not missing data.

    Found against the Gazebo patrol_road world: the lidar publishes happily
    but nothing is within range, so every sector minimum is None. Treating
    that as "no data" left the wall-clearance gate PENDING for the whole run,
    so Phase 5 could never complete on open ground.
    """
    phase = ph.Phase5AutonomousRun()
    run = [_run_status(travelled=0.1 * i, scan_left_min=None,
                       scan_right_min=None, scan_front_min=None)
           for i in range(300)]
    run.append(_run_status(state='finished', travelled=30.0,
                           follower_speed=0.0, scan_left_min=None,
                           scan_right_min=None, scan_front_min=None))
    assert drive(phase, run) == ph.PASS
    assert status_of(phase, 'wall clearance') == ph.PASS


def test_phase5_wall_gate_stays_pending_without_any_lidar():
    phase = ph.Phase5AutonomousRun()
    run = [_run_status(travelled=0.1 * i, scan_ok=False, scan_left_min=None,
                       scan_right_min=None, scan_front_min=None)
           for i in range(50)]
    drive(phase, run)
    assert status_of(phase, 'wall clearance') == ph.PENDING


def test_phase5_fails_when_a_wall_comes_inside_a_metre():
    phase = ph.Phase5AutonomousRun()
    drive(phase, [_run_status(scan_right_min=0.7)] * 10)
    assert status_of(phase, 'wall clearance') == ph.FAIL


# -- Phase 6 -----------------------------------------------------------------

def _obstacle_run():
    """Drive the sequence: retreat to -1.2, resume, back in lane."""
    out = []
    travelled = 0.0
    for _ in range(60):                       # approach
        travelled += 0.1
        out.append(_run_status(travelled=travelled))
    for i in range(20):                       # ramp out to -1.2
        travelled += 0.1
        out.append(_run_status(state='retreating', travelled=travelled,
                               d_cmd=-1.2 * (i + 1) / 20,
                               scan_front_min=3.0))
    for _ in range(40):                       # past the obstacle
        travelled += 0.1
        out.append(_run_status(state='retreating', travelled=travelled,
                               d_cmd=-1.2, scan_front_min=1.2))
    for i in range(20):                       # ramp home
        travelled += 0.1
        out.append(_run_status(state='resuming', travelled=travelled,
                               d_cmd=-1.2 * (19 - i) / 20))
    for _ in range(60):
        travelled += 0.1
        out.append(_run_status(travelled=travelled))
    return out


def test_phase6_passes_a_textbook_retreat():
    phase = ph.Phase6Obstacle()
    assert drive(phase, _obstacle_run()) == ph.PASS
    assert phase.max_retreat == pytest.approx(1.2)
    assert phase.states == ['driving', 'retreating', 'resuming', 'driving']


def test_phase6_fails_the_instant_d_cmd_goes_positive():
    # Passing on the left is passing on the blocked side.
    phase = ph.Phase6Obstacle()
    run = _obstacle_run()
    run[70] = _run_status(state='retreating', d_cmd=+0.6, travelled=7.0)
    assert drive(phase, run) == ph.FAIL
    assert status_of(phase, 'd_cmd never positive') == ph.FAIL


def test_phase6_fails_a_stall_longer_than_three_seconds():
    phase = ph.Phase6Obstacle()
    run = _obstacle_run()
    stalled = [_run_status(state='blocked', follower_speed=0.0,
                           travelled=6.0)] * 25          # 5 s at dt 0.2
    assert drive(phase, run[:60] + stalled + run[60:]) == ph.FAIL
    assert status_of(phase, 'never stalled') == ph.FAIL


def test_phase6_fails_a_resume_that_takes_too_long():
    # The textbook run is back in lane 1.6 m after entering `resuming`; hold
    # it to 1.0 m and the same run has to fail.
    th = ph.Thresholds(resume_within_m=1.0)
    phase = ph.Phase6Obstacle(th)
    assert drive(phase, _obstacle_run()) == ph.FAIL
    assert status_of(phase, 'back in lane') == ph.FAIL


def test_phase6_does_not_count_the_parked_robot_after_the_run_as_a_stall():
    # Found end to end: the robot finishes, sits still at the far end, and a
    # naive stall counter turns every completed run into a failure.
    phase = ph.Phase6Obstacle()
    parked = [_run_status(state='finished', follower_speed=0.0,
                          travelled=30.0)] * 200          # 40 s at dt 0.2
    assert drive(phase, _obstacle_run() + parked) == ph.PASS
    assert phase.max_stationary_s == 0.0


def test_phase6_does_not_count_the_wait_before_the_run_as_a_stall():
    # Start is pressed, then the operator walks back to the robot.
    phase = ph.Phase6Obstacle()
    waiting = [_run_status(follower_speed=0.0, travelled=0.0)] * 100
    assert drive(phase, waiting + _obstacle_run()) == ph.PASS


def test_phase6_still_catches_a_stall_in_the_middle_of_a_run():
    phase = ph.Phase6Obstacle()
    run = _obstacle_run()
    stalled = [_run_status(state='blocked', follower_speed=0.0,
                           travelled=6.0)] * 25
    assert drive(phase, run[:60] + stalled + run[60:]) == ph.FAIL
    assert phase.max_stationary_s == pytest.approx(5.0, abs=0.3)


# -- Phase 7 -----------------------------------------------------------------

def test_phase7_passes_when_the_robot_slows_then_stops():
    phase = ph.Phase7GnssFault()
    run = [snap(follower_ok=True, sigma_h=0.02, follower_speed=0.4)] * 20
    run += [snap(follower_ok=True, sigma_h=0.09, follower_speed=0.20)] * 20
    run += [snap(follower_ok=True, sigma_h=0.30, follower_speed=0.0)] * 20
    assert drive(phase, run) == ph.PASS


def test_phase7_accepts_a_fix_lost_outright():
    # A bucket over the antenna usually removes the fix rather than inflating
    # sigma. The follower stops on either, so the gate must credit either --
    # otherwise the most realistic version of this test hangs on pending.
    phase = ph.Phase7GnssFault()
    run = [snap(follower_ok=True, sigma_h=0.02, follower_speed=0.4)] * 20
    run += [snap(follower_ok=True, raw_ok=False, sigma_raw=None, sigma_h=None,
                 follower_speed=0.0)] * 20
    assert drive(phase, run) == ph.PASS
    assert phase.fix_lost


def test_phase7_does_not_punish_the_robot_for_recovering():
    # Lift the bucket off the antenna and the robot is entitled to drive
    # again. Judging that against a latched "fix was lost" flag would fail
    # the phase for doing exactly what it should.
    phase = ph.Phase7GnssFault()
    run = [snap(follower_ok=True, sigma_h=0.02, follower_speed=0.4)] * 20
    run += [snap(follower_ok=True, raw_ok=False, sigma_raw=None, sigma_h=None,
                 follower_speed=0.0)] * 20
    run += [snap(follower_ok=True, sigma_h=0.02, follower_speed=0.4)] * 40
    assert drive(phase, run) == ph.PASS
    assert not phase.drove_degraded


def test_phase7_stays_pending_on_a_partial_degrade():
    # Slowing proves the ramp; it does not prove the stop.
    phase = ph.Phase7GnssFault()
    run = [snap(follower_ok=True, sigma_h=0.10, follower_speed=0.15)] * 40
    assert drive(phase, run) == ph.PENDING
    assert status_of(phase, 'slowed down') == ph.PASS
    assert status_of(phase, 'came to a stop') == ph.PENDING


def test_phase7_fails_a_robot_that_keeps_driving_on_a_bad_fix():
    phase = ph.Phase7GnssFault()
    run = [snap(follower_ok=True, sigma_h=0.30, follower_speed=0.4)] * 40
    assert drive(phase, run) == ph.FAIL
    assert status_of(phase, 'never drove fast') == ph.FAIL


# -- phase plumbing ----------------------------------------------------------

def test_operator_mark_overrides_the_computed_verdict():
    # Phase 4's recorded-route comparison happens offline, so the operator has
    # to be able to put the answer on the record.
    phase = ph.Phase4LeverArm()
    drive(phase, [snap(lever_x=0.28, lever_y=+0.42)] * 20)
    assert phase.verdict() == ph.FAIL
    phase.mark(ph.PASS)
    assert phase.verdict() == ph.PASS
    phase.reset()
    assert phase.verdict() == ph.PENDING


def test_reset_clears_a_latched_failure():
    phase = ph.Phase5AutonomousRun()
    drive(phase, [_run_status(cross_track=0.9)] * 5)
    assert phase.verdict() == ph.FAIL
    phase.reset()
    assert phase.max_cross_track == 0.0
    assert phase.verdict() == ph.PENDING


def test_every_phase_starts_pending_and_serialises():
    for phase in ph.build_phases():
        assert phase.verdict() == ph.PENDING, phase.name
        data = phase.as_dict()
        assert data['gate'] and data['action'] and data['name']
        assert isinstance(data['checks'], list) and data['checks']


def test_build_phases_covers_zero_to_seven_in_order():
    assert [p.index for p in ph.build_phases()] == list(range(8))


def test_thresholds_match_the_shipped_stack_configs():
    # These mirror confidence_gate.yaml, route_alley.yaml and chassis.yaml. If
    # one of those moves, this test is the reminder to move the other.
    th = ph.Thresholds()
    assert th.sigma_gate_m == 0.05          # confidence_gate max_horizontal_sigma_m
    assert th.sigma_slow_m == 0.05          # route_alley sigma_slow_m
    assert th.sigma_stop_m == 0.15          # route_alley sigma_stop_m
    assert th.lever_arm_y_m == -0.42        # chassis gnss_link y
    assert th.teach_sample_dist_m == 1.0    # route_alley sample_dist_m
    assert th.loop_closure_tolerance_m == 3.0
