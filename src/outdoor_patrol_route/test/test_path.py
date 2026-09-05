# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Geometry tests for the smoothed, arc-length-parameterised path.

The circle is the useful fixture here: its length, tangents and offset
polylines are all known in closed form, so every property can be checked
against an exact answer rather than against a previous run.
"""

import math

import numpy as np
import pytest

from outdoor_patrol_route.path import (Path, forward_gap, savitzky_golay)

RADIUS = 10.0


def circle(count=60, radius=RADIUS, noise=0.0, seed=1):
    angles = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
    points = np.stack([radius * np.cos(angles), radius * np.sin(angles)],
                      axis=1)
    if noise:
        points = points + np.random.default_rng(seed).normal(
            0.0, noise, points.shape)
    return points


def test_loop_length_matches_the_circle():
    path = Path(circle(), loop=True, smooth_window=0)
    assert path.length == pytest.approx(2 * math.pi * RADIUS, rel=1e-3)


def test_projection_returns_station_and_signed_lateral():
    path = Path(circle(), loop=True, smooth_window=0)
    # A point on the inside of a counter-clockwise circle is to the LEFT.
    inside = (RADIUS - 1.5, 0.0)
    _, lateral = path.project(*inside)
    assert lateral == pytest.approx(1.5, abs=0.05)

    outside = (RADIUS + 1.5, 0.0)
    _, lateral = path.project(*outside)
    assert lateral == pytest.approx(-1.5, abs=0.05)


def test_offset_and_projection_are_inverses():
    path = Path(circle(), loop=True, smooth_window=0)
    for station in (0.0, 7.5, 31.4, 55.0):
        for offset in (0.0, -2.4, 1.2):
            x, y = path.offset_at(station, offset)
            s_back, lateral = path.project(x, y, s_hint=station)
            assert s_back == pytest.approx(station, abs=0.1)
            assert lateral == pytest.approx(offset, abs=0.02)


def test_stations_of_matches_pointwise_projection():
    path = Path(circle(), loop=True, smooth_window=0)
    points = np.array([path.offset_at(20.0 + i * 0.4, -1.0)
                       for i in range(12)])
    station, lateral, interior = path.stations_of(points, 18.0, 28.0)
    assert interior.all()
    assert lateral == pytest.approx(np.full(12, -1.0), abs=0.02)
    assert station == pytest.approx(20.0 + np.arange(12) * 0.4, abs=0.1)


def test_points_outside_the_window_are_not_interior():
    path = Path(circle(), loop=True, smooth_window=0)
    behind = np.array([path.offset_at(5.0, 0.0)])
    _, _, interior = path.stations_of(behind, 20.0, 30.0)
    assert not interior.any()


def test_smoothing_reduces_tangent_noise_without_shrinking_the_circle():
    truth = Path(circle(), loop=True, smooth_window=0)
    noisy_points = circle(noise=0.02)

    rough = Path(noisy_points, loop=True, smooth_window=0)
    smooth = Path(noisy_points, loop=True, smooth_window=5)

    def tangent_error(path):
        errors = []
        for station in np.linspace(0, truth.length, 200, endpoint=False):
            x, y, _ = truth.pose_at(station)
            s_here, _ = path.project(x, y)
            _, _, yaw = path.pose_at(s_here)
            _, _, want = truth.pose_at(station)
            errors.append(abs(math.atan2(math.sin(yaw - want),
                                         math.cos(yaw - want))))
        return float(np.sqrt(np.mean(np.square(errors))))

    assert tangent_error(smooth) < tangent_error(rough)
    # Smoothing must not eat the geometry: a 5-station window on a 60-station
    # circle should stay well inside 1 % of the true circumference.
    assert smooth.length == pytest.approx(truth.length, rel=0.01)


def test_savitzky_golay_with_a_short_window_is_the_identity():
    points = circle(noise=0.05)
    assert savitzky_golay(points, 3, loop=True) == pytest.approx(points)
    assert savitzky_golay(points, 0, loop=True) == pytest.approx(points)


def test_savitzky_golay_preserves_a_straight_line():
    line = np.stack([np.arange(20.0), np.zeros(20)], axis=1)
    assert savitzky_golay(line, 5, loop=False) == pytest.approx(line, abs=1e-9)


def test_forward_gap_wraps_the_short_way_round_a_loop():
    assert forward_gap(1.0, 99.0, 100.0, loop=True) == pytest.approx(-2.0)
    assert forward_gap(99.0, 1.0, 100.0, loop=True) == pytest.approx(2.0)
    assert forward_gap(1.0, 99.0, 100.0, loop=False) == pytest.approx(98.0)


def test_open_path_clamps_instead_of_wrapping():
    line = np.stack([np.arange(0.0, 20.0), np.zeros(20)], axis=1)
    path = Path(line, loop=False, smooth_window=0)
    assert path.wrap_s(-5.0) == 0.0
    assert path.wrap_s(1e6) == pytest.approx(path.length)


def test_duplicate_control_points_do_not_break_the_spline():
    points = np.vstack([circle(20), circle(20)[:1]])
    path = Path(points, loop=True, smooth_window=0)
    assert np.isfinite(path.xy).all()
    assert path.length > 0.0


# -- curvature and offset folding --------------------------------------------

def test_curvature_of_a_circle_is_one_over_its_radius():
    # Counter-clockwise, so REP-103 signed curvature is positive (left turn).
    path = Path(circle(120), loop=True, smooth_window=0)
    assert np.median(path.curvature) == pytest.approx(1.0 / RADIUS, rel=0.02)
    assert (path.curvature > 0).all()


def test_curvature_is_signed_by_turn_direction():
    ccw = Path(circle(120), loop=True, smooth_window=0)
    cw = Path(circle(120)[::-1], loop=True, smooth_window=0)
    assert np.median(ccw.curvature) > 0.0
    assert np.median(cw.curvature) < 0.0


def test_curvature_of_a_straight_line_is_zero():
    points = np.stack([np.arange(0.0, 30.0), np.zeros(30)], axis=1)
    path = Path(points, loop=False, smooth_window=0)
    assert np.abs(path.curvature).max() < 1e-6


def test_curvature_is_not_spiked_by_the_heading_wrap():
    # A straight line along -x sits exactly on the +/-pi discontinuity. Without
    # unwrapping, differencing the heading there reports a curvature of about
    # 2*pi / (2*step) -- 63 m^-1 at the default 5 cm step.
    points = np.stack([-np.arange(0.0, 30.0), np.zeros(30)], axis=1)
    path = Path(points, loop=False, smooth_window=0)
    assert np.abs(path.curvature).max() < 1e-6


def test_offset_scale_stretches_outside_and_shrinks_inside():
    path = Path(circle(120), loop=True, smooth_window=0)
    # Left turn: a left (+) offset is the inside, a right (-) offset outside.
    assert np.median(path.offset_scale(+2.0)) == pytest.approx(0.8, rel=0.05)
    assert np.median(path.offset_scale(-2.0)) == pytest.approx(1.2, rel=0.05)


def test_offset_folds_once_the_offset_reaches_the_corner_radius():
    path = Path(circle(120, radius=3.0), loop=True, smooth_window=0)
    assert not path.offset_folds(+2.0)[0]      # inside, but still short of 3 m
    assert path.offset_folds(+3.5)[0]          # inside and past the radius
    assert not path.offset_folds(-8.0)[0]      # outside never folds


def test_offset_folds_reports_where_it_happens():
    # A straight, then a tight left bend: only the bend can fold.
    straight = np.stack([np.arange(0.0, 12.0), np.zeros(12)], axis=1)
    angles = np.linspace(0.0, math.pi / 2, 12)[1:]
    bend = np.stack([11.0 + 1.5 * np.sin(angles),
                     1.5 - 1.5 * np.cos(angles)], axis=1)
    path = Path(np.vstack([straight, bend]), loop=False)
    folds, scale, station = path.offset_folds(+2.5)
    assert folds and scale <= 0.0
    assert station > 10.0


def test_min_turn_radius_finds_the_tightest_corner_on_one_side():
    path = Path(circle(120, radius=4.0), loop=True, smooth_window=0)
    radius, _ = path.min_turn_radius(sign=+1.0)     # it turns left
    assert radius == pytest.approx(4.0, rel=0.05)
    # No right-hand bend anywhere, so a right retreat can never fold.
    assert math.isinf(path.min_turn_radius(sign=-1.0)[0])


def test_min_turn_radius_of_a_straight_is_infinite():
    points = np.stack([np.arange(0.0, 30.0), np.zeros(30)], axis=1)
    path = Path(points, loop=False, smooth_window=0)
    assert math.isinf(path.min_turn_radius()[0])


def _recorder_stations(dense, dist_trig=1.0, yaw_trig_deg=5.0):
    """Apply route_recorder's two triggers: 1 m of travel OR 5 deg of yaw."""
    out = [dense[0]]
    last = dense[0]
    last_yaw = None
    for i in range(1, len(dense)):
        x, y = dense[i]
        px, py = dense[i - 1]
        yaw = math.atan2(y - py, x - px)
        moved = math.hypot(x - last[0], y - last[1])
        turned = (999.0 if last_yaw is None
                  else abs((math.degrees(yaw - last_yaw) + 180) % 360 - 180))
        if moved >= dist_trig or turned >= yaw_trig_deg:
            out.append((x, y))
            last = (x, y)
            last_yaw = yaw
    return np.array(out)


def _rounded_square(half, radius, fine=0.02):
    """Densely sampled rounded square -- the ground truth a robot drives."""
    side = half - radius
    arcs = []
    for cx, cy, q in ((side, side, 0.5), (side, -side, 0.0),
                      (-side, -side, -0.5), (-side, side, -1.0)):
        a0 = q * math.pi
        for k in range(int(radius * math.pi / 2 / fine) + 1):
            a = a0 - k * fine / radius
            arcs.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    full = []
    for i, point in enumerate(arcs):
        full.append(point)
        nxt = arcs[(i + 1) % len(arcs)]
        gap = math.hypot(nxt[0] - point[0], nxt[1] - point[1])
        if gap > 0.05:
            for t in np.arange(fine, gap, fine):
                full.append((point[0] + (nxt[0] - point[0]) * t / gap,
                             point[1] + (nxt[1] - point[1]) * t / gap))
    return np.array(full)


def test_curvature_survives_the_recorders_uneven_station_spacing():
    """A real recording must not make a good corner look like a bad one.

    route_recorder samples every 1 m on a straight but every 5 deg round a
    bend -- 8.7 cm on a 1 m corner, a spacing ratio over 50:1. Catmull-Rom
    interpolates every station, so it ripples at those transitions. The
    ripple is sub-centimetre in position, but curvature is a second
    derivative: differencing adjacent 5 cm samples reported 0.24 m for a
    genuine 1.00 m corner, which would condemn a perfectly good route.
    """
    stations = _recorder_stations(_rounded_square(1.75, 1.0))
    spacing = np.hypot(*np.diff(stations, axis=0).T)
    assert spacing.max() / spacing.min() > 10.0     # genuinely uneven

    radius, _ = Path(stations, loop=True).min_turn_radius()
    # Within 25%, and on the conservative side: reporting a corner slightly
    # tighter than it is warns early, which is the safe direction.
    assert 0.75 <= radius <= 1.05


def test_curvature_tracks_corner_radius_across_a_range():
    for true_radius, half in ((1.5, 1.75), (1.0, 1.75), (0.75, 1.6)):
        stations = _recorder_stations(_rounded_square(half, true_radius))
        radius, _ = Path(stations, loop=True).min_turn_radius()
        assert radius == pytest.approx(true_radius, rel=0.30), true_radius
        assert radius <= true_radius * 1.05, 'must not over-report radius'


def test_offset_scale_marks_folded_samples_for_the_markers():
    """The corridor display uses this to draw gaps instead of a tangle.

    Offsetting a wiggly hand-taught route by 2-3 m inverts it wherever the
    local radius is smaller than the offset. Drawing those inverted segments
    puts corridor lines where no corridor exists; the sign of offset_scale is
    what lets the follower drop them.
    """
    # Tight circle: a 2 m offset on the inside is well past the 1 m radius.
    path = Path(circle(120, radius=1.0), loop=True, smooth_window=0)
    inside = path.offset_scale(+2.0)
    assert (inside <= 0.0).all(), 'every inside sample should read as folded'
    outside = path.offset_scale(-2.0)
    assert (outside > 0.0).all(), 'the outside of a bend never folds'


def test_offset_scale_is_all_positive_on_a_gentle_route():
    path = Path(circle(120, radius=20.0), loop=True, smooth_window=0)
    assert (path.offset_scale(+2.0) > 0.0).all()
    assert (path.offset_scale(-2.0) > 0.0).all()
