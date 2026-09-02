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
