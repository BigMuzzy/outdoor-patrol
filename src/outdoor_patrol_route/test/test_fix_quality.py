# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Tests for horizontal_sigma, the fix-quality number speed decisions use.

Every case here is a bug that was actually in the code, or a hazard the
function exists to prevent. The numbers come from a measured 20-minute
RTK-fixed soak (runs/gnss/soak_day1_report.txt), not from invention.
"""

import math

import pytest
from sensor_msgs.msg import NavSatFix, NavSatStatus

from outdoor_patrol_route.route_file import horizontal_sigma


def make_fix(east_var=0.0004, north_var=0.0004,
             cov_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
             status=NavSatStatus.STATUS_FIX):
    msg = NavSatFix()
    msg.status.status = status
    msg.position_covariance = [east_var, 0.0, 0.0,
                               0.0, north_var, 0.0,
                               0.0, 0.0, 4 * east_var]
    msg.position_covariance_type = cov_type
    return msg


def test_uses_the_worse_axis_not_just_east():
    """The original bug: reading cov[0] alone understated sigma.

    Measured at this site, north was 1.7x worse than east (0.017 m vs
    0.010 m), so an east-only reading would let the follower drive on a fix
    confidence_gate -- which tests max(cov[0], cov[4]) -- calls degraded.
    """
    msg = make_fix(east_var=0.010 ** 2, north_var=0.017 ** 2)
    assert horizontal_sigma(msg) == pytest.approx(0.017, abs=1e-6)

    # And symmetrically, when east is the worse axis.
    msg = make_fix(east_var=0.030 ** 2, north_var=0.010 ** 2)
    assert horizontal_sigma(msg) == pytest.approx(0.030, abs=1e-6)


def test_agrees_with_the_confidence_gate_threshold():
    """Both sides must classify a borderline fix the same way.

    confidence_gate passes when sqrt(max(cov[0], cov[4])) <= 0.05. If this
    function disagreed, the follower would drive on fixes the gate had
    already inflated, or stop on fixes it had passed.
    """
    gate_limit = 0.05
    # East fine, north just over: the gate degrades this, so must we.
    msg = make_fix(east_var=0.01 ** 2, north_var=0.06 ** 2)
    assert horizontal_sigma(msg) > gate_limit
    # Both comfortably inside.
    msg = make_fix(east_var=0.01 ** 2, north_var=0.02 ** 2)
    assert horizontal_sigma(msg) <= gate_limit


def test_unknown_covariance_is_infinite_not_zero():
    """The hazard: an all-zero covariance must not read as a perfect fix.

    position_covariance is all-zero when the type is UNKNOWN. A naive
    sqrt(cov[0]) returns 0.0, which is the *best possible* sigma and would
    command full speed on a fix carrying no quality information at all.
    """
    msg = make_fix(east_var=0.0, north_var=0.0,
                   cov_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN)
    assert horizontal_sigma(msg) == math.inf


def test_no_fix_is_infinite():
    """NO_FIX must stop the robot even if covariance looks healthy.

    On the RAW driver topic these are not filtered out, unlike on
    /gnss/fix_gated where confidence_gate drops them.
    """
    msg = make_fix(east_var=0.001 ** 2, north_var=0.001 ** 2,
                   status=NavSatStatus.STATUS_NO_FIX)
    assert horizontal_sigma(msg) == math.inf


def test_measured_rtk_fixed_soak_passes_the_gate():
    """Real numbers from runs/gnss/soak_day1: 0.017 m median, 0.021 m worst."""
    assert horizontal_sigma(
        make_fix(0.010 ** 2, 0.017 ** 2)) == pytest.approx(0.017, abs=1e-6)
    assert horizontal_sigma(
        make_fix(0.011 ** 2, 0.021 ** 2)) == pytest.approx(0.021, abs=1e-6)
    # Worst observed still has better than 2x margin on the 0.05 m gate.
    assert horizontal_sigma(make_fix(0.011 ** 2, 0.021 ** 2)) < 0.05 / 2


def test_infinite_sigma_survives_arithmetic_downstream():
    """Inf must flow through the follower's speed scaling as a full stop."""
    sigma = horizontal_sigma(
        make_fix(cov_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN))
    slow, stop = 0.10, 0.50
    # Mirrors _fix_penalty: >= stop means zero speed.
    assert not sigma <= slow
    assert sigma >= stop


@pytest.mark.parametrize('east, north, expected', [
    (0.02, 0.02, 0.02),
    (0.05, 0.01, 0.05),
    (0.01, 0.05, 0.05),
    (0.30, 0.30, 0.30),
])
def test_sigma_round_trip(east, north, expected):
    assert horizontal_sigma(
        make_fix(east ** 2, north ** 2)) == pytest.approx(expected, abs=1e-9)
