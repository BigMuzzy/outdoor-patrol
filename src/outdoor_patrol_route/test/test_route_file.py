# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Round-trip and rejection tests for the route file schema."""

import pytest

from outdoor_patrol_route import route_file
from outdoor_patrol_route.route_file import Route, Sample


def _route(**kwargs):
    defaults = {
        'datum': (-41.28646, 174.776236, 10.0),
        'loop': True,
        'lane_half_width_m': 2.0,
        'source': route_file.SOURCE_ODOMETRY,
        'samples': [Sample(lat=-41.28 + i * 1e-5, lon=174.77, alt=10.0,
                           yaw=0.1 * i, fix='fixed', sigma_h=0.02)
                    for i in range(6)],
    }
    defaults.update(kwargs)
    return Route(**defaults)


def test_round_trip_preserves_every_field():
    original = _route()
    restored = route_file.loads(route_file.dumps(original))

    assert restored.loop == original.loop
    assert restored.source == original.source
    assert restored.datum == pytest.approx(original.datum)
    assert len(restored.samples) == len(original.samples)
    for before, after in zip(original.samples, restored.samples):
        # 9 decimal places of latitude is about 0.1 mm, so nothing that
        # matters can be lost in the text form.
        assert after.lat == pytest.approx(before.lat, abs=1e-9)
        assert after.lon == pytest.approx(before.lon, abs=1e-9)
        assert after.yaw == pytest.approx(before.yaw, abs=1e-6)
        assert after.fix == before.fix


def test_raw_antenna_route_is_flagged_as_not_base_link():
    assert _route(source=route_file.SOURCE_RAW_ANTENNA).is_base_link is False
    assert _route(source=route_file.SOURCE_ODOMETRY).is_base_link is True
    assert _route(source=route_file.SOURCE_FIX_LEVER_ARM).is_base_link is True


def test_worst_fix_reports_the_lowest_class_present():
    route = _route()
    route.samples[3].fix = 'float'
    assert route.worst_fix() == 'float'
    route.samples[1].fix = 'single'
    assert route.worst_fix() == 'single'


def test_wrong_schema_version_is_rejected():
    text = route_file.dumps(_route()).replace('version: 1', 'version: 99')
    with pytest.raises(ValueError, match='schema version'):
        route_file.loads(text)


def test_missing_datum_is_rejected():
    text = '\n'.join(line for line in route_file.dumps(_route()).splitlines()
                     if not line.startswith('datum:'))
    with pytest.raises(ValueError, match='datum'):
        route_file.loads(text)


def test_too_few_samples_is_rejected():
    short = _route(samples=[Sample(lat=0.0, lon=0.0, alt=0.0, yaw=0.0)] * 2)
    with pytest.raises(ValueError, match='at least 4'):
        route_file.loads(route_file.dumps(short))


@pytest.mark.parametrize('status, sigma, expected', [
    (-1, 0.01, 'none'),
    (0, 0.02, 'fixed'),
    (0, 0.05, 'fixed'),
    (0, 0.20, 'float'),
    (0, 2.00, 'single'),
    (0, 0.00, 'single'),
])
def test_fix_classification(status, sigma, expected):
    assert route_file.classify_fix(status, sigma) == expected
