#!/usr/bin/env python3
# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Characterise GNSS accuracy at a site from one or more static soaks.

Answers three questions, in decreasing order of how much they matter:

1. **Will the stack drive here?** ``confidence_gate`` compares
   ``sqrt(max(cov[0], cov[4]))`` -- the WORSE per-axis 1-sigma, not a radial
   error -- against ``max_horizontal_sigma_m`` (0.05 m). That number is
   reported by the receiver, so this is directly measurable and is the actual
   go/no-go.

2. **Is the receiver's self-reported sigma honest?** Compare what it claims
   against the scatter it actually produces. This matters because the gate
   trusts the claim. A receiver that under-reports sigma passes fixes the gate
   would otherwise stop, which is the failure you cannot see from outside.

3. **What does the provider's "3-7 cm" mean?** Partially answerable -- see the
   limits below. It is the weakest of the three and, fortunately, the least
   important: the gate never sees the provider's datasheet.

## What a static soak can and cannot measure

A single session measures **precision** (scatter about the session mean), not
**accuracy** (offset from truth). RTK error is strongly time-correlated:
multipath geometry, residual ionosphere and the base-rover baseline all drift
over minutes to hours. Inside one 10-minute session a 4 cm systematic offset
looks like a *constant*, not like scatter, so the within-session standard
deviation **underestimates** true error, often badly.

Providers normally quote accuracy against truth. Comparing your session
scatter to their accuracy figure is therefore apples-to-oranges, and flatters
the provider.

Two ways to do better, in order of cost:

* **Occupy the same physical mark on several different days**, at different
  times of day so the satellite geometry differs, and pass every session at
  once. The spread of session *means* captures the slowly-varying error that
  within-session scatter hides. No survey equipment needed, and it is the
  single biggest improvement available.
* Occupy a surveyed benchmark if you have one and pass ``--truth``; then the
  offsets are true accuracy rather than a proxy.

## Distinguishing 1-sigma from CEP from 95%

For a roughly circular bivariate normal fix cloud the usual radial measures
are fixed multiples of the per-axis sigma:

===============  ==================
Measure          multiple of sigma
===============  ==================
1-sigma          1.00
CEP (50%)        1.18
DRMS (~65%)      1.41
R95              2.45
2DRMS (~95%)     2.83
===============  ==================

A spec quoted at 95% is about **2.4x** the per-axis sigma, and that IS
distinguishable from a 1-sigma spec by measurement. CEP is only 1.18x and is
**not** reliably separable from 1-sigma given site-to-site variation -- but
that does not matter, because both lead to the same operational conclusion.

## Sample independence

Do not be impressed by sample count. At 5 Hz a 10-minute soak is 3000 samples,
but GNSS error decorrelates over tens of seconds, so the number of
*independent* samples is nearer 10-30. This script estimates the integrated
autocorrelation time, reports an effective N, and widens its confidence
interval accordingly. A tight-looking sigma from one short session is not
evidence.

Usage::

    ros2 bag record -o soak_day1 /um982_driver/fix     # 10+ min, robot still
    analyse_gnss_soak.py --bag soak_day1 [--bag soak_day2 ...]
    analyse_gnss_soak.py --self-test    # verify the maths on known input
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

# WGS84.
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)

#: What confidence_gate allows through: metres of per-axis 1-sigma.
GATE_SIGMA_M = 0.05

#: Radial measures as multiples of the per-axis sigma (circular bivariate
#: normal), used to test which reading of a provider spec fits the data.
SPEC_MULTIPLES = {
    '1-sigma': 1.00,
    'CEP (50%)': 1.18,
    'DRMS (~65%)': 1.41,
    'R95': 2.45,
    '2DRMS (~95%)': 2.83,
}

#: NavSatFix covariance_type values.
_COV_UNKNOWN = 0
_COV_DIAGONAL_KNOWN = 2
_COV_APPROXIMATED = 3


def _to_ecef(lat_deg, lon_deg, alt):
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    n = _A / np.sqrt(1.0 - _E2 * np.sin(lat) ** 2)
    return np.stack([(n + alt) * np.cos(lat) * np.cos(lon),
                     (n + alt) * np.cos(lat) * np.sin(lon),
                     (n * (1.0 - _E2) + alt) * np.sin(lat)], axis=-1)


def geodetic_to_enu(lat_deg, lon_deg, alt, datum):
    """Local ENU metres about `datum` = (lat, lon, alt)."""
    d_lat, d_lon, d_alt = datum
    delta = (_to_ecef(np.asarray(lat_deg), np.asarray(lon_deg),
                      np.asarray(alt))
             - _to_ecef(np.array(d_lat), np.array(d_lon), np.array(d_alt)))
    lat, lon = math.radians(d_lat), math.radians(d_lon)
    east = np.array([-math.sin(lon), math.cos(lon), 0.0])
    north = np.array([-math.sin(lat) * math.cos(lon),
                      -math.sin(lat) * math.sin(lon), math.cos(lat)])
    up = np.array([math.cos(lat) * math.cos(lon),
                   math.cos(lat) * math.sin(lon), math.sin(lat)])
    return delta @ east, delta @ north, delta @ up


def autocorrelation_time(series: np.ndarray, dt: float) -> float:
    """Integrated autocorrelation time, in seconds.

    tau_int = 1 + 2 * sum(rho_k), truncated at the first non-positive
    autocorrelation (Geyer's initial-positive-sequence rule) so noise in the
    tail does not accumulate. Scaled by the sample period to give seconds.
    """
    x = np.asarray(series, dtype=float)
    x = x - x.mean()
    n = len(x)
    if n < 8 or np.allclose(x, 0.0):
        return dt

    var = float(np.dot(x, x)) / n
    if var <= 0.0:
        return dt

    total = 0.0
    for lag in range(1, min(n // 2, 4000)):
        rho = float(np.dot(x[:-lag], x[lag:])) / (n * var)
        if rho <= 0.0:
            break
        total += rho
    return dt * (1.0 + 2.0 * total)


def sigma_confidence(sigma: float, n_eff: float):
    """Rough 95% interval on an estimated sigma from n_eff samples.

    The standard error of a sample standard deviation is about
    sigma / sqrt(2 * (n - 1)). With a handful of independent samples this
    interval is wide, which is the point: it stops a tight-looking number from
    a short soak being read as precision it has not earned.
    """
    if n_eff <= 2:
        return (0.0, float('inf'))
    rel = 1.0 / math.sqrt(2.0 * (n_eff - 1.0))
    return (sigma * max(0.0, 1.0 - 1.96 * rel), sigma * (1.0 + 1.96 * rel))


def analyse_session(east, north, up, times, reported_sigma, label='session'):
    """Statistics for one static occupation."""
    east = np.asarray(east, dtype=float)
    north = np.asarray(north, dtype=float)
    up = np.asarray(up, dtype=float)
    times = np.asarray(times, dtype=float)

    duration = float(times[-1] - times[0]) if len(times) > 1 else 0.0
    dt = duration / max(len(times) - 1, 1) if duration > 0 else 0.2

    de, dn, du = east - east.mean(), north - north.mean(), up - up.mean()
    sigma_e, sigma_n, sigma_u = (float(np.std(de)), float(np.std(dn)),
                                 float(np.std(du)))
    # The gate's own quantity: the worse horizontal axis.
    sigma_axis = max(sigma_e, sigma_n)
    radial = np.hypot(de, dn)

    tau = max(autocorrelation_time(de, dt), autocorrelation_time(dn, dt))
    n_eff = max(2.0, duration / tau) if tau > 0 else float(len(east))

    return {
        'label': label,
        'samples': int(len(east)),
        'duration_s': duration,
        'rate_hz': (len(east) / duration) if duration > 0 else float('nan'),
        'mean_enu': (float(east.mean()), float(north.mean()),
                     float(up.mean())),
        'sigma_e': sigma_e,
        'sigma_n': sigma_n,
        'sigma_u': sigma_u,
        'sigma_axis': sigma_axis,
        'sigma_axis_ci': sigma_confidence(sigma_axis, n_eff),
        'radial_cep': float(np.percentile(radial, 50)),
        'radial_r95': float(np.percentile(radial, 95)),
        'radial_max': float(np.max(radial)),
        'tau_s': tau,
        'n_eff': n_eff,
        'reported_sigma_median': (float(np.median(reported_sigma))
                                  if len(reported_sigma) else float('nan')),
        'reported_sigma_max': (float(np.max(reported_sigma))
                               if len(reported_sigma) else float('nan')),
    }


def spec_consistency(sigma_axis: float, spec_lo: float, spec_hi: float):
    """Which readings of a provider spec are consistent with a measured sigma.

    A spec quoted as measure M means the provider's number is
    SPEC_MULTIPLES[M] * sigma, so an observed sigma implies a spec value of
    sigma * multiple. Report every reading whose implied value falls inside
    the quoted range.
    """
    return [{'reading': name,
             'implied_spec_m': sigma_axis * multiple,
             'consistent': spec_lo <= sigma_axis * multiple <= spec_hi}
            for name, multiple in SPEC_MULTIPLES.items()]


# --------------------------------------------------------------------------

def _resolve_bag(path: str):
    # Check existence first. Without this a missing path falls through to
    # rosbag2, which reports "no plugin found that could open URI" -- an
    # accurate but thoroughly misleading way to say "no such file".
    if not os.path.exists(path):
        raise SystemExit(
            '%s does not exist.\n'
            '  Record a soak first (robot parked, 15+ min):\n'
            "    ssh robot 'ros2 bag record -o /data/soak_day1 "
            "/um982_driver/fix'\n"
            '  then copy it to this machine:\n'
            '    scp -r robot:~/code/outdoor-patrol/deploy/data/soak_day1 '
            '/tmp/' % path)
    if os.path.isdir(path):
        if os.path.exists(os.path.join(path, 'metadata.yaml')):
            return path, ''
        for name in sorted(os.listdir(path)):
            if name.endswith('.mcap'):
                return os.path.join(path, name), 'mcap'
            if name.endswith('.db3'):
                return os.path.join(path, name), 'sqlite3'
        raise SystemExit('%s: no metadata.yaml and no storage file' % path)
    if path.endswith('.mcap'):
        return path, 'mcap'
    if path.endswith('.db3'):
        return path, 'sqlite3'
    return path, ''


def read_bag(path: str, topic: str):
    """(lat, lon, alt, times, reported_sigma, meta) from a bag of fixes."""
    # Resolve the path BEFORE importing rclpy: a missing bag should say so,
    # not fail with ModuleNotFoundError because the shell has no ROS sourced.
    uri, storage_id = _resolve_bag(path)

    try:
        from rclpy.serialization import deserialize_message
        import rosbag2_py
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit(
            'ROS 2 is not on the Python path (%s).\n'
            '  source /opt/ros/jazzy/setup.bash' % exc)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit('%s: no %s in the bag (has: %s)'
                         % (path, topic, ', '.join(sorted(types))))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    message_type = get_message(types[topic])

    lat, lon, alt, times, sigma = [], [], [], [], []
    cov_types, statuses = set(), set()
    while reader.has_next():
        _, data, stamp = reader.read_next()
        m = deserialize_message(data, message_type)
        cov_types.add(int(m.position_covariance_type))
        statuses.add(int(m.status.status))
        if m.position_covariance_type == _COV_UNKNOWN or m.status.status < 0:
            continue
        lat.append(m.latitude)
        lon.append(m.longitude)
        alt.append(m.altitude)
        times.append(stamp * 1e-9)
        # Exactly the quantity confidence_gate tests.
        sigma.append(math.sqrt(max(m.position_covariance[0],
                                   m.position_covariance[4], 0.0)))

    if len(lat) < 30:
        raise SystemExit('%s: only %d usable fixes; a soak needs minutes of '
                         'data' % (path, len(lat)))

    return (np.array(lat), np.array(lon), np.array(alt), np.array(times),
            np.array(sigma), {'cov_types': sorted(cov_types),
                              'statuses': sorted(statuses)})


# --------------------------------------------------------------------------

def report(sessions, metas, spec_lo, spec_hi, truth=None):
    print('=' * 72)
    print('GNSS site characterisation -- %d session(s)' % len(sessions))
    print('=' * 72)

    for s, meta in zip(sessions, metas):
        cov = meta['cov_types']
        source = ('receiver GST (measured)' if cov == [_COV_DIAGONAL_KNOWN]
                  else 'HDOP heuristic (estimated)'
                  if cov == [_COV_APPROXIMATED] else 'MIXED %s' % cov)
        print()
        print('%s' % s['label'])
        print('  %d fixes over %.1f min at %.1f Hz'
              % (s['samples'], s['duration_s'] / 60.0, s['rate_hz']))
        print('  covariance source : %s' % source)
        print('  receiver claims   : %.3f m per-axis 1-sigma (median), '
              '%.3f m worst'
              % (s['reported_sigma_median'], s['reported_sigma_max']))
        print('  actually scattered: %.3f m east, %.3f m north, %.3f m up'
              % (s['sigma_e'], s['sigma_n'], s['sigma_u']))
        lo, hi = s['sigma_axis_ci']
        print('  worse axis        : %.3f m  (95%% CI %.3f - %.3f)'
              % (s['sigma_axis'], lo, hi))
        print('  radial            : CEP %.3f m, R95 %.3f m, max %.3f m'
              % (s['radial_cep'], s['radial_r95'], s['radial_max']))
        print('  decorrelation     : %.0f s -> only ~%.0f independent samples'
              % (s['tau_s'], s['n_eff']))
        if s['n_eff'] < 10:
            print('    NOTE: under 10 independent samples. This sigma is an '
                  'impression, not a measurement -- soak for longer.')

    measured = float(np.median([s['sigma_axis'] for s in sessions]))
    worst_claim = max(s['reported_sigma_max'] for s in sessions)
    median_claim = float(np.median([s['reported_sigma_median']
                                    for s in sessions]))

    print()
    print('-' * 72)
    print('1. Will the stack drive here?')
    print('-' * 72)
    print('  confidence_gate passes a fix when the receiver reports')
    print('  sqrt(max(cov[0], cov[4])) <= %.2f m.' % GATE_SIGMA_M)
    print('  Receiver reported %.3f m median, %.3f m worst.'
          % (median_claim, worst_claim))
    if worst_claim <= GATE_SIGMA_M:
        print('  -> PASSES throughout. The robot will drive.')
    elif median_claim <= GATE_SIGMA_M:
        print('  -> MARGINAL. The median passes but the worst case does not, '
              'so the robot will stop intermittently.')
    else:
        print('  -> BLOCKED. The robot will not move at this site.')

    print()
    print('-' * 72)
    print('2. Is the receiver telling the truth about its own sigma?')
    print('-' * 72)
    ratio = measured / median_claim if median_claim > 0 else float('inf')
    print('  claims %.3f m, scatters %.3f m within a session -> ratio %.2f'
          % (median_claim, measured, ratio))
    if ratio > 2.0:
        print('  -> UNDER-REPORTING. It scatters more than it claims, so the')
        print('     gate is passing fixes it should be stopping. Treat the')
        print('     %.2f m threshold as effectively looser than it reads.'
              % GATE_SIGMA_M)
    elif ratio < 0.5:
        print('  -> CONSERVATIVE. It claims more error than it shows: safe, '
              'but it will stop the robot earlier than necessary.')
    else:
        print('  -> Consistent. The number the gate tests means what it says.')
    print('  (Within-session only. It cannot see a slowly-varying bias --')
    print('   that is what multiple sessions are for.)')

    print()
    print('-' * 72)
    print('3. What does a "%.0f-%.0f cm" provider spec mean?'
          % (spec_lo * 100, spec_hi * 100))
    print('-' * 72)
    rows = spec_consistency(measured, spec_lo, spec_hi)
    for row in rows:
        print('  %-14s would imply a spec of %.3f m   %s'
              % (row['reading'], row['implied_spec_m'],
                 'CONSISTENT' if row['consistent'] else '-'))
    fits = [r['reading'] for r in rows if r['consistent']]
    print()
    if not fits:
        print('  None fit. Either this site is better than the spec (likely '
              'close to the base station), or the quoted range is accuracy '
              'against truth while this is session precision.')
    else:
        print('  Consistent with: %s' % ', '.join(fits))
    print('  CAUTION: 1-sigma and CEP differ by only 1.18x and cannot be')
    print('  separated by this test. 1-sigma vs 95% differ by 2.4x and can.')

    print()
    print('-' * 72)
    print('4. Accuracy, not just precision')
    print('-' * 72)
    if truth is not None:
        offsets = [math.hypot(s['mean_enu'][0] - truth[0],
                              s['mean_enu'][1] - truth[1]) for s in sessions]
        print('  Against the surveyed mark: %s m'
              % ', '.join('%.3f' % o for o in offsets))
        print('  This is true horizontal accuracy.')
    elif len(sessions) < 2:
        print('  UNKNOWN, and this is the important gap. One session measures')
        print('  scatter about its own mean, so a slowly-varying bias -- for')
        print('  RTK, several cm -- is invisible to it.')
        print()
        print('  Occupy the same physical mark on another day at a different')
        print('  time and pass both bags. The spread of the session means is')
        print('  exactly the part a single session cannot show you.')
    else:
        means = np.array([s['mean_enu'][:2] for s in sessions])
        spread = means - means.mean(axis=0)
        between = float(np.hypot(np.std(spread[:, 0]), np.std(spread[:, 1])))
        print('  Session means scatter by %.3f m across %d sessions.'
              % (between, len(sessions)))
        print('  That includes the slowly-varying error a single session')
        print('  hides, so it is the better estimate of real accuracy.')
        if between > measured * 1.5:
            print('  -> Between-session error EXCEEDS within-session scatter,')
            print('     the normal RTK signature: the honest number is')
            print('     %.3f m, not %.3f m.' % (between, measured))
        if between > GATE_SIGMA_M:
            print('  -> It also exceeds the %.2f m gate, so position can be '
                  'off by more than the gate implies even while passing.'
                  % GATE_SIGMA_M)
    print()


# --------------------------------------------------------------------------

def self_test() -> int:
    """Verify the estimators against data with known properties."""
    print('Self-test: recovering known values from synthetic data')
    rng = np.random.default_rng(7)
    failures = []

    n, dt, true_sigma = 20000, 0.2, 0.031
    t = np.arange(n) * dt

    # 1. White noise of known sigma.
    e = rng.normal(0, true_sigma, n)
    nn = rng.normal(0, true_sigma, n)
    s = analyse_session(e, nn, np.zeros(n), t, np.full(n, true_sigma), 'white')
    print('  white noise sigma %.4f -> measured %.4f (worse axis)'
          % (true_sigma, s['sigma_axis']))
    if abs(s['sigma_axis'] - true_sigma) > 0.1 * true_sigma:
        failures.append('white-noise sigma off by more than 10%')
    print('  white noise tau %.2f s (expect ~%.2f)' % (s['tau_s'], dt))
    if s['tau_s'] > 10 * dt:
        failures.append('tau inflated on uncorrelated data')

    # 2. Correlated noise: tau must find the correlation time and n_eff must
    #    collapse. The whole "3000 samples is not 3000 samples" caveat rests
    #    on this working.
    tau_true = 60.0
    alpha = math.exp(-dt / tau_true)
    corr = np.zeros(n)
    for i in range(1, n):
        corr[i] = alpha * corr[i - 1] + rng.normal(0, 1.0)
    corr *= true_sigma / np.std(corr)
    s2 = analyse_session(corr, corr * 0.5, np.zeros(n), t,
                         np.full(n, true_sigma), 'correlated')
    print('  correlated tau %.0f s (true %.0f), n_eff %.0f of %d samples'
          % (s2['tau_s'], tau_true, s2['n_eff'], n))
    if not (0.3 * tau_true < s2['tau_s'] < 3.0 * tau_true):
        failures.append('tau not within 3x of the true correlation time')
    if s2['n_eff'] > n / 50:
        failures.append('n_eff did not collapse on correlated data')

    # 3. Spec classification: a 2 cm cloud reads as a 3-7 cm spec under R95
    #    (2.45x), not under 1-sigma.
    rows = {r['reading']: r for r in spec_consistency(0.020, 0.03, 0.07)}
    print('  sigma 2 cm vs a 3-7 cm spec: 1-sigma %s, R95 %s'
          % ('fits' if rows['1-sigma']['consistent'] else 'no',
             'fits' if rows['R95']['consistent'] else 'no'))
    if rows['1-sigma']['consistent'] or not rows['R95']['consistent']:
        failures.append('spec_consistency mis-classified a 2 cm cloud')

    # 4. A 5 cm cloud should read as 1-sigma, and NOT as 95%.
    rows5 = {r['reading']: r for r in spec_consistency(0.050, 0.03, 0.07)}
    print('  sigma 5 cm vs a 3-7 cm spec: 1-sigma %s, R95 %s'
          % ('fits' if rows5['1-sigma']['consistent'] else 'no',
             'fits' if rows5['R95']['consistent'] else 'no'))
    if not rows5['1-sigma']['consistent'] or rows5['R95']['consistent']:
        failures.append('spec_consistency mis-classified a 5 cm cloud')

    # 5. ENU: a known northward step must come back as northing.
    lat0, lon0, d_north = 47.5431, -121.8805, 10.0
    e2, n2, _ = geodetic_to_enu(
        np.array([lat0, lat0 + d_north / 111320.0]), np.array([lon0, lon0]),
        np.array([0.0, 0.0]), (lat0, lon0, 0.0))
    print('  ENU: 10 m north -> east %.3f, north %.3f' % (e2[1], n2[1]))
    if abs(n2[1] - d_north) > 0.05 or abs(e2[1]) > 0.05:
        failures.append('ENU conversion wrong')

    # 6. Confidence interval must widen as n_eff shrinks.
    wide = sigma_confidence(0.03, 5)
    narrow = sigma_confidence(0.03, 500)
    print('  CI width at n_eff=5: %.4f, at n_eff=500: %.4f'
          % (wide[1] - wide[0], narrow[1] - narrow[0]))
    if (wide[1] - wide[0]) <= (narrow[1] - narrow[0]):
        failures.append('confidence interval does not widen with fewer '
                        'samples')

    print()
    if failures:
        print('SELF-TEST FAILED')
        for f in failures:
            print('  - %s' % f)
        return 1
    print('SELF-TEST PASSED -- the estimators recover known inputs')
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--bag', action='append', default=[],
                        help='Bag of a static soak. Repeat for each session.')
    parser.add_argument('--topic', default='/um982_driver/fix',
                        help='Use the RAW driver fix, not /gnss/fix_gated: '
                             'the gate inflates covariance and would corrupt '
                             'the very number being measured.')
    parser.add_argument('--spec-low', type=float, default=0.03)
    parser.add_argument('--spec-high', type=float, default=0.07)
    parser.add_argument('--truth', nargs=2, type=float,
                        metavar=('EAST', 'NORTH'),
                        help='Surveyed position of the mark in the ENU frame '
                             'of the first session. Turns precision into '
                             'accuracy.')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()
    if not args.bag:
        parser.error('need --bag (or --self-test)')

    sessions, metas, datum = [], [], None
    for path in args.bag:
        lat, lon, alt, times, sigma, meta = read_bag(path, args.topic)
        if datum is None:
            datum = (float(lat[0]), float(lon[0]), float(alt[0]))
        # Every session goes into the FIRST session's ENU frame, so the
        # session means are directly comparable instead of each sitting at
        # its own origin.
        east, north, up = geodetic_to_enu(lat, lon, alt, datum)
        sessions.append(analyse_session(east, north, up, times, sigma,
                                        os.path.basename(path.rstrip('/'))))
        metas.append(meta)

    report(sessions, metas, args.spec_low, args.spec_high,
           tuple(args.truth) if args.truth else None)
    return 0


if __name__ == '__main__':
    sys.exit(main())
