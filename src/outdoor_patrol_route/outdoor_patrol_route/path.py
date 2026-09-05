# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Smoothed, arc-length-parameterised path through the recorded stations.

Two jobs:

1. **Smooth.** Recorded stations are 1 m apart and carry 2--3 cm of RTK noise,
   which is several degrees of tangent jitter if you differentiate the raw
   polyline -- and pure pursuit steers on the tangent, so that jitter comes
   straight out at the wheels. A centripetal Catmull-Rom spline through the
   stations removes it. Centripetal (alpha = 0.5) rather than uniform because
   uniform Catmull-Rom can loop back on itself where sample spacing is uneven,
   which is exactly what happens when the yaw trigger fires on a corner.

2. **Answer the two questions the follower asks.** "Where is station s, offset
   d sideways?" (the look-ahead point) and "what station is this point at, and
   how far off the path is it?" (cross-track error, and where a LiDAR return
   falls in the corridor).

A measured correction to the design in issue #8: a Catmull-Rom spline on its
own does **not** remove the noise, because it interpolates -- it passes exactly
through every noisy station, so the jitter survives into the tangent. Scored
against the sim road with 2 cm of fix noise on 1 m stations, spline-only gives
1.70 deg RMS tangent error. A quadratic Savitzky-Golay pre-pass over the
stations, then the spline, gives 0.94 deg. The window has to stay at or below
the corner radius: at 7 and 9 stations it starts cutting the 5 m corners and
the error climbs again (1.00 and 1.31 deg). Hence `smooth_window` defaults
to 5.

Frame is `map`, metres, ENU. Lateral offsets are REP-103 signed: **+ is left,
- is right**, so the right shoulder is at negative offsets.
"""

from __future__ import annotations

import math

import numpy as np

# Spacing of the internal resampled polyline. Everything downstream indexes
# into it, so this sets the quantisation of both station and look-ahead.
DEFAULT_STEP_M = 0.05

# Control points closer than this are treated as duplicates. A duplicate makes
# the centripetal knot spacing zero and the spline blows up.
_MIN_CONTROL_SPACING_M = 1e-3

# Stations in the Savitzky-Golay pre-smoothing window. Must stay at or below
# the tightest corner radius in stations, or the fit cuts the corner. See the
# module docstring for the measured trade-off.
DEFAULT_SMOOTH_WINDOW = 5

# Arc length over which curvature is measured. Long enough to ride over the
# spline ripple that uneven station spacing produces, and close to the scale
# that pure pursuit and the offset lanes actually respond to. See
# Path._curvatures.
CURVATURE_BASELINE_M = 0.5


def savitzky_golay(points: np.ndarray, window: int, order: int = 2,
                   loop: bool = False) -> np.ndarray:
    """Fit a local polynomial through each station and keep its centre value.

    Quadratic by default, because a quadratic reproduces a circular arc almost
    exactly over a short window -- so this attenuates noise without flattening
    the corners the way a moving average does.

    A window of 3 with order 2 is the identity (three points define the
    quadratic), so anything below 5 is simply "off".

    On an OPEN path the ends get the usual Savitzky-Golay treatment: fit one
    polynomial to the first (last) full window and evaluate it at each edge
    station. Clamping the index instead -- repeating the endpoint to fill the
    window -- weights that endpoint several times and drags the first and last
    stations inward, which on a straight test line moves them by 0.17 m.
    """
    window = int(window)
    points = np.asarray(points, dtype=float)
    if window < 5 or window % 2 == 0 or len(points) < window:
        return points

    half = window // 2
    offsets = np.arange(-half, half + 1)
    basis = np.vander(offsets.astype(float), order + 1, increasing=True)
    # Row 0 of the pseudo-inverse evaluates the fitted polynomial at z = 0;
    # the whole matrix gives its coefficients, which the edges need.
    pseudo = np.linalg.pinv(basis)

    n = len(points)
    if loop:
        index = (np.arange(n)[:, None] + offsets[None, :]) % n
        return np.einsum('j,ijk->ik', pseudo[0], points[index])

    out = np.empty_like(points)
    index = np.arange(half, n - half)[:, None] + offsets[None, :]
    out[half:n - half] = np.einsum('j,ijk->ik', pseudo[0], points[index])

    for block, centre, targets in (
            (points[:window], half, range(0, half)),
            (points[n - window:], n - 1 - half, range(n - half, n))):
        coefficients = pseudo @ block
        for i in targets:
            t = float(i - centre)
            out[i] = sum(coefficients[k] * t ** k for k in range(order + 1))
    return out


def _dedupe(points: np.ndarray, loop: bool) -> np.ndarray:
    keep = [points[0]]
    for p in points[1:]:
        if np.hypot(*(p - keep[-1])) >= _MIN_CONTROL_SPACING_M:
            keep.append(p)
    out = np.asarray(keep, dtype=float)
    if loop and len(out) > 1 and np.hypot(*(out[-1] - out[0])) < \
            _MIN_CONTROL_SPACING_M:
        # A closed loop should not repeat its first point as its last; the
        # wraparound is implicit.
        out = out[:-1]
    return out


def _catmull_rom(points: np.ndarray, loop: bool,
                 per_segment: int = 24) -> np.ndarray:
    """Dense polyline through `points` (centripetal Catmull-Rom)."""
    n = len(points)
    if n < 2:
        return points.copy()
    if n < 4 and not loop:
        return points.copy()

    alpha = 0.5
    dense = []
    last = n if loop else n - 1
    for i in range(last):
        if loop:
            p = [points[(i - 1) % n], points[i], points[(i + 1) % n],
                 points[(i + 2) % n]]
        else:
            p = [points[max(i - 1, 0)], points[i], points[min(i + 1, n - 1)],
                 points[min(i + 2, n - 1)]]
        p0, p1, p2, p3 = p

        # Centripetal knot sequence.
        t0 = 0.0
        t1 = t0 + max(float(np.linalg.norm(p1 - p0)), 1e-9) ** alpha
        t2 = t1 + max(float(np.linalg.norm(p2 - p1)), 1e-9) ** alpha
        t3 = t2 + max(float(np.linalg.norm(p3 - p2)), 1e-9) ** alpha

        ts = np.linspace(t1, t2, per_segment, endpoint=False)
        for t in ts:
            # Barry-Goldman pyramidal form.
            a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
            a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
            b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
            dense.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)

    if not loop:
        dense.append(points[-1])
    return np.asarray(dense, dtype=float)


class Path:
    """Arc-length-parameterised, smoothed path with a signed lateral axis."""

    def __init__(self, control_points, loop: bool = False,
                 step_m: float = DEFAULT_STEP_M,
                 smooth_window: int = DEFAULT_SMOOTH_WINDOW):
        pts = _dedupe(np.asarray(control_points, dtype=float), loop)
        if len(pts) < 2:
            raise ValueError('a path needs at least 2 distinct points')

        self.loop = bool(loop)
        self.step = float(step_m)
        self.smooth_window = int(smooth_window)

        pts = savitzky_golay(pts, self.smooth_window, loop=self.loop)
        dense = _catmull_rom(pts, self.loop)
        self.xy, self.length = self._resample(dense, self.step, self.loop)
        self.tangent = self._tangents(self.xy, self.loop)
        # Left normal, REP-103: rotate the tangent +90 degrees.
        self.normal = np.stack(
            [-self.tangent[:, 1], self.tangent[:, 0]], axis=1)
        self.yaw = np.arctan2(self.tangent[:, 1], self.tangent[:, 0])
        self.curvature = self._curvatures(self.yaw, self.step, self.loop)

    # -- construction helpers ---------------------------------------------

    @staticmethod
    def _resample(dense: np.ndarray, step: float, loop: bool):
        seg = np.diff(dense, axis=0)
        seg_len = np.hypot(seg[:, 0], seg[:, 1])
        if loop:
            closing = dense[0] - dense[-1]
            seg_len = np.append(seg_len, float(np.hypot(*closing)))
            dense = np.vstack([dense, dense[0]])
        cumulative = np.concatenate([[0.0], np.cumsum(seg_len)])
        total = float(cumulative[-1])

        count = max(2, int(round(total / step)))
        # Land exactly on `total` for an open path; stop one step short for a
        # loop so the wraparound is not a duplicate point.
        targets = (np.arange(count) * (total / count) if loop
                   else np.linspace(0.0, total, count + 1))
        x = np.interp(targets, cumulative, dense[:, 0])
        y = np.interp(targets, cumulative, dense[:, 1])
        return np.stack([x, y], axis=1), total

    @staticmethod
    def _tangents(xy: np.ndarray, loop: bool) -> np.ndarray:
        """Unit tangents by central difference on the resampled curve.

        The curve is already smooth (it is a spline sampled every few
        centimetres), so a central difference and the analytic derivative
        agree to well under the noise floor -- and the central difference
        cannot disagree with the arc-length parameterisation the rest of this
        class uses, which the analytic form can.
        """
        if loop:
            nxt = np.roll(xy, -1, axis=0)
            prv = np.roll(xy, 1, axis=0)
        else:
            nxt = np.vstack([xy[1:], xy[-1]])
            prv = np.vstack([xy[0], xy[:-1]])
        d = nxt - prv
        norm = np.hypot(d[:, 0], d[:, 1])
        norm[norm < 1e-12] = 1.0
        return d / norm[:, None]

    @staticmethod
    def _curvatures(yaw: np.ndarray, step: float, loop: bool,
                    baseline_m: float = CURVATURE_BASELINE_M) -> np.ndarray:
        """Signed curvature, dtheta/ds. Positive turns left (REP-103).

        Measured over a BASELINE of about half a metre rather than between
        adjacent 5 cm samples, for two reasons -- one numerical, one physical.

        Numerically, the recorder's two triggers produce wildly uneven station
        spacing: 1 m on a straight and 8.7 cm round a 1 m corner, a ratio of
        over 50:1 on a driveway square. Catmull-Rom interpolates every station
        exactly, so at those transitions the spline ripples. The ripple is
        sub-centimetre in position but curvature is a second derivative, and a
        point-to-point estimate turns it into spikes: measured on a realistic
        recording of a 1.0 m corner, adjacent-sample differencing reports
        0.24 m. Reading that as the corner radius would condemn a perfectly
        good route.

        Physically, half a metre is nearer the scale that matters anyway.
        Nothing downstream responds to curvature at 5 cm: pure pursuit steers
        at a look-ahead of a metre or more, and the offset lane this feeds is
        displaced by a similar amount. Both average over exactly the ripple
        that the short baseline mistakes for a corner.

        The heading is unwrapped before differencing. Without that, every
        pass through +/-pi reads as 2*pi/baseline of curvature -- 12 m^-1 on a
        half-metre baseline, which would swamp any real corner.
        """
        n = len(yaw)
        if n < 2:
            return np.zeros(n)

        half = max(1, int(round(0.5 * baseline_m / step)))
        span = 2.0 * half * step

        if loop:
            unwrapped = np.unwrap(yaw)
            # A closed loop accumulates a whole turn, so the seam carries real
            # signal; undo only the jump that the roll itself introduces.
            turns = round((unwrapped[-1] - unwrapped[0]) / (2.0 * math.pi))
            nxt = np.roll(unwrapped, -half)
            prv = np.roll(unwrapped, half)
            nxt[-half:] += turns * 2.0 * math.pi
            prv[:half] -= turns * 2.0 * math.pi
            return (nxt - prv) / span

        unwrapped = np.unwrap(yaw)
        if n <= 2 * half:
            # Too short for the baseline: fall back to the whole path.
            return np.full(n, (unwrapped[-1] - unwrapped[0])
                           / max((n - 1) * step, 1e-9))
        out = np.empty(n)
        out[half:n - half] = (unwrapped[2 * half:] - unwrapped[:-2 * half]) / span
        out[:half] = out[half]
        out[n - half:] = out[n - half - 1]
        return out

    # -- queries -----------------------------------------------------------

    def _index(self, s: float) -> int:
        i = int(round(s / self.step))
        if self.loop:
            return i % len(self.xy)
        return min(max(i, 0), len(self.xy) - 1)

    def wrap_s(self, s: float) -> float:
        """Clamp (open path) or wrap (loop) a station into range."""
        if self.loop:
            return s % self.length
        return min(max(s, 0.0), self.length)

    def pose_at(self, s: float):
        """(x, y, yaw) at station s."""
        i = self._index(self.wrap_s(s))
        return float(self.xy[i, 0]), float(self.xy[i, 1]), float(self.yaw[i])

    def offset_at(self, s: float, lateral: float):
        """Point `lateral` metres left of station s (- is right)."""
        i = self._index(self.wrap_s(s))
        p = self.xy[i] + lateral * self.normal[i]
        return float(p[0]), float(p[1])

    def offset_polyline(self, lateral: float) -> np.ndarray:
        """The whole path displaced sideways -- for corridor markers."""
        return self.xy + lateral * self.normal

    def offset_scale(self, lateral: float) -> np.ndarray:
        """Arc-length stretch of the offset curve at each station.

        Displacing by ``d`` along the left normal gives an offset curve whose
        arc length element is ``(1 - d*kappa) ds``. So this is 1 on a
        straight, above 1 on the outside of a bend and below 1 on the inside.

        At zero the offset curve has collapsed to a cusp, and below zero it
        has **inverted**: consecutive points run backwards along it. That is
        not a degraded lane, it is a lane pointing the wrong way, and
        :meth:`offset_at` will happily return points from it.
        """
        return 1.0 - float(lateral) * self.curvature

    def offset_folds(self, lateral: float):
        """Where, if anywhere, offsetting by `lateral` inverts the path.

        Returns ``(folds, worst_scale, station)``. Offsets on the OUTSIDE of
        every bend never fold, so for a one-sided retreat only that side
        needs checking.
        """
        scale = self.offset_scale(lateral)
        i = int(np.argmin(scale))
        return bool(scale[i] <= 0.0), float(scale[i]), float(i * self.step)

    def min_turn_radius(self, sign: float = 0.0):
        """Tightest turn radius, optionally only for turns to one side.

        ``sign`` > 0 considers only left turns, < 0 only right turns, 0 both.
        Returns ``(radius, station)``, radius ``inf`` for a path with no
        bend in that direction.
        """
        kappa = self.curvature
        if sign > 0:
            mask = kappa > 0
        elif sign < 0:
            mask = kappa < 0
        else:
            mask = np.ones(len(kappa), dtype=bool)
        if not np.any(mask):
            return float('inf'), 0.0
        magnitude = np.where(mask, np.abs(kappa), 0.0)
        i = int(np.argmax(magnitude))
        if magnitude[i] <= 1e-9:
            return float('inf'), float(i * self.step)
        return float(1.0 / magnitude[i]), float(i * self.step)

    def _window(self, s_lo: float, s_hi: float) -> np.ndarray:
        """Indices covering [s_lo, s_hi], wrapping if this is a loop."""
        n = len(self.xy)
        lo = int(math.floor(s_lo / self.step))
        hi = int(math.ceil(s_hi / self.step))
        idx = np.arange(lo, hi + 1)
        if self.loop:
            return idx % n
        return idx[(idx >= 0) & (idx < n)]

    def project(self, x: float, y: float, s_hint=None,
                window_m: float = 8.0):
        """Nearest station to (x, y), and the signed lateral offset there.

        `s_hint` restricts the search to +/- `window_m` around a previous
        station. Without it the whole path is searched, which on a loop can
        latch onto the wrong side of a hairpin -- so the follower always
        passes a hint after the first fix.
        """
        if s_hint is None:
            idx = np.arange(len(self.xy))
        else:
            idx = self._window(s_hint - window_m, s_hint + window_m)
            if len(idx) == 0:
                idx = np.arange(len(self.xy))

        q = np.array([x, y], dtype=float)
        deltas = self.xy[idx] - q
        best = int(np.argmin(deltas[:, 0] ** 2 + deltas[:, 1] ** 2))
        i = int(idx[best])

        # Refine to sub-step resolution along the local tangent.
        along = float(np.dot(q - self.xy[i], self.tangent[i]))
        along = max(-self.step, min(self.step, along))
        s = self.wrap_s(i * self.step + along)
        lateral = float(np.dot(q - self.xy[i], self.normal[i]))
        return s, lateral

    def stations_of(self, points: np.ndarray, s_lo: float, s_hi: float):
        """Vectorised (station, lateral) for many points over a window.

        Used to place LiDAR returns in the corridor: one call per control
        cycle for the whole forward window, rather than a projection per
        point.

        Returns (station, lateral, in_window) with `in_window` False for
        points whose nearest path index sits on the window edge -- those are
        outside the window rather than genuinely nearest to it.
        """
        idx = self._window(s_lo, s_hi)
        if len(points) == 0 or len(idx) == 0:
            empty = np.zeros(len(points))
            return empty, empty, np.zeros(len(points), dtype=bool)

        window_xy = self.xy[idx]
        # (M, N) squared distances.
        dx = points[:, 0][:, None] - window_xy[None, :, 0]
        dy = points[:, 1][:, None] - window_xy[None, :, 1]
        nearest = np.argmin(dx * dx + dy * dy, axis=1)

        path_idx = idx[nearest]
        rel = points - self.xy[path_idx]
        lateral = np.einsum('ij,ij->i', rel, self.normal[path_idx])
        along = np.einsum('ij,ij->i', rel, self.tangent[path_idx])
        station = np.array(
            [self.wrap_s(i * self.step) for i in path_idx]) + along

        interior = (nearest > 0) & (nearest < len(idx) - 1)
        return station, lateral, interior


def forward_gap(s_here: float, s_there: float, length: float,
                loop: bool) -> float:
    """Signed distance from s_here to s_there, forward around a loop."""
    d = s_there - s_here
    if loop:
        d %= length
        if d > length / 2.0:
            d -= length
    return d
