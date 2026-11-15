"""Exact controller polynomials, analytic limit checks and reproducible retiming."""
from __future__ import annotations

import math
import numpy as np
from numpy.polynomial import Polynomial


class Trajectory:
    def __init__(self, points):
        if len(points) < 2:
            raise ValueError("a motion needs at least two points")
        self.points = points
        self.times = np.array([p["time_s"] for p in points], dtype=float)
        if self.times[0] != 0 or not np.all(np.diff(self.times) > 1e-8):
            raise ValueError("trajectory times must increase strictly from zero")
        self.n = len(points[0]["positions"])
        self.coefficients = []
        for a, b, dt in zip(points, points[1:], np.diff(self.times)):
            q0, q1 = np.array(a["positions"]), np.array(b["positions"])
            v0, v1 = np.array(a["velocities"]), np.array(b["velocities"])
            a0, a1 = np.array(a["accelerations"]), np.array(b["accelerations"])
            if not all(len(x) == self.n for x in (q0, q1, v0, v1, a0, a1)):
                raise ValueError("joint dimension mismatch")
            # Unit-time coefficients, independently derived from six boundary conditions.
            c0, c1, c2 = q0, v0 * dt, a0 * dt * dt / 2
            residual = np.stack([q1 - c0 - c1 - c2,
                                 v1 * dt - c1 - 2*c2,
                                 a1 * dt*dt - 2*c2])
            tail = np.linalg.solve(np.array([[1, 1, 1], [3, 4, 5], [6, 12, 20]]), residual)
            self.coefficients.append(np.stack([c0, c1, c2, *tail]))
        self.coefficients = np.array(self.coefficients)
        if not np.all(np.isfinite(self.coefficients)):
            raise ValueError("non-finite trajectory")
        self.duration = float(self.times[-1])

    def sample(self, t, derivative=0):
        if t < 0:
            return np.array(self.points[0]["positions"]) if derivative == 0 else np.zeros(self.n)
        if t >= self.duration:
            return np.array(self.points[-1]["positions"]) if derivative == 0 else np.zeros(self.n)
        k = min(int(np.searchsorted(self.times, t, side="right")) - 1, len(self.times) - 2)
        dt = self.times[k+1] - self.times[k]
        x = (t - self.times[k]) / dt
        c = self.coefficients[k]
        for _ in range(derivative):
            c = c[1:] * np.arange(1, len(c))[:, None] / dt
        return np.array([np.polynomial.polynomial.polyval(x, c[:, j]) for j in range(self.n)])

    def segment_extrema(self, derivative):
        rows = []
        for coeff, dt in zip(self.coefficients, np.diff(self.times)):
            lower, upper = [], []
            for j in range(self.n):
                p = Polynomial(coeff[:, j]).deriv(derivative) / dt**derivative
                roots = p.deriv().roots()
                x = [0., 1.] + [float(r.real) for r in roots
                                if abs(r.imag) < 1e-8 and 0 < r.real < 1]
                values = p(np.asarray(x))
                lower.append(min(values))
                upper.append(max(values))
            rows.append((lower, upper))
        return np.asarray(rows)

    def extrema(self, derivative):
        rows = self.segment_extrema(derivative)
        return rows[:, 0].min(axis=0), rows[:, 1].max(axis=0)

    def _old_extrema(self, derivative):
        lo, hi = np.full(self.n, np.inf), np.full(self.n, -np.inf)
        for coeff, dt in zip(self.coefficients, np.diff(self.times)):
            for j in range(self.n):
                p = Polynomial(coeff[:, j]).deriv(derivative) / dt**derivative
                roots = p.deriv().roots()
                x = [0., 1.] + [float(r.real) for r in roots
                                if abs(r.imag) < 1e-8 and 0 < r.real < 1]
                values = p(np.asarray(x))
                lo[j], hi[j] = min(lo[j], min(values)), max(hi[j], max(values))
        return lo, hi

    def peaks(self, derivative):
        lo, hi = self.extrema(derivative)
        return np.maximum(abs(lo), abs(hi))

    def reversed(self):
        return [{**p, "time_s": self.duration - p["time_s"],
                 "velocities": [-x for x in p["velocities"]]}
                for p in reversed(self.points)]


def retime_native(points, config, ceiling):
    """One uniform stretch preserves the controller curve and all phase joins."""
    from utils.fjt_trajectory import with_derivatives
    points = with_derivatives(points)
    # The real robot driver's exact goal builder clamps the final acceleration
    # to zero. Certify that actual payload, including the last polynomial.
    points[-1]["accelerations"] = [0.] * len(points[-1]["positions"])
    points[0]["accelerations"] = [0.] * len(points[0]["positions"])
    path = Trajectory(points)
    velocity = np.array(config["joint_velocity_limits"]) * ceiling
    acceleration = np.array(config["joint_acceleration_limits"]) * ceiling**2
    factor = max(1., float(np.max(path.peaks(1) / velocity)),
                 float(np.sqrt(np.max(path.peaks(2) / acceleration))))
    if config.get("joint_jerk_limits"):
        factor = max(factor, float(np.cbrt(np.max(path.peaks(3) / config["joint_jerk_limits"]))))
    factor *= 1.002
    return [{**p, "time_s": p["time_s"] * factor,
             "velocities": [v / factor for v in p["velocities"]],
             "accelerations": [a / factor**2 for a in p["accelerations"]]}
            for p in points]


def limit_check(path: Trajectory, config, ceiling=1.):
    lo, hi = path.extrema(0)
    if np.any(lo < np.array(config["joint_limits_lower"]) - 1e-8) or np.any(
            hi > np.array(config["joint_limits_upper"]) + 1e-8):
        return {"ok": False, "reason": "interpolant_joint_limits"}
    peaks = {d: path.peaks(d) for d in (1, 2, 3)}
    for order, field, scale in [(1, "joint_velocity_limits", ceiling),
                                (2, "joint_acceleration_limits", ceiling**2),
                                (3, "joint_jerk_limits", 1.)]:
        if config.get(field) and np.any(peaks[order] > np.array(config[field]) * scale + 1e-7):
            return {"ok": False, "reason": field, "peaks": peaks[order].tolist()}
    for endpoint in (path.points[0], path.points[-1]):
        if max(map(abs, endpoint["velocities"])) > 1e-7:
            return {"ok": False, "reason": "nonrest_boundary"}
    return {"ok": True, "velocity": peaks[1].tolist(), "acceleration": peaks[2].tolist(),
            "jerk": peaks[3].tolist(), "jerk_configured": bool(config.get("joint_jerk_limits"))}
