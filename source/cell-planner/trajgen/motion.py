"""Geometric paths and their time parameterisation.

* ``SCurve``: rest-to-rest unit-distance profile, trapezoidal velocity (V, A) smoothed by a moving average of width
  ``tj`` chosen from the jerk cap (= jerk-limited "double-S": peak velocity <= V, acceleration <= A, jerk <= J; the duration grows by tj).
  Closed form via the antiderivative of the trapezoid's position.
* ``Segment``: a piecewise-linear joint path (knots) with per-joint speed/acceleration caps, timed by one SCurve on
  its path parameter. The parameter is the knot-to-knot max joint step normalised by the caps, so dq/ds is
  constant on every piece and the caps hold on every piece: |dq_j/dt| = |dq_j/ds| * ds/dt <= cap_j. (The small
  curvature term at the knots of a Cartesian line is checked afterwards by finite differences.) Its duration is
  rounded UP to a multiple of the output sample interval, so every segment starts and ends on a sample, at rest.
* ``plan_transit``: collision-checked joint-space path q0 -> q1: direct line, raised mid via, HOME via, then a
  small RRT-Connect with shortcutting (all edges checked with ``SceneModel.segment_ok``). When the electrode
  off-weld rule alone blocks the direct line, two gentler mid vias (a quarter / half of the way to the raised one) come
  before the raised one.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


class SCurve:
    """Rest-to-rest profile over unit distance with velocity <= V, acceleration <= A and jerk <= J.

    Trapezoidal velocity (V, A) smoothed by a moving average of width tj: the acceleration of the trapezoid takes
    the values +A / 0 / -A, so the smoothed jerk is (a(t) - a(t - tj)) / tj <= dA / tj, with dA = A if the cruise
    phase lasts at least tj and dA = 2A otherwise (triangular or short cruise). tj is chosen so that dA / tj = J."""

    def __init__(self, V: float, A: float, J: float | None = None):
        V, A = float(V), float(A)
        if not (V > 0 and A > 0):
            raise ValueError("V and A must be > 0")
        if V * V / A >= 1.0:                      # triangular: never reaches V
            ta = np.sqrt(1.0 / A)
            T = 2 * ta
            V = A * ta
        else:
            ta = V / A
            T = 1.0 / V + ta
        tj = 0.0
        if J is not None and np.isfinite(J) and J > 0:
            tj = A / J
            if T - 2 * ta < tj:
                tj = 2 * A / J
        self.V, self.A, self.ta, self.Tt, self.tj = V, A, ta, T, tj
        self.T = T + self.tj

    def _s_trap(self, t):
        A, V, ta, T = self.A, self.V, self.ta, self.Tt
        t = np.asarray(t, float)
        return np.where(t <= 0, 0.0, np.where(t <= ta, 0.5 * A * t ** 2, np.where(
            t <= T - ta, 0.5 * A * ta ** 2 + V * (t - ta), np.where(t <= T, 1.0 - 0.5 * A * (T - t) ** 2, 1.0))))

    def _F(self, t):
        """Antiderivative of _s_trap with F(0) = 0."""
        A, V, ta, T = self.A, self.V, self.ta, self.Tt
        t = np.asarray(t, float)
        F1 = A * ta ** 3 / 6.0
        F2 = lambda x: F1 + 0.5 * A * ta ** 2 * (x - ta) + 0.5 * V * (x - ta) ** 2
        Fm = F2(T - ta)
        F3 = lambda x: Fm + (x - (T - ta)) - (A / 6.0) * (ta ** 3 - (T - x) ** 3)
        FT = F3(T)
        return np.where(t <= 0, 0.0, np.where(t <= ta, A * t ** 3 / 6.0, np.where(
            t <= T - ta, F2(t), np.where(t <= T, F3(t), FT + (t - T)))))

    def s(self, t):
        t = np.asarray(t, float)
        if self.tj <= 1e-9:
            return np.clip(self._s_trap(t), 0.0, 1.0)
        return np.clip((self._F(t) - self._F(t - self.tj)) / self.tj, 0.0, 1.0)


@dataclass
class Segment:
    phase: str                 # "retract" | "transit" | "approach" | "home" | "wait"
    knots: np.ndarray          # (m, 6) joint path (piecewise linear), m >= 1
    v_cap: np.ndarray          # (6,) rad/s
    a_cap: np.ndarray          # (6,) rad/s^2
    j_cap: np.ndarray | None = None   # (6,) rad/s^3 (None = no jerk cap)
    dt: float = 0.05
    wait_s: float = 0.0        # phase "wait": stand still this long (knots = [q])
    info: dict = field(default_factory=dict)

    def __post_init__(self):
        K = np.asarray(self.knots, float).reshape(-1, 6)
        self.knots = K
        if self.phase == "wait" or len(K) < 2 or np.abs(K[-1] - K[0]).max() + np.abs(np.diff(K, axis=0)).sum() < 1e-10:
            self.n = int(round(self.wait_s / self.dt)) if self.phase == "wait" else 0
            self.T = self.n * self.dt
            self.prof, self.u = None, None
            return
        dQ = np.diff(K, axis=0)
        du = np.abs(dQ / self.v_cap).max(1)                 # time-like piece lengths at full cap
        keep = du > 1e-12
        if not keep.all():
            K = np.vstack([K[:1], K[1:][keep]])
            dQ, du = np.diff(K, axis=0), du[keep]
            self.knots = K
        L = float(du.sum())
        self.u = np.concatenate([[0.0], np.cumsum(du)]) / L
        dqds = dQ / (du / L)[:, None]                         # (pieces, 6) dq/ds, s in [0, 1]
        g = np.abs(dqds)
        V = float(np.min(self.v_cap / np.maximum(g.max(0), 1e-12)))
        A = float(np.min(self.a_cap / np.maximum(g.max(0), 1e-12)))
        J = None if self.j_cap is None else float(np.min(self.j_cap / np.maximum(g.max(0), 1e-12)))
        self._time(V, A, J)
        if len(K) > 2:
            # curved paths (Cartesian strokes): the joint velocity turns at every knot, which the per-piece caps do
            # not see. Check the sampled motion (20 Hz differences, from rest to rest) and slow the profile down
            # until it holds; segments that already hold are unchanged.
            for _ in range(12):
                X = self.samples()
                X = np.vstack([X[:1], X, X[-1:]])
                v = np.diff(X, axis=0) / self.dt
                a = np.diff(v, axis=0) / self.dt
                f = max(float((np.abs(a) / self.a_cap).max()), float((np.abs(v) / self.v_cap).max()))
                if f <= 1.0 + 1e-6:          # round-3 finding: 1e-3 let the end of a descent reach 1.0017 x a_cap
                    break
                V, A = V / f, A / f
                J = None if J is None else J / f
                self._time(V, A, J)
                self.info["slowed_for_knots"] = round(f, 4)

    def _time(self, V, A, J):
        self.prof = SCurve(V, A, J)
        self.n = max(1, int(np.ceil(self.prof.T / self.dt - 1e-9)))
        self.T = self.n * self.dt
        self._k = self.prof.T / self.T                        # time stretch onto the sample grid (<= 1)

    @property
    def q0(self):
        return self.knots[0]

    @property
    def q1(self):
        return self.knots[-1]

    def sample(self, t):
        """q at relative times t (clipped to [0, T])."""
        t = np.clip(np.asarray(t, float), 0.0, self.T)
        if self.prof is None:
            return np.repeat(self.knots[-1:], t.size, 0).reshape(t.shape + (6,))
        s = self.prof.s(t * self._k)
        K = self.knots
        k = np.clip(np.searchsorted(self.u, s, side="right") - 1, 0, len(K) - 2)
        f = (s - self.u[k]) / np.maximum(self.u[k + 1] - self.u[k], 1e-15)
        return K[k] + f[..., None] * (K[k + 1] - K[k])

    def samples(self):
        """(n+1, 6) samples at 0, dt, ..., T (the first one is the segment start)."""
        return self.sample(np.arange(self.n + 1) * self.dt)


def concat_segments(segs: list[Segment], dt: float):
    """Join segments into one sample array: (q (N,6), phase (N,) str, seg_index (N,)). Sample 0 is the start;
    each later sample carries the label of the segment it ends."""
    q, ph, si = [], [], []
    for i, s in enumerate(segs):
        X = s.samples()
        if not q:
            q.append(X[:1]); ph.append(s.phase); si.append(i)
        if len(X) > 1:
            if np.abs(X[0] - q[-1][-1]).max() > 1e-6:
                raise ValueError(f"segment {i} ({s.phase}) does not start where the previous one ended")
            q.append(X[1:]); ph += [s.phase] * (len(X) - 1); si += [i] * (len(X) - 1)
    Q = np.concatenate(q) if q else np.zeros((0, 6))
    return Q, np.array(ph, dtype="<U16"), np.array(si, int)


# ------------------------------------------------------------------------------------------- transit planning
def plan_transit(model, rid, q0, q1, *, home=None, res: float = 0.02, rrt_iters: int = 2000, seed: int = 0):
    """Collision-checked joint path q0 -> q1 as a list of vias (incl. both ends), or None."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    if np.abs(q1 - q0).max() < 1e-9:
        return [q0]
    home = model.robots[rid].home if home is None else np.asarray(home, float)
    mid = 0.5 * (q0 + q1)
    raised = mid.copy()
    raised[1:3] = home[1:3]
    cands = [[q0, q1], [q0, raised, q1], [q0, home, q1]]
    for n, c in enumerate(cands):
        if all(model.segment_ok(rid, a, b, res) for a, b in zip(c[:-1], c[1:])):
            return [np.asarray(x, float) for x in c]
        if n == 0 and getattr(model, "electrode_on", False) and _blocked_by_electrode_only(model, rid, c, res):
            # electrode off-weld rule (round 2): a joint line between two poses near a cube top can sag the
            # electrode into it; only then, gentler lifts of the midpoint (a quarter / half of the way to the raised
            # via) come before the raised one (elsewhere the candidates and so the plans are those of the rule off)
            cands[1:1] = [[q0, mid + f * (raised - mid), q1] for f in (0.25, 0.5)]
    return rrt_connect(model, rid, q0, q1, home, res=res, iters=rrt_iters, seed=seed)


def _blocked_by_electrode_only(model, rid, vias, res) -> bool:
    """True when the via path fails segment_ok with the electrode off-weld rule and passes without it."""
    model.electrode_on = False
    try:
        return all(model.segment_ok(rid, a, b, res) for a, b in zip(vias[:-1], vias[1:]))
    finally:
        model.electrode_on = True


def rrt_connect(model, rid, q0, q1, home, *, res=0.02, iters=2000, step=0.3, seed=0):
    if not (model.static_ok(rid, q0) and model.static_ok(rid, q1)):
        return None
    r = model.robots[rid]
    rng = np.random.default_rng(seed)
    pts = np.stack([q0, q1, home])
    lo = np.maximum(r.q_lo, pts.min(0) - 1.0)
    hi = np.minimum(r.q_hi, pts.max(0) + 1.0)
    A, B, PA, PB = [q0], [q1], [-1], [-1]

    def extend(T, P, x):
        d = np.abs(np.array(T) - x).max(1)
        j = int(np.argmin(d))
        v = x - T[j]
        n = np.abs(v).max()
        y = x if n <= step else T[j] + v / n * step
        if model.segment_ok(rid, T[j], y, res):
            T.append(y); P.append(j)
            return len(T) - 1, np.abs(y - x).max() < 1e-9
        return None, False

    def path(T, P, k):
        out = []
        while k != -1:
            out.append(T[k]); k = P[k]
        return out[::-1]

    swapped = False
    for _ in range(iters):
        x = rng.uniform(lo, hi) if rng.uniform() > 0.1 else (B[0])
        ka, _ = extend(A, PA, x)
        if ka is not None:
            while True:
                kb, reached = extend(B, PB, A[ka])
                if kb is None:
                    break
                if reached:
                    pa, pb = path(A, PA, ka), path(B, PB, kb)
                    full = pa + pb[::-1][1:]
                    if swapped:
                        full = full[::-1]
                    return shortcut(model, rid, full, res)
        A, B, PA, PB = B, A, PB, PA
        swapped = not swapped
    return None


def shortcut(model, rid, vias, res=0.02, rounds=60, seed=1):
    rng = np.random.default_rng(seed)
    v = [np.asarray(x, float) for x in vias]
    for _ in range(rounds):
        if len(v) <= 2:
            break
        i, j = sorted(rng.choice(len(v), 2, replace=False))
        if j - i >= 2 and model.segment_ok(rid, v[i], v[j], res):
            v = v[:i + 1] + v[j:]
    return v


def line_time(dq, v_cap, a_cap, j_cap=None) -> float:
    """Duration of a straight rest-to-rest joint move dq under the caps (no grid rounding)."""
    g = np.abs(np.asarray(dq, float))
    if g.max() < 1e-12:
        return 0.0
    gg = np.maximum(g, 1e-12)
    J = None if j_cap is None else float(np.min(j_cap / gg))
    return SCurve(float(np.min(v_cap / gg)), float(np.min(a_cap / gg)), J).T
