"""Smoothing post-processor: C2-continuous, jerk-limited trajectories through an exported plan.

Pipeline (``smooth_plan``):

1. **Rests.** Per robot, the executor-clock polyline (legs + standing still between them) is split into moving
   intervals between rests (holds, completion margins, waits, the stationary ends of every leg). Every rest pose
   is kept exactly: weld poses, cut poses and HOME are anchors.
2. **Geometric smoothing per arm (source clock).** Each moving interval [ta, tb] is fitted by a quintic B-spline
   (uniform knots, ``knot_dt``) with a third-difference penalty (P-spline). The end conditions
   q(ta) = qa, q'(ta) = q''(ta) = 0 (same at tb) are exact (the first / last three coefficients are fixed), so the
   arm leaves and reaches every rest with zero velocity and acceleration: the concatenation is C2 and the jerk is
   bounded. The penalty weight is the largest one (bisection on log10) for which the fit stays within
   ``tol_geo`` (joint-space Euclidean distance to the source polyline, rad) and ``tol_time`` (same-instant joint
   deviation, rad). Both arms keep the source clock, so their pairing in time changes only by the fit deviation.
3. **Time scaling (common clock).** Within each block where any arm moves (between instants where all arms
   stand still), one time map sigma(tau) for all arms: pointwise bounds on r = dsigma/dtau from the velocity,
   acceleration and jerk limits, forward / backward passes on u = r^2 for the acceleration (TOPP-style phase plane),
   then an erosion + smoothing kernel of width ``W`` (keeps u below its bound and makes it smooth, so the jerk is
   bounded), and a search over W and a limit scale kappa for the shortest block that passes a dense v / a / jerk
   check. ``r_cap`` = 1 (default, "preserve") never runs faster than the plan; a larger cap gives the
   time-optimal-ish variant. Rests between blocks keep their durations, so the executor's holds and margins stay.
   Holds of one arm inside a block are bounded below by ``hold_s`` through r <= L_hold / hold_s.
4. **Output.** Per leg, waypoints on a uniform grid of ``out_dt`` (default the KR C4 IPO cycle, 12 ms) with
   positions, velocities and accelerations; leg durations are rounded up to whole cycles (the arm is at rest at a
   leg end). ``tools.ctrl.export`` writes them as a umarell BT plan whose replay (player.bt_plan) is the truth that
   ``tools.ctrl.safety`` re-checks.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from scipy import sparse
from scipy.interpolate import BSpline
from scipy.ndimage import maximum_filter1d, minimum_filter1d
from scipy.sparse.linalg import spsolve

from tools.ctrl.limits import profile as _profile


@dataclass
class SmoothCfg:
    profile: str = "umarell"
    knot_dt: float = 0.05            # s, knot spacing of the quintic P-spline (source clock)
    fit_hz: float = 200.0            # source samples per second used in the fit
    tol_geo: float = 5e-4            # rad, max joint-space distance to the source polyline
    tol_time: float = 1e-2           # rad, max same-instant joint deviation
    r_cap: float = 1.0               # max clock rate vs the source (1 = never faster than the plan)
    ds: float = 0.002                # s, time-scaling grid (source clock)
    windows: tuple = (0.02, 0.05, 0.1, 0.2, 0.4)        # s, smoothing kernel half-widths
    kappas: tuple = (1.0, 0.95, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2)
    hold_s: float = 0.5
    out_dt: float = 0.012            # s, output waypoint interval
    eps_move: float = 1e-9           # rad, below this two waypoints are "the same pose"

    def to_dict(self):
        return asdict(self)


# ───────────────────────────────────────────── per-robot polyline and rests
def robot_polyline(plan, r, max_snap=1e-3):
    """Executor-clock polyline of robot index r: (t (M,), q (M, 6)), standing still between legs."""
    rid = plan.robot_ids[r]
    T, Q = [0.0], [plan.home[r].copy()]
    for leg in plan.robot_legs(rid):
        P = leg.positions.copy()
        if np.abs(P[0] - Q[-1]).max() <= max_snap:      # tiny start jumps (umarell rounding, <= 1.4e-4 rad):
            P[0] = Q[-1]                                 # the controller starts from the arm's actual pose
        T += [float(leg.times[0]), *map(float, leg.times)]
        Q += [Q[-1].copy(), *P]
    T.append(max(plan.t_end, T[-1]))
    Q.append(Q[-1].copy())
    T, Q = np.asarray(T), np.asarray(Q)
    keep = np.r_[True, (np.diff(T) > 1e-12) | (np.abs(np.diff(Q, axis=0)).max(1) > 1e-12)]
    T, Q = T[keep], Q[keep]
    jump = (np.diff(T) <= 1e-12) & (np.abs(np.diff(Q, axis=0)).max(1) > 1e-6)
    if jump.any():
        raise ValueError(f"{rid}: position jump at t = {T[1:][jump][:3]} (cannot be smoothed)")
    keep = np.r_[True, np.diff(T) > 1e-12]
    return T[keep], Q[keep]


def moving_intervals(T, Q, eps=1e-9):
    mov = np.abs(np.diff(Q, axis=0)).max(1) > eps
    out, i = [], 0
    n = len(mov)
    while i < n:
        if mov[i]:
            j = i
            while j + 1 < n and mov[j + 1]:
                j += 1
            out.append((i, j + 1))            # vertex indices: motion from T[i] to T[j + 1]
            i = j + 1
        else:
            i += 1
    return out


# ───────────────────────────────────────────── quintic P-spline fit with rest-to-rest ends
def _knots(ta, tb, h):
    n_int = max(1, int(np.ceil((tb - ta) / h - 1e-9)))
    inner = np.linspace(ta, tb, n_int + 1)
    return np.r_[[ta] * 5, inner, [tb] * 5]


def _poly_dist(tf, Xf, Tp, Qp, w=0.3):
    """Euclidean distance (rad) of each fitted point to the source polyline segments within +- w (source time)."""
    lo = np.clip(np.searchsorted(Tp, tf - w, "left") - 1, 0, len(Tp) - 2)
    hi = np.clip(np.searchsorted(Tp, tf + w, "right"), 1, len(Tp) - 1)
    best = np.full(len(tf), np.inf)
    K = int((hi - lo).max())
    for k in range(K):
        s = np.minimum(lo + k, len(Tp) - 2)
        valid = (lo + k) < hi
        A, B = Qp[s], Qp[s + 1]
        d = B - A
        L2 = (d * d).sum(1)
        x = np.where(L2 > 0, ((Xf - A) * d).sum(1) / np.where(L2 > 0, L2, 1), 0.0)
        P = A + np.clip(x, 0, 1)[:, None] * d
        dist = np.linalg.norm(Xf - P, axis=1)
        best = np.where(valid, np.minimum(best, dist), best)
    return best


def fit_interval(Tp, Qp, ta, tb, cfg: SmoothCfg, min_knot_dt=0.0125):
    """Quintic P-spline on [ta, tb] through the rests Q(ta), Q(tb). -> (BSpline (6 columns), info).
    If even the least-smoothed fit misses the tolerance, the knot spacing is halved (down to ``min_knot_dt``)."""
    h = cfg.knot_dt
    while True:
        sp, info = _fit_interval(Tp, Qp, ta, tb, cfg, h)
        info["knot_dt"] = h
        if info["tol_met"] or h / 2 < min_knot_dt - 1e-12:
            return sp, info
        h /= 2


def _fit_interval(Tp, Qp, ta, tb, cfg: SmoothCfg, h):
    qa = Qp[np.searchsorted(Tp, ta)]
    qb = Qp[np.searchsorted(Tp, tb)]
    kn = _knots(ta, tb, h)
    n = len(kn) - 6
    m = n - 6
    ns = max(int(np.ceil((tb - ta) * cfg.fit_hz)) + 1, 8)
    ts = np.linspace(ta, tb, ns)
    ts = np.unique(np.r_[ts, Tp[(Tp > ta) & (Tp < tb)]])
    Y = np.stack([np.interp(ts, Tp, Qp[:, j]) for j in range(6)], -1)
    cfix = np.zeros((n, 6))
    cfix[:3] = qa
    cfix[-3:] = qb
    if m <= 0:
        sp = BSpline(kn, cfix, 5)
        return sp, _fit_info(sp, ts, Y, Tp, Qp, None, cfg)
    B = BSpline.design_matrix(ts, kn, 5).tocsc()
    Bf = B[:, 3:n - 3]
    rhs0 = Bf.T @ (Y - B @ cfix)
    BtB = (Bf.T @ Bf).tocsc()
    D = sparse.diags([-1.0, 3.0, -3.0, 1.0], [0, 1, 2, 3], shape=(n - 3, n)).tocsc()
    DtD = (D.T @ D).tocsc()
    Pff = DtD[3:n - 3][:, 3:n - 3]
    Pfc = DtD[3:n - 3] @ cfix
    scale = len(ts) / n

    def solve(lg):
        lam = 10.0 ** lg * scale
        c = cfix.copy()
        c[3:n - 3] = np.atleast_2d(spsolve((BtB + lam * Pff).tocsc(), rhs0 - lam * Pfc)).reshape(m, 6)
        return BSpline(kn, c, 5)

    def ok(sp):
        X = sp(ts)
        dt_ = np.abs(X - Y).max()
        if dt_ > cfg.tol_time:
            return False, dt_, None
        dg = _poly_dist(ts, X, Tp, Qp).max()
        return dg <= cfg.tol_geo, dt_, dg

    lo_, hi_ = -10.0, 6.0
    sp_hi = solve(hi_)
    if ok(sp_hi)[0]:
        return sp_hi, _fit_info(sp_hi, ts, Y, Tp, Qp, hi_, cfg)
    sp_lo = solve(lo_)
    if not ok(sp_lo)[0]:
        inf = _fit_info(sp_lo, ts, Y, Tp, Qp, lo_, cfg)
        inf["tol_met"] = False
        return sp_lo, inf
    best = sp_lo
    blg = lo_
    for _ in range(14):
        mid = 0.5 * (lo_ + hi_)
        sp = solve(mid)
        if ok(sp)[0]:
            lo_, best, blg = mid, sp, mid
        else:
            hi_ = mid
    return best, _fit_info(best, ts, Y, Tp, Qp, blg, cfg)


def _fit_info(sp, ts, Y, Tp, Qp, lg, cfg):
    X = sp(ts)
    dg = float(_poly_dist(ts, X, Tp, Qp).max())
    dt_ = float(np.abs(X - Y).max())
    return dict(log10_lambda=lg, dev_geo_rad=dg, dev_time_rad=dt_, n_coef=len(sp.c),
                tol_met=bool(dg <= cfg.tol_geo * (1 + 1e-9) and dt_ <= cfg.tol_time * (1 + 1e-9)))


class SmoothPath:
    """Per robot: list of (ta, tb, spline) moving pieces; stationary (piecewise constant) elsewhere."""

    def __init__(self, plan, cfg: SmoothCfg):
        self.plan, self.cfg = plan, cfg
        self.pieces, self.poly, self.fit = [], [], []
        for r in range(plan.R):
            Tp, Qp = robot_polyline(plan, r)
            self.poly.append((Tp, Qp))
            pcs, inf = [], []
            for i, j in moving_intervals(Tp, Qp, cfg.eps_move):
                sp, info = fit_interval(Tp, Qp, Tp[i], Tp[j], cfg)
                pcs.append((float(Tp[i]), float(Tp[j]), sp, sp.derivative(1), sp.derivative(2), sp.derivative(3)))
                info.update(robot=plan.robot_ids[r], ta=round(float(Tp[i]), 4), tb=round(float(Tp[j]), 4))
                inf.append(info)
            self.pieces.append(pcs)
            self.fit.append(inf)

    def eval(self, s, nu_max=3):
        """s (n,) source-clock times -> list [Q, Q', Q'', Q'''] each (n, R, 6)."""
        s = np.asarray(s, float)
        out = [np.zeros((len(s), self.plan.R, 6)) for _ in range(nu_max + 1)]
        for r in range(self.plan.R):
            Tp, Qp = self.poly[r]
            idx = np.clip(np.searchsorted(Tp, s, "right") - 1, 0, len(Tp) - 1)
            out[0][:, r] = Qp[idx]                       # stationary default (exact at rests)
            for ta, tb, *f in self.pieces[r]:
                m = (s >= ta) & (s <= tb)
                if m.any():
                    for nu in range(nu_max + 1):
                        out[nu][m, r] = f[nu](s[m])
        return out

    def moving_mask(self, s):
        m = np.zeros(len(s), bool)
        for r in range(self.plan.R):
            for ta, tb, *_ in self.pieces[r]:
                m |= (s > ta) & (s < tb)
        return m

    def blocks(self):
        """Merged [a, b] source intervals in which at least one arm moves."""
        iv = sorted((ta, tb) for pcs in self.pieces for ta, tb, *_ in pcs)
        out = []
        for a, b in iv:
            if out and a <= out[-1][1] + 1e-9:
                out[-1][1] = max(out[-1][1], b)
            else:
                out.append([a, b])
        return [tuple(x) for x in out]


# ───────────────────────────────────────────── time scaling
def _derivs_tau(D1, D2, D3, r, rs, rss):
    """Joint v / a / jerk in tau from path derivatives (n, R, 6) and r, r', r'' (n,)."""
    r_, rs_, rss_ = r[:, None, None], rs[:, None, None], rss[:, None, None]
    v = D1 * r_
    a = D2 * r_ ** 2 + D1 * r_ * rs_
    j = D3 * r_ ** 3 + 3 * D2 * r_ ** 2 * rs_ + D1 * (r_ ** 2 * rss_ + r_ * rs_ ** 2)
    return v, a, j


def _passes(u, D1, D2, A, ds):
    """Forward / backward phase-plane passes: |Q'' u + Q' u'/2| <= A (per joint)."""
    a1, a2 = np.abs(D1), np.abs(D2)
    n = len(u)
    for i in range(n - 1):
        m = a1[i] > 1e-9
        if m.any():
            g = np.maximum(A[m] - a2[i][m] * u[i], 0.0)
            u[i + 1] = min(u[i + 1], u[i] + float(np.min(2 * ds * g / a1[i][m])))
    for i in range(n - 1, 0, -1):
        m = a1[i] > 1e-9
        if m.any():
            g = np.maximum(A[m] - a2[i][m] * u[i], 0.0)
            u[i - 1] = min(u[i - 1], u[i] + float(np.min(2 * ds * g / a1[i][m])))
    return u


def _smooth_u(u, W):
    if W <= 1:
        return u
    e = minimum_filter1d(u, size=2 * W + 1, mode="nearest")
    k = np.hanning(2 * W + 3)[1:-1]
    k /= k.sum()
    p = np.pad(e, W, mode="edge")
    return np.minimum(np.convolve(p, k, mode="valid"), u)


def _evaluate(u, D1, D2, D3, V, A0, J0, ds):
    r = np.sqrt(np.maximum(u, 1e-12))
    rs = np.gradient(r, ds)
    rss = np.gradient(rs, ds)
    v, ac, jk = _derivs_tau(D1, D2, D3, r, rs, rss)
    over = np.maximum.reduce([(np.abs(v) / V).max(axis=(1, 2)), (np.abs(ac) / A0).max(axis=(1, 2)),
                              (np.abs(jk) / J0).max(axis=(1, 2))])
    T = float((2 * ds / (r[:-1] + r[1:])).sum())
    return r, rs, rss, over, T


def scale_block(sp: SmoothPath, a, b, cfg: SmoothCfg, holds=(), max_iter=40):
    """Time map of one block [a, b] (source clock). -> dict(s grid, tau grid, r, rs, rss, info).

    For each kernel half-width W: u_bound = r_max^2 (pointwise velocity / acceleration / jerk bounds, rate cap,
    hold caps); repeat {phase-plane passes for the acceleration; erosion + Hann smoothing of width W; dense
    v / a / jerk check on the grid; where the check fails, lower u_bound locally by over^(-2/3) (jerk ~ r^3)}
    until it passes. The fastest passing W wins; if none passes, a global limit scale kappa is tried on top."""
    p = _profile(cfg.profile)
    plan = sp.plan
    n = max(int(np.ceil((b - a) / cfg.ds)), 4)
    s = np.linspace(a, b, n + 1)
    ds = (b - a) / n
    _, D1, D2, D3 = sp.eval(s)
    V = (p.v_frac * plan.v_max)[None]
    A0 = (p.a_frac * plan.a_max)[None]
    J0 = (p.j_frac * plan.a_max)[None]
    cap = np.full(n + 1, cfg.r_cap)
    for (h0, h1) in holds:
        # the hold must last >= hold_s on the new clock: L_in / r + L_out >= hold_s, where L_in is the part of the
        # source hold inside this block and L_out the part in the surrounding rests (rate 1)
        L_in = max(0.0, min(h1, b) - max(h0, a))
        need = cfg.hold_s - ((h1 - h0) - L_in)
        if L_in > 1e-9 and need > 1e-9:
            m = (s >= max(h0, a)) & (s <= min(h1, b))
            cap[m] = np.minimum(cap[m], max(L_in / need, 1e-3))
    D1f, D2f = D1.reshape(n + 1, -1), D2.reshape(n + 1, -1)
    best, tried = None, 0
    for kappa in cfg.kappas:
        A, J = A0 * kappa, J0 * kappa
        with np.errstate(divide="ignore", invalid="ignore"):
            r_v = np.nanmin(np.where(np.abs(D1) > 1e-12, V * min(1.0, kappa ** 0.5) / np.abs(D1), np.inf), axis=(1, 2))
            r_a = np.sqrt(np.nanmin(np.where(np.abs(D2) > 1e-12, A / np.abs(D2), np.inf), axis=(1, 2)))
            r_j = np.cbrt(np.nanmin(np.where(np.abs(D3) > 1e-12, J / np.abs(D3), np.inf), axis=(1, 2)))
        rmax = np.minimum.reduce([cap, r_v, r_a, r_j])
        for w in cfg.windows:
            W = max(1, int(round(w / ds)))
            ub = rmax ** 2
            for it in range(max_iter):
                tried += 1
                u = _smooth_u(_passes(ub.copy(), D1f, D2f, A.reshape(-1), ds), W)
                r, rs, rss, over, T = _evaluate(u, D1, D2, D3, V, A0, J0, ds)
                ok = bool(over.max() <= 1.0 + 1e-3)
                if ok or it == max_iter - 1:
                    break
                f = maximum_filter1d(np.where(over > 1.0, over, 0.0), size=4 * W + 1, mode="nearest")
                ub = np.where(f > 1.0, ub * np.clip(np.where(f > 1.0, f, 1.0) ** (-2.0 / 3.0), 0.3, 0.97), ub)
            cand = dict(T=T, ok=ok, over=float(over.max()), kappa=kappa, window_s=w, r=r, rs=rs, rss=rss, iters=it + 1)
            if best is None or (ok and (not best["ok"] or T < best["T"])) or (not ok and not best["ok"] and cand["over"] < best["over"]):
                best = cand
        if best["ok"]:
            break
    tau = np.concatenate([[0.0], np.cumsum(2 * ds / (best["r"][:-1] + best["r"][1:]))])
    return dict(s=s, tau=tau, r=best["r"], rs=best["rs"], rss=best["rss"],
                info=dict(a=round(a, 4), b=round(b, 4), src_s=round(b - a, 4), new_s=round(best["T"], 4),
                          ok=best["ok"], over=round(best["over"], 4), kappa=best["kappa"], window_s=best["window_s"],
                          iters=best["iters"], n_tried=tried))


class TimeMap:
    """Piecewise map source clock sigma <-> new clock tau: blocks retimed, rests at rate 1 (all arms stand still
    there, so r does not matter for the joint derivatives)."""

    def __init__(self, blocks):
        self.blocks = blocks
        S, Tt, R, RS, RSS = [np.array([0.0])], [np.array([0.0])], [np.array([1.0])], [np.array([0.0])], [np.array([0.0])]
        shift = 0.0
        self._offs = []
        for bl in blocks:
            a = bl["s"][0]
            off = a + shift
            self._offs.append(off)
            S.append(bl["s"]); Tt.append(off + bl["tau"]); R.append(bl["r"]); RS.append(bl["rs"]); RSS.append(bl["rss"])
            shift += bl["tau"][-1] - (bl["s"][-1] - a)
        self.S, self.T = np.concatenate(S), np.concatenate(Tt)
        self.R, self.RS, self.RSS = np.concatenate(R), np.concatenate(RS), np.concatenate(RSS)
        self.shift_end = shift
        self._herm = None
        if blocks and blocks[0]["s"][0] <= 1e-12:
            self.S, self.T, self.R, self.RS, self.RSS = (x[1:] for x in (self.S, self.T, self.R, self.RS, self.RSS))
        o = np.argsort(self.S, kind="stable")
        self.S, self.T, self.R, self.RS, self.RSS = self.S[o], self.T[o], self.R[o], self.RS[o], self.RSS[o]

    def tau(self, sig):
        sig = np.atleast_1d(np.asarray(sig, float))
        out = np.interp(sig, self.S, self.T)
        last = sig > self.S[-1]
        out[last] = self.T[-1] + (sig[last] - self.S[-1])
        return out

    def sigma(self, tau):
        """tau -> (sigma, r, r', r'')."""
        tau = np.atleast_1d(np.asarray(tau, float))
        if self._herm is None:
            from scipy.interpolate import CubicHermiteSpline
            self._herm = []
            for bl, off in zip(self.blocks, self._offs):    # per block: dsigma/dtau = r, C1, O(ds^4) accurate
                self._herm.append((off, off + bl["tau"][-1], CubicHermiteSpline(off + bl["tau"], bl["s"], bl["r"])))
        sig = np.interp(tau, self.T, self.S)               # rests: exact (rate 1)
        for t0, t1, h in self._herm:
            m = (tau >= t0) & (tau <= t1)
            if m.any():
                sig[m] = h(tau[m])
        last = tau > self.T[-1]
        sig[last] = self.S[-1] + (tau[last] - self.T[-1])
        r = np.interp(sig, self.S, self.R)
        rs = np.interp(sig, self.S, self.RS)
        rss = np.interp(sig, self.S, self.RSS)
        return sig, r, rs, rss

    @property
    def added_s(self):
        return float(self.shift_end)


def smooth_plan(plan, cfg: SmoothCfg | None = None):
    """-> dict(legs: new player Leg objects (with .vel / .acc), path, timemap, info)."""
    from player.timeline import Leg
    cfg = cfg or SmoothCfg()
    sp = SmoothPath(plan, cfg)
    holds = [(e[2], e[3]) for e in plan.weld_events]
    blocks = [scale_block(sp, a, b, cfg, holds) for a, b in sp.blocks()]
    if cfg.r_cap > 1.0:          # the rate-capped (preserve) schedule is feasible too: keep the faster one per block
        from dataclasses import replace as _rep
        c1 = _rep(cfg, r_cap=1.0)
        alt = [scale_block(sp, a, b, c1, holds) for a, b in sp.blocks()]
        blocks = [y if (y["info"]["ok"] and (not x["info"]["ok"] or y["tau"][-1] < x["tau"][-1])) else x
                  for x, y in zip(blocks, alt)]
    tm = TimeMap(blocks)
    new_legs = []
    for rid in plan.robot_ids:
        r = plan.robot_ids.index(rid)
        for leg in plan.robot_legs(rid):
            t0n, t1n = tm.tau([leg.t0, leg.t1])
            nstep = max(1, int(np.ceil((t1n - t0n) / cfg.out_dt - 1e-9)))
            tt = t0n + np.arange(nstep + 1) * cfg.out_dt
            sig, rr, rs, _ = tm.sigma(np.minimum(tt, t1n))
            sig = np.clip(sig, leg.t0, leg.t1)
            Qs, D1, D2, _ = sp.eval(sig)
            pos = Qs[:, r]
            vel = D1[:, r] * rr[:, None]
            acc = D2[:, r] * rr[:, None] ** 2 + D1[:, r] * (rr * rs)[:, None]
            pos[-1] = leg.positions[-1]
            vel[[0, -1]] = 0.0
            acc[[0, -1]] = 0.0
            nl = Leg(robot=rid, op_id=leg.op_id, kind=leg.kind, t0=float(tt[0]), t1=float(tt[-1]), times=tt,
                     positions=pos, phase=np.array(["smoothed"] * len(tt), dtype="<U16"), hold_s=leg.hold_s,
                     vscale=1.0, operation_type=leg.operation_type, node_type=leg.node_type, bb_key=leg.bb_key)
            nl.t_done = nl.t1 + (leg.t_done - leg.t1)
            nl.vel, nl.acc = vel, acc
            new_legs.append(nl)
    fit = [x for f in sp.fit for x in f]
    info = dict(cfg=cfg.to_dict(), n_pieces=len(fit), fit_tol_met=all(f["tol_met"] for f in fit),
                max_dev_geo_rad=round(max([f["dev_geo_rad"] for f in fit], default=0.0), 6),
                max_dev_time_rad=round(max([f["dev_time_rad"] for f in fit], default=0.0), 6),
                fit_tol_failures=[f for f in fit if not f["tol_met"]][:5],
                n_blocks=len(blocks), blocks_ok=all(b["info"]["ok"] for b in blocks),
                block_failures=[b["info"] for b in blocks if not b["info"]["ok"]][:5],
                added_motion_s=round(tm.added_s, 3),
                kappa_min=min([b["info"]["kappa"] for b in blocks], default=None))
    return dict(legs=new_legs, path=sp, timemap=tm, info=info)
