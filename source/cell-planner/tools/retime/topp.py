"""Time-optimal path parameterisation with path-varying velocity / acceleration / jerk limits (pure numpy).

The path is q(s) on a uniform grid of the path parameter s (arc length in joint space for one arm, or the source
clock for several arms on one clock), with derivatives D1 = dq/ds, D2, D3 (n+1, K). The time law s(t) gives

    v = D1 sd,   a = D2 sd^2 + D1 sdd,   j = D3 sd^3 + 3 D2 sd sdd + D1 sddd

Limits are given per grid point and per coordinate (V, A, J arrays), so the weld zones (slow) and the transit (full
speed) are one constraint set and the solver accelerates / decelerates continuously between them: there are no
velocity jumps at a zone boundary, the deceleration into a slow zone happens before it.

Algorithm:
1. pointwise bound on sd: min(cap, V/|D1|, sqrt(A/|D2|), cbrt(0.8 J/|D3|));
2. TOPP phase-plane passes on u = sd^2 for the acceleration (as in tools/ctrl/smooth.py of track K, generalised to
   varying limits): u_{i+1} <= u_i + 2 ds min_k max(A_k - |D2_k| u_i, 0)/|D1_k|, and the mirror image backwards;
3. jerk smoothing (S-curve): the path speed sd(t) of the TOPP profile is filtered in TIME by a moving average of width
   T_f (a box FIR filter, Biagiotti & Melchiorri): the path acceleration becomes continuous (linear ramps of length
   T_f), the path length is kept (rescaled exactly) and a rest stays a rest, so starts and stops are S-curves. The
   duration grows by about T_f per piece. Without a jerk limit the plain bang-bang TOPP profile is used;
4. dense check of |v|/V, |a|/A, |j|/J and sd/cap on the time grid (1 ms; time derivatives of the filtered profile);
   where it fails, the bound is lowered locally (by the excess ratio, around the violation) and steps 2-4 are
   repeated (up to 25 times); every iterate's remaining excess is removed by a uniform slow-down of the piece
   (sd -> kappa sd scales v, a, jerk by kappa, kappa^2, kappa^3) and the fastest iterate / filter width wins.
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import maximum_filter1d

TOL = 1e-3


FA, FJ = 1.0, 0.8   # shares of A / J the centripetal terms may use in the pointwise bounds (the rest is left for the
#                     tangential terms, so a profile riding the bound stays feasible)


def _bounds(D1, D2, D3, V, A, J, cap):
    with np.errstate(divide="ignore", invalid="ignore"):
        rv = np.min(np.where(np.abs(D1) > 1e-12, V / np.abs(D1), np.inf), axis=1)
        ra = np.sqrt(np.min(np.where(np.abs(D2) > 1e-12, FA * A / np.abs(D2), np.inf), axis=1))
        rj = (np.cbrt(np.min(np.where(np.abs(D3) > 1e-12, FJ * J / np.abs(D3), np.inf), axis=1)) if J is not None
              else np.full(len(D1), np.inf))
    return np.minimum.reduce([cap, rv, ra, rj])


def _passes(ub, D1, D2, A, ds):
    """Forward / backward passes (pure python on lists, ~2 us per grid point) with the conservative per-step bound
    sdd <= min_k max(A_k - |D2_k| u, 0) / |D1_k| (both directions). The exact signed bounds put the profile on the
    limit everywhere, and the S-curve filter then needs more local slow-downs: the conservative bound gave the
    shorter final times on the canonical timelines (docs/p3_speed.md)."""
    a1 = np.abs(D1)
    inf = float("inf")
    inv = np.where(a1 > 1e-9, 1.0 / np.maximum(a1, 1e-300), inf).tolist()
    a2 = np.abs(D2).tolist()
    Aa = np.asarray(A, float)
    # step i -> i +- 1 uses the smaller of the two limits (exact at a step change of the limits, e.g. a zone entry)
    Al = np.minimum(Aa, np.r_[Aa[1:], Aa[-1:]]).tolist()
    Ab = np.minimum(Aa, np.r_[Aa[:1], Aa[:-1]]).tolist()
    u = ub.tolist()
    n = len(u)
    K = len(Al[0])
    h = 2.0 * ds
    for sweep in (range(n - 1), range(n - 1, 0, -1)):
        fwd = sweep.step > 0
        AL = Al if fwd else Ab
        for i in sweep:
            x = u[i]
            Ai, bi, ci = AL[i], a2[i], inv[i]
            best = 1e300
            for k in range(K):
                c = ci[k]
                if c != inf:
                    g = Ai[k] - bi[k] * x
                    v = (g if g > 0.0 else 0.0) * c
                    if v < best:
                        best = v
            y = x + h * best
            j = i + 1 if fwd else i - 1
            if y < u[j]:
                u[j] = y
    return np.asarray(u)


def time_law(u, ds, T_f=0.0, dt=1e-3):
    """u on the s grid -> time law on a uniform time grid: (t, s, sd, sdd, sddd). With T_f > 0 the path speed is
    filtered by a moving average of width T_f (S-curve)."""
    n1 = len(u)
    S = np.arange(n1) * ds
    r = np.sqrt(np.maximum(u, 0.0))
    tau = np.concatenate([[0.0], np.cumsum(2 * ds / np.maximum(r[:-1] + r[1:], 1e-12))])
    Tt = float(tau[-1])
    if T_f > 0:
        dt = min(dt, T_f / 20.0)
    tg = np.arange(0.0, Tt + dt, dt)
    sd = np.interp(np.interp(tg, tau, S), S, r)
    if T_f > 0:
        m = max(1, int(round(T_f / dt)))
        sd = np.convolve(sd, np.ones(m) / m)
        tg = np.arange(len(sd)) * dt
    s = np.concatenate([[0.0], np.cumsum(0.5 * (sd[1:] + sd[:-1]) * dt)])
    if s[-1] > 0:
        f = S[-1] / s[-1]
        s, sd = s * f, sd * f                  # exact path length (the trapezoid sum differs by O(dt^2))
    sdd = np.gradient(sd, dt)
    sddd = np.gradient(sdd, dt)
    return tg, np.clip(s, 0.0, S[-1]), sd, sdd, sddd


def derivs(D1, D2, D3, sd, sdd, sddd):
    """Joint v / a / jerk (n, K) from path derivatives and the time law."""
    a_, b_, c_ = sd[:, None], sdd[:, None], sddd[:, None]
    v = D1 * a_
    a = D2 * a_ ** 2 + D1 * b_
    j = D3 * a_ ** 3 + 3 * D2 * a_ * b_ + D1 * c_
    return v, a, j


def interp_cols(x, xp, F):
    """Linear interpolation of the columns of F (len(xp), K) at x."""
    i = np.clip(np.searchsorted(xp, x) - 1, 0, len(xp) - 2)
    w = ((x - xp[i]) / (xp[i + 1] - xp[i]))[:, None]
    return F[i] * (1 - w) + F[i + 1] * w


def _evaluate(law, S, D1, D2, D3, V, A, J, capf, dfun=None):
    tg, s, sd, sdd, sddd = law
    if dfun is not None:                       # exact path derivatives at every time sample (corners of the path
        d1, d2, d3 = dfun(s)                   # can peak between grid points)
    else:
        d1, d2, d3 = (interp_cols(s, S, D) for D in (D1, D2, D3))
    k = np.clip(np.round(s / (S[1] - S[0])).astype(int), 0, len(S) - 1)
    v, a, j = derivs(d1, d2, d3, sd, sdd, sddd)
    ov = (np.abs(v) / V[k]).max(1)
    oa = (np.abs(a) / A[k]).max(1)
    oj = (np.abs(j) / J[k]).max(1) if J is not None else np.zeros(len(tg))
    oc = np.where(np.isfinite(capf[k]), sd / capf[k], 0.0)
    return ov, oa, oj, oc, k


def scale_path(D1, D2, D3, ds, V, A, J=None, cap=None, filters_s=(0.1, 0.15, 0.2), max_iter=25, tol=TOL,
               dt=1e-3, check=None, dfun=None):
    """Time scaling of one path piece. D1..D3, V, A, J: (n+1, K); cap: (n+1,) upper bound on sd (0 = rest);
    ``filters_s``: S-curve filter widths tried when J is given (the fastest passing one wins). ``check``: optional
    (V, A, J) the result is checked against (default: the planning limits V, A, J; the planner may use tighter ones).
    ``dfun(s) -> (D1, D2, D3)``: exact path derivatives for the check (default: linear interpolation of the grid).

    Returns dict(t, s, sd, sdd, sddd (time grid), T, ok, over / over_v / over_a / over_j / over_cap (max ratios),
    filter_s, iters, kappa)."""
    n1 = len(D1)
    S = np.arange(n1) * ds
    cap = np.full(n1, np.inf) if cap is None else np.asarray(cap, float)
    V = np.broadcast_to(V, D1.shape)
    A = np.broadcast_to(A, D1.shape)
    J = None if J is None else np.broadcast_to(J, D1.shape)
    rmax = _bounds(D1, D2, D3, V, A, J, cap)
    if not np.isfinite(rmax).all():
        raise ValueError("unbounded rate: give a finite cap where the path does not move")
    capf = np.where(cap > 0, cap, np.inf)          # cap 0 marks a rest point (u = 0 there), not checked as a rate
    Vc, Ac, Jc = (V, A, J) if check is None else (np.broadcast_to(check[0], D1.shape),
                                                    np.broadcast_to(check[1], D1.shape),
                                                    None if check[2] is None else np.broadcast_to(check[2], D1.shape))
    best = None

    def finish(law, ov, oa, oj, oc, T_f, it):
        """Remove a remaining excess by a uniform slow-down (kappa) -> candidate dict."""
        tg, s, sd, sdd, sddd = law
        over = np.maximum.reduce([ov, oa, oj, oc])
        kappa = 1.0
        if over.max() > 1.0 + tol:
            kappa = (1.0 - 1e-4) * min(1.0, 1.0 / max(ov.max(), oc.max(), 1e-12), 1.0 / np.sqrt(max(oa.max(), 1e-12)),
                                       1.0 / np.cbrt(max(oj.max(), 1e-12)))
            tg, sd, sdd, sddd = tg / kappa, sd * kappa, sdd * kappa ** 2, sddd * kappa ** 3
            ov, oc, oa, oj = ov * kappa, oc * kappa, oa * kappa ** 2, oj * kappa ** 3
            over = np.maximum.reduce([ov, oa, oj, oc])
        return dict(t=tg, s=s, sd=sd, sdd=sdd, sddd=sddd, T=float(tg[-1]), ok=bool(over.max() <= 1.0 + tol),
                    over=float(over.max()), over_v=float(ov.max()), over_a=float(oa.max()), over_j=float(oj.max()),
                    over_cap=float(oc.max()), filter_s=T_f, iters=it + 1, kappa=float(kappa))

    for T_f in (filters_s if J is not None else (0.0,)):
        ub = rmax ** 2
        for it in range(max_iter):
            u = _passes(ub.copy(), D1, D2, A, ds)
            law = time_law(u, ds, T_f, dt)
            ov, oa, oj, oc, k = _evaluate(law, S, D1, D2, D3, Vc, Ac, Jc, capf, dfun)
            # every iterate is a candidate (after its kappa): the local reductions are not monotone in the duration
            cand = finish(law, ov, oa, oj, oc, T_f, it)
            if best is None or (cand["ok"] and (not best["ok"] or cand["T"] < best["T"])) or \
                    (not cand["ok"] and not best["ok"] and cand["over"] < best["over"]):
                best = cand
            over = np.maximum.reduce([ov, oa, oj, oc])
            if over.max() <= 1.0 + tol:
                break
            f = np.zeros(n1)
            bad = over > 1.0 + tol
            np.maximum.at(f, k[bad], over[bad])
            w = max(3, int(round(0.25 * (T_f if T_f > 0 else 0.02) * max(float(law[2].max()), 1e-9) / ds)))
            f = maximum_filter1d(f, size=2 * w + 1, mode="nearest")
            ub = np.where(f > 1.0, ub * np.clip(np.where(f > 1.0, f, 1.0) ** (-1.0), 0.25, 0.95), ub)
    return best
