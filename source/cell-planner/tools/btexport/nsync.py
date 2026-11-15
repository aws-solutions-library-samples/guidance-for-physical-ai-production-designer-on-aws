"""N-robot rest-synchronous export of a continuous motion (any number of arms, any trajgen scene).

The two-arm umarell-cell path uses execute.legs_sync_rest unchanged. This module is the same method for R arms whose
limits are not a umarell cell's (trajgen scenes with 1-4 robots, also two-robot scenes such as two_robots_moved):

1. cut instants: the fewest instants that stab every weld's window (``execute.choose_cuts``), preferring instants
   where the arms move slowly; a weld window is the interval in which the welding arm stands still at its weld
   (weld event arrival .. departure);
2. one common-clock time warp for all arms (:func:`warp`, the phase-plane scaling of ``execute.time_warp`` for R x 6
   joints): every arm brakes to rest at every cut and re-accelerates after it, joint accelerations <= ``A``; the clock
   never runs faster than the input (u <= 1), so the configurations that occur together are the input's;
3. legs: every arm is cut at the same instants; the leg of the welding arm that ends at a cut inside its window is
   the weld leg (``op_id`` = the weld id, ``hold_s``), the others are moves; executor pause (hold + 1.0 s margin)
   with every arm at rest.

Pure numpy; used by tools.btexport.core for the ``retime`` backend on scenes (and 2-robot scenes).
"""
from __future__ import annotations

import numpy as np


def _r(x, n=4):
    return None if x is None else round(float(x), n)


U_FLOOR = 1e-12              # clock speed^2 below this counts as stopped (warp's stall guard)
MAX_CLOCK_FACTOR = 10.0      # the warped clock may be at most this many times the input's duration (round 2)


class WarpError(ValueError):
    """The common-clock warp cannot produce a bounded clock (docs/n2_traj2.md: round-1 MemoryError)."""


def check_clock(tau, T_in: float, where: str = "warp") -> None:
    """Refuse a warped clock longer than MAX_CLOCK_FACTOR x the input duration before anything is allocated over it
    (round 1: a stalled clock reached 2.5e7 s and acc_peak asked for 37-186 GiB)."""
    T = float(tau[-1] - tau[0])
    lim = MAX_CLOCK_FACTOR * max(float(T_in), 1.0)
    if not np.isfinite(T) or T > lim:
        raise WarpError(f"{where}: warped clock {T:.4g} s exceeds {MAX_CLOCK_FACTOR:g} x the input's {T_in:.4g} s "
                        f"(the clock stalls: u -> 0 between consecutive samples)")


def windows(t, q, weld_events, robot_ids, tol=1e-6):
    """[(robot, wid, k_first, k_last, k_complete)] on the grid ``t``: the samples in [arrival, departure] (extended
    while the arm stays still)."""
    t = np.asarray(t, float)
    out = []
    for rid, wid, ta, tl in weld_events:
        ri = list(robot_ids).index(rid)
        kf = int(np.searchsorted(t, float(ta) - 1e-9))
        kl = int(np.searchsorted(t, float(tl) + 1e-9)) - 1
        kf = min(max(kf, 0), len(t) - 1)
        kl = min(max(kl, kf), len(t) - 1)
        while kl + 1 < len(t) and np.abs(q[kl + 1, ri] - q[kf, ri]).max() < tol:
            kl += 1
        out.append((rid, str(wid), kf, kl, kf))
    return out


def warp(t, Q, rest_idx, A, *, sub=4, h=0.05, a_mvc=None):
    """``execute.time_warp`` for D = R x 6 joints: ``Q`` (n, D); ``A`` / ``a_mvc`` (D,) or per fine sample (M, D)
    (lower limits inside weld zones / where the jerk iteration softens the braking). -> (s, tau, qf (M, D), u)."""
    t = np.asarray(t, float)
    Q = np.asarray(Q, float)
    n, D = Q.shape
    ds = (t[1] - t[0]) / sub
    s = t[0] + np.arange((n - 1) * sub + 1) * ds
    M = len(s)
    interp = lambda x: np.stack([np.interp(np.clip(x, t[0], t[-1]), t, Q[:, k]) for k in range(D)], -1)  # noqa: E731
    qf = interp(s)
    d1 = (interp(s + h / 2) - interp(s - h / 2)) / h
    d2 = (interp(s + h) - 2 * qf + interp(s - h)) / h ** 2
    A = np.broadcast_to(np.asarray(A, float), (M, D)) if np.ndim(A) == 2 else \
        np.broadcast_to(np.asarray(A, float).reshape(D), (M, D))
    Am = A if a_mvc is None else (np.broadcast_to(np.asarray(a_mvc, float), (M, D)) if np.ndim(a_mvc) == 2 else
                                  np.broadcast_to(np.asarray(a_mvc, float).reshape(D), (M, D)))
    ad1, ad2 = np.abs(d1), np.abs(d2)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.minimum(1.0, np.nanmin(np.where(ad2 > 1e-12, (Am / ad2) ** 2, np.inf), axis=1))
    rest = sorted({0, len(s) - 1} | {int(k) * sub for k in rest_idx})
    u[rest] = 0.0
    for i in range(len(s) - 1):
        m = ad1[i] > 1e-9
        if m.any():
            u[i + 1] = min(u[i + 1], float(np.min(u[i] + 2 * ds * np.maximum(A[i, m] - ad2[i, m] * np.sqrt(u[i]), 0.0)
                                                   / ad1[i, m])))
    for i in range(len(s) - 1, 0, -1):
        m = ad1[i - 1] > 1e-9
        if m.any():
            up = np.minimum(1.0, u[i] + 2 * ds * A[i - 1, m] / ad1[i - 1, m])
            ub = np.min(u[i] + 2 * ds * np.maximum(A[i - 1, m] - ad2[i - 1, m] * np.sqrt(up), 0.0) / ad1[i - 1, m])
            if ub < U_FLOOR and u[i] < U_FLOOR:
                # stall guard (round 2, docs/n2_traj2.md): braking to a rest point where the path curvature term at
                # the upper estimate sqrt(up) eats the whole budget (a timeline that ends / stops while moving) gave
                # u = 0 on two consecutive samples, a fine step of 2 ds / 1e-9 = 2.5e7 s and a MemoryError. Solve the
                # same step implicitly at the new speed itself: x^2 = u[i] + 2 ds (A - |d2| x) / |d1|, x = sqrt(u),
                # which is > 0 whenever A > 0 (the warped motion's acceleration is re-measured by the caller).
                b = 2 * ds * ad2[i - 1, m] / ad1[i - 1, m]
                c = u[i] + 2 * ds * A[i - 1, m] / ad1[i - 1, m]
                x = 0.5 * (np.sqrt(b * b + 4 * c) - b)
                ub = float(np.min(x * x))
            u[i - 1] = min(u[i - 1], float(ub))
    su = np.sqrt(np.maximum(u, 0.0))
    dtau = 2 * ds / np.maximum(su[:-1] + su[1:], 1e-9)
    tau = np.concatenate([[0.0], np.cumsum(dtau)])
    return s, tau, qf, u


def leaf_jerk(tau, qf, bounds, a_max_flat, *, h=0.05, hz=200.0):
    """Within-leaf jerk of the warped motion on the project's 0.05 s third-difference metric (a_max per second):
    for every leaf [tau[bounds[k]], tau[bounds[k+1]]] the stencil t +- 1.5 h must lie inside the leaf (the rest
    point at a cut is a leaf boundary, not a jerk inside a trajectory). -> (centers tau (m,), ratio (m,))."""
    cs, rs = [], []
    for k in range(len(bounds) - 1):
        a, b = float(tau[bounds[k]]), float(tau[bounds[k + 1]])
        if b - a <= 3 * h + 2.0 / hz:
            continue
        c = np.arange(a + 1.5 * h, b - 1.5 * h + 1e-12, 1.0 / hz)
        f = lambda x: np.stack([np.interp(x, tau, qf[:, d]) for d in range(qf.shape[1])], -1)  # noqa: E731
        j = (f(c + 1.5 * h) - 3 * f(c + 0.5 * h) + 3 * f(c - 0.5 * h) - f(c - 1.5 * h)) / h ** 3
        cs.append(c)
        rs.append((np.abs(j) / a_max_flat).max(axis=1))
    if not cs:
        return np.zeros(0), np.zeros(0)
    return np.concatenate(cs), np.concatenate(rs)


def choose_cuts(windows, speed_cost, forbid=None):
    """``execute.choose_cuts`` with forbidden instants (an arm moving inside a weld zone): among the instants of
    the earliest-ending open window, an allowed one is always preferred; then most windows stabbed, least braking,
    latest. Returns ({weld index: cut}, sorted cuts, [weld indices whose cut had to be a forbidden instant])."""
    forbid = np.zeros(len(speed_cost), bool) if forbid is None else np.asarray(forbid, bool)
    open_ = set(range(len(windows)))
    cut_of, cuts, bad = {}, [], []
    while open_:
        i = min(open_, key=lambda i: windows[i][3])
        kf, kl = windows[i][2], windows[i][3]
        best = None
        for c in range(kf, kl + 1):
            cover = [j for j in open_ if windows[j][2] <= c <= windows[j][3]]
            key = (bool(forbid[c]), -len(cover), int(min(speed_cost[c], 1e6) / 0.05), -c)
            if best is None or key < best[0]:
                best = (key, c, cover)
        _, c, cover = best
        cuts.append(c)
        for j in cover:
            cut_of[j] = c
            open_.discard(j)
            if forbid[c]:
                bad.append(j)
    return cut_of, sorted(set(cuts)), bad


def zone_masks(t, q, robot_ids, tcp, wpos, v_max, *, labels_t=None, labels=None, inner_m=0.05, move_frac=0.005):
    """Per sample of ``t`` (n,) and arm: ``zone`` (n, R) = inside a weld operation (tools.retime's approach /
    lift-off labels, nearest label sample) or with the TCP within ``inner_m`` of any weld of the task while moving;
    ``moving`` (n, R) = some joint above ``move_frac`` x v_max. -> (zone, moving, dist (n, R) to the nearest weld)."""
    t = np.asarray(t, float)
    q = np.asarray(q, float)
    n, R = q.shape[0], q.shape[1]
    vel = np.abs(np.gradient(q, t, axis=0)) / np.asarray(v_max, float).reshape(1, R, 6)
    moving = vel.max(axis=2) > move_frac
    W = np.stack([np.asarray(v, float) for v in wpos.values()]) if wpos else np.zeros((0, 3))
    dist = np.full((n, R), np.inf)
    for r, rid in enumerate(robot_ids):
        if len(W):
            P = tcp[rid](q[:, r])
            dist[:, r] = np.linalg.norm(P[:, None, :] - W[None], axis=2).min(axis=1)
    zone = (dist <= inner_m) & moving
    if labels is not None and labels_t is not None:
        lab = np.asarray(labels)
        k = np.clip(np.searchsorted(np.asarray(labels_t, float), t), 0, len(labels_t) - 1)
        L = lab[k]
        if L.ndim == 1:
            L = L[:, None]
        zone |= ((L == 1) | (L == 2))            # tools.retime L_APPROACH / L_LIFTOFF
    return zone, moving, dist


def forbidden_cuts(t, q, zone, moving, a_max, *, margin=1.5, jerk_cap=None):
    """(n,) instants where no cut may be placed: some arm is inside its weld zone (``zone_masks``: labelled
    approach / lift-off, or moving within 5 cm of a weld), or the arm's braking to
    rest before the cut / its restart after it (time max_j |qd_j| / a_max_j, x ``margin``, + a_max / jerk_cap for
    the jerk ramps) would reach into a weld zone of that arm (the braking would have to happen inside an approach or
    lift-off, where the weld acceleration rule leaves no budget)."""
    t = np.asarray(t, float)
    q = np.asarray(q, float)
    n, R = q.shape[0], q.shape[1]
    vel = np.abs(np.gradient(q, t, axis=0))
    tb = (vel / np.asarray(a_max, float).reshape(1, R, 6)).max(axis=2) * margin + (1.0 / jerk_cap if jerk_cap else 0.0)
    zm = np.asarray(zone, bool)          # labelled approach / lift-off at any speed (the creep into the weld too)
    out = zm.any(axis=1).copy()
    for r in range(R):
        zr = np.flatnonzero(zm[:, r])
        if not len(zr):
            continue
        tz = t[zr]
        mv = np.flatnonzero(moving[:, r])
        lo = np.searchsorted(tz, t[mv] - tb[mv, r], "left")          # a zone sample in [t - tb, t + tb]
        hi = np.searchsorted(tz, t[mv] + tb[mv, r], "right")
        out[mv[hi > lo]] = True
    return out


def acc_peak(tau, qf, h=0.05, hz=200.0, max_s: float | None = None):
    """Per-joint max |q''| (rad/s^2) of the warped motion on the project's 0.05 s second-difference metric.
    ``max_s``: refuse (WarpError) a clock longer than this instead of allocating its grid."""
    if max_s is not None and float(tau[-1] - tau[0]) > float(max_s):
        raise WarpError(f"acc_peak: clock {float(tau[-1] - tau[0]):.4g} s > {float(max_s):.4g} s")
    ts = np.arange(tau[0], tau[-1] + 1e-9, 1.0 / hz)
    f = lambda x: np.stack([np.interp(np.clip(x, tau[0], tau[-1]), tau, qf[:, k])  # noqa: E731
                            for k in range(qf.shape[1])], -1)
    return (np.abs(f(ts + h) - 2 * f(ts) + f(ts - h)) / h ** 2).max(0)


def legs_sync_rest_n(t, q, weld_events, robot_ids, v_max, a_max, *, hold_s=0.5, margin=1.0, home_last=True,
                     acc_frac=1.0, acc_mode="envelope", sub=4, max_iter=4, zone=None, zone_afrac=None, forbid=None,
                     jerk_cap=None, max_jerk_iter=12):
    """R-arm sync_rest export. ``t`` (n,) uniform, ``q`` (n, R, 6), ``v_max`` / ``a_max`` (R, 6). -> (legs, info).

    Optional (P3U round 2, default off; tools.btexport.core turns them on for tools.retime input):
      ``forbid`` (n,) bool  instants where no cut may be placed (an arm moving inside a weld operation); a weld whose
                            window has only forbidden instants is listed in ``info["cut_in_weld_zone"]`` (the caller
                            falls back to the window export);
      ``zone`` (n, R) bool + ``zone_afrac``  acceleration limit ``zone_afrac`` x a_max for arm r inside its weld zone
                            (umarell's weld rule 0.078: the braking into a cut keeps the weld (v, a) rule there; the
                            clock never runs faster than the input, so the velocity cap holds anyway);
      ``jerk_cap``          within-leaf jerk limit (a_max per s, 0.05 s third difference): where the braking into /
                            the start from a cut exceeds it, the acceleration limit is lowered locally and the warp
                            re-run (the bang-bang braking of the phase-plane warp becomes a ramp)."""
    import execute as X  # noqa: F401
    from player.timeline import Leg
    t = np.asarray(t, float)
    q = np.asarray(q, float)
    R = q.shape[1]
    rids = list(robot_ids)
    dt = float(np.median(np.diff(t)))
    if np.abs(np.diff(t) - dt).max() > 1e-6:                     # resample to a uniform grid
        tu = np.arange(t[0], t[-1] + 1e-9, dt)
        q = np.stack([np.stack([np.interp(tu, t, q[:, r, j]) for j in range(6)], -1) for r in range(R)], 1)
        if forbid is not None:
            forbid = np.interp(tu, t, np.asarray(forbid, float)) > 0
        if zone is not None:
            zone = np.stack([np.interp(tu, t, np.asarray(zone, float)[:, r]) > 0 for r in range(R)], 1)
        t = tu
    a_max = np.asarray(a_max, float).reshape(R, 6)
    win = windows(t, q, weld_events, rids)
    vel = np.abs(np.gradient(q, t, axis=0))
    Qf = q.reshape(len(t), R * 6)
    peak = (acc_peak(t, Qf) / a_max.reshape(-1)).reshape(R, 6)
    target = np.full((R, 6), acc_frac) if acc_mode == "strict" else np.maximum(acc_frac, peak)
    cost = (vel / (target * a_max)).max(axis=(1, 2))
    cut_of, cuts, bad = choose_cuts(win, cost, forbid)
    cuts = [c for c in cuts if 0 < c < len(t) - 1]
    M = (len(t) - 1) * sub + 1
    base0 = np.broadcast_to((target * a_max).reshape(-1), (M, R * 6))
    base = base0.copy()
    if zone is not None and zone_afrac is not None:
        zf = np.repeat(np.asarray(zone, bool), 6, axis=1)                       # (n, D)
        seg = np.minimum(np.arange(M) // sub, len(t) - 1)
        zfine = zf[seg] | zf[np.minimum(seg + 1, len(t) - 1)]
        base = np.where(zfine, np.minimum(base, float(zone_afrac) * np.broadcast_to(a_max.reshape(-1), base.shape)),
                        base)
    soft = np.ones(M)                       # local softening factor of the acceleration limit (jerk iteration)
    bounds_f = [0] + [c * sub for c in cuts] + [M - 1]
    scale, it, jit, jmax, jhist = 1.0, 0, 0, None, []
    s = tau = qf = u = None
    for jit in range(max_jerk_iter if jerk_cap else 1):
        for it in range(max_iter):
            Aeff = base * soft[:, None] * scale
            # the zone limit applies to the clock's own acceleration (braking / restarting, the u' term); the
            # curvature bound keeps the unreduced limit (the input already obeys the weld rule in its zones)
            s, tau, qf, u = warp(t, Qf, cuts, Aeff, sub=sub, a_mvc=base0)
            check_clock(tau, t[-1] - t[0], "legs_sync_rest_n")
            got = acc_peak(tau, qf).reshape(R, 6) / a_max
            over = float(np.max(got / target))
            if over <= 1.05:
                break
            if it < max_iter - 1:
                scale /= over * 1.02
        if not jerk_cap:
            break
        c, jr = leaf_jerk(tau, qf, bounds_f, a_max.reshape(-1))
        jmax = float(jr.max()) if len(jr) else 0.0
        jhist.append(round(jmax, 3))
        if jmax <= jerk_cap * 1.03:
            break
        # soften where the warp (u < 1 anywhere in the stencil c +- 1.5 h: the braking onset counts) makes the
        # jerk: +-1.5 h (+20 %) around every offending stencil centre
        uf = np.min([np.interp(c + x * 0.05, tau, u) for x in (-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5)], axis=0)
        hot = (jr > jerk_cap * 1.0) & (uf < 0.999)
        if not hot.any():
            break                                       # the input's own jerk (u = 1): nothing the warp can change
        sc = np.interp(c[hot], tau, s)
        w = np.interp(c[hot] + 0.09, tau, s) - sc, sc - np.interp(c[hot] - 0.09, tau, s)
        fac = np.clip(jerk_cap / jr[hot] * 0.92, 0.25, 0.95)
        prev = soft.copy()
        for x, wp, wm, f in zip(sc, w[0], w[1], fac):
            i0, i1 = np.searchsorted(s, x - wm - 1e-12), np.searchsorted(s, x + wp + 1e-12)
            soft[i0:i1] = np.minimum(soft[i0:i1], prev[i0:i1] * f)
    ii = np.arange(len(s))
    near = u < 0.25
    keep = ((ii % sub) == 0) | ((np.abs(u - 1.0) > 1e-3) & (ii % 4 == 0)) | near | np.r_[near[1:], False] | \
        np.r_[False, near[:-1]]
    if jerk_cap:                                 # keep the softened ramps resolved (every 2nd fine point)
        keep |= (soft < 1.0) & (ii % 2 == 0)
    kidx = np.flatnonzero(keep)
    qf = qf.reshape(len(s), R, 6)
    bounds = [0] + [c * sub for c in cuts] + [len(s) - 1]
    weld_at = {(win[i][0], c * sub): i for i, c in cut_of.items()}
    legs, shift = [], 0.0
    for k in range(len(bounds) - 1):
        f0, f1 = bounds[k], bounds[k + 1]
        idx = kidx[(kidx >= f0) & (kidx <= f1)]
        ts = tau[idx]
        last = k == len(bounds) - 2
        step = []
        for ri, rid in enumerate(rids):
            kind, op, hold = "move", f"{rid.upper()}_S{k:02d}", 0.0
            wi = weld_at.get((rid, f1))
            if wi is not None:
                kind, op, hold = "weld", win[wi][1], hold_s
            if last and home_last:
                kind, op, hold = "home", "home", 0.0
            leg = Leg(robot=rid, op_id=op, kind=kind, t0=float(ts[0] + shift), t1=float(ts[-1] + shift),
                      times=ts + shift, positions=qf[idx, ri].copy(),
                      phase=np.array(["transit"] * len(ts), dtype="<U16"), hold_s=hold, vscale=1.0,
                      operation_type="" if kind == "home" else ("weld" if kind == "weld" else "move"),
                      node_type="MoveHome" if kind == "home" else "ExecuteTrajectory",
                      bb_key=f"traj_{rid}_n{k}" if kind == "home" else "")
            leg.t_done = leg.t1 + leg.hold_s + margin
            step.append(leg)
        legs += step
        shift += max(l.hold_s for l in step) + margin if not last else 0.0
    info = {"n_cuts": len(cuts), "n_welds": len(win),
            "uncut": [win[i][1] for i, c in cut_of.items() if not (0 < c < len(t) - 1)],
            "acc_mode": acc_mode, "acc_frac": acc_frac, "input_acc_peak_frac": _r(peak.max(), 3),
            "warp_scale": _r(scale, 4), "warp_iterations": it + 1, "warped_motion_s": _r(tau[-1], 3),
            "rollout_motion_s": _r(t[-1] - t[0], 3), "slowdown_s": _r(tau[-1] - (t[-1] - t[0]), 3),
            "waypoints_per_arm": int(len(kidx)), "method": "tools.btexport.nsync (R-arm sync_rest)"}
    if forbid is not None:
        info["cut_in_weld_zone"] = sorted({win[i][1] for i in bad})
        info["forbidden_instant_frac"] = _r(float(np.mean(forbid)), 4)
    if zone is not None and zone_afrac is not None:
        info["zone_afrac"] = zone_afrac
    if jerk_cap:
        info.update(jerk_cap_amax_per_s=jerk_cap, leaf_jerk_max_amax_per_s=_r(jmax, 3), jerk_iterations=jit + 1,
                    jerk_history=jhist, softened_frac=_r(float(np.mean(soft < 1.0)), 4),
                    soft_min=_r(float(soft.min()), 3))
    return legs, info
