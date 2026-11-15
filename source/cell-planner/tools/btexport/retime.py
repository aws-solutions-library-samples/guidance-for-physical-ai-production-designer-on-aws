"""Speed profiles of the exporter.

``umarell`` (default): the source's own timing. Rollouts are exported with execute.py's ``sync_rest`` time warp
(0.49 a_max envelope, the clock never runs faster than the rollout); plans (umarell, trajgen) keep their legs.

``full``: every non-weld motion as fast as the URDF velocity limits and the acceleration limits allow, accelerating
to and braking from that speed (no instantaneous speed switch); weld operations stay slow. Implemented as a
common-clock phase-plane time scaling (TOPP style, forward / backward passes) of the source path:
  * ONE clock for all arms of a segment, so the configurations that occur together are exactly the source's (the
    arm-arm clearance along the path is unchanged; only the timing changes);
  * velocity: |dq/dtau| <= v_max (URDF ``<limit velocity>``, tools.btexport.limits) per joint, exact for the
    piecewise-linear interpolation umarell's controller / validators use (segment slopes);
  * acceleration: |q' u'/2| + |q''| max(u, sqrt(u)) <= A (u = (ds/dtau)^2; the max() covers both the smooth term
    q'' u and the 20 Hz kinks, which a 0.05 s difference metric sees as |q''| sqrt(u)); A = acc_frac x a_max with
    a_max from the per-robot override (KR240 default [3,3,3,6,6,8], umarell REVIEW value). The result is re-checked
    on a dense grid and A is reduced until the check passes (execute.legs_sync_rest's loop);
  * weld operations: the approach descent (the samples before the arm reaches the weld with its TCP within
    ``weld_radius``, default 0.10 m = umarell's approach / retract offset, of THAT weld), the dwell and the lift-off
    (the samples after it leaves, within the radius), and waypoints with phase "approach" / "retract": while the
    welding arm moves there, each of its joints stays <= ``weld_vfrac`` x v_max (default 0.28 = umarell's approach
    speed, velocity scale 0.7 x approach fraction 0.4); optionally (``weld_source_cap``) also never faster than the
    source. The hold itself is the executor's hold_duration_s (0.5 s) with the arm at rest at the end of its leg;
  * rest points (executor pauses, step ends and every leg's own start / end): u = 0, so every arm brakes to rest
    there; per joint, the speed next to one is also bounded by its braking distance on its own path (sqrt(2 A L),
    ``rest_brake_bound``), and the exported waypoints within REST_RAMP_S of a leg's ends are exact samples of the
    warped curve every REST_RAMP_DT (``warp_s_at``), so the executor's linear interpolation starts and ends each leg
    at rest (btexport's rest check, 3% v_max over 5 ms);
  * where no arm moves, the clock keeps the source time (u <= 1), and it never runs more than ``u_max`` (25, i.e.
    5x) faster than the source, so waypoints keep distinct time stamps.

If ``tools.retime`` (track SPEED) is importable and exposes ``common_clock_warp`` with this signature, it is used
instead (``backend()``); otherwise this module's implementation.
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

WELD_VFRAC = 0.28          # umarell approach speed: max_velocity_scaling 0.7 x approach_velocity_fraction 0.4
WELD_RADIUS = 0.10         # umarell approach / retract offset (m)
WELD_AFRAC = WELD_VFRAC ** 2   # umarell weld acceleration rule (accel = vscale^2): 0.078 a_max
MASK_PAD = 0.005           # the warp's weld window is this much wider than the radius (the stroke starts AT 0.10 m)
U_MAX = 25.0               # clock at most 5x faster than the source
REST_RAMP_S = 0.020        # exported waypoints within this much of a leg's start / end lie exactly on the warped curve
REST_RAMP_DT = 0.0025      # ... spaced this far apart (the executor interpolates linearly between waypoints)


def backend():
    try:
        import tools.retime as R   # noqa: F401  (track SPEED)
        fn = getattr(R, "common_clock_warp", None)
        if callable(fn):
            return fn, "tools.retime.common_clock_warp"
    except Exception:
        pass
    return common_clock_warp, "tools.btexport.retime.common_clock_warp"


def _fine_grid(t: np.ndarray, sub: int):
    n = len(t)
    frac = np.arange(sub) / sub
    s = (t[:-1, None] + (t[1:] - t[:-1])[:, None] * frac[None]).ravel()
    s = np.concatenate([s, t[-1:]])
    seg = np.minimum(np.arange(len(s)) // sub, n - 2)
    return s, seg


def warp_s_at(s, tau, u, x):
    """Source clock s at new-clock instants ``x`` of a warp (s, tau, u) from ``common_clock_warp``: inside each
    fine interval ds/dtau runs linearly in tau (constant path acceleration, the profile the forward / backward
    passes bound by A), so s(tau) is the exact quadratic there, not a straight line; leaving rest (u = 0) the arm
    accelerates as the warp says instead of jumping to the interval's mean speed."""
    s, tau, x = np.asarray(s, float), np.asarray(tau, float), np.atleast_1d(np.asarray(x, float))
    su = np.sqrt(np.maximum(np.asarray(u, float), 0.0))
    i = np.clip(np.searchsorted(tau, x, side="right") - 1, 0, len(tau) - 2)
    dtau = np.maximum(tau[i + 1] - tau[i], 1e-12)
    dt = np.clip(x - tau[i], 0.0, dtau)
    out = s[i] + su[i] * dt + 0.5 * (su[i + 1] - su[i]) / dtau * dt * dt
    return np.clip(out, s[i], s[i + 1])


def rest_ramp_times(tau0: float, tau1: float, ramp: float = REST_RAMP_S, dt: float = REST_RAMP_DT) -> np.ndarray:
    """New-clock instants of the waypoints next to a leg's rest ends (start ``tau0``, end ``tau1``): every ``dt``
    within ``ramp`` of either end (at most to the leg's middle). With linear interpolation between them, the speed
    the executor plays over the first / last 5 ms is the warped curve's, a x 5 ms / 2 <= 0.02 v_max at the KR 240's
    accelerations (btexport's rest check: 0.03), where one coarse first segment (14 ms at 11.5 rad/s^2 from rest:
    0.08 rad/s, 0.044 v_max) was not."""
    half = 0.5 * (tau1 - tau0)
    k = np.arange(dt, min(ramp, half - 0.5 * dt) + 1e-12, dt)
    return np.concatenate([tau0 + k, tau1 - k[::-1]]) if len(k) else np.zeros(0)


def rest_brake_bound(qf, ad1, seg_slope, As, rest) -> np.ndarray:
    """Upper bound on u (M,) from the braking distance to the rest points: a joint that moves without a pause from
    a rest point (or up to one) has speed <= sqrt(2 A L), L = its path length to that rest point on its own
    piecewise-linear path. The 0.05 s curvature term of ``common_clock_warp`` smooths a kink out; at a source leg's
    end (U3: a long segment, then a ~3 ms tail of tiny steps) it let the arm reach the tail at speed and stop inside
    it (0.04 v_max over the executor's last 5 ms). ``qf`` (M, D) fine samples, ``ad1`` (M, D) |dq/ds| there,
    ``seg_slope`` (M-1, D) dq/ds of each fine interval, ``As`` (M, D) acceleration limits, ``rest`` sorted indices.
    A joint that pauses between here and the rest point is not bound by it (inf)."""
    M, D = qf.shape
    cum = np.vstack([np.zeros((1, D)), np.cumsum(np.abs(np.diff(qf, axis=0)), axis=0)])
    still = np.abs(seg_slope) <= 1e-9                                                         # (M-1, D)
    ii = np.arange(M - 1)[:, None]
    last_still = np.maximum.accumulate(np.where(still, ii, -1), axis=0)                       # <= i
    first_still = np.minimum.accumulate(np.where(still, ii, M)[::-1], axis=0)[::-1]           # >= i
    last_still_before = np.vstack([np.full((1, D), -1), last_still])                          # still seg < k
    first_still_from = np.vstack([first_still, np.full((1, D), M)])                           # still seg >= k
    ra = np.asarray(rest)
    k = np.arange(M)
    rb = ra[np.clip(np.searchsorted(ra, k, side="right") - 1, 0, len(ra) - 1)][:, None]
    rf = ra[np.clip(np.searchsorted(ra, k, side="left"), 0, len(ra) - 1)][:, None]
    Lb = np.where((rb <= k[:, None]) & (last_still_before < rb), cum - cum[rb[:, 0]], np.inf)
    Lf = np.where((rf >= k[:, None]) & (first_still_from >= rf), cum[rf[:, 0]] - cum, np.inf)
    L = np.minimum(Lb, Lf)
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.nanmin(np.where((ad1 > 1e-9) & np.isfinite(L), 2.0 * As * L / ad1 ** 2, np.inf), axis=1)


def common_clock_warp(t, q, rest_idx, a_lim, *, v_lim, sub=10, h=0.05, a_mvc=None, slow=None,
                      weld_vfrac: float = WELD_VFRAC, u_max: float = U_MAX, weld_source_cap: bool = False,
                      weld_afrac: Optional[float] = None):
    """Time scaling of a multi-arm path ``q`` (n, R, 6) given at times ``t`` (n,) (any spacing, strictly
    increasing). ``rest_idx``: sample indices where every arm must be at rest. ``a_lim`` / ``a_mvc`` / ``v_lim``:
    (R, 6). ``slow``: (n,) bool, samples inside a weld operation; there each joint of the welding arm stays
    <= ``weld_vfrac`` x v_lim (and, with ``weld_source_cap``, never faster than the source) and, with ``weld_afrac``,
    its acceleration limit is ``weld_afrac`` x a_lim (umarell's weld rule 0.078 = 0.28^2). Returns (s (M,), tau (M,), q (M, R, 6), u (M,)) on
    a grid of ``sub`` points per source interval (s = source clock, tau = new clock)."""
    t = np.asarray(t, float)
    n = len(t)
    q = np.asarray(q, float).reshape(n, -1, 6)
    R = q.shape[1]
    D = R * 6
    Q = q.reshape(n, D)
    s, seg = _fine_grid(t, sub)
    M = len(s)
    interp = lambda x: np.stack([np.interp(np.clip(x, t[0], t[-1]), t, Q[:, k]) for k in range(D)], -1)
    qf = interp(s)
    slope = (Q[1:] - Q[:-1]) / (t[1:] - t[:-1])[:, None]            # (n-1, D) exact segment velocities
    on_knot = (np.arange(M) % sub) == 0
    prev = np.maximum(seg - 1, 0)
    ad1 = np.abs(slope[seg])
    ad1 = np.where(on_knot[:, None], np.maximum(ad1, np.abs(slope[prev])), ad1)
    d2 = (interp(s + h) - 2 * qf + interp(s - h)) / h ** 2
    ad2 = np.abs(d2)
    A = np.asarray(a_lim, float).reshape(D)
    Am = A if a_mvc is None else np.asarray(a_mvc, float).reshape(D)
    V = np.asarray(v_lim, float).reshape(D)
    with np.errstate(divide="ignore", invalid="ignore"):
        u_vel = np.nanmin(np.where(ad1 > 1e-12, (V / ad1) ** 2, np.inf), axis=1)
        c = np.nanmin(np.where(ad2 > 1e-12, Am / ad2, np.inf), axis=1)
    u_acc = np.where(c >= 1.0, c, c * c)
    As = np.broadcast_to(A, (M, D)).copy()                    # per-sample acceleration limit (weld zones lower)
    u = np.minimum(np.minimum(u_vel, u_acc), u_max)
    still = ad1.max(axis=1) < 1e-9
    u[still] = np.minimum(u[still], 1.0)
    if slow is not None:
        sl = np.asarray(slow, bool).reshape(n, -1)
        if sl.shape[1] == 1:
            sl = np.repeat(sl, R, axis=1)
        slf = sl[seg] | sl[np.minimum(seg + 1, n - 1)]                  # (M, R)
        ad1r = ad1.reshape(M, R, 6)
        moving = (ad1r > 1e-9).any(axis=2) & slf                         # (M, R) welding arm moves
        with np.errstate(divide="ignore", invalid="ignore"):
            u_w = np.nanmin(np.where(ad1r > 1e-12, (weld_vfrac * V.reshape(R, 6) / ad1r) ** 2, np.inf), axis=2)
        cap = np.where(moving, np.minimum(1.0, u_w) if weld_source_cap else u_w, np.inf).min(axis=1)
        u = np.minimum(u, cap)
        if weld_afrac is not None:
            fac = np.where(np.repeat(slf, 6, axis=1), float(weld_afrac), 1.0)      # (M, D)
            As = As * fac
            Ams = np.broadcast_to(Am, (M, D)) * fac
            with np.errstate(divide="ignore", invalid="ignore"):
                cw = np.nanmin(np.where(ad2 > 1e-12, Ams / ad2, np.inf), axis=1)
            u = np.minimum(u, np.where(cw >= 1.0, cw, cw * cw))
    rest = sorted({0, M - 1} | {int(k) * sub for k in rest_idx if 0 <= int(k) < n})
    u = np.minimum(u, rest_brake_bound(qf, ad1, slope[seg[:-1]], As, rest))
    u[rest] = 0.0
    g = lambda x: np.maximum(x, np.sqrt(np.maximum(x, 0.0)))
    ds = np.diff(s)
    for i in range(M - 1):
        m = ad1[i] > 1e-9
        if m.any():
            ub = float(np.min(u[i] + 2 * ds[i] * np.maximum(As[i, m] - ad2[i, m] * g(u[i]), 0.0) / ad1[i, m]))
            u[i + 1] = min(u[i + 1], ub)
    for i in range(M - 1, 0, -1):
        m = ad1[i - 1] > 1e-9
        if m.any():
            # the kink / curvature term at the known end point u[i] (an upper estimate of u[i-1] can drive the
            # budget to 0 next to a rest point and freeze the clock there); the dense re-check covers the rest
            ub = np.min(u[i] + 2 * ds[i - 1] * np.maximum(As[i - 1, m] - ad2[i - 1, m] * g(u[i]), 0.0) / ad1[i - 1, m])
            u[i - 1] = min(u[i - 1], float(ub))
    su = np.sqrt(np.maximum(u, 0.0))
    dtau = 2 * ds / np.maximum(su[:-1] + su[1:], 1e-9)
    tau = np.concatenate([[0.0], np.cumsum(dtau)]) + t[0]
    return s, tau, qf.reshape(M, R, 6), u


# ───────────────────────────────────────── weld-operation windows ─────────────────────────────────────────
def tcp_functions(cell_id: Optional[str] = None, scene=None) -> dict:
    """{robot id: f(q (n, 6)) -> TCP (n, 3)} for a umarell cell (env.kinematics) or a trajgen scene."""
    if scene is not None:
        from trajgen.robot import RobotModel
        from trajgen.scene import load_scene
        sc = load_scene(scene)
        out = {}
        for r in sc.robots:
            m = RobotModel(r)
            out[r.id] = (lambda mm: (lambda Q: mm.tcp(np.asarray(Q, float))[0]))(m)
        return out
    from env.cell import load_cell
    from env.kinematics import ArmKinematics
    c = load_cell(cell_id)
    return {rid: (lambda k: (lambda Q: k.tcp(np.asarray(Q, float))[0]))(ArmKinematics(c.robots[rid]))
            for rid in c.robots}


def weld_positions(cell_id: Optional[str] = None, task=None) -> dict:
    if task is not None:
        import json
        d = json.load(open(task))
        return {str(o["id"]): np.asarray(o["position"], float) for o in d["operations"] if o.get("type") == "weld"}
    from env.cell import load_cell
    return {w.wid: w.pos for w in load_cell(cell_id).welds}


def weld_op_mask(q, robot_ids, tcp: dict, events, wpos: dict, radius: float = WELD_RADIUS, phases=None) -> np.ndarray:
    """(n, R): True where arm r is inside a weld operation. ``events``: [(robot, weld_id, k_arrive, k_depart)]
    sample indices of the arm reaching the weld and leaving it (k_arrive <= k_depart; None = outside the samples).
    The operation is the approach descent (the contiguous run of samples before k_arrive with the TCP within
    ``radius`` + 5 mm of THAT weld), the dwell [k_arrive, k_depart] and the lift-off (the run after k_depart within
    ``radius`` + 5 mm). ``phases`` (n, R) strings: waypoints with phase approach / retract count too."""
    q = np.asarray(q, float)
    n = q.shape[0]
    m = np.zeros((n, len(robot_ids)), bool)
    dcache = {}
    for rid, wid, ka, kd in events:
        if wid not in wpos or rid not in robot_ids:
            continue
        r = list(robot_ids).index(rid)
        key = (r, wid)
        if key not in dcache:
            dcache[key] = np.linalg.norm(tcp[rid](q[:, r]) - wpos[wid], axis=1)
        d = dcache[key]
        if ka is not None:
            k = int(np.clip(ka, 0, n - 1))
            m[k, r] = True
            while k - 1 >= 0 and d[k - 1] <= radius + MASK_PAD:
                k -= 1
                m[k, r] = True
        if kd is not None:
            k = int(np.clip(kd, 0, n - 1))
            m[k, r] = True
            while k + 1 < n and d[k + 1] <= radius + MASK_PAD:
                k += 1
                m[k, r] = True
        if ka is not None and kd is not None:
            m[int(np.clip(ka, 0, n - 1)):int(np.clip(kd, 0, n - 1)) + 1, r] = True
    if phases is not None:
        ph = np.asarray(phases)
        m |= (ph == "approach") | (ph == "retract")
    return m


def make_rollout_warp(t, q, robot_ids, v_lim, tcp, weld_events, wpos, *, radius=WELD_RADIUS,
                      weld_vfrac=WELD_VFRAC, info: Optional[dict] = None, weld_source_cap: bool = False,
                      weld_afrac: Optional[float] = None):
    """A ``warp_fn`` for execute.legs_sync_rest (full profile on a two-arm rollout). ``weld_events``: the env's
    [(robot, weld, t_start, t_end)] (t_start: start of the in-tolerance dwell, t_end: weld complete)."""
    t = np.asarray(t, float)
    ev = [(r, w, int(np.searchsorted(t, a - 1e-9)), int(np.searchsorted(t, b - 1e-9))) for r, w, a, b in weld_events]
    slow = weld_op_mask(q, robot_ids, tcp, ev, wpos, radius)
    fn, name = backend()
    if info is not None:
        info.update(backend=name, weld_op_samples=int(slow.any(axis=1).sum()), samples=int(len(slow)),
                    weld_radius_m=radius, weld_vfrac=weld_vfrac, weld_source_cap=bool(weld_source_cap),
                    weld_afrac=weld_afrac)

    def warp(t_, q_, rest_idx, a_lim, sub=10, h=0.05, a_mvc=None):
        # a_lim carries legs_sync_rest's iteration scale; the curve limit is scaled with it (a_mvc ignored)
        return fn(t_, q_, rest_idx, a_lim, v_lim=v_lim, sub=sub, h=h, a_mvc=a_lim, slow=slow, weld_vfrac=weld_vfrac,
                  weld_source_cap=weld_source_cap, weld_afrac=weld_afrac)
    return warp


# ───────────────────────────────────────── per-step retime of a steps plan ─────────────────────────────────────────
def retime_steps(steps: Sequence[Sequence], robot_ids, home, v_lim, a_lim, tcp, wpos, *,
                 margin: float = 1.0, acc_frac: float = 1.0, radius=WELD_RADIUS, weld_vfrac=WELD_VFRAC,
                 sub: int = 10, max_iter: int = 5, dyn_check=None, weld_source_cap: bool = False,
                 weld_afrac: Optional[float] = None):
    """Retime a sync-at-rest plan (list of steps, each a list of legs that start together; every arm at rest at a
    step's start and at the end of its leg) step by step on a common clock per step. Leg ids, kinds, holds and the
    step structure are kept; leg starts are re-derived with the executor rule (step end = max(t1 + hold) + margin).
    Returns (new steps, info)."""
    from player.timeline import Leg, sample_leg
    rids = list(robot_ids)
    R = len(rids)
    home = np.asarray(home, float).reshape(R, 6)
    last = {rid: home[i].copy() for i, rid in enumerate(rids)}
    fn, name = backend()
    out, rows = [], []
    prev_weld = {}
    t_cur = 0.0
    for k, step in enumerate(steps):
        t_step = min(l.t0 for l in step)
        t_end = max(l.t1 for l in step)
        ts = np.unique(np.concatenate([np.asarray(l.times, float) for l in step] + [[t_step, t_end]]))
        ts = ts[(ts >= t_step - 1e-12) & (ts <= t_end + 1e-12)]
        if len(ts) < 2:
            ts = np.array([t_step, t_step + 1e-3])
        Q = np.zeros((len(ts), R, 6))
        ph = np.full((len(ts), R), "", dtype="<U16")
        for i, rid in enumerate(rids):
            leg = next((l for l in step if l.robot == rid), None)
            if leg is None:
                Q[:, i] = last[rid]
                continue
            Q[:, i] = np.where((ts < leg.t0)[:, None], leg.positions[0], sample_leg(leg, ts))
            idx = np.clip(np.searchsorted(leg.times, ts, side="left"), 0, len(leg.times) - 1)
            ph[:, i] = np.where((ts >= leg.t0) & (ts <= leg.t1), leg.phase[idx], "")
        ev = []
        for leg in step:
            if leg.kind == "weld":                                   # arrival at the end of the leg
                ev.append((leg.robot, leg.op_id, int(np.searchsorted(ts, leg.t1 - 1e-9)), None))
            pw = prev_weld.get(leg.robot)
            if pw is not None:                                       # lift-off from the previous weld
                ev.append((leg.robot, pw, None, int(np.searchsorted(ts, leg.t0 - 1e-9))))
        slow = weld_op_mask(Q, rids, tcp, ev, wpos, radius, phases=ph)
        # every leg's own start and end is a rest point: a leg ending (or starting) inside the step, or a source leg
        # whose last sample still moves (U3 ends: ~0.12 rad/s), would otherwise keep that speed on the common clock
        rest = sorted({0, len(ts) - 1} | {int(np.clip(np.searchsorted(ts, x - 1e-9), 0, len(ts) - 1))
                                          for l in step for x in (l.t0, l.t1)})
        scale, it = 0.92, 0
        for it in range(max_iter):
            s, tau, qf, u = fn(ts, Q, rest, a_lim * acc_frac * scale, v_lim=v_lim, sub=sub, a_mvc=a_lim * acc_frac * scale,
                               slow=slow, weld_vfrac=weld_vfrac, weld_source_cap=weld_source_cap,
                               weld_afrac=weld_afrac)
            if dyn_check is None:
                break
            over = dyn_check(tau, qf)
            if over <= 1.02 or it == max_iter - 1:
                break
            scale /= over * 1.02
        tau = tau - tau[0]
        warp_t = lambda x: np.interp(x, s, tau)
        new_step = []
        for leg in step:
            i = rids.index(leg.robot)
            m = (s >= leg.t0 - 1e-9) & (s <= leg.t1 + 1e-9)
            ii = np.arange(len(s))
            du = np.abs(np.diff(u, prepend=u[0])) > 0.01 * np.maximum(u, 1e-9)
            keep = m & (((ii % sub) == 0) | (u < 0.25) | (du & (ii % 2 == 0)))
            idx = np.flatnonzero(keep)
            if len(idx) == 0 or s[idx[0]] > leg.t0 + 1e-9:
                idx = np.r_[int(np.flatnonzero(m)[0]), idx]
            tt = tau[idx]
            sel = np.r_[True, np.diff(tt) > 1e-3]
            sel[-1] = True
            if len(sel) > 1 and tt[-1] - tt[np.flatnonzero(sel)[-2]] <= 1e-3:
                sel[np.flatnonzero(sel)[-2]] = False
            idx, tt = idx[sel], tt[sel]
            sw, pos = s[idx], qf[idx, i]
            # the waypoints next to the rest ends: exact samples of the warped curve every REST_RAMP_DT (they replace
            # the fine-grid points there; the leg's first / last sample stays)
            ramp = rest_ramp_times(float(tt[0]), float(tt[-1]))
            if len(ramp):
                inner = (tt > tt[0]) & (tt < tt[-1]) & (tt - tt[0] > REST_RAMP_S + 0.5 * REST_RAMP_DT) & \
                    (tt[-1] - tt > REST_RAMP_S + 0.5 * REST_RAMP_DT)
                inner[0] = inner[-1] = True
                sr = warp_s_at(s, tau, u, ramp)
                qr = np.stack([np.interp(sr, s, qf[:, i, j]) for j in range(6)], -1)
                tt = np.concatenate([tt[inner], ramp])
                sw = np.concatenate([sw[inner], sr])
                pos = np.concatenate([pos[inner], qr])
                o = np.argsort(tt, kind="stable")
                tt, sw, pos = tt[o], sw[o], pos[o]
            pidx = np.clip(np.searchsorted(leg.times, sw, side="left"), 0, len(leg.times) - 1)
            t0n = t_cur + float(warp_t(leg.t0))
            nl = Leg(robot=leg.robot, op_id=leg.op_id, kind=leg.kind, t0=t0n, t1=t_cur + float(tt[-1]),
                     times=t_cur + tt, positions=pos.copy(), phase=leg.phase[pidx].copy(),
                     hold_s=leg.hold_s, vscale=1.0, operation_type=leg.operation_type, node_type=leg.node_type,
                     bb_key=leg.bb_key)
            nl.times[0] = nl.t0
            nl.t_done = nl.t1 + nl.hold_s + margin
            new_step.append(nl)
            last[leg.robot] = leg.positions[-1].copy()
            prev_weld[leg.robot] = leg.op_id if leg.kind == "weld" else None
        rows.append({"step": k, "src_s": round(t_end - t_step, 4), "new_s": round(float(tau[-1]), 4),
                     "acc_scale": round(scale, 4), "iterations": it + 1, "weld_op_samples": int(slow.any(axis=1).sum())})
        out.append(new_step)
        t_cur = max(l.t_done for l in new_step)
    return out, {"backend": name, "steps": rows, "weld_radius_m": radius, "weld_vfrac": weld_vfrac,
                 "acc_frac": acc_frac, "weld_source_cap": bool(weld_source_cap), "weld_afrac": weld_afrac}


# ───────────────────────────────────────── tools.retime (track SPEED) as the full-speed backend ─────────────────────
def via_tools_retime(src, work, *, profile: str = "full", mode: str = "auto", timeout: int = 7200) -> tuple:
    """Run ``python -m tools.retime --profile <profile>`` (track SPEED, docs/p3_speed.md) on the source's motion and
    return the retimed motion (t, q (N, R, 6) in ``src.robot_ids`` order, weld events, info). Input: rollouts as a
    timeline npz (the rollout arrays + weld events); trajgen runs as their own timeline.npz (any number of robots);
    other plans as the plan JSON (tools.retime replays it on the executor clock). Context: ``--cell`` for umarell
    cells (incl. trajgen runs in a umarell cell), else ``--scene``. A robot override (KWR_ROBOT_OVERRIDE) is inherited
    by the subprocess, so tools.retime reads the same URDF / acceleration limits. Cached: an existing retimed.npz
    whose recorded key (command, override, input md5 and the md5 of every URDF / cell / scene file,
    ``limits.inputs_fingerprint``) equals this one is reused."""
    import json
    import os
    import subprocess
    import sys
    from pathlib import Path
    from player.timeline import timeline_from_arrays
    from trajgen.ntimeline import NTimeline
    work = Path(work)
    work.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[2]
    tlp = (src.meta or {}).get("timeline")
    tlp = None if not tlp else (Path(tlp) if Path(tlp).is_absolute() else root / tlp)
    if src.kind == "rollout" and src.scene is not None and not src.umarell_cell and tlp is not None and tlp.exists():
        inp = tlp                     # an N-robot scene timeline (e.g. tools/ctrl): the file itself, exact legs
    elif src.kind == "rollout" and len(src.robot_ids) != 2:
        from player.timeline import Leg
        inp = work / "source_timeline.npz"
        t_, q_ = np.asarray(src.t, float), np.asarray(src.q, float)
        legs = []
        for r, rid in enumerate(src.robot_ids):
            lg = Leg(robot=rid, op_id=f"ROLLOUT_{rid}", kind="move", t0=float(t_[0]), t1=float(t_[-1]),
                     times=t_.copy(), positions=q_[:, r].copy(), phase=np.array([""] * len(t_), dtype="<U16"))
            lg.t_done = lg.t1
            legs.append(lg)
        NTimeline(t=t_, q=q_, robot_ids=tuple(src.robot_ids), legs=legs, weld_events=list(src.weld_events),
                  meta={"scene": str(src.scene) if src.scene else None, "source": src.spec},
                  home=np.asarray(src.home, float)).save(inp)
    elif src.kind == "rollout":
        inp = work / "source_timeline.npz"
        timeline_from_arrays(src.t, src.q, weld_events=src.weld_events,
                             meta={"cell": src.cell_id, "source": src.spec}).save(inp)
    elif src.run_dir is not None and (Path(src.run_dir) / "timeline.npz").exists():
        inp = Path(src.run_dir) / "timeline.npz"
    else:
        inp = work / "source_plan.json"
        inp.write_text(json.dumps(src.plan))
    out = work / f"retimed_{profile}.npz"
    cmd = [sys.executable, "-m", "tools.retime", "--timeline", str(inp), "--profile", profile, "--mode", mode,
           "--out", str(out)]
    cmd += ["--scene", str(src.scene)] if src.scene is not None and not src.umarell_cell else ["--cell", src.cell_id]
    env = dict(os.environ, PYTHONPATH=str(root))
    import hashlib
    from tools.btexport.limits import inputs_fingerprint
    fp = inputs_fingerprint(scene=src.scene) if src.scene is not None and not src.umarell_cell else \
        inputs_fingerprint(cell_id=src.cell_id)
    from tools.retime.core import RETIME_VERSION
    key = {"cmd": cmd[1:], "override": env.get("KWR_ROBOT_OVERRIDE", ""),
           "input_md5": hashlib.md5(Path(inp).read_bytes()).hexdigest(), "inputs": fp,
           "retime_version": RETIME_VERSION}
    kf = work / f"retimed_{profile}.cmd.json"
    cached = out.exists() and kf.exists() and \
        {k: v for k, v in json.loads(kf.read_text()).items() if k != "inputs_migrated"} == key
    rc = 0
    if not cached:
        kf.unlink(missing_ok=True)
        out.unlink(missing_ok=True)
        p = subprocess.run(cmd, cwd=str(root), capture_output=True, text=True, timeout=timeout, env=env)
        (work / f"retime_{profile}.log").write_text(p.stdout[-20000:] + "\n--- stderr ---\n" + p.stderr[-20000:])
        rc = p.returncode
        if not out.exists():
            raise RuntimeError(f"tools.retime failed (exit {p.returncode}): {p.stderr[-1500:]}")
        kf.write_text(json.dumps(key))
    rep = json.loads(out.with_suffix(".json").read_text()) if out.with_suffix(".json").exists() else {}
    tl = NTimeline.load(out)
    rids = list(tl.robot_ids)
    order = [rids.index(r) for r in src.robot_ids]
    q = np.asarray(tl.q, float)[:, order]
    ev = [(str(e[0]), str(e[1]), float(e[2]), float(e[3])) for e in tl.weld_events]
    return np.asarray(tl.t, float), q, ev, {"cmd": " ".join(cmd[1:]), "returncode": rc, "cached": cached,
                                            "out": str(out), "input": str(inp),
                                            "report": {k: rep.get(k) for k in ("mode", "makespan_s", "last_weld_s",
                                                                               "source_end_s", "source_last_weld_s",
                                                                               "sync", "safety", "dynamics",
                                                                               "limits", "cfg", "wall_s")}}
