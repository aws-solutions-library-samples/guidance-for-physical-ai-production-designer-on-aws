"""Speed-profile retimer for any controller's timeline (N robots, one shared clock). docs/p3_speed.md.

Pipeline (``retime``):

1. **Per-arm path.** Each arm's motion is read from the timeline (grid samples plus the exact leg samples) and split
   at its rests (holds, waits, start, end). A weld hold is the arm's stationary run at the completion pose
   (joints within 1e-6 rad; for controllers that creep, the run with the TCP within 0.5 mm, flattened to the
   completion pose). Every moving piece is fitted by track K's quintic P-spline with rest-to-rest ends
   (tools.ctrl.smooth.fit_interval: q' = q'' = 0 at both ends, within 0.5 mrad of the source polyline), so the
   geometry is C2 and the jerk is bounded.
2. **Labels.** Along each piece the TCP decides the segment kind: ``approach`` (the end of a piece that runs into a
   hold, while the TCP is within ``approach_offset_m`` of that weld = umarell's approach_offset_m 0.10 m), ``liftoff``
   (the start of a piece that leaves a hold, within the offset of that weld), ``home`` (the remaining transit after
   the arm's last weld, when the profile has a home rule) and ``transit`` (everything else). approach, hold and
   liftoff are the weld operations.
3. **Time scaling** (tools.retime.topp): per grid point limits = fraction x URDF velocity / configured acceleration
   (and jerk) per segment kind; TOPP passes accelerate from / brake to the slow weld segments continuously, then an
   S-curve smoothing for the jerk limit.
4. **Shared clock.** ``independent`` mode: every arm runs its pieces back to back (holds last ``hold_s``, other rests
   shrink to zero), then the arm-arm clearance is checked densely (umarell sphere gate, plus MuJoCo hulls near
   contact) and where the new relative timing breaks it, a synchronisation wait is inserted at a rest point of one
   arm (an arm at rest = a safe point for that arm; the wait is accepted only if the whole remaining timeline is
   clear again). ``coupled`` mode: one time map for all arms (the source pairing of the arms' configurations is
   kept exactly, safe by construction up to the spline deviation; a weld zone of one arm slows the others).
   ``auto``: independent, falling back to coupled if the repair fails.
5. **Re-check** (``check_result``): the dense safety re-check of the context (tools/retime/context.py) on the
   retimed motion at ``check_hz`` and on the source, the fair_speed.md validity rule (no new violation kind, no
   inherited one deeper by > 1 mm), and the dynamics maxima against the URDF limits.
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np

from tools.retime.topp import derivs, scale_path

ROOT = Path(__file__).resolve().parents[2]
LABELS = ("transit", "approach", "liftoff", "home", "hold", "rest")
WELD_INNER_M = 0.05          # every sample this close to the weld, contiguous with its hold, is a weld operation
RETIME_VERSION = 3           # bump when the time law / labels / output grid change (tools.btexport's cache key)
                             # 3 (round 2): the hull-buffer + skew-window repair pass (HullSkewConflicts)
L_TRANSIT, L_APPROACH, L_LIFTOFF, L_HOME, L_HOLD, L_REST = range(6)


# ───────────────────────────────────────────── profiles
@dataclass
class Profile:
    name: str
    v: float                 # transit velocity fraction of the URDF v_max
    a: float                 # transit acceleration fraction of a_max
    weld_v: float            # weld operations (approach descent, lift-off): velocity fraction
    weld_a: float
    home_v: float | None     # final return to HOME (None = transit)
    home_a: float | None
    j_frac: float | None     # jerk limit, a_max per second (None = no jerk limit: plain TOPP, bang-bang accel)

    def to_dict(self):
        return asdict(self)


# umarell: max_velocity_scaling 0.7 (accel = scale^2 = 0.49), approach 0.7 x approach_velocity_fraction 0.4 = 0.28
# (accel 0.078), MoveHome / last lift-off home_velocity_scale 0.4 (0.16); jerk 5 a_max/s (track K's umarell profile;
# umarell's own saved plans peak at ~4.5).  full: transit at 1.0 x URDF v_max / 1.0 x a_max, weld operations exactly
# as umarell runs them, jerk 10 a_max/s (K's soft hardware reference; KUKA publishes no jerk limit).
PROFILES = {
    "full": Profile("full", 1.0, 1.0, 0.28, 0.28 ** 2, None, None, 10.0),
    "umarell": Profile("umarell", 0.7, 0.49, 0.28, 0.28 ** 2, 0.4, 0.16, 5.0),
}


@dataclass
class RetimeCfg:
    profile: str = "full"
    v_scale: float | None = None          # override the profile's transit velocity fraction
    a_scale: float | None = None
    weld_v: float | None = None           # override the weld-operation velocity fraction (accel = weld_v^2)
    weld_a: float | None = None           # override the weld-operation acceleration fraction
    jerk: bool = True                     # False: no jerk limit (plain TOPP)
    j_frac: float | None = None
    approach_offset_m: float | None = None    # None: context default (0.10 m)
    hold_s: float = 0.5                   # weld hold (still at the completion pose); umarell hold_duration_s
    mode: str = "auto"                    # auto | independent | coupled
    ds: float = 0.004                     # s (source clock), time-scaling grid
    r_cap: float = 50.0                   # max path rate vs the source clock (conditioning where the source creeps)
    repair_hz: float = 50.0
    fine_hz: float = 400.0                # fine re-check of every repaired schedule (gate without margin)
    repair_margin_m: float = 0.005        # extra clearance above the gate demanded by the repair
    max_repairs: int = 200
    hull_repair: bool = True
    hull_prefilter_m: float = 0.5         # hull check only where the sphere clearance is below this
    repair_prefer: tuple = ("anchored", "makespan", "wait")   # repair strategies tried; the shortest success kept
    repair_budget_s: float = 240.0        # wall-time budget per greedy strategy (anchored first, it is fast)
    hull_skew_repair: bool = True         # round 2 (RETIME_VERSION 3): final pass on the verifier's hull buffer under
                                          # the +-skew window (HullSkewConflicts; scene contexts with a hull buffer)
    hull_skew_hz: float = 100.0           # its grid = the dense re-check's check grid (trajgen.verify at 100 Hz)
    hull_skew_s: float = 0.1              # = the re-check's skew_window_s
    hull_margin_m: float = 0.002          # extra hull distance above hull_min demanded by that pass
    hull_cert_margins_m: tuple = (0.006, 0.015)   # escalated margins at the samples of a pair whose continuous
                                          # certificate fails after the pass (one repair round each)
    geom_refit: bool = True               # round 2: refit a piece whose spline breaks a single-arm rule the source
                                          # holds (joint limits, floor, boxes, box hulls, rule W; scene contexts)
    verbose: bool = False                 # print every sync-repair step to stderr
    out_hz: float = 250.0                 # output grid (4 ms = KUKA RSI cycle)
    still_tol: float = 1e-6
    lenient_tcp_m: float = 0.0005
    tol_geo: float = 5e-4                 # rad, spline fit tolerance (track K default)
    quasi_rest_frac: float = 0.02         # stop points where the source creeps below this fraction of v_max ...
    quasi_rest_min_s: float = 1.0         # ... for at least this long (start / end of the run, and every
    quasi_rest_every_s: float = 3.0       #     quasi_rest_every_s inside it): sync points for creeping controllers

    def resolved(self) -> Profile:
        p = PROFILES[self.profile]
        kw = {}
        if self.v_scale is not None:
            kw.update(v=self.v_scale, a=self.a_scale if self.a_scale is not None else self.v_scale ** 2
                      if self.profile == "umarell" else p.a)
        elif self.a_scale is not None:
            kw.update(a=self.a_scale)
        if self.weld_v is not None:
            kw.update(weld_v=self.weld_v, weld_a=self.weld_v ** 2)
        if self.weld_a is not None:
            kw.update(weld_a=self.weld_a)
        if not self.jerk:
            kw.update(j_frac=None)
        elif self.j_frac is not None:
            kw.update(j_frac=self.j_frac)
        return replace(p, **kw)

    def to_dict(self):
        d = asdict(self)
        d["resolved_profile"] = self.resolved().to_dict()
        return d


# ───────────────────────────────────────────── source
@dataclass
class Source:
    path: str
    robot_ids: tuple
    t: np.ndarray
    q: np.ndarray                         # (N, R, 6) grid samples
    home: np.ndarray
    weld_events: list                     # [(rid, wid, s, e)]
    meta: dict
    tl: object = None                     # NTimeline

    @property
    def last_weld(self):
        return max(e[3] for e in self.weld_events) if self.weld_events else float("nan")

    def polyline(self, r):
        """Exact per-arm polyline: grid times + this arm's leg sample times + weld-event instants."""
        rid = self.robot_ids[r]
        ts = [self.t]
        for l in self.tl.legs:
            if l.robot == rid:
                ts.append(np.asarray(l.times, float))
        ts.append(np.array([x for e in self.weld_events if e[0] == rid for x in (e[2], e[3])], float))
        T = np.unique(np.round(np.concatenate(ts), 9))
        T = T[(T >= self.t[0] - 1e-12) & (T <= self.t[-1] + 1e-12)]
        Q = self.tl.sample(T)[:, r]
        return T, Q


def load_source(path) -> Source:
    """Timeline npz (player.timeline or trajgen NTimeline) or a umarell BT plan json."""
    from trajgen.ntimeline import NTimeline, as_ntimeline
    p = Path(path)
    if p.suffix == ".json":
        from player.bt_plan import timeline_from_bt_plan
        tl = as_ntimeline(timeline_from_bt_plan(str(p)))
        tl.meta.setdefault("source_plan", str(p))
    else:
        tl = NTimeline.load(p)
    ev = [(str(e[0]), str(e[1]), float(e[2]), float(e[3])) for e in tl.weld_events]
    return Source(path=str(p), robot_ids=tuple(tl.robot_ids), t=np.asarray(tl.t, float), q=np.asarray(tl.q, float),
                  home=np.asarray(tl.home, float), weld_events=ev, meta=dict(tl.meta), tl=tl)


# ───────────────────────────────────────────── per-arm path
@dataclass
class Rest:
    a: float                  # source interval
    b: float
    pose: np.ndarray
    kind: str                 # start | hold | wait | end
    welds: list = field(default_factory=list)   # [(wid, e)] holds in this rest
    dur: float = 0.0          # duration on the new clock (before sync waits)


@dataclass
class Piece:
    ta: float
    tb: float
    spline: object
    fit: dict
    s: np.ndarray = None      # grid (source clock)
    lab: np.ndarray = None    # labels per grid point
    D: list = None            # [D1, D2, D3] (n+1, 6)
    res: dict = None          # scale_path result
    T: float = 0.0
    g: object = None          # the same geometry parameterised by joint-space arc length (rad), independent mode
    L: float = 0.0            # arc length (rad)


@dataclass
class ArmPath:
    rid: str
    rests: list               # len(pieces) + 1
    pieces: list
    flattened: list = field(default_factory=list)
    src: tuple | None = None  # (T, Q) the arm's source polyline after hold flattening (geometry checks)


def _still_run(T, Q, k, tol):
    ok = np.abs(Q - Q[k]).max(1) <= tol
    a = b = k
    while a > 0 and ok[a - 1]:
        a -= 1
    while b < len(T) - 1 and ok[b + 1]:
        b += 1
    return a, b


def quasi_rest_points(T, Q, v_max, cfg: RetimeCfg):
    """Vertex indices where the source arm creeps (speed < quasi_rest_frac x v_max for >= quasi_rest_min_s):
    the first and last vertex of each such run and one every quasi_rest_every_s inside it. They become stop points
    (zero-length rests): the retimed arm stops there, so a synchronisation wait can be inserted."""
    dt = np.maximum(np.diff(T), 1e-12)
    vr = (np.abs(np.diff(Q, axis=0)) / dt[:, None] / v_max).max(1)
    slow = vr < cfg.quasi_rest_frac
    moving = np.abs(np.diff(Q, axis=0)).max(1) > 1e-9
    out, k, n = [], 0, len(slow)
    while k < n:
        if slow[k] and moving[k]:
            k0 = k
            while k < n and slow[k] and moving[k]:
                k += 1
            if T[k] - T[k0] >= cfg.quasi_rest_min_s:
                pts = [k0, k]
                t = T[k0] + cfg.quasi_rest_every_s
                while t < T[k] - 0.5 * cfg.quasi_rest_every_s:
                    pts.append(int(np.searchsorted(T, t)))
                    t += cfg.quasi_rest_every_s
                out += pts
        else:
            k += 1
    return sorted(set(out))


def build_arm(src: Source, ctx, r: int, cfg: RetimeCfg) -> ArmPath:
    from tools.ctrl.smooth import SmoothCfg, fit_interval, moving_intervals
    rid = src.robot_ids[r]
    T, Q = src.polyline(r)
    Q = Q.copy()
    evs = sorted([e for e in src.weld_events if e[0] == rid], key=lambda e: e[3])
    flat, forced = [], []
    for (_, w, s, e) in evs:
        k = int(np.argmin(np.abs(T - e)))
        a, b = _still_run(T, Q, k, cfg.still_tol)
        if T[b] - T[a] < 0.05:                     # creeping hold: lenient TCP run, flattened to the completion pose
            P = ctx.tcp(rid, Q)
            ok = np.linalg.norm(P - P[k], axis=1) <= cfg.lenient_tcp_m
            a = b = k
            while a > 0 and ok[a - 1] and T[a - 1] >= s - 1e-9:
                a -= 1
            while b < len(T) - 1 and ok[b + 1] and T[b + 1] <= e + 1e-9:
                b += 1
        dev = float(np.abs(Q[a:b + 1] - Q[k]).max())
        Q[a:b + 1] = Q[k]                          # the hold is exactly still (deviation logged)
        if dev > 1e-9:
            flat.append(dict(weld=w, src=[round(float(T[a]), 4), round(float(T[b]), 4)], max_joint_dev_rad=dev))
        if b == a:
            forced.append(k)                       # no still run at all: a zero-length rest (stop) at the completion
    scfg = SmoothCfg(tol_geo=cfg.tol_geo)
    mov = moving_intervals(T, Q, 1e-9)
    stops = quasi_rest_points(T, Q, ctx.limits[rid].v_max, cfg) if cfg.quasi_rest_frac > 0 else []
    stops = sorted(set(stops) | set(forced))
    if stops:
        out = []
        for (i, j) in mov:
            cut = [k for k in stops if i < k < j and ((T[k] - T[i] >= 0.05 and T[j] - T[k] >= 0.05) or k in forced)]
            b = [i] + cut + [j]
            out += list(zip(b[:-1], b[1:]))
        mov = out
    # rests = complement of the moving intervals
    bounds = [(0, mov[0][0])] if mov else [(0, len(T) - 1)]
    for (i0, j0), (i1, _) in zip(mov[:-1], mov[1:]):
        bounds.append((j0, i1))
    if mov:
        bounds.append((mov[-1][1], len(T) - 1))
    rests = []
    for n, (i, j) in enumerate(bounds):
        a, b = float(T[i]), float(T[j])
        ws = [(w, e) for (_, w, s, e) in evs if a - 1e-6 <= e <= b + 1e-6]
        kind = "hold" if ws else ("start" if n == 0 else ("end" if n == len(bounds) - 1 else ("stop" if i == j else "wait")))
        rests.append(Rest(a=a, b=b, pose=Q[i].copy(), kind=kind, welds=ws))
    # welds that fall inside a moving interval (no still run at all): attach to the nearest following rest
    seen = {w for rs in rests for w, _ in rs.welds}
    for (_, w, s, e) in evs:
        if w not in seen:
            k = min(range(len(rests)), key=lambda n: max(rests[n].a - e, e - rests[n].b, 0.0))
            rests[k].welds.append((w, e))
            rests[k].kind = "hold"
    pieces = []
    for (i, j) in mov:
        sp, info = fit_interval(T, Q, T[i], T[j], scfg)
        if cfg.geom_refit and getattr(ctx, "model", None) is not None:
            sp, info = _refit_new_violations(ctx.model, rid, T, Q, i, j, sp, info, cfg, fit_interval, SmoothCfg)
        pieces.append(Piece(ta=float(T[i]), tb=float(T[j]), spline=sp, fit=info))
    return ArmPath(rid=rid, rests=rests, pieces=pieces, flattened=flat, src=(T, Q))


GEOM_KEYS = ("joint", "floor_sphere", "floor_hull", "box", "box_hull")


def _margins(model, rid, Qx) -> dict:
    """Per rule the worst single-arm margin of configurations Qx (SceneModel.static_margins with box hulls and, when
    the scene enables it, the electrode off-weld rule; rule_W as -(max wedge excess), so < 0 violates everywhere)."""
    el = bool(getattr(model, "electrode_on", False))
    m = model.static_margins(rid, Qx, hull_box=model.box_hull_on, electrode=el)
    out = {k: float(np.min(m[k])) for k in GEOM_KEYS + ("electrode",) if k in m}
    out["rule_W"] = -float(np.max(m["wedge_excess"]))
    return out


def _source_margins(model, rid, T, Q, ta, tb) -> dict:
    """``_margins`` of the source polyline over [ta, tb] (its samples and their midpoints)."""
    i, j = int(np.searchsorted(T, ta - 1e-9)), int(np.searchsorted(T, tb + 1e-9)) - 1
    Ts = T[i:j + 1]
    ts = np.unique(np.r_[Ts, 0.5 * (Ts[:-1] + Ts[1:])])
    return _margins(model, rid, np.stack([np.interp(ts, T, Q[:, k]) for k in range(6)], -1))


def _new_violations(mf: dict, ms: dict) -> dict:
    """{rule: (fit, source)} where the fitted geometry breaks a rule (margin < 0) deeper than the source does."""
    return {k: (a, ms[k]) for k, a in mf.items() if a < -1e-9 and a < ms[k] - 1e-9}


def _geom_new(model, rid, T, Q, i, j, sp):
    """Single-arm rules the fitted piece breaks where the source polyline holds them: {rule: (fit min, source
    min)} on the source samples of the piece and their midpoints."""
    Ts = T[i:j + 1]
    ts = np.unique(np.r_[Ts, 0.5 * (Ts[:-1] + Ts[1:])])
    return _new_violations(_margins(model, rid, sp(ts)), _source_margins(model, rid, T, Q, T[i], T[j]))


def _refit_new_violations(model, rid, T, Q, i, j, sp, info, cfg, fit_interval, SmoothCfg):
    """Round 2 (RETIME_VERSION 3, docs/n2_traj2.md): the P-spline may leave the source polyline by tol_geo (0.5
    mrad), which took two round-1 trajgen timelines 0.07 mm into a box hull / 0.1 mrad past a joint limit where the
    source keeps them by 0.01 mm / 0.1 mrad. Such a piece is refitted with the tolerance divided by 5 (up to 4 times);
    the report keeps what was done."""
    bad = _geom_new(model, rid, T, Q, i, j, sp)
    if not bad:
        return sp, info
    tol, hist = cfg.tol_geo, []
    for _ in range(4):
        hist.append({"tol_geo": tol, "new": {k: [round(a, 7), round(b, 7)] for k, (a, b) in bad.items()}})
        tol /= 5.0
        sp, info = fit_interval(T, Q, T[i], T[j], SmoothCfg(tol_geo=tol))
        bad = _geom_new(model, rid, T, Q, i, j, sp)
        if not bad:
            break
    info = dict(info, geom_refit=hist, geom_refit_ok=not bad, tol_geo_final=tol)
    return sp, info


def arclength_spline(spline, ta, tb, step=0.004, m=20000):
    """Re-parameterise a piece q(t) (source clock) by joint-space arc length: quintic interpolation of the path at
    uniform arc-length spacing ``step`` (rad; arc length by Simpson's rule on ``m`` samples, t(L) by PCHIP). The
    source's own time law (its accelerations / stops inside the piece) then no longer enters D2 / D3, which are
    purely geometric (corners of the source path show up as large curvature, and the time law slows down there)."""
    from scipy.integrate import cumulative_simpson
    from scipy.interpolate import PchipInterpolator, make_interp_spline
    tf = np.linspace(ta, tb, m)
    sig = np.linalg.norm(spline.derivative(1)(tf), axis=1)
    L = np.maximum.accumulate(cumulative_simpson(sig, x=tf, initial=0.0))
    keep = np.r_[True, np.diff(L) > 0]
    tL = PchipInterpolator(L[keep], tf[keep])
    Lt = float(L[-1])
    n = max(12, int(np.ceil(Lt / step)))
    Lu = np.linspace(0.0, Lt, n + 1)
    Qu = spline(np.clip(tL(Lu), ta, tb))
    Qu[0], Qu[-1] = spline(ta), spline(tb)
    g = make_interp_spline(Lu, Qu, k=5)
    g.src_map = (tf[::20].copy(), L[::20].copy())      # source time -> arc length (for the anchored schedule)
    return g, Lt


def _grid(a, b, ds):
    n = max(int(np.ceil((b - a) / ds)), 4)
    return np.linspace(a, b, n + 1), (b - a) / n


def label_piece(ctx, arm: ArmPath, k: int, s, Qs, prof: Profile, offset, mono_tol=0.001, inner=WELD_INNER_M):
    """Segment labels along piece k (grid s, positions Qs): approach = the final stretch into the next hold with the
    TCP within ``offset`` of that weld and approaching it monotonically (``mono_tol`` slack); lift-off = the first
    stretch out of the previous hold within ``offset`` and moving away monotonically; transit (or home) otherwise.
    Inside ``inner`` (5 cm) of the weld the stretch contiguous with the hold is approach / lift-off whether or not it
    is monotonic (P3U round 2: creeping rollouts that wobble in the last centimetres ran that part at transit
    acceleration)."""
    lab = np.full(len(s), L_TRANSIT, np.int8)
    P = ctx.tcp(arm.rid, Qs)
    r0, r1 = arm.rests[k], arm.rests[k + 1]
    holds_after = any(rs.kind == "hold" for rs in arm.rests[k + 1:])
    if prof.home_v is not None and not holds_after and any(rs.kind == "hold" for rs in arm.rests[:k + 1]):
        lab[:] = L_HOME
    if r1.kind == "hold":
        for w, _ in r1.welds:
            if w in ctx.welds:
                d = np.linalg.norm(P - ctx.welds[w], axis=1)
                i, dmax = len(s) - 1, -np.inf
                # walk back from the hold while the TCP is within the offset and still on its way in (walking back,
                # the distance to the weld keeps growing, 1 mm slack): the approach descent
                while i >= 0 and d[i] <= offset and d[i] >= dmax - mono_tol:
                    lab[i] = L_APPROACH
                    dmax = max(dmax, d[i])
                    i -= 1
                while i >= 0 and d[i] <= min(inner, offset):      # non-monotonic wobble close to the sheet
                    lab[i] = L_APPROACH
                    i -= 1
    if r0.kind == "hold":
        for w, _ in r0.welds:
            if w in ctx.welds:
                d = np.linalg.norm(P - ctx.welds[w], axis=1)
                i, dmax = 0, -np.inf
                # walk forward from the hold while within the offset and moving away: the lift-off
                while i < len(s) and d[i] <= offset and d[i] >= dmax - mono_tol and lab[i] != L_APPROACH:
                    lab[i] = L_LIFTOFF
                    dmax = max(dmax, d[i])
                    i += 1
                while i < len(s) and d[i] <= min(inner, offset) and lab[i] != L_APPROACH:
                    lab[i] = L_LIFTOFF
                    i += 1
    return lab


def limit_arrays(lab, lim, prof: Profile):
    """(n, 6) V, A, J for labels ``lab`` and RobotLimits ``lim``."""
    vf = np.full(len(lab), prof.v)
    af = np.full(len(lab), prof.a)
    weld = (lab == L_APPROACH) | (lab == L_LIFTOFF)
    vf[weld], af[weld] = prof.weld_v, prof.weld_a
    if prof.home_v is not None:
        home = lab == L_HOME
        vf[home], af[home] = prof.home_v, prof.home_a
    V = vf[:, None] * lim.v_max[None]
    A = af[:, None] * lim.a_max[None]
    if prof.j_frac is None:
        J = None
    else:
        jb = lim.j_max if lim.j_max is not None else prof.j_frac * lim.a_max
        J = np.broadcast_to(jb[None], V.shape)
    return V, A, J


def zone_labels(lab, s, sd0, T_f, codes=(L_APPROACH, L_LIFTOFF, L_HOME), margin=1.25):
    """Labels for the LIMITS: every weld (and home) zone grown at both ends by the distance the S-curve filter smears
    the speed (margin x T_f x the unfiltered TOPP path speed ``sd0`` at that zone boundary), so that the filtered
    profile is already slow and done braking at the true zone boundary. The true labels are kept for reporting and
    for the check."""
    if T_f <= 0:
        return lab
    out = lab.copy()
    ds = s[1] - s[0]
    for code in codes:
        m = lab == code
        if not m.any():
            continue
        idx = np.flatnonzero(m)
        br = np.flatnonzero(np.diff(idx) > 1)
        for i0, i1 in zip(np.r_[idx[0], idx[br + 1]], np.r_[idx[br], idx[-1]]):
            d0 = margin * T_f * float(sd0[max(i0 - 1, 0)]) + 2 * ds
            d1 = margin * T_f * float(sd0[min(i1 + 1, len(s) - 1)]) + 2 * ds
            g = (s >= s[i0] - d0) & (s <= s[i1] + d1) & (out == L_TRANSIT)
            out[g] = code
    return out


def _filters(prof: Profile):
    """S-curve filter widths: an acceleration step of a_max ramps at the jerk limit over 1 / j_frac s."""
    return (0.0,) if prof.j_frac is None else tuple(x / prof.j_frac for x in (1.0, 2.0, 3.0, 4.0))


def scale_piece(D, s, ds, lab, lims, prof, cap, g=None):
    """Best (fastest valid) time law over the S-curve filter widths. lims: list of (slice, RobotLimits) for the
    coordinates of D (one arm, or every arm on one clock); lab: (n,) or (n, R) labels."""
    from tools.retime.topp import _bounds, _passes
    best = None
    lab2 = lab if lab.ndim == 2 else lab[:, None]
    V0, A0 = np.empty_like(D[0]), np.empty_like(D[0])
    J0 = None if prof.j_frac is None else np.empty_like(D[0])
    for r, (sl, lim) in enumerate(lims):
        v, a, j = limit_arrays(lab2[:, r], lim, prof)
        V0[:, sl], A0[:, sl] = v, a
        if J0 is not None:
            J0[:, sl] = j
    # unfiltered TOPP speed: sets how far the S-curve filter smears the speed into a zone
    sd0 = np.sqrt(np.maximum(_passes(_bounds(D[0], D[1], D[2], V0, A0, J0, cap) ** 2, D[0], D[1], A0, ds), 0.0))
    for T_f in _filters(prof):
        V, A = np.empty_like(D[0]), np.empty_like(D[0])
        J = None if prof.j_frac is None else np.empty_like(D[0])
        for r, (sl, lim) in enumerate(lims):
            lz = zone_labels(lab2[:, r], s, sd0, T_f)
            v, a, j = limit_arrays(lz, lim, prof)        # planning: zones grown by the filter smear
            V[:, sl], A[:, sl] = v, a
            if J is not None:
                J[:, sl] = j
        dfun = None if g is None else (lambda x: tuple(g.derivative(j)(np.clip(x, s[0], s[-1])) for j in (1, 2, 3)))
        res = scale_path(D[0], D[1], D[2], ds, V, A, J, cap=cap, filters_s=(T_f,), check=(V0, A0, J0), dfun=dfun)
        if best is None or (res["ok"] and (not best["ok"] or res["T"] < best["T"])):
            best = res
    return best


def scale_arm(ctx, arm: ArmPath, prof: Profile, cfg: RetimeCfg, dL=0.002):
    """Independent mode: each piece on its arc-length parameter (grid dL rad), at rest at both ends."""
    lim = ctx.limits[arm.rid]
    off = cfg.approach_offset_m if cfg.approach_offset_m is not None else ctx.approach_offset_m
    model = getattr(ctx, "model", None) if cfg.geom_refit else None
    for k, pc in enumerate(arm.pieces):
        pc.g, pc.L = arclength_spline(pc.spline, pc.ta, pc.tb)
        s, ds = _grid(0.0, pc.L, dL)
        Qs = pc.g(s)
        if model is not None and arm.src is not None:
            # round 2: the arc-length re-interpolation is the executed geometry; it must not break a single-arm
            # rule the source holds (2010095: 16 um into a box hull where the source keeps 12 um)
            ms = _source_margins(model, arm.rid, arm.src[0], arm.src[1], pc.ta, pc.tb)
            bad = _new_violations(_margins(model, arm.rid, Qs), ms)
            step, tol, hist = 0.004, cfg.tol_geo, []
            while bad and len(hist) < 6:
                hist.append({"step": step, "tol_geo": tol, "new": {x: [round(a, 7), round(b, 7)] for x, (a, b)
                                                                    in bad.items()}})
                if len(hist) % 2:
                    step /= 4.0                            # finer arc-length interpolation first ...
                else:
                    from tools.ctrl.smooth import SmoothCfg, fit_interval
                    tol /= 5.0                             # ... then a tighter spline
                    pc.spline, pc.fit = fit_interval(arm.src[0], arm.src[1], pc.ta, pc.tb, SmoothCfg(tol_geo=tol))
                pc.g, pc.L = arclength_spline(pc.spline, pc.ta, pc.tb, step=step)
                s, ds = _grid(0.0, pc.L, dL)
                Qs = pc.g(s)
                bad = _new_violations(_margins(model, arm.rid, Qs), ms)
            if hist:
                pc.fit = dict(pc.fit, geom_refit=pc.fit.get("geom_refit", []) + hist, geom_refit_ok=not bad,
                              tol_geo_final=tol, arclength_step=step)
        D = [pc.g.derivative(m)(s) for m in (1, 2, 3)]
        lab = label_piece(ctx, arm, k, s, Qs, prof, off)
        cap = np.full(len(s), 1e3)
        cap[0] = cap[-1] = 0.0                          # rest-to-rest
        res = scale_piece(D, s, ds, lab, [(slice(0, 6), lim)], prof, cap, g=pc.g)
        pc.s, pc.lab, pc.D, pc.res, pc.T = s, lab, D, res, float(res["T"])
    for n, rs in enumerate(arm.rests):
        rs.dur = cfg.hold_s * len(rs.welds) if rs.kind == "hold" else 0.0


def law_at(res, tl):
    """(s, sd, sdd, sddd) of a scale_path result at local times tl (linear on its 1 ms grid)."""
    t = res["t"]
    return tuple(np.interp(tl, t, res[k]) for k in ("s", "sd", "sdd", "sddd"))


# ───────────────────────────────────────────── schedule (independent arms, sync waits)
class Schedule:
    """Per arm: rest_0, piece_0, rest_1, ..., rest_P on the shared clock; ``waits[r][k]`` extra time in rest k,
    ``stretch[r][k]`` >= 1 a uniform slow-down of piece k (its time law run 1/f as fast: v, a, jerk scale by f^-1,
    f^-2, f^-3, so the limits still hold)."""

    def __init__(self, arms):
        self.arms = arms
        self.waits = [np.zeros(len(a.rests)) for a in arms]
        self.stretch = [np.ones(len(a.pieces)) for a in arms]
        self._sig = {}

    def copy(self):
        s = Schedule.__new__(Schedule)
        s.arms, s.waits, s._sig = self.arms, [w.copy() for w in self.waits], self._sig
        s.stretch = [f.copy() for f in self.stretch]
        return s

    def items(self, r):
        """[(kind, idx, t0, t1)] on the shared clock."""
        a, w = self.arms[r], self.waits[r]
        out, t = [], 0.0
        for k, rs in enumerate(a.rests):
            d = rs.dur + w[k]
            out.append(("rest", k, t, t + d))
            t += d
            if k < len(a.pieces):
                T = a.pieces[k].T * self.stretch[r][k]
                out.append(("move", k, t, t + T))
                t += T
        return out

    def end(self, r):
        return self.items(r)[-1][3]

    @property
    def makespan(self):
        return max(self.end(r) for r in range(len(self.arms)))

    def sample(self, ts, r, nu=0, exact=False):
        """Arm r at shared times ts -> list of (len(ts), 6) arrays [q, qd, qdd, qddd][:nu+1] and labels."""
        ts = np.asarray(ts, float)
        a = self.arms[r]
        out = [np.zeros((len(ts), 6)) for _ in range(nu + 1)]
        lab = np.full(len(ts), L_REST, np.int8)
        its = self.items(r)
        for kind, k, t0, t1 in its:
            m = (ts >= t0) & (ts < t1) if (kind, k) != its[-1][:2] else ts >= t0
            if not m.any():
                continue
            if kind == "rest":
                out[0][m] = a.rests[k].pose
                lab[m] = L_REST
                if a.rests[k].kind == "hold":
                    lab[m & (ts < t0 + a.rests[k].dur - 1e-9)] = L_HOLD
                continue
            pc = a.pieces[k]
            f = self.stretch[r][k]
            tl = np.clip(ts[m] - t0, 0.0, pc.T * f) / f
            sg, sd, sdd, sddd = law_at(pc.res, tl)
            sd, sdd, sddd = sd / f, sdd / f ** 2, sddd / f ** 3
            sg = np.clip(sg, 0.0, pc.L)
            out[0][m] = pc.g(sg)
            idx = np.clip(np.round(sg / (pc.s[1] - pc.s[0])).astype(int), 0, len(pc.s) - 1)
            lab[m] = pc.lab[idx]
            if nu >= 1:
                D1, D2, D3 = (pc.g.derivative(j)(sg) for j in (1, 2, 3))
                v, ac, jk = derivs(D1, D2, D3, sd, sdd, sddd)
                for j, x in enumerate((v, ac, jk)[:nu]):
                    out[j + 1][m] = x
        return out, lab

    def sample_all(self, ts, nu=0, exact=False):
        res = [self.sample(ts, r, nu, exact) for r in range(len(self.arms))]
        Q = [np.stack([x[0][j] for x in res], 1) for j in range(nu + 1)]
        return Q, np.stack([x[1] for x in res], 1)

    def weld_events(self):
        ev = []
        for r, a in enumerate(self.arms):
            for kind, k, t0, t1 in self.items(r):
                if kind == "rest" and a.rests[k].kind == "hold":
                    for n, (w, _) in enumerate(a.rests[k].welds):
                        h0 = t0 + n * (a.rests[k].dur / max(len(a.rests[k].welds), 1))
                        ev.append((a.rid, w, round(h0, 6), round(h0 + a.rests[k].dur / len(a.rests[k].welds), 6)))
        return sorted(ev, key=lambda e: e[2])


class Conflicts:
    """Arm-arm conflicts on the shared clock: sphere clearance below the required value of the pair, or (near
    contact) a MuJoCo hull penetration where the source has none."""

    def __init__(self, ctx, robot_ids, req, hz, hull_pairs=(), hull_prefilter=0.5):
        self.ctx, self.rids, self.req, self.hz = ctx, robot_ids, req, hz
        self.pairs = [(i, j) for i in range(len(robot_ids)) for j in range(i + 1, len(robot_ids))]
        self.hull_pairs = set(hull_pairs)
        self.pre = hull_prefilter
        self.n_eval = 0

    def first(self, sched: Schedule, t_from=0.0, t_to=None):
        """First conflict in [t_from, t_to] -> (t, (i, j), clearance, t_episode_end) or None. The episode is the
        contiguous run of conflicting samples of that pair starting at t."""
        self.n_eval += 1
        T = sched.makespan if t_to is None else min(t_to, sched.makespan)
        dt = 1.0 / self.hz
        ts = np.arange(max(0.0, t_from - dt), T + dt, dt)
        (Q,), _ = sched.sample_all(ts)
        best = None
        for (i, j) in self.pairs:
            c = self.ctx.pair_clearance(self.rids[i], self.rids[j], Q[:, i], Q[:, j])
            badm = c < self.req[(i, j)]
            if (i, j) in self.hull_pairs:
                near = np.flatnonzero((c < self.pre) & ~badm)
                if near.size:
                    hc = self.ctx.hull_pair_contact(self.rids[i], self.rids[j], Q[near, i], Q[near, j])
                    badm[near[hc]] = True
            bad = np.flatnonzero(badm)
            if bad.size and (best is None or ts[bad[0]] < best[0]):
                k = bad[0]
                while k + 1 < len(ts) and badm[k + 1]:
                    k += 1
                best = (float(ts[bad[0]]), (i, j), float(c[bad[0]]), float(ts[k]))
        return best


class HullSkewConflicts:
    """Robot-robot hull buffer conflicts the way the dense re-check judges them (trajgen.verify: ``arm_arm_hull`` =
    MuJoCo hull distance >= collision.arm_arm.hull_min_distance_m, ``arm_arm_hull_skew`` = the same with one arm's
    motion shifted by every grid step within +-skew_s), on a ``hz`` grid of the shared clock (round 2, RETIME_VERSION
    3; docs/n2_traj2.md: the repair only knew sphere clearance and hull *contact*, so 3-8 % of trajgen's retimed
    timelines lost the 3 cm buffer or its skew robustness and failed the re-check). ``req[(i, j)]``: required
    distance per pair. A conflict between arm i at time t - d and arm j at t is reported at min(t - d, t); its
    episode runs while some shift still conflicts. Exact MuJoCo distances only where a lower bound (the per-joint-
    frame bounding spheres of the hull covers, SceneModel.hull_groups) is below the requirement."""

    def __init__(self, ctx, robot_ids, req, hz=100.0, skew_s=0.1, cert=None):
        self.ctx, self.rids, self.req, self.hz = ctx, list(robot_ids), dict(req), float(hz)
        self.m = ctx.model
        self.K = int(round(float(skew_s) * self.hz))
        self.pairs = sorted(self.req)
        self.cert = dict(cert) if cert else None   # per pair: threshold of the continuous certificate (hull_min)
        self.n_eval = 0
        self.n_exact = 0
        self.n_cert = 0

    def _centres(self, r, Q):
        F, C, Rr = self.m.hull_groups(self.rids[r])
        return self.m.hull_group_centres(self.rids[r], Q), Rr

    def min_series(self, Q, ts=None, pairs=None, req=None, certify: bool = False):
        """{(i, j): ((len(Q),) min over shifts |m| <= K of the hull distance of arm i at sample k - m vs arm j at k,
        exact below ``req`` (default self.req; lower bounds above), the shift of the minimum, and (``certify`` with
        ``self.cert``) a bool mask of the samples k where the verifier's continuous certificate over the window fails
        (trajgen.verify.timing_tolerance: per-sample bound d - (e_a + e_b) / 2 on the piecewise-linear sample path,
        then trajgen.certify.certify_rects with the band K; round 2: 2030140 kept 32.8 mm at the samples and 29.0 mm
        between them)}."""
        out = {}
        N = len(Q)
        ar = np.arange(N)
        cert = self.cert if certify else None
        if cert is not None:
            from trajgen.certify import interval_disp
            E = {r: self.m.step_margin(self.rids[r], Q[:, r]) for pq in self.pairs for r in pq}
            D = {r: interval_disp(self.m, self.rids[r], Q[:, r]) for r in E}
        for (i, j) in (pairs or self.pairs):
            thr = (req or self.req)[(i, j)]
            tc = None if cert is None else cert[(i, j)]
            Ci, ri = self._centres(i, Q[:, i])
            Cj, rj = self._centres(j, Q[:, j])
            best = np.full(N, np.inf)
            arg = np.zeros(N, int)
            fail = np.zeros(N, bool)
            for m in range(-self.K, self.K + 1):
                ii = np.clip(ar - m, 0, N - 1)
                d = Ci[ii][:, :, None] - Cj[:, None]
                lb = (np.sqrt(np.einsum("nabk,nabk->nab", d, d)) - ri[None, :, None] - rj[None, None]).reshape(N, -1).min(1)
                val = lb.copy()
                if tc is None:
                    near = np.flatnonzero((lb < thr) & (lb < best))
                else:
                    lip = 0.5 * (E[i][ii] + E[j])
                    near = np.flatnonzero((lb < thr) | (lb - lip < tc))
                if len(near):
                    self.n_exact += len(near)
                    val[near] = self.m.mj_pair_min_distance(self.rids[i], self.rids[j], Q[ii[near], i], Q[near, j])
                upd = val < best
                best[upd], arg[upd] = val[upd], m
                if tc is not None:
                    unc = np.flatnonzero((val - lip < tc) & (val >= tc) & ~fail)
                    if len(unc):
                        fail[unc[self._cert_fail(i, j, Q, D, ii[unc], unc, val[unc], tc)]] = True
            out[(i, j)] = (best, arg, fail)
        return out

    def _cert_fail(self, i, j, Q, D, ca, cb, d0, thr) -> np.ndarray:
        """bool mask over the rectangles (ca[k], cb[k]): the certificate fails there (batch first, then halves)."""
        from trajgen.certify import certify_rects
        self.n_cert += 1
        cr = certify_rects(self.m, self.rids[i], self.rids[j], Q[:, i], Q[:, j], ca.astype(float), cb.astype(float),
                           thr, DA=D[i], DB=D[j], d0=d0, band=float(self.K))
        if cr["ok"]:
            return np.zeros(len(ca), bool)
        if len(ca) == 1:
            return np.ones(1, bool)
        h = len(ca) // 2
        return np.concatenate([self._cert_fail(i, j, Q, D, ca[:h], cb[:h], d0[:h], thr),
                               self._cert_fail(i, j, Q, D, ca[h:], cb[h:], d0[h:], thr)])

    def first(self, sched, t_from=0.0, t_to=None):
        self.n_eval += 1
        T = sched.makespan if t_to is None else min(t_to, sched.makespan)
        dt = 1.0 / self.hz
        k0 = max(0, int(np.floor((t_from - (self.K + 1) * dt) * self.hz)))
        k1 = int(np.ceil((T + (self.K + 1) * dt) * self.hz))
        ts = np.arange(k0, k1 + 1) * dt               # the verifier's grid (multiples of 1 / hz)
        (Q,), _ = sched.sample_all(np.minimum(ts, sched.makespan))
        best = None
        for (i, j), (dmin, arg, fail) in self.min_series(Q, certify=True).items():
            bad = (dmin < self.req[(i, j)]) | fail
            if not bad.any():
                continue
            k = np.flatnonzero(bad)
            sh = np.where(fail[k] & (dmin[k] >= self.req[(i, j)]), self.K, np.abs(arg[k]))   # certificate: any shift
            t_lo = np.minimum(ts[k], ts[k] - arg[k] * dt)
            t_hi = np.maximum(ts[k], ts[k] - arg[k] * dt)
            t_lo = np.minimum(t_lo, ts[k] - sh * dt)
            t_hi = np.maximum(t_hi, ts[k] + sh * dt)
            sel = (t_hi >= t_from - 1e-9) & (t_lo <= T + 1e-9)
            if not sel.any():
                continue
            k, t_lo, t_hi = k[sel], t_lo[sel], t_hi[sel]
            a = int(np.argmin(t_lo))
            e = a
            while e + 1 < len(k) and k[e + 1] == k[e] + 1:
                e += 1
            t_ep = float(t_hi[a:e + 1].max())
            c = (max(float(t_lo[a]), 0.0), (i, j), float(dmin[k[a]]), t_ep)
            if best is None or c[0] < best[0]:
                best = c
        return best


class MultiConflicts:
    """The earliest conflict of several conflict checks (``first`` interface of Conflicts)."""

    def __init__(self, checks):
        self.checks = [c for c in checks if c is not None]
        self.hz = max(c.hz for c in self.checks)

    @property
    def n_eval(self):
        return sum(c.n_eval for c in self.checks)

    def first(self, sched, t_from=0.0, t_to=None):
        cs = [c.first(sched, t_from, t_to) for c in self.checks]
        cs = [c for c in cs if c is not None]
        return min(cs, key=lambda c: c[0]) if cs else None


def hull_requirements(src: Source, ctx, cfg, hz=100.0, skew_s=0.1):
    """Per pair: the hull distance the retimed motion must keep under +-skew_s (HullSkewConflicts): hull_min +
    ``cfg.hull_margin_m`` where the source keeps hull_min under the skew window, else the source's own minimum - 1 mm
    (an inherited violation may not get deeper, fair_speed.md). None when the context has no hull buffer."""
    m = getattr(ctx, "model", None)
    if m is None or not m.scene.collision.hull_backstop or len(src.robot_ids) < 2:
        return None, {}
    hmin = float(m.hull_min)
    rids = list(src.robot_ids)
    R = len(rids)
    probe = HullSkewConflicts(ctx, rids, {(i, j): hmin + cfg.hull_margin_m for i in range(R) for j in range(i + 1, R)},
                              hz=hz, skew_s=skew_s)
    ts = np.arange(0, src.t[-1] + 1e-9, 1.0 / hz)
    Q = src.tl.sample(ts)
    req, srcmin = {}, {}
    for pq, (dmin, _, _) in probe.min_series(Q).items():
        sm = float(dmin.min())
        srcmin[pq] = sm
        req[pq] = hmin + cfg.hull_margin_m if sm >= hmin - 1e-9 else sm - 1e-3
    return req, srcmin


def anchor(sched: Schedule):
    """Initial waits that keep the source's order at every rest: arm i leaves its rest k (source time b) only when
    every other arm has reached source time b on the new clock (processed in source order, so each arm's motion up
    to b is already fixed). Between rests the arms move at their own speed."""
    R = len(sched.arms)

    def reach(j, o):
        """new-clock time at which arm j is where it was at source time o."""
        a = sched.arms[j]
        for kind, k, t0, t1 in sched.items(j):
            if kind == "rest":
                rs = a.rests[k]
                if rs.a - 1e-9 <= o <= rs.b + 1e-9:
                    return t0
            else:
                pc = a.pieces[k]
                if pc.ta < o < pc.tb:
                    tf, L = pc.g.src_map
                    sv = float(np.interp(o, tf, L))
                    sl, tl = pc.res["s"], pc.res["t"]
                    keep = np.r_[True, np.diff(sl) > 0]
                    return t0 + float(np.interp(sv, sl[keep], tl[keep])) * sched.stretch[j][k]
        return sched.end(j)

    ev = sorted((a.rests[k].b, i, k) for i, a in enumerate(sched.arms) for k in range(len(a.rests) - 1))
    for b, i, k in ev:
        it = [y for y in sched.items(i) if y[:2] == ("rest", k)][0]
        ready = it[2] + sched.arms[i].rests[k].dur + sched.waits[i][k]
        need = max([reach(j, b) for j in range(R) if j != i] + [ready])
        sched.waits[i][k] += max(0.0, need - ready)
    return sched


def _rest_candidates(sched: Schedule, r, t, n=6):
    its = [x for x in sched.items(r) if x[0] == "rest" and x[2] <= t + 1e-9]
    return [x[1] for x in its[-n:]][::-1]


def repair(sched: Schedule, conf: Conflicts, cfg: RetimeCfg, log, n_rests=6, prefer="makespan", budget_s=None):
    """Insert synchronisation waits at rest points until no conflict remains (or give up).

    For the first conflict (time t*, pair (i, j), conflict episode [t*, t_e]) every candidate is tried, for each
    arm of the pair: a wait at one of its last ``n_rests`` rests that start before t* (an arm at rest is at a safe
    point for itself; waits 0.04 .. 41 s doubling, then bisection), or a uniform slow-down (x1.1 .. x3) of the piece
    it is in at t* or of the one before (a synchronisation without a stop). A candidate is accepted if, from that rest on, the first conflict
    of the modified schedule lies after t_e (the episode is resolved; a later conflict is the next iteration's; only
    the window up to t_e is evaluated). If no candidate resolves the episode, one that moves the first conflict at
    least min(1 s, episode) later is accepted. ``prefer="makespan"``: the candidate with the smallest resulting
    makespan wins (ties: the shorter wait); ``prefer="wait"``: the shortest wait wins (ties: makespan)."""
    ladder = [0.04 * 2 ** k for k in range(11)]           # 0.04 .. 41 s (bisection: >= 0.02 s)
    stretch_ladder = (1.1, 1.25, 1.5, 2.0, 3.0)
    t_start = time.time()
    for it in range(cfg.max_repairs):
        c = conf.first(sched)
        if c is None:
            return True
        if budget_s is not None and time.time() - t_start > budget_s:
            log.append(dict(iter=it, result=f"budget {budget_s:.0f} s exhausted"))
            return False
        t_star, (i, j), clr, t_ep = c
        opts = []
        # criterion 1: the whole conflict episode is resolved; criterion 2 (long episodes, e.g. two arms working
        # side by side for many seconds): the first conflict moves at least min(1 s, episode) later
        for goal in (t_ep + 1.0 / conf.hz, t_star + min(1.0, max(t_ep - t_star, 5.0 / conf.hz))):
            for x in (i, j):
                # slow-down of the piece the arm is in at t* (or the last one before it): a sync without a stop
                pcs = [y for y in sched.items(x) if y[0] == "move" and y[2] <= t_star + 1e-9][-2:]
                for _, k, t_mv, _ in pcs:
                    for fct in stretch_ladder:
                        s2 = sched.copy()
                        s2.stretch[x][k] *= fct
                        c2 = conf.first(s2, t_from=t_mv, t_to=goal + 2.0 / conf.hz)
                        if c2 is None or c2[0] > goal + 1e-9:
                            opts.append((round(s2.makespan, 3), ("stretch", fct), x, k))
                            break
                for k in _rest_candidates(sched, x, t_star, n_rests):
                    t_rest = [y for y in sched.items(x) if y[:2] == ("rest", k)][0][2]

                    def clear(d):
                        s2 = sched.copy()
                        s2.waits[x][k] += d
                        c2 = conf.first(s2, t_from=t_rest, t_to=goal + 2.0 / conf.hz)
                        return (c2 is None or c2[0] > goal + 1e-9), c2
                    for d in ladder:
                        ok, c2 = clear(d)
                        if ok:
                            lo, hi = (d / 2 if d > ladder[0] else 0.0), d
                            for _ in range(4 if d > ladder[0] else 0):
                                mid = 0.5 * (lo + hi)
                                ok3, c3 = clear(mid)
                                if ok3:
                                    hi, c2 = mid, c3
                                else:
                                    lo = mid
                            s2 = sched.copy()
                            s2.waits[x][k] += hi
                            opts.append((round(s2.makespan, 3), ("wait", hi), x, k))
                            break
            if opts:
                break
        if not opts:
            log.append(dict(iter=it, t=round(t_star, 3), pair=[sched.arms[i].rid, sched.arms[j].rid],
                            clearance=round(clr, 4), result="no wait found"))
            return False
        # cost of an option for the "wait" strategy: the added time (wait, or the piece's extra duration)
        cost = lambda o: o[1][1] if o[1][0] == "wait" else sched.arms[o[2]].pieces[o[3]].T * sched.stretch[o[2]][o[3]] * (o[1][1] - 1)  # noqa: E731
        ms, (kind, d), x, k = min(opts, key=lambda o: (o[0], cost(o))) if prefer == "makespan" else \
            min(opts, key=lambda o: (cost(o), o[0]))
        if kind == "wait":
            sched.waits[x][k] += d
        else:
            sched.stretch[x][k] *= d
        if cfg.verbose:
            import sys
            print(f"  repair[{prefer}] {it}: conflict t={t_star:.2f} {sched.arms[i].rid}|{sched.arms[j].rid} "
                  f"clr={clr:.4f} -> {kind} {sched.arms[x].rid} {'rest' if kind == 'wait' else 'piece'} {k} {d:.3f}, makespan {ms:.2f} "
                  f"({conf.n_eval} evals)", file=sys.stderr, flush=True)
        if kind == "wait":
            log.append(dict(iter=it, t=round(t_star, 3), t_episode_end=round(t_ep, 3),
                            pair=[sched.arms[i].rid, sched.arms[j].rid], clearance=round(clr, 4),
                            wait_robot=sched.arms[x].rid, rest_index=int(k), rest_kind=sched.arms[x].rests[k].kind,
                            rest_src_s=round(sched.arms[x].rests[k].a, 3), wait_s=round(d, 4), makespan=float(ms)))
        else:
            pc = sched.arms[x].pieces[k]
            log.append(dict(iter=it, t=round(t_star, 3), t_episode_end=round(t_ep, 3),
                            pair=[sched.arms[i].rid, sched.arms[j].rid], clearance=round(clr, 4),
                            slow_robot=sched.arms[x].rid, piece_index=int(k), piece_src_s=[round(pc.ta, 3),
                                                                                          round(pc.tb, 3)],
                            stretch=round(float(sched.stretch[x][k]), 4),
                            added_s=round(pc.T * float(sched.stretch[x][k]) * (1 - 1 / d), 4), makespan=float(ms)))
    return conf.first(sched) is None


# ───────────────────────────────────────────── coupled mode (one clock for all arms)
def arm_q(arm: ArmPath, sig, nu=0):
    """Arm configuration (nu = 0) or its source-clock derivative (nu >= 1) at source times sig."""
    sig = np.asarray(sig, float)
    out = np.zeros((len(sig), 6))
    if nu == 0:
        for rs in arm.rests:
            m = (sig >= rs.a - 1e-12) & (sig <= rs.b + 1e-12)
            out[m] = rs.pose
        if arm.rests and len(sig):
            out[sig > arm.rests[-1].b] = arm.rests[-1].pose
    for pc in arm.pieces:
        m = (sig > pc.ta) & (sig < pc.tb)
        if m.any():
            out[m] = (pc.spline if nu == 0 else pc.spline.derivative(nu))(sig[m])
    return out


class Coupled:
    """One time map for every arm, so the arms' configurations occur together exactly as in the source (the path
    parameter is a monotone function of the source clock). Same sampling interface as Schedule.

    The source clock is cut into blocks in which at least one arm moves (all arms are at rest at block ends). Inside a
    block the path parameter is the arc length of the STACKED joint vector of all arms (so the source's own time law
    does not enter the path derivatives), plus a virtual coordinate during weld holds; one time law per block
    (rest-to-rest). The all-at-rest gaps between blocks
    get the weld-hold time they contain (``hold_s`` per weld, shared proportionally with the parts of the hold inside
    blocks, where the path rate is capped so that the hold still lasts >= hold_s); other gaps shrink to zero."""

    def __init__(self, src: Source, ctx, arms, prof: Profile, cfg: RetimeCfg, dL=0.002, step=0.004):
        from scipy.integrate import cumulative_simpson
        from scipy.interpolate import PchipInterpolator, make_interp_spline
        self.arms = arms
        R = len(arms)
        off = cfg.approach_offset_m if cfg.approach_offset_m is not None else ctx.approach_offset_m
        iv = sorted((p.ta, p.tb) for a in arms for p in a.pieces)
        blocks = []
        for x0, x1 in iv:
            if blocks and x0 <= blocks[-1][1] + 1e-9:
                blocks[-1][1] = max(blocks[-1][1], x1)
            else:
                blocks.append([x0, x1])
        holds = [(rs.a, rs.b, cfg.hold_s * len(rs.welds)) for a in arms for rs in a.rests if rs.kind == "hold"]
        lims = [(slice(6 * r, 6 * r + 6), ctx.limits[a.rid]) for r, a in enumerate(arms)]
        self.segs = []                     # [(kind, src_a, src_b, t0, t1, res|None)]
        t, prev = 0.0, 0.0
        self.oks, self.overs = [], []
        for (ba, bb) in blocks + [[None, None]]:
            ga, gb = prev, (ba if ba is not None else max([prev] + [h[1] for h in holds]))
            if gb > ga + 1e-12:                    # gap [prev, ba]: all arms at rest
                dur = 0.0
                for h0, h1, need in holds:
                    ov = min(h1, gb) - max(h0, ga)
                    if ov > 0:
                        dur = max(dur, need * ov / max(h1 - h0, 1e-12))
                self.segs.append(("gap", ga, gb, t, t + dur, None))
                t += dur
            if ba is None:
                break
            m = max(4000, int((bb - ba) / 0.0005))
            sf = np.linspace(ba, bb, m)
            dQ = np.concatenate([arm_q(a, sf, 1) for a in arms], 1)
            # a virtual coordinate that advances at 1 (rad per source second) while any arm holds a weld inside the
            # block, so every hold spans a path interval on which the rate cap below keeps it >= hold_s
            hm = np.zeros(len(sf))
            for h0, h1, need in holds:
                hm[(sf >= h0) & (sf <= h1)] = 1.0
            L = np.maximum.accumulate(cumulative_simpson(np.sqrt((dQ ** 2).sum(1) + hm ** 2), x=sf, initial=0.0))
            Lt = float(L[-1])
            keep = np.r_[True, np.diff(L) > 0]
            sig_of = PchipInterpolator(L[keep], sf[keep])
            nu_ = max(12, int(np.ceil(Lt / step)))
            su = np.linspace(0.0, Lt, nu_ + 1)
            sgu = np.clip(sig_of(su), ba, bb)
            sgu[0], sgu[-1] = ba, bb
            Qu = np.concatenate([arm_q(a, sgu) for a in arms], 1)
            g = make_interp_spline(su, Qu, k=5)
            S, ds = _grid(0.0, Lt, dL)
            D = [g.derivative(j)(S) for j in (1, 2, 3)]
            sigS = np.clip(sig_of(S), ba, bb)
            cap = np.full(len(S), 1e3)
            cap[0] = cap[-1] = 0.0
            lab = np.full((len(S), R), L_TRANSIT, np.int8)
            for r, a in enumerate(arms):
                for k, pc in enumerate(a.pieces):
                    mm = (sigS >= pc.ta) & (sigS <= pc.tb)
                    if mm.any():
                        lab[mm, r] = label_piece(ctx, a, k, sigS[mm], pc.spline(sigS[mm]), prof, off)
            for h0, h1, need in holds:
                if h1 <= ba or h0 >= bb:
                    continue
                s0, s1 = np.interp([max(h0, ba), min(h1, bb)], sf, L)
                frac = (min(h1, bb) - max(h0, ba)) / max(h1 - h0, 1e-12)
                i0 = int(np.clip(np.floor(s0 / ds + 1e-9), 0, len(S) - 1))
                i1 = int(np.clip(np.ceil(s1 / ds - 1e-9), 0, len(S) - 1))
                if i1 > i0:                        # the grid cells covering the hold, traversed in >= its share
                    mm = np.zeros(len(S), bool)
                    mm[i0:i1 + 1] = True
                    mm &= cap > 0
                    cap[mm] = np.minimum(cap[mm], (S[i1] - S[i0]) / max(need * frac, 1e-9))
            res = scale_piece(D, S, ds, lab, lims, prof, cap, g=g)
            self.oks.append(bool(res["ok"]))
            self.overs.append(float(res["over"]))
            res.update(lab=lab, ds=ds, g=g, L=L, sf=sf, sig_of=sig_of)
            self.segs.append(("block", ba, bb, t, t + res["T"], res))
            t += res["T"]
            prev = bb
        self.T = t
        self.res = dict(ok=all(self.oks), over=max(self.overs or [0.0]))
        self.waits = [np.zeros(len(a.rests)) for a in arms]

    @property
    def makespan(self):
        return self.T

    def _law(self, ts):
        """-> per sample: source time, block index (-1 in gaps), path s, sd, sdd, sddd."""
        ts = np.asarray(ts, float)
        sg = np.zeros(len(ts))
        blk = np.full(len(ts), -1)
        S, sd, sdd, sddd = (np.zeros(len(ts)) for _ in range(4))
        for n, (kind, a, b, t0, t1, res) in enumerate(self.segs):
            m = (ts >= t0) & (ts <= t1 + 1e-12)
            if not m.any():
                continue
            if kind == "gap":
                sg[m] = a + (b - a) * np.clip((ts[m] - t0) / max(t1 - t0, 1e-12), 0, 1) if t1 > t0 else b
                blk[m] = -1
            else:
                x = law_at(res, np.clip(ts[m] - t0, 0, res["T"]))
                S[m], sd[m], sdd[m], sddd[m] = x
                sg[m] = np.clip(res["sig_of"](x[0]), a, b)
                blk[m] = n
        last = ts > self.T
        if last.any():
            sg[last] = self.segs[-1][2]
            blk[last] = -1
        return sg, blk, S, sd, sdd, sddd

    def tau_of(self, sig):
        """new-clock time at source time sig."""
        sig = np.atleast_1d(np.asarray(sig, float))
        out = np.zeros(len(sig))
        for kind, a, b, t0, t1, res in self.segs:
            m = (sig >= a - 1e-12) & (sig <= b + 1e-12)
            if not m.any():
                continue
            if kind == "gap":
                out[m] = t0 + (t1 - t0) * np.clip((sig[m] - a) / max(b - a, 1e-12), 0, 1)
            else:
                sv = np.interp(sig[m], res["sf"], res["L"])
                ss, tv = res["s"], res["t"]
                keep = np.r_[True, np.diff(ss) > 0]
                out[m] = t0 + np.interp(sv, ss[keep], tv[keep])
        return out

    def sample(self, ts, r, nu=0, exact=True):
        ts = np.asarray(ts, float)
        a = self.arms[r]
        sg, blk, S, sd, sdd, sddd = self._law(np.clip(ts, 0, self.T))
        out = [np.zeros((len(ts), 6)) for _ in range(nu + 1)]
        out[0] = arm_q(a, sg)
        lab = np.full(len(ts), L_REST, np.int8)
        for rs in a.rests:
            if rs.kind == "hold":
                lab[(sg >= rs.a - 1e-12) & (sg <= rs.b + 1e-12)] = L_HOLD
        sl = slice(6 * r, 6 * r + 6)
        for n in np.unique(blk[blk >= 0]):
            res = self.segs[n][5]
            m = blk == n
            moving = np.zeros(len(ts), bool)
            for pc in a.pieces:
                moving |= (sg > pc.ta) & (sg < pc.tb)
            mm = m & moving
            if mm.any():
                # positions from the same stacked path g(S) whose derivatives the time law limits (P3U round 2: the
                # per-arm splines at sig_of(S) differ from g by <= 0.4 mrad, but sig_of is only C1, so their 4 ms
                # finite-difference acceleration reached 1.1-1.6 a_max); rests keep the exact rest / weld pose
                out[0][mm] = res["g"](S[mm])[:, sl]
            if nu >= 1 and m.any():
                g = res["g"]
                D1, D2, D3 = (g.derivative(j)(S[m])[:, sl] for j in (1, 2, 3))
                v, ac, jk = derivs(D1, D2, D3, sd[m], sdd[m], sddd[m])
                for j, x in enumerate((v, ac, jk)[:nu]):
                    out[j + 1][m] = x
            if mm.any():
                k = np.clip(np.round(S[mm] / res["ds"]).astype(int), 0, len(res["lab"]) - 1)
                lab[mm] = res["lab"][k, r]
        return out, lab

    def sample_all(self, ts, nu=0, exact=True):
        res = [self.sample(ts, r, nu, exact) for r in range(len(self.arms))]
        Q = [np.stack([x[0][j] for x in res], 1) for j in range(nu + 1)]
        return Q, np.stack([x[1] for x in res], 1)

    def weld_events(self):
        ev = []
        for a in self.arms:
            for rs in a.rests:
                if rs.kind == "hold":
                    h0, h1 = (float(x) for x in self.tau_of([rs.a, rs.b]))
                    per = (h1 - h0) / len(rs.welds)
                    for n, (w, _) in enumerate(rs.welds):
                        ev.append((a.rid, w, round(h0 + n * per, 6), round(h0 + n * per + per, 6)))
        return sorted(ev, key=lambda e: e[2])


# ───────────────────────────────────────────── driver
@dataclass
class Result:
    src: Source
    ctx: object
    cfg: RetimeCfg
    profile: Profile
    mode: str
    plan: object              # Schedule | Coupled
    arms: list
    info: dict

    @property
    def makespan(self):
        return self.plan.makespan

    def weld_events(self):
        return self.plan.weld_events()


def source_requirements(src: Source, ctx, cfg: RetimeCfg, hz=100.0):
    """Per pair: required clearance (gate + margin, but never more than the source's own minimum - 1 mm and never
    less than the gate unless the source itself is below it) and whether the source has hull contacts."""
    ts = np.arange(0, src.t[-1] + 1e-9, 1.0 / hz)
    Q = src.tl.sample(ts)
    R = len(src.robot_ids)
    req, srcmin, hull = {}, {}, []
    for i in range(R):
        for j in range(i + 1, R):
            c = ctx.pair_clearance(src.robot_ids[i], src.robot_ids[j], Q[:, i], Q[:, j])
            m = float(c.min())
            srcmin[(i, j)] = m
            g = ctx.gate
            req[(i, j)] = max(g + 1e-4, min(g + cfg.repair_margin_m, m - 1e-3)) if m > g else m - 1e-3
            hull.append((i, j))
    return req, srcmin, hull


def retime(src: Source, ctx, cfg: RetimeCfg | None = None, log_fn=None) -> Result:
    cfg = cfg or RetimeCfg()
    prof = cfg.resolved()
    t0 = time.time()
    missing = [r for r in src.robot_ids if r not in ctx.limits]
    if missing:
        raise ValueError(f"timeline robots {missing} not in the context {ctx.name} ({ctx.robot_ids})")
    arms = [build_arm(src, ctx, r, cfg) for r in range(len(src.robot_ids))]
    info = dict(n_pieces=[len(a.pieces) for a in arms], flattened_holds=[a.flattened for a in arms],
                geom_refits=[{"robot": a.rid, "piece": k, "ok": p.fit.get("geom_refit_ok"),
                              "tol_geo_final": p.fit.get("tol_geo_final")}
                             for a in arms for k, p in enumerate(a.pieces) if "geom_refit" in p.fit],
                fit_tol_met=all(p.fit["tol_met"] for a in arms for p in a.pieces),
                fit_max_dev_geo_rad=round(max([p.fit["dev_geo_rad"] for a in arms for p in a.pieces] or [0.0]), 6))
    req, srcmin, hull_pairs = source_requirements(src, ctx, cfg)
    info["pair_required_clearance_m"] = {f"{src.robot_ids[i]}|{src.robot_ids[j]}": round(v, 4)
                                         for (i, j), v in req.items()}
    info["pair_source_min_clearance_m"] = {f"{src.robot_ids[i]}|{src.robot_ids[j]}": round(v, 4)
                                           for (i, j), v in srcmin.items()}
    plan, mode, rep_log = None, None, []
    if cfg.mode in ("auto", "independent"):
        for a in arms:
            scale_arm(ctx, a, prof, cfg)
        info["piece_scaling_ok"] = all(p.res["ok"] for a in arms for p in a.pieces)
        info["piece_scaling_worst_over"] = round(max([p.res["over"] for a in arms for p in a.pieces] or [0.0]), 4)
        info["independent_makespan_before_sync_s"] = round(Schedule(arms).makespan, 3)
        conf = Conflicts(ctx, src.robot_ids, req, cfg.repair_hz,
                         hull_pairs if cfg.hull_repair else (), cfg.hull_prefilter_m)
        # fine pass: the repair samples at repair_hz with a margin; at full speed an arm moves several cm between
        # 50 Hz samples, so every repaired schedule is re-checked at fine_hz against the gate itself (no margin) and
        # repaired further if needed (a schedule that passes the dense re-check is unchanged by this pass)
        req_fine = {pq: (ctx.gate + 1e-6 if srcmin[pq] > ctx.gate else srcmin[pq] - 1e-3) for pq in req}
        conf_fine = Conflicts(ctx, src.robot_ids, req_fine, cfg.fine_hz,
                              hull_pairs if cfg.hull_repair else (), cfg.hull_prefilter_m)
        skew_conf = None
        if cfg.hull_skew_repair and len(arms) > 1:
            req_h, src_h = hull_requirements(src, ctx, cfg, hz=cfg.hull_skew_hz, skew_s=cfg.hull_skew_s)
            if req_h:
                hmin = float(ctx.model.hull_min)
                cert = {pq: min(hmin, v) for pq, v in req_h.items()}      # the verifier's certificate threshold
                # repair on the sampled check (fast); the continuous certificate only checks the result (below)
                skew_conf = HullSkewConflicts(ctx, src.robot_ids, req_h, hz=cfg.hull_skew_hz, skew_s=cfg.hull_skew_s)
                skew_cert = HullSkewConflicts(ctx, src.robot_ids, req_h, hz=cfg.hull_skew_hz, skew_s=cfg.hull_skew_s,
                                              cert=cert)
                info["pair_required_hull_skew_m"] = {f"{src.robot_ids[i]}|{src.robot_ids[j]}": round(v, 4)
                                                     for (i, j), v in req_h.items()}
                info["pair_source_hull_skew_min_m"] = {f"{src.robot_ids[i]}|{src.robot_ids[j]}": round(v, 4)
                                                       for (i, j), v in src_h.items()}
        tries = []
        for prefer in (cfg.repair_prefer if len(arms) > 1 else ("makespan",)):
            sched, log_ = Schedule(arms), []
            if prefer == "anchored":
                anchor(sched)
                log_.append(dict(anchored=True, initial_waits_s=round(float(sum(w.sum() for w in sched.waits)), 3),
                                 makespan=round(float(sched.makespan), 3)))
            ok_ = repair(sched, conf, cfg, log_, prefer="makespan" if prefer == "anchored" else prefer,
                         budget_s=cfg.repair_budget_s * (3 if prefer == "anchored" else 1)) if len(arms) > 1 else True
            if ok_ and len(arms) > 1 and conf_fine.first(sched) is not None:
                log_.append(dict(fine_pass=True, hz=cfg.fine_hz))
                ok_ = repair(sched, conf_fine, cfg, log_, prefer="makespan" if prefer == "anchored" else prefer,
                             budget_s=cfg.repair_budget_s)
            # (the schedule is clean for conf_fine here, so only the hull-skew check decides whether to repair)
            if ok_ and len(arms) > 1 and skew_conf is not None:
                sk = HullSkewConflicts(ctx, src.robot_ids, skew_conf.req, hz=cfg.hull_skew_hz, skew_s=cfg.hull_skew_s)
                for rnd in range(len(cfg.hull_cert_margins_m) + 1):
                    if sk.first(sched) is not None:
                        log_.append(dict(hull_skew_pass=True, round=rnd, hz=cfg.hull_skew_hz, skew_s=cfg.hull_skew_s,
                                         req_m={f"{src.robot_ids[a]}|{src.robot_ids[b]}": round(v, 4)
                                                for (a, b), v in sk.req.items()}))
                        ok_ = repair(sched, MultiConflicts([conf_fine, sk]), cfg, log_,
                                     prefer="makespan" if prefer == "anchored" else prefer, budget_s=cfg.repair_budget_s)
                        if not ok_:
                            break
                    c = skew_cert.first(sched)            # the verifier's continuous certificate, once per round
                    if c is None or rnd == len(cfg.hull_cert_margins_m):
                        if c is not None:
                            log_.append(dict(hull_cert_fail=True, round=rnd, t=round(c[0], 3)))
                        break
                    # samples clear, certificate not (fast motion dips between the 100 Hz samples): demand a larger
                    # margin at the samples of that pair and repair again
                    pq = c[1]
                    req2 = dict(sk.req)
                    if req2[pq] >= skew_cert.cert[pq]:
                        req2[pq] = max(req2[pq], skew_cert.cert[pq] + cfg.hull_cert_margins_m[rnd])
                    log_.append(dict(hull_cert_fail=True, round=rnd, t=round(c[0], 3),
                                     pair=[src.robot_ids[pq[0]], src.robot_ids[pq[1]]], new_req_m=round(req2[pq], 4)))
                    sk = HullSkewConflicts(ctx, src.robot_ids, req2, hz=cfg.hull_skew_hz, skew_s=cfg.hull_skew_s)
            tries.append((not ok_, sched.makespan, prefer, sched, log_, ok_))
        _, _, prefer, sched, rep_log, ok = min(tries, key=lambda x: (x[0], x[1]))
        added = sum(a.pieces[k].T * (sched.stretch[r][k] - 1) for r, a in enumerate(arms) for k in range(len(a.pieces)))
        info["sync_repair"] = dict(ok=ok, strategy=prefer,
                                   n_waits=int(sum((w > 1e-9).sum() for w in sched.waits)),
                                   total_wait_s=round(float(sum(w.sum() for w in sched.waits)), 3),
                                   n_slowdowns=int(sum((f > 1 + 1e-9).sum() for f in sched.stretch)),
                                   slowdown_added_s=round(float(added), 3),
                                   n_conflict_evals=conf.n_eval, log=rep_log,
                                   strategies={t[2]: dict(ok=t[5], makespan_s=round(float(t[1]), 3),
                                                          n_repair_steps=sum(1 for x in t[4] if "iter" in x and
                                                                             "result" not in x))
                                               for t in tries})
        if ok:
            plan, mode = sched, "independent"
        elif cfg.mode == "independent":
            plan, mode = sched, "independent (repair FAILED)"
    if plan is None:
        # one clock for every arm: no synchronisation needed, so no stop points at the creeping runs
        carms = [build_arm(src, ctx, r, replace(cfg, quasi_rest_frac=0.0)) for r in range(len(src.robot_ids))]
        plan, mode = Coupled(src, ctx, carms, prof, cfg), "coupled"
        arms = carms
        info["coupled_scaling_ok"] = bool(plan.res["ok"])
        info["coupled_worst_over"] = round(plan.res["over"], 4)
        info["coupled_blocks"] = len(plan.oks)
    info["mode"] = mode
    info["retime_s"] = round(time.time() - t0, 2)
    return Result(src=src, ctx=ctx, cfg=cfg, profile=prof, mode=mode, plan=plan, arms=arms, info=info)


# ───────────────────────────────────────────── outputs
def dynamics(res: Result, hz=None):
    """Maxima of |v|/v_max, |a|/a_max, |jerk| (a_max/s and rad/s^3) per arm, and per segment kind."""
    hz = hz or res.cfg.out_hz
    ts = np.arange(0, res.makespan + 1e-9, 1.0 / hz)
    (Q, V, A, Jk), lab = res.plan.sample_all(ts, nu=3, exact=True)
    out = {}
    for r, rid in enumerate(res.src.robot_ids):
        lim = res.ctx.limits[rid]
        vr = np.abs(V[:, r]) / lim.v_max
        ar = np.abs(A[:, r]) / lim.a_max
        jr = np.abs(Jk[:, r]) / lim.a_max
        d = dict(v_ratio_max=round(float(vr.max()), 4), a_ratio_max=round(float(ar.max()), 4),
                 jerk_amax_per_s_max=round(float(jr.max()), 3),
                 jerk_rad_s3_max=round(float(np.abs(Jk[:, r]).max()), 3),
                 vel_jump_max_rad_s=round(float(np.abs(np.diff(V[:, r], axis=0)).max()), 5),
                 vel_jump_over_a_dt=round(float((np.abs(np.diff(V[:, r], axis=0)) / (lim.a_max * (1 / hz))).max()), 4))
        for k, name in enumerate(LABELS[:4]):
            m = lab[:, r] == k
            if m.any():
                d[f"{name}_v_ratio_max"] = round(float(vr[m].max()), 4)
                d[f"{name}_a_ratio_max"] = round(float(ar[m].max()), 4)
                d[f"{name}_time_s"] = round(float(m.sum() / hz), 3)
        mt = lab[:, r] == L_TRANSIT
        if mt.any():
            d["transit_time_at_ge_95pct_v_s"] = round(float((vr[mt].max(1) >= 0.95 * res.profile.v).sum() / hz), 3)
        # consistency of the stored positions with the stored velocities / accelerations (P3U round 2: the coupled
        # mode's positions did not follow its own time law): finite differences of q on the output grid
        dtk = np.diff(ts)
        fdv = np.diff(Q[:, r], axis=0) / dtk[:, None]
        d["qd_vs_fd_ratio_max"] = round(float((np.abs(fdv - 0.5 * (V[1:, r] + V[:-1, r])) / lim.v_max).max()), 5)
        if len(ts) > 2:
            fda = (fdv[1:] - fdv[:-1]) / (0.5 * (dtk[1:] + dtk[:-1]))[:, None]
            d["fd_a_ratio_max"] = round(float((np.abs(fda) / lim.a_max).max()), 4)
        d.update(weld_zone_unlabelled(res, r, ts, Q[:, r], lab[:, r], vr))
        out[rid] = d
    tot = dict(v_ratio_max=max(x["v_ratio_max"] for x in out.values()),
               a_ratio_max=max(x["a_ratio_max"] for x in out.values()),
               jerk_amax_per_s_max=max(x["jerk_amax_per_s_max"] for x in out.values()),
               weld_v_ratio_max=max([x.get(f"{k}_v_ratio_max", 0.0) for x in out.values()
                                     for k in ("approach", "liftoff")]),
               vel_jump_over_a_dt_max=max(x["vel_jump_over_a_dt"] for x in out.values()),
               qd_vs_fd_ratio_max=max(x["qd_vs_fd_ratio_max"] for x in out.values()),
               fd_a_ratio_max=max(x.get("fd_a_ratio_max", 0.0) for x in out.values()),
               weld_zone_unlabelled_frac=max(x.get("weld_zone_unlabelled_frac", 0.0) for x in out.values()),
               weld_zone_unlabelled_v_ratio_max=max(x.get("weld_zone_unlabelled_v_ratio_max", 0.0)
                                                    for x in out.values()))
    return dict(per_robot=out, all=tot, hz=hz)


def weld_zone_unlabelled(res: Result, r: int, ts, Q, lab, vr) -> dict:
    """Share of each weld's zone (TCP within approach_offset of THAT weld, contiguous with its hold on the retimed
    clock) that is not labelled approach / hold / lift-off, and the peak joint speed there (fraction of v_max)."""
    rid = res.src.robot_ids[r]
    off = res.cfg.approach_offset_m if res.cfg.approach_offset_m is not None else res.ctx.approach_offset_m
    welds = getattr(res.ctx, "welds", {}) or {}
    n_zone = n_un = 0
    vmax = 0.0
    P = None
    for e in res.weld_events():
        if e[0] != rid or e[1] not in welds:
            continue
        if P is None:
            P = res.ctx.tcp(rid, Q)
        d = np.linalg.norm(P - welds[e[1]], axis=1) <= off
        k0 = int(np.clip(np.searchsorted(ts, e[2]), 0, len(ts) - 1))
        k1 = int(np.clip(np.searchsorted(ts, e[3]), 0, len(ts) - 1))
        idx = []
        k = k0
        while k - 1 >= 0 and d[k - 1]:
            k -= 1
            idx.append(k)
        k = k1
        while k + 1 < len(ts) and d[k + 1]:
            k += 1
            idx.append(k)
        if not idx:
            continue
        idx = np.asarray(idx)
        un = idx[(lab[idx] == L_TRANSIT) | (lab[idx] == L_HOME)]
        n_zone += len(idx)
        n_un += len(un)
        if len(un):
            vmax = max(vmax, float(vr[un].max()))
    return {"weld_zone_unlabelled_frac": round(n_un / n_zone, 4) if n_zone else 0.0,
            "weld_zone_unlabelled_v_ratio_max": round(vmax, 4)}


def to_ntimeline(res: Result, hz=None):
    """Retimed motion as an NTimeline (legs per arm: one per moving piece, ``weld`` legs end at a hold)."""
    from player.timeline import Leg
    from trajgen.ntimeline import NTimeline
    hz = hz or res.cfg.out_hz
    T = res.makespan
    # uniform grid (P3U round 2: a short last step made a non-uniform 250 Hz grid): the last sample is the first
    # grid point >= T; the motion is at rest at its end pose there
    n_out = int(np.ceil(T * hz - 1e-6))
    ts = np.round(np.arange(n_out + 1) / hz, 9)
    (Q, V, A), lab = res.plan.sample_all(np.minimum(ts, T), nu=2, exact=True)
    legs = []
    ev = res.weld_events()
    for r, a in enumerate(res.arms):
        rid = a.rid
        # moving intervals on the output grid: contiguous runs where the arm is not at rest
        mv = np.abs(np.diff(Q[:, r], axis=0)).max(1) > 1e-12
        k = 0
        n = 0
        while k < len(mv):
            if not mv[k]:
                k += 1
                continue
            k0 = k
            while k < len(mv) and mv[k]:
                k += 1
            i0, i1 = k0, k
            t1 = ts[i1]
            hold = [e for e in ev if e[0] == rid and abs(e[2] - t1) < 2.0 / hz]
            kind = "weld" if hold else ("home" if i1 == len(ts) - 1 or not mv[i1:].any() else "move")
            lg = Leg(robot=rid, op_id=hold[0][1] if hold else f"{rid}_M{n:02d}", kind=kind, t0=float(ts[i0]),
                     t1=float(t1), times=ts[i0:i1 + 1].copy(), positions=Q[i0:i1 + 1, r].copy(),
                     phase=np.array([LABELS[int(x)] if x < 4 else "" for x in lab[i0:i1 + 1, r]], dtype="<U16"),
                     hold_s=float(hold[0][3] - hold[0][2]) if hold else 0.0)
            lg.t_done = lg.t1 + lg.hold_s
            legs.append(lg)
            n += 1
    meta = dict(src=res.src.path, retime=dict(profile=res.profile.to_dict(), mode=res.mode,
                                              cfg=res.cfg.to_dict()), clock="retimed (tools.retime)",
                cell=getattr(res.ctx, "name", None) if res.ctx.kind == "cell" else None,
                scene=str(res.ctx.dir) if res.ctx.kind == "scene" else None)
    tl = NTimeline(t=ts, q=Q, robot_ids=tuple(res.src.robot_ids), legs=legs, weld_events=ev, meta=meta,
                   home=np.asarray(res.src.home, float))
    return tl, dict(qd=V, qdd=A, labels=lab)


def save(res: Result, out, report: dict | None = None):
    tl, extra = to_ntimeline(res)
    if report is not None:
        tl.meta["report"] = report
    p = tl.save(out)
    # extra arrays (velocities, accelerations, labels) appended
    z = dict(np.load(p, allow_pickle=False))
    z.update(qd=extra["qd"], qdd=extra["qdd"], labels=extra["labels"],
             labels_legend=np.array(json.dumps(list(LABELS))))
    np.savez_compressed(p, **z)
    return p


# ───────────────────────────────────────────── dense re-check
_SRC_CHECK = {}


def check_source(src: Source, ctx, hz=400.0, hull_every=1):
    key = (src.path, ctx.name, hz, hull_every)
    if key not in _SRC_CHECK:
        T = float(src.t[-1])
        ts = np.arange(0, T + 1e-9, 1.0 / hz)
        Q = src.tl.sample(ts)
        _SRC_CHECK[key] = ctx.full_check(ts, Q, src.weld_events, hull_every=hull_every)
    return _SRC_CHECK[key]


def check_result(res: Result, hz=400.0, hull_every=1):
    """Dense re-check of the retimed motion (exact time law, ``hz`` samples) against the source (same checks)."""
    t0 = time.time()
    ts = np.arange(0, res.makespan + 1e-9, 1.0 / hz)
    (Q,), _ = res.plan.sample_all(ts, exact=True)
    rep = res.ctx.full_check(ts, Q, res.weld_events(), hull_every=hull_every)
    srep = check_source(res.src, res.ctx, hz, hull_every)
    valid, reasons, inherited = res.ctx.verdict(rep["violations"], srep["violations"])
    return dict(valid=bool(valid), new_or_worse=reasons, inherited=inherited, hz=hz, hull_every=hull_every,
                retimed=rep, source=srep, check_s=round(time.time() - t0, 1))


def summary(res: Result, chk: dict | None = None, dyn: dict | None = None):
    ev = res.weld_events()
    src_ev = res.src.weld_events
    out = dict(source=res.src.path, context=dict(kind=res.ctx.kind, name=res.ctx.name), mode=res.mode,
               profile=res.profile.to_dict(),
               source_last_weld_s=round(res.src.last_weld, 3), source_end_s=round(float(res.src.t[-1]), 3),
               last_weld_s=round(max(e[3] for e in ev), 3) if ev else None, makespan_s=round(res.makespan, 3),
               n_welds=len(ev), n_welds_source=len(src_ev),
               sync=res.info.get("sync_repair", {}).get("n_waits"),
               sync_slowdowns=res.info.get("sync_repair", {}).get("n_slowdowns"),
               sync_total_wait_s=res.info.get("sync_repair", {}).get("total_wait_s"),
               info={k: v for k, v in res.info.items() if k != "sync_repair"})
    if "sync_repair" in res.info:
        out["sync_log"] = res.info["sync_repair"]["log"]
    if dyn is not None:
        out["dynamics"] = dyn
    if chk is not None:
        r, s = chk["retimed"], chk["source"]
        g = lambda d, *ks: _dig(d, ks)   # noqa: E731
        out["safety"] = dict(valid=chk["valid"], new_or_worse=chk["new_or_worse"], inherited=chk["inherited"],
                             hz=chk["hz"], violations=r.get("violations"), source_violations=s.get("violations"),
                             sphere_min_m=g(r, "sphere_min_clearance_m") or g(r, "arm_arm_min_m"),
                             source_sphere_min_m=g(s, "sphere_min_clearance_m") or g(s, "arm_arm_min_m"),
                             hull_contacts=g(r, "hull_contact_samples"), rule_W=g(r, "rule_W"),
                             welds=g(r, "welds"), checks=g(r, "checks"), check_s=chk["check_s"])
    return out


def _dig(d, ks):
    for k in ks:
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d
