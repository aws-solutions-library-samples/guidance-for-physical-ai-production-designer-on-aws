"""N-arm coordination on one clock by prioritized planning ("priority" coordination; any robot count).

Each robot runs its own sequence of legs (trajgen.schedule.Planner: [wait] + straight lift-off + checked joint
transit + straight descent, then the dwell; the last leg returns HOME). The executor completes a leg after its
motion + hold + ``executor_margin_s``; the robot stands still in between. Robots are planned one after another in a
priority order, each against the trajectories already fixed:

  * a robot that is not planned yet stands at its HOME for the whole cycle; a planned robot holds its last pose
    (HOME) after its last leg. So every new leg is checked against everything that will really happen, and
    standing at HOME is always safe for a robot (every robot planned before it was checked against it standing there);
  * a leg is placed at the earliest start delay (a stationary prefix at the current pose) at which the leg, the
    standing before it and the hold + margin after it are clear of every other robot: the 20 Hz conflict map screen
    (corrected spheres > gate + margin + chord allowance, umarell parity spheres if enabled, both with a +-skew
    window) followed by an exact 100 Hz check (linear interpolation, the same skew window on the spheres, and the
    robot-robot MuJoCo hull distance >= collision.arm_arm.hull_min_distance_m at every 100 Hz shift within the
    window);
  * deadlock freedom: a weld leg is committed only if an escape exists afterwards (a HOME leg from that weld at some
    later delay). When no next weld can be reached from the current weld, the robot takes its escape (a yield to
    HOME, an ExecuteTrajectory "move") and continues from HOME. Once all robots planned before it are back HOME the
    world is static, so a weld whose HOME -> weld -> HOME legs are clear of the others standing at HOME (the
    eligibility test, ``static_eligible``) is always schedulable;
  * candidates: the next ``window`` welds of the robot's queue (sequence groups kept in order), the one finishing
    earliest is taken;
  * several priority orders are tried; the shortest executor makespan wins (an order that fails is skipped).

``group_order="global"`` plans the sequence groups as phases: every robot finishes the group (back HOME) before any
robot starts the next one.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from trajgen.schedule import LegPlan


@dataclass
class CLeg:
    rid: str
    target: str                  # weld id | "home" (yield or final)
    kind: str                    # "weld" | "yield" | "home"
    opt: object                  # WeldOption | None
    k_disp: int                  # global sample index: the robot is free (previous leg complete)
    d: int                       # stationary prefix (samples)
    leg: LegPlan                 # motion samples (n + 1)
    hold_n: int
    margin_n: int
    cmin: float = np.nan

    @property
    def n(self):
        return len(self.leg.Q) - 1

    @property
    def k_start(self):
        return self.k_disp + self.d

    @property
    def k_end(self):
        return self.k_start + self.n

    @property
    def k_done(self):
        return self.k_end + self.hold_n + self.margin_n


class Track:
    """Committed trajectory of one robot on the global 20 Hz grid (holds its last pose afterwards)."""

    def __init__(self, rid, home):
        self.rid = rid
        self.home = np.asarray(home, float)
        self.chunks = [self.home[None].copy()]
        self.K = 0
        self.legs: list[CLeg] = []
        self.pos = None                    # None (HOME) or (wid, option index)
        self._arr = None

    @property
    def q(self):
        if self._arr is None or len(self._arr) != self.K + 1:
            self._arr = np.concatenate(self.chunks)
        return self._arr

    @property
    def q_last(self):
        return self.chunks[-1][-1]

    def stand(self, n):
        if n > 0:
            self.chunks.append(np.repeat(self.q_last[None], n, 0))
            self.K += n

    def copy(self) -> "Track":
        t = Track.__new__(Track)
        t.rid, t.home, t.chunks, t.K, t.legs, t.pos, t._arr = (self.rid, self.home, list(self.chunks), self.K,
                                                               list(self.legs), self.pos, self._arr)
        return t

    def commit(self, cl: CLeg):
        assert cl.k_disp == self.K
        self.stand(cl.d)
        Q = cl.leg.Q
        if np.abs(Q[0] - self.q_last).max() > 1e-6:
            raise ValueError(f"{self.rid}: leg to {cl.target} does not start at the current pose")
        self.chunks.append(Q[1:].copy())
        self.K += len(Q) - 1
        self.stand(cl.hold_n + cl.margin_n)
        self.legs.append(cl)


@dataclass
class CoordResult:
    ok: bool
    tracks: dict
    order: list
    failed: list = field(default_factory=list)   # [{"robot", "welds", "reason"}]
    objective_s: float = np.inf
    stats: dict = field(default_factory=dict)


class _Other:
    """Sphere data of a fixed robot trajectory (20 Hz samples + dense) for the screens."""

    def __init__(self, co, rid, q):
        self.rid = rid
        self.q = q
        self.K = len(q) - 1
        m = co.m
        self.S = m.pair_spheres(rid, q).astype(np.float32)
        self.lo, self.hi = self.S.min(1), self.S.max(1)
        self.U = m.umarell_spheres(rid, q).astype(np.float32) if co.parity else None
        if self.U is not None:
            self.ulo, self.uhi = self.U.min(1), self.U.max(1)
        # sphere-group bounding spheres (lower bounds for the screen, PairChecker.group_spheres)
        self.SG = co.ck.group_spheres(rid, self.S, "S")
        self.UG = co.ck.group_spheres(rid, self.U, "U") if self.U is not None else None
        self._dense = None
        self.co = co

    def idx(self, k):
        return np.clip(k, 0, self.K)

    def dense(self):
        if self._dense is None:
            ck = self.co.ck
            D, S, U = ck.dense(self.rid, self.q)
            self._dense = (D, S, U)
        return self._dense


class Coordinator:
    PREFIX_CACHE = True              # reuse the tracks of identical order prefixes (run_order); tests turn it off

    def __init__(self, model, planner, checker, reach, cost, task, *, executor_margin: float, skew_s: float = 0.1,
                 chord_extra: float = 0.02, window: int = 2, n_branch: int = 1, n_confirm: int = 6,
                 group_order: str = "per_arm", chunk: int = 160, log=print):
        self.m, self.pl, self.ck, self.reach, self.cost, self.task = model, planner, checker, reach, cost, task
        self.rids = model.rids
        self.dt = planner.dt
        self.parity = checker.parity
        self.thr = checker.gate + chord_extra
        self.thr_u = (checker.gate_u + chord_extra) if self.parity else None
        self.pr = checker.pair_r
        self.w = int(round(skew_s / self.dt))
        self.w_base = self.w
        self.skew_s = self.w * self.dt
        self.w_extra = {}                  # (rid, leg target) -> widened skew window (samples), lateness repair
        self.hold_n = int(np.ceil(planner.dwell / self.dt - 1e-9))
        self.margin_n = int(np.ceil(executor_margin / self.dt - 1e-9))
        self.window, self.n_branch, self.n_confirm = max(1, window), max(1, n_branch), n_confirm
        self.group_order = group_order
        self.chunk = chunk
        self.log = log
        self.home = {r: model.robots[r].home for r in self.rids}
        g = list(task.sequence_groups_order)
        self.rank = {w.id: (g.index(w.group) if w.group in g else len(g)) for w in task.welds}
        self.stats = {"maps": 0, "exact_checks": 0, "exact_fail": 0, "yields": 0, "legs": 0}
        self._legs = {}
        self.eligible = {}                 # (wid, rid) -> option indices (static_eligible), set by the pipeline
        # parking (round p3): a leg that would wait >= margin + park_min_n samples on the weld is split into a
        # lift-off-only leg (dispatched at once, op <ROBOT>_PARK_<n>) and the rest from the retract pose, when that
        # ends no later; the robot then waits at the retract pose instead of with the tool on the weld
        self.park = True
        self.park_min_n = 0
        # prefix cache (round p3): the tracks after planning the first i robots of an order depend only on their
        # queues, their widened windows and the options, so a re-plan (lateness repair, other orders with the same
        # head) reuses them
        self._prefix = {}
        # forced minimum start delays (samples) per (robot, leg target): the start-delay local search (round p3)
        self.min_delay = {}
        self.wids = {w.id for w in task.welds}

    # ------------------------------------------------------------------ legs
    def opt(self, key, rid):
        return None if key is None else self.reach[(key[0], rid)][key[1]]

    def q_of(self, rid, key):
        if key is not None and key[0] == "ret":          # parked at the retract pose of weld option key[1:]
            return self.opt(key[1:], rid).q_ret
        return self.home[rid] if key is None else self.opt(key, rid).q

    def leg(self, rid, frm, to):
        """Leg from pose key ``frm`` (None = HOME, (wid, k) = on the weld, ("ret", wid, k) = parked at its retract
        pose) to ``to`` ((wid, k) or "home"), cached."""
        k = (rid, frm, to)
        if k not in self._legs:
            at = None if (frm is not None and frm[0] == "ret") else self.opt(frm, rid)
            tgt = "home" if to == "home" else self.opt(to, rid)
            self._legs[k] = self.pl.leg(rid, self.q_of(rid, frm), at, tgt)
            self.stats["legs"] += 1
        return self._legs[k]

    def lift_leg(self, rid, pos):
        """The straight lift-off out of weld option ``pos`` alone (a parking leg), cached."""
        k = (rid, pos, "lift")
        if k not in self._legs:
            self._legs[k] = self.pl.lift(rid, self.opt(pos, rid))
        return self._legs[k]

    def park_name(self, rid, track) -> str:
        n = 1 + sum(1 for cl in track.legs if cl.kind == "park")
        op = f"{rid.upper()}_PARK_{n}"
        while op in self.wids:                            # never the id of a task weld
            op += "_P"
        return op

    def _park_alt(self, rid, track, others, pos, to, L, d, post, w_leg, escape=None):
        """Parking alternative of leg L placed at delay d from weld option ``pos``: (lift leg P, its op id, the rest
        L2 from the retract pose, its delay d2, P's delay dP), with P at its earliest delay dP (0 unless another
        robot passes the retract pose first; dP + margin <= d) and L2 at its earliest delay after P is complete (and
        ``escape(end of L2 + post)`` true); None when parking is off, not possible, or ends later than L."""
        if not self.park or pos is None or pos[0] == "ret" or d < self.margin_n + self.park_min_n:
            return None
        P = self.lift_leg(rid, pos)
        if P is None or len(P.Q) < 2:
            return None
        K = track.K
        nP = len(P.Q) - 1
        op = self.park_name(rid, track)
        L2 = self.leg(rid, ("ret",) + tuple(pos), to)
        if L2 is None:
            return None
        n2 = len(L2.Q) - 1
        d_max = d - self.margin_n                      # the latest lift-off: L2 then starts as L's transit did
        # the lift-off at its earliest delay (0 unless another robot passes the retract pose first), else later:
        # half way, and at the latest (the robot then waits at the retract pose for the completion margin only)
        tried, why = [], "lift-off blocked"
        dP0, _ = self.place(rid, others, K, P, self.margin_n, w=self.w_for(rid, op))
        cands = [] if dP0 is None or dP0 > d_max else sorted({dP0, (dP0 + d_max) // 2, d_max})
        for dP in cands:
            if dP != dP0:
                dq, _ = self.place(rid, others, K, P, self.margin_n, d_from=dP, w=self.w_for(rid, op))
                if dq != dP:
                    continue
            K2 = K + dP + nP + self.margin_n
            need = None if escape is None else (lambda dd, K2=K2: escape(K2 + dd + n2 + post))
            d2, _ = self.place(rid, others, K2, L2, post, need=need, w=w_leg)
            if d2 is not None and K2 + d2 + n2 <= K + d + len(L.Q) - 1:
                self.stats["parks"] = self.stats.get("parks", 0) + 1
                return P, op, L2, d2, dP
            why = "the leg from the retract pose ends later"
        self.stats["park_failed"] = self.stats.get("park_failed", 0) + 1
        self.park_fail = getattr(self, "park_fail", {})
        self.park_fail[why] = self.park_fail.get(why, 0) + 1
        return None

    def _commit_park(self, rid, track, others, P, op, dP: int = 0):
        c = self.cmin(rid, others, track.K, P, dP, self.margin_n, w=self.w_for(rid, op))
        track.commit(CLeg(rid, op, "park", None, track.K, dP, P, 0, self.margin_n, c))

    # ------------------------------------------------------------------ screens
    def _clr(self, A, Alo, Ahi, B, Blo, Bhi, thr, GA=None, GB=None, PA=None, PB=None):
        """(len(A), len(B)) sphere clearance - thr, exact where the AABB bound is below 0 (else the bound). With
        the sphere-group bounds GA / GB (centres, radii per row; PairChecker.group_spheres) the exact value is
        computed only where the group bound is below 1e-4 as well; elsewhere the (positive) group bound is kept, so
        the sign of every entry, which is all the screen uses, is that of the exact value. With the padded fine
        group index arrays PA / PB (PairChecker.group_index) the remaining entries are evaluated on the sphere pairs
        of the group pairs whose bound is below 1e-4 only (the same float32 values as the full evaluation for those
        pairs; every other pair is >= 1e-4, so the sign is again that of the exact value)."""
        gap = np.maximum(0.0, np.maximum(Blo[None] - Ahi[:, None], Alo[:, None] - Bhi[None]))
        M = np.sqrt((gap * gap).sum(-1)) - 2 * self.pr - thr
        ii, jj = np.nonzero(M < 0)
        if GA is not None and GB is not None and len(ii):
            from trajgen.schedule import PairChecker
            lb = PairChecker.group_lb(GA, GB, ii, jj, cut=2 * self.pr + thr + 1e-3) - 2 * self.pr - thr
            keep = lb >= 1e-4
            M[ii[keep], jj[keep]] = lb[keep]
            ii, jj = ii[~keep], jj[~keep]
        if PA is not None and PB is not None and GA is not None and GB is not None:
            C2A, R2A, C2B, R2B = GA[2], GA[3], GB[2], GB[3]
            for s in range(0, len(ii), 2048):
                i_, j_ = ii[s:s + 2048], jj[s:s + 2048]
                d = C2A[i_][:, :, None] - C2B[j_][:, None]
                lbg = np.sqrt(np.einsum("nabk,nabk->nab", d, d)) - R2A[i_][:, :, None] - R2B[j_][:, None] \
                    - 2 * self.pr - thr
                tn, ta, tb = np.nonzero(lbg < 1e-4)
                ar, br = A[i_], B[j_]
                c = 0.5 * (ar.mean(1, keepdims=True) + br.mean(1, keepdims=True))
                a = ar[tn[:, None], PA[ta]] - c[tn]
                b = br[tn[:, None], PB[tb]] - c[tn]
                d2 = (a * a).sum(-1)[:, :, None] + (b * b).sum(-1)[:, None, :] - 2.0 * np.einsum("nik,njk->nij", a, b)
                best = np.full(len(i_), np.inf, dtype=d2.dtype)
                np.minimum.at(best, tn, d2.reshape(len(tn), -1).min(1))
                M[i_, j_] = np.sqrt(np.maximum(best, 0.0)) - 2 * self.pr - thr
            self.stats["maps"] += 1
            return M
        for s in range(0, len(ii), 1024):
            a, b = A[ii[s:s + 1024]], B[jj[s:s + 1024]]
            c = 0.5 * (a.mean(1, keepdims=True) + b.mean(1, keepdims=True))
            a, b = a - c, b - c
            d2 = (a * a).sum(-1)[:, :, None] + (b * b).sum(-1)[:, None, :] - 2.0 * np.einsum("nik,njk->nij", a, b)
            M[ii[s:s + 1024], jj[s:s + 1024]] = np.sqrt(np.maximum(d2.reshape(len(a), -1).min(1), 0.0)) - 2 * self.pr - thr
        self.stats["maps"] += 1
        return M

    def _map(self, rid, Q, o: _Other, kk):
        """(len(Q), len(kk)) min(corrected - thr, parity - thr_u) between robot rid at configurations Q and the other
        robot at global samples kk."""
        S = self.m.pair_spheres(rid, Q).astype(np.float32)
        ix = o.idx(kk)
        u, inv = np.unique(ix, return_inverse=True)
        ck = self.ck
        M = self._clr(S, S.min(1), S.max(1), o.S[u], o.lo[u], o.hi[u], self.thr,
                      ck.group_spheres(rid, S, "S"), tuple(x[u] for x in o.SG),
                      ck.group_index(rid, "S"), ck.group_index(o.rid, "S"))
        if self.parity:
            U = self.m.umarell_spheres(rid, Q).astype(np.float32)
            M = np.minimum(M, self._clr(U, U.min(1), U.max(1), o.U[u], o.ulo[u], o.uhi[u], self.thr_u,
                                        ck.group_spheres(rid, U, "U"), tuple(x[u] for x in o.UG),
                                        ck.group_index(rid, "U"), ck.group_index(o.rid, "U")))
        return M[:, inv]

    @staticmethod
    def _winmin(x, w, axis=-1):
        """Sliding min over a centred window of half width w along ``axis`` (edges: shrunk window)."""
        if w <= 0:
            return x
        out = x.copy()
        n = x.shape[axis]
        for s in range(1, w + 1):
            a = np.take(x, np.arange(s, n), axis=axis)
            b = np.take(x, np.arange(0, n - s), axis=axis)
            sl_lo = [slice(None)] * x.ndim
            sl_hi = [slice(None)] * x.ndim
            sl_lo[axis] = slice(0, n - s)
            sl_hi[axis] = slice(s, n)
            out[tuple(sl_lo)] = np.minimum(out[tuple(sl_lo)], a)
            out[tuple(sl_hi)] = np.minimum(out[tuple(sl_hi)], b)
        return out

    def screen(self, rid, others, K, L: LegPlan, post_n: int, d0: int, d1: int) -> np.ndarray:
        """bool (d1 - d0,): delay d (stationary at L.Q[0] over [K, K+d], the leg over [K+d, K+d+n], standing at
        L.Q[-1] over [K+d+n, K+d+n+post_n]) passes the conflict-map screen against every other robot."""
        nD = d1 - d0
        ok = np.ones(nD, bool)
        Q = L.Q
        n = len(Q) - 1
        w = self.w
        dd = np.arange(d0, d1)
        for o in others:
            # standing before the leg: K .. K + d (prefix)
            kk = np.arange(K, K + d1 + 1)
            pre = self._map(rid, Q[:1], o, kk)[0]
            pre = self._winmin(pre, w)
            cm = np.minimum.accumulate(pre)
            ok &= cm[dd] >= 0
            if not ok.any():
                return ok
            # the leg: column c of M is global sample K + d0 - w + c
            kk = np.arange(K + d0 - w, K + d1 + n + w)
            M = self._winmin(self._map(rid, Q, o, kk), w, axis=1)
            a = np.arange(n + 1)
            cols = (dd - d0)[:, None] + w + a[None, :]
            ok &= M[a[None, :], cols].min(1) >= 0
            if not ok.any():
                return ok
            # standing after the leg
            if post_n > 0:
                kk = np.arange(K + d0 + n - w, K + d1 + n + post_n + w)
                post = self._winmin(self._map(rid, Q[-1:], o, kk)[0], w)
                # window [K+d+n, K+d+n+post_n] -> columns (d - d0) + w .. + post_n
                cs = np.lib.stride_tricks.sliding_window_view(post, post_n + 1)
                ok &= cs.min(1)[(dd - d0) + w] >= 0
        return ok

    def exact(self, rid, others, K, L: LegPlan, d: int, post_n: int, want_min: bool = False) -> tuple[bool, float]:
        """Exact 100 Hz check of the placed leg (prefix + leg + post) against every other robot over the +-skew
        window: spheres at the nominal alignment and at whole-sample shifts, robot-robot hull distance >=
        hull_min at every dense shift (trajgen.schedule.PairChecker.check_window). The returned min clearance is
        exact only with ``want_min`` (else a pass / fail check; ``cmin`` gives it for the committed legs)."""
        self.stats["exact_checks"] += 1
        ck = self.ck
        DA, SA, UA = ck.dense_padded(rid, L.Q, d, post_n)    # == ck.dense of [Q0 x d, Q, Q[-1] x post_n]
        s = ck.sub
        cmin = np.inf
        ia = np.arange(len(DA))
        for o in others:
            DB, SB, UB = o.dense()
            okk, c, _ = ck.check_window(rid, o.rid, DA, SA, UA, DB, SB, UB, ia, K * s + ia, self.w, want_min=want_min)
            cmin = min(cmin, c)
            if not okk:
                self.stats["exact_fail"] += 1
                return False, cmin
        return True, cmin

    def cmin(self, rid, others, K, L: LegPlan, d: int, post_n: int, w: int | None = None) -> float:
        """Min sphere clearance (both models, over the skew window ``w``) of a placed leg: the value ``exact``
        reports for it with want_min (reporting only; computed for the committed legs)."""
        if not others:
            return np.inf
        ck = self.ck
        DA, SA, UA = ck.dense_padded(rid, L.Q, d, post_n)
        ia = np.arange(len(DA))
        w = self.w if w is None else int(w)
        out = np.inf
        for o in others:
            DB, SB, UB = o.dense()
            out = min(out, ck.window_min(rid, o.rid, DA, SA, UA, DB, SB, UB, ia, K * ck.sub + ia, w))
        return out

    def w_for(self, rid, target) -> int:
        """Skew window (samples) for placing robot rid's leg ``target``: the base window, widened by the lateness
        repair (trajgen.pipeline) for legs whose clearance depended on a tighter alignment."""
        return max(self.w_base, int(self.w_extra.get((rid, target), 0)))

    def place(self, rid, others, K, L: LegPlan, post_n: int, d_from: int = 0, need=None, w: int | None = None):
        """Earliest delay d >= d_from passing the screen and the exact check (and ``need(d)`` if given), with the
        skew window ``w`` (samples; default the base window). Returns (d, cmin) or (None, None). The search ends
        once the other robots are static (later delays cannot help)."""
        w_old = self.w
        if w is not None:
            self.w = int(w)
        try:
            return self._place(rid, others, K, L, post_n, d_from, need)
        finally:
            self.w = w_old

    def _place(self, rid, others, K, L: LegPlan, post_n: int, d_from: int = 0, need=None):
        if not others:
            return (d_from, np.inf) if (need is None or need(d_from)) else (None, None)
        K_static = max(o.K for o in others)
        d_max = max(d_from, K_static - K) + 1 + self.w
        d0 = d_from
        tries = 0
        while d0 <= d_max:
            d1 = min(d0 + self.chunk, d_max + 1)
            ok = self.screen(rid, others, K, L, post_n, d0, d1)
            for d in (d0 + np.flatnonzero(ok)):
                good, c = self.exact(rid, others, K, L, int(d), post_n)
                if good and (need is None or need(int(d))):
                    return int(d), c
                tries += 1
                if tries >= self.n_confirm and d < K_static - K:
                    break                      # many near misses: jump ahead
            d0 = d1
        return None, None

    # ------------------------------------------------------------------ eligibility
    def static_eligible(self, rid, wid) -> tuple[list, str]:
        """Options k of (wid, rid) whose HOME -> weld and weld -> HOME legs are clear of every other robot standing
        at HOME (the condition that makes the weld schedulable once the others are back HOME)."""
        return self._eligible_combine([self.static_eligible_option(rid, wid, k)
                                       for k in range(len(self.reach.get((wid, rid), [])))])

    def static_eligible_option(self, rid, wid, k) -> str:
        """'good' | 'notransit' | 'blocked' for option k of (wid, rid) (static_eligible)."""
        others = [_Other(self, r, self.home[r][None]) for r in self.rids if r != rid]
        La, Lb = self.leg(rid, None, (wid, k)), self.leg(rid, (wid, k), "home")
        if La is None or Lb is None:
            return "notransit"
        if not others:
            return "good"
        for L, post in ((La, self.hold_n), (Lb, 0)):
            if not self.screen(rid, others, 0, L, post, 0, 1)[0] or not self.exact(rid, others, 0, L, 0, post)[0]:
                return "blocked"
        return "good"

    @staticmethod
    def _eligible_combine(st: list) -> tuple[list, str]:
        good, why = [], ""
        for k, x in enumerate(st):
            if x == "good":
                good.append(k)
            elif x == "notransit":
                why = why or "no collision-free joint transit between HOME and the weld"
            else:
                why = "blocked by another robot standing at HOME (arm-arm gate / hulls)"
        return good, why

    # ------------------------------------------------------------------ planning
    def _candidates(self, rid, queue, pos, groups_left):
        if not queue:
            return []
        head_rank = self.rank[queue[0]] if self.group_order != "ignore" else 0
        cands = [w for w in queue[:self.window] if self.group_order == "ignore" or self.rank[w] == head_rank]
        q0 = self.q_of(rid, pos) if pos is None else self.opt(pos, rid).q_ret
        out = []
        for w in cands:
            ops = self.reach[(w, rid)]
            ks = self.eligible.get((w, rid), list(range(len(ops))))
            costs = sorted(((self.cost.move(rid, q0, ops[k].q_pre) + self.cost._stroke(ops[k]), k) for k in ks))
            out += [(w, k) for _, k in costs[:self.n_branch]]
        return out

    def plan_robot(self, rid, track: Track, queue: list, others, *, final_home: bool = True, phase: str = ""):
        """Plan robot ``rid``'s queue on its track. Returns (ok, remaining welds, reason)."""
        pos = track.pos
        queue = list(queue)
        n_yield = sum(1 for cl in track.legs if cl.kind == "yield")
        while queue:
            best = None
            for (w, k) in self._candidates(rid, queue, pos, None):
                L = self.leg(rid, pos, (w, k))
                if L is None:
                    continue
                Lh = self.leg(rid, (w, k), "home")
                if Lh is None:
                    continue
                K = track.K
                n = len(L.Q) - 1
                post = self.hold_n + self.margin_n

                w_esc = max(self.w_for(rid, f"{rid.upper()}_YIELD_{n_yield + 1}{phase}"), self.w_for(rid, "home"))

                def escape_ok(d, L=L, Lh=Lh, K=K, n=n, post=post, w_esc=w_esc):
                    de, _ = self.place(rid, others, K + d + n + post, Lh, self.margin_n, w=w_esc)
                    return de is not None

                d, c = self.place(rid, others, K, L, post, need=escape_ok, w=self.w_for(rid, w),
                                  d_from=int(self.min_delay.get((rid, w), 0)))
                if d is None:
                    continue
                end = K + d + n + post

                def escape_at(Ke, Lh=Lh, w_esc=w_esc):
                    return self.place(rid, others, Ke, Lh, self.margin_n, w=w_esc)[0] is not None
                if best is None or end < best[0]:
                    best = (end, w, k, L, d, c, escape_at)
            if best is not None:
                end, w, k, L, d, c, escape_at = best
                post = self.hold_n + self.margin_n
                alt = self._park_alt(rid, track, others, pos, (w, k), L, d, post, self.w_for(rid, w), escape_at)
                if alt is not None:
                    self._commit_park(rid, track, others, alt[0], alt[1], alt[4])
                    L, d = alt[2], alt[3]
                c = self.cmin(rid, others, track.K, L, d, post, w=self.w_for(rid, w))
                track.commit(CLeg(rid, w, "weld", self.opt((w, k), rid), track.K, d, L, self.hold_n, self.margin_n, c))
                pos = track.pos = (w, k)
                queue.remove(w)
                continue
            if pos is not None:                  # take the escape: yield to HOME, continue from there
                Lh = self.leg(rid, pos, "home")
                d, c = self.place(rid, others, track.K, Lh, self.margin_n,
                                  w=self.w_for(rid, f"{rid.upper()}_YIELD_{n_yield + 1}{phase}"))
                if d is None:
                    return False, queue, f"{rid}: no escape to HOME from {pos[0]} (escape not found with the widened " \
                        "lateness window)" if self.w_extra else f"{rid}: no escape to HOME from {pos[0]} (internal " \
                        "error: escape vanished)"
                n_yield += 1
                self.stats["yields"] += 1
                w_y = self.w_for(rid, f"{rid.upper()}_YIELD_{n_yield}{phase}")
                alt = self._park_alt(rid, track, others, pos, "home", Lh, d, self.margin_n, w_y)
                if alt is not None:
                    self._commit_park(rid, track, others, alt[0], alt[1], alt[4])
                    Lh, d = alt[2], alt[3]
                c = self.cmin(rid, others, track.K, Lh, d, self.margin_n, w=w_y)
                track.commit(CLeg(rid, f"{rid.upper()}_YIELD_{n_yield}{phase}", "yield", None, track.K, d, Lh, 0,
                                  self.margin_n, c))
                pos = track.pos = None
                continue
            return False, queue, (f"{rid}: no collision-free start found for {queue[:self.window]} from HOME "
                                  f"(blocked by the robots planned before it)")
        if pos is not None and final_home:
            Lh = self.leg(rid, pos, "home")
            d, c = self.place(rid, others, track.K, Lh, self.margin_n, w=self.w_for(rid, "home"),
                              d_from=int(self.min_delay.get((rid, "home"), 0)))
            if d is None:
                return False, [], f"{rid}: no final return to HOME (internal error: escape vanished)"
            alt = self._park_alt(rid, track, others, pos, "home", Lh, d, self.margin_n, self.w_for(rid, "home"))
            if alt is not None:
                self._commit_park(rid, track, others, alt[0], alt[1], alt[4])
                Lh, d = alt[2], alt[3]
            c = self.cmin(rid, others, track.K, Lh, d, self.margin_n, w=self.w_for(rid, "home"))
            track.commit(CLeg(rid, "home", "home", None, track.K, d, Lh, 0, self.margin_n, c))
            track.pos = None
        elif pos is not None:                    # phase end (global group order): back HOME as a yield
            Lh = self.leg(rid, pos, "home")
            d, c = self.place(rid, others, track.K, Lh, self.margin_n,
                              w=self.w_for(rid, f"{rid.upper()}_YIELD_{n_yield + 1}{phase}"))
            if d is None:
                return False, [], f"{rid}: no return to HOME at the end of a group phase"
            w_y = self.w_for(rid, f"{rid.upper()}_YIELD_{n_yield + 1}{phase}")
            alt = self._park_alt(rid, track, others, pos, "home", Lh, d, self.margin_n, w_y)
            if alt is not None:
                self._commit_park(rid, track, others, alt[0], alt[1], alt[4])
                Lh, d = alt[2], alt[3]
            c = self.cmin(rid, others, track.K, Lh, d, self.margin_n, w=w_y)
            n_yield += 1
            track.commit(CLeg(rid, f"{rid.upper()}_YIELD_{n_yield}{phase}", "yield", None, track.K, d, Lh, 0,
                              self.margin_n, c))
            track.pos = None
        return True, [], ""

    def run_order(self, queues: dict, order: list) -> CoordResult:
        tracks = {r: Track(r, self.home[r]) for r in self.rids}
        failed = []
        use_prefix = self.group_order != "global" and self.PREFIX_CACHE
        pkey = (self.window, bool(self.park), self.park_min_n, self.w_base, self.n_branch)
        if self.group_order == "global":
            ranks = sorted({self.rank[w] for q in queues.values() for w in q})
            phases = [{r: [w for w in queues.get(r, []) if self.rank[w] == g] for r in self.rids} for g in ranks]
        else:
            phases = [{r: list(queues.get(r, [])) for r in self.rids}]
        for pi, ph in enumerate(phases):
            last = pi == len(phases) - 1
            if pi > 0:                                    # barrier: everyone is HOME and idle
                barrier = max(t.K for t in tracks.values())
                for t in tracks.values():
                    t.stand(barrier - t.K)
            for rid in order:
                if use_prefix:
                    pkey = pkey + ((rid, tuple(ph.get(rid, [])),
                                    tuple(sorted((k[1], v) for k, v in self.w_extra.items() if k[0] == rid)),
                                    tuple(sorted((k[1], v) for k, v in self.min_delay.items() if k[0] == rid))),)
                    hit = self._prefix.get(pkey)
                    if hit is not None:
                        tracks = {r: t.copy() for r, t in hit.items()}
                        self.stats["prefix_hits"] = self.stats.get("prefix_hits", 0) + 1
                        continue
                if not ph.get(rid):
                    continue
                others = [_Other(self, r, tracks[r].q) for r in self.rids if r != rid]
                has_more = any(phases[j].get(rid) for j in range(pi + 1, len(phases)))
                ok, rest, why = self.plan_robot(rid, tracks[rid], ph[rid], others,
                                                final_home=not has_more, phase="" if last else f"_G{pi + 1}")
                if not ok:
                    failed.append({"robot": rid, "welds": rest, "reason": why})
                    return CoordResult(False, tracks, order, failed, np.inf, dict(self.stats))
                if use_prefix:
                    if len(self._prefix) > 256:
                        self._prefix.clear()
                    self._prefix[pkey] = {r: t.copy() for r, t in tracks.items()}
        obj = max([cl.k_done for t in tracks.values() for cl in t.legs] + [0]) * self.dt   # executor end
        res = CoordResult(True, tracks, order, failed, obj, dict(self.stats))
        res.min_delay = dict(self.min_delay)
        return res

    # ------------------------------------------------------------------ parallel evaluation (trajgen.parallel)
    def run_orders(self, jobs: list, workers: int = 1, label: str = "") -> list:
        """[run_order(queues, order) with window / w_extra / min_delay] for jobs [(queues, order, window, w_extra[,
        min_delay])], in forked workers when ``workers`` > 1 (the results are those of the serial calls; the legs the
        workers planned are merged into this coordinator's cache). Restores ``window``, ``w_extra``, ``min_delay``."""
        from trajgen.parallel import pmap
        w0, x0, m0 = self.window, dict(self.w_extra), dict(self.min_delay)
        try:
            if workers <= 1 or len(jobs) <= 1:
                out = []
                for job in jobs:
                    q, o, win, wx = job[:4]
                    self.window, self.w_extra = max(1, int(win)), dict(wx or {})
                    self.min_delay = dict(job[4] if len(job) > 4 and job[4] else {})
                    out.append(self.run_order(q, list(o)))
                return out
            self._fork_keys = set(self._legs)
            res = pmap(_task_run_order, self, jobs, workers, self.log, label)
            out = []
            for r, new in res:
                self.merge_legs(new)
                out.append(r)
            return out
        finally:
            self.window, self.w_extra, self.min_delay = w0, x0, m0

    def new_legs(self) -> dict:
        """Legs planned since the last fork (worker side of run_orders / static eligibility)."""
        base = getattr(self, "_fork_keys", None)
        return {k: v for k, v in self._legs.items() if base is None or k not in base}

    def merge_legs(self, new: dict):
        for k, v in new.items():
            self._legs.setdefault(k, v)

    def static_eligible_all(self, pairs: list, workers: int = 1) -> dict:
        """{(rid, wid): static_eligible(rid, wid)} for pairs [(rid, wid)], in forked workers when workers > 1."""
        from trajgen.parallel import pmap
        if workers <= 1 or len(pairs) <= 1:
            return {p: self.static_eligible(*p) for p in pairs}
        self._fork_keys = set(self._legs)
        items = [(r, w, k) for r, w in pairs for k in range(len(self.reach.get((w, r), [])))]   # one task per option
        res = pmap(_task_static_eligible, self, items, workers, self.log, "eligibility")
        st = {}
        for it, (x, new) in zip(items, res):
            self.merge_legs(new)
            st[it] = x
        return {(r, w): self._eligible_combine([st[(r, w, k)] for k in range(len(self.reach.get((w, r), [])))])
                for r, w in pairs}

    def select(self, orders: list, results: list) -> CoordResult:
        """``run``'s choice among the results of ``orders`` (first strictly shortest; failures skipped)."""
        best, fails = None, []
        self.order_results = []
        for order, res in zip(orders, results):
            self.order_results.append((list(order), round(res.objective_s, 3) if res.ok else None))
            self.log(f"[coord] order {order}: {'ok' if res.ok else 'FAILED'} "
                     + (f"executor end {res.objective_s:.2f} s" if res.ok else str(res.failed)))
            if res.ok and (best is None or res.objective_s < best.objective_s - 1e-9):
                best = res
            if not res.ok:
                fails.append(res)
        if best is None:
            r = fails[-1] if fails else CoordResult(False, {}, [], [{"robot": None, "welds": [], "reason": "no order"}])
            r.stats = dict(self.stats)
            return r
        best.stats = dict(self.stats)
        return best

    def run(self, queues: dict, orders: list) -> CoordResult:
        best, fails = None, []
        self.order_results = []
        for order in orders:
            res = self.run_order(queues, order)
            self.order_results.append((list(order), round(res.objective_s, 3) if res.ok else None))
            self.log(f"[coord] order {order}: {'ok' if res.ok else 'FAILED'} "
                     + (f"executor end {res.objective_s:.2f} s" if res.ok else str(res.failed)))
            if res.ok and (best is None or res.objective_s < best.objective_s - 1e-9):
                best = res
            if not res.ok:
                fails.append(res)
        if best is None:
            r = fails[-1] if fails else CoordResult(False, {}, [], [{"robot": None, "welds": [], "reason": "no order"}])
            r.stats = dict(self.stats)
            return r
        best.stats = dict(self.stats)
        return best


def _task_run_order(coord, job):
    q, o, win, wx = job[:4]
    coord.window, coord.w_extra = max(1, int(win)), dict(wx or {})
    coord.min_delay = dict(job[4] if len(job) > 4 and job[4] else {})
    r = coord.run_order(q, list(o))
    return r, coord.new_legs()


def _task_static_eligible(coord, item):
    r = coord.static_eligible_option(*item)
    return r, coord.new_legs()


def priority_orders(rids, loads: dict, n_orders: int = 3) -> list:
    """Priority orders to try: by workload (longest route first), the scene order, the reverse workload order, then
    rotations; distinct, at most ``n_orders``."""
    import itertools
    rids = list(rids)
    cands = [sorted(rids, key=lambda r: (-loads.get(r, 0.0), rids.index(r))), rids,
             sorted(rids, key=lambda r: (loads.get(r, 0.0), rids.index(r)))]
    cands += [rids[k:] + rids[:k] for k in range(1, len(rids))]
    cands += [list(p) for p in itertools.permutations(rids)]
    out = []
    for c in cands:
        if c not in out:
            out.append(c)
        if len(out) >= n_orders:
            break
    return out


def legs_from_tracks(tracks: dict, rids, dt: float, sync: bool = True):
    """Committed tracks -> (player Legs on the executor clock, WaitForRobot deps {(rid, op): [(rj, op_j, delay_s)]}).

    Leg dispatch = when the robot's previous leg is complete. With ``sync`` every leg is held relative to the other
    robots' completion events instead of the clock: for each other robot rj, take its latest ExecuteTrajectory op_j
    that the schedule completes before this leg starts moving (and not before the robot's previous leg is
    dispatched); the plan then runs, in parallel with the previous leg, a branch WaitForRobot(op_j) -> Delay(delay_s)
    with delay_s = this leg's start - op_j's completion (trajgen.btplan.plan_from_sequences). So the leg starts at
    max(previous leg complete, every op_j complete + its delay): nominally exactly the scheduled start, and when
    another robot runs late (the executor completes legs at its 20 Hz polls, so lateness accumulates along a
    program) the leg follows it instead of keeping the clock. The leg's trajectory then has no stationary prefix;
    legs without such a gate keep the start delay as a stationary prefix after the previous leg."""
    from player.timeline import Leg

    done = {r: [(cl.k_done, cl) for cl in tracks[r].legs if cl.kind != "home"] for r in rids}
    legs, deps = [], {}
    for r in rids:
        prev_done = 0                       # the executor dispatches a robot's next leg when its previous one completes
        prev_disp = 0                       # dispatch of the previous leg (gate branches run in parallel with it)
        for cl in tracks[r].legs:
            if cl.kind == "weld":
                op, kind, otype, node = cl.target, "weld", "weld", "ExecuteTrajectory"
            elif cl.kind in ("yield", "park"):
                op, kind, otype, node = cl.target, "move", "move", "ExecuteTrajectory"
            else:
                op, kind, otype, node = "home", "home", "", "MoveHome"
            k_disp = prev_done
            dd = []
            if sync:
                for rj in rids:
                    if rj == r:
                        continue
                    prev = [x for x in done[rj] if prev_disp <= x[0] <= cl.k_start]
                    if prev:
                        kd, cj = max(prev, key=lambda x: x[0])
                        dd.append((rj, cj.target, round((cl.k_start - kd) * dt, 9)))
                if dd:
                    k_disp = cl.k_start
            d = cl.k_start - k_disp
            L = cl.leg.with_wait(d) if d > 0 else cl.leg
            times = np.round((k_disp + np.arange(len(L.Q))) * dt, 9)
            hold = cl.hold_n * dt
            leg = Leg(robot=r, op_id=op, kind=kind, t0=float(times[0]), t1=float(times[-1]), times=times,
                      positions=L.Q.copy(), phase=L.phase.copy(), hold_s=float(hold), vscale=1.0, operation_type=otype,
                      node_type=node, bb_key=f"traj_{r}_{op}")
            leg.t_done = float((cl.k_done) * dt)
            legs.append(leg)
            prev_done, prev_disp = cl.k_done, k_disp
            if dd:
                deps[(r, op)] = dd
    return legs, deps
