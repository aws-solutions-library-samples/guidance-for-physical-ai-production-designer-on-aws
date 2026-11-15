"""Steps 3-5 building blocks: per-phase speed caps, per-weld legs, and the dense two-arm step check.

Leg (one umarell ExecuteTrajectory / MoveHome): [wait] + [straight on-axis lift-off out of the previous weld] +
collision-checked joint transit + [straight Cartesian descent onto the weld]; it ends at rest ON the weld and is
followed by the dwell (hold). The return to HOME is a leg without descent. Every segment starts and ends at rest
and its duration is a multiple of the output sample interval.

Two-arm coordination = umarell's executor step model ("sync at rest"): the program is a sequence of steps; in a
step each arm runs at most one leg, both legs are dispatched together and the step ends when both legs and their
holds are over (the executor then adds its completion margin with BOTH arms standing still). A start delay is
encoded as a stationary prefix of the leg's own trajectory (so umarell's per-step validate_trajectory_set sees the
true timing). For every candidate step the two arms are checked jointly on a dense grid (>= 100 Hz, linear
interpolation of the 20 Hz samples = what umarell's controller executes): corrected-sphere clearance >
gate + margin and MuJoCo hull distance >= collision.arm_arm.hull_min_distance_m between the arms, both over the
+-skew dispatch window (spheres at whole-sample shifts, hulls at every dense shift). Single-arm rules are guaranteed by construction (IK
strokes and transits are checked) and re-verified at the end (trajgen.verify).

The search over steps (which welds, which arm, which delays) is in trajgen.search.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from trajgen.motion import Segment, concat_segments, plan_transit
from trajgen.reach import WeldOption


# ------------------------------------------------------------------------------------------- speeds
@dataclass
class Speeds:
    """Per-phase joint caps for one robot (rad/s, rad/s^2) and jerk windows."""
    v: dict
    a: dict
    j: dict
    label: str = "umarell"
    info: dict = field(default_factory=dict)


def make_speeds(scene, rid, mode: str = "umarell", jerk_factor: float | None = None) -> Speeds:
    """umarell: the scene's planning rules (transit v_s, accel a_max * rule(v_s); descent/lift-off at
    v_s * approach_fraction with the same rule; HOME at home_velocity_scale).
    fast: transit/HOME at velocity_scale, accel rule ``vscale`` (a_max * v_s), descent/lift-off unchanged speed
    cap (v_s * approach_fraction) with accel a_max * v_s; a looser jerk cap.
    full: transit / park / yield / HOME at 1.0 x the URDF velocity limit, 1.0 x the scene acceleration, jerk
    10 a_max/s; descent / lift-off exactly as umarell (trajgen.fullspeed, docs/p3_tgfull.md)."""
    if mode == "full":                          # round p3 TGFULL: non-weld motion at full URDF speed
        from trajgen.fullspeed import full_speeds
        return full_speeds(scene, rid, jerk_factor)
    pl = scene.planning
    r = scene.robot(rid)
    vs = pl.velocity_scale
    vh = min(pl.home_velocity_scale, vs)
    va = pl.approach_vscale
    if mode == "umarell":
        sc = {"transit": vs, "approach": va, "home": vh}
        acc = {k: pl.accel_scale(s) for k, s in sc.items()}
        jf = jerk_factor if jerk_factor is not None else (pl.max_jerk_factor or 4.0)
    elif mode == "fast":
        sc = {"transit": vs, "approach": va, "home": vs}
        acc = {"transit": vs, "approach": vs, "home": vs}
        jf = jerk_factor if jerk_factor is not None else (pl.max_jerk_factor or 8.0)
    else:
        raise ValueError(f"unknown speed mode {mode}")
    v = {k: r.v_max * s for k, s in sc.items()}
    a = {k: r.a_max * acc[k] for k in sc}
    j = {k: r.a_max * jf for k in sc}           # jerk cap jf * a_max per second (all phases)
    v["retract"], a["retract"], j["retract"] = v["approach"], a["approach"], j["approach"]
    return Speeds(v, a, j, mode, {"vscale": sc, "accel_frac": acc, "jerk_factor": jf})


# ------------------------------------------------------------------------------------------- legs
@dataclass
class LegPlan:
    rid: str
    target: str                  # weld id or "home"
    opt: WeldOption | None       # weld option reached (None for home)
    segments: list
    Q: np.ndarray                # (N, 6) samples at dt, Q[0] = start
    phase: np.ndarray            # (N,)
    hold_s: float
    vias: int = 0
    wait_n: int = 0

    @property
    def T(self):
        return (len(self.Q) - 1) * self.dt

    dt: float = 0.05

    def with_wait(self, n: int) -> "LegPlan":
        if n <= 0:
            return self
        Q = np.vstack([np.repeat(self.Q[:1], n, 0), self.Q])
        ph = np.concatenate([np.array(["wait"] * n, dtype="<U16"), self.phase])
        ph[0] = "wait"
        return LegPlan(self.rid, self.target, self.opt, self.segments, Q, ph, self.hold_s, self.vias, n, self.dt)


class Planner:
    def __init__(self, model, speeds: dict, dt: float, dwell_s: float, rrt_iters: int = 2000, log=print):
        self.m, self.sp, self.dt, self.dwell = model, speeds, float(dt), float(dwell_s)
        self.rrt_iters = rrt_iters
        self.rrt_seed = 0
        self._transit = {}
        self.log = log
        self.stats = {"transit_direct": 0, "transit_via": 0, "transit_rrt": 0, "transit_fail": 0}

    def seg(self, rid, phase, knots, cap=None):
        s = self.sp[rid]
        c = cap or phase
        return Segment(phase, knots, s.v[c], s.a[c], s.j[c], self.dt)

    def transit(self, rid, q0, q1):
        key = (rid, tuple(np.round(q0, 6)), tuple(np.round(q1, 6)))
        if key not in self._transit:
            vias = plan_transit(self.m, rid, q0, q1, rrt_iters=self.rrt_iters, seed=self.rrt_seed)
            if vias is None:
                self.stats["transit_fail"] += 1
            else:
                self.stats["transit_direct" if len(vias) <= 2 else "transit_via"] += 1
            self._transit[key] = vias
        return self._transit[key]

    def leg(self, rid, q_start, at: WeldOption | None, target):
        """Leg from q_start (resting on weld ``at`` if given) to ``target`` (WeldOption or "home")."""
        segs = []
        q = np.asarray(q_start, float)
        if at is not None:
            if np.abs(at.q - q).max() > 1e-6:
                raise ValueError("leg start is not the weld configuration")
            segs.append(self.seg(rid, "retract", at.liftoff))
            q = at.q_ret
        home = target == "home"
        q_goal = self.m.robots[rid].home if home else target.q_pre
        vias = self.transit(rid, q, q_goal)
        if vias is None:
            return None
        cap = "home" if home else "transit"
        for a, b in zip(vias[:-1], vias[1:]):
            segs.append(self.seg(rid, "home" if home else "transit", np.stack([a, b]), cap))
        if not home:
            segs.append(self.seg(rid, "approach", target.descent))
        segs = [s for s in segs if s.n > 0]
        if not segs:
            Q, ph = q[None], np.array(["transit"], dtype="<U16")
        else:
            Q, ph, _ = concat_segments(segs, self.dt)
        return LegPlan(rid, "home" if home else target.wid, None if home else target, segs, Q, ph,
                       0.0 if home else self.dwell, max(0, len(vias) - 2), 0, self.dt)


    def lift(self, rid, at: WeldOption):
        """The straight on-axis lift-off out of weld ``at`` alone, at the descent / lift-off caps, ending at rest at
        the retract pose (a parking leg, round p3); None if the stroke is empty."""
        segs = [s for s in [self.seg(rid, "retract", at.liftoff)] if s.n > 0]
        if not segs:
            return None
        Q, ph, _ = concat_segments(segs, self.dt)
        return LegPlan(rid, "park", None, segs, Q, ph, 0.0, 0, 0, self.dt)


# ------------------------------------------------------------------------------------------- pair checking
def _unique_pairs(ia, ib, nb: int):
    """np.unique(np.stack([ia, ib], 1), axis=0, return_index=True) for non-negative index pairs with ib < nb, via
    one int64 key (same sorted pairs, same first indices; much faster than the row-wise unique)."""
    ia = np.asarray(ia, np.int64)
    ib = np.asarray(ib, np.int64)
    key = ia * np.int64(nb) + ib
    u, first = np.unique(key, return_index=True)
    return np.stack([u // nb, u % nb], 1), first


class PairChecker:
    """Joint check of two arms over a step on a dense grid (``sub`` points per output sample)."""

    def __init__(self, model, dt: float, check_hz: float = 100.0, clearance_margin: float = 0.01, hull: bool = True,
                 umarell_parity: bool = True, skew_s: float = 0.0):
        self.m = model
        self.skew_n = int(round(float(skew_s) / float(dt)))      # dispatch skew window (output samples)
        self.parity = bool(umarell_parity and model.umarell is not None)
        self.gate_u = (model.umarell.warning_distance + clearance_margin) if self.parity else None
        self.dt = float(dt)
        self.sub = max(1, int(np.ceil(dt * check_hz - 1e-9)))
        self.gate = model.scene.collision.arm_arm_gate + clearance_margin
        self.pair_r = model.scene.collision.pair_radius
        self.hull = bool(hull and model.scene.collision.hull_backstop)
        # robot-robot hull buffer (collision.arm_arm.hull_min_distance_m), held at every alignment of the skew window
        self.hull_min = float(model.hull_min) if self.hull else 0.0
        self.hull_thr = self.hull_min
        self._cache = {}
        self._hcache = {}
        self._gcache = {}
        self._groups = {}
        self.n_checks = 0
        self.n_hull = 0
        self.n_exact_sph = 0
        self.n_pairs_sph = 0

    def dense(self, rid, Q):
        """(M, 6) linear interpolation of the samples, ``sub`` points per interval, and its spheres."""
        key = (rid, Q.shape, hash(Q.tobytes()))
        if key not in self._cache:
            Q = np.asarray(Q, float)
            if len(Q) > 1:
                f = np.arange(self.sub) / self.sub
                D = (Q[:-1, None] + f[None, :, None] * (Q[1:] - Q[:-1])[:, None]).reshape(-1, 6)
                D = np.vstack([D, Q[-1:]])
            else:
                D = Q.copy()
            S = self.m.pair_spheres(rid, D)
            U = self.m.umarell_spheres(rid, D) if self.parity else None
            if len(self._cache) > 400:
                self._cache.clear()
            self._cache[key] = (D, S, U)
        return self._cache[key]

    def dense_padded(self, rid, Q, pre: int, post: int):
        """``dense`` of [Q[0] x pre, Q, Q[-1] x post] composed from the cached dense arrays of Q (bitwise the same
        arrays: interpolating between equal samples gives that sample, and the FK / hull-centre / displacement
        computations are row-wise), with the hull-centre and step-displacement caches of the composed path filled
        from those of Q (round p3 speed-up: the priority coordinator checks the same leg at many start delays)."""
        D, S, U = self.dense(rid, Q)
        if pre <= 0 and post <= 0:
            return D, S, U
        s = self.sub
        a, b = pre * s, post * s

        def pad(X):
            if X is None:
                return None
            parts = []
            if a:
                parts.append(np.repeat(X[:1], a, 0))
            parts.append(X)
            if b:
                parts.append(np.repeat(X[-1:], b, 0))
            return np.concatenate(parts)
        D2, S2, U2 = pad(D), pad(S), pad(U)
        for X, X2, kind in ((S, S2, "S"), (U, U2, "U")):
            if X is not None:
                self._gcache[id(X2)] = (X2, (rid, kind), tuple(pad(Y) for Y in self.group_spheres(rid, X, kind)))
        if self.hull:
            G = self.hull_centres(rid, D)
            k2 = ("h", rid, D2.shape, hash(D2.tobytes()))
            if len(self._hcache) > 400:
                self._hcache.clear()
            self._hcache[k2] = pad(G)
            Di, _ = self.step_disp(rid, D)
            if len(D) > 1:
                parts = ([np.zeros(a)] if a else []) + [Di] + ([np.zeros(b)] if b else [])
                Di2 = np.concatenate(parts)
            else:
                Di2 = np.zeros(max(1, len(D2) - 1))
            e = np.empty(len(D2))
            if len(D2) > 1:
                e[0], e[-1] = Di2[0], Di2[-1]
                e[1:-1] = np.maximum(Di2[:-1], Di2[1:])
            else:
                e[:] = 0.0
            self._hcache[("e", rid, D2.shape, hash(D2.tobytes()))] = (Di2, e)
        return D2, S2, U2

    # ------------------------------------------------------------------ sphere-group lower bounds
    def sphere_groups(self, rid, kind: str = "S", thr: float | None = None):
        """Fixed partition of robot rid's pair-check spheres (kind 'S': corrected, 'U': umarell parity) into groups
        of spheres that stay within ``thr`` of each other over 64 random configurations (greedy). Returns (perm,
        starts, counts). Any partition gives a valid bound (the group spheres are recomputed per configuration);
        compact groups give a tight one."""
        thr = 0.3 if thr is None else float(thr)
        key = (rid, kind, thr)
        if key not in self._groups:
            r = self.m.robots[rid]
            Q = np.random.default_rng(5).uniform(r.q_lo, r.q_hi, (64, 6))
            S = self.m.pair_spheres(rid, Q) if kind == "S" else self.m.umarell_spheres(rid, Q)
            n = S.shape[1]
            Dm = np.zeros((n, n))
            for k in range(len(S)):
                Dm = np.maximum(Dm, np.linalg.norm(S[k][:, None] - S[k][None], axis=-1))
            groups = []
            for i in range(n):
                for g in groups:
                    if Dm[i, g].max() <= thr:
                        g.append(i)
                        break
                else:
                    groups.append([i])
            perm = np.array([i for g in groups for i in g], int)
            counts = np.array([len(g) for g in groups], int)
            starts = np.concatenate([[0], np.cumsum(counts)[:-1]]).astype(int)
            self._groups[key] = (perm, starts, counts)
        return self._groups[key]

    def group_index(self, rid, kind: str = "S", thr: float = 0.3) -> np.ndarray:
        """(G, gmax) sphere indices of the groups of ``sphere_groups`` (rows padded with the group's first index)."""
        key = ("pad", rid, kind, thr)
        if key not in self._groups:
            perm, starts, counts = self.sphere_groups(rid, kind, thr)
            g = int(counts.max())
            P = np.empty((len(counts), g), int)
            for i, (a, n) in enumerate(zip(starts, counts)):
                P[i, :n] = perm[a:a + n]
                P[i, n:] = perm[a]
            self._groups[key] = P
        return self._groups[key]

    def group_spheres(self, rid, S, kind: str = "S"):
        """Bounding spheres of the sphere groups at every row of S, two levels (coarse groups of ``sphere_groups``
        thr 0.9, fine thr 0.3) and of the whole set: (C1 (N, G1, 3), R1 (N, G1), C2, R2, c0 (N, 3), r0 (N,)), cached
        by array identity."""
        ent = self._gcache.get(id(S))
        if ent is not None and ent[0] is S and ent[1] == (rid, kind):
            return ent[2]
        X = np.asarray(S, float)
        out = []
        for thr in (0.9, 0.3):
            perm, starts, counts = self.sphere_groups(rid, kind, thr)
            P = X[:, perm]
            C = np.add.reduceat(P, starts, axis=1) / counts[None, :, None]
            d = np.linalg.norm(P - np.repeat(C, counts, axis=1), axis=-1)
            out += [C, np.maximum.reduceat(d, starts, axis=1)]
        c0 = X.mean(1)
        r0 = np.linalg.norm(X - c0[:, None], axis=-1).max(1)
        out = tuple(out + [c0, r0])
        if len(self._gcache) > 128:
            self._gcache.clear()
        self._gcache[id(S)] = (S, (rid, kind), out)
        return out

    @staticmethod
    def group_lb(GA, GB, ia, ib, cut: float = np.inf, chunk: int = 4096) -> np.ndarray:
        """(len(ia),) lower bound of min |sphere centre a - sphere centre b| over the rows (ia[k], ib[k]) from the
        group spheres GA / GB (``group_spheres``): the whole-set bound, refined with the coarse and then the fine
        groups where it is still below ``cut``."""
        C1A, R1A, C2A, R2A, c0A, r0A = GA
        C1B, R1B, C2B, R2B, c0B, r0B = GB
        out = np.linalg.norm(c0A[ia] - c0B[ib], axis=-1) - r0A[ia] - r0B[ib]
        for CA, RA, CB, RB in ((C1A, R1A, C1B, R1B), (C2A, R2A, C2B, R2B)):
            sel = np.flatnonzero(out < cut)
            for k in range(0, len(sel), chunk):
                q = sel[k:k + chunk]
                a, b = ia[q], ib[q]
                d = CA[a][:, :, None] - CB[b][:, None]
                dd = np.sqrt(np.einsum("nabk,nabk->nab", d, d)) - RA[a][:, :, None] - RB[b][:, None]
                out[q] = np.maximum(out[q], dd.reshape(len(a), -1).min(1))
        return out

    def sphere_min(self, ra, rb, SA, SB, pairs, pr: float, gate: float, kind: str = "S", want_min: bool = True):
        """Exact (min clearance over the row pairs, indices k of the pairs with clearance <= gate) of
        ``SceneModel.pair_clearance_spheres`` over pairs[:, 0] / pairs[:, 1] of SA / SB, computing the exact value
        only where a sphere-group lower bound does not exclude it (bitwise the same min and the same failing pairs
        as evaluating every pair: rows are computed independently). ``want_min=False``: only the failing pairs are
        exact; the returned min is then only the min over the rows that were evaluated (a pass / fail check)."""
        P = len(pairs)
        self.n_pairs_sph += P
        GA = self.group_spheres(ra, SA, kind)
        GB = self.group_spheres(rb, SB, kind)
        lb = self.group_lb(GA, GB, pairs[:, 0], pairs[:, 1], cut=gate + 2.0 * pr + 0.05) - 2.0 * pr - 1e-7
        c = np.full(P, np.inf)
        done = np.zeros(P, bool)

        def ex(idx):
            for k in range(0, len(idx), 512):
                p = pairs[idx[k:k + 512]]
                c[idx[k:k + 512]] = self.m.pair_clearance_spheres(SA[p[:, 0]], SB[p[:, 1]], pr)
            done[idx] = True
            self.n_exact_sph += len(idx)

        need = np.flatnonzero(lb <= gate)
        if len(need):
            ex(need)
        self.n_need_sph = getattr(self, "n_need_sph", 0) + len(need)
        if not want_min:
            return (float(c.min()) if P else np.inf), np.flatnonzero(c <= gate)
        m = float(c[done].min()) if done.any() else np.inf
        cand = np.flatnonzero(~done & (lb < m))           # rows that may still hold the minimum, by bound
        cand = cand[np.argsort(lb[cand], kind="stable")]
        for i in range(0, len(cand), 256):
            k = cand[i:i + 256]
            k = k[lb[k] < m]
            if not len(k):
                break
            ex(k)
            m = min(m, float(c[k].min()))
        cmin = float(c.min()) if P else np.inf
        return cmin, np.flatnonzero(c <= gate)

    def check(self, QA, dA: int, QB, dB: int, n_tail: int, ra: str | None = None, rb: str | None = None,
              want_min: bool = True):
        """Arm ``ra`` runs samples QA after dA samples of standing at QA[0], arm ``rb`` likewise; both then stand for
        ``n_tail`` samples after the later end. Returns (ok, min clearance, first bad time). Defaults: the scene's
        first two robots."""
        self.n_checks += 1
        ra = self.m.rids[0] if ra is None else ra
        rb = self.m.rids[1] if rb is None else rb
        DA, SA, UA = self.dense(ra, QA)
        DB, SB, UB = self.dense(rb, QB)
        s = self.sub
        N = max(dA * s + len(DA), dB * s + len(DB)) + n_tail * s
        ia = np.clip(np.arange(N) - dA * s, 0, len(DA) - 1)
        ib0 = np.arange(N) - dB * s
        return self.check_window(ra, rb, DA, SA, UA, DB, SB, UB, ia, ib0, self.skew_n, want_min=want_min)

    def check_window(self, ra, rb, DA, SA, UA, DB, SB, UB, ia, ib0, skew_n: int, want_min: bool = True):
        """Arm ra at dense indices ``ia`` against arm rb at the nominal dense indices ``ib0`` (unclipped) with arm rb
        dispatched up to ``skew_n`` output samples early or late: spheres (corrected + parity) at the nominal
        alignment and at every whole-sample shift; hull distance >= ``hull_min`` at the nominal alignment and at
        EVERY dense shift within the window (round-3 finding: a 50-60 ms lag put the castings, which the spheres do
        not cover, into contact). Returns (ok, min sphere clearance, first bad time)."""
        s, nb = self.sub, len(DB)
        c = np.inf
        if skew_n:
            sh = [t * s for t in range(-skew_n, skew_n + 1) if t != 0]
            IA = np.concatenate([ia] * len(sh))
            IB = np.concatenate([np.clip(ib0 + x, 0, nb - 1) for x in sh])
            ok, c, tb = self.check_pairs(ra, rb, DA, SA, UA, DB, SB, UB, IA, IB, hull=False, want_min=want_min)
            if not ok:
                return ok, c, tb
        ok, c2, tb = self.check_pairs(ra, rb, DA, SA, UA, DB, SB, UB, ia, np.clip(ib0, 0, nb - 1), hull=False,
                                      want_min=want_min)
        c = min(c, c2)
        if not ok or not self.hull:
            return ok, c, tb
        sh = range(-skew_n * s, skew_n * s + 1)    # (hull_pairs certifies half a step around each: continuous)
        IA = np.concatenate([ia] * len(sh))
        IB = np.concatenate([np.clip(ib0 + x, 0, nb - 1) for x in sh])
        ok, tb = self.hull_pairs(ra, rb, DA, SA, DB, SB, IA, IB)
        if not ok:
            tb = (tb % len(ia)) / s * self.dt
        return ok, c, tb

    def check_min(self, QA, dA: int, QB, dB: int, n_tail: int, ra: str | None = None, rb: str | None = None) -> float:
        """The min sphere clearance ``check`` returns for a passing step (reporting; no hull check)."""
        ra = self.m.rids[0] if ra is None else ra
        rb = self.m.rids[1] if rb is None else rb
        DA, SA, UA = self.dense(ra, QA)
        DB, SB, UB = self.dense(rb, QB)
        s = self.sub
        N = max(dA * s + len(DA), dB * s + len(DB)) + n_tail * s
        ia = np.clip(np.arange(N) - dA * s, 0, len(DA) - 1)
        ib0 = np.arange(N) - dB * s
        return self.window_min(ra, rb, DA, SA, UA, DB, SB, UB, ia, ib0, self.skew_n)

    def window_min(self, ra, rb, DA, SA, UA, DB, SB, UB, ia, ib0, skew_n: int) -> float:
        """The min sphere clearance ``check_window`` returns for a passing check (every whole-sample shift within
        the window and the nominal alignment, both sphere models), without the hull check."""
        s, nb = self.sub, len(DB)
        c = np.inf
        if skew_n:
            sh = [t * s for t in range(-skew_n, skew_n + 1) if t != 0]
            IA = np.concatenate([ia] * len(sh))
            IB = np.concatenate([np.clip(ib0 + x, 0, nb - 1) for x in sh])
            _, c, _ = self.check_pairs(ra, rb, DA, SA, UA, DB, SB, UB, IA, IB, hull=False)
        _, c2, _ = self.check_pairs(ra, rb, DA, SA, UA, DB, SB, UB, ia, np.clip(ib0, 0, nb - 1), hull=False)
        return min(c, c2)

    def hull_centres(self, rid, D):
        """Cached (len(D), F, 3) hull-group centres (SceneModel.hull_group_centres) of dense samples D."""
        key = ("h", rid, D.shape, hash(D.tobytes()))
        if key not in self._hcache:
            if len(self._hcache) > 400:
                self._hcache.clear()
            self._hcache[key] = self.m.hull_group_centres(rid, D).astype(np.float32)
        return self._hcache[key]

    def step_disp(self, rid, D):
        """Cached (interval displacement bound (len(D) - 1,), per-sample max of the adjacent intervals (len(D),))
        of the dense path D (trajgen.certify)."""
        key = ("e", rid, D.shape, hash(D.tobytes()))
        if key not in self._hcache:
            from trajgen.certify import interval_disp
            Di = interval_disp(self.m, rid, D)
            e = np.empty(len(D))
            if len(D) > 1:
                e[0], e[-1] = Di[0], Di[-1]
                e[1:-1] = np.maximum(Di[:-1], Di[1:])
            else:
                e[:] = 0.0
            self._hcache[key] = (Di, e)
        return self._hcache[key]

    def hull_pairs(self, ra, rb, DA, SA, DB, SB, ia, ib):
        """Hull distance >= hull_min for the dense index pairs (ia[k], ib[k]) AND, continuously, within half a dense
        step of each pair in time and in relative shift (round-4 finding: inter-sample dips): a pair passes when its
        MuJoCo distance - (e_a + e_b) / 2 >= hull_min (e = the hull-point displacement bound of the adjacent dense
        intervals), else the branch and bound of trajgen.certify decides. MuJoCo only where the per-frame bounding
        spheres of the hulls (SceneModel.hull_groups, a lower bound of the hull distance) are closer than hull_min +
        that margin. Returns (ok, index k of the first bad pair or None)."""
        pairs, first = _unique_pairs(ia, ib, len(DB))
        GA, GB = self.hull_centres(ra, DA), self.hull_centres(rb, DB)
        rA, rB = self.m.hull_groups(ra)[2].astype(np.float32), self.m.hull_groups(rb)[2].astype(np.float32)
        IA, eA = self.step_disp(ra, DA)
        IB, eB = self.step_disp(rb, DB)
        lip = 0.5 * (eA[pairs[:, 0]] + eB[pairs[:, 1]])
        near = []
        for k in range(0, len(pairs), 4096):
            p = pairs[k:k + 4096]
            d = GA[p[:, 0]][:, :, None] - GB[p[:, 1]][:, None]
            lb = (np.sqrt(np.einsum("nabk,nabk->nab", d, d)) - rA[None, :, None] - rB[None, None]).reshape(len(p), -1).min(1)
            near.append(k + np.flatnonzero(lb < self.hull_thr + lip[k:k + 4096] + 1e-4))
        near = np.concatenate(near) if near else np.zeros(0, int)
        self.n_hull += len(near)
        if len(near):
            dist = self.m.mj_pair_min_distance(ra, rb, DA[pairs[near, 0]], DB[pairs[near, 1]])
            hit = dist < self.hull_thr
            if hit.any():
                return False, int(first[near[hit]].min())
            unc = np.flatnonzero(dist - lip[near] < self.hull_thr)
            if len(unc):
                from trajgen.certify import certify_rects
                sel = near[unc]
                self.n_certify = getattr(self, "n_certify", 0) + len(sel)
                cr = certify_rects(self.m, ra, rb, DA, DB, pairs[sel, 0].astype(float), pairs[sel, 1].astype(float),
                                   self.hull_thr, DA=IA, DB=IB, d0=dist[unc])
                if not cr["ok"]:
                    return False, int(first[sel].min())
        return True, None

    def check_pairs(self, ra, rb, DA, SA, UA, DB, SB, UB, ia, ib, hull: bool | None = None, want_min: bool = True):
        """Check the aligned dense index pairs (ia[k], ib[k]). Returns (ok, min clearance, first bad time)."""
        hull = self.hull if hull is None else (hull and self.hull)
        s = self.sub
        # only distinct (ia, ib) pairs matter (both standing -> one sample)
        pairs, first = _unique_pairs(ia, ib, len(SB))
        cmin, badk = self.sphere_min(ra, rb, SA, SB, pairs, self.pair_r, self.gate, "S", want_min)
        if len(badk):
            bad = first[badk].min()
            return False, cmin, bad / s * self.dt
        if self.parity:
            cu, badu = self.sphere_min(ra, rb, UA, UB, pairs, self.m.umarell.pair_radius, self.gate_u, "U", want_min)
            if len(badu):
                return False, min(cmin, cu), first[badu].min() / s * self.dt
            cmin = min(cmin, cu)
        if hull:
            ok, k = self.hull_pairs(ra, rb, DA, SA, DB, SB, ia, ib)
            if not ok:
                return False, cmin, k / s * self.dt
        return True, cmin, None
