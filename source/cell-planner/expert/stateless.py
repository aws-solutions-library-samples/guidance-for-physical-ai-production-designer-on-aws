"""Markov (stateless) two-arm weld expert: pi*(obs) -> action, for DAgger labelling (Track G2).

``StatelessExpert(env).act(obs) -> (12,)`` normalised joint velocities, like ``ScriptedExpert.act``, but the
action is a deterministic function of the observation only (both arms' ``robot_q``, goals, task
status / owner / queue rank). There is no committed plan, no reservation table and no clock, so it can
label ANY state a learner visits, and its labels are consistent with what the GNN policy observes.
(Results of deterministic sub-computations -- weld IK per goal pose, segment validity -- are memoised;
memoisation does not change the input -> output map.)

Per arm (``single-arm nominal controller``), phase inferred from the state:
  descent  q is in the joint-space tube (``tube`` rad, max-norm) of the current goal's descent path
           (pre-weld -> weld, straight Cartesian line along the approach vector, fixed orientation, from
           ``expert.ik.WeldIK.solve_axis``): follow the path at 0.4 x speed; at its end servo onto the weld
           pose and hold (the env counts the dwell and then switches the goal).
  retract  q is in the tube of a DONE weld's descent path (this arm's), off its first sample: follow it
           backwards to the pre-weld (lift-off along -approach). Fallback (tube missed but the TCP is on
           the weld's approach line below the standoff): Cartesian DLS lift-off along -approach.
  transit  otherwise: straight joint-space line to the goal's pre-weld configuration if that segment is
           free of hull self-collision / floor (``ScriptedExpert.segment_ok``, coal hulls, >= 5 mm); else
           via a fixed via of the goal (pre-weld with a2/a3 at HOME, "raised"), else via HOME. Speed:
           synchronised (straight line in joint space), full speed, P-slowdown over the last 3 steps.
  idle     no goal: retract if needed, then transit HOME (park).
The weld IK solution per goal pose is chosen deterministically: the usable branch of the nearest
canonical weld in data/weld_ik.json with the lowest weighted joint distance from HOME (+ penalties for
a thin hull margin / short standoff), refined to the actual (noisy) pose; other branches as fallback.

Arm-arm coordination (umarell sphere metric), from observable state only:
  * nominal swept path S_i: the configurations the single-arm controller would pass through from the
    current state until it rests (on its weld, or HOME), sampled every ``sweep_res`` rad.
  * priority arm p = the arm with more remaining welds (current goal included); ties -> ``tie_robot``.
    It changes only when p completes a weld (while it stands on that weld).
  * p moves iff clearance(S_p, q_n) > ``thr`` (its whole remaining sweep against the other arm's current
    configuration). The other arm n moves iff clearance(S_n, S_p) > ``thr`` (n's sweep against p's whole
    sweep, not time-aligned, so it is safe whatever p's timing), else it holds.
  * p blocked by n: p holds; n finishes a descent / dwell / retract it is in (if that is clear of p's
    current pose), otherwise n evades towards HOME (retract + transit HOME) if that is clear of p's pose.
  Safety argument (kinematic env, both arms follow this rule): p's next pose lies on S_p, which is clear of
  n's current pose; n's next pose lies on S_n, which is clear of all of S_p. Margin ``thr`` = 0.20 m gate
  + sampling / sub-step slack.
Clearances between swept sets are computed with a KD-tree on the sphere centres (uniform pair radius 0.12).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from env.cell import ROBOT_IDS
from expert.ik import APPROACH_OFFSET_M
from expert.scripted import ScriptedExpert, resample_path, rot6_to_R

W_COST = np.array([1.0, 1.0, 1.0, 0.6, 0.6, 0.4])


@dataclass
class GoalPlan:
    rid: str
    pos: np.ndarray
    approach: np.ndarray
    q_weld: np.ndarray | None = None
    path: np.ndarray | None = None        # (M, 6) pre-weld -> weld, one sample per descent control step
    q_pre: np.ndarray | None = None
    standoff: float = 0.0
    vias: list = field(default_factory=list)   # fixed via candidates (raised pre-weld, HOME)
    penalty: float = 0.0                       # thin hull margin / short standoff
    ok: bool = False


class StatelessExpert:
    def __init__(self, env=None, cell=None, tube: float = 0.03, retract_tube: float = 0.02, entry_tol_deg: float = 12.0, transit_margin: float = 0.03, liftoff_step: float = 0.025, via_tol: float = 0.02, yield_mode: str = "home", sweep_res: float = 0.015, thr: float = 0.26,
                 tie_robot: str = "robot2", approach_frac: float = 0.4, slow_steps: float = 3.0,
                 priority: str = "fixed", lookahead_legs: int = 2, verbose: bool = False):
        self.sx = ScriptedExpert(env, cell=cell)        # geometry checks, IK, table, sphere model
        self.env = env
        self.cell = self.sx.cell
        self.sm = self.sx.sm
        self.dt = self.sx.dt
        self.vmax = {r: self.sx.vmax[r] for r in ROBOT_IDS}
        self.home = {r: self.cell.robots[r].home.copy() for r in ROBOT_IDS}
        self.tube, self.sweep_res, self.thr = tube, sweep_res, thr
        self.retract_tube = retract_tube
        self.entry_tol_deg = entry_tol_deg
        self.transit_margin = transit_margin
        self.liftoff_step = liftoff_step
        self.via_tol = via_tol
        self.yield_mode = yield_mode
        self.tie_robot, self.approach_frac, self.slow_steps = tie_robot, approach_frac, slow_steps
        self.priority = priority
        self.lookahead_legs = lookahead_legs
        self.verbose = verbose
        from env.kinematics import ArmKinematics
        self.kin = {r: ArmKinematics(self.cell.robots[r]) for r in ROBOT_IDS}
        self._uniq = {r: self.sm._arm[r].uniq for r in ROBOT_IDS}
        self._plans: dict = {}
        self._seg: dict = {}
        self._legs: dict = {}
        self.stats = {"modes": {}, "coord": {}, "plan_fail": [], "deadlock": 0}
        self.last = {}

    def reset(self, obs=None):
        """Nothing to reset (stateless); clears per-episode statistics only."""
        self.stats = {"modes": {}, "coord": {}, "plan_fail": [], "deadlock": 0}

    # ------------------------------------------------------------------ memoised geometry
    def config_ok(self, rid, q, margin=None) -> bool:
        """Hull self-collision (coal) >= margin (default: the IK table's 5 mm), floor, fixture boxes."""
        c = self.sx.ik.check(rid, q)
        if not c.valid or (margin is not None and c.self_dist < margin):
            return False
        return bool(self.sm.obstacle_free(rid, q)) if self.sx.fixture else True

    def seg_ok(self, rid, a, b, margin=None) -> bool:
        a, b = np.asarray(a, float), np.asarray(b, float)
        key = (rid, tuple(np.round(a, 4)), tuple(np.round(b, 4)), margin)
        v = self._seg.get(key)
        if v is None:
            n = max(1, int(np.ceil(np.abs(b - a).max() / self.sx.check_res)))
            v = all(self.config_ok(rid, a + (b - a) * j / n, margin) for j in range(1, n + 1))
            if len(self._seg) > 50_000:
                self._seg.clear()
            self._seg[key] = v
        return v

    def goal_plans(self, rid, pos, R, approach) -> list:
        """All usable IK branches of a goal pose (memoised), sorted by their cost from HOME."""
        key = (rid, tuple(np.round(pos, 4)), tuple(np.round(R[:, :2].ravel(), 3)))
        gps = self._plans.get(key)
        if gps is None:
            gps = self._make_plans(rid, np.asarray(pos, float), R, np.asarray(approach, float))
            if len(self._plans) > 2_000:
                self._plans.clear()
            self._plans[key] = gps
        return gps

    def goal_plan(self, rid, pos, R, approach, q=None) -> GoalPlan:
        """The branch for configuration q: the one whose descent tube contains q, else the lowest
        (weighted joint distance from q to its pre-weld + penalties). q None: the HOME-cost branch."""
        gps = self.goal_plans(rid, pos, R, approach)
        if not gps:
            return GoalPlan(rid, np.asarray(pos, float), np.asarray(approach, float))
        if q is None or len(gps) == 1:
            return gps[0]
        q = np.asarray(q, float)
        best, bd = None, np.inf
        for gp in gps:
            d = float(np.abs(gp.path - q).max(-1).min())
            if d < bd:
                best, bd = gp, d
        if bd < self.tube:
            return best
        return min(gps, key=lambda gp: float(np.sum(W_COST * np.abs(gp.q_pre - q))) + gp.penalty)

    def _make_plans(self, rid, pos, R, approach) -> list:
        sx, ik = self.sx, self.sx.ik
        app = approach / (np.linalg.norm(approach) + 1e-12)
        extra = (lambda q: bool(self.sm.obstacle_free(rid, q))) if sx.fixture else None
        kw = dict(approach=approach, standoff=APPROACH_OFFSET_M, min_standoff=0.045, n_steps=20,
                  extra_check=extra, max_spin=sx.max_spin, tilts=sx.tilts)
        home = self.home[rid]
        hints = sx._hints(rid, pos)
        sols = []
        for h in hints:     # each canonical branch refined to the actual (noisy) pose
            r = ik.solve_axis(rid, p=pos, R=R, hints=[h], spin_window=15.0, **kw)
            sols += [c for c in r if c.path is not None and len(c.path) >= 10]
        if not sols:
            seeds = [home] + ik.default_seeds(rid, n_random=150, target_world=pos)
            sols = [c for c in ik.solve_axis(rid, p=pos, R=R, seeds=seeds, **kw) if c.path is not None and len(c.path) >= 10]
        out = []
        for c in sols:
            if any(np.abs(c.q - g.q_weld).max() < 0.15 for g in out):
                continue
            desc = c.path[::-1]                                  # pre -> weld
            step = self.vmax[rid] * self.approach_frac * self.dt
            d = resample_path(desc, step)
            if not sx.dense_ok(rid, desc[0], d):
                continue
            gp = GoalPlan(rid, pos, app)
            gp.q_weld, gp.path = c.q.copy(), np.vstack([desc[:1], d])
            gp.q_pre = gp.path[0].copy()
            gp.standoff = float(c.standoff_m)
            gp.penalty = (0.5 if len(c.path) < 21 else 0.0) + (1.0 if c.thin else 0.0)
            raised = gp.q_pre.copy(); raised[1:3] = home[1:3]
            gp.vias = [raised, home.copy()]
            gp.ok = True
            out.append(gp)
        out.sort(key=lambda g: float(np.sum(W_COST * np.abs(g.q_pre - home))) + g.penalty)
        if not out:
            self.stats["plan_fail"].append((rid, np.round(pos, 3).tolist()))
        return out

    # ------------------------------------------------------------------ obs parsing
    def _goals(self, obs):
        return self.sx._goals(obs)

    def _done_welds(self, obs, i):
        """[(pos, R, approach)] of done welds owned by arm i."""
        st = np.asarray(obs["task_status"]) > 0.5
        own = np.asarray(obs["task_owner"])[:, i] > 0.5
        m = np.asarray(obs["task_mask"]) > 0.5
        out = []
        for k in np.flatnonzero(st & own & m):
            out.append((np.asarray(obs["task_pos"][k], float), rot6_to_R(np.asarray(obs["task_rot6"][k], float)),
                        np.asarray(obs["task_approach"][k], float)))
        return out

    def _remaining(self, obs, i):
        own = np.asarray(obs["task_owner"])[:, i] > 0.5
        open_ = (np.asarray(obs["task_status"]) < 0.5) & (np.asarray(obs["task_mask"]) > 0.5)
        return int((own & open_).sum())

    # ------------------------------------------------------------------ single-arm nominal controller
    @staticmethod
    def _tube_idx(path, q):
        d = np.abs(path - q).max(-1)
        k = int(np.argmin(d))
        return k, float(d[k])

    def _line(self, a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.ceil(np.abs(b - a).max() / self.sweep_res)))
        return a + (b - a) * (np.arange(1, n + 1) / n)[:, None]

    def _transit_target(self, rid, q, q_goal, vias):
        """(next joint-space target, remaining via points incl. q_goal). Direct line, else via a fixed via
        (raised pre-weld, HOME); first with a comfortable hull margin (``transit_margin``), then the IK table's."""
        home = self.home[rid]
        cands = list(vias) + ([home] if not any(np.abs(v - home).max() < 1e-9 for v in vias) else [])
        for m in (self.transit_margin, None):
            if self.seg_ok(rid, q, q_goal, m):
                return q_goal, [q_goal]
            for v in cands:
                near = np.abs(q - v).max() < self.via_tol     # via reached (tolerance: no vanishing labels)
                if near and self.seg_ok(rid, v, q_goal, m):
                    return q_goal, [q_goal]
                if not near and self.seg_ok(rid, q, v, m) and self.seg_ok(rid, v, q_goal, m):
                    return v, [v, q_goal]
        self.stats["modes"]["transit_blocked"] = self.stats["modes"].get("transit_blocked", 0) + 1
        return q_goal, [q_goal]

    def _retract(self, rid, i, q, obs):
        """(target, sweep) if arm i is on a done weld's descent path / approach line, else None."""
        tcp = None
        for pos, R, app in self._done_welds(obs, i):
            if tcp is None:
                tcp = self.kin[rid].tcp(q)[0]
            if np.linalg.norm(tcp - pos) > 0.15:
                continue
            gp = self.goal_plan(rid, pos, R, app, q)
            a = app / (np.linalg.norm(app) + 1e-12)
            rel = tcp - pos
            depth = float(rel @ -a)                    # height above the weld along -approach
            lat = np.linalg.norm(rel - (-a) * depth)
            so = gp.standoff if gp.ok else APPROACH_OFFSET_M
            if gp.ok and depth < so - 0.005:           # only below the pre-weld height (else transit <-> retract)
                k, d = self._tube_idx(gp.path, q)
                if d < self.retract_tube:
                    if k == 0:
                        return None
                    return gp.path[k - 1], gp.path[k - 1::-1]
            # hysteresis from the observed joint velocity (Markov in the obs): while already lifting off, continue to
            # standoff + 3 cm; otherwise start only below standoff - 3 cm (else transit <-> lift-off livelock)
            qd = np.asarray(obs["robot_qd"][i], float)
            v_up = float((self.kin[rid].jacobian(q)[:3] @ qd) @ -a)
            top = so + 0.03 if v_up > 0.05 else so - 0.03
            if lat < 0.02 and -0.02 < depth < top and \
                    float(-self.kin[rid].tcp(q)[1][:, 0] @ a) > np.cos(np.deg2rad(10.0)):
                tgt = self._dls_step(rid, q, tcp - a * self.liftoff_step)   # lift-off, orientation kept
                return tgt, np.vstack([tgt])
        return None

    def _dls_step(self, rid, q, p_target, damping=0.03):
        """One DLS step: TCP to p_target, orientation kept."""
        p, _ = self.kin[rid].tcp(q)
        J = self.kin[rid].jacobian(q)
        e = np.concatenate([p_target - p, np.zeros(3)])
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e)
        return q + dq

    def nominal(self, rid, i, q, goal, obs, evade=False):
        """Single-arm controller: (mode, joint target for this step, swept path (N,6) incl. q)."""
        if goal is not None and not evade:
            pos, R, app, _ = goal
            gp = self.goal_plan(rid, pos, R, app, q)
            if gp.ok:
                k, d = self._tube_idx(gp.path, q)
                if d < self.tube and (k > 0 or d < 0.5 * self.tube):
                    if k > 0 and self._reapproach(i, k, gp, obs):
                        return "reapproach", gp.path[k - 1], np.vstack([q, gp.path[k - 1::-1], gp.path[1:]])
                    if k >= len(gp.path) - 1 or np.abs(q - gp.q_weld).max() < 1e-4:
                        mode = "hold" if np.abs(q - gp.q_weld).max() < 1e-3 else "descent"
                        return mode, gp.q_weld, np.vstack([q, gp.q_weld])
                    return "descent", gp.path[k + 1], np.vstack([q, gp.path[k + 1:]])
        r = self._retract(rid, i, q, obs)
        if r is not None:
            tgt, sw = r
            rest = self._after_retract(rid, i, sw[-1], goal, obs, evade)
            return "retract", tgt, np.vstack([q, sw, rest])
        if goal is None or evade:
            home = self.home[rid]
            if np.abs(q - home).max() < 1e-4:
                return "park", home, q[None]
            tgt, vias = self._home_target(rid, q)
            return ("evade" if evade else "home"), tgt, self._sweep_lines(q, vias)
        pos, R, app, _ = goal
        gp = self.goal_plan(rid, pos, R, app, q)
        if not gp.ok:   # unreachable: park HOME
            home = self.home[rid]
            tgt, vias = self._transit_target(rid, q, home, [])
            return "fail", tgt, self._sweep_lines(q, vias)
        tgt, vias = self._transit_target(rid, q, gp.q_pre, gp.vias)
        return "transit", tgt, np.vstack([self._sweep_lines(q, vias), gp.path[1:]])

    def _home_target(self, rid, q):
        """Retreat HOME radially: first a2/a3 to their HOME values (arm pulled back towards its own base,
        a1 and the wrist kept), then a joint-space line HOME. Falls back to the plain transit rule."""
        home = self.home[rid]
        raised = q.copy(); raised[1:3] = home[1:3]
        for m in (self.transit_margin, None):
            if np.abs(raised - q).max() > self.via_tol and self.seg_ok(rid, q, raised, m) and self.seg_ok(rid, raised, home, m):
                return raised, [raised, home]
            if m is not None and self.seg_ok(rid, q, home, m):
                return home, [home]
        return self._transit_target(rid, q, home, [])

    def _reapproach(self, i, k, gp, obs) -> bool:
        """Back up the descent path (and re-descend from the pre-weld) when the entry direction is off: umarell's
        entry gate (TCP motion over the last 0.04 m within 15 deg of the approach) would never pass on the weld.
        At the end of the path with a bad entry -> start backing up; while backing up (previous step moved up the
        path, from robot_qd) with a bad entry -> continue up to the pre-weld, then descend normally."""
        ent = np.asarray(obs["robot_entry"][i], float)
        if np.linalg.norm(ent) < 0.5:
            return False
        bad = float(ent @ gp.approach) < np.cos(np.deg2rad(self.entry_tol_deg))
        if not bad:
            return False
        if k >= len(gp.path) - 2:
            return True
        qd = np.asarray(obs["robot_qd"][i], float)
        up = gp.path[k - 1] - gp.path[k]
        return float(qd @ up) > 0.25 * float(np.linalg.norm(qd) * np.linalg.norm(up)) and np.linalg.norm(qd) > 1e-3

    def _after_retract(self, rid, i, q_end, goal, obs, evade):
        if goal is None or evade:
            _, vias = self._home_target(rid, q_end)
            return self._sweep_lines(q_end, vias)[1:]
        pos, R, app, _ = goal
        gp = self.goal_plan(rid, pos, R, app, q_end)
        if not gp.ok:
            return self._line(q_end, self.home[rid])
        _, vias = self._transit_target(rid, q_end, gp.q_pre, gp.vias)
        return np.vstack([self._sweep_lines(q_end, vias)[1:], gp.path[1:]])

    def _sweep_lines(self, q, vias):
        pts = [q[None]]
        a = q
        for v in vias:
            pts.append(self._line(a, v))
            a = v
        return np.vstack(pts)

    def _action(self, rid, q, tgt, mode):
        vs = self.vmax[rid] * self.dt
        d = (np.asarray(tgt, float) - q) / vs
        n = float(np.abs(d).max())
        if n <= 1.0 or mode in ("descent", "retract", "hold", "reapproach", "evade", "evade_local", "home", "yield_home"):
            return d / max(n, 1.0)          # evade / home: full speed until arrival (decisive labels)
        return d / max(n, self.slow_steps) if n < self.slow_steps else d / n

    # ------------------------------------------------------------------ coordination
    def _pts(self, rid, Q):
        S = self.sm.spheres(rid, np.asarray(Q, float).reshape(-1, 6))[:, self._uniq[rid]]
        return S.reshape(-1, 3)

    def clearance(self, rid_a, Qa, rid_b, Qb, cap: float = 1.0) -> float:
        """Min umarell sphere clearance between two sets of configurations (all pairs, not time-aligned).
        Values above ``cap`` are reported as ``cap`` (early-exit KD-tree query); callers compare with thr < cap."""
        A, B = self._pts(rid_a, Qa), self._pts(rid_b, Qb)
        if len(A) > len(B):
            A, B = B, A
        r2 = 2 * self.sm.pair_radius
        d, _ = cKDTree(B).query(A, k=1, distance_upper_bound=cap + r2)
        m = float(d.min())
        return cap if not np.isfinite(m) else min(cap, m - r2)

    def priority_arm(self, obs) -> int:
        if self.priority == "remaining":
            rem = [self._remaining(obs, i) for i in range(2)]
            if rem[0] != rem[1]:
                return int(np.argmax(rem))
        return ROBOT_IDS.index(self.tie_robot)

    def next_legs(self, rid, i, goal, obs, k=2, q=None):
        """Sweeps of arm i's next k legs after the current goal (queue order from task_rank)."""
        out, g, rank, qref = [], goal, 1, q
        for _ in range(k):
            if g is None:
                break
            gp = self.goal_plan(rid, *g[:3], qref)
            qref = gp.q_pre
            if not gp.ok:
                break
            ng = self._queued_goal(obs, i, rank)
            out.append(self._leg_from(rid, gp, ng))
            g, rank = ng, rank + 1
        return np.vstack(out) if out else np.zeros((0, 6))

    def _queued_goal(self, obs, i, rank):
        r = np.asarray(obs["task_rank"], float)
        own = np.asarray(obs["task_owner"])[:, i] > 0.5
        m = (np.asarray(obs["task_mask"]) > 0.5) & (np.asarray(obs["task_status"]) < 0.5) & own
        ks = np.flatnonzero(m & (np.abs(r - rank / 16) < 1e-4))
        if not len(ks):
            return None
        k = int(ks[0])
        return (np.asarray(obs["task_pos"][k], float), rot6_to_R(np.asarray(obs["task_rot6"][k], float)),
                np.asarray(obs["task_approach"][k], float), k)

    def _leg_from(self, rid, gp, ng):
        """Leg from resting on gp's weld: retract, transit to ng's pre-weld (or HOME if ng is None), descent."""
        key = (rid, id(gp), None if ng is None else tuple(np.round(ng[0], 4)))
        v = self._legs.get(key)
        if v is not None:
            return v
        if len(self._legs) > 2_000:
            self._legs.clear()
        ret = gp.path[::-1]
        if ng is None:
            _, vias = self._home_target(rid, gp.q_pre)
            v = np.vstack([ret, self._sweep_lines(gp.q_pre, vias)[1:]])
        else:
            g2 = self.goal_plan(rid, *ng[:3], gp.q_pre)
            if not g2.ok:
                v = ret
            else:
                _, vias = self._transit_target(rid, gp.q_pre, g2.q_pre, g2.vias)
                v = np.vstack([ret, self._sweep_lines(gp.q_pre, vias)[1:], g2.path[1:]])
        self._legs[key] = v
        return v

    def _clear(self, rid_a, Qa, rid_b, Qb):
        if len(Qa) == 0 or len(Qb) == 0:
            return np.inf
        return self.clearance(rid_a, Qa, rid_b, Qb)

    def act(self, obs, return_info: bool = False):
        q = [np.asarray(obs["robot_q"][k], float) for k in range(2)]
        goals = self._goals(obs)
        g = [goals[r] for r in ROBOT_IDS]
        nom = [self.nominal(r, i, q[i], g[i], obs) for i, r in enumerate(ROBOT_IDS)]
        p = self.priority_arm(obs)
        n = 1 - p
        rp, rn = ROBOT_IDS[p], ROBOT_IDS[n]
        thr = self.thr
        modes = [None, None]
        act = np.zeros((2, 6))
        S_p = nom[p][2]
        c_p = self._clear(rp, S_p, rn, q[n][None])

        def go(k, nm):
            act[k] = self._action(ROBOT_IDS[k], q[k], nm[1], nm[0])
            modes[k] = nm[0]

        def evade_n(against, local_k=8):
            ev = self.nominal(rn, n, q[n], g[n], obs, evade=True)
            if ev[0] == "park":
                return None
            if self._clear(rn, ev[2], rp, against) > thr:
                return ev, "evade"
            if self._clear(rn, ev[2][:local_k], rp, against) > thr:
                return ev, "evade_local"
            return None

        if c_p > thr:
            go(p, nom[p])
            S_pf = np.vstack([S_p, self.next_legs(rp, p, g[p], obs, self.lookahead_legs, q[p])])
            if nom[n][0] == "park":
                modes[n] = "park"; coord = "go"
            elif self._clear(rn, nom[n][2], rp, S_pf) > thr:
                go(n, nom[n]); coord = "go"
            elif self.yield_mode == "home" and nom[n][0] not in ("descent", "hold", "reapproach"):
                # yield = retreat HOME (no "hold in place" labels away from HOME: those make stable fixed points
                # for a cloned policy at the boundary of the other arm's sweep); hold only if the way home is blocked
                e = evade_n(S_p)
                if e is not None:
                    go(n, e[0]); modes[n] = "yield_home"; coord = "n_yield_home"
                else:
                    modes[n] = "yield"; coord = "n_yield"
            elif self._clear(rn, q[n][None], rp, S_pf) > thr:
                modes[n] = "yield"; coord = "n_yield"
            else:
                e = evade_n(S_p)
                if e is not None:
                    go(n, e[0]); modes[n] = e[1]; coord = "n_clear_" + e[1]
                else:
                    modes[n] = "yield_in_path"; coord = "n_yield_in_path"
        else:
            modes[p] = "blocked"
            if nom[n][0] in ("descent", "hold", "retract", "reapproach") and self._clear(rn, nom[n][2], rp, q[p][None]) > thr:
                go(n, nom[n]); coord = "p_blocked_n_finish"
            else:
                e = evade_n(q[p][None])
                if e is not None:
                    go(n, e[0]); modes[n] = e[1]; coord = "p_blocked_n_" + e[1]
                else:
                    modes[n] = "stuck"; coord = "deadlock"
                    self.stats["deadlock"] += 1
        for m in modes:
            self.stats["modes"][m] = self.stats["modes"].get(m, 0) + 1
        self.stats["coord"][coord] = self.stats["coord"].get(coord, 0) + 1
        out = np.clip(act, -1.0, 1.0).reshape(-1).astype(np.float32)
        self.last = dict(modes=modes, prio=p, coord=coord, c_p=c_p)
        if return_info:
            return out, self.last
        return out
