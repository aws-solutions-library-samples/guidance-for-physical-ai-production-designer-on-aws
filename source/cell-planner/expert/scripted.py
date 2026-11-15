"""Scripted two-arm weld expert (Track X).

``ScriptedExpert(env).act(obs) -> action (12,)`` in the env's normalised joint-velocity action
space (``q += clip(a,-1,1) * v_max * velocity_scale * dt``). It reads only the observation
(``robot_q``, ``robot_goal`` + mask, ``task_pos/approach/mask``), so it follows whatever weld queue
the env assigns.

Per robot, when its goal changes, a plan of dense joint waypoints (one per control step) is built:
  1. retract: straight Cartesian lift-off from the weld just finished to its standoff
     (mirror of umarell ``_cartesian_retreat``),
  2. transit: synchronised trapezoidal joint move to the pre-weld pose (0.10 m back along
     ``-approach_vector``, same orientation -- umarell's ``approach_x = x - av * offset``; a shorter
     standoff >= 0.045 m when the branch cannot reach 0.10 m or the fixture blocks it). Vias:
     direct, raised mid configuration, HOME, else RRT-Connect + shortcut. Every waypoint and
     midpoint is checked for hull self-collision (coal, >= 5 mm), floor (hull vertices z >= 0 and
     umarell's sphere rule) and, with a fixture, umarell's box rule (``SphereModel.obstacle_free``),
  3. descent: straight Cartesian line pre-weld -> weld at 0.4 x speed (umarell
     ``_cartesian_descent``), fixed orientation; the plan ends ON the weld and the arm rests there
     (dwell) until the env marks the weld done. No goal -> HOME.
IK: ``expert.ik.WeldIK`` (6D DLS), seeded from ``data/weld_ik.json`` (nearest canonical weld) and
the current configuration, falling back to a wide seed set, so perturbed welds are handled.

Arm-arm coordination (umarell metric, ``env.kinematics.SphereModel.pair_clearance``): reservation
style. A plan is *committed* only if, time-aligned against the other arm's committed future (its
remaining waypoints, then resting at the last one), the predicted clearance stays above
``0.20 + clearance_margin`` (0.25 m) at every step and midpoint; otherwise the arm waits. Since the
env is kinematic and follows the waypoints exactly, waiting is always safe. After
``2 * retreat_after`` blocked steps the margin drops to ``clearance_margin_min`` (still > 0.20 m).
Deadlock (both at rest, both blocked): one arm retreats HOME (retract + transit) and re-plans from
there. Priority alternates between the arms.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from env.cell import ROBOT_IDS, load_cell
from expert.ik import APPROACH_OFFSET_M, WeldIK, cluster

ROOT = Path(__file__).resolve().parents[1]
WARN = 0.20


def rot6_to_R(r6):
    a, b = np.asarray(r6[:3], float), np.asarray(r6[3:6], float)
    x = a / (np.linalg.norm(a) + 1e-12)
    b = b - x * (x @ b)
    y = b / (np.linalg.norm(b) + 1e-12)
    return np.stack([x, y, np.cross(x, y)], axis=1)


def trapezoid_samples(q0, q1, vmax, amax, dt):
    """Synchronised trapezoidal profile q0 -> q1, sampled every dt (excluding q0, including q1).

    All joints follow the slowest joint's normalised profile; the duration is then stretched
    until every joint's per-step change is <= vmax_j * dt (so the env never clips the action)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = q1 - q0
    ad = np.abs(d)
    if ad.max() < 1e-9:
        return np.zeros((0, 6))
    t_acc = vmax / amax
    tri = ad < vmax * t_acc
    T = np.where(tri, 2 * np.sqrt(ad / amax), ad / vmax + t_acc)
    k = int(np.argmax(T))
    L, v, a = ad[k], vmax[k], amax[k]
    Tm = float(T[k])

    def profile(Tm):
        # time-scaled version of joint k's profile (duration Tm >= T[k])
        sc = T[k] / Tm
        vv, aa = v * sc, a * sc * sc
        n = max(1, int(np.ceil(Tm / dt - 1e-9)))
        ts = np.arange(1, n + 1) * (Tm / n)
        if tri[k]:
            s = np.where(ts <= Tm / 2, 0.5 * aa * ts ** 2, L - 0.5 * aa * (Tm - ts) ** 2)
        else:
            ta = vv / aa
            s = np.where(ts <= ta, 0.5 * aa * ts ** 2,
                         np.where(ts <= Tm - ta, 0.5 * aa * ta ** 2 + vv * (ts - ta), L - 0.5 * aa * (Tm - ts) ** 2))
        s = np.clip(s / L, 0, 1)
        s[-1] = 1.0
        return q0 + s[:, None] * d

    for _ in range(20):
        out = profile(Tm)
        steps = np.abs(np.diff(np.vstack([q0, out]), axis=0)).max(0)
        ratio = float((steps / (vmax * dt)).max())
        if ratio <= 1.0 + 1e-9:
            return out
        Tm *= max(ratio, 1.01)
    return out


def resample_path(qs, step_max, dt=None):
    """Dense constant-speed resampling of a joint path so that every per-step joint change is
    <= step_max (vector, rad/step). Returns samples excluding qs[0]."""
    qs = np.asarray(qs, float)
    out = []
    for a, b in zip(qs[:-1], qs[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b - a) / step_max) - 1e-9)))
        for i in range(1, n + 1):
            out.append(a + (b - a) * i / n)
    return np.array(out).reshape(-1, 6)


@dataclass
class Plan:
    wps: np.ndarray                      # (N,6) waypoints, one per control step
    phase: list                          # per waypoint: "retract"|"transit"|"descent"|"dwell"|"home"
    goal_pos: np.ndarray | None = None
    weld_q: np.ndarray | None = None
    weld_idx: int | None = None          # task index in obs
    retract: np.ndarray | None = None    # path weld -> pre-weld for the next plan
    i: int = 0
    info: dict = field(default_factory=dict)

    @property
    def remaining(self):
        return len(self.wps) - self.i


class ScriptedExpert:
    def __init__(self, env=None, cell=None, table_path=None, dt=0.05, vel_scale=0.7,
                 transit_frac=1.0, approach_frac=0.4, acc_frac=0.7, horizon=10,
                 clearance_margin=0.05, clearance_margin_min=0.015, retreat_after=20, check_res=0.04, fixture=None, verbose=False):
        if cell is None:
            cell = getattr(getattr(env, "cell", None), "cell_id", None) or getattr(env, "cell_id", None) \
                or "welding_cell_2_small"
        self.env = env
        self.cell = load_cell(cell) if isinstance(cell, str) else cell
        self.dt = float(getattr(env, "dt", dt))
        self.vel_scale = float(getattr(env, "vscale", vel_scale))
        self.ik = WeldIK(self.cell)
        from env.kinematics import SphereModel
        self.sm = SphereModel(self.cell)
        self.fixture = self.cell.has_fixture if fixture is None else fixture
        # weld pose rule: the env/umarell gate the tool axis only -> spin free, tilt <= 4.5 deg. If the
        # env also gates the full rotation (RewardConfig.rot_tol_deg), keep spin + tilt inside it.
        rot_tol = getattr(getattr(getattr(env, "cfg", None), "reward", None), "rot_tol_deg", None)
        if rot_tol is not None:
            self.ik.strict_rot = True
            self.max_spin, self.tilts = max(0.0, float(rot_tol) - 1.0), ()
        else:
            self.max_spin, self.tilts = None, (2.0, 3.0, 4.0, 4.5)
        self.vmax = {r: self.cell.robots[r].v_max * self.vel_scale for r in ROBOT_IDS}
        self.amax = {r: self.cell.robots[r].a_max * acc_frac for r in ROBOT_IDS}
        self.transit_frac, self.approach_frac = transit_frac, approach_frac
        self.horizon, self.margin, self.margin_min, self.check_res = horizon, clearance_margin, clearance_margin_min, check_res
        self.retreat_after = retreat_after
        self.verbose = verbose
        tp = Path(table_path) if table_path else ROOT / "data" / "weld_ik.json"
        self.table = json.load(open(tp)) if tp.exists() else None
        if self.table is not None:
            from robotmodel.stamp import check_table     # warns if solved for another robot model (P3U round 2)
            check_table(tp, self.cell.cell_id, "IK table")
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        self.active: dict[str, Plan | None] = {r: None for r in ROBOT_IDS}
        self.pending: dict[str, Plan | None] = {r: None for r in ROBOT_IDS}
        self.goal_key = {r: "init" for r in ROBOT_IDS}
        self.last_weld = {r: None for r in ROBOT_IDS}
        self.retreating = {r: False for r in ROBOT_IDS}
        self.blocked = {r: 0 for r in ROBOT_IDS}
        self.prio = "robot2"
        self.legs = []          # committed plans: {robot, step (control step of commit), q0, plan}
        self.n_steps = 0
        self.stats = {"waits": {r: 0 for r in ROBOT_IDS}, "backoffs": {r: 0 for r in ROBOT_IDS},
                      "replans_home": {r: 0 for r in ROBOT_IDS}, "plan_fail": []}

    # ------------------------------------------------------------------ geometry checks
    def config_ok(self, rid, q):
        c = self.ik.check(rid, q)
        if not c.valid:
            return False
        if self.fixture:
            return bool(self.sm.obstacle_free(rid, q))
        return True

    def segment_ok(self, rid, q0, q1):
        n = max(1, int(np.ceil(np.abs(np.asarray(q1) - q0).max() / self.check_res)))
        return all(self.config_ok(rid, q0 + (q1 - q0) * i / n) for i in range(1, n + 1))

    def transit(self, rid, q0, q1):
        """Collision-checked (self/floor[/fixture]) joint-space path q0 -> q1 as a list of vias:
        direct, via a raised mid configuration, via HOME, else a small RRT-Connect + shortcut."""
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        home = self.cell.robots[rid].home
        mid = 0.5 * (q0 + q1)
        raised = mid.copy(); raised[1:3] = home[1:3]
        cands = [[q0, q1], [q0, raised, q1], [q0, home, q1]]
        for c in cands:
            if all(self.segment_ok(rid, a, b) for a, b in zip(c[:-1], c[1:])) \
                    and self.dense_ok(rid, q0, self.dense_transit(rid, c, self.transit_frac)):
                return c
        c = self.rrt_connect(rid, q0, q1)
        if c is not None and self.dense_ok(rid, q0, self.dense_transit(rid, c, self.transit_frac)):
            return c
        return None

    def dense_ok(self, rid, q0, wps):
        """Every control-step waypoint and every midpoint (the env sub-samples each step) valid."""
        if len(wps) == 0:
            return True
        a = np.vstack([np.asarray(q0, float)[None], wps])
        pts = np.concatenate([a[1:], 0.5 * (a[1:] + a[:-1])])
        return all(self.config_ok(rid, x) for x in pts)

    def rrt_connect(self, rid, q0, q1, iters=1500, step=0.3, seed=0):
        """Bidirectional RRT in joint space with ``segment_ok`` edges; shortcut-smoothed."""
        if not (self.config_ok(rid, q0) and self.config_ok(rid, q1)):
            return None
        r = self.cell.robots[rid]
        rng = np.random.default_rng(seed)
        lo, hi = r.q_lo.copy(), r.q_hi.copy()
        # sample a box around the two endpoints (+ HOME) rather than the full +-350 deg wrist range
        pts = np.stack([q0, q1, r.home])
        lo = np.maximum(lo, pts.min(0) - 1.0); hi = np.minimum(hi, pts.max(0) + 1.0)
        A, B = [q0], [q1]
        PA, PB = [-1], [-1]

        def extend(T, P, x):
            d = np.linalg.norm(np.array(T) - x, axis=1)
            j = int(np.argmin(d))
            v = x - T[j]
            n = np.linalg.norm(v)
            y = x if n <= step else T[j] + v / n * step
            if self.segment_ok(rid, T[j], y):
                T.append(y); P.append(j)
                return len(T) - 1, np.linalg.norm(y - x) < 1e-9
            return None, False

        def path(T, P, k):
            out = []
            while k != -1:
                out.append(T[k]); k = P[k]
            return out[::-1]

        for it in range(iters):
            x = rng.uniform(lo, hi) if rng.uniform() > 0.1 else (B[0] if len(A) <= len(B) else A[0])
            ka, _ = extend(A, PA, x)
            if ka is None:
                A, B, PA, PB = B, A, PB, PA
                continue
            # connect B towards the new node greedily
            while True:
                kb, reached = extend(B, PB, A[ka])
                if kb is None:
                    break
                if reached:
                    pa, pb = path(A, PA, ka), path(B, PB, kb)
                    full = pa + pb[::-1][1:]
                    if not np.allclose(full[0], q0):
                        full = full[::-1]
                    return self._shortcut(rid, full)
            A, B, PA, PB = B, A, PB, PA
        return None

    def _shortcut(self, rid, vias, rounds=40, seed=1):
        rng = np.random.default_rng(seed)
        v = [np.asarray(x, float) for x in vias]
        for _ in range(rounds):
            if len(v) <= 2:
                break
            i, j = sorted(rng.choice(len(v), 2, replace=False))
            if j - i >= 2 and self.segment_ok(rid, v[i], v[j]):
                v = v[:i + 1] + v[j:]
        return v

    def dense_transit(self, rid, vias, frac=1.0):
        out = [trapezoid_samples(a, b, self.vmax[rid] * frac, self.amax[rid], self.dt) for a, b in zip(vias[:-1], vias[1:])]
        return np.concatenate(out) if out else np.zeros((0, 6))

    # ------------------------------------------------------------------ IK for a goal
    def _hints(self, rid, pos):
        """(q, spin, tilt, tilt_dir) of the usable table solutions of the nearest canonical weld."""
        if not self.table:
            return []
        best, bd = None, 0.1
        for wid, w in self.table["welds"].items():
            d = np.linalg.norm(np.array(w["pos"]) - pos)
            if d < bd:
                best, bd = w, d
        if best is None:
            return []
        return [(np.array(s["q"]), s.get("spin_deg", 0.0), s.get("tilt_deg", 0.0), s.get("tilt_dir_deg", 0.0))
                for s in best["robots"][rid]["solutions"] if s["usable"]]

    def weld_options(self, rid, pos, R, approach, q_cur, wide=False):
        """Usable (cost, weld q, descent path pre->weld) options, sorted by cost = weighted joint
        travel from q_cur (+ penalties for a thin hull margin / shortened standoff).

        ``WeldIK.solve_axis`` (spin about the tool axis free, small tilt if needed, descent
        checked): first warm-started from the table solutions of the nearest canonical weld with a
        +-15 deg spin window; if that yields nothing, a full solve (all branches, full spin sweep)."""
        extra = (lambda q: bool(self.sm.obstacle_free(rid, q))) if self.fixture else None
        kw = dict(approach=approach, standoff=APPROACH_OFFSET_M, min_standoff=0.045, n_steps=20,
                  extra_check=extra, max_spin=self.max_spin, tilts=self.tilts)
        hints = None if wide else self._hints(rid, pos)
        if not wide and not hints:
            return self.weld_options(rid, pos, R, approach, q_cur, wide=True)
        if wide:
            seeds = [np.asarray(q_cur, float)] + self.ik.default_seeds(rid, n_random=150, target_world=pos)
            sols = self.ik.solve_axis(rid, p=pos, R=R, seeds=seeds, **kw)
        else:
            sols = self.ik.solve_axis(rid, p=pos, R=R, hints=hints, spin_window=15.0, **kw)
        w = np.array([1.0, 1.0, 1.0, 0.6, 0.6, 0.4])
        opts = []
        for c in sols:
            path = c.path
            if path is None or len(path) < 10:
                continue
            cost = float(np.sum(w * np.abs(path[-1] - q_cur))) + (0.5 if len(path) < 21 else 0.0) \
                + (1.0 if c.thin else 0.0)
            opts.append((cost, c.q, path[::-1]))
        opts.sort(key=lambda t: t[0])
        if not opts and not wide:
            return self.weld_options(rid, pos, R, approach, q_cur, wide=True)
        return opts

    def make_plan(self, rid, q_cur, goal_pos=None, R=None, approach=None, weld_idx=None, via_home=False):
        """Plan from q_cur: [retract if resting on the last weld] + transit + descent (ends ON the
        weld; the dwell is the rest that follows). goal_pos None = go HOME."""
        pre, phases = [], []
        q0 = np.asarray(q_cur, float)
        lw = self.last_weld[rid]
        if lw is not None and np.abs(q0 - lw[0]).max() < 1e-3:
            step = self.vmax[rid] * self.approach_frac * self.dt
            r = resample_path(np.vstack([q0, lw[1][1:]]), step)
            pre.append(r); phases += ["retract"] * len(r)
            q0 = r[-1] if len(r) else q0
        home = self.cell.robots[rid].home
        if goal_pos is None:
            vias = self.transit(rid, q0, home) or [q0]   # no collision-free way home: stay
            t = self.dense_transit(rid, vias, self.transit_frac)
            pre.append(t); phases += ["home"] * len(t)
            wps = np.concatenate(pre) if pre else np.zeros((0, 6))
            return Plan(wps, phases, info={"home": True})
        opts = self.weld_options(rid, goal_pos, R, approach, q0)
        for cost, qw, desc in opts:
            if via_home:
                v2, v3 = self.transit(rid, q0, home), self.transit(rid, home, desc[0])
                vias = v2 + v3[1:] if (v2 is not None and v3 is not None) else None
            else:
                vias = self.transit(rid, q0, desc[0])
            if vias is None:
                continue
            t = self.dense_transit(rid, vias, self.transit_frac)
            if not self.dense_ok(rid, q0, t):
                continue
            step = self.vmax[rid] * self.approach_frac * self.dt
            d = resample_path(desc, step)
            if not self.dense_ok(rid, desc[0], d):     # descent incl. midpoints (retract = reverse)
                continue
            wps = np.concatenate(pre + [t, d])
            phases = phases + ["transit"] * len(t) + ["descent"] * len(d)
            return Plan(wps, phases, goal_pos=np.asarray(goal_pos), weld_q=qw, weld_idx=weld_idx,
                        retract=desc[::-1], info={"cost": cost, "n_vias": len(vias),
                                                   "standoff_m": float(np.linalg.norm(
                                                       self.ik.tcp_world(rid, desc[0])[0] - goal_pos))})
        self.stats["plan_fail"].append((rid, np.round(goal_pos, 3).tolist()))
        # unreachable weld: park at HOME (out of the other arm's way); the env queue is stuck
        q1 = q0
        vias = self.transit(rid, q1, home) or [q1]
        t = self.dense_transit(rid, vias, self.transit_frac)
        pre.append(t); phases += ["home"] * len(t)
        return Plan(np.concatenate(pre), phases, goal_pos=np.asarray(goal_pos),
                    weld_idx=weld_idx, info={"failed": True})

    # ------------------------------------------------------------------ observation parsing
    def _goals(self, obs):
        out = {}
        for k, rid in enumerate(ROBOT_IDS):
            if float(obs["robot_goal_mask"][k]) < 0.5:
                out[rid] = None
                continue
            g = np.asarray(obs["robot_goal"][k], float)
            pos, R = g[:3], rot6_to_R(g[3:9])
            tp = np.asarray(obs["task_pos"], float)
            mask = np.asarray(obs["task_mask"], float) > 0.5
            d = np.where(mask, np.linalg.norm(tp - pos, axis=1), np.inf)
            j = int(np.argmin(d))
            ok = bool(np.isfinite(d[j]) and d[j] < 1e-3)
            approach = np.asarray(obs["task_approach"][j], float) if ok else np.array([0.0, 0.0, -1.0])
            out[rid] = (pos, R, approach, j if ok else None)
        return out

    # ------------------------------------------------------------------ coordination
    def _future(self, rid, q):
        """Committed future of an arm: remaining active waypoints, then rest at the last one."""
        p = self.active[rid]
        if p is None or p.i >= len(p.wps):
            return q[None]
        return p.wps[p.i:]

    def _compatible(self, f1, f2):
        """Min predicted clearance between robot1's future f1 (n1,6) and robot2's f2 (n2,6)
        (time-aligned, the shorter one rests at its last configuration)."""
        n = max(len(f1), len(f2))
        a = np.vstack([f1, np.repeat(f1[-1:], n - len(f1), 0)]) if len(f1) < n else f1
        b = np.vstack([f2, np.repeat(f2[-1:], n - len(f2), 0)]) if len(f2) < n else f2
        # also check the midpoints (the env sub-samples each control step)
        c = self.sm.pair_clearance(a, b)
        m = self.sm.pair_clearance(0.5 * (a[1:] + a[:-1]), 0.5 * (b[1:] + b[:-1])) if n > 1 else c
        return float(min(c.min(), m.min()))

    def idle(self):
        return all(not self._moving(r) and self.pending[r] is None for r in ROBOT_IDS)

    def _moving(self, rid):
        p = self.active[rid]
        return p is not None and p.i < len(p.wps)

    def _try_commit(self, rid, plan, q, other, margin=None):
        if plan is None or len(plan.wps) == 0:
            return True
        # the candidate starts from the current configuration of this arm
        cand = np.vstack([q[rid][None], plan.wps])
        ofut = np.vstack([q[other][None], self._future(other, q[other])])
        c = self._compatible(cand, ofut) if rid == ROBOT_IDS[0] else self._compatible(ofut, cand)
        if c > WARN + (self.margin if margin is None else margin):
            self.active[rid] = plan
            plan.i = 0
            self.legs.append({"robot": rid, "step": self.n_steps, "q0": q[rid].copy(), "plan": plan})
            self.stats["commit_clearance"] = min(self.stats.get("commit_clearance", np.inf), c)
            return True
        return False

    # ------------------------------------------------------------------ act
    def act(self, obs):
        q = {rid: np.asarray(obs["robot_q"][k], float) for k, rid in enumerate(ROBOT_IDS)}
        goals = self._goals(obs)
        # 1. new goals -> pending plans (computed when the arm is not moving)
        for rid in ROBOT_IDS:
            g = goals[rid]
            key = None if g is None else tuple(np.round(g[0], 5))
            if key != self.goal_key[rid] and not self._moving(rid):
                if self.active[rid] is not None and self.active[rid].weld_q is not None \
                        and np.abs(q[rid] - self.active[rid].weld_q).max() < 1e-3:
                    self.last_weld[rid] = (self.active[rid].weld_q, self.active[rid].retract)
                self.goal_key[rid] = key
                self.pending[rid] = self.make_plan(rid, q[rid], *(g if g is not None else (None,) * 4))
                self.active[rid] = None
                self.blocked[rid] = 0
        # 1b. back at HOME after a retreat: re-plan towards the (unchanged) goal
        for rid in ROBOT_IDS:
            g = goals[rid]
            if g is not None and self.pending[rid] is None and not self._moving(rid) \
                    and self.active[rid] is not None and self.active[rid].info.get("home"):
                self.pending[rid] = self.make_plan(rid, q[rid], *g)
                self.active[rid] = None
                self.blocked[rid] = 0
        # 2. commit pending plans (priority arm first)
        a, b = ROBOT_IDS
        ra = self.pending[a] is not None
        rb = self.pending[b] is not None
        order = (a, b) if self.prio == a else (b, a)
        for rid in order:
            other = b if rid == a else a
            if self.pending[rid] is not None and not self._moving(rid):
                # after a long wait accept the small margin (still > 0.20 m at every predicted
                # step and midpoint; the kinematic env follows the waypoints exactly)
                m = self.margin_min if self.blocked[rid] >= 2 * self.retreat_after else None
                if self._try_commit(rid, self.pending[rid], q, other, margin=m):
                    self.pending[rid] = None
                    self.blocked[rid] = 0
                    self.prio = other   # alternate priority once served
                else:
                    self.blocked[rid] += 1
                    self.stats["waits"][rid] += 1
        # 3. deadlock: both at rest with blocked pending plans -> one retreats via HOME
        for rid in order[::-1]:
            other = b if rid == a else a
            if self.pending[rid] is not None and self.blocked[rid] >= self.retreat_after \
                    and self.blocked[rid] % 10 == 0 \
                    and not self._moving(rid) and not self._moving(other) \
                    and (self.pending[other] is not None or goals[other] is None) \
                    and not self.retreating[rid]:
                g = goals[rid]
                if self.active[rid] is not None and self.active[rid].weld_q is not None \
                        and np.abs(q[rid] - self.active[rid].weld_q).max() < 1e-3:
                    self.last_weld[rid] = (self.active[rid].weld_q, self.active[rid].retract)
                ret = self.make_plan(rid, q[rid])          # retract + HOME; re-planned from HOME later
                if not ret.info.get("failed") and self._try_commit(rid, ret, q, other, margin=self.margin_min):
                    self.pending[rid] = None
                    self.retreating[rid] = True
                    self.blocked[rid] = 0
                    self.stats["replans_home"][rid] += 1
                    break
        # 4. execute
        act = np.zeros((2, 6))
        for k, rid in enumerate(ROBOT_IDS):
            p = self.active[rid]
            target = q[rid]
            if p is not None and p.i < len(p.wps):
                target = p.wps[p.i]
                p.i += 1
                if p.i >= len(p.wps):
                    self.retreating[rid] = False
            act[k] = (target - q[rid]) / (self.vmax[rid] * self.dt)
        self.n_steps += 1
        return np.clip(act, -1.0, 1.0).reshape(-1).astype(np.float32)
