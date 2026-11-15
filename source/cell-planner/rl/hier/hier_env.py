"""Semi-MDP wrapper: decision points of a two-arm weld scheduler on top of ``KukaWeldEnv`` (Track H).

A decision is taken for one arm whenever that arm is idle (episode start, primitive finished). If both
arms are idle at the same instant, robot1 decides first, then robot2 (zero elapsed time in between).
Between decisions, the executor's primitives are played through ``KukaWeldEnv.step`` at 20 Hz, so
every configuration is scored by the env's own collision checks (umarell sphere gate + MuJoCo hull
self / arm-arm / floor contacts) and welds complete by the env's own pose + dwell rule.

Action of the deciding arm (``N_ACT`` = T_MAX * K + 4):
  a = t * K + b   weld task slot t with IK branch b (instances.Branch), executed as executor.weld
  T_MAX*K + j     j = 0, 1, 2: wait 0.5 / 1 / 2 s;  j = 3: go HOME
Masks only express feasibility of the *task* (weld open and not claimed by the other arm, branch
exists for this arm, HOME action only when not at HOME); nothing about collisions is masked.

The env's weld queue is driven directly: choosing weld t makes t the arm's goal (the env only
completes a robot's current goal); after completion the arm has no goal until its next decision.

Reward per elapsed control step: ``-time_cost * dt`` while welds remain, ``+weld_bonus`` per completed
weld, terminal ``+success_bonus`` when all welds are done, terminal ``collision_penalty`` on any
collision, terminal ``timeout_penalty`` after ``max_time_s``. Rewards inside a transition are
discounted with ``gamma ** (elapsed seconds)``; the transition returns its duration ``tau`` (s).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from env.cell import ROBOT_IDS
from env.weld_env import T_MAX, EnvConfig, KukaWeldEnv

from .executor import SPECIALS, Executor, Primitive
from .instances import K_BRANCH, Branch

N_SPECIAL = len(SPECIALS)
N_ACT = T_MAX * K_BRANCH + N_SPECIAL
A_HOME = T_MAX * K_BRANCH + 3


@dataclass
class HierConfig:
    cell_id: str = "welding_cell_2_small"
    gamma: float = 0.99                 # per second
    time_cost: float = 1.0              # per second
    weld_bonus: float = 2.0
    success_bonus: float = 10.0
    collision_penalty: float = -60.0
    timeout_penalty: float = -100.0
    max_time_s: float | None = None     # None: time_base + time_per_weld * n_welds
    time_base: float = 30.0
    time_per_weld: float = 6.0
    speed_profile: str = "ours"         # executor.PROFILES: "ours" | "umarell_rule" | "umarell_observed"
    max_tilt_deg: float | None = None   # branch-quality constraint (instances.branch_ok), None = off
    min_self_dist: float | None = None  # m, hull self distance at the weld and along the descent
    exclude_offset0: bool = False       # never sample pool offset 0 (= the canonical position) in random programs
    time_feature: str = "left"          # "left" (time to the limit) | "elapsed" (runs hier_e/f, v2 features)
    no_mutual_wait: bool = False        # True: an arm may not wait while the other arm waits (hurt: see docs)
    # randomisation (training / held-out)
    n_welds: tuple = (2, 15)
    p_full: float = 0.4                 # prob. of all 15 welds (else uniform subset size)
    p_canonical: float = 0.1            # exact canonical task (HOME start, offset 0)
    start_noise: float = 0.1
    pos_noise: float = 0.03
    record: bool = False


def decode(a: int):
    if a < T_MAX * K_BRANCH:
        return "weld", a // K_BRANCH, a % K_BRANCH
    return "special", a - T_MAX * K_BRANCH, -1


class HierWeldEnv:
    def __init__(self, cfg: HierConfig | None = None, pool=None, solver=None):
        self.cfg = cfg or HierConfig()
        self.env = KukaWeldEnv(EnvConfig(cell_id=self.cfg.cell_id, episode_len=10 ** 7, assignment="umarell",
                                         shuffle_prob=0.0))
        self.cell = self.env.cell
        self.dt = self.env.dt
        self.ex = Executor(self.cell, dt=self.dt, dwell_steps=self.env.cfg.reward.dwell_steps,
                           profile=self.cfg.speed_profile)
        self.pool = pool
        self.solver = solver
        self.wid_index = {w.wid: i for i, w in enumerate(self.cell.welds)}
        self.home = self.env.home.copy()
        self.rng = np.random.default_rng(0)
        self.feat = None                 # features.FeatureBuilder, set lazily
        self.infeasible = []

    # ------------------------------------------------------------------ episode setup
    def sample_program(self, rng, canonical=False, n_welds=None, fresh=False):
        """-> wids, positions (n,3), branches [{rid: [Branch]}], q0 (2,6), offsets id list."""
        cfg = self.cfg
        all_w = [w.wid for w in self.cell.welds]
        if canonical:
            wids = all_w
        else:
            if n_welds is None:
                n = len(all_w) if rng.uniform() < cfg.p_full else int(rng.integers(cfg.n_welds[0], cfg.n_welds[1] + 1))
            else:
                n = int(n_welds)
            wids = sorted(rng.choice(all_w, n, replace=False).tolist()) if n < len(all_w) else all_w
        pos, br, ids = [], [], []
        for wid in wids:
            if canonical and self.pool is not None and not fresh:
                p, b = self.pool[wid][0]
                ids.append((wid, 0))
            elif fresh or self.pool is None:
                base = self.cell.welds[self.wid_index[wid]].pos
                p = np.asarray(base, float) + (0 if canonical else rng.uniform(-cfg.pos_noise, cfg.pos_noise, 3))
                b = self.solver.solve_weld(wid, p)
                ids.append((wid, None))
            else:
                m = int(rng.integers(1 if cfg.exclude_offset0 else 0, len(self.pool[wid])))
                p, b = self.pool[wid][m]
                ids.append((wid, m))
            pos.append(np.asarray(p, float)); br.append(b)
        if canonical:
            q0 = self.home.copy()
        else:
            self.env.np_random = rng
            q0 = self.env._sample_start(cfg.start_noise)
        return dict(wids=wids, pos=np.array(pos), branches=br, q0=q0, ids=ids)

    def reset(self, seed=None, options=None):
        options = dict(options or {})
        rng = np.random.default_rng(seed)
        self.rng = rng
        if "program" in options:
            prog = options["program"]
        else:
            canonical = options.get("canonical", False) or (not options.get("no_canonical", False)
                                                           and rng.uniform() < self.cfg.p_canonical)
            prog = self.sample_program(rng, canonical=canonical, n_welds=options.get("n_welds"),
                                       fresh=options.get("fresh", False))
        # branch-quality constraint (tilt / hull self distance), then: welds no arm can reach are dropped + reported
        if self.cfg.max_tilt_deg is not None or self.cfg.min_self_dist is not None:
            from .instances import branch_ok
            prog = dict(prog)
            prog["branches"] = [{r: [b for b in br[r] if branch_ok(b, self.cfg.max_tilt_deg, self.cfg.min_self_dist)]
                                 for r in br} for br in prog["branches"]]
        keep = [i for i, b in enumerate(prog["branches"]) if any(len(b[r]) for r in ROBOT_IDS)]
        self.infeasible = [prog["wids"][i] for i in range(len(prog["wids"])) if i not in keep]
        self.prog = prog
        self.wids = [prog["wids"][i] for i in keep]
        self.task_pos = prog["pos"][keep]
        self.branches = [prog["branches"][i] for i in keep]
        e = self.env
        e.reset(options={"canonical": True, "welds": self.wids,
                         "assignment": {"robot1": list(self.wids), "robot2": []}, "q0": prog["q0"]})
        n = len(self.wids)
        e._task_pos[:n] = self.task_pos            # noisy positions (rotation unchanged: rot_noise 0)
        e.queues = [[], []]
        e.queue_pos = [0, 0]
        e._task_owner[:] = 0
        self.n = n
        self.max_time = self.cfg.max_time_s if self.cfg.max_time_s is not None \
            else self.cfg.time_base + self.cfg.time_per_weld * n
        self.status = np.zeros(T_MAX, np.int64)    # 0 open, 1 done
        self.claim = np.full(T_MAX, -1)            # arm index working on the task
        self.done_by = np.full(T_MAX, -1)
        self.prims: list[Primitive | None] = [None, None]
        self.t = 0
        self.min_clr = float(e._min_clr)
        self.collision = None
        self.finished = n == 0
        self.makespan = 0.0
        self.n_decisions = [0, 0]
        self.history = []          # decisions (t, arm, action)
        self.prim_decision = [-1, -1]   # decision index that started each arm's current primitive
        self.blame = []            # decision indices whose primitives were executing at the collision
        self.weld_log = []         # (robot, wid, t_start_s, t_end_s) like env.weld_events
        self.arrivals = []         # (robot, wid, t_s): the arm reaches the weld configuration
        if self.cfg.record:
            self.rec_t, self.rec_q, self.rec_phase = [0.0], [e.q.copy()], [["idle", "idle"]]
        self.deciding = 0
        self._advance(accumulate=False)
        return self.obs()

    # ------------------------------------------------------------------ helpers
    def q(self, i=None):
        return self.env.q if i is None else self.env.q[i]

    def at_home(self, i):
        return float(np.abs(self.env.q[i] - self.home[i]).max()) < 1e-3

    def branch(self, i, t, b) -> Branch | None:
        if t >= self.n:
            return None
        lst = self.branches[t][ROBOT_IDS[i]]
        return lst[b] if b < len(lst) else None

    def action_mask(self, i=None):
        i = self.deciding if i is None else i
        m = np.zeros(N_ACT, bool)
        for t in range(self.n):
            if self.status[t] == 0 and self.claim[t] < 0:
                nb = len(self.branches[t][ROBOT_IDS[i]])
                m[t * K_BRANCH: t * K_BRANCH + min(nb, K_BRANCH)] = True
        m[A_HOME] = not self.at_home(i)
        # waiting is pointless (the world is static) while the other arm waits too: not offered then,
        # unless there is nothing else to do
        po = self.prims[1 - i]
        other_waits = po is not None and not po.done and po.kind == "wait"
        if not (self.cfg.no_mutual_wait and other_waits and m.any()):
            m[T_MAX * K_BRANCH: T_MAX * K_BRANCH + 3] = True
        return m

    def make_primitive(self, i, a):
        rid = ROBOT_IDS[i]
        kind, x, b = decode(int(a))
        if kind == "weld":
            br = self.branch(i, x, b)
            assert br is not None, (a, x, b)
            return self.ex.weld(rid, self.env.q[i], br, x, b)
        return self.ex.special(rid, self.env.q[i], x)

    def obs(self):
        if self.feat is None:
            from .features import FeatureBuilder
            self.feat = FeatureBuilder(self)
        return self.feat.build()

    # ------------------------------------------------------------------ stepping
    def step(self, a):
        """Apply action ``a`` for the deciding arm, run to the next decision point.
        Returns obs, reward (discounted sum inside the transition), done, info (tau = elapsed s)."""
        i = self.deciding
        m = self.action_mask(i)
        if not m[int(a)]:
            raise ValueError(f"invalid action {a} for arm {i}")
        p = self.make_primitive(i, a)
        p.t_start = self.t
        self.prims[i] = p
        if p.kind == "weld":
            self.claim[p.task] = i
            self.env.queues[i] = self.env.queues[i][:self.env.queue_pos[i]] + [p.task]
            self.env._task_owner[p.task, i] = 1
        self.n_decisions[i] += 1
        self.prim_decision[i] = len(self.history)
        self.history.append((self.t, i, int(a)))
        return self._advance(accumulate=True)

    def _advance(self, accumulate=True):
        cfg, e = self.cfg, self.env
        R, tau, steps = 0.0, 0.0, 0
        n_welds = 0
        done = False
        reason = None
        while True:
            if self.finished or self.collision is not None:
                done = True
                break
            idle = [k for k in range(2) if self.prims[k] is None or self.prims[k].done]
            if idle:
                self.deciding = idle[0]
                break
            if self.t * self.dt >= self.max_time:
                R += (cfg.gamma ** tau) * cfg.timeout_penalty
                done, reason = True, "timeout"
                self.collision = None
                self.timeout = True
                break
            # one control step for both arms
            q = e.q
            target = np.zeros((2, 6))
            n_ev = len(e.weld_events)
            for k in range(2):
                target[k] = self.ex.next_target(self.prims[k], False)
            act = (target - q) / (e.v_max * e.dt)
            _, _, _, _, info = e.step(act.reshape(-1))
            self.t += 1
            steps += 1
            for k in range(2):        # arm reached the weld configuration (end of the descent) in this step
                p = self.prims[k]
                if p.kind == "weld" and p.i == p.hold_at and p.extra_hold == 0:
                    self.arrivals.append((ROBOT_IDS[k], self.wids[p.task], self.t * self.dt))
            self.min_clr = min(self.min_clr, float(info.get("min_clearance_substeps", info["min_clearance"])))
            r = -cfg.time_cost * self.dt
            for ev in e.weld_events[n_ev:]:
                k = ROBOT_IDS.index(ev[0])
                ti = self.wids.index(ev[1])
                self.status[ti] = 1
                self.done_by[ti] = k
                self.claim[ti] = -1
                p = self.prims[k]
                if p is not None and p.kind == "weld" and p.task == ti:
                    p.completed = True
                r += cfg.weld_bonus
                n_welds += 1
                self.weld_log.append((ev[0], ev[1], ev[2] * self.dt, ev[3] * self.dt))
            if self.cfg.record:
                self.rec_t.append(self.t * self.dt); self.rec_q.append(e.q.copy())
                self.rec_phase.append([self._phase_name(k) for k in range(2)])
            if info["collision"]:
                self.collision = dict(kind=info["collision_kind"], t=self.t * self.dt,
                                      contacts=info.get("contacts"))
                self.blame = [self.prim_decision[k] for k in range(2) if self.prim_decision[k] >= 0]
                r += cfg.collision_penalty
            elif int(self.status[:self.n].sum()) == self.n:
                self.finished = True
                self.makespan = self.t * self.dt
                r += cfg.success_bonus
            R += (cfg.gamma ** tau) * r
            tau += self.dt
            # weld primitives whose hold expired without completion: release the claim
            for k in range(2):
                p = self.prims[k]
                if p is not None and p.kind == "weld" and p.done and not p.completed and self.claim[p.task] == k:
                    self.claim[p.task] = -1
                    e.queues[k] = e.queues[k][:e.queue_pos[k]]
                    e._task_owner[p.task, k] = 0
        info = dict(tau=tau, steps=steps, welds=n_welds, done=done, success=self.finished,
                    collision=self.collision, timeout=reason == "timeout", t=self.t * self.dt,
                    welds_done=int(self.status[:self.n].sum()), n_welds=self.n, min_clearance=self.min_clr,
                    makespan=self.makespan if self.finished else None, infeasible=list(self.infeasible),
                    blame=list(self.blame))
        self.last_info = info
        if not accumulate:
            return None
        return (None if done else self.obs()), R, done, info

    def _phase_name(self, k):
        from .executor import PHASE_NAMES
        p = self.prims[k]
        if p is None or p.done:
            return "idle"
        return PHASE_NAMES[int(p.phase[min(p.i, len(p.phase)) - 1])] if p.i > 0 else PHASE_NAMES[int(p.phase[0])]
