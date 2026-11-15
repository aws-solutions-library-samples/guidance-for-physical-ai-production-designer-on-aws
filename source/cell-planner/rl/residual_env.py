"""Residual RL on top of the scripted expert (Track RR).

The env still receives normalised joint velocities at 20 Hz. Per robot the policy outputs
u = (u_joint (6), u_rate (1)) in [-1, 1]; the executed action is

    a = a_track(sigma) + r          (normalised joint velocity, then the env's clip to [-1, 1])

* ``a_track(sigma)``: the base controller (``RateExpert``, a subclass of ``expert.scripted.ScriptedExpert``)
  tracks the expert's committed dense joint path exactly (deadbeat) on a policy-controlled clock: the
  progress s along the plan advances by sigma waypoints per step (sigma = 1 is the expert's own timing),
  a_track = (path(s + sigma) - q) / (0.7 v_max dt). sigma is the *rate residual*:
  sigma_cmd = 1 + b_r * u_rate. Its dynamics are physical: per step sigma may change only so much that
  the joint acceleration d(sigma v)/dt = sigma' v + sigma^2 a_nom stays within the acceleration budget
  A = accel_frac (0.7, the expert's trapezoid setting) x URDF a_max, and sigma <= sigma_max(s), a
  speed profile computed per plan (velocity <= 0.7 v_max, sigma^2 a_nom <= A on the expert's ramps, and
  braking-feasible for every slower part ahead: vias, descent onset, the stop on the weld). Where the
  expert's own profile is discontinuous (descent onset, stop on the weld) sigma <= 1. So the arm always
  stays exactly on the expert's collision-checked path; sigma < 1 slows / stops it (yield), sigma > 1
  speeds it up where the expert is not at its limits (mostly the 0.4 x speed descents and retracts).
* ``r``: the *joint residual* r = b_j * u_joint (b_j = ``joint_bound``, e.g. 0.3), rate-limited
  (|r_t - r_{t-1}| <= 0.5 A per step) and gated: only in the free-space phases (transit / home) and only
  when enough free-space waypoints remain ahead for r to decay to 0 before the descent starts. With the
  deadbeat tracker r acts as a bounded joint-space offset of the arm from the path (|offset| <= b_j
  0.7 v_max dt per joint); in the precision phases (descent, dwell, retract) r = 0, so the straight-line
  entry (umarell's 15 deg entry gate) and the 10 mm / 5 deg dwell are not perturbed by it.
* Plan finished: the arm holds the plan's end point (braking clamp |a| <= sqrt(2 A |d|)).

With u = 0 (sigma = 1, r = 0) the executed actions are *identical* to ``ScriptedExpert.act`` (tested:
canonical rollout, bit-identical joint trajectory).

Base variants (``ResidualConfig.base``):
  "full"     the full expert (reservation-style yielding: a plan is committed only if the predicted
             clearance against the other arm's committed future stays > 0.25 m; deadlock retreat).
  "noyield"  the same planner, but every plan is committed at once (no inter-arm yielding, no
             deadlock retreat): per-arm motion only. Without a residual it collides in ~all episodes;
             the policy has to learn the coordination (with the clock: stop / slow / go).

Extra observation ``res_feat`` (2, RES_FEAT_DIM) per robot (base-controller state, appended to the
GNN robot-node features by ``rl.residual_models.ResGraphBuilder``): the base command at sigma=1 (6),
sigma (1), phase one-hot [transit, home, descent, retract, wait, rest] (6), remaining plan steps (1),
joint-residual gate (1), predicted arm-arm clearance (umarell sphere metric) over the next ``horizon``
steps for: both arms continue at sigma = 1 / this arm stops / the other arm stops (3), time of the
minimum (1), has-plan (1), the current residual bounds b_j, b_r (2), and the predicted clearance with
both arms continuing at their current clock rates (1).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from env.cell import ROBOT_IDS
from env.weld_env import EnvConfig, KukaWeldEnv
from expert.scripted import WARN, ScriptedExpert

PHASES = ("transit", "home", "descent", "retract", "wait", "rest")
RES_FEAT_DIM = 23
ACT_DIM = 7
CLR_SCALE = 0.5

# Same split as scripts/run_expert.py (the expert's 46.0 s canonical result uses it).
EXPERT_QUEUES = {
    "robot1": ["W03", "W02", "W06", "W05", "W09", "W08"],
    "robot2": ["W15", "W13", "W14", "W10", "W12", "W11", "W07", "W04", "W01"],
}


@dataclass
class ResidualConfig:
    base: str = "full"                     # "full" | "noyield"
    joint_bound: float = 0.3               # |joint residual| <= joint_bound (normalised joint velocity)
    rate_bound: float = 1.0                # sigma_cmd = 1 + rate_bound * u_rate, sigma in [0, 1 + rate_bound]
    rate_delta: float = 0.2                # max |sigma_t - sigma_{t-1}| (on top of the acceleration limit)
    joint_phases: tuple = ("transit", "home")
    accel_frac: float = 0.7                # acceleration budget = accel_frac * URDF a_max (expert trapezoids: 0.7)
    res_rate_frac: float = 0.5             # |r_t - r_{t-1}| <= res_rate_frac * A
    standstill_retreat: int = 0            # >0: an arm held still that many steps mid-plan re-plans via HOME
    standstill_sigma: float = 0.05         # "held still" = clock rate below this
    horizon: int = 40                      # look-ahead (steps) of the predicted-clearance features
    horizon_stride: int = 4


def default_env_config(**kw) -> EnvConfig:
    """Env for the residual track: expert split, env reward + time penalty (step_penalty -0.005)."""
    from env.rewards import RewardConfig
    rew = kw.pop("reward", None) or RewardConfig(step_penalty=-0.005)
    cfg = EnvConfig(assignment=dict(EXPERT_QUEUES), reward=rew)
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


class RateExpert(ScriptedExpert):
    """ScriptedExpert whose plan execution runs on a policy-controlled clock (see module doc).

    ``plan_step(obs)`` = steps 1-3 of ``ScriptedExpert.act`` (re-plan on goal change, commit,
    deadlock retreat); ``execute(sigma_cmd, joint_res, ...)`` = step 4 with the clock / residual.
    ``yielding=False`` commits every plan immediately (no arm-arm coordination).
    """

    def __init__(self, env=None, yielding: bool = True, accel_frac: float = 0.7, sigma_cap: float = 2.0,
                 joint_phases=("transit", "home"), res_rate_frac: float = 0.5, standstill_retreat: int = 0,
                 standstill_sigma: float = 0.05, **kw):
        if not yielding:
            kw.update(clearance_margin=-1e3, clearance_margin_min=-1e3, retreat_after=10 ** 9)
        super().__init__(env, **kw)
        self.yielding = yielding
        self.accel_frac, self.sigma_cap = float(accel_frac), float(sigma_cap)
        self.joint_phases, self.res_rate_frac = tuple(joint_phases), float(res_rate_frac)
        self.standstill_retreat, self.standstill_sigma = int(standstill_retreat), float(standstill_sigma)
        self.a_max = {r: self.cell.robots[r].a_max for r in ROBOT_IDS}

    def reset(self):
        super().reset()
        self.sigma = np.ones(len(ROBOT_IDS))
        self.r = np.zeros((len(ROBOT_IDS), 6))
        self.stand = np.zeros(len(ROBOT_IDS), int)

    def accel_budget(self, rid):
        """Max change of the normalised command per step: accel_frac * a_max * dt / (0.7 v_max)."""
        return self.accel_frac * self.a_max[rid] * self.dt / self.vmax[rid]

    # ---------------------------------------------------------------- commit bookkeeping
    def _try_commit(self, rid, plan, q, other, margin=None):
        ok = super()._try_commit(rid, plan, q, other, margin)
        if ok and plan is not None and self.active[rid] is plan:
            plan.q0 = np.asarray(q[rid], float).copy()
            plan.s = 0.0
            self._prepare(rid, plan)
        return ok

    def _prepare(self, rid, p):
        """Nominal per-step velocities v (N,6), accelerations alpha (N,6), speed profile sig_max (N,),
        and the free-space run length ahead of each waypoint (for the joint-residual gate)."""
        N = len(p.wps)
        V = self.vmax[rid] * self.dt
        P = np.vstack([p.q0[None], p.wps])
        v = np.diff(P, axis=0) / V                                     # (N, 6) step k: P[k] -> P[k+1]
        alpha = np.diff(np.vstack([np.zeros((1, 6)), v]), axis=0)      # change at step k (from rest at k=0)
        A = self.accel_budget(rid)
        av, aa = np.abs(v), np.abs(alpha)
        with np.errstate(divide="ignore", invalid="ignore"):
            c_vel = 1.0 / np.maximum(av.max(1), 1e-9)                  # sigma |v| <= 1
            rho = (aa / A).max(1)                                      # sigma^2 |alpha| <= A
            c_acc = np.where(rho > 1.0 + 1e-6, 1.0, 1.0 / np.sqrt(np.maximum(rho, 1e-12)))
            dec = (A / np.maximum(av, 1e-9)).min(1) / self.sigma_cap   # sigma decrease per waypoint
        prof = np.minimum(np.minimum(c_vel, c_acc), self.sigma_cap)
        # the stop at the end: the expert stops from |v_{N-1}| (its own discontinuity)
        with np.errstate(divide="ignore", invalid="ignore"):
            end = (np.maximum(A, av[-1]) / np.maximum(av[-1], 1e-9)).min()
        prof[-1] = min(prof[-1], end)
        for k in range(N - 2, -1, -1):
            prof[k] = min(prof[k], prof[k + 1] + dec[k])
        p.v, p.alpha, p.sig_max = v, alpha, np.maximum(prof, 1.0)
        jp = np.array([ph in self.joint_phases for ph in p.phase], bool)
        run = np.zeros(N + 1, int)
        for k in range(N - 1, -1, -1):
            run[k] = run[k + 1] + 1 if jp[k] else 0
        p.jp_run = run[:N]

    # ---------------------------------------------------------------- planning (act steps 1-3)
    def plan_step(self, obs):
        """Exactly steps 1-3 of ``ScriptedExpert.act`` (copied; the parent's act() is unchanged)."""
        q = {rid: np.asarray(obs["robot_q"][k], float) for k, rid in enumerate(ROBOT_IDS)}
        goals = self._goals(obs)
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
        for rid in ROBOT_IDS:
            g = goals[rid]
            if g is not None and self.pending[rid] is None and not self._moving(rid) \
                    and self.active[rid] is not None and self.active[rid].info.get("home"):
                self.pending[rid] = self.make_plan(rid, q[rid], *g)
                self.active[rid] = None
                self.blocked[rid] = 0
        a, b = ROBOT_IDS
        order = (a, b) if self.prio == a else (b, a)
        for rid in order:
            other = b if rid == a else a
            if self.pending[rid] is not None and not self._moving(rid):
                m = self.margin_min if self.blocked[rid] >= 2 * self.retreat_after else None
                if self._try_commit(rid, self.pending[rid], q, other, margin=m):
                    self.pending[rid] = None
                    self.blocked[rid] = 0
                    self.prio = other
                else:
                    self.blocked[rid] += 1
                    self.stats["waits"][rid] += 1
        for rid in order[::-1]:
            other = b if rid == a else a
            if self.pending[rid] is not None and self.blocked[rid] >= self.retreat_after \
                    and self.blocked[rid] % 10 == 0 \
                    and not self._moving(rid) and not self._moving(other) \
                    and (self.pending[other] is not None or goals[other] is None) \
                    and not self.retreating[rid]:
                if self.active[rid] is not None and self.active[rid].weld_q is not None \
                        and np.abs(q[rid] - self.active[rid].weld_q).max() < 1e-3:
                    self.last_weld[rid] = (self.active[rid].weld_q, self.active[rid].retract)
                ret = self.make_plan(rid, q[rid])
                if not ret.info.get("failed") and self._try_commit(rid, ret, q, other, margin=self.margin_min):
                    self.pending[rid] = None
                    self.retreating[rid] = True
                    self.blocked[rid] = 0
                    self.stats["replans_home"][rid] += 1
                    break
        # optional (noyield base): an arm the policy has held still for standstill_retreat steps in the
        # middle of a plan re-plans via HOME (the expert's deadlock retreat, triggered by the policy's
        # standstill instead of a blocked reservation). Off by default; never fires with sigma = 1.
        if self.standstill_retreat > 0:
            for k, rid in enumerate(ROBOT_IDS):
                other = ROBOT_IDS[1 - k]
                if self.stand[k] >= self.standstill_retreat and not self.retreating[rid] \
                        and not self.retreating[other] and goals[rid] is not None:
                    ret = self.make_plan(rid, q[rid])
                    if not ret.info.get("failed") and len(ret.wps):
                        self.pending[rid] = None
                        self.active[rid] = None
                        if self._try_commit(rid, ret, q, other, margin=-1e3):
                            self.retreating[rid] = True
                            self.stats["replans_home"][rid] += 1
                    self.stand[k] = 0
                    break
        self._q_now = q
        return q

    # ---------------------------------------------------------------- path helpers
    @staticmethod
    def path_point(p, x):
        """Point at progress x in [0, N] of plan p (0 = commit configuration, k = waypoint k-1)."""
        N = len(p.wps)
        x = float(np.clip(x, 0.0, N))
        k = int(np.floor(x + 1e-9))
        f = x - k
        qa = p.q0 if k == 0 else p.wps[k - 1]
        if f < 1e-9 or k >= N:
            return np.array(qa, float)
        return (1 - f) * qa + f * p.wps[k]

    def _moving_plan(self, rid):
        p = self.active[rid]
        return p if (p is not None and len(p.wps) and p.i < len(p.wps) and hasattr(p, "sig_max")) else None

    def phase_of(self, rid):
        p = self.active[rid]
        if p is not None and p.i < len(p.wps):
            k = min(len(p.wps) - 1, int(np.floor(getattr(p, "s", p.i) + 1e-9)))
            return p.phase[k] if p.phase[k] in PHASES else "transit"
        if self.pending[rid] is not None:
            return "wait"
        return "rest"

    def future(self, rid, q, sigma=1.0, H=40, stride=4):
        """(H // stride, 6) configurations at steps stride, 2*stride, ..., H at a constant clock sigma."""
        steps = np.arange(stride, H + 1, stride)
        p = self.active[rid]
        if p is None or p.i >= len(p.wps) or sigma <= 0:
            return np.repeat(np.asarray(q, float)[None], len(steps), 0)
        s0 = getattr(p, "s", float(p.i))
        return np.stack([self.path_point(p, s0 + sigma * k) for k in steps])

    def gate_open(self, rid, joint_bound):
        """Joint residual allowed: free-space phase and enough free-space waypoints left for r to decay."""
        p = self._moving_plan(rid)
        if p is None or joint_bound <= 0:
            return False
        k = int(np.floor(p.s + 1e-9))
        A = self.accel_budget(rid).min() * self.res_rate_frac
        need = int(np.ceil(joint_bound / A)) + 2
        return bool(p.jp_run[k] > need * self.sigma_cap)

    def hold_command(self, rid, q):
        """Plan finished: back to / rest on its end point (braking clamp); 0 if already there."""
        p = self.active[rid]
        if p is None or len(p.wps) == 0 or np.abs(p.wps[-1] - q).max() < 1e-4:
            return np.zeros(6)
        d = (p.wps[-1] - q) / (self.vmax[rid] * self.dt)
        vs = np.sqrt(2.0 * self.accel_budget(rid) * np.abs(d))
        return np.clip(d, -vs, vs)

    def sigma_step(self, rid, sigma, sigma_cmd, rate_delta):
        """New clock rate: towards sigma_cmd, within rate_delta and the acceleration budget
        |dsigma v_k + sigma^2 alpha_k| <= max(A, |alpha_k|) (per joint), and <= sigma_max ahead."""
        p = self._moving_plan(rid)
        lo, hi = -rate_delta, rate_delta
        if p is None:
            return float(np.clip(sigma + np.clip(sigma_cmd - sigma, lo, hi), 0.0, self.sigma_cap))
        k = int(np.floor(p.s + 1e-9))
        v, al = p.v[k], p.alpha[k]
        A = np.maximum(self.accel_budget(rid), np.abs(al) + 1e-9)
        m = np.abs(v) > 1e-9
        if m.any():
            a = (-A[m] - sigma ** 2 * al[m]) / v[m]
            b = (A[m] - sigma ** 2 * al[m]) / v[m]
            lo = max(lo, float(np.minimum(a, b).max()))
            hi = min(hi, float(np.maximum(a, b).min()))
        if lo > hi:                       # infeasible by a hair (rounding): keep the rate
            lo = hi = 0.0
        new = sigma + float(np.clip(sigma_cmd - sigma, lo, hi))
        k1 = min(len(p.wps), int(np.ceil(p.s + max(new, 1.0))) + 1)
        return float(np.clip(new, 0.0, p.sig_max[k:k1].min()))

    # ---------------------------------------------------------------- execution (act step 4)
    def execute(self, sigma_cmd, joint_res, gate, rate_delta=0.2):
        """sigma_cmd (2,), joint_res (2,6) target joint residual (normalised), gate (2,) 0/1.
        Returns (a (2,6) env action, info dict)."""
        q = self._q_now
        a = np.zeros((2, 6))
        a_base = np.zeros((2, 6))
        sig = self.sigma.copy()
        for k, rid in enumerate(ROBOT_IDS):
            V = self.vmax[rid] * self.dt
            p = self._moving_plan(rid)
            sig[k] = self.sigma_step(rid, float(self.sigma[k]), float(sigma_cmd[k]), rate_delta)
            self.stand[k] = self.stand[k] + 1 if (p is not None and sig[k] < self.standstill_sigma) else 0
            dr = self.accel_budget(rid) * self.res_rate_frac
            r_t = np.asarray(joint_res[k], float) * gate[k]
            self.r[k] = self.r[k] + np.clip(r_t - self.r[k], -dr, dr)
            if p is not None:
                N = len(p.wps)
                a_base[k] = (self.path_point(p, p.s + 1.0) - q[rid]) / V
                s1 = min(float(N), p.s + sig[k])
                a[k] = (self.path_point(p, s1) - q[rid]) / V + self.r[k]
                p.s = float(N) if N - s1 < 1e-6 else s1
                p.i = max(p.i, int(np.floor(p.s + 1e-9)))
                if p.i >= N:
                    self.retreating[rid] = False
            else:
                a_base[k] = a[k] = self.hold_command(rid, q[rid])
                a[k] = a[k] + self.r[k]
        self.sigma = sig
        self.n_steps += 1
        return np.clip(a, -1.0, 1.0), dict(a_base=a_base, sigma=sig.copy())


class ResidualEnv:
    """Wrapper: KukaWeldEnv + RateExpert base + residual action (2, 7). See the module doc.

    Not a gym.Env subclass (dict obs + extra key), but the same reset/step API."""

    def __init__(self, env_cfg: EnvConfig | None = None, rcfg: ResidualConfig | None = None, expert_kw=None):
        self.env = KukaWeldEnv(env_cfg or default_env_config())
        self.rcfg = rcfg or ResidualConfig()
        self.base = RateExpert(self.env, yielding=(self.rcfg.base == "full"), accel_frac=self.rcfg.accel_frac,
                               sigma_cap=1.0 + self.rcfg.rate_bound, joint_phases=self.rcfg.joint_phases,
                               res_rate_frac=self.rcfg.res_rate_frac,
                               standstill_retreat=self.rcfg.standstill_retreat,
                               standstill_sigma=self.rcfg.standstill_sigma, **(expert_kw or {}))
        self.sm = self.base.sm
        self.bounds = np.array([self.rcfg.joint_bound, self.rcfg.rate_bound], float)   # can be annealed
        self._obs = None

    @property
    def cfg(self):
        return self.env.cfg

    def set_bounds(self, joint_bound=None, rate_bound=None):
        if joint_bound is not None:
            self.bounds[0] = joint_bound
        if rate_bound is not None:
            self.bounds[1] = rate_bound

    # ---------------------------------------------------------------- features
    def _features(self, obs):
        b, rc = self.base, self.rcfg
        q = b.plan_step(obs)
        H, st = rc.horizon, rc.horizon_stride
        F = np.zeros((2, RES_FEAT_DIM), np.float32)
        fut1 = [b.future(r, q[r], 1.0, H, st) for r in ROBOT_IDS]
        hold = [np.repeat(q[r][None], len(fut1[0]), 0) for r in ROBOT_IDS]
        c_both = self.sm.pair_clearance(fut1[0], fut1[1])
        c_r1stop = self.sm.pair_clearance(hold[0], fut1[1])
        c_r2stop = self.sm.pair_clearance(fut1[0], hold[1])
        futs = [b.future(r, q[r], float(b.sigma[k]), H, st) for k, r in enumerate(ROBOT_IDS)]
        c_cur = self.sm.pair_clearance(futs[0], futs[1])
        self._gate = np.zeros(2)
        for k, rid in enumerate(ROBOT_IDS):
            ph = b.phase_of(rid)
            p = b._moving_plan(rid)
            has = p is not None
            V = b.vmax[rid] * b.dt
            a1 = (b.path_point(p, p.s + 1.0) - q[rid]) / V if has else b.hold_command(rid, q[rid])
            rem = (len(p.wps) - p.s) if has else 0.0
            gate = float(b.gate_open(rid, float(self.bounds[0])))
            self._gate[k] = gate
            c_self, c_other = (c_r1stop, c_r2stop) if k == 0 else (c_r2stop, c_r1stop)
            F[k, :6] = np.clip(a1, -1, 1)
            F[k, 6] = b.sigma[k] / 2.0
            F[k, 7 + PHASES.index(ph)] = 1.0
            F[k, 13] = np.tanh(rem / 40.0)
            F[k, 14] = gate
            F[k, 15] = np.tanh((c_both.min() - WARN) / CLR_SCALE * 4)
            F[k, 16] = np.tanh((c_self.min() - WARN) / CLR_SCALE * 4)
            F[k, 17] = np.tanh((c_other.min() - WARN) / CLR_SCALE * 4)
            F[k, 18] = (np.argmin(c_both) + 1) / len(c_both)
            F[k, 19] = float(has)
            F[k, 20:22] = self.bounds
            F[k, 22] = np.tanh((c_cur.min() - WARN) / CLR_SCALE * 4)
        self._has_plan = F[:, 19] > 0.5
        out = dict(obs)
        out["res_feat"] = F
        return out

    # ---------------------------------------------------------------- API
    def reset(self, seed=None, options=None):
        obs, info = self.env.reset(seed=seed, options=options)
        self.base.reset()
        self._obs = self._features(obs)
        return self._obs, info

    def effective_u(self, u):
        """The part of u that has an effect (gated joint residual, rate only with an active plan)."""
        u = np.clip(np.asarray(u, np.float32).reshape(2, ACT_DIM), -1, 1).copy()
        u[:, :6] *= self._gate[:, None]
        return u

    def step(self, u):
        u = np.clip(np.asarray(u, float).reshape(2, ACT_DIM), -1, 1)
        bj, br = self.bounds
        sigma_cmd = np.clip(1.0 + br * u[:, 6], 0.0, 1.0 + br)
        joint_res = bj * u[:, :6]
        gate = self._gate.copy()
        a, inf = self.base.execute(sigma_cmd, joint_res, gate, self.rcfg.rate_delta)
        obs, r, term, trunc, info = self.env.step(a.reshape(-1).astype(np.float32))
        info["res"] = dict(a=a, a_base=inf["a_base"], sigma=inf["sigma"], gate=gate,
                           joint_res=self.base.r.copy())
        self._obs = self._features(obs)
        return self._obs, r, term, trunc, info

    def zero_action(self):
        return np.zeros((2, ACT_DIM), np.float32)

    def config(self) -> dict:
        return asdict(self.rcfg)
