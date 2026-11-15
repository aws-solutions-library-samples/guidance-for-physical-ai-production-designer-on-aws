"""Low-level motion primitives for the hierarchical scheduler (Track H).

Built from Track X's building blocks (``expert.scripted.trapezoid_samples`` / ``resample_path``), but
**without** any coordination, via search or collision checking: the transit is always the direct
synchronised trapezoid in joint space (0.7 x URDF velocity, 0.7 x acceleration limits). Whether a
primitive collides with the other arm, with itself or with the floor is entirely the policy's
problem; the env (``KukaWeldEnv``) scores every 20 Hz step.

Primitives (dense joint waypoints, one per control step):
  weld  : transit (current q -> branch pre-weld) + Cartesian descent (0.4 x speed) + hold on the weld
          (``dwell_steps``; extended while the env has not completed the weld, at most ``max_extra_hold``)
          + retract (descent reversed). The arm becomes idle at the pre-weld pose.
  wait  : hold the current configuration for n steps.
  home  : direct trapezoid to HOME.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from expert.scripted import resample_path, trapezoid_samples

WAIT_S = (0.5, 1.0, 2.0)
SPECIALS = ("wait0.5", "wait1", "wait2", "home")
PH_TRANSIT, PH_DESCENT, PH_HOLD, PH_RETRACT, PH_WAIT, PH_HOME = range(6)
PHASE_NAMES = ("transit", "descent", "hold", "retract", "wait", "home")


@dataclass
class Primitive:
    kind: str                      # "weld" | "wait" | "home"
    wps: np.ndarray                # (N,6)
    phase: np.ndarray              # (N,) int
    task: int = -1                 # env task index (weld)
    branch: int = -1
    i: int = 0                     # next waypoint
    hold_at: int = -1              # first hold waypoint index (weld)
    hold_end: int = -1             # first retract waypoint index (weld)
    extra_hold: int = 0
    completed: bool = False
    t_start: int = 0               # env step at commit
    info: dict = field(default_factory=dict)

    @property
    def remaining(self):
        return len(self.wps) - self.i

    @property
    def done(self):
        return self.i >= len(self.wps)

    def future(self, q_now):
        """Nominal remaining waypoints (>= 1 row; the arm then rests at the last one)."""
        if self.done:
            return np.asarray(q_now, float)[None]
        return self.wps[self.i:]


@dataclass(frozen=True)
class SpeedProfile:
    """Fractions of the URDF joint velocity limits (v) and of robots.yaml joint_acceleration_limits (a).
    ``desc_a`` None = constant-speed descent (no acceleration limit)."""
    name: str
    transit_v: float
    transit_a: float
    desc_v: float
    desc_a: float | None
    home_v: float
    home_a: float


# "ours": the executor used for training (0.7 v, 0.7 a; descent/retract at 0.4 x 0.7 v, constant speed).
# "umarell_rule": umarell's planner limits as configured (robots.yaml planning + time_parameterization.parameterize:
#   v = vscale * v_max, a = vscale^2 * a_max; transit vscale = max_velocity_scaling 0.7, approach =
#   approach_velocity_fraction 0.4 x transit (obstacles.yaml) = 0.28, home_velocity_scale 0.4).
# "umarell_observed": what the saved umarell plan bt_1790851970 actually reaches (TOPP-RA legs <= ~0.4 v_max,
#   p99 ~0.2 a_max): umarell's rule evaluated at vscale 0.4 everywhere (transit/home 0.4 v, 0.16 a; approach
#   0.4 x 0.4 = 0.16 v, 0.0256 a).
PROFILES = {
    "ours": SpeedProfile("ours", 0.7, 0.7, 0.28, None, 0.7, 0.7),
    "umarell_rule": SpeedProfile("umarell_rule", 0.7, 0.49, 0.28, 0.28 ** 2, 0.4, 0.16),
    "umarell_observed": SpeedProfile("umarell_observed", 0.4, 0.16, 0.16, 0.16 ** 2, 0.4, 0.16),
}


def path_trapezoid(path, vmax, amax, dt):
    """Time-parameterise a joint polyline (n,6) with a 1-D trapezoid in its arc parameter so that every joint
    respects vmax/amax (straight-line approximation per segment, then a per-step speed check). Samples every dt,
    excluding path[0], including path[-1]."""
    path = np.asarray(path, float)
    seg = np.diff(path, axis=0)
    # arc parameter = cumulative max over joints of |dq| / vmax (time at full speed)
    w = np.abs(seg) / vmax
    ds = w.max(1)
    S = np.concatenate([[0.0], np.cumsum(ds)])
    L = S[-1]
    if L < 1e-12:
        return np.zeros((0, 6))
    # in the parameter s (seconds at full speed), speed <= 1 and accel <= a_s with a_s from the binding joint
    dirn = np.abs(path[-1] - path[0]) / max(L, 1e-12)            # rad per unit s (approx.)
    a_s = float(np.min(np.where(dirn > 1e-9, amax / np.maximum(dirn, 1e-12), np.inf)))
    a_s = a_s if np.isfinite(a_s) else 1e9
    t_acc = 1.0 / a_s
    if L < t_acc:                                                  # triangular
        T = 2 * np.sqrt(L / a_s)
    else:
        T = L + t_acc
    for _ in range(30):
        n = max(1, int(np.ceil(T / dt - 1e-9)))
        ts = np.arange(1, n + 1) * (T / n)
        sc = (L + t_acc) / T if L >= t_acc else 2 * np.sqrt(L / a_s) / T
        if L >= t_acc:
            v, a = sc, a_s * sc * sc
            ta = v / a
            st = np.where(ts <= ta, 0.5 * a * ts ** 2,
                          np.where(ts <= T - ta, 0.5 * a * ta ** 2 + v * (ts - ta), L - 0.5 * a * (T - ts) ** 2))
        else:
            a = a_s * sc * sc
            st = np.where(ts <= T / 2, 0.5 * a * ts ** 2, L - 0.5 * a * (T - ts) ** 2)
        st = np.clip(st, 0, L)
        st[-1] = L
        out = np.stack([np.interp(st, S, path[:, j]) for j in range(6)], 1)
        steps = np.abs(np.diff(np.vstack([path[0], out]), axis=0))
        r = float((steps / (vmax * dt)).max())
        if r <= 1.0 + 1e-9:
            return out
        T *= max(r, 1.01)
    return out


class Executor:
    def __init__(self, cell, dt=0.05, vel_scale=0.7, acc_frac=0.7, approach_frac=0.4, dwell_steps=10,
                 max_extra_hold=30, profile: str | SpeedProfile | None = None):
        """``profile`` (name in PROFILES or a SpeedProfile) overrides vel_scale / acc_frac / approach_frac;
        None keeps the original behaviour (= "ours" with the defaults)."""
        from env.cell import ROBOT_IDS
        self.rids = ROBOT_IDS
        self.dt = dt
        if profile is None:
            profile = SpeedProfile("custom", vel_scale, acc_frac, vel_scale * approach_frac, None, vel_scale, acc_frac)
        elif isinstance(profile, str):
            profile = PROFILES[profile]
        self.profile = profile
        P = profile
        self.vmax = {r: cell.robots[r].v_max * P.transit_v for r in ROBOT_IDS}
        self.amax = {r: cell.robots[r].a_max * P.transit_a for r in ROBOT_IDS}
        self.vdesc = {r: cell.robots[r].v_max * P.desc_v for r in ROBOT_IDS}
        self.adesc = {r: (None if P.desc_a is None else cell.robots[r].a_max * P.desc_a) for r in ROBOT_IDS}
        self.vhome = {r: cell.robots[r].v_max * P.home_v for r in ROBOT_IDS}
        self.ahome = {r: cell.robots[r].a_max * P.home_a for r in ROBOT_IDS}
        self.home = {r: cell.robots[r].home.copy() for r in ROBOT_IDS}
        self.approach_frac = P.desc_v / max(P.transit_v, 1e-9)
        self.dwell_steps = dwell_steps
        self.max_extra_hold = max_extra_hold

    def transit(self, rid, q0, q1):
        return trapezoid_samples(q0, q1, self.vmax[rid], self.amax[rid], self.dt)

    def descent(self, rid, desc):
        if self.adesc[rid] is None:
            return resample_path(np.asarray(desc, float), self.vdesc[rid] * self.dt)
        return path_trapezoid(desc, self.vdesc[rid], self.adesc[rid], self.dt)

    def weld(self, rid, q_now, branch, task, b_idx=-1):
        t = self.transit(rid, q_now, branch.q_pre)
        d = self.descent(rid, branch.desc)
        q_w = d[-1] if len(d) else branch.q_pre
        h = np.repeat(q_w[None], self.dwell_steps, 0)
        r = d[::-1][1:]
        r = np.vstack([r, branch.q_pre[None]]) if len(d) else r
        wps = np.concatenate([t.reshape(-1, 6), d, h, r])
        ph = np.concatenate([np.full(len(t), PH_TRANSIT), np.full(len(d), PH_DESCENT), np.full(len(h), PH_HOLD),
                             np.full(len(r), PH_RETRACT)])
        hold_at = len(t) + len(d)
        return Primitive("weld", wps, ph, task=task, branch=b_idx, hold_at=hold_at, hold_end=hold_at + len(h),
                         info={"n_transit": len(t), "n_descent": len(d)})

    def wait(self, rid, q_now, seconds):
        n = max(1, int(round(seconds / self.dt)))
        return Primitive("wait", np.repeat(np.asarray(q_now, float)[None], n, 0), np.full(n, PH_WAIT))

    def go_home(self, rid, q_now):
        t = trapezoid_samples(q_now, self.home[rid], self.vhome[rid], self.ahome[rid], self.dt).reshape(-1, 6)
        if len(t) == 0:
            t = np.asarray(q_now, float)[None]
        return Primitive("home", t, np.full(len(t), PH_HOME))

    def special(self, rid, q_now, j):
        if j < len(WAIT_S):
            return self.wait(rid, q_now, WAIT_S[j])
        return self.go_home(rid, q_now)

    def next_target(self, p: Primitive, completed_now: bool):
        """Waypoint for this control step; handles the weld hold extension. Advances p.i."""
        if p.kind == "weld":
            if completed_now:
                p.completed = True
            if p.i == p.hold_end and not p.completed and p.extra_hold < self.max_extra_hold:
                p.extra_hold += 1
                return p.wps[p.hold_end - 1]       # keep holding on the weld
        q = p.wps[p.i]
        p.i += 1
        return q
