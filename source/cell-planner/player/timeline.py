"""Continuous multi-arm timeline: legs on one executor clock, resampled joint arrays.

Two arms ``robot1`` / ``robot2`` by default (``ROBOT_IDS``); ``robot_ids`` names any other set of arms (the
umarell executor replay of an N-arm cell, ``player.bt_plan.timeline_from_bt_plan(..., robot_ids=...)``). With the
default pair everything (arrays, files) is exactly as before.

Pure numpy + json (no pinocchio, no mujoco, no yaml), so it imports in both the RL venv
(numpy 2) and the umarell venv (numpy<2).

Sampling semantics follow umarell exactly:
* inside a leg, joint positions are linearly interpolated on time, with the same index
  rule as ``agents/trajectory/tools/time_parameterization.py:_sample_at_time``
  (``hi`` = first sample with ``time_s >= t``, ``lo = hi - 1``). The simulated robot
  controller (``controller/pinocchio_robot_controller.py:_sample_trajectory``) and the
  executor's schedule check (``agents/cell/bt_executor.py:interpolate_trajectory``)
  interpolate linearly as well, so all three agree.
* an arm that has finished a leg holds that leg's last configuration until its next leg
  is dispatched; before its first leg it sits at ``home``.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

ROBOT_IDS = ("robot1", "robot2")
HOME = np.array([0.0, -np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0])
DT = 0.05  # 20 Hz, the plans' own sample interval and the env control step


@dataclass
class Leg:
    """One dispatched trajectory of one arm, placed on the absolute clock.

    ``times`` are absolute (seconds on the timeline clock, already divided by
    ``velocity_scale``), ``positions`` (M, 6), ``phase`` (M,) strings ("transit" /
    "approach" / "" when the source had no phase).
    """
    robot: str
    op_id: str
    kind: str                      # "weld" | "move" | "home"
    t0: float                      # dispatch time (after any start delay / interlock)
    t1: float                      # trajectory end: arm stops here
    times: np.ndarray
    positions: np.ndarray
    phase: np.ndarray
    t_done: float = 0.0            # executor marks the node complete (t1 + hold + margin)
    hold_s: float = 0.0            # hold_duration_s port (0 for MoveHome)
    vscale: float = 1.0
    operation_type: str = ""      # port value ("weld" / "move"), "" for MoveHome
    node_type: str = "ExecuteTrajectory"
    bb_key: str = ""               # blackboard key in the source plan, if any
    synthetic: str = ""            # "" | "interlock" | "blind_home" (executor-generated)

    @property
    def duration(self) -> float:
        return self.t1 - self.t0

    @property
    def t_approach(self) -> Optional[float]:
        """Absolute time the terminal approach stroke starts (first 'approach' sample)."""
        idx = np.flatnonzero(self.phase == "approach")
        return float(self.times[idx[0]]) if idx.size else None

    def rel_trajectory(self, ndigits_q: int = 6, ndigits_t: int = 4) -> list[dict]:
        """The leg as an umarell trajectory list ``[{positions, time_s, phase}]``,
        time relative to the leg start and multiplied back by ``vscale``."""
        out = []
        for tt, q, ph in zip(self.times, self.positions, self.phase):
            d = {"positions": [round(float(v), ndigits_q) for v in q],
                 "time_s": round(float((tt - self.t0) * self.vscale), ndigits_t)}
            if ph:
                d["phase"] = str(ph)
            out.append(d)
        return out

    def to_json(self) -> dict:
        d = asdict(self)
        d["times"] = self.times.tolist()
        d["positions"] = self.positions.tolist()
        d["phase"] = [str(p) for p in self.phase]
        return d

    @classmethod
    def from_json(cls, d: dict) -> "Leg":
        d = dict(d)
        d["times"] = np.asarray(d["times"], float)
        d["positions"] = np.asarray(d["positions"], float).reshape(-1, 6)
        d["phase"] = np.asarray(d["phase"], dtype="<U16")
        return cls(**d)

    @classmethod
    def from_trajectory(cls, robot: str, op_id: str, kind: str, t0: float, traj: list[dict],
                        vscale: float = 1.0, **kw) -> "Leg":
        """Build from an umarell trajectory list (relative ``time_s``)."""
        rel = np.array([float(p.get("time_s", 0.0)) for p in traj]) / (vscale or 1.0)
        pos = np.array([list(p["positions"])[:6] for p in traj], float)
        ph = np.array([str(p.get("phase", "")) for p in traj], dtype="<U16")
        return cls(robot=robot, op_id=op_id, kind=kind, t0=float(t0), t1=float(t0 + rel[-1]),
                   times=t0 + rel, positions=pos, phase=ph, vscale=float(vscale or 1.0), **kw)


def sample_leg(leg: Leg, t: np.ndarray) -> np.ndarray:
    """Vectorised ``_sample_at_time`` on one leg (absolute times). Returns (len(t), 6)."""
    t = np.atleast_1d(np.asarray(t, float))
    times, pos = leg.times, leg.positions
    if len(times) == 1:
        return np.repeat(pos[:1], len(t), axis=0)
    hi = np.searchsorted(times, t, side="left")          # first index with times >= t
    hi = np.clip(hi, 1, len(times) - 1)
    lo = hi - 1
    span = times[hi] - times[lo]
    f = np.where(span > 0, (t - times[lo]) / np.where(span > 0, span, 1.0), 1.0)
    q = pos[lo] + f[:, None] * (pos[hi] - pos[lo])
    q = np.where((t <= times[0])[:, None], pos[0], q)
    q = np.where((t >= times[-1])[:, None], pos[-1], q)
    return q


@dataclass
class Timeline:
    t: np.ndarray                                   # (N,)
    q: np.ndarray                                   # (N, R, 6), R = len(robot_ids) (2 by default)
    legs: list = field(default_factory=list)        # list[Leg]
    weld_events: list = field(default_factory=list)  # [(robot, weld_id, t_start, t_end)]
    meta: dict = field(default_factory=dict)
    home: np.ndarray = field(default_factory=lambda: np.stack([HOME, HOME]))  # (R, 6)
    robot_ids: tuple = ROBOT_IDS                    # arm order of the second axis of q / home

    # ── construction ────────────────────────────────────────────────────────
    @classmethod
    def from_legs(cls, legs: Sequence[Leg], *, dt: float = DT, t_end: Optional[float] = None,
                  home: Optional[np.ndarray] = None, weld_hold_s: Optional[float] = None,
                  meta: Optional[dict] = None, robot_ids: Optional[Sequence[str]] = None) -> "Timeline":
        """Resample ``legs`` on a uniform grid ``0, dt, ...`` up to ``t_end``
        (default: the last leg's ``t_done``, else its ``t1``). ``robot_ids``: the arms (default robot1 / robot2)."""
        rids = ROBOT_IDS if robot_ids is None else tuple(robot_ids)
        R = len(rids)
        home = np.stack([HOME] * R) if home is None else np.asarray(home, float).reshape(R, 6)
        legs = sorted(legs, key=lambda l: (l.t0, rids.index(l.robot)))
        if t_end is None:
            t_end = max([max(l.t_done, l.t1) for l in legs] + [0.0])
        n = int(np.floor(t_end / dt + 1e-9)) + 1
        t = np.round(np.arange(n) * dt, 10)
        tl = cls(t=t, q=np.zeros((n, R, 6)), legs=list(legs), meta=dict(meta or {}), home=home, robot_ids=rids)
        tl.q = tl.sample(t)
        tl.weld_events = tl._weld_events(weld_hold_s)
        return tl

    def _weld_events(self, weld_hold_s: Optional[float]) -> list:
        """(robot, weld_id, t_start, t_end): the TCP is at the weld from the end of the
        weld leg (``t1``) through the executor hold (``hold_s``, default 0.5 s)."""
        ev = []
        for l in self.legs:
            if l.kind == "weld":
                h = l.hold_s if weld_hold_s is None else weld_hold_s
                ev.append((l.robot, l.op_id, round(l.t1, 6), round(l.t1 + h, 6)))
        return sorted(ev, key=lambda e: e[2])

    # ── queries ─────────────────────────────────────────────────────────────
    def robot_legs(self, robot: str) -> list:
        return sorted([l for l in self.legs if l.robot == robot], key=lambda l: l.t0)

    def sample(self, t) -> np.ndarray:
        """q at times ``t`` -> (..., R, 6), exact umarell semantics (no resampling error)."""
        t = np.asarray(t, float)
        flat = np.atleast_1d(t).ravel()
        R = len(self.robot_ids)
        out = np.zeros((flat.size, R, 6))
        for r, rid in enumerate(self.robot_ids):
            q = np.repeat(self.home[r][None], flat.size, axis=0)
            for leg in self.robot_legs(rid):        # later legs override earlier ones
                m = flat >= leg.t0
                if m.any():
                    q[m] = sample_leg(leg, flat[m])
            out[:, r] = q
        return out.reshape(t.shape + (R, 6))

    def leave_time(self, leg: Leg) -> float:
        """When this arm next starts moving after ``leg`` (its next dispatch), or the end."""
        nxt = [l.t0 for l in self.robot_legs(leg.robot) if l.t0 > leg.t0 + 1e-9]
        return min(nxt) if nxt else float(self.t[-1])

    @property
    def makespan(self) -> float:
        """Time the last arm stops moving (end of the last trajectory)."""
        return max(l.t1 for l in self.legs) if self.legs else 0.0

    @property
    def executor_end(self) -> float:
        """Time the executor returns (last node complete, incl. completion margins)."""
        return max(l.t_done for l in self.legs) if self.legs else 0.0

    def welds_done_by(self, t: float) -> list[str]:
        return [e[1] for e in self.weld_events if e[3] <= t + 1e-9]

    # ── I/O ─────────────────────────────────────────────────────────────────
    def save(self, path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        extra = {} if tuple(self.robot_ids) == ROBOT_IDS else \
            {"robot_ids_json": np.array(json.dumps(list(self.robot_ids)))}      # trajgen.ntimeline's key
        np.savez_compressed(
            path, t=self.t, q=self.q, home=self.home,
            legs_json=np.array(json.dumps([l.to_json() for l in self.legs])),
            weld_events_json=np.array(json.dumps([list(e) for e in self.weld_events])),
            meta_json=np.array(json.dumps(self.meta)), **extra)
        return path

    @classmethod
    def load(cls, path) -> "Timeline":
        z = np.load(path, allow_pickle=False)
        legs = [Leg.from_json(d) for d in json.loads(str(z["legs_json"]))]
        ev = [tuple(e) for e in json.loads(str(z["weld_events_json"]))]
        rids = ROBOT_IDS
        if "robot_ids_json" in z.files:
            ids = tuple(json.loads(str(z["robot_ids_json"])))
            if set(ids) != set(ROBOT_IDS):            # the pair (any column order) keeps the old player semantics
                rids = ids
        return cls(t=z["t"], q=z["q"], legs=legs, weld_events=ev,
                   meta=json.loads(str(z["meta_json"])), home=z["home"], robot_ids=rids)


def timeline_from_arrays(t: np.ndarray, q: np.ndarray, weld_events=(), meta=None,
                         op_prefix: str = "ROLLOUT") -> Timeline:
    """Wrap a raw rollout (t (N,), q (N,2,6)) as a Timeline with one leg per arm."""
    t = np.asarray(t, float)
    q = np.asarray(q, float).reshape(len(t), 2, 6)
    legs = [Leg(robot=rid, op_id=f"{op_prefix}_{rid}", kind="move", t0=float(t[0]), t1=float(t[-1]),
                times=t.copy(), positions=q[:, r].copy(), phase=np.array([""] * len(t), dtype="<U16"),
                t_done=float(t[-1]))
            for r, rid in enumerate(ROBOT_IDS)]
    return Timeline(t=t, q=q, legs=legs, weld_events=list(weld_events), meta=dict(meta or {}),
                    home=q[0].copy())
