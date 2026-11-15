"""Canonical plans of every controller as :class:`Plan` objects (per-robot dispatched legs on the executor clock).

Two sources:
* umarell BT plans (two robots robot1 / robot2): ``player.bt_plan.timeline_from_bt_plan`` replays the plan with
  umarell's executor rules (Parallel steps, hold_duration_s, 1.0 s completion margin). Each leg is one
  FollowJointTrajectory goal of one arm; between legs the arm stands still.
* trajgen samples (1-4 robots, any ids): ``trajgen/samples/<name>/expected/timeline.npz`` (NTimeline legs).

``CANONICAL`` lists the plan of every controller that the repo treats as canonical (README Results §1 / §4,
docs/fair_speed.md §1). Paths are relative to the repo root.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from player.timeline import Leg, sample_leg

ROOT = Path(__file__).resolve().parents[2]

# name -> (kind, path, cell / sample, note)
CANONICAL = {
    "umarell_bt_1790851970": ("bt", "third_party/sample-umarell/cells/welding_cell_2_small/agentic_planner/plans/"
                              "bt_1790851970_welding_cell_2_small.json", "welding_cell_2_small",
                              "umarell LLM agent + IK + OMPL + TOPP-RA (parallel arms)"),
    "umarell_bt_1790848575": ("bt", "third_party/sample-umarell/cells/welding_cell_2_small/agentic_planner/plans/"
                              "bt_1790848575_welding_cell_2_small.json", "welding_cell_2_small",
                              "umarell plan, sequential arms"),
    "expert": ("bt", "data/exec/expert_expert_2_small_raw_plan.json", "welding_cell_2_small",
               "scripted expert, execute.py sync_rest export"),
    "hier_q": ("bt", "data/exec/hier_q76500_2_small_raw_plan.json", "welding_cell_2_small",
               "hierarchical GNN scheduler, sync_rest export"),
    "ppo_v3": ("bt", "runs/ppo_v3h/exec/ppo_v3h_snap500_plan.json", "welding_cell_2_small",
               "PPO v3 snap500, sync_rest export (expert homing tail)"),
    "ppo_v1": ("bt", "data/exec/ppo_r8_2_small_raw_plan.json", "welding_cell_2_small",
               "PPO v1 r8, sync_rest export"),
    "dagger": ("bt", "data/exec/e2e2_dA4_v10_plan.json", "welding_cell_2_small", "DAgger G2 v10, sync_rest export"),
    "residual": ("bt", "data/exec/residual_s1_2_small_plan.json", "welding_cell_2_small",
                 "residual RR res_full_s1, sync_rest export (no HOME leg)"),
    "ppo_2t": ("bt", "runs/ppo_2t_exec/ppo_2t_u175_2t_raw_plan.json", "welding_cell_2t",
               "PPO 2t snap175, sync_rest export"),
    "expert_2t": ("bt", "data/exec/expert_expert_2t_raw_plan.json", "welding_cell_2t",
                  "scripted expert in 2t, sync_rest export"),
    "hier_t1": ("bt", "results/hier_p3_2t_t1_canon_plan_canonical.json", "welding_cell_2t",
                "hier_t1, per-weld-cut export (evaluate_hier --export)"),
}
TRAJGEN_SAMPLES = ("welding_cell_2_small", "welding_cell_2t", "two_robots_moved", "welds_moved", "side_welds",
                   "one_robot", "three_robots", "four_robots", "four_robots_fixture", "synthetic_1r", "synthetic_3r",
                   "synthetic_4r")          # `infeasible` has no complete plan (exit 2)
for _s in TRAJGEN_SAMPLES:
    CANONICAL[f"trajgen_{_s}"] = ("trajgen", f"trajgen/samples/{_s}", _s, f"trajgen sample {_s} (expected/)")


@dataclass
class Plan:
    name: str
    kind: str                         # "bt" | "trajgen"
    source: str                       # plan file / sample dir (repo-relative)
    cell: str                         # umarell cell id (bt) or sample name (trajgen)
    robot_ids: tuple
    legs: list                        # list[player.timeline.Leg], absolute executor-clock times
    home: np.ndarray                  # (R, 6)
    weld_events: list                 # [(robot, weld_id, t_hold_start, t_hold_end)]
    v_max: np.ndarray                 # (R, 6)
    a_max: np.ndarray
    q_lo: np.ndarray
    q_hi: np.ndarray
    t_end: float                      # executor end (last node complete)
    meta: dict = field(default_factory=dict)
    bt: dict | None = None            # the BT plan (kind "bt")

    @property
    def R(self) -> int:
        return len(self.robot_ids)

    def robot_legs(self, rid) -> list:
        return sorted([l for l in self.legs if l.robot == rid], key=lambda l: l.t0)

    @property
    def makespan(self) -> float:
        return max(l.t1 for l in self.legs)

    @property
    def last_weld_s(self) -> float:
        return max(e[3] for e in self.weld_events) if self.weld_events else float("nan")

    def sample(self, ts) -> np.ndarray:
        """(len(ts), R, 6) with umarell's semantics (linear inside a leg, stand still between legs)."""
        ts = np.atleast_1d(np.asarray(ts, float))
        out = np.zeros((len(ts), self.R, 6))
        for r, rid in enumerate(self.robot_ids):
            q = np.repeat(self.home[r][None], len(ts), axis=0)
            for leg in self.robot_legs(rid):
                m = ts >= leg.t0
                if m.any():
                    q[m] = sample_leg(leg, ts[m])
            out[:, r] = q
        return out


def _bt_plan(name, path, cell_id, note) -> Plan:
    from env.cell import load_cell
    from player.bt_plan import timeline_from_bt_plan
    from player.timeline import ROBOT_IDS
    p = ROOT / path
    tl = timeline_from_bt_plan(str(p))
    c = load_cell(cell_id)
    st = lambda k: np.stack([getattr(c.robots[r], k) for r in ROBOT_IDS])
    return Plan(name=name, kind="bt", source=path, cell=cell_id, robot_ids=tuple(ROBOT_IDS), legs=list(tl.legs),
                home=np.asarray(tl.home, float), weld_events=[tuple(e) for e in tl.weld_events],
                v_max=st("v_max"), a_max=st("a_max"), q_lo=st("q_lo"), q_hi=st("q_hi"),
                t_end=float(tl.meta["executor_end_s"]), meta=dict(note=note, steps=tl.meta.get("steps", [])),
                bt=json.load(open(p)))


def _trajgen_plan(name, path, sample, note) -> Plan:
    from trajgen.ntimeline import NTimeline
    from trajgen.scene import load_scene
    d = ROOT / path
    tl = NTimeline.load(d / "expected" / "timeline.npz")
    sc = load_scene(d / "scene.yaml")
    rob = {r.rid: r for r in sc.robots} if hasattr(sc.robots[0], "rid") else {r.id: r for r in sc.robots}
    st = lambda k: np.stack([np.asarray(getattr(rob[r], k), float) for r in tl.robot_ids])
    ev = [tuple(e) for e in tl.weld_events]
    return Plan(name=name, kind="trajgen", source=path, cell=sample, robot_ids=tuple(tl.robot_ids),
                legs=list(tl.legs), home=np.asarray(tl.home, float), weld_events=ev,
                v_max=st("v_max"), a_max=st("a_max"), q_lo=st("q_lo"), q_hi=st("q_hi"),
                t_end=float(max(max(l.t_done, l.t1) for l in tl.legs)), meta=dict(note=note))


def load(name: str) -> Plan:
    kind, path, cell, note = CANONICAL[name]
    return _bt_plan(name, path, cell, note) if kind == "bt" else _trajgen_plan(name, path, cell, note)


def from_bt_file(path, cell_id: str, name: str | None = None) -> Plan:
    """Any umarell-format BT plan (e.g. an exported smoothed plan)."""
    p = Path(path)
    rel = str(p.resolve().relative_to(ROOT)) if p.resolve().is_relative_to(ROOT) else str(p)
    return _bt_plan(name or p.stem, rel, cell_id, "")


def from_legs(base: Plan, legs, name: str, t_end: float | None = None, weld_events=None, meta=None) -> Plan:
    """A plan with the same robots / limits as ``base`` but new legs (e.g. smoothed)."""
    legs = list(legs)
    if weld_events is None:
        weld_events = sorted([(l.robot, l.op_id, round(l.t1, 6), round(l.t1 + l.hold_s, 6)) for l in legs
                              if l.kind == "weld"], key=lambda e: e[2])
    return Plan(name=name, kind=base.kind, source=base.source, cell=base.cell, robot_ids=base.robot_ids,
                legs=legs, home=base.home.copy(), weld_events=list(weld_events), v_max=base.v_max, a_max=base.a_max,
                q_lo=base.q_lo, q_hi=base.q_hi,
                t_end=float(t_end if t_end is not None else max(max(l.t_done, l.t1) for l in legs)),
                meta=dict(meta or {}), bt=base.bt)


def synthetic(legs, robot_ids=("robot1", "robot2"), v_max=None, a_max=None, home=None, name="synthetic") -> Plan:
    """Test helper: a plan from hand-made legs with given limits."""
    R = len(robot_ids)
    v = np.full((R, 6), 2.0) if v_max is None else np.asarray(v_max, float).reshape(R, 6)
    a = np.full((R, 6), 3.0) if a_max is None else np.asarray(a_max, float).reshape(R, 6)
    h = np.zeros((R, 6)) if home is None else np.asarray(home, float).reshape(R, 6)
    ev = sorted([(l.robot, l.op_id, round(l.t1, 6), round(l.t1 + l.hold_s, 6)) for l in legs if l.kind == "weld"],
                key=lambda e: e[2])
    return Plan(name=name, kind="synthetic", source="", cell="", robot_ids=tuple(robot_ids), legs=list(legs), home=h,
                weld_events=ev, v_max=v, a_max=a, q_lo=np.full((R, 6), -7.0), q_hi=np.full((R, 6), 7.0),
                t_end=float(max(max(l.t_done, l.t1) for l in legs)))


def make_leg(robot, times, positions, kind="move", op_id="L", hold_s=0.0, margin=1.0) -> Leg:
    times = np.asarray(times, float)
    pos = np.asarray(positions, float).reshape(len(times), 6)
    leg = Leg(robot=robot, op_id=op_id, kind=kind, t0=float(times[0]), t1=float(times[-1]), times=times,
              positions=pos, phase=np.array([""] * len(times), dtype="<U16"), hold_s=hold_s)
    leg.t_done = leg.t1 + hold_s + margin
    return leg
