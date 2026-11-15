"""umarell-fast orchestrator: umarell's classic planning flow driven by code, with every independent piece of
work running at the same time in a process pool. Shared by U2 (decider + Opus fallback) and U3 (rules only).

The classic flow the LLM drives (agents/trajectory/classic_prompt.py) is

    run_mrta_pipeline -> one run_trajectory_planner call per step -> validate_approach / validate_trajectory_set
    -> recover failures -> chain_trajectories -> assemble_behaviour_tree

and every tool here is umarell's own (called through ``.func``), so the plan JSON / XML and every gate are
umarell's. What changes is WHO decides and WHEN things run:

1. **Allocation** -- ``run_mrta_pipeline`` in the parent (0.1 s), while the worker pool starts; then every weld's
   arm is checked by IK from HOME (all at once) and a step's welds are re-assigned among the arms that reach them
   (:meth:`Orchestrator.fix_allocation`; umarell's score mirrors side-panel welds to the far arm).
2. **All legs at once.** A leg's start is the end of the arm's previous leg. That end is a deterministic function of
   the previous start (IK warm-started from it: approach waypoint, then a 6-step straight descent;
   :func:`umarell_fast.workers.predict_end`), so the parent predicts every arm's chain by IK alone and submits every
   leg of every arm immediately, each with its chaining state injected. When a leg comes back its real end is
   compared with the prediction; on a mismatch only the dependants are re-predicted and re-planned. Validation of
   the terminal approach (``validate_approach``) runs inside the same task.
3. **Failures** -- every recovery candidate the :class:`Policy` proposes runs concurrently; the highest-priority
   candidate that plans is kept once every higher-priority one has finished. When none plans, the policy may hand the
   operation to another arm (``reallocate_operation``) -- appended as that arm's last operation -- or declare it
   unreachable.
4. **Steps** -- every step is gated with umarell's assembler gate (``_gate_multi_arm_step``) at the same time, with
   the bystander poses the assembler will use. Steps the gate would serialise get improvement candidates
   concurrently (start delays from ``find_start_delays``; coupled re-plans in both orders); blocked steps get coupled
   re-plans and park moves.
5. **Assembly** -- candidate trees (home at end, without home, final parks) are assembled by
   ``assemble_behaviour_tree`` in workers at the same time; the first acceptable shape is re-assembled in the parent
   session and saved with umarell's ``plan_store.save_plan`` (same location and format as the LLM planner).

Every decision is a method of :class:`Policy`. U3 uses the base class (rules). U2 overrides the methods it wants a
model for. Nothing here calls an LLM.
"""
from __future__ import annotations

import concurrent.futures as cf
from concurrent.futures.process import BrokenProcessPool
import copy
import json
import multiprocessing as mp
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np

from umarell_fast import plan_cache as PC
from umarell_fast import workers as W

TOL_RAD = 1e-5         # a real end "equals" the predicted one (umarell's chain tolerance is 1e-3)
# pool watchdog (Orchestrator._wait): how often a wait looks at the pool, and how long tasks may be in flight without
# any of them finishing before the run fails as hung (one OMPL task takes seconds to a few minutes)
POOL_POLL_S = 5.0
STALL_S = float(os.environ.get("UF_STALL_S", 1800))
CHAIN_TOL_RAD = 1e-3   # behaviour_tree._CHAIN_TOL_RAD


# ─── data ────────────────────────────────────────────────────────────────────

@dataclass(eq=False)
class Leg:
    """One motion of one arm: a weld (``src`` = the task operation id) or a move (via / park)."""
    ns: str
    op_id: str
    req: dict
    kind: str = "weld"                      # weld | via | park
    src: Optional[str] = None
    status: str = "pending"                 # pending | planning | waiting | recovering | ok | dropped
    start: Optional[list] = None            # the start it was (or is being) planned from
    pred_end: Optional[list] = None
    traj: Optional[list] = None
    end: Optional[list] = None
    approach: Optional[dict] = None
    failures: list = field(default_factory=list)
    version: int = 0
    attempts: list = field(default_factory=list)
    planning_time_s: float = 0.0
    rec: Optional[dict] = None              # what the planner recorded when it shipped the leg (workers.leg_records)
    firm: bool = True                       # planned from a real end (False: from the previous leg's prediction)
    ctx_pose: Optional[str] = None          # the previous leg's pose this leg was planned with (pose_key)

    @property
    def pose(self) -> Optional[dict]:
        """The working pose the arm stands in after this leg: the one the planner recorded (its real orientation,
        e.g. a spun weld pose), else the one the request asks for."""
        if self.rec is not None and "last_pose" in self.rec:
            return self.rec["last_pose"]
        return W.leg_pose(self.req)

    def set_rec(self, r: dict) -> None:
        if "last_pose" in r:
            self.rec = {k: r.get(k) for k in ("last_pose", "weld_pose", "op_spec")}

    def short(self) -> dict:
        return {"ns": self.ns, "op_id": self.op_id, "kind": self.kind, "src": self.src, "status": self.status,
                "approach_ok": (self.approach or {}).get("matches") if self.approach else None,
                "n_wp": len(self.traj or []), "duration_s": dur(self.traj), "attempts": self.attempts,
                "failures": [{k: f.get(k) for k in ("reason_code", "error") if k in f} for f in self.failures][:3]}


def dur(traj) -> float:
    return float(traj[-1].get("time_s", 0.0)) if traj else 0.0


def pose_key(p: Optional[dict]) -> str:
    """A comparable form of a working pose (None = no seam)."""
    if not p:
        return "-"
    o = p.get("orientation") or {}
    return json.dumps([[round(float(v), 6) for v in p.get("target") or []],
                       [round(float(v), 6) for v in p.get("approach_vector") or []],
                       {k: round(float(o[k]), 6) for k in sorted(o)}])


def gap(a, b) -> float:
    if a is None or b is None:
        return float("inf")
    return float(np.max(np.abs(np.asarray(a, float) - np.asarray(b, float))))


@dataclass
class Candidate:
    """A recovery / repair option: legs to plan in order on one arm (the last one replaces the failed leg)."""
    name: str
    legs: list                     # [Leg] templates (not yet planned)
    priority: int = 0
    coupled: bool = False


@dataclass
class Params:
    velocity_scale: float = 0.3            # transit (Opus's most common choice in track L; cell max 0.7)
    approach_velocity_scale: float = 0.25
    approach_offset_m: float = 0.12
    via_heights_m: tuple = (0.25, 0.35, 0.5)
    park_lifts_m: tuple = (0.3, 0.5)
    park_retreats_m: tuple = (0.0, 0.4, 0.7)
    max_gate_rounds: int = 3
    max_repairs: int = 12
    handoff_parks: int = 4
    improve_timeout_s: float = 8.0     # serialised-step improvement candidates (delays / coupled)
    repair_timeout_s: float = 15.0     # blocked-step repair candidates
    retries: int = 2
    retreat_offsets_m: tuple = (0.08, 0.05, 0.03)   # liftoff_blocked: shorter on-axis lift-off (umarell's lever)
    max_same_conflict: int = 2         # repairs of one conflict (same legs, same arms) before it is given up
    overlap_rounds: int = 32           # N-arm overlap compaction: rounds of (all candidates gated at once)
    overlap_budget_s: float = 180.0    # ... and its wall-time budget (the program so far stays valid at any point)
    home_return: bool = True           # recovery: a planned return to HOME before a weld that failed chained
    stage_max: int = 10                # recovery: staging moves (one per seed, all at once); 0 = off
    stage_arm_deg: tuple = (-70.0, 110.0)          # A2 / A3 of the staging seeds (a KR-type arm reaching forward)
    stage_wrists_deg: tuple = ((0.0, 45.0), (90.0, 45.0), (-90.0, 45.0), (180.0, -45.0), (0.0, -45.0))  # A4, A5


# ─── policy (all discrete choices) ───────────────────────────────────────────

class Policy:
    """Every discrete choice of the orchestrator. The base class is U3's rule set (no model):

    * weld order and arm = MRTA's (Opus kept it 88/88 in track L);
    * step mode = PARALLEL for every multi-arm step (Opus 57/57); umarell's gate serialises what does not clear;
    * planning parameters = constants (:class:`Params`);
    * recovery = all candidates of umarell's ladder at once, chosen by ladder order;
    * reallocation after the ladder; unreachable when every arm failed (umarell's rule);
    * home_at_end = true, false only when no home shape assembles.
    """
    name = "rules"

    def __init__(self, params: Optional[Params] = None):
        self.p = params or Params()

    # -- requests -------------------------------------------------------------------------------
    def weld_request(self, ns: str, op: dict, op_id: Optional[str] = None) -> dict:
        p = op["position"]
        r = {"robot_ns": ns, "operation_id": op_id or op["id"], "operation_type": op.get("type", "weld"),
             "target": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
             "approach_vector": list(op.get("approach_vector", [0, 0, -1])), "approach": True,
             "approach_offset_m": self.p.approach_offset_m,
             "approach_velocity_scale": self.p.approach_velocity_scale}
        ori = op.get("weld_orientation") or op.get("orientation")
        if ori:
            r["orientation"] = dict(ori)
            r["constrain_orientation"] = True
        return r

    def move_request(self, ns: str, op_id: str, xyz) -> dict:
        return {"robot_ns": ns, "operation_id": op_id, "operation_type": "move",
                "target": {"x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2])},
                "approach_vector": [0, 0, -1], "approach": False}

    def call_kwargs(self) -> dict:
        return {"velocity_scale": self.p.velocity_scale, "approach_offset_m": self.p.approach_offset_m}

    # -- recovery ---------------------------------------------------------------------------------
    def recovery_candidates(self, orch: "Orchestrator", leg: Leg) -> list:
        """Candidates for a failed leg, all planned at once (priority = ladder order; lower is better).

        Reason codes (skills/trajectory-classic/recover-from-planner-failure): a weld point inside a fixture box
        has no fix (empty list -> unreachable without reallocation); IK failure -> straight to reallocation;
        everything else gets: retry (OMPL is randomised), a via point above the weld at several heights (the move
        Opus chose in 13 of 21 excessive_tool_rise cases), and a longer / shorter standoff.
        """
        codes = {f.get("reason_code") for f in leg.failures}
        errs = " ".join(str(f.get("error", "")) for f in leg.failures)
        diag = [f.get("target_diagnosis") for f in leg.failures if f.get("target_diagnosis")]
        if any((d or {}).get("target_status") == "inside_box" for d in diag):
            return []
        if "IK failed" in errs and not codes - {None}:
            return []
        cands = []
        pr = 0
        if "liftoff_blocked" in codes:
            # umarell 76ea13a refuses a leg whose on-axis lift-off from the previous weld is blocked (a retry or a
            # different approach does not change the lift-off); its recovery table's lever: a shorter retreat
            for ro in self.p.retreat_offsets_m:
                l2 = orch.clone_leg(leg)
                l2.req["retreat_offset_m"] = ro
                cands.append(Candidate(f"retreat{int(round(ro * 100))}", [l2], priority=pr)); pr += 1
        for k in range(self.p.retries):
            cands.append(Candidate(f"retry{k + 1}", [orch.clone_leg(leg, suffix=None)], priority=pr)); pr += 1
        if leg.kind == "weld":
            av = np.asarray(leg.req.get("approach_vector", [0, 0, -1]), float)
            av = av / (np.linalg.norm(av) + 1e-9)
            t = leg.req["target"]
            tgt = np.array([t["x"], t["y"], t["z"]])
            for h in self.p.via_heights_m:
                via = orch.new_move(leg.ns, f"{leg.op_id}_VIA{int(round(h * 100))}", tgt - av * h, kind="via")
                cands.append(Candidate(f"via{int(round(h * 100))}", [via, orch.clone_leg(leg)], priority=pr))
                pr += 1
            for off in (0.2, 0.08):
                l2 = orch.clone_leg(leg)
                l2.req["approach_offset_m"] = off
                cands.append(Candidate(f"offset{int(off * 100)}", [l2], priority=pr)); pr += 1
        else:
            for dz in (0.15, 0.3):
                l2 = orch.clone_leg(leg)
                l2.req["target"] = dict(l2.req["target"], z=float(l2.req["target"]["z"]) + dz)
                cands.append(Candidate(f"raise{int(dz * 100)}", [l2], priority=pr)); pr += 1
        stage = self.stage_candidates(orch, leg, pr)
        cands += stage + self.home_candidates(orch, leg, pr + len(stage))
        return cands

    def stage_seeds(self, orch: "Orchestrator", leg: Leg) -> list:
        """Warm starts for staging moves (6-axis arms): A1 as at HOME, then A1 facing the weld (when they differ by
        more than 10 deg), the arm reaching forward (``stage_arm_deg``), each wrist of ``stage_wrists_deg`` -- the
        branches the weld-pose search and the IK can land on -- then HOME; the first ``stage_max`` that are not the
        leg's own start (it just failed). (body_weld_3: the clear seeds of W03 / robot1, W07 / W09 / W10 on robot2
        all have A1 as at HOME; facing the weld helps the arms that stand at an angle to it.)

        Not reduced to "distinct branches" by an IK of the specified pose: seeds whose specified-pose IK lands on
        the same branch still differ in what the weld-pose search finds from them (body_weld_3 W03 on robot1: from
        HOME none of the 407 spin / tilt poses is usable, from a wrist-up seed with the same specified-pose branch
        the search picks spin 105 deg and the stroke is clear)."""
        ns = leg.ns
        home = [float(v) for v in orch.home[ns]]
        seeds: list = []
        if len(home) >= 6:
            face = orch.facing_a1(ns, leg.req)
            a1s = [home[0]] + ([face] if abs(face - home[0]) > np.radians(10.0) else [])
            a2, a3 = (np.radians(v) for v in self.p.stage_arm_deg)
            for a1 in a1s:
                for a4, a5 in self.p.stage_wrists_deg:
                    q = list(home)
                    q[:6] = [a1, a2, a3, np.radians(a4), np.radians(a5), 0.0]
                    seeds.append([float(v) for v in q])
        seeds.append(home)
        out = [q for q in seeds if leg.start is None or gap(q, leg.start) > CHAIN_TOL_RAD]
        return out[:max(0, int(self.p.stage_max))]

    def stage_candidates(self, orch: "Orchestrator", leg: Leg, priority: int) -> list:
        """A weld whose straight descent is blocked from where the arm starts may be clear on another arm branch:
        the planner warm-starts the weld-pose search, the IK at the weld / stand-off and the descent from the leg's
        start, so the start decides the wrist / elbow branch (body_weld_3: W03 on robot1 and W10 on robot2 are
        blocked from HOME and from the previous weld's end by the forearm on the C-pillar / sill -- every one of the
        search's 407 spin / tilt poses -- and clear from a wrist-up start). The repair: a staging move to the
        stand-off configuration of a branch whose stroke is clear (workers.plan_stage: checked with the planner's
        own predicate from each seed of :meth:`stage_seeds`, then planned there with umarell's joint-target
        planner: lift-off out of the previous seam, collision-free path), then the weld from there, with the pose
        the search chose from the stand-off pinned. One candidate per seed (they run at once), after the ladder's
        own levers, before the trip HOME."""
        if leg.kind != "weld" or self.p.stage_max <= 0 or leg.start is None or not leg.req.get("orientation"):
            return []
        out = []
        for i, seed in enumerate(self.stage_seeds(orch, leg)):
            out.append(Candidate(f"stage{i + 1}", [orch.stage_leg(leg.ns, leg, seed), orch.clone_leg(leg)],
                                 priority=priority + i))
        return out

    def home_candidates(self, orch: "Orchestrator", leg: Leg, priority: int) -> list:
        """A weld that failed from a CHAINED start (the end of the arm's previous weld) may plan from HOME: the
        planner solves the weld's IK, its orientation search and the stroke's descent warm-started from where the
        arm stands, so from the previous weld's pose it can land on another wrist / elbow branch whose straight
        descent hits the panel, or the lift-off out of the previous seam leaves the arm where no clear descent
        starts (body_weld_3: W02 / W08 plan from HOME, fail chained). The repair: a planned return to
        HOME (umarell's own return: on-axis lift-off, collision-free path; workers.plan_home_return), then the weld
        exactly as an arm's first weld plans it -- also with the ladder's first via / longer standoff from there.
        Last in the ladder (it costs the trip home); none for an arm already at HOME."""
        if leg.kind != "weld" or not self.p.home_return or leg.start is None:
            return []
        if gap(leg.start, orch.home.get(leg.ns)) <= CHAIN_TOL_RAD:
            return []                       # planned from HOME already: the trip home changes nothing
        out = []
        av = np.asarray(leg.req.get("approach_vector", [0, 0, -1]), float)
        av = av / (np.linalg.norm(av) + 1e-9)
        t = leg.req["target"]
        tgt = np.array([t["x"], t["y"], t["z"]])
        h = self.p.via_heights_m[0]
        variants = [("home", []), (f"home_via{int(round(h * 100))}",
                                   [orch.new_move(leg.ns, f"{leg.op_id}_VIA{int(round(h * 100))}", tgt - av * h)])]
        for name, vias in variants:
            out.append(Candidate(name, [orch.home_leg(leg.ns, leg.op_id)] + vias + [orch.clone_leg(leg)],
                                 priority=priority))
            priority += 1
        l2 = orch.clone_leg(leg)
        l2.req["approach_offset_m"] = 0.2
        out.append(Candidate("home_offset20", [orch.home_leg(leg.ns, leg.op_id), l2], priority=priority))
        return out

    def allow_reallocation(self, leg: Leg) -> bool:
        diag = [f.get("target_diagnosis") for f in leg.failures if f.get("target_diagnosis")]
        return leg.kind == "weld" and not any((d or {}).get("target_status") == "inside_box" for d in diag)

    # -- steps --------------------------------------------------------------------------------
    def step_mode(self, step: dict) -> str:
        return "PARALLEL"

    def improve_sequential(self, step: dict) -> bool:
        """Try to make a step the gate would serialise run in parallel (delays / coupled re-plans)."""
        return True

    def choose_step_variant(self, variants: list) -> Optional[dict]:
        """``variants``: ``[{"name", "mode", "duration_s", ...}]`` that passed the gate. Prefer PARALLEL, then the
        shortest step."""
        ok = [v for v in variants if v.get("mode") in ("PARALLEL", "SEQUENTIAL")]
        if not ok:
            return None
        return min(ok, key=lambda v: (v["mode"] != "PARALLEL", v["duration_s"], v.get("priority", 0)))

    def check_allocation(self) -> bool:
        """Before planning, move a weld MRTA gave to an arm that cannot reach it (IK from HOME) to one that can,
        within its MRTA step (see :meth:`Orchestrator.fix_allocation`)."""
        return True

    def compact_steps(self) -> bool:
        """Merge single-arm steps into parallel ones where the gate allows (after the repairs)."""
        return True

    def overlap_steps(self) -> bool:
        """3+ arms: after the pairwise merges, let independent arms' chains overlap (:meth:`Orchestrator.overlap`):
        a leg moves up into an earlier step when its arm has nothing in between and every changed step passes
        umarell's gate. Two-arm cells keep the pairwise compaction only."""
        return True

    def choose_blocked_repair(self, cands: list) -> Optional[dict]:
        """Repair of a blocked step: a coupled re-plan (no new motion, same pairing) before a deferral (no new
        motion, new pairing) before a park move (new motion); within a kind PARALLEL, then the shortest."""
        ok = [c for c in cands if c.get("mode") in ("PARALLEL", "SEQUENTIAL")]
        if not ok:
            return None
        return min(ok, key=lambda c: (c.get("priority", 0), c["mode"] != "PARALLEL", c["duration_s"]))

    def park_targets(self, orch: "Orchestrator", ns: str, from_q: list) -> list:
        """Park poses for an arm standing at ``from_q``: its TCP raised and pulled back toward its own base
        (the "toward own base and raised" poses Opus found by nudging in track L), all tried at once."""
        tcp = orch.tcp(ns, from_q)
        base = np.asarray(orch.robot_cfg[ns]["base_position"], float)
        d = base[:2] - tcp[:2]
        n = np.linalg.norm(d)
        d = d / n if n > 1e-6 else np.zeros(2)
        out = [("home", orch.tcp(ns, orch.home[ns]))]          # the arm's HOME tool position first
        for back in sorted(self.p.park_retreats_m, reverse=True):  # then farthest back first
            for up in self.p.park_lifts_m:
                xyz = np.array([tcp[0] + d[0] * back, tcp[1] + d[1] * back, tcp[2] + up])
                out.append((f"b{int(back * 100)}u{int(up * 100)}", xyz))
        return out

    def home_variants(self) -> list:
        """Plan shapes tried at assembly, in preference order."""
        return [{"name": "home", "home_at_end": True}, {"name": "no_home", "home_at_end": False}]

    def accept_tree(self, result: dict) -> bool:
        return bool(result.get("ok"))


class PoolFailure(RuntimeError):
    """The worker pool cannot finish the run: a worker process died (SIGKILL: most likely the kernel's OOM killer;
    a crash) or tasks were in flight for ``stall_s`` without any of them finishing (a hung worker). The orchestrator
    raises it from its waits instead of waiting forever; ``python -m umarell_fast.u3`` reports it and exits 1."""


def _exit_note(code: Optional[int]) -> str:
    if code is None:
        return "running"
    if code < 0:
        import signal
        try:
            name = signal.Signals(-code).name
        except ValueError:
            name = f"signal {-code}"
        return f"killed by {name}" + (" (most likely the kernel's OOM killer: too many workers for the memory)"
                                      if -code == 9 else "")
    return f"exit code {code}"


# ─── orchestrator ────────────────────────────────────────────────────────────

class Orchestrator:
    def __init__(self, repo: Path, cell: str, cell_base: Path, env: dict, policy: Policy, *, workers: int = 8,
                 kernels: str = "npplus", seed: Optional[int] = 0, task: str = "car_door_spot_welding.json",
                 log: Callable[[str], None] = print, pool: Optional[cf.Executor] = None):
        self.repo, self.cell, self.cell_base, self.env = Path(repo), cell, Path(cell_base), env
        self.policy, self.n_workers, self.kernels, self.seed, self.task_name = policy, workers, kernels, seed, task
        self.log = log
        self._pool = pool
        self.events: list = []
        self.t0 = time.monotonic()
        self.steps: list = []            # [{ns: Leg}]
        self.futs: dict = {}             # future -> (kind, payload)
        self.recovering: dict = {}       # leg -> group dict
        self.unreachable: list = []
        self.reallocated: list = []
        self.gate_info: dict = {}
        self.phase_t: dict = {}
        self._n = 0
        self.stall_s = STALL_S
        self.poll_s = POOL_POLL_S
        self._last_done = time.monotonic()   # the last time any task finished (the stall watchdog)
        self._t_sub: dict = {}               # future -> monotonic submit time, while it runs
        self._hit_wall: dict = {}            # plan cache hit future -> the task's recorded wall time
        self._fut_key: dict = {}             # cached task future -> its plan cache key
        self._consumed: list = []            # plan cache keys in the order this run used their results (journal)
        self._dropped: list = []             # plan cache keys this run dropped at a deadline (journal)

    # -- infrastructure -----------------------------------------------------------------------
    def now(self) -> float:
        return round(time.monotonic() - self.t0, 3)

    def pool(self) -> cf.Executor:
        if self._pool is None:
            ctx = mp.get_context("spawn")
            self._pool = cf.ProcessPoolExecutor(
                max_workers=self.n_workers, mp_context=ctx, initializer=W.init,
                initargs=(str(self.repo), self.cell, self.env, self.kernels, self.seed))
        return self._pool

    def plan_cache(self):
        """The persistent task cache of this run (umarell_fast.plan_cache; U3_PLAN_CACHE=1), else None. Opened on
        first use: the key holds the kernels, which setup_parent may still change."""
        if not PC.enabled():
            return None
        if getattr(self, "_plan_cache", None) is None:
            try:
                self._plan_cache = PC.PlanCache(PC.cache_root(), self.cell, self.cell_base / self.cell, self.repo,
                                                self.kernels, self.env, log=self.log)
                self.log(f"[uf] plan cache {self._plan_cache.dir} ({self._plan_cache.n_files} cell files hashed in "
                         f"{self._plan_cache.digest_s} s)")
            except Exception as exc:  # noqa: BLE001 -- no cache, plan as before
                self.log(f"[uf] plan cache off ({exc!r})")
                self._plan_cache = False
        return self._plan_cache or None

    @property
    def _journal(self) -> dict:
        """{plan cache key: position} of the last successful run of this cell (its order of using task results)."""
        pc = self.plan_cache()
        if pc is None:
            return {}
        if getattr(self, "_journal_idx", None) is None:
            self._journal_idx = {}
            for i, k in enumerate(pc.read_journal()):
                self._journal_idx.setdefault(k, i)
        return self._journal_idx

    def _replay_order(self, f) -> tuple:
        """Order to use finished tasks in: plan cache hits (they finish all at once) as the last run used them (its
        journal), else by the task's recorded run time; then everything else as submitted. A warm run so makes
        the decisions of the run it replays (the deadlines of _within too)."""
        if f in self._hit_wall:
            return (0, self._journal.get(self._fut_key.get(f), float("inf")), self._hit_wall[f])
        return (1, 0, (self.futs.get(f) or (None, None, 0.0))[2])

    @property
    def _journal_dropped(self) -> set:
        """Plan cache keys the replayed run dropped at a deadline (its journal's ``dropped``)."""
        pc = self.plan_cache()
        if pc is None:
            return set()
        if getattr(self, "_journal_drop", None) is None:
            self._journal_drop = set(pc.read_journal_dropped())
        return self._journal_drop

    def _drop_key(self, f) -> None:
        k = self._fut_key.get(f)
        if k is not None:
            self._dropped.append(k)

    def _consume(self, f) -> None:
        k = self._fut_key.get(f)
        if k is not None:
            self._consumed.append(k)

    def submit(self, kind: str, payload: Any, fn: Callable, *args) -> cf.Future:
        pc = self.plan_cache() if fn not in (W.ping, W.kernel_selfcheck) else None
        name = getattr(fn, "__name__", "")
        key = None
        if pc is not None:
            key = pc.task_key(name, args)
            hit, res = pc.get(name, key)
            if hit:                                  # a finished future: callers wait / iterate as on any other
                fut: cf.Future = cf.Future()
                fut.set_result(res)
                # its run time when it was computed: hits come back in that order and miss the same deadlines
                # (_as_completed / _within), so a warm run makes the cold run's decisions
                self._hit_wall[fut] = float(res.get("wall_s") or 0.0) if isinstance(res, dict) else 0.0
                self._fut_key[fut] = key
                self.futs[fut] = (kind, payload, self.now())
                self._last_done = time.monotonic()
                return fut
        if fn is W.gate_many and len(args) == 2 and len(args[1]) > 1 and not isinstance(self.pool(),
                                                                                      cf.ThreadPoolExecutor):
            fut = self._split_gate_many(*args)
            if pc is not None:                       # combined in the parent: stored from its done callback
                fut.add_done_callback(lambda f, pc=pc, name=name, key=key: pc.put(name, key, f.result())
                                      if not f.cancelled() and f.exception() is None else None)
        elif pc is not None:                         # a miss: the worker runs the task and stores its result
            cand = pc.similar_candidate(name, key)
            splice = pc.splice_candidate(name, args) if cand is None else None
            if splice is not None:                   # the same weld planned from another start: re-plan the transit
                fut = self.pool().submit(PC.splice_or_run, splice, str(pc.dir), key, name, pc.neg_ttl_s, fn, *args,
                                         similar_dir=str(pc.similar_dir), obst=pc.obst)
                pc._count(name, "splice_tried")
                fut.add_done_callback(lambda f, pc=pc, name=name: pc._count(name, "splice_used")
                                      if not f.cancelled() and f.exception() is None and isinstance(f.result(), dict)
                                      and f.result().get("_cache") == "splice" else None)
            elif cand is not None:                     # planned in a similar cell: re-checked here, else planned
                fut = self.pool().submit(PC.reuse_or_run, cand, str(pc.dir), key, name, pc.neg_ttl_s, fn, *args,
                                         similar_dir=str(pc.similar_dir), obst=pc.obst)
                pc._count(name, "similar_tried")
                fut.add_done_callback(lambda f, pc=pc, name=name: pc._count(name, "similar_reused")
                                      if not f.cancelled() and f.exception() is None and isinstance(f.result(), dict)
                                      and str(f.result().get("_cache", "")).startswith("similar") else None)
            else:
                fut = self.pool().submit(PC.run_and_store, str(pc.dir), key, name, pc.neg_ttl_s, fn, *args,
                                         similar_dir=str(pc.similar_dir), obst=pc.obst)
            pc._count(name, "stores")
        else:
            fut = self.pool().submit(fn, *args)
        if key is not None:
            self._fut_key[fut] = key
        self.futs[fut] = (kind, payload, self.now())
        self._t_sub[fut] = time.monotonic()
        fut.add_done_callback(self._task_done)
        return fut

    # -- pool watchdog --------------------------------------------------------------------------
    def _task_done(self, f) -> None:
        self._last_done = time.monotonic()
        self._t_sub.pop(f, None)

    def _pool_processes(self) -> dict:
        pool = self._pool
        return dict(getattr(pool, "_processes", None) or {}) if isinstance(pool, cf.ProcessPoolExecutor) else {}

    def check_pool(self, pending=()) -> None:
        """PoolFailure when the pool cannot finish ``pending``: a worker process is gone (the executor's broken flag
        or a worker's exit code) or nothing finished for ``stall_s`` while tasks were in flight."""
        dead = {pid: p.exitcode for pid, p in self._pool_processes().items() if p.exitcode is not None}
        broken = getattr(self._pool, "_broken", False)
        if dead or broken:
            what = ", ".join(f"pid {pid} {_exit_note(c)}" for pid, c in sorted(dead.items())) or str(broken)
            raise PoolFailure(f"a U3 worker process died ({what}); {len(pending)} task(s) in flight, "
                              f"{self.n_workers} workers")
        now = time.monotonic()
        # stalled: nothing finished since the oldest in-flight task was submitted, for stall_s (a parent phase that
        # ran with nothing in flight does not count)
        oldest = min((self._t_sub.get(f, now) for f in pending), default=now)
        idle = now - max(self._last_done, oldest)
        if pending and idle > self.stall_s:
            raise PoolFailure(f"no U3 task finished for {idle:.0f} s with {len(pending)} in flight (a hung worker; "
                              f"UF_STALL_S={self.stall_s:.0f}), {self.n_workers} workers")

    def _wait(self, fs, return_when=cf.FIRST_COMPLETED, timeout: Optional[float] = None):
        """``cf.wait`` that never waits on a dead or hung pool: it wakes every ``poll_s`` and runs
        :meth:`check_pool` (PoolFailure). A future that failed with BrokenProcessPool is a PoolFailure too."""
        fs = list(fs)
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            step = self.poll_s if deadline is None else max(0.0, min(self.poll_s, deadline - time.monotonic()))
            done, pending = cf.wait(fs, timeout=step, return_when=return_when)
            for f in done:
                exc = f.exception() if not f.cancelled() else None
                if isinstance(exc, BrokenProcessPool):
                    raise PoolFailure(f"the U3 worker pool broke: {exc} ({self._death_note()})") from exc
            if not pending or (done and return_when == cf.FIRST_COMPLETED):
                return done, pending
            if deadline is not None and time.monotonic() >= deadline:
                return done, pending
            self.check_pool(pending)

    def _death_note(self) -> str:
        dead = {pid: p.exitcode for pid, p in self._pool_processes().items() if p.exitcode is not None}
        return ", ".join(f"pid {pid} {_exit_note(c)}" for pid, c in sorted(dead.items())) or "no exit code seen"

    def _as_completed(self, fs, timeout: Optional[float] = None):
        """``cf.as_completed`` on :meth:`_wait` (cf.TimeoutError past ``timeout``, PoolFailure on a dead / hung
        pool)."""
        pending = set(fs)
        deadline = None if timeout is None else time.monotonic() + timeout
        while pending:
            left = None if deadline is None else max(0.0, deadline - time.monotonic())
            done, pending = self._wait(pending, cf.FIRST_COMPLETED, left)
            if not done:
                raise cf.TimeoutError(f"{len(pending)} (of {len(fs)}) futures unfinished")
            for f in sorted(done, key=self._replay_order):
                self._consume(f)
                yield f

    def _within(self, jobs: dict, timeout_s: Optional[float]):
        """``as_completed`` over ``jobs`` with a deadline: candidates still running after ``timeout_s`` are dropped
        (left to finish in the pool, results ignored) so one slow OMPL solve does not hold up the decision."""
        try:
            for f in self._as_completed(list(jobs), timeout=timeout_s):
                # replaying a run (its journal): a result that run dropped at this deadline -- a cached one that ran
                # past it then, or one computed now that was dropped then (so never stored) -- is dropped again, so
                # the replay stays on that run's path
                if timeout_s is not None and f in self.futs and (
                        (f in self._hit_wall and self._hit_wall[f] > timeout_s)
                        or self._fut_key.get(f) in self._journal_dropped):
                    # a cached result whose task ran past this deadline when it was computed: dropped, as then
                    kind, k, t_sub = self.futs.pop(f)
                    self._drop_key(f)
                    self.events.append({"kind": kind, "t_submit": t_sub, "t_done": None,
                                        "dropped_after_s": timeout_s, "cached": True})
                    continue
                yield f
        except cf.TimeoutError:
            late = [f for f in jobs if f in self.futs and not f.done()]
            for f in late:
                kind, k, t_sub = self.futs.pop(f)
                self._drop_key(f)
                self.events.append({"kind": kind, "t_submit": t_sub, "t_done": None, "dropped_after_s": timeout_s,
                                    "variant": (jobs[f] or {}).get("name") if isinstance(jobs[f], dict) else None})
            for f in list(jobs):            # finished in the meantime: still usable
                if f in self.futs and f.done():
                    yield f

    def _split_gate_many(self, cell: str, checks: list) -> cf.Future:
        """A multi-step gate as one task per step (all at once), combined into one future with gate_many's
        result shape -- the checks are independent, so the program's longest check is the latency."""
        outer: cf.Future = cf.Future()
        subs = [self.pool().submit(W.gate_many, cell, [c]) for c in checks]
        left = [len(subs)]
        lock = threading.Lock()

        def done(_f):
            with lock:
                left[0] -= 1
                if left[0]:
                    return
            try:
                rs = [f.result() for f in subs]
                outer.set_result({"modes": [r["modes"][0] for r in rs], "reports": [r["reports"][0] for r in rs],
                                  "wall_s": round(sum(r["wall_s"] for r in rs), 3), "pid": rs[0].get("pid"),
                                  "split": len(rs)})
            except Exception as exc:  # noqa: BLE001
                outer.set_exception(exc)

        for f in subs:
            f.add_done_callback(done)
        return outer

    def record(self, kind: str, t_submit: float, res: Any, extra: Optional[dict] = None) -> None:
        ev = {"kind": kind, "t_submit": t_submit, "t_done": self.now()}
        if isinstance(res, dict):
            ev["wall_s"] = res.get("wall_s")
            ev["pid"] = res.get("pid")
        ev.update(extra or {})
        self.events.append(ev)

    # -- umarell helpers (parent session) --------------------------------------------------------
    def setup_parent(self) -> dict:
        """Open the umarell session in this process; install + self-check the kernels."""
        from umarell_fast.u2 import session as S
        S.setup(self.repo, self.cell, self.env, kernels="none")
        info = {"requested": self.kernels}
        if self.kernels != "none":
            # same kernels in this process (the parent runs IK predictions and MRTA); the self-check runs in a
            # worker, concurrently with the planning (see run)
            try:
                self._parent_kernels = (self.kernels, W.install_kernels(self.kernels))
            except Exception as exc:  # noqa: BLE001
                info["install_error"] = repr(exc)
                self.log(f"[uf] kernels {self.kernels!r} cannot be installed ({exc!r}) -> umarell's own kernels")
                self.kernels = "none"
        info["effective"] = self.kernels
        from agents.trajectory.tools.planning_tools import _cell_robots
        from agents.home_pose import HOME_JOINTS_FALLBACK, home_pose
        from agents.trajectory.tools import robot_kinematics as rk
        self.robot_cfg = _cell_robots(self.cell)
        self.robots = sorted(self.robot_cfg)
        self.home = {ns: list(map(float, home_pose(self.cell, ns, HOME_JOINTS_FALLBACK))) for ns in self.robots}
        self._models = {ns: rk._load_pinocchio_model(self.cell, ns)[0] for ns in self.robots}
        self.kernel_info = info
        return info

    def tcp(self, ns: str, q) -> np.ndarray:
        from agents.trajectory.tools.robot_kinematics import _fk_ee
        m = self._models[ns]
        cfg = self.robot_cfg[ns]
        return _fk_ee(m, m.createData(), np.asarray(q, float)[:m.nq], np.asarray(cfg["base_position"], float),
                      float(cfg.get("base_orientation_yaw", 0.0)))

    def call(self, name: str, **kw):
        from umarell_fast.u2 import session as S
        return S.call(name, **kw)

    def new_move(self, ns: str, op_id: str, xyz, kind: str = "via") -> Leg:
        self._n += 1
        return Leg(ns=ns, op_id=op_id, req=self.policy.move_request(ns, op_id, xyz), kind=kind)

    def facing_a1(self, ns: str, req: dict) -> float:
        """The A1 (rad) that turns arm ``ns`` toward ``req``'s weld: the A1 of HOME plus the angle, about the base,
        between where HOME's TCP is and the weld (FK, so the base yaw and the axis sign need no convention)."""
        home = np.asarray(self.home[ns], float)
        base = np.asarray(self.robot_cfg[ns]["base_position"], float)
        t = req["target"]
        w = np.array([float(t["x"]), float(t["y"])]) - base[:2]
        h = np.asarray(self.tcp(ns, home), float)[:2] - base[:2]
        ang = float(np.arctan2(h[0] * w[1] - h[1] * w[0], h @ w))
        q = home.copy()
        q[0] += 0.1
        h2 = np.asarray(self.tcp(ns, q), float)[:2] - base[:2]
        sign = 1.0 if (h[0] * h2[1] - h[1] * h2[0]) >= 0 else -1.0     # +A1 turns the TCP counter-clockwise?
        return float(home[0] + sign * ang)

    def stage_leg(self, ns: str, weld: Leg, seed: list) -> Leg:
        """A staging move before ``weld`` (workers.plan_stage): to the stand-off configuration ``seed`` leads to,
        when that branch's stroke into the weld is clear."""
        self._n += 1
        op_id = f"{weld.op_id}_STAGE{self._n}"
        av = np.asarray(weld.req.get("approach_vector", [0, 0, -1]), float)
        av = av / (np.linalg.norm(av) + 1e-9)
        t = weld.req["target"]
        off = float(weld.req.get("approach_offset_m") or self.policy.p.approach_offset_m)
        req = self.policy.move_request(ns, op_id, np.array([t["x"], t["y"], t["z"]]) - av * off)
        req.update(stage_for=copy.deepcopy(weld.req), seeds=[[float(v) for v in seed]])
        return Leg(ns=ns, op_id=op_id, req=req, kind="stage")

    def home_leg(self, ns: str, before_op: str) -> Leg:
        """A planned return to HOME (a move leg; workers.plan_home_return plans it) before ``before_op``."""
        self._n += 1
        op_id = f"{before_op}_HOME"
        req = self.policy.move_request(ns, op_id, self.tcp(ns, self.home[ns]))
        req["home_return"] = True
        return Leg(ns=ns, op_id=op_id, req=req, kind="home")

    def clone_leg(self, leg: Leg, suffix: Optional[str] = None) -> Leg:
        return Leg(ns=leg.ns, op_id=leg.op_id + (suffix or ""), req=copy.deepcopy(leg.req), kind=leg.kind,
                   src=leg.src)

    # -- sequences --------------------------------------------------------------------------------
    def seq(self, ns: str) -> list:
        return [st[ns] for st in self.steps if ns in st and st[ns].status != "dropped"]

    def expected_start(self, ns: str, i: int) -> tuple:
        """(start, prev_pose, firm) for leg i of arm ns: firm = based on planned legs only."""
        s = self.seq(ns)
        if i == 0:
            return self.home[ns], None, True
        prev = s[i - 1]
        if prev.status == "ok":
            return prev.end, prev.pose, True
        return prev.pred_end, prev.pose, False

    def chain_ctx(self, ns: str, prev: Optional[Leg], steps: Optional[list] = None) -> dict:
        """The chaining state a task planning from the end of ``prev`` (None: from HOME) is handed besides the
        start: ``prev_pose`` (the working pose the arm stands in) and, on umarell 76ea13a, ``welds`` / ``specs``
        (the weld poses / specified strokes the planner recorded for this arm's planned legs up to ``prev``)."""
        out: dict = {"prev_pose": prev.pose if prev is not None else None}
        if prev is None:
            return out
        welds, specs = {}, {}
        for st in (self.steps if steps is None else steps):
            l = st.get(ns)
            if isinstance(l, Leg) and l.status == "ok" and l.rec:
                if l.rec.get("weld_pose"):
                    welds[l.op_id] = l.rec["weld_pose"]
                if l.rec.get("op_spec"):
                    specs[l.op_id] = l.rec["op_spec"]
            if l is prev:
                break
        if welds:
            out["welds"] = welds
        if specs:
            out["specs"] = specs
        return out

    def predict(self, ns: str, start, leg: Leg) -> Optional[list]:
        if leg.req.get("home_return"):
            return list(self.home[ns])          # a return to HOME ends there
        if leg.req.get("stage_for"):
            return leg.end or leg.pred_end      # a staging move ends at its branch's stand-off (as last planned)
        t = time.monotonic()
        pc = self.plan_cache()
        args = (self.cell, ns, list(start), leg.req, self.policy.p.approach_offset_m)
        hit, e = pc.get("predict_end", key := pc.task_key("predict_end", args)) if pc is not None else (False, None)
        if not hit:
            e = W.predict_end(*args)
            if pc is not None:
                pc.put("predict_end", key, e)
        self.events.append({"kind": "predict", "t_submit": self.now(), "t_done": self.now(),
                            "wall_s": round(time.monotonic() - t, 4), "pid": os.getpid(), "op": leg.op_id})
        return e

    # -- phase: plan every leg ---------------------------------------------------------------------
    def schedule(self) -> None:
        """Bring every arm's chain up to date: (re)submit each leg whose expected start is known and differs from
        the start it is planned / being planned from; refresh predictions downstream."""
        for ns in self.robots:
            s = self.seq(ns)
            for i, leg in enumerate(s):
                start, prev_pose, firm = self.expected_start(ns, i)
                if start is None:
                    break                       # unknown start (prediction failed): wait for the leg before
                if leg.status == "waiting" and gap(leg.start, start) <= TOL_RAD \
                        and leg.ctx_pose == pose_key(prev_pose):
                    if firm:                    # the start it failed from is now real: the failure stands
                        leg.firm = True
                        self.start_recovery(leg)
                    continue
                if leg.status in ("ok", "planning", "recovering") and gap(leg.start, start) <= TOL_RAD \
                        and (leg.ctx_pose is None or leg.ctx_pose == pose_key(prev_pose)):
                    if leg.status != "ok" and leg.pred_end is None:
                        leg.pred_end = self.predict(ns, start, leg)
                    continue
                # (re)plan from the new start
                if leg.status == "recovering":
                    self.recovering.pop(leg, None)
                leg.version += 1
                leg.status, leg.start, leg.traj, leg.end = "planning", list(start), None, None
                leg.pred_end = self.predict(ns, start, leg)
                leg.ctx_pose = pose_key(prev_pose)
                leg.firm = bool(firm)
                spec = {"ns": ns, "op_id": leg.op_id, "req": leg.req, "start": list(start),
                        **self.chain_ctx(ns, s[i - 1] if i > 0 else None)}
                self.submit("leg", (leg, leg.version), W.plan_chain, self.cell, [spec], self.policy.call_kwargs())

    def on_leg(self, leg: Leg, version: int, res: dict, t_sub: float) -> None:
        self.record("leg", t_sub, res, {"op": leg.op_id, "ok": res.get("ok"), "stale": version != leg.version})
        if version != leg.version or leg.status == "dropped":
            return
        r = res["legs"][0]
        leg.planning_time_s += float(r.get("planning_time_s") or 0.0)
        if r["ok"]:
            self._accept(leg, r)
            leg.attempts.append("plan")
        else:
            leg.failures = r.get("failures") or []
            leg.attempts.append("plan:" + ",".join(str(f.get("reason_code")) for f in leg.failures))
            if not leg.firm:
                # planned from the previous leg's PREDICTED end: wait for its real end (schedule re-plans when it
                # differs, recovers when it is the same) instead of recovering from a start that may never be
                leg.status = "waiting"
                return
            self.start_recovery(leg)

    def _accept(self, leg: Leg, r: dict) -> None:
        if r.get("req"):
            leg.req = r["req"]                  # planned with a pinned weld pose (after a staging move)
        leg.status, leg.traj, leg.end = "ok", r["traj"], r["end"]
        leg.approach = r.get("approach")
        leg.set_rec(r)
        if gap(leg.end, leg.pred_end) > TOL_RAD:
            self.log(f"[uf] {leg.ns}/{leg.op_id}: real end differs from the IK prediction by "
                     f"{gap(leg.end, leg.pred_end):.2e} rad -> re-planning its dependants")
        leg.pred_end = leg.end

    def start_recovery(self, leg: Leg) -> None:
        cands = self.policy.recovery_candidates(self, leg)
        leg.status = "recovering"
        if not cands:
            self.recovery_exhausted(leg)
            return
        grp = {"version": leg.version, "cands": cands, "done": {}, "t0": self.now()}
        self.recovering[leg] = grp
        start = leg.start
        s = self.seq(leg.ns)
        i = s.index(leg)
        ctx = self.chain_ctx(leg.ns, s[i - 1] if i > 0 else None)
        for c in cands:
            specs = [{"ns": l.ns, "op_id": l.op_id, "req": l.req} for l in c.legs]
            specs[0].update(start=list(start), **ctx)
            self.submit("cand", (leg, leg.version, c), W.plan_chain, self.cell, specs, self.policy.call_kwargs())

    def on_cand(self, leg: Leg, version: int, cand: Candidate, res: dict, t_sub: float) -> None:
        self.record("cand", t_sub, res, {"op": leg.op_id, "cand": cand.name, "ok": res.get("ok")})
        grp = self.recovering.get(leg)
        if grp is None or grp["version"] != version or leg.version != version:
            return
        grp["done"][cand.name] = res
        # highest-priority success whose betters have all finished
        for c in sorted(grp["cands"], key=lambda c: c.priority):
            r = grp["done"].get(c.name)
            if r is None:
                return                      # a better candidate is still running
            if r.get("ok"):
                self.apply_candidate(leg, c, r)
                return
        self.recovery_exhausted(leg)        # every candidate finished, none planned

    def apply_candidate(self, leg: Leg, cand: Candidate, res: dict) -> None:
        self.recovering.pop(leg, None)
        planned = res["legs"]
        *pre, last = list(zip(cand.legs, planned))
        leg.attempts.append(f"recovered:{cand.name}")
        leg.req = cand.legs[-1].req
        si = next(k for k, st in enumerate(self.steps) if st.get(leg.ns) is leg)
        for tmpl, r in reversed(pre):      # vias get their own step right before the weld's step
            tmpl.start = r.get("start")
            tmpl.pred_end = r["end"]
            tmpl.status, tmpl.traj, tmpl.end, tmpl.approach = "ok", r["traj"], r["end"], r.get("approach")
            tmpl.set_rec(r)
            tmpl.attempts.append(f"recovery:{cand.name}")
            self.steps.insert(si, {leg.ns: tmpl})
        leg.start = last[1].get("start")
        if pre:
            leg.ctx_pose = pose_key(pre[-1][0].pose)   # it now chains from its via
        self._accept(leg, last[1])
        self.log(f"[uf] {leg.ns}/{leg.op_id} recovered by {cand.name}")

    def recovery_exhausted(self, leg: Leg) -> None:
        self.recovering.pop(leg, None)
        tried = sorted({leg.ns} | set(getattr(leg, "_tried", set())))
        leg._tried = set(tried)
        target = None
        if self.policy.allow_reallocation(leg) and leg.src:
            op = self.ops[leg.src]
            order = [{"robot_ns": ns, "operation_ids": [l.op_id for l in self.seq(ns)]} for ns in self.robots]
            ra = self.call("reallocate_operation", cell_id=self.cell, operation=op, tried_robots=tried,
                           robots=order)
            if ra.get("success"):
                target = ra["to_robot"]
            leg.attempts.append(f"reallocate:{ra.get('to_robot') or ra.get('reason_code')}")
        # remove it from this arm (its dependants now chain from the leg before it)
        for st in self.steps:
            if st.get(leg.ns) is leg:
                del st[leg.ns]
        self.steps = [st for st in self.steps if any(isinstance(v, Leg) for v in st.values())]
        if target:
            new = Leg(ns=target, op_id=leg.op_id, req=self.policy.weld_request(target, self.ops[leg.src]),
                      kind="weld", src=leg.src)
            new._tried = set(tried)
            new.attempts = list(leg.attempts)
            self.steps.append({target: new})          # appended as the arm's last operation (umarell's rule)
            self.reallocated.append({"op": leg.op_id, "from": leg.ns, "to": target})
            self.log(f"[uf] {leg.op_id}: {leg.ns} failed every candidate -> reallocated to {target}")
        else:
            leg.status = "dropped"
            self.unreachable.append({"op": leg.op_id, "tried": tried, "attempts": leg.attempts,
                                     "failures": [{k: f.get(k) for k in ("reason_code", "error")}
                                                  for f in leg.failures]})
            self.log(f"[uf] {leg.op_id}: UNREACHABLE (tried {tried})")

    def drain(self, handlers: dict) -> None:
        """Process completions until nothing is in flight."""
        while self.futs:
            done, _ = self._wait(list(self.futs), return_when=cf.FIRST_COMPLETED)
            # deterministic order: plan cache hits (finished together) by their tasks' recorded run time, then the rest
            for fut in sorted(done, key=self._replay_order):
                self._consume(fut)
                kind, payload, t_sub = self.futs.pop(fut)
                try:
                    res = fut.result()
                except Exception as exc:  # noqa: BLE001 -- a worker crash fails that task only
                    res = {"ok": False, "error": repr(exc), "legs": [{"ok": False, "failures": [
                        {"reason_code": "worker_error", "error": repr(exc)}]}]}
                handlers[kind](payload, res, t_sub)
            self.schedule()

    def plan_all_legs(self) -> None:
        self.schedule()
        self.drain({"leg": lambda p, r, t: self.on_leg(p[0], p[1], r, t),
                    "cand": lambda p, r, t: self.on_cand(p[0], p[1], p[2], r, t)})

    # -- phase: steps / gate ----------------------------------------------------------------------
    def step_legs(self, st: dict) -> dict:
        """The planned legs of a step (keys starting with ``_`` are step metadata: delays, gate notes)."""
        return {ns: l for ns, l in st.items() if isinstance(l, Leg) and l.status == "ok"}

    def bystanders_per_step(self, steps: Optional[list] = None) -> list:
        """The ``bystanders`` argument the assembler will pass for each step (arms that moved before and are not
        in the step, at the end of their latest leg)."""
        at, out = {}, []
        for st in (self.steps if steps is None else steps):
            legs = self.step_legs(st)
            out.append({ns: q for ns, q in at.items() if ns not in legs})
            for ns, l in legs.items():
                at[ns] = l.end
        return out

    def gate_all(self, only: Optional[set] = None) -> dict:
        bys = self.bystanders_per_step()
        pend = {}
        for k, st in enumerate(self.steps):
            if only is not None and k not in only:
                continue
            legs = self.step_legs(st)
            if len(legs) > 1 or bys[k]:
                delays = {ns: float(st.get("_delay", {}).get(ns, 0.0)) for ns in legs} if "_delay" in st else {}
                f = self.submit("gate", k, W.gate_step, self.cell, {ns: l.traj for ns, l in legs.items()},
                                delays, bys[k], True)
                pend[f] = k
        out = {}
        for f in self._as_completed(list(pend)):
            kind, k, t_sub = self.futs.pop(f)
            res = f.result()
            self.record("gate", t_sub, res, {"step": k, "mode": res.get("mode")})
            self.steps[k]["_gate"] = {"mode": res.get("mode"), "report": res.get("report")}
            out[k] = res
        return out

    def repair_steps(self) -> dict:
        """Gate every step at once, then

        * serialised steps: improvement candidates for all of them at once (start delays, coupled re-plans in
          both orders) -- they keep every endpoint, so no other step's verdict changes;
        * blocked steps, earliest first (a repair changes what later steps see): coupled re-plans and park moves
          for every arm involved, all candidates at once, each judged by the gate on the steps it changes; then the
          dependants are re-planned and every step is re-gated at once.
        """
        verdicts = self.gate_all()
        verdicts.update(self._improve({k: v for k, v in verdicts.items() if v.get("mode") == "SEQUENTIAL"}))
        repairs = 0
        # no-progress guard: a conflict is the blocked step's legs + the arms the gate blames. A repair that brings
        # the same conflict back (a handoff that moved it, a park that did not clear it) made no progress: the next
        # attempt excludes what was chosen before, and after max_same_conflict attempts the conflict is given up
        # (it is never repaired the same way twice, so the loop cannot spin on one conflict).
        seen: dict = {}
        dead: set = set()
        self.no_progress: list = []
        while repairs < self.policy.p.max_repairs:
            blocked = sorted(k for k, v in verdicts.items()
                             if v.get("mode") is None and not self.steps[k].get("_unfixable"))
            blocked = [k for k in blocked if self._conflict_sig(k) not in dead]
            if not blocked:
                break
            k = blocked[0]
            sig = self._conflict_sig(k)
            tried = seen.setdefault(sig, [])
            if len(tried) >= self.policy.p.max_same_conflict:
                dead.add(sig)
                self.steps[k]["_unfixable"] = True
                self.no_progress.append({"legs": list(sig[0]), "arms": list(sig[1]), "tried": list(tried)})
                self.log(f"[uf] step {k}: conflict {sig} is back after {tried} -> no progress, giving it up")
                continue
            if tried:
                self.no_progress.append({"legs": list(sig[0]), "arms": list(sig[1]), "tried": list(tried)})
                self.log(f"[uf] step {k}: same conflict as before {tried[-1]} (no progress) -> excluding "
                         f"{tried} and trying something else")
            repairs += 1
            ex = set(tried)
            chosen = self._repair_blocked(k, exclude=ex) or self._handoff(k, exclude=ex)
            if not chosen:
                self.steps[k]["_unfixable"] = True
                dead.add(sig)
                self.log(f"[uf] step {k}: blocked and no candidate repairs it")
                continue
            tried.append(chosen)
            self.plan_all_legs()             # parks / changed ends -> re-plan the dependants
            verdicts = self.gate_all()
            verdicts.update(self._improve({k2: v for k2, v in verdicts.items()
                                           if v.get("mode") == "SEQUENTIAL" and "_info" not in self.steps[k2]}))
        self.verdicts = verdicts
        self.n_repairs = repairs
        return verdicts

    # -- conflicts: which arms the gate blames ------------------------------------------------------
    @staticmethod
    def _worst_pair(rep: Optional[dict]) -> list:
        """The arm pair a gate report names as colliding (``parallel.worst_pair`` when the parallel check failed;
        a step refused for another reason -- a static breach -- names no pair)."""
        par = (rep or {}).get("parallel") or {}
        return list(par.get("worst_pair") or []) if par.get("collision_free") is False else []

    @classmethod
    def _implicated(cls, rep: Optional[dict], arms) -> set:
        """The arms a blocked gate report blames: its colliding pair and its blocked arms; when it names none (a
        report without that detail), every arm of the step (``arms``: moving + standing by)."""
        out = set(cls._worst_pair(rep)) | set((rep or {}).get("blocked_arms") or [])
        return out or set(arms)

    def _conflict_sig(self, k: int) -> tuple:
        """A blocked step's conflict: its legs (arm, operation) and the arms the gate blames."""
        st = self.steps[k]
        legs = self.step_legs(st)
        rep = (st.get("_gate") or {}).get("report")
        arms = set(legs) | set(self.bystanders_per_step()[k])
        return (tuple(sorted((ns, l.op_id) for ns, l in legs.items())), tuple(sorted(self._implicated(rep, arms))))

    def conflict_pairs(self, k: int, arms: list) -> list:
        """The pairs of arms to serialise for blocked step k (``arms``: moving + standing by, sorted): the pair the
        gate reports as colliding (:meth:`_worst_pair`); without one, every pair of a blocked (else moving) arm with
        another arm. Two arms: always the one pair."""
        rep = (self.steps[k].get("_gate") or {}).get("report") or {}
        wp = sorted({a for a in self._worst_pair(rep) if a in arms})
        if len(wp) == 2:
            return [tuple(wp)]
        if len(arms) == 2:
            return [tuple(arms)]
        mov = [a for a in (rep.get("blocked_arms") or []) if a in arms] or sorted(self.step_legs(self.steps[k]))
        out: list = []
        for a in mov:
            for b in arms:
                p = tuple(sorted((a, b)))
                if a != b and p not in out:
                    out.append(p)
        return out or [tuple(arms[:2])]

    def deferred_steps(self, k: int, ns: str) -> list:
        """The step list with arm ``ns``'s legs from step ``k`` on shifted one step later (pairing changes, legs and
        chain do not). Steps from ``k`` on lose their delays (they were for the old pairs)."""
        head = self.steps[:k]
        tail = [{o: l for o, l in st.items() if isinstance(l, Leg) and o != ns} for st in self.steps[k:]]
        xs = [st[ns] for st in self.steps[k:] if isinstance(st.get(ns), Leg)]
        tail = [st for st in tail if st] or []
        # keep the other arms' steps in order; X's legs start one step later
        out = list(head)
        if not tail:
            return out + [{ns: x} for x in xs]
        out.append(tail[0])
        j = 0
        for st in tail[1:]:
            st = dict(st)
            if j < len(xs) and ns not in st:
                st[ns] = xs[j]; j += 1
            out.append(st)
        out += [{ns: x} for x in xs[j:]]
        return out

    def compact(self) -> int:
        """Merge single-arm steps back into parallel ones where umarell's gate allows it: for a single-arm step i
        of arm A, the next leg of another arm B, if it sits alone in a later step j and B has nothing in between,
        moves up into step i (B's order and chain are unchanged; only the timing changes). Every position's
        candidate is gated at once (steps i..j of the hypothetical program); the earliest valid merges that do
        not overlap are applied; repeat until none is valid. Returns the number of merges."""
        merges = 0
        rejected: set = set()
        for _ in range(64):
            cands = []
            for i, st in enumerate(self.steps):
                li = self.step_legs(st)
                if len(li) != 1:
                    continue
                (a,) = li
                for j in range(i + 1, len(self.steps)):
                    lj = self.step_legs(self.steps[j])
                    others = [o for o in lj if o != a]
                    if not others:
                        continue
                    b = others[0]
                    if len(lj) == 1 and not any(b in self.step_legs(self.steps[m]) for m in range(i + 1, j)):
                        key = (lj[b].op_id, li[a].op_id)
                        if key not in rejected:
                            cands.append((i, j, b, key))
                    break                          # only B's next leg (the first later step with another arm)
            if not cands:
                break
            jobs = {}
            for i, j, b, key in cands:
                hyp = [dict(x) for x in self.steps]
                hyp[i][b] = hyp[j].pop(b)
                for x in (hyp[i], hyp[j]):
                    x.pop("_delay", None)
                hyp = [x for x in hyp if self.step_legs(x)]
                hb = self.bystanders_per_step(hyp)
                jj = min(j, len(hyp))
                checks = [({o: l.traj for o, l in self.step_legs(hyp[m]).items()}, {}, hb[m]) for m in range(i, jj)]
                jobs[self.submit("gate_many", i, W.gate_many, self.cell, checks)] = (i, j, key, hyp)
            ok = []
            for f in self._as_completed(list(jobs)):
                kind, _, t_sub = self.futs.pop(f)
                i, j, key, hyp = jobs[f]
                res = f.result()
                modes = res.get("modes") or [None]
                self.record("gate", t_sub, res, {"step": i, "variant": f"merge_{key[0]}_into_{key[1]}",
                                                 "modes": modes})
                if all(m == "PARALLEL" for m in modes[:1]) and all(m is not None for m in modes):
                    for m, md in zip(range(i, len(hyp)), modes):
                        hyp[m]["_mode"] = md          # the verdicts the merge is accepted with
                    ok.append((i, j, key, hyp))
                else:
                    rejected.add(key)
            if not ok:
                break
            i, j, key, hyp = min(ok, key=lambda x: x[0])   # earliest merge; the rest are re-evaluated next round
            self.steps = hyp
            self.steps[i].setdefault("_info", {})["merged"] = f"{key[0]} moved up next to {key[1]}"
            merges += 1
        self.n_merges = merges
        return merges

    # -- N-arm overlap: each arm's chain as early as the gates allow ------------------------------
    def max_concurrent(self) -> int:
        """The task's ``extensions.strategy.max_concurrent_robots`` (the backend defaults it to the robot count),
        else the robot count."""
        n = getattr(self, "max_concurrent_robots", None)
        try:
            n = int(n) if n is not None else len(self.robots)
        except (TypeError, ValueError):
            n = len(self.robots)
        return max(1, min(n, len(self.robots)))

    def _mode(self, st: dict) -> Optional[str]:
        """A step's current gate verdict: the last one recorded on it (repair_steps / compaction / overlap), else
        its ``_gate``; a step nobody gated (one arm, no bystander) runs as is."""
        if "_mode" in st:
            return st["_mode"]
        g = st.get("_gate")
        if g is not None:
            return g.get("mode")
        return "PARALLEL"

    def _seed_modes(self) -> None:
        """The verdicts repair_steps ended with (improvements included, which ``_gate`` does not show) onto the
        step dicts, so they travel with the steps when compaction reorders the list."""
        for k, v in (getattr(self, "verdicts", None) or {}).items():
            if 0 <= k < len(self.steps) and isinstance(v, dict) and "_mode" not in self.steps[k]:
                self.steps[k]["_mode"] = v.get("mode")

    def program_s(self, steps: Optional[list] = None) -> float:
        """The program's duration as the step structure gives it: per step the longest leg (+ its start delay) when
        the step runs PARALLEL, the sum of its legs otherwise."""
        tot = 0.0
        for st in (self.steps if steps is None else steps):
            legs = self.step_legs(st)
            if legs:
                tot += self._step_dur(self._mode(st) or "SEQUENTIAL", {ns: l.traj for ns, l in legs.items()},
                                      st.get("_delay") or {})
        return round(tot, 3)

    def overlap(self) -> int:
        """Merge non-conflicting steps across arms (3+ arms): MRTA, the allocation fix, reallocations (appended at
        an arm's end) and the handoffs leave steps of two arms while the others wait, although they work at the
        other end of the workpiece. Here every arm keeps its own leg order and chain; only WHEN a leg runs changes.

        A candidate moves planned legs of step j up into an earlier step i: one arm B's leg (B has nothing in steps
        i..j-1, so the leg still starts where B stands: the chain is unchanged), or the whole step j (every arm of it
        has nothing in i..j-1 -- a pair whose legs only pay off together), if step i then has at most
        ``max_concurrent_robots`` arms. What a move changes is steps i..j only: the legs move in i, their arms stand
        at the legs' ends instead of their previous ones in i+1..j-1, and in j they are bystanders (or j
        disappears). Those steps of the hypothetical program are gated with umarell's assembler gate, with the
        bystanders and start delays the assembler will use, and the move is accepted when

        * step i clears PARALLEL (a merge the assembler would serialise is pointless),
        * no other changed step is refused, and none that ran PARALLEL is serialised,
        * the program does not get longer (:meth:`program_s`; moving a long leg next to short ones does not pay);
          moves that shorten it go first, a move that only brings a chain forward waits for a round without one.

        All candidates (per leg and per step: the earliest untried position) are gated at once; the accepted moves
        are applied, as many as touch disjoint step ranges (their verdicts stay valid); then the next round. A step therefore only waits for the arms it conflicts with. Returns the number of moves."""
        maxc = self.max_concurrent()
        self._seed_modes()
        rejected: set = set()
        moves = 0
        t_end = time.monotonic() + float(self.policy.p.overlap_budget_s)
        self.overlap_log: list = []

        def last_before(steps: list, ns: str, j: int) -> int:
            return max((m for m in range(j) if isinstance(steps[m].get(ns), Leg)), default=-1)

        for _ in range(int(self.policy.p.overlap_rounds)):
            if time.monotonic() > t_end:
                self.log("[uf] overlap: time budget used up")
                break
            steps = self.steps
            cands, seen = [], set()
            for j, st in enumerate(steps):
                legs = self.step_legs(st)
                if not legs:
                    continue
                groups = [(ns,) for ns in sorted(legs)]
                if len(legs) > 1:
                    groups.append(tuple(sorted(legs)))
                for arms in groups:
                    if any(not isinstance(st.get(ns), Leg) or st[ns].status != "ok" for ns in arms):
                        continue
                    lo = max(last_before(steps, ns, j) for ns in arms) + 1
                    for i in range(lo, j):
                        tgt = self.step_legs(steps[i])
                        if not tgt or len(tgt) + len(arms) > maxc or steps[i].get("_unfixable"):
                            continue
                        key = (tuple((ns, st[ns].op_id) for ns in arms),
                               tuple(sorted(l.op_id for l in tgt.values())))
                        if key in rejected or key in seen:
                            continue
                        seen.add(key)
                        cands.append((i, j, arms, key))
                        break
            if not cands:
                break
            base_s = self.program_s(steps)
            jobs = {}
            for i, j, arms, key in cands:
                hyp = [dict(x) for x in steps]
                for ns in arms:
                    hyp[i][ns] = hyp[j].pop(ns)
                if "_delay" in hyp[j]:
                    hyp[j]["_delay"] = {o: d for o, d in hyp[j]["_delay"].items() if o not in arms}
                last = j if self.step_legs(hyp[j]) else j - 1
                hyp = [x for x in hyp if self.step_legs(x)]
                hb = self.bystanders_per_step(hyp)
                checks = []
                for m in range(i, last + 1):
                    legs = self.step_legs(hyp[m])
                    dl = {o: float(d) for o, d in (hyp[m].get("_delay") or {}).items() if o in legs and d}
                    checks.append(({o: l.traj for o, l in legs.items()}, dl, hb[m]))
                jobs[self.submit("gate_many", i, W.gate_many, self.cell, checks)] = (i, j, last, arms, key, hyp)
            ok = []
            for f in self._as_completed(list(jobs)):
                kind, _, t_sub = self.futs.pop(f)
                i, j, last, arms, key, hyp = jobs[f]
                res = f.result()
                modes = list(res.get("modes") or [])
                reps = list(res.get("reports") or [None] * len(modes))
                name = "+".join(f"{ns}/{op}" for ns, op in key[0])
                self.record("gate", t_sub, res, {"step": i, "variant": f"overlap_{name}_into_{i}", "modes": modes})
                why = None
                if len(modes) != last - i + 1:
                    why = "gate"
                elif modes[0] != "PARALLEL":
                    why = f"step {i}: {modes[0]}"
                else:
                    for m, md in zip(range(i + 1, last + 1), modes[1:]):
                        if md is None or (self._mode(steps[m]) == "PARALLEL" and md != "PARALLEL"):
                            why = f"step {m}: {self._mode(steps[m])} -> {md}"
                            break
                gain = None
                if why is None:
                    for m, md in zip(range(i, last + 1), modes):
                        hyp[m]["_mode"] = md
                    new_s = self.program_s(hyp)
                    gain = round(base_s - new_s, 3)
                    if gain < -1e-6:
                        why = f"no gain ({base_s} -> {new_s} s)"
                self.overlap_log.append({"legs": name, "into": i, "from": j, "modes": modes, "gain_s": gain,
                                         "rejected": why})
                if why is not None:
                    rejected.add(key)
                    continue
                ok.append({"i": i, "j": j, "last": last, "arms": arms, "modes": modes, "reps": reps, "gain": gain,
                           "name": name})
            if not ok:
                continue                       # every candidate refused: the next round tries later positions
            # moves that shorten the program first (biggest gain, then earliest); a move that only brings an arm's
            # leg forward without shortening the program yet (its step-mate still sets the step's length) is taken
            # only in a round without a shortening one -- it frees the arm's next leg to move up the round after.
            # Every move brings legs strictly earlier, so the rounds end.
            pos = [c for c in ok if c["gain"] > 1e-6]
            chosen, used = [], []
            for c in sorted(pos or ok, key=lambda c: (-c["gain"], c["i"])):
                if all(c["j"] < a or c["i"] > b for a, b in used):
                    chosen.append(c)
                    used.append((c["i"], c["j"]))
            # apply the latest first: removing an emptied step j only shifts indices beyond every earlier range
            for c in sorted(chosen, key=lambda c: -c["i"]):
                i, j = c["i"], c["j"]
                src, dst = self.steps[j], dict(self.steps[i])
                for ns in c["arms"]:
                    leg = src.pop(ns)
                    dst[ns] = leg
                    leg.attempts.append(f"overlap:step{j}->{i}")
                if "_delay" in src:
                    src["_delay"] = {o: d for o, d in src["_delay"].items() if o not in c["arms"]}
                info = dict(dst.get("_info") or {})
                info["overlapped"] = list(info.get("overlapped", [])) + [f"{c['name']} from step {j}"]
                dst["_info"] = info
                self.steps[i] = dst
                for m, md, rp in zip(range(i, c["last"] + 1), c["modes"], c["reps"]):
                    self.steps[m]["_mode"] = md
                    self.steps[m]["_gate"] = {"mode": md, "report": rp}
                if not self.step_legs(src):
                    del self.steps[j]
                moves += 1
                self.log(f"[uf] overlap: {c['name']} runs in step {i} (was {j}), -{c['gain']} s")
        self.n_overlaps = moves
        return moves

    def _handoff(self, k: int, exclude: Optional[set] = None) -> Optional[str]:
        """Last repair of a blocked step k (what Opus did in track L when two arms could not share the zone): one
        arm F finishes all its remaining legs alone while the other arm S waits (optionally parked first), F then
        parks (optionally), and S does its remaining legs. Both orders, every park option of each arm (and no
        park), all planned at once (S's remaining chain is re-planned from its park in one task), every resulting
        step gated at once; the policy picks the shortest acceptable program. Returns the chosen variant's name
        (None: nothing acceptable).

        F and S are the pair of arms the gate blames for the conflict (:meth:`conflict_pairs`; with two arms, the
        two). With 3+ arms only that pair is serialised: every other arm keeps its legs from step k on (same legs,
        same chain, original grouping) and runs them in parallel with F's and S's steps (variant ``..._par``), or,
        if that does not clear, after them (the plain variant), so no weld leg is lost. A variant is acceptable
        when every step with an F or S leg clears and no step left blocked blames F or S (a conflict of the other
        arms among themselves is left to the next round of repair_steps); the steps are judged with every arm
        where it really stands, so a handoff that only moves the conflict to a later step is refused."""
        exclude = exclude or set()
        st = self.steps[k]
        arms = sorted(set(self.step_legs(st)) | set(self.bystanders_per_step()[k]))
        if len(arms) < 2:
            return None
        pairs = self.conflict_pairs(k, arms)
        head = self.steps[:k]
        involved = sorted({a for p in pairs for a in p})
        rem = {ns: [x[ns] for x in self.steps[k:] if isinstance(x.get(ns), Leg) and x[ns].status == "ok"]
               for ns in involved}
        before = {}
        for ns in involved:
            prior = [x[ns] for x in head if isinstance(x.get(ns), Leg) and x[ns].status == "ok"]
            before[ns] = prior[-1] if prior else None
        npark = self.policy.p.handoff_parks
        orders = []
        for p in pairs:
            for o in (tuple(p), tuple(p[::-1])):
                if o not in orders:
                    orders.append(o)
        jobs = {}
        s_done: set = set()
        f_done: set = set()
        for order in orders:
            F, S_ = order
            # S: optional park, then its remaining legs re-planned as one chain (planned once per arm: the same
            # jobs serve every order where it is S)
            if S_ not in s_done:
                s_done.add(S_)
                s_from = before[S_]
                s_start = s_from.end if s_from else self.home[S_]
                s_ctx = self.chain_ctx(S_, s_from)
                s_opts = [None] + self.policy.park_targets(self, S_, s_start)[:npark]
                for sp in s_opts:
                    specs = []
                    park = None
                    if sp is not None:
                        park = self.new_move(S_, f"{S_.upper()}_PARK{self._n}", sp[1], kind="park")
                        specs.append({"ns": S_, "op_id": park.op_id, "req": park.req})
                    specs += [{"ns": S_, "op_id": l.op_id, "req": l.req} for l in rem[S_]]
                    if not specs:
                        continue
                    specs[0].update(start=list(s_start), **s_ctx)
                    jobs[self.submit("handoff", k, W.plan_chain, self.cell, specs, self.policy.call_kwargs())] = \
                        {"arm": S_, "role": "S", "park": park, "name": sp[0] if sp else "none"}
            # F: optional park after its last remaining leg
            if F not in f_done:
                f_done.add(F)
                f_last = rem[F][-1] if rem[F] else before[F]
                if f_last is not None:
                    for fp in self.policy.park_targets(self, F, f_last.end)[:npark]:
                        park = self.new_move(F, f"{F.upper()}_PARK{self._n}", fp[1], kind="park")
                        spec = {"ns": F, "op_id": park.op_id, "req": park.req, "start": f_last.end,
                                **self.chain_ctx(F, f_last)}
                        jobs[self.submit("handoff", k, W.plan_chain, self.cell, [spec],
                                         self.policy.call_kwargs())] = \
                            {"arm": F, "role": "F", "park": park, "name": fp[0]}
        res_by = {}
        for f in self._as_completed(list(jobs)):
            kind, _, t_sub = self.futs.pop(f)
            var = jobs[f]
            res = f.result()
            self.record(kind, t_sub, res, {"step": k, "variant": f"{var['role']}:{var['arm']}:{var['name']}",
                                           "ok": res.get("ok")})
            if res.get("ok"):
                res_by.setdefault((var["role"], var["arm"]), []).append((var, res))
        planned = {}
        for order in orders:
            F, S_ = order
            if res_by.get(("S", S_)):
                planned[order] = {"S": res_by[("S", S_)], "F": [None] + res_by.get(("F", F), [])}
        gate_jobs = {}
        for order, d in planned.items():
            F, S_ = order
            # every other arm (not F / S, involved in step k or not) keeps its legs from step k on, in their
            # original grouping: a handoff must never lose a weld leg
            others = [{ns: l for ns, l in x.items() if isinstance(l, Leg) and ns not in (F, S_)}
                      for x in self.steps[k:]]
            others = [x for x in others if x]
            for svar, sres in d["S"]:
                if any(r.get("approach") and not r["approach"].get("matches") for r in sres["legs"]):
                    continue
                for fv in d["F"]:
                    rs = list(sres["legs"])
                    fs: list = []
                    if svar["park"] is not None:
                        # a fresh Leg per variant (the same planned park serves several variants)
                        fs.append({S_: self._mk_leg(self.clone_leg(svar["park"]), rs.pop(0))})
                    s_legs = []
                    for tmpl, r in zip(rem[S_], rs):
                        nl = self.clone_leg(tmpl)
                        nl.attempts = list(tmpl.attempts) + ["handoff"]
                        s_legs.append(self._mk_leg(nl, r))
                    fs += [{F: l} for l in rem[F]]
                    if fv is not None:
                        fs.append({F: self._mk_leg(self.clone_leg(fv[0]["park"]), fv[1]["legs"][0])})
                    fs += [{S_: l} for l in s_legs]
                    name = f"handoff_{F}_first_S{svar['name']}_F{fv[0]['name'] if fv else 'none'}"
                    layouts = [("", list(head) + fs + [dict(x) for x in others])]
                    if others:
                        par = [dict(x) for x in fs]
                        for i, g in enumerate(others):
                            if i < len(par):
                                par[i].update(g)
                            else:
                                par.append(dict(g))
                        layouts.insert(0, ("_par", list(head) + par))
                    for suffix, hyp in layouts:
                        if name + suffix in exclude:
                            continue
                        hb = self.bystanders_per_step(hyp)
                        checks = [({o: l.traj for o, l in self.step_legs(hyp[i]).items()}, {}, hb[i])
                                  for i in range(k, len(hyp))]
                        gate_jobs[self.submit("gate_many", k, W.gate_many, self.cell, checks)] = \
                            {"name": name + suffix, "hyp": hyp, "pair": (F, S_)}
        cands = []
        htried: list = []
        for f in self._as_completed(list(gate_jobs)):
            kind, _, t_sub = self.futs.pop(f)
            var = gate_jobs[f]
            res = f.result()
            modes = res.get("modes") or [None]
            reps = res.get("reports") or [{} for _ in modes]
            self.record("gate", t_sub, res, {"step": k, "variant": var["name"], "modes": modes})
            pair = set(var["pair"])
            ok = len(modes) == len(var["hyp"]) - k
            left_blocked = 0
            for i, (m, r) in enumerate(zip(modes, reps)):
                if m is not None:
                    continue
                x = var["hyp"][k + i]
                if pair & set(self.step_legs(x)):
                    ok = False                       # a step of F or S does not clear
                    break
                hb_arms = set(self.step_legs(x)) | set((r or {}).get("bystanders") or [])
                if pair & self._implicated(r, hb_arms or pair):
                    ok = False                       # the conflict moved to a later step instead of clearing
                    break
                left_blocked += 1
            htried.append({"name": var["name"], "modes": modes, "ok": ok,
                           "steps": [{o: l.op_id for o, l in self.step_legs(x).items()} for x in var["hyp"][k:]],
                           "fail": [{kk: r.get(kk) for kk in ("blocked_arms", "parallel", "bystanders",
                                                              "blocked_by_parked")}
                                    for r, m in zip(reps, modes) if m is None][:2]})
            if ok:
                total = sum(max((dur(l.traj) for l in self.step_legs(x).values()), default=0.0)
                            for x in var["hyp"][k:])
                cands.append({**var, "mode": "SEQUENTIAL", "duration_s": round(total, 3), "priority": 3,
                              "left_blocked": left_blocked})
        st.setdefault("_info", {})["handoff_candidates"] = [{kk: c[kk] for kk in ("name", "duration_s",
                                                                                   "left_blocked")}
                                                            for c in cands]
        st["_info"]["handoff_tried"] = htried
        st["_info"]["handoff_pairs"] = ["/".join(p) for p in pairs]
        st["_info"]["handoff_planned"] = {"/".join(o): {"S": len(d["S"]), "F": len(d["F"]) - 1}
                                          for o, d in planned.items()}
        # fewest steps left blocked first (a full clear before a partial one), then the policy's rule
        best = None
        if cands:
            least = min(c["left_blocked"] for c in cands)
            best = self.policy.choose_blocked_repair([c for c in cands if c["left_blocked"] == least])
        if best is None:
            return None
        info = st["_info"]
        self.steps = best["hyp"]
        self.steps[k].setdefault("_info", {}).update({kk: v for kk, v in info.items() if kk not in self.steps[k]
                                                      .get("_info", {})})
        self.steps[k]["_info"]["chosen"] = best["name"]
        self.log(f"[uf] step {k}: blocked -> {best['name']} ({best['duration_s']} s of handoff steps; pair "
                 f"{'/'.join(best['pair'])})")
        return best["name"]

    def _mk_leg(self, tmpl: Leg, r: dict) -> Leg:
        """A template leg with a planned result attached."""
        tmpl.status, tmpl.start, tmpl.traj, tmpl.end = "ok", r.get("start"), r["traj"], r["end"]
        tmpl.pred_end, tmpl.approach = r["end"], r.get("approach")
        tmpl.set_rec(r)
        tmpl.version += 1
        return tmpl

    def _prev_leg(self, leg: Leg) -> Optional[Leg]:
        s = self.seq(leg.ns)
        i = s.index(leg)
        return s[i - 1] if i > 0 else None

    def _spec(self, leg: Leg, start=None) -> dict:
        prev = self._prev_leg(leg)
        return {"ns": leg.ns, "op_id": leg.op_id, "req": leg.req,
                "start": list(start if start is not None else leg.start), **self.chain_ctx(leg.ns, prev)}

    def _coupled_jobs(self, k: int, legs: dict) -> dict:
        jobs = {}
        names = sorted(legs)
        for order in (names, names[::-1]):
            specs = [self._spec(legs[ns]) for ns in order]
            jobs[self.submit("coupled", k, W.plan_coupled, self.cell, specs, self.policy.call_kwargs())] = \
                {"name": "coupled_" + "_".join(order), "k": k}
        return jobs

    def _coupled_ok(self, legs: dict, res: dict) -> Optional[dict]:
        """A coupled re-plan is usable only if it keeps every endpoint (the chain) and every weld approach."""
        if not res.get("ok"):
            return None
        new = {r["ns"]: r for r in res["legs"]}
        if any(gap(new[ns]["end"], legs[ns].end) > TOL_RAD for ns in new):
            return None
        if any(new[ns].get("approach") and not new[ns]["approach"].get("matches") for ns in new):
            return None
        return {"delays": {}, "trajs": {ns: new[ns]["traj"] for ns in new},
                "approach": {ns: new[ns].get("approach") for ns in new}, "recs": {ns: new[ns] for ns in new}}

    @staticmethod
    def _step_dur(mode: str, trajs: dict, delays: dict) -> float:
        if mode == "SEQUENTIAL":
            return round(sum(dur(t) for t in trajs.values()), 3)
        return round(max(dur(t) + float(delays.get(ns, 0.0)) for ns, t in trajs.items()), 3)

    def _improve(self, todo: dict) -> dict:
        """Serialised two-arm steps: start-delay and coupled variants for all of them at once, each gated; keep the
        policy's choice when it is faster than serialising. Returns the new verdicts of the steps it changed."""
        todo = {k: v for k, v in todo.items() if len(self.step_legs(self.steps[k])) >= 2
                and self.policy.improve_sequential(self.steps[k])}
        if not todo:
            return {}
        bys = self.bystanders_per_step()
        jobs = {}
        for k in todo:
            legs = self.step_legs(self.steps[k])
            jobs[self.submit("delay", k, W.start_delays, self.cell, {ns: l.traj for ns, l in legs.items()})] = \
                {"name": "delay", "k": k}
            jobs.update(self._coupled_jobs(k, legs))
        gate_jobs = {}
        for f in self._within(jobs, self.policy.p.improve_timeout_s):
            kind, k, t_sub = self.futs.pop(f)
            var = jobs[f]
            res = f.result()
            self.record(kind, t_sub, res, {"step": k, "variant": var["name"]})
            legs = self.step_legs(self.steps[k])
            if var["name"] == "delay":
                fix = (res or {}).get("fix")
                if not fix:
                    continue
                var.update(delays=fix["start_delays"], trajs={ns: l.traj for ns, l in legs.items()})
            else:
                ok = self._coupled_ok(legs, res)
                if ok is None:
                    continue
                var.update(ok)
            gate_jobs[self.submit("gate", k, W.gate_step, self.cell, var["trajs"], var["delays"], bys[k], True)] = var
        variants: dict = {k: [] for k in todo}
        for f in self._as_completed(list(gate_jobs)):
            kind, k, t_sub = self.futs.pop(f)
            var = gate_jobs[f]
            res = f.result()
            self.record("gate", t_sub, res, {"step": k, "variant": var["name"], "mode": res.get("mode")})
            if res.get("mode") is not None:
                variants[k].append({**var, "mode": res["mode"], "report": res.get("report"),
                                    "duration_s": self._step_dur(res["mode"], var["trajs"], var["delays"])})
        out = {}
        for k, v in todo.items():
            st = self.steps[k]
            legs = self.step_legs(st)
            cur = self._step_dur("SEQUENTIAL", {ns: l.traj for ns, l in legs.items()}, {})
            best = self.policy.choose_step_variant(variants[k])
            st["_info"] = {"was": "SEQUENTIAL", "sequential_s": cur,
                           "variants": [{kk: vv for kk, vv in x.items() if kk in ("name", "mode", "duration_s",
                                                                                  "delays")} for x in variants[k]]}
            if best is not None and best["mode"] == "PARALLEL" and best["duration_s"] < cur:
                self._apply_variant(st, legs, best)
                out[k] = {"mode": "PARALLEL", "report": best.get("report")}
        return out

    def _apply_variant(self, st: dict, legs: dict, var: dict) -> None:
        for ns, t in var["trajs"].items():
            legs[ns].traj = t
            if (var.get("approach") or {}).get(ns):
                legs[ns].approach = var["approach"][ns]
            if (var.get("recs") or {}).get(ns):
                legs[ns].set_rec(var["recs"][ns])
            legs[ns].attempts.append(f"step:{var['name']}")
        if var.get("delays"):
            st["_delay"] = {ns: float(d) for ns, d in var["delays"].items() if d}
        else:
            st.pop("_delay", None)
        st.setdefault("_info", {})["chosen"] = var["name"]

    def _repair_blocked(self, k: int, exclude: Optional[set] = None) -> Optional[str]:
        """Blocked step k: coupled re-plans (if two arms move), a re-plan of each moving leg routed around the
        standing arm the gate blames (``route_*``: that arm's last leg planned first in the same call, so the moving
        leg is planned around its swept volume -- no new motion, same endpoints), and a park move before the step
        for every arm that is involved (moving -- also from HOME -- or standing by), every park pose of the policy,
        plus a via above the weld before each moving weld leg; all planned at once, each judged by the gate on
        every step it changes. Applies the policy's choice and returns its name; None if nothing unblocks the step.
        ``exclude``: candidate names not to choose (repairs that already failed to clear this conflict)."""
        exclude = exclude or set()
        st = self.steps[k]
        legs = self.step_legs(st)
        bys = self.bystanders_per_step()[k]
        jobs = {}
        if len(legs) >= 2:
            jobs.update(self._coupled_jobs(k, legs))
        rep = (st.get("_gate") or {}).get("report") or {}
        blamed = [b for b in self._worst_pair(rep) if b in bys]
        for ns in sorted(legs):
            for b in blamed:
                prior = [l for l in self.seq(b) if self._step_index(l) < k]
                if not prior or prior[-1].start is None or legs[ns].start is None:
                    continue
                specs = [self._spec(prior[-1]), self._spec(legs[ns])]
                jobs[self.submit("route", k, W.plan_coupled, self.cell, specs, self.policy.call_kwargs())] = \
                    {"name": f"route_{ns}_around_{b}", "k": k, "route": ns}
        if len(legs) >= 2:
            # defer one arm: its legs from step k on move one step later (no re-planning: same legs, same chain,
            # only the pairing changes -- what Opus did by planning robot2's later welds after robot1 parked)
            for ns in sorted(legs):
                hyp = self.deferred_steps(k, ns)
                hb = self.bystanders_per_step(hyp)
                checks = []
                for i in (k, k + 1):
                    if i < len(hyp):
                        hl = self.step_legs(hyp[i])
                        checks.append(({o: l.traj for o, l in hl.items()}, {}, hb[i]))
                var = {"name": f"defer_{ns}", "k": k, "hyp": hyp, "delays": {},
                       "trajs": {o: l.traj for o, l in self.step_legs(hyp[k]).items()}}
                gate_jobs_pre = self.submit("gate_many", k, W.gate_many, self.cell, checks)
                jobs[gate_jobs_pre] = var
        for ns in sorted(set(legs) | set(bys)):
            if ns in legs:
                frm = self._prev_leg(legs[ns])
            else:
                prior = [l for l in self.seq(ns) if self._step_index(l) < k]
                frm = prior[-1] if prior else None
            if frm is None and ns not in legs:
                continue                       # an arm still at HOME is not a bystander
            start = frm.end if frm is not None else self.home[ns]
            moves = [(f"park_{ns}_{name}", "park", xyz) for name, xyz in self.policy.park_targets(self, ns, start)
                     if frm is not None or name != "home"]
            if ns in legs and legs[ns].kind == "weld":
                # a via above the weld (the recovery ladder's lever: a different route into the weld)
                av = np.asarray(legs[ns].req.get("approach_vector", [0, 0, -1]), float)
                av = av / (np.linalg.norm(av) + 1e-9)
                t = legs[ns].req["target"]
                tgt = np.array([t["x"], t["y"], t["z"]])
                moves += [(f"via_{ns}_{int(round(h * 100))}", "via", tgt - av * h) for h in self.policy.p.via_heights_m]
            for name, kind, xyz in moves:
                park = self.new_move(ns, f"{ns.upper()}_{kind.upper()}{self._n}", xyz, kind=kind)
                specs = [{"ns": ns, "op_id": park.op_id, "req": park.req, "start": list(start),
                          **self.chain_ctx(ns, frm)}]
                if ns in legs:
                    specs.append({"ns": ns, "op_id": legs[ns].op_id, "req": legs[ns].req})
                jobs[self.submit("park", k, W.plan_chain, self.cell, specs, self.policy.call_kwargs())] = \
                    {"name": name, "k": k, "arm": ns, "park": park, "frm": frm}
        gate_jobs = {}
        cands = []
        tried: list = []
        for f in self._within(jobs, self.policy.p.repair_timeout_s):
            kind, _, t_sub = self.futs.pop(f)
            var = jobs[f]
            res = f.result()
            if kind != "gate_many":
                self.record(kind, t_sub, res, {"step": k, "variant": var["name"], "ok": res.get("ok")})
            tried.append({"name": var["name"], "ok": res.get("ok"), "modes": res.get("modes"),
                          "fail": [f.get("reason_code") for l in (res.get("legs") or []) for f in l.get("failures", [])]})
            if kind == "gate_many":             # a deferral: already judged
                modes = res.get("modes") or [None]
                self.record("gate", t_sub, res, {"step": k, "variant": var["name"], "modes": modes})
                if all(m is not None for m in modes):
                    cands.append({**var, "mode": modes[0], "priority": 1,
                                  "duration_s": self._step_dur(modes[0], var["trajs"], {})})
                continue
            if kind == "coupled":
                ok = self._coupled_ok(legs, res)
                if ok is None:
                    continue
                var.update(ok)
                checks = [(var["trajs"], {}, bys)]
            elif kind == "route":
                ns = var["route"]
                r = next((x for x in res.get("legs") or [] if x.get("ns") == ns), None)
                if not r or not r.get("ok") or gap(r.get("end"), legs[ns].end) > TOL_RAD or \
                        (r.get("approach") and not r["approach"].get("matches")):
                    continue
                var.update(delays={}, trajs={**{o: l.traj for o, l in legs.items()}, ns: r["traj"]},
                           approach={ns: r.get("approach")}, recs={ns: r})
                checks = [(var["trajs"], {}, bys)]
            else:
                if not res.get("ok"):
                    continue
                ns = var["arm"]
                r = res["legs"]
                var["park_res"] = r[0]
                var["leg_res"] = r[1] if len(r) > 1 else None
                if var["leg_res"] is not None and var["leg_res"].get("approach") and \
                        not var["leg_res"]["approach"].get("matches"):
                    continue
                # bystanders of the park step: every other arm that has moved, where it stands before step k
                bys_park = {o: q for o, q in bys.items() if o != ns}
                for o, l in legs.items():
                    if o != ns:
                        pl = self._prev_leg(l)
                        if pl is not None:
                            bys_park[o] = pl.end
                trajs_k = {o: l.traj for o, l in legs.items()}
                bys_k = dict(bys)
                if ns in legs:
                    trajs_k[ns] = var["leg_res"]["traj"]
                else:
                    bys_k[ns] = r[0]["end"]
                var.update(trajs=trajs_k, delays={})
                checks = [({ns: r[0]["traj"]}, {}, bys_park), (trajs_k, {}, bys_k)]
            gate_jobs[self.submit("gate_many", k, W.gate_many, self.cell, checks)] = var
        for f in self._as_completed(list(gate_jobs)):
            kind, _, t_sub = self.futs.pop(f)
            var = gate_jobs[f]
            res = f.result()
            modes = res.get("modes") or [None]
            self.record("gate", t_sub, res, {"step": k, "variant": var["name"], "modes": modes})
            tried.append({"name": var["name"], "gate_modes": modes,
                          "reports": [{kk: r.get(kk) for kk in ("mode", "blocked_arms", "serial_attempts",
                                                                 "parallel", "bystanders")}
                                      for r in res.get("reports", [])]})
            if any(m is None for m in modes):
                continue
            d = self._step_dur(modes[-1], var["trajs"], var["delays"])
            if "park_res" in var:
                d += dur(var["park_res"]["traj"])
            cands.append({**var, "mode": modes[-1], "duration_s": round(d, 3),
                          "priority": 0 if var["name"].startswith(("coupled", "route")) else 2})
        st.setdefault("_info", {})["blocked_candidates"] = [
            {kk: c[kk] for kk in ("name", "mode", "duration_s")} for c in cands]
        st["_info"]["blocked_tried"] = tried
        best = self.policy.choose_blocked_repair([c for c in cands if c["name"] not in exclude])
        if best is None:
            return None
        if "hyp" in best:
            self.steps = best["hyp"]
            self.steps[k].setdefault("_info", {})["chosen"] = best["name"]
            self.log(f"[uf] step {k}: blocked -> {best['name']} ({best['mode']})")
            return best["name"]
        if "park_res" not in best:
            self._apply_variant(st, legs, best)
            self.log(f"[uf] step {k}: blocked -> {best['name']} ({best['mode']})")
            return best["name"]
        ns, park, r0 = best["arm"], best["park"], best["park_res"]
        park.status, park.start, park.traj, park.end = "ok", r0["start"], r0["traj"], r0["end"]
        park.set_rec(r0)
        park.pred_end = park.end
        park.attempts.append(f"park for step {k}")
        self.steps.insert(k, {ns: park, "_info": {"park_for_blocked_step": True}})
        if best.get("leg_res") is not None:
            leg = legs[ns]
            leg.version += 1
            leg.start = best["leg_res"]["start"]
            leg.ctx_pose = pose_key(park.pose)
            self._accept(leg, best["leg_res"])
            leg.attempts.append("replanned_after_park")
        st["_info"]["chosen"] = best["name"]
        self.log(f"[uf] step {k}: blocked -> {best['name']} ({best['mode']}, {best['duration_s']} s)")
        return best["name"]

    def _step_index(self, leg: Leg) -> int:
        return next(k for k, st in enumerate(self.steps) if st.get(leg.ns) is leg)

    # -- phase: assembly --------------------------------------------------------------------------
    def plan_json(self, home_at_end: bool = True, extra_steps: Optional[list] = None) -> dict:
        steps = []
        for st in list(self.steps) + list(extra_steps or []):
            legs = self.step_legs(st)
            if not legs:
                continue
            delays = st.get("_delay", {})
            robots = []
            for ns in sorted(legs):
                l = legs[ns]
                e = {"robot_ns": ns, "operation_id": l.op_id,
                     "operation_type": "weld" if l.kind == "weld" else "move", "hold_duration_s": 0.5}
                if delays.get(ns):
                    e["start_delay_s"] = float(delays[ns])
                robots.append(e)
            steps.append({"mode": self.policy.step_mode(st) if len(robots) > 1 else "SEQUENTIAL",
                          "robots": robots})
        return {"steps": steps, "home_at_end": bool(home_at_end)}

    def all_trajs(self, extra_steps: Optional[list] = None) -> list:
        out = []
        for st in list(self.steps) + list(extra_steps or []):
            for ns, l in self.step_legs(st).items():
                out.append((ns, l.op_id, l.traj))
        return out

    def arm_state(self, extra_steps: Optional[list] = None) -> dict:
        """Each arm's chaining state at the end of the program: where it stands, its working pose and (76ea13a)
        every weld pose / op spec the planner recorded for it (the assembler's home step and approach audit)."""
        st = {}
        steps = list(self.steps) + list(extra_steps or [])
        for ns in self.robots:
            s = [x[ns] for x in steps if isinstance(x.get(ns), Leg) and x[ns].status == "ok"]
            if s:
                ctx = self.chain_ctx(ns, s[-1], steps)
                st[ns] = {"end": s[-1].end, "pose": ctx.pop("prev_pose"), **ctx}
        return st

    def assemble(self) -> dict:
        """Try every plan shape at once in workers; re-assemble the preferred acceptable one in this session."""
        variants = self.policy.home_variants()
        shapes = [{**variants[0], "extra": []}] if variants else []
        if variants and variants[0].get("home_at_end"):
            # final parks (for an arm that ends inside the other's warning band)
            for ns in self.robots:
                s = self.seq(ns)
                if not s:
                    continue
                for name, xyz in self.policy.park_targets(self, ns, s[-1].end)[:3]:
                    shapes.append({"name": f"final_park_{ns}_{name}", "home_at_end": True, "park": (ns, xyz),
                                   "extra": None})
        shapes += [{**v, "extra": []} for v in variants[1:]]
        # plan the final parks first (all at once)
        pj = {}
        for i, sh in enumerate(shapes):
            if sh.get("park"):
                ns, xyz = sh["park"]
                s = self.seq(ns)
                park = self.new_move(ns, f"{ns.upper()}_PARK_END_{i}", xyz, kind="park")
                spec = {"ns": ns, "op_id": park.op_id, "req": park.req, "start": s[-1].end,
                        **self.chain_ctx(ns, s[-1])}
                pj[self.submit("park", i, W.plan_chain, self.cell, [spec], self.policy.call_kwargs())] = (i, park)
        for f in self._as_completed(list(pj)):
            kind, i, t_sub = self.futs.pop(f)
            i, park = pj[f]
            res = f.result()
            self.record("park", t_sub, res, {"op": park.op_id, "ok": res.get("ok")})
            if res.get("ok"):
                r0 = res["legs"][0]
                park.status, park.start, park.traj, park.end = "ok", r0["start"], r0["traj"], r0["end"]
                park.set_rec(r0)
                shapes[i]["extra"] = [{park.ns: park}]
        shapes = [s for s in shapes if s["extra"] is not None]
        jobs = {}
        for i, sh in enumerate(shapes):
            plan = self.plan_json(sh["home_at_end"], sh["extra"])
            jobs[self.submit("assemble", i, W.assemble_trial, self.cell, self.all_trajs(sh["extra"]),
                             self.arm_state(sh["extra"]), plan)] = i
        results = {}

        def settled():
            # the most preferred shape that assembled, once every more preferred one has finished (and failed)
            for i in range(len(shapes)):
                if i not in results:
                    return None
                if self.policy.accept_tree(results[i]):
                    return i
            return -1

        for f in self._as_completed(list(jobs)):
            kind, i, t_sub = self.futs.pop(f)
            res = f.result()
            self.record("assemble", t_sub, res, {"shape": shapes[i]["name"], "ok": res.get("ok")})
            results[i] = res
            if settled() is not None:
                break
        for f in list(jobs):                       # less preferred trials still running: not needed
            if f in self.futs:
                self.futs.pop(f)
                f.cancel()
        self.assembly_trials = [{"shape": shapes[i]["name"], "ok": results[i].get("ok"),
                                 "error": (results[i].get("result") or {}).get("error"),
                                 "wall_s": results[i].get("wall_s")} for i in sorted(results)]
        chosen = next((i for i in range(len(shapes)) if self.policy.accept_tree(results.get(i, {}))), None)
        if chosen is None:
            return {"success": False, "trials": self.assembly_trials}
        sh = shapes[chosen]
        plan = self.plan_json(sh["home_at_end"], sh["extra"])
        t = time.monotonic()
        chain = self.parent_chain_check(sh, plan)
        if os.environ.get("UF_REASSEMBLE", "0") == "1":
            # optional: repeat the chosen assembly in this session (same function, same inputs; the home step's
            # OMPL solve is randomised, so the tree can differ from the trial's)
            res = self.call("assemble_behaviour_tree", cell_id=self.cell, plan=plan)
        else:
            # the worker ran umarell's own assemble_behaviour_tree on exactly these trajectories and this plan
            res = {**results[chosen]["result"], "behaviour_tree": results[chosen]["behaviour_tree"],
                   "success": True, "assembled_in": "worker"}
        self.events.append({"kind": "assemble_final", "t_submit": self.now(), "t_done": self.now(),
                            "wall_s": round(time.monotonic() - t, 3), "pid": os.getpid()})
        if res.get("success") and not self.policy.accept_tree({"ok": True, "result": res}):
            res = {"success": False, "error": "policy rejected the tree"}
        if sh.get("extra"):
            for st in sh["extra"]:
                self.steps.append(st)
        return {"success": bool(res.get("success")), "shape": sh["name"], "result": res, "plan": plan,
                "chain": chain, "trials": self.assembly_trials}

    def parent_chain_check(self, sh: dict, plan: dict) -> dict:
        """Load the chosen program into THIS session's trajectory cache and run umarell's chain_trajectories on it
        (the assembler checked it in the worker; this is the record in the parent, where the plan is saved)."""
        from agents.trajectory.tools import trajectory_cache as tc
        tc.clear_trajectory_cache(self.cell)
        tc.reset_robot_configs(self.cell)
        for ns, op_id, traj in self.all_trajs(sh["extra"]):
            tc.cache_trajectory(self.cell, ns, op_id, traj)
        for ns, s in self.arm_state(sh["extra"]).items():
            W.inject(self.cell, ns, s["end"], s["pose"], s.get("welds"), s.get("specs"))
        order = [{"robot_ns": ns, "operation_ids": [r["operation_id"] for st in plan["steps"] for r in st["robots"]
                                                   if r["robot_ns"] == ns]} for ns in self.robots]
        return self.call("chain_trajectories", cell_id=self.cell, robots=order)

    # -- run --------------------------------------------------------------------------------------
    def load_ops(self) -> list:
        from umarell_fast.u2 import session as S
        task = S.load_task(self.cell_base, self.cell, self.task_name)
        self.ops = {o["id"]: o for o in task["operations"]}
        self.max_concurrent_robots = ((task.get("extensions") or {}).get("strategy") or {}).get(
            "max_concurrent_robots")
        return task["operations"]

    def build_steps(self, mrta: dict) -> None:
        """MRTA's steps -> the step list: one weld leg per arm per step, targets / orientations / approach
        vectors copied from the task (what Opus did for 237/237 requests in track L)."""
        self.mrta = {k: v for k, v in mrta.items() if k != "steps"}
        for st in mrta["steps"]:
            row = {}
            for ns in self.robots:
                op = st.get(ns)
                if isinstance(op, dict):
                    row[ns] = Leg(ns=ns, op_id=op["id"], req=self.policy.weld_request(ns, self.ops[op["id"]]),
                                  kind="weld", src=op["id"])
            if row:
                self.steps.append(row)
        # umarell dc60669+: MRTA leaves out a weld that no arm reaches by its geometric model (``unreachable``: no
        # arm within its workspace_radius on the approach side) instead of forcing it onto one. U3 does not take
        # that model's word for it: such a weld is planned like any other, as a step of its own on the arm whose
        # base is nearest, so the IK allocation check, the failure ladder and the reallocation decide -- and it is
        # reported unreachable only when every arm failed, as before.
        placed = {l.src for st in self.steps for l in st.values() if isinstance(l, Leg)}
        self.mrta_unreachable = []
        for u in mrta.get("unreachable") or []:
            oid = u.get("operation_id") if isinstance(u, dict) else u
            if oid is None or oid not in self.ops or oid in placed or not self.robots:
                continue
            p = np.asarray(self.ops[oid]["position"], float)[:2]
            ns = min(self.robots, key=lambda r: float(np.linalg.norm(
                p - np.asarray(self.robot_cfg[r]["base_position"], float)[:2])))
            self.steps.append({ns: Leg(ns=ns, op_id=oid, req=self.policy.weld_request(ns, self.ops[oid]),
                                       kind="weld", src=oid, attempts=["mrta:unreachable"])})
            placed.add(oid)
            self.mrta_unreachable.append({"op": oid, "arm": ns,
                                          "reason": u.get("reason") if isinstance(u, dict) else None})
        if self.mrta_unreachable:
            self.log(f"[uf] MRTA: {len(self.mrta_unreachable)} weld(s) outside every arm's modelled reach, planned "
                     f"anyway on the nearest arm: " + ", ".join(f"{m['op']}->{m['arm']}" for m in self.mrta_unreachable))

    def behind_weld(self, ns: str, op: dict, min_horizontal: float = 0.5) -> bool:
        """Whether arm ``ns`` would have to reach AROUND the workpiece for ``op``: the weld is approached mostly
        horizontally (the approach vector's horizontal part >= ``min_horizontal``) and the arm's base lies on the
        far side of the weld's plane (the tool travels along the approach vector into the weld, so it comes from
        the side the base must be on). Welds approached from above or below are never "behind"."""
        av = np.asarray(op.get("approach_vector") or [0, 0, -1], float)
        n = float(np.linalg.norm(av))
        if n < 1e-9:
            return False
        av = av / n
        h = av[:2]
        if float(np.linalg.norm(h)) < min_horizontal:
            return False
        base = np.asarray(self.robot_cfg[ns]["base_position"], float)[:2]
        p = np.asarray(op["position"], float)[:2]
        return float(np.dot(p - base, h)) < 0.0

    def fix_allocation(self) -> list:
        """MRTA's arm per weld, checked by IK before any planning: umarell's allocation score (approach vector .
        direction to the base, ``task_allocation._robot_scores``) assumes the base lies along the approach
        direction, which holds for welds approached from above and is backwards for side panels approached
        horizontally (body_weld_3: every weld went to the arm on the OTHER side of the car, failed with ik_failed
        and was reallocated, appended at the end of the arm's list -- MRTA's pairing lost, minutes of planning
        spent on failures). Each weld's arm is asked whether it reaches the weld from HOME (IK of the approach
        waypoint + the straight descent, :func:`workers.predict_end`; all checks at once); only where it does not,
        every other arm is asked, and each MRTA step is re-assigned to the arms that reach its welds (most
        reachable welds, then most kept on MRTA's arm, then nearest bases), keeping the step's pairing -- the
        mirrored allocation becomes a swap within each step. A weld no arm reaches stays where MRTA put it (the
        planner's failure / reallocation ladder handles it). Returns the moves made.

        The IK is collision-blind: with a long tool an arm on the far side of the workpiece can "reach" a side weld
        by IK, around or through the body (body_weld_3 with WELDING_GUN: robot2 reaches the left quarter-panel welds
        by IK, so the mirrored allocation stood, every one failed planning and was reallocated to the end of the
        near arm's chain -- MRTA's order and pairing lost, and those welds then planned from a chained start).
        An arm whose base lies on the far side of a horizontally approached weld (:meth:`behind_weld`) does not
        count as reaching it."""
        off = self.policy.p.approach_offset_m

        def reach(pairs: list) -> dict:
            jobs = {}
            out = {}
            for ns, src in pairs:
                if self.behind_weld(ns, self.ops[src]):
                    out[(ns, src)] = False
                    self.events.append({"kind": "reach", "t_submit": self.now(), "t_done": self.now(),
                                        "op": src, "arm": ns, "ok": False, "why": "base behind the weld"})
                    continue
                req = self.policy.weld_request(ns, self.ops[src])
                jobs[self.submit("reach", (ns, src), W.predict_chain, self.cell, ns, list(self.home[ns]), [req],
                                 off)] = (ns, src)
            for f in self._as_completed(list(jobs)):
                kind, _, t_sub = self.futs.pop(f)
                try:
                    res = f.result()
                except Exception as exc:  # noqa: BLE001 -- a failed check is "unknown": keep MRTA's arm
                    res = {"ends": [], "error": repr(exc)}
                ends = res.get("ends") or []
                out[jobs[f]] = bool(ends) and ends[0] is not None if "error" not in res else None
                self.record("reach", t_sub, res, {"op": jobs[f][1], "arm": jobs[f][0], "ok": out[jobs[f]]})
            return out

        rows = [{ns: l for ns, l in st.items() if isinstance(l, Leg) and l.kind == "weld" and l.src}
                for st in self.steps]
        ok = reach([(ns, l.src) for row in rows for ns, l in row.items()])
        bad = sorted({l.src for row in rows for ns, l in row.items() if ok.get((ns, l.src)) is False})
        self.allocation_fixes: list = []
        if not bad:
            return self.allocation_fixes
        ok.update(reach([(ns, src) for src in bad for ns in self.robots if (ns, src) not in ok]))

        def dist(ns, src):
            return float(np.linalg.norm(np.asarray(self.ops[src]["position"], float)[:2]
                                        - np.asarray(self.robot_cfg[ns]["base_position"], float)[:2]))

        import itertools
        for st, row in zip(self.steps, rows):
            if all(ok.get((ns, l.src)) is not False for ns, l in row.items()):
                continue
            legs = list(row.values())
            best, best_key = None, None
            for perm in itertools.permutations(self.robots, len(legs)):
                key = (sum(1 for a, l in zip(perm, legs) if ok.get((a, l.src))),
                       sum(1 for a, l in zip(perm, legs) if a == l.ns),
                       -round(sum(dist(a, l.src) for a, l in zip(perm, legs)), 6))
                if best_key is None or key > best_key:
                    best, best_key = perm, key
            cur = sum(1 for l in legs if ok.get((l.ns, l.src)))
            if best is None or best_key[0] <= cur:
                continue
            for ns in row:
                del st[ns]
            for a, l in zip(best, legs):
                if a == l.ns:
                    st[a] = l
                    continue
                st[a] = Leg(ns=a, op_id=l.op_id, req=self.policy.weld_request(a, self.ops[l.src]), kind="weld",
                            src=l.src, attempts=[f"allocation:{l.ns}->{a}"])
                self.allocation_fixes.append({"op": l.op_id, "from": l.ns, "to": a,
                                              "reaches": bool(ok.get((a, l.src)))})
        if self.allocation_fixes:
            self.log(f"[uf] allocation: {len(self.allocation_fixes)} weld(s) moved to an arm that reaches them "
                     f"(IK from HOME): " + ", ".join(f"{m['op']} {m['from']}->{m['to']}"
                                                     for m in self.allocation_fixes))
        return self.allocation_fixes

    def run(self, tree_id: Optional[str] = None) -> dict:
        t_run = time.monotonic()
        requested = self.kernels
        warm = [self.submit("ping", None, W.ping) for _ in range(self.n_workers)]  # start the pool meanwhile
        kinfo = self.setup_parent()
        if self.kernels != requested:          # kernels unavailable: workers must not use them either
            for f in warm:
                self.futs.pop(f, None)
            self.close()
            warm = [self.submit("ping", None, W.ping) for _ in range(self.n_workers)]
        self.phase_t["setup"] = self.now()
        ops = self.load_ops()
        # umarell dc60669+ honours an assigned_robot in run_mrta_pipeline's call only when it is one of the run's
        # locks (bound by the trajectory agent at the start of a run); the task file's own pins are, so bind them
        # here as the agent does (76ea13a: the same pins, already honoured from the operations).
        try:
            from agents.trajectory import operator_directives as od
            od.set_locks(od.all_locks(ops))
        except (ImportError, AttributeError):
            pass
        mrta = self.call("run_mrta_pipeline", cell_id=self.cell, operations=ops)
        if mrta.get("error"):
            return {"success": False, "error": f"run_mrta_pipeline: {mrta['error']}"}
        self.build_steps(mrta)
        self.phase_t["mrta"] = self.now()
        for f in warm:
            self.futs.pop(f, None)
        self._wait(warm, cf.ALL_COMPLETED)
        self.phase_t["pool_ready"] = self.now()
        chk_fut = None
        if self.kernels != "none":
            chk_fut = self.pool().submit(W.kernel_selfcheck, self.cell,
                                         int(os.environ.get("UF_KERNEL_CHECK_N", 40)),
                                         float(os.environ.get("UF_KERNEL_CHECK_MAX_S", 15)))
        if self.policy.check_allocation():
            self.fix_allocation()
        self.phase_t["allocation"] = self.now()
        self.plan_all_legs()
        self.phase_t["legs"] = self.now()
        self.repair_steps()
        self.phase_t["steps"] = self.now()
        if self.policy.compact_steps():
            self._seed_modes()
            self.compact()
            if len(self.robots) > 2 and self.policy.overlap_steps():
                self.overlap()
        self.phase_t["compact"] = self.now()
        if chk_fut is not None:
            chk = chk_fut.result()
            self.kernel_info["selfcheck"] = chk
            self.events.append({"kind": "kernel_selfcheck", "t_submit": self.phase_t["pool_ready"],
                                "t_done": self.now(), "wall_s": chk.get("wall_s"), "pid": chk.get("pid")})
            if not chk.get("ok"):
                # the planning ran on kernels that disagree with umarell's predicate: discard it and plan again on
                # umarell's own functions (what the self-check exists for), instead of returning an invalid run
                self.log(f"[uf] kernel self-check FAILED ({chk}) -> discarding this planning, re-running it on "
                         f"umarell's own kernels (--kernels none)")
                failed = dict(self.kernel_info)
                self._fallback_to_own_kernels()
                out = self.run(tree_id=tree_id)
                out.setdefault("kernels", {})["fallback_from"] = failed
                out["kernel_fallback"] = {"from": failed.get("effective"), "to": "none",
                                          "selfcheck": {k: chk.get(k) for k in ("checked", "mismatches", "error")}}
                out["wall_s"] = round(time.monotonic() - t_run, 3)
                return out
        asm = self.assemble()
        self.phase_t["assembly"] = self.now()
        # complete = every task operation is in the tree (an unreachable weld is not a plan of the task)
        out = {"success": bool(asm["success"]) and not self.unreachable, "assembled": bool(asm["success"]),
               "complete": not self.unreachable, "kernels": kinfo, "mrta": self.mrta, "assembly": {
            k: v for k, v in asm.items() if k not in ("result",)}, "unreachable": self.unreachable,
            "reallocated": self.reallocated, "allocation_fixes": getattr(self, "allocation_fixes", []),
            "mrta_unreachable": getattr(self, "mrta_unreachable", []),
            "no_progress": getattr(self, "no_progress", []), "phase_t": self.phase_t,
            "compaction": {"merges": getattr(self, "n_merges", 0), "overlaps": getattr(self, "n_overlaps", 0),
                           "max_concurrent_robots": self.max_concurrent(), "program_s": self.program_s(),
                           "overlap_log": getattr(self, "overlap_log", [])},
            "steps": [{"legs": {ns: l.op_id for ns, l in self.step_legs(st).items()},
                       "delays": st.get("_delay"), "info": st.get("_info"), "gate": st.get("_gate")}
                      for st in self.steps]}
        res = asm.get("result") or {}
        if asm["success"]:
            bt = res["behaviour_tree"]
            tree_id = tree_id or f"bt_{int(time.time())}_{self.cell}"
            bt["tree_id"] = tree_id
            from agents.trajectory.tools.plan_store import save_plan, get_plan
            saved = save_plan(self.cell, bt)
            back = get_plan(self.cell, tree_id)
            from agents.trajectory.tools.behaviour_tree import validate_behaviour_tree
            val = validate_behaviour_tree.func(back) if back else {"valid": False}
            out.update(tree_id=saved, plan_reloaded=back is not None, plan_valid=bool(val.get("valid")),
                       step_gates=res.get("step_gates"), serialised_steps=res.get("serialised_steps"),
                       home_step={k: v for k, v in (res.get("home_step") or {}).items()
                                  if k in ("mode", "gated", "coupled_arms", "blind_arms", "order")})
            out["plan_file"] = str(self.cell_base / self.cell / "agentic_planner" / "plans" / f"{tree_id}.json")
        else:
            out["error"] = res.get("error") or "no plan shape assembled"
            out["assembly_error"] = {k: v for k, v in res.items() if k in ("error", "blocked_steps", "home_step",
                                                                        "breaks", "missing")}
        out["legs"] = [l.short() for st in self.steps for l in st.values() if isinstance(l, Leg)]
        out["n_steps"] = len(self.steps)
        out["wall_s"] = round(time.monotonic() - t_run, 3)
        out["events"] = self.events
        out["stats"] = stats(self.events, out["wall_s"], self.n_workers)
        pc = self.plan_cache()
        if pc is not None:
            pc.write_meta({"tree_id": out.get("tree_id"), "success": out.get("success")})
            if out.get("success"):
                pc.write_journal(self._consumed, self._dropped)
            out["cache"] = pc.info()
            per = ", ".join(f"{k} {v['hits']}/{v['hits'] + v['misses']}" for k, v in sorted(pc.stats["by_fn"].items()))
            self.log(f"[uf] plan cache: {pc.stats['hits']} hit(s), {pc.stats['misses']} miss(es) ({per}); similar "
                     f"cell: {pc.stats['similar_reused']} of {pc.stats['similar_tried']} candidate(s) reused; spliced weld legs: "
                     f"{pc.stats['splice_used']} of {pc.stats['splice_tried']}")
        return out

    def _fallback_to_own_kernels(self) -> None:
        """After a failed kernel self-check: umarell's own collision functions back in this process, a fresh worker
        pool without kernels, and the planning state cleared (MRTA, legs and steps are recomputed by ``run``)."""
        kernels, orig = getattr(self, "_parent_kernels", ("none", {}))
        try:
            W.uninstall_kernels(kernels, orig)
        except Exception as exc:  # noqa: BLE001
            self.log(f"[uf] could not uninstall kernels {kernels!r} in the parent ({exc!r})")
        self._parent_kernels = ("none", {})
        for f in list(self.futs):
            f.cancel()
        self.close()
        try:
            from agents.trajectory.tools.trajectory_cache import clear_trajectory_cache
            clear_trajectory_cache(self.cell)
        except Exception:  # noqa: BLE001 -- no umarell session (tests) or an older cache API
            pass
        self.kernels = "none"
        self._plan_cache = None                  # its key holds the kernels
        self._journal_idx = None
        self._journal_drop = None
        self._consumed, self._dropped = [], []
        self.steps, self.futs, self.recovering = [], {}, {}
        self.unreachable, self.reallocated, self.gate_info = [], [], {}
        self.events.append({"kind": "kernel_fallback", "t_submit": self.now(), "t_done": self.now()})

    def close(self, kill: bool = False) -> None:
        """Shut the pool down; ``kill``: SIGKILL its worker processes first (after a PoolFailure: a hung worker
        would otherwise keep the interpreter from exiting)."""
        if self._pool is not None:
            if kill:
                for p in self._pool_processes().values():
                    try:
                        p.kill()
                    except Exception:  # noqa: BLE001
                        pass
            self._pool.shutdown(wait=False, cancel_futures=True)
            self._pool = None


def stats(events: list, wall: float, n_workers: int) -> dict:
    """Busy time per task kind, worker utilisation and the achieved concurrency."""
    by: dict = {}
    busy = 0.0
    for e in events:
        w = float(e.get("wall_s") or 0.0)
        d = by.setdefault(e["kind"], {"n": 0, "busy_s": 0.0})
        d["n"] += 1
        d["busy_s"] = round(d["busy_s"] + w, 3)
        if e["kind"] not in ("predict", "assemble_final", "ping"):
            busy += w
    pids = {e.get("pid") for e in events if e.get("pid") and e["kind"] not in ("predict", "assemble_final")}
    return {"by_kind": by, "worker_busy_s": round(busy, 3), "workers": n_workers, "worker_pids": len(pids),
            "mean_concurrency": round(busy / wall, 2) if wall > 0 else None,
            "utilisation": round(busy / (wall * n_workers), 3) if wall > 0 and n_workers else None}
