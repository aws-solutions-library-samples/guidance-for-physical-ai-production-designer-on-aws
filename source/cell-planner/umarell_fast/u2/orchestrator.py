"""U2: a deterministic orchestrator that drives umarell's own tools in code, in parallel.

Pipeline (umarell's classic workflow, docs/scouting/planner.md §1, with every tool argument produced in code):

1. ``run_mrta_pipeline`` (umarell's allocation, weld order and step pairing; its moved weld positions are used).
2. **Goal chain by IK.** The goal joints of every leg are a deterministic function of the leg's target and its start
   (umarell's planner solves ``run_ik`` warm-started from the arm's previous goal), so the start state of every leg
   is known before anything is planned. ``plan`` therefore runs **every leg of every arm at once** in a process
   pool, each with its start state injected (umarell_fast.u2.workers), instead of one leg per LLM turn.
3. **Recovery** of a failed leg: the candidates of umarell's own recovery ladder (retry with another seed, a via
   point above the weld or above the previous weld, a different standoff, slower transit) are planned
   **concurrently**; the decision router picks among those that planned (rule: first in the reason-code ladder).
   A candidate that changes the leg's goal re-predicts and re-plans the arm's later legs (again in parallel).
   When every candidate fails: ``reallocate_operation`` and plan the weld at the end of the arm it names.
4. ``validate_approach`` for every weld and ``chain_trajectories`` for every arm (a failed approach is recovered
   like a planner failure).
5. ``assemble_behaviour_tree`` from the MRTA pairing (two-arm steps requested PARALLEL; umarell's gate
   serialises / refuses). Variants (home_at_end true / false) are assembled concurrently in the pool. A refusal
   by a parked bystander is answered with park moves: candidates toward the arm's own base and raised (and the
   arm's HOME tool position) are planned and assembled in parallel; Opus generates park points only if all of
   them fail. Serialised two-arm steps get a start-delay variant (``find_start_delays``), kept if it shortens the
   makespan.
6. ``plan_store.save_plan`` (umarell's plan JSON + BT.CPP XML), then umarell's own ``validate_behaviour_tree`` and
   ``plan_store.get_plan`` on the stored file.

Every discrete choice goes through ``decisions.Router`` and is logged with its source and confidence.
"""
from __future__ import annotations

import copy
import itertools
import json
import multiprocessing as mp
import os
import shutil
import threading
import time
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, ThreadPoolExecutor, wait
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from umarell_fast.u2 import session as S
from umarell_fast.u2 import workers as W
from umarell_fast.u2.decisions import Question, Router

ROOT = Path(__file__).resolve().parents[2]
EPS_Q = 1e-6

# umarell's recovery ladder (skills/trajectory-classic/recover-from-planner-failure, docs/uf_decider_study.md §1),
# expressed as candidate strategies; order = the rule's preference per reason code.
STRATEGIES = {
    "retry": "re-plan the same leg with another random seed (OMPL RRTConnect is randomised)",
    "via_above": "add a via point 0.25 m directly above the weld, then plan the weld from there",
    "via_lift": "add a via point 0.30 m above the PREVIOUS weld (lift out of the previous seam first)",
    "via_high": "add a via point 0.40 m above the weld (a higher route)",
    "offset_long": "longer standoff / lift-off: approach_offset_m 0.15, retreat_offset_m 0.20",
    "offset_short": "shorter standoff: approach_offset_m 0.07 (tight pocket)",
    "slow": "slower transit (velocity_scale 0.35): smaller speed allowance in the clearance gate",
}
LADDER = {
    "excessive_tool_rise": ["via_above", "via_lift", "via_high", "retry", "offset_long", "slow", "offset_short"],
    "trajectory_in_collision": ["offset_long", "via_above", "retry", "slow", "via_lift", "via_high", "offset_short"],
    "goal_in_collision": ["offset_short", "offset_long", "retry", "via_above", "slow", "via_lift", "via_high"],
    "inter_arm_clearance": ["retry", "via_above", "slow", "via_lift", "via_high", "offset_long", "offset_short"],
    "approach_mismatch": ["offset_long", "offset_short", "retry", "via_above", "slow", "via_lift", "via_high"],
    "self_collision": ["retry", "offset_long", "offset_short", "via_above", "via_lift", "via_high", "slow"],
    None: ["retry", "via_above", "offset_long", "slow", "via_lift", "via_high", "offset_short"],
}


@dataclass
class Item:
    uid: int
    robot: str
    op_id: str
    kind: str                     # weld | move
    request: dict                 # umarell motion request (without current_joints)
    group: float                  # MRTA step it belongs to (vias / parks get the group of their weld)
    op: dict | None = None        # the MRTA operation dict (welds)
    call_kw: dict = field(default_factory=dict)
    status: str = "pending"       # pending | ok | failed | recovering | dropped
    traj: list | None = None
    state_in: dict | None = None
    state_out: dict | None = None
    version: int = 0
    strategy: str = "plan"
    reason: str | None = None
    failures: list = field(default_factory=list)
    attempts: int = 0
    tried_robots: list = field(default_factory=list)


def _q_eq(a, b) -> bool:
    if a is None or b is None:
        return a is None and b is None
    return len(a) == len(b) and float(np.max(np.abs(np.asarray(a, float) - np.asarray(b, float)))) < EPS_Q


def _state_eq(a: dict | None, b: dict | None) -> bool:
    a = a or {}
    b = b or {}
    if not _q_eq(a.get("last_config"), b.get("last_config")):
        return False
    pa, pb = a.get("last_pose"), b.get("last_pose")
    if (pa is None) != (pb is None):
        return False
    if pa is None:
        return True
    return np.allclose(pa["target"], pb["target"]) and np.allclose(pa["approach_vector"], pb["approach_vector"])


def weld_request(robot: str, op: dict, extra: dict | None = None) -> dict:
    """The motion request Opus wrote in 237/237 cases (docs/uf_decider_study.md §2): copies of the MRTA op."""
    p = op["position"]
    ori = op.get("weld_orientation") or op.get("orientation")
    req = {"robot_ns": robot, "operation_id": op["id"], "operation_type": op.get("type", "weld"),
           "target": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
           "approach_vector": list(op.get("approach_vector", [0.0, 0.0, -1.0])),
           "approach": True}
    if ori:
        req["orientation"] = dict(ori)
        req["constrain_orientation"] = True
    req.update(extra or {})
    return req


def move_request(robot: str, op_id: str, xyz, av=(0.0, 0.0, -1.0)) -> dict:
    return {"robot_ns": robot, "operation_id": op_id, "operation_type": "move",
            "target": {"x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2])},
            "approach_vector": list(av), "approach": False}


class U2:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.cell = cfg["cell"]
        self.run_dir = Path(cfg["run_dir"]).resolve()
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.repo = Path(cfg["repo"]).resolve()
        self.t0 = time.perf_counter()
        self.events = open(self.run_dir / "events.jsonl", "a")
        self._ev_lock = threading.Lock()
        self.uid = itertools.count(1)
        self.seq: dict[str, list[Item]] = {}
        self.robots: list[str] = []
        self.phase_s: dict[str, float] = {}
        self.counts = {"plan_tasks": 0, "assemble_tasks": 0, "recoveries": 0, "reallocations": 0,
                       "downstream_replans": 0, "park_moves": 0}
        self.seed = itertools.count(int(cfg.get("seed", 0)) * 100003 + 1)
        self.router: Router = cfg["router"]
        self.llm = cfg.get("llm")
        self.result: dict = {}
        self.vs = cfg.get("velocity_scale")

    # ---------------------------------------------------------------------------------------------------------
    def ev(self, kind: str, **kw) -> None:
        rec = {"t": round(time.perf_counter() - self.t0, 4), "ev": kind, **kw}
        with self._ev_lock:
            self.events.write(json.dumps(rec, default=str) + "\n")
            self.events.flush()
        if self.cfg.get("verbose"):
            print(f"[u2 {rec['t']:8.2f}] {kind} " + " ".join(f"{k}={v}" for k, v in kw.items())[:300], flush=True)

    def _phase(self, name: str, t_start: float) -> None:
        self.phase_s[name] = round(time.perf_counter() - t_start, 3)
        self.ev("phase", name=name, s=self.phase_s[name])

    # ---------------------------------------------------------------------------------------------------------
    def run(self) -> dict:
        cell = self.cell
        self.cell_base = S.make_cell_base(self.repo, self.run_dir, cell)
        env = S.env_for(self.repo, cell, self.cell_base, self.run_dir / "scratch", self.cfg.get("env"))
        t = time.perf_counter()
        S.setup(self.repo, cell, env, kernels=self.cfg.get("kernels", "npplus"))
        task = S.load_task(self.cell_base, cell, self.cfg.get("task", "car_door_spot_welding.json"))
        self.operations = task["operations"]
        self.nw = int(self.cfg.get("workers") or max(2, (os.cpu_count() or 4) - 2))
        self._env = env
        self.pool = None
        self.threads = ThreadPoolExecutor(max_workers=8)
        nw = self.nw
        self._phase("setup", t)
        self.ev("start", cell=cell, workers=nw, mode=self.router.mode, kernels=self.cfg.get("kernels"),
                repo=str(self.repo))
        try:
            t = time.perf_counter()
            self.mrta = S.call("run_mrta_pipeline", cell_id=cell, operations=self.operations)
            if self.mrta.get("error"):
                raise RuntimeError(f"MRTA failed: {self.mrta['error']}")
            self.robots = list(self.mrta.get("robots") or [])
            self._build_sequences()
            self._phase("mrta", t)
            t = time.perf_counter()
            for r in self.robots:
                self._chain_states(r)          # IK goal chain; also warms the robot models / YAML caches
            self._start_pool()                 # fork AFTER the caches are warm, so every worker inherits them
            self._phase("predict+fork", t)
            t = time.perf_counter()
            self._plan_all()
            self._phase("plan", t)
            t = time.perf_counter()
            self._validate_and_chain()
            self._phase("validate", t)
            t = time.perf_counter()
            self._assemble()
            self._phase("assemble", t)
            t = time.perf_counter()
            self._save()
            self._phase("save", t)
        finally:
            if self.pool is not None:
                self.pool.shutdown(wait=False, cancel_futures=True)
            self.threads.shutdown(wait=False)
        self.result["wall_s"] = round(time.perf_counter() - self.t0, 3)
        self.result["phase_s"] = self.phase_s
        self.result["counts"] = self.counts
        self.ev("done", wall_s=self.result["wall_s"], success=self.result.get("success"))
        return self.result

    def _start_pool(self) -> None:
        try:                                   # import OMPL's bindings once in the parent (inherited by fork)
            from agents.trajectory.tools import ompl_planner as _op
            _op._import_ompl()
        except Exception:  # noqa: BLE001
            pass
        ctx = mp.get_context("fork")
        self.pool = ProcessPoolExecutor(max_workers=self.nw, mp_context=ctx, initializer=W.init_worker,
                                        initargs=(str(self.repo), self.cell, self._env,
                                                  self.cfg.get("kernels", "npplus"), int(self.cfg.get("seed", 0))))
        for f in [self.pool.submit(os.getpid) for _ in range(self.nw)]:
            f.result()

    # -- 1. sequences from MRTA --------------------------------------------------------------------------------
    def _build_sequences(self) -> None:
        self.seq = {r: [] for r in self.robots}
        for st in self.mrta["steps"]:
            g = float(st["step"])
            for r in self.robots:
                op = st.get(r)
                if isinstance(op, dict) and op.get("id"):
                    req = weld_request(r, op, self.cfg.get("request_extra"))
                    it = Item(next(self.uid), r, op["id"], "weld", req, g, op=op, call_kw=self._kw())
                    self.seq[r].append(it)
        self.ev("allocation", **{r: [i.op_id for i in s] for r, s in self.seq.items()})

    def _kw(self, **over) -> dict:
        kw = {}
        if self.vs is not None:
            kw["velocity_scale"] = self.vs
        kw.update({k: v for k, v in over.items() if v is not None})
        return kw

    # -- 2. goal chain prediction ------------------------------------------------------------------------------
    def _predict_out(self, it: Item, state_in: dict) -> dict:
        """The state umarell's planner leaves after ``it`` from ``state_in``, without planning.

        ompl_planner.plan_trajectories_ompl is deterministic up to the leg's END joints: IK of the approach point
        warm-started from the chained start (``via_q``), the OMPL transit ending exactly at ``via_q``, then the
        straight Cartesian descent (6 IK steps, each warm-started from the previous one) into the weld. With
        U2_01 the planner chains the next leg from that last sample. A request without an orientation ends at the
        IK goal. The last working pose is the request's target / approach vector / orientation when constrained.
        """
        from agents.trajectory.tools.pinocchio import run_ik
        from agents.trajectory.tools.ompl_planner import _home_config, HOME_JOINTS_DEFAULT
        ik = getattr(run_ik, "func", run_ik)
        req = it.request
        warm = (state_in or {}).get("last_config") or list(_home_config(self.cell, it.robot, HOME_JOINTS_DEFAULT))
        nq = len(warm)
        t = req["target"]
        goal = np.array([float(t["x"]), float(t["y"]), float(t["z"])])
        ori = req.get("orientation") or {}
        ik_ori = {}
        if req.get("constrain_orientation") and ori:
            ik_ori = {"qx": float(ori.get("x", 0.0)), "qy": float(ori.get("y", 0.707)), "qz": float(ori.get("z", 0.0)),
                      "qw": float(ori.get("w", 0.707)), "constrain_orientation": True}
        r = ik(self.cell, it.robot, float(goal[0]), float(goal[1]), float(goal[2]), warm_start_joints=list(warm),
               **ik_ori)
        if not r.get("success"):
            return {"last_config": None, "last_pose": None, "ik_failed": True}
        end = [float(v) for v in r["joint_positions"][:nq]]
        av = np.asarray(req.get("approach_vector", [0, 0, -1]), float)
        av = av / (np.linalg.norm(av) + 1e-9)
        pose = None
        if ik_ori:
            try:
                off = float(req.get("approach_offset_m", it.call_kw.get("approach_offset_m", 0.1)))
            except (TypeError, ValueError):
                off = 0.1
            if not off > 0.0:
                off = float(it.call_kw.get("approach_offset_m", 0.1))
            via_xyz = goal - av * off
            rv = ik(self.cell, it.robot, float(via_xyz[0]), float(via_xyz[1]), float(via_xyz[2]),
                    warm_start_joints=list(warm), **ik_ori)
            if rv.get("success"):
                w = list(rv["joint_positions"][:nq])
                ok = True
                for i in range(1, 7):
                    a = i / 6
                    p = via_xyz * (1.0 - a) + goal * a
                    ri = ik(self.cell, it.robot, float(p[0]), float(p[1]), float(p[2]), warm_start_joints=w, **ik_ori)
                    if not ri.get("success"):
                        ok = False
                        break
                    w = ri["joint_positions"]
                if ok:
                    end = [float(v) for v in w[:nq]]
            pose = {"target": goal.tolist(), "approach_vector": av.tolist(), "orientation": dict(ori)}
        return {"last_config": end, "last_pose": pose}

    def _chain_states(self, robot: str, start_idx: int = 0) -> None:
        """(Re)compute the expected state_in of every item of ``robot`` from ``start_idx`` on."""
        items = [i for i in self.seq[robot] if i.status not in ("dropped", "failed")]
        st = {"last_config": None, "last_pose": None}
        for k, it in enumerate(items):
            if k >= start_idx and it.status != "ok":
                it.state_in = copy.deepcopy(st)
            if it.status == "ok" and it.state_out is not None:
                st = it.state_out
            elif k >= start_idx:
                st = self._predict_out(it, st)
                it.__dict__["predicted_out"] = (copy.deepcopy(it.state_in), st)
                if st.get("ik_failed"):
                    st = {"last_config": None, "last_pose": None, "ik_failed": True}
            else:
                st = it.state_out or self._predict_out(it, st)

    # -- task submission ---------------------------------------------------------------------------------------
    def _submit_plan(self, key, states: dict, calls: list, stop_on_fail=True):
        self.counts["plan_tasks"] += 1
        task = {"key": key, "states": states, "seed": next(self.seed), "calls": calls, "stop_on_fail": stop_on_fail}
        return self.pool.submit(W.plan_task, task)

    def _call(self, it: Item, req: dict | None = None, **kw) -> dict:
        return {"motion_requests": [req or it.request], **(kw or it.call_kw)}

    # -- 2b. plan everything -----------------------------------------------------------------------------------
    def _plan_all(self) -> None:
        self.futs: dict = {}          # future -> ("leg", item, version) | ("cand", rec, name)
        self.recs: dict[int, dict] = {}
        for r in self.robots:
            for it in self.seq[r]:
                self._submit_leg(it)
        self._drain()

    def _submit_leg(self, it: Item) -> None:
        it.version += 1
        it.status = "pending"
        f = self._submit_plan(("leg", it.uid, it.version), {it.robot: it.state_in}, [self._call(it)])
        self.futs[f] = ("leg", it, it.version)
        self.ev("submit", op=it.op_id, robot=it.robot, v=it.version)

    def _drain(self) -> None:
        while self.futs:
            done, _ = wait(list(self.futs), return_when=FIRST_COMPLETED)
            for f in done:
                tag = self.futs.pop(f)
                try:
                    res = f.result()
                except Exception as exc:  # noqa: BLE001 -- a dead worker: treat as a failed task
                    res = {"ok": False, "error": f"{type(exc).__name__}: {exc}", "calls": []}
                if tag[0] == "leg":
                    self._on_leg(tag[1], tag[2], res)
                elif tag[0] == "cand":
                    self._on_candidate(tag[1], tag[2], res)
                elif tag[0] == "decision":
                    self._on_decision(tag[1], res)

    def _on_leg(self, it: Item, version: int, res: dict) -> None:
        if version != it.version or it.status == "dropped":
            return                                    # superseded by a re-plan
        call = (res.get("calls") or [{}])[-1]
        r = call.get("result") or {}
        self.ev("leg_done", op=it.op_id, robot=it.robot, ok=res.get("ok"), wall=res.get("wall_s"),
                fails=[x.get("reason_code") for x in r.get("failures") or []], err=res.get("error"))
        if res.get("ok"):
            self._accept(it, r["trajectories"][it.robot], call["state_out"][it.robot], "plan")
            return
        fails = r.get("failures") or [{"error": res.get("error")}]
        it.failures.append(fails)
        it.reason = next((x.get("reason_code") for x in fails if x.get("reason_code")), None)
        self._start_recovery(it, it.reason)

    def _accept(self, it: Item, traj: list, state_out: dict, strategy: str, extra_items: list | None = None,
                expected: dict | None = None) -> None:
        """Accept a planned leg. ``expected``: the end state the arm's later legs were planned from (default: the
        IK prediction from the leg's own start)."""
        from agents.trajectory.tools import trajectory_cache as tc
        if expected is None:
            expected = self._expected_out(it)
        it.traj = traj
        it.state_out = state_out
        it.status = "ok"
        it.strategy = strategy
        tc.cache_trajectory(self.cell, it.robot, it.op_id, traj)
        for x in extra_items or []:
            tc.cache_trajectory(self.cell, x.robot, x.op_id, x.traj)
        if expected is not None and not _state_eq(expected, state_out):
            # The leg ends somewhere else than predicted (a via moved the IK warm start): every later leg of this
            # arm was planned from the wrong start -> re-predict and re-plan them (in parallel).
            self._replan_downstream(it)

    def _expected_out(self, it: Item) -> dict:
        pin_, pout = it.__dict__.get("predicted_out") or (None, None)
        if pout is not None and _state_eq(pin_, it.state_in):
            return pout                         # computed by _chain_states for this very start state
        return self._predict_out(it, it.state_in or {})

    def _replan_downstream(self, it: Item) -> None:
        items = [i for i in self.seq[it.robot] if i.status not in ("dropped", "failed")]
        k = items.index(it)
        later = items[k + 1:]
        if not later:
            return
        for x in later:
            if x.status in ("ok", "pending", "failed", "recovering"):
                x.status = "pending"
        self._chain_states(it.robot, k + 1)
        for x in later:
            if x.status == "pending":
                self.counts["downstream_replans"] += 1
                self._submit_leg(x)
        self.ev("downstream_replan", robot=it.robot, after=it.op_id, n=len(later))

    # -- 3. recovery ---------------------------------------------------------------------------------------------
    def _prev_item(self, it: Item) -> Item | None:
        items = [i for i in self.seq[it.robot] if i.status not in ("dropped", "failed")]
        k = items.index(it)
        return items[k - 1] if k > 0 else None

    def _candidate_calls(self, it: Item, name: str) -> tuple[list, list]:
        """(calls, extra items) of a recovery candidate."""
        req = it.request
        tgt = req["target"]
        if name == "retry":
            return [self._call(it)], []
        if name in ("via_above", "via_high"):
            dz = 0.25 if name == "via_above" else 0.40
            via = Item(next(self.uid), it.robot, f"{it.op_id}_VIA{'H' if name == 'via_high' else ''}", "move",
                       move_request(it.robot, f"{it.op_id}_VIA{'H' if name == 'via_high' else ''}",
                                    (tgt["x"], tgt["y"], tgt["z"] + dz)), it.group, call_kw=self._kw())
            return [self._call(via), self._call(it)], [via]
        if name == "via_lift":
            prev = self._prev_item(it)
            if prev is None:
                p = None
                lp = None
            else:
                lp = (prev.state_out or {}).get("last_pose")
                p = prev.request["target"]
            if p is None:
                return [], []
            via = Item(next(self.uid), it.robot, f"{it.op_id}_LIFT", "move",
                       move_request(it.robot, f"{it.op_id}_LIFT", (p["x"], p["y"], p["z"] + 0.30)), it.group,
                       call_kw=self._kw())
            return [self._call(via), self._call(it)], [via]
        if name == "offset_long":
            r2 = {**req, "approach_offset_m": 0.15, "retreat_offset_m": 0.20}
            return [self._call(it, r2)], []
        if name == "offset_short":
            r2 = {**req, "approach_offset_m": 0.07}
            return [self._call(it, r2)], []
        if name == "slow":
            return [{"motion_requests": [req], **self._kw(), "velocity_scale": 0.35}], []
        raise ValueError(name)

    def _start_recovery(self, it: Item, reason: str | None) -> None:
        it.status = "recovering"
        it.attempts += 1
        self.counts["recoveries"] += 1
        ladder = LADDER.get(reason, LADDER[None])
        rec = {"item": it, "version": it.version, "reason": reason, "ladder": ladder, "cands": {}, "pending": set(),
               "decision": None, "decision_done": False, "t0": time.perf_counter()}
        self.recs[it.uid] = rec
        for name in ladder:
            calls, extra = self._candidate_calls(it, name)
            if not calls:
                continue
            f = self._submit_plan(("cand", it.uid, name), {it.robot: it.state_in}, calls)
            self.futs[f] = ("cand", rec, name)
            rec["cands"][name] = {"extra": extra, "res": None, "calls": calls}
            rec["pending"].add(name)
        self.ev("recovery_start", op=it.op_id, robot=it.robot, reason=reason, cands=list(rec["cands"]))
        # the decision is asked concurrently with the candidate planning (it only selects among the results)
        q = Question("recovery", f"Planner failure on {it.op_id} ({it.robot}), reason_code={reason}. Which recovery "
                     "strategy should be applied?", [(n, STRATEGIES[n]) for n in rec["cands"]],
                     rule=next(iter(rec["cands"]), "retry"))
        state = self.state_text(focus=f"failed leg {it.op_id} on {it.robot}: reason_code={reason}; failures="
                                f"{json.dumps(self._brief_failures(it.failures[-1:]))[:600]}")
        f = self.threads.submit(self.router.decide, state, q)
        self.futs[f] = ("decision", rec)

    @staticmethod
    def _brief_failures(fails):
        out = []
        for fl in fails:
            for x in fl or []:
                out.append({k: x.get(k) for k in ("reason_code", "error", "tool_rise_m") if x.get(k) is not None})
        return [{**o, "error": str(o.get("error", ""))[:160]} for o in out]

    def _on_candidate(self, rec: dict, name: str, res: dict) -> None:
        rec["cands"][name]["res"] = res
        rec["pending"].discard(name)
        self.ev("cand_done", op=rec["item"].op_id, cand=name, ok=res.get("ok"), wall=res.get("wall_s"))
        self._maybe_resolve(rec)

    def _on_decision(self, rec: dict, res) -> None:
        rec["decision"] = res
        rec["decision_done"] = True
        self._maybe_resolve(rec)

    def _maybe_resolve(self, rec: dict) -> None:
        if rec.get("resolved") or rec["pending"] or not rec["decision_done"]:
            return
        rec["resolved"] = True
        it: Item = rec["item"]
        if it.version != rec["version"] or it.status == "dropped":
            return
        ok = [n for n in rec["ladder"] if n in rec["cands"] and (rec["cands"][n]["res"] or {}).get("ok")]
        d = rec["decision"]
        chosen = d.choice if d is not None else None
        if ok:
            final = chosen if chosen in ok else ok[0]
            if d is not None and final != chosen:
                self.router.note_override(d, final, f"{chosen} did not plan; first successful in ladder")
            self._apply_candidate(it, final, rec["cands"][final])
            self.ev("recovered", op=it.op_id, robot=it.robot, strategy=final, ok=ok,
                    s=round(time.perf_counter() - rec["t0"], 3))
            return
        self.ev("recovery_failed", op=it.op_id, robot=it.robot, reason=rec["reason"])
        if it.attempts < int(self.cfg.get("max_recovery_rounds", 2)):
            # second round: all candidates again (other seeds) -- umarell's "~2 attempts on that arm"
            it.version += 1
            self._start_recovery(it, rec["reason"])
            return
        self._reallocate(it)

    def _apply_candidate(self, it: Item, name: str, cand: dict) -> None:
        expected = self._expected_out(it)          # what the later legs of this arm were planned from
        res = cand["res"]
        extra: list[Item] = cand["extra"]
        calls = res["calls"]
        for x, c in zip(extra, calls[:len(extra)]):
            x.traj = c["result"]["trajectories"][x.robot]
            x.state_in = it.state_in
            x.state_out = c["state_out"][x.robot]
            x.status = "ok"
            x.strategy = f"{name}:via"
        last = calls[-1]
        if extra:
            # insert the via(s) before the weld in the arm's sequence; the weld now starts at the via's goal
            k = self.seq[it.robot].index(it)
            self.seq[it.robot][k:k] = extra
            it.state_in = extra[-1].state_out
        final_call = cand["calls"][-1]
        it.request = final_call["motion_requests"][0]
        it.call_kw = {k: v for k, v in final_call.items() if k != "motion_requests"}
        self._accept(it, last["result"]["trajectories"][it.robot], last["state_out"][it.robot], name, extra,
                     expected=expected)

    def _reallocate(self, it: Item) -> None:
        """All candidates failed twice on this arm: umarell's ``reallocate_operation``, plan at the other arm's end."""
        tried = sorted(set(it.tried_robots + [it.robot]))
        robots_order = [{"robot_ns": r, "operation_ids": [i.op_id for i in self.seq[r] if i.status != "dropped"]}
                        for r in self.robots]
        ra = S.call("reallocate_operation", cell_id=self.cell, operation=it.op, tried_robots=tried,
                    robots=robots_order) if it.op else {"success": False, "error": "not a weld"}
        self.ev("reallocate", op=it.op_id, tried=tried, result={k: ra.get(k) for k in ("success", "to_robot",
                                                                                          "reason_code", "error")
                                                                if k in ra})
        new = ra.get("to_robot")
        if not ra.get("success") or not new or new in tried:
            it.status = "failed"
            self.ev("unreachable", op=it.op_id, tried=tried)
            self._replan_after_removal(it.robot)
            return
        self.counts["reallocations"] += 1
        it.status = "dropped"
        moved = Item(next(self.uid), new, it.op_id, "weld", weld_request(new, it.op, self.cfg.get("request_extra")),
                     group=max([i.group for s in self.seq.values() for i in s] + [0]) + 1, op=it.op,
                     call_kw=self._kw(), tried_robots=tried)
        self.seq[it.robot].remove(it)
        # downstream legs of the old arm now start where the previous leg ended -> re-plan them
        self._replan_after_removal(it.robot)
        self.seq[new].append(moved)
        self._chain_states(new)
        self._submit_leg(moved)

    def _replan_after_removal(self, robot: str) -> None:
        """An item left the arm's chain (reallocated or unreachable): the first later leg whose start no longer
        matches the chain, and every leg after it, are re-planned."""
        items = [i for i in self.seq[robot] if i.status not in ("dropped", "failed")]
        st = {"last_config": None, "last_pose": None}
        for x in items:
            if x.status == "ok" and _state_eq(x.state_in, st):
                st = x.state_out
                continue
            # start changed: re-plan this and every later leg
            idx = items.index(x)
            for y in items[idx:]:
                y.status = "pending"
            self._chain_states(robot, idx)
            for y in items[idx:]:
                self.counts["downstream_replans"] += 1
                self._submit_leg(y)
            return

    # -- 4. validate + chain ----------------------------------------------------------------------------------
    def _validate_and_chain(self) -> None:
        for rnd in range(3):
            bad = []
            welds = [i for r in self.robots for i in self.seq[r] if i.kind == "weld" and i.status == "ok"]
            for it in welds:
                op = it.op
                ori = op.get("weld_orientation") or op.get("orientation")
                if not ori:
                    continue
                va = S.call("validate_approach", cell_id=self.cell, robot_ns=it.robot, expected_orientation=ori,
                            operation_id=it.op_id, target_position=list(op["position"]),
                            approach_vector=list(op.get("approach_vector", [0, 0, -1])))
                it.__dict__["approach"] = {k: va.get(k) for k in ("matches", "orientation_error_deg",
                                                                  "position_error_m",
                                                                  "approach_direction_error_deg") if k in va}
                if not va.get("matches"):
                    bad.append(it)
            chain = S.call("chain_trajectories", cell_id=self.cell, robots=self._chain_arg())
            self.ev("validate", round=rnd, approach_bad=[i.op_id for i in bad], chained=chain.get("chained"),
                    breaks=chain.get("breaks"))
            self.chain = chain
            if not bad and chain.get("chained"):
                return
            for it in bad:
                it.failures.append([{"reason_code": "approach_mismatch", "error": json.dumps(it.approach)}])
                self._start_recovery(it, "approach_mismatch")
            for br in chain.get("breaks") or []:
                op = br.get("to_operation_id")
                for r in self.robots:
                    for x in self.seq[r]:
                        if x.op_id == op and x.status == "ok":
                            x.status = "pending"
                            self._chain_states(r)
                            self._submit_leg(x)
            self._drain()

    def _chain_arg(self) -> list:
        return [{"robot_ns": r, "operation_ids": [i.op_id for i in self.seq[r] if i.status == "ok"]}
                for r in self.robots if any(i.status == "ok" for i in self.seq[r])]

    # -- 5. assemble -------------------------------------------------------------------------------------------
    def _steps(self, delays: dict | None = None, modes: dict | None = None, with_groups: bool = False):
        """Plan steps from the MRTA pairing: per group, the arms' last items (welds) in one step (requested
        PARALLEL), earlier items of the group (vias / parks) in steps before it, right-aligned across arms."""
        groups = sorted({i.group for r in self.robots for i in self.seq[r] if i.status == "ok"})
        steps, sgroups = [], []
        for g in groups:
            per = {r: [i for i in self.seq[r] if i.status == "ok" and i.group == g] for r in self.robots}
            depth = max(len(v) for v in per.values())
            for k in range(depth):
                ents = []
                for r in self.robots:
                    lst = per[r]
                    pad = depth - len(lst)        # right-align: every arm's last item lands in the last sub-step
                    if k - pad >= 0:
                        x = lst[k - pad]
                        e = {"robot_ns": r, "operation_id": x.op_id, "operation_type": x.request["operation_type"]}
                        if delays and x.op_id in delays:
                            e["start_delay_s"] = delays[x.op_id]
                        ents.append(e)
                if ents:
                    key = "+".join(e["operation_id"] for e in ents)
                    mode = (modes or {}).get(key, "PARALLEL")
                    steps.append({"mode": mode if len(ents) > 1 else "SEQUENTIAL", "robots": ents})
                    sgroups.append(g)
        return (steps, sgroups) if with_groups else steps

    def _snapshot(self) -> dict:
        trajs, last_cfg, last_pose = [], [], []
        for r in self.robots:
            items = [i for i in self.seq[r] if i.status == "ok"]
            for i in items:
                trajs.append({"robot_ns": r, "operation_id": i.op_id, "trajectory": i.traj})
            if items:
                so = items[-1].state_out or {}
                if so.get("last_config"):
                    last_cfg.append({"robot_ns": r, "joints": so["last_config"]})
                if so.get("last_pose"):
                    last_pose.append({"robot_ns": r, "pose": so["last_pose"]})
        return {"trajectories": trajs, "last_config": last_cfg, "last_pose": last_pose}

    def _submit_assemble(self, key, plan: dict):
        self.counts["assemble_tasks"] += 1
        return self.pool.submit(W.assemble_task, {"key": key, "snapshot": self._snapshot(), "plan": plan,
                                                  "seed": next(self.seed)})

    def _assemble_variants(self, rnd, home_first: bool):
        """Assemble with home_at_end true and false concurrently. Returns (best | None, refusal result)."""
        steps = self._steps(modes=self.modes)
        variants = {"home": {"home_at_end": True, "steps": steps}, "nohome": {"home_at_end": False, "steps": steps}}
        order = ["home", "nohome"] if home_first else ["nohome", "home"]
        futs = {k: self._submit_assemble(("asm", rnd, k), v) for k, v in variants.items()}
        res = {k: f.result() for k, f in futs.items()}
        for k in order:
            r = res[k]
            self.ev("assemble", round=rnd, variant=k, ok=r.get("ok"), wall=r.get("wall_s"),
                    error=str((r.get("result") or {}).get("error") or r.get("error") or "")[:200],
                    makespan=(r.get("schedule") or {}).get("makespan_s"),
                    blocked=self._brief_blocked((r.get("result") or {}).get("blocked_steps") or []),
                    home_step=str((r.get("result") or {}).get("home_step") or "")[:300])
        for k in order:
            if res[k].get("ok"):
                if k != order[0]:
                    self.ev("home_fallback", used=k)
                return (k, variants[k], res[k]), None
        return None, res[order[0]].get("result") or {}

    @staticmethod
    def _first_blocked(refusal: dict) -> int | None:
        idx = [b.get("step") for b in (refusal or {}).get("blocked_steps") or [] if b.get("step") is not None]
        return min(idx) if idx else None

    def _assemble(self) -> None:
        # discrete choices before the first assembly: step modes (one decider request for all) + home_at_end
        steps = self._steps()
        two_arm = [s for s in steps if len(s["robots"]) > 1]
        qs = [Question("step_mode", f"Step {'+'.join(e['operation_id'] for e in s['robots'])}: both arms in the "
                       "same step. Run them PARALLEL (assembler gate verifies) or SEQUENTIAL?",
                       [("PARALLEL", "both arms move at once; umarell's gate serialises it if the pair breaches"),
                        ("SEQUENTIAL", "one arm after the other")], rule="PARALLEL") for s in two_arm]
        qs.append(Question("home_at_end", "Should the tree end with all arms returning HOME (home_at_end)?",
                           [("true", "append the coupled, gated MoveHome step"),
                            ("false", "end at the last operation (one-way last weld)")], rule="true", kind="noul"))
        decs = self.router.decide_many(self.state_text(focus="all legs planned and validated; about to assemble"), qs)
        self.modes = {"+".join(e["operation_id"] for e in s["robots"]): d.choice for s, d in zip(two_arm, decs)}
        self.home_first = decs[-1].choice == "true"
        best, refusal = None, None
        rounds = int(self.cfg.get("max_assemble_rounds", 6))
        for rnd in range(rounds):
            best, refusal = self._assemble_variants(rnd, self.home_first)
            if best is not None:
                break
            if not self._restructure(refusal, rnd):
                break
            if getattr(self, "_restructure_best", None) is not None:
                best = self._restructure_best
                break
        if best is None:
            self.result.update(success=False, error="assembly refused in every round",
                               last_refusal={"error": str((refusal or {}).get("error"))[:500],
                                             "blocked": self._brief_blocked((refusal or {}).get("blocked_steps")
                                                                            or [])})
            return
        best = self._improve_with_delays(best)
        q = Question("accept_tree", "The behaviour tree assembled and validated. Accept it as the plan?",
                     [("true", "accept and save"), ("false", "keep searching for a better tree")], rule="true",
                     kind="noul")
        d = self.router.decide(self.state_text(focus=f"tree assembled: makespan "
                                               f"{(best[2].get('schedule') or {}).get('makespan_s')} s"), q)
        if d.choice != "true":
            self.router.note_override(d, "true", "no further search implemented beyond the delay pass; accepted")
        self.best = best

    # -- refusal: time-separate the conflicting part of the program ---------------------------------------------
    def _backup(self) -> dict:
        return {r: [copy.copy(i) for i in self.seq[r]] for r in self.robots}

    def _restore(self, b: dict) -> None:
        from agents.trajectory.tools import trajectory_cache as tc
        self.seq = {r: list(v) for r, v in b.items()}
        for r in self.robots:
            for i in self.seq[r]:
                if i.status == "ok" and i.traj:
                    tc.cache_trajectory(self.cell, r, i.op_id, i.traj)

    def _restructure(self, refusal: dict, rnd: int) -> bool:
        """umarell refused the tree because, from some step on, the arms block each other even one at a time
        (or an idle arm parked at its last weld blocks the other). The fix Opus applied in the logged runs
        (s2_a4: R2_PARK, R1_PARK), done in code: from the first blocked step, one arm FIRST does all its remaining
        operations while the other is parked out of the way (toward its own base, raised); then the first arm parks
        and the second finishes. Which arm goes first is a decision (rule: the arm with less remaining work; the
        other order is tried if the first does not get the tree past the blocked step). Park candidates are planned
        in parallel; Opus generates park points only when no code candidate plans."""
        self._restructure_best = None
        s = self._first_blocked(refusal)
        if s is None or len(self.robots) != 2:
            return False
        steps, sgroups = self._steps(modes=self.modes, with_groups=True)
        if s - 1 >= len(sgroups):
            return False
        g_s = sgroups[s - 1]
        rem = {r: [i for i in self.seq[r] if i.status == "ok" and i.group >= g_s] for r in self.robots}
        rule_first = min(self.robots, key=lambda r: (len(rem[r]), self.robots.index(r)))
        opts = [(f"{r}_first", f"{r} finishes its remaining {len(rem[r])} operation(s) while the other arm is parked; "
                 f"then {r} parks and the other arm finishes") for r in self.robots]
        opts.append(("give_up", "stop and report the refusal"))
        q = Question("assemble_reaction", f"assemble_behaviour_tree refused the tree from step {s} on "
                     f"({str(refusal.get('error'))[:200]}). How should the conflicting part be restructured?",
                     opts, rule=f"{rule_first}_first")
        d = self.router.decide(self.state_text(focus=f"assembly refused, blocked steps "
                                               f"{json.dumps(self._brief_blocked(refusal.get('blocked_steps') or []))[:900]}"),
                               q)
        first = d.choice[:-len("_first")] if d.choice.endswith("_first") else rule_first
        if d.choice == "give_up":
            self.router.note_override(d, f"{rule_first}_first", "give_up is only taken after both orders failed")
        order = [first] + [r for r in self.robots if r != first]
        best_progress, best_state = s, None
        backup = self._backup()
        for a in order:
            b = next(r for r in self.robots if r != a)
            ok = self._apply_split(a, b, g_s, rnd)
            if ok:
                found, ref2 = self._assemble_variants(f"{rnd}.{a}", self.home_first)
                if found is not None:
                    self._restructure_best = found
                    self.ev("restructured", first=a, at_step=s, ok=True)
                    return True
                s2 = self._first_blocked(ref2)
                self.ev("restructured", first=a, at_step=s, ok=False, next_blocked=s2,
                        error=str(ref2.get("error"))[:200])
                if s2 is not None and s2 > best_progress:
                    best_progress, best_state = s2, self._backup()
            self._restore(backup)
        if best_state is not None:
            self._restore(best_state)
            return True              # progress: the caller re-assembles and may restructure again further on
        return False

    def _units(self, items: list) -> list[list]:
        """Consecutive items with the same group (a via + its weld) stay together."""
        out: list[list] = []
        for i in items:
            if out and out[-1][-1].group == i.group:
                out[-1].append(i)
            else:
                out.append([i])
        return out

    def _apply_split(self, a: str, b: str, g_s: float, rnd) -> bool:
        before_b = [i for i in self.seq[b] if i.status == "ok" and i.group < g_s]
        rem_a = [i for i in self.seq[a] if i.status == "ok" and i.group >= g_s]
        rem_b = [i for i in self.seq[b] if i.status == "ok" and i.group >= g_s]
        from agents.trajectory.tools import trajectory_cache as tc
        if before_b and before_b[-1].strategy == "park" and len(before_b) > 1:
            # b already stands at a park from an earlier split: re-plan the park from the leg before it instead of
            # stacking a second park on top
            old_park = before_b.pop()
            old_park.status = "dropped"
            self.seq[b].remove(old_park)
            self.counts["park_moves"] -= 1
        park_b = None
        if before_b and rem_a:  # an arm that has not moved yet stands at HOME and is not gated as a bystander
            park_b = self._plan_park(b, before_b[-1], f"{rnd}b", others={a: [i.traj for i in rem_a]})
            if park_b is None:
                return False
        # regroup from g_s on: [park_b + a's first unit], a's units..., [park_a + b's first unit], b's units...
        ua, ub = self._units(rem_a), self._units(rem_b)
        g = g_s
        for k, u in enumerate(ua):
            for i in u:
                i.group = g + k
        g_after_a = g + max(len(ua), 1)
        for k, u in enumerate(ub):
            for i in u:
                i.group = g_after_a + k
        if park_b is not None:
            park_b.group = g
            k = self.seq[b].index(before_b[-1])
            self.seq[b].insert(k + 1, park_b)
            tc.cache_trajectory(self.cell, b, park_b.op_id, park_b.traj)
            self.counts["park_moves"] += 1
            if rem_b:
                # b's remaining legs now start at its park: re-plan them all at once (recovery machinery included)
                for y in rem_b:
                    y.status = "pending"
                k = [i for i in self.seq[b] if i.status not in ("dropped", "failed")].index(park_b)
                self._chain_states(b, k + 1)
                for y in rem_b:
                    self.counts["downstream_replans"] += 1
                    self._submit_leg(y)
                self._drain()
                if any(y.status != "ok" for y in rem_b):
                    return False
        last_a = ([i for i in self.seq[a] if i.status == "ok"] or [None])[-1]
        park_a = None
        if rem_b and last_a is not None and not (last_a.strategy == "park" and not rem_a):
            park_a = self._plan_park(a, last_a, f"{rnd}a", others={b: [i.traj for i in rem_b]})
            if park_a is None:
                return False
            park_a.group = g_after_a
            self.seq[a].append(park_a)
            tc.cache_trajectory(self.cell, a, park_a.op_id, park_a.traj)
            self.counts["park_moves"] += 1
        self.ev("split", first=a, second=b, at_group=g_s, park_a=park_a.op_id if park_a else None,
                park_b=park_b.op_id if park_b else None,
                clearance_a=park_a.__dict__.get("clearance") if park_a else None,
                clearance_b=park_b.__dict__.get("clearance") if park_b else None)
        return True

    @staticmethod
    def _brief_blocked(blocked):
        out = []
        for b in blocked:
            o = {k: b.get(k) for k in ("step", "arms", "blocked_arms", "blocked_by_parked", "bystanders", "order")}
            o["parallel_min"] = (b.get("parallel") or {}).get("min_clearance_m")
            o["serial"] = [(x.get("robot_ns"), x.get("min_clearance_m")) for x in b.get("serial_attempts") or []]
            out.append(o)
        return out

    def _park_candidates(self, ns: str, anchor: Item) -> list:
        from agents.trajectory.tools.planning_tools import _home_tcp, _cell_robots
        rob = _cell_robots(self.cell).get(ns) or {}
        base = np.asarray(rob.get("base_position") or rob.get("position") or [0, 0, 0], float)[:3]
        t = anchor.request["target"]
        p = np.array([t["x"], t["y"], t["z"]], float)
        d = base[:2] - p[:2]
        d = d / (np.linalg.norm(d) + 1e-9)
        cands = []
        for back in (0.3, 0.6, 0.9, 1.2):
            for up in (0.3, 0.5, 0.7):
                q = p.copy()
                q[:2] += d * back
                q[2] += up
                cands.append(q.round(3).tolist())
        home = _home_tcp(self.cell, [ns]).get(ns)
        if home:
            cands.append([float(v) for v in home])
        return cands

    def _park_clearance(self, ns: str, q: list, others: dict) -> float:
        """Min arm-to-arm clearance of the other arm's legs ``others`` ({ns: [traj, ...]}) with arm ``ns`` standing
        at ``q`` (umarell's validate_trajectory_set, the gate's own check)."""
        from agents.trajectory.tools.behaviour_tree import _parked
        worst = float("inf")
        for o, trajs in others.items():
            for tr in trajs:
                if not tr:
                    continue
                r = S.call("validate_trajectory_set", cell_id=self.cell, trajectories={o: tr, ns: _parked(q)},
                           suggest_delay=False)
                m = r.get("min_clearance_m")
                if m is not None:
                    worst = min(worst, float(m))
        return worst

    def _plan_park(self, ns: str, anchor: Item, tag: str, others: dict | None = None) -> Item | None:
        """Plan every park candidate for arm ``ns`` from the end of ``anchor`` concurrently. Among those that plan,
        keep the first (nearest) whose standing pose clears the other arm's legs ``others`` by the warning distance
        + 5 cm (else the one with the largest clearance) -- the search Opus did by hand by nudging park targets.
        Opus generates candidates only if no code candidate plans."""
        from agents.trajectory.tools.planning_tools import _load_collision_config
        need = float(_load_collision_config(self.cell)["warning_distance"]) + 0.05
        cands = self._park_candidates(ns, anchor)
        for attempt in range(2):
            futs = []
            for j, xyz in enumerate(cands):
                op_id = f"{ns.upper()}_PARK{tag}{attempt}{j:02d}"
                pk = Item(next(self.uid), ns, op_id, "move", move_request(ns, op_id, xyz), anchor.group,
                          call_kw=self._kw())
                pk.state_in = anchor.state_out
                self.counts["plan_tasks"] += 1
                task = {"key": ("park", op_id), "states": {ns: anchor.state_out}, "seed": next(self.seed),
                        "calls": [self._call(pk)], "stop_on_fail": True,
                        "score_against": {"ns": ns, "others": others} if others else None}
                futs.append((pk, self.pool.submit(W.plan_task, task)))   # plans AND scores the pose in the worker
            planned = []
            for pk, f in futs:
                r = f.result()
                if r.get("ok"):
                    c = r["calls"][-1]
                    pk.traj = c["result"]["trajectories"][ns]
                    pk.state_out = c["state_out"][ns]
                    pk.status = "ok"
                    pk.strategy = "park"
                    pk.__dict__["clearance"] = r.get("clearance") if others else None
                    planned.append(pk)
                self.ev("park_cand", robot=ns, op=pk.op_id, target=pk.request["target"], ok=r.get("ok"),
                        clearance=pk.__dict__.get("clearance"))
            if planned:
                if not others:
                    return planned[0]
                good = [pk for pk in planned if pk.clearance >= need]
                if good:
                    return good[0]
                if attempt == 1 or self.llm is None:
                    return max(planned, key=lambda x: x.clearance)
            if attempt == 0 and self.llm is not None:
                best = max((pk.clearance for pk in planned), default=None)
                st = self.state_text(focus=f"arm {ns} must park out of the way after {anchor.op_id}; code park "
                                     f"candidates {cands}: {len(planned)} planned, best clearance to the other arm's "
                                     f"remaining legs {best} m (need {need:.2f} m)")
                pts, _ = self.router.generate_points(
                    st, f"Give 1-3 Cartesian park points (TCP, world frame, metres) for {ns} after {anchor.op_id}: "
                        f"clear of the other arm's remaining work by at least {need:.2f} m, toward {ns}'s own base "
                        "and raised, reachable within the joint limits.")
                fallback = max(planned, key=lambda x: x.clearance) if planned else None
                cands = [[p["x"], p["y"], p["z"]] for p in pts]
                if not cands:
                    return fallback
                self._park_fallback = fallback
            else:
                return None
        return None

    def _improve_with_delays(self, best):
        """Serialised two-arm steps: try a start delay that keeps them PARALLEL (umarell's find_start_delays);
        assemble the delayed variant and keep it if the makespan drops (decision ``start_delay``)."""
        name, plan, res = best
        gates = (res.get("result") or {}).get("step_gates") or []
        serial = [g for g in gates if g.get("mode") == "SEQUENTIAL" and len(g.get("arms") or []) > 1]
        if not serial:
            return best
        steps = plan["steps"]
        delays = {}
        jobs = []
        for g in serial:               # one find_start_delays per serialised step, all in the pool at once
            idx = int(g["step"]) - 1
            if idx < 0 or idx >= len(steps):
                continue
            ents = steps[idx]["robots"]
            legs = {}
            for e in ents:
                it = next((i for i in self.seq[e["robot_ns"]] if i.op_id == e["operation_id"]), None)
                if it is not None and it.traj:
                    legs[e["robot_ns"]] = it.traj
            jobs.append((g, ents, self.pool.submit(W.func_task, {"key": g["step"], "fn": "find_start_delays",
                                                                 "legs": legs})))
        for g, ents, f in jobs:
            r = f.result()
            fd = r.get("value") if r.get("ok") else None
            if not r.get("ok"):
                self.ev("delay_error", step=g["step"], error=str(r.get("error"))[:200])
            if fd:
                for e in ents:
                    dl = fd["start_delays"].get(e["robot_ns"], 0.0)
                    if dl > 0:
                        delays[e["operation_id"]] = dl
        if not delays:
            return best
        plan2 = {**plan, "steps": self._steps(delays=delays, modes=self.modes)}
        r2 = self._submit_assemble(("delay",), plan2).result()
        m1 = (res.get("schedule") or {}).get("makespan_s")
        m2 = (r2.get("schedule") or {}).get("makespan_s") if r2.get("ok") else None
        self.ev("delay_variant", delays=delays, ok=r2.get("ok"), makespan=m2, base_makespan=m1)
        q = Question("start_delay", f"Serialised two-arm steps can run PARALLEL with start delays {delays}: "
                     f"makespan {m2} s vs {m1} s serialised. Use the delayed variant?",
                     [("true", "use start_delay_s (temporal interlock), step stays PARALLEL"),
                      ("false", "keep the serialised steps")],
                     rule="true" if (m2 is not None and m1 is not None and m2 < m1 - 1e-6) else "false", kind="noul")
        d = self.router.decide(self.state_text(focus="start-delay variant evaluated"), q)
        use = d.choice == "true" and m2 is not None
        if d.choice == "true" and m2 is None:
            self.router.note_override(d, "false", "delayed variant was refused by the assembler")
        return (name + "+delay", plan2, r2) if use else best

    # -- 6. save ------------------------------------------------------------------------------------------------
    def _save(self) -> None:
        if getattr(self, "best", None) is None:
            self._save_failed()
            return
        name, plan, res = self.best
        tree = res["result"]["behaviour_tree"]
        tree_id = self.cfg.get("tree_id") or tree.get("tree_id")
        tree["tree_id"] = tree_id
        from agents.trajectory.tools import plan_store
        saved = plan_store.save_plan(self.cell, tree)
        loaded = plan_store.get_plan(self.cell, saved) if saved else None
        val = S.call("validate_behaviour_tree", tree=(loaded or {}).get("behaviour_tree") or tree)
        src = Path(os.environ["CELL_BASE_PATH"]) / self.cell / "agentic_planner" / "plans"
        files = sorted(src.glob(f"{saved}*")) if saved else []
        out_plan = None
        for f in files:
            dst = self.run_dir / f.name
            shutil.copy2(f, dst)
            if f.suffix == ".json":
                out_plan = dst
        self.result.update(
            success=bool(saved and val.get("valid")), tree_id=saved, plan_file=str(out_plan) if out_plan else None,
            variant=name, plan=plan, schedule=res.get("schedule"),
            step_gates=[{k: g.get(k) for k in ("step", "requested_mode", "mode", "arms", "start_delays")}
                        for g in (res["result"].get("step_gates") or [])],
            serialised_steps=res["result"].get("serialised_steps"),
            validate_behaviour_tree={k: val.get(k) for k in ("valid", "errors", "warnings") if k in val},
            plan_store_get_plan_ok=bool(loaded), chain=self.chain.get("chained") if hasattr(self, "chain") else None,
            approach={i.op_id: i.__dict__.get("approach") for r in self.robots for i in self.seq[r]
                      if i.kind == "weld"},
            welds_in_tree=sorted(i.op_id for r in self.robots for i in self.seq[r] if i.kind == "weld"
                                 and i.status == "ok"),
            unreachable=sorted(i.op_id for r in self.robots for i in self.seq[r] if i.status == "failed"),
            sequences=self._seq_summary())

    def _seq_summary(self) -> dict:
        return {r: [{"op": i.op_id, "kind": i.kind, "strategy": i.strategy, "status": i.status, "group": i.group,
                     "target": i.request.get("target") if i.kind == "move" else None}
                    for i in self.seq[r]] for r in self.robots}

    def _save_failed(self) -> None:
        self.result.setdefault("success", False)
        self.result["sequences"] = self._seq_summary()

    # -- state text for the decision makers ----------------------------------------------------------------------
    def state_text(self, focus: str = "") -> str:
        lines = [f"Cell {self.cell}: {len(self.robots)} arms ({', '.join(self.robots)}), "
                 f"{len(self.operations)} operations."]
        for r in self.robots:
            parts = []
            for i in self.seq.get(r, []):
                if i.status == "dropped":
                    continue
                tag = {"ok": "ok", "pending": "planning", "recovering": "FAILED-recovering", "failed": "FAILED"}.get(
                    i.status, i.status)
                parts.append(f"{i.op_id}({i.kind},{tag}{'' if i.strategy in ('plan',) else ',' + i.strategy})")
            lines.append(f"{r} sequence (MRTA order): " + ", ".join(parts))
        if focus:
            lines.append("Now: " + focus)
        return "\n".join(lines)
