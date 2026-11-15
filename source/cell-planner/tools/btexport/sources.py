"""Inputs of the exporter -> one ``Source`` object.

  rollout   continuous two-arm motion (t (N,), q (N, 2, 6), weld events) of a per-step controller: the export cuts
            it into executor-consistent synchronous steps (execute.py ``sync_rest``);
  plan      executor-consistent legs that already are a behaviour tree (a umarell plan, a trajgen plan.json):
            the export keeps the tree (any number of robots, gates included) and re-emits it.

``--from`` specs (``resolve``):
  expert[:assign] | stateless | hier:<ckpt>[,la=<mode>|auto] | hier_la:<ckpt> | residual:<ckpt> | e2e:<ckpt> |
  ppo:<ckpt>                              run the controller first (canonical task from HOME, execute.py's rollout +
                                         its home tail); hier_la = hier with the learned lookahead of its la_args.json
  umarell:<plan name | plan.json>         a saved umarell plan
  trajgen:<scene dir | sample name>[,<extra trajgen args>]   run ``python -m trajgen`` first (cached in --work)
  tgrun:<run dir>                         a trajgen run directory (plan.json, report.json, timeline.npz)
  timeline:<npz>                          a Timeline / NTimeline file (+ its weld events); N robots (e.g. tools/ctrl's
                                         smoothed trajgen motions): its scene from meta.source or ``scene=``
  exec:<results/exec_*.json>              an execute.py result (its rollout timeline + home-tail flag)
  plan:<plan.json>                        any plan in umarell's envelope
  a bare path is classified by its content.
"""
from __future__ import annotations

import json
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONTROLLER_KINDS = ("expert", "stateless", "hier", "hier_la", "residual", "e2e", "ppo", "random")


@dataclass
class Source:
    kind: str                         # "rollout" | "plan"
    cell_id: str                      # cell id written into the plan
    robot_ids: tuple
    home: np.ndarray                  # (R, 6)
    spec: str
    scene: Optional[Path] = None      # trajgen scene (N robots / exported umarell cell); None: umarell cell
    task: Optional[Path] = None
    umarell_cell: Optional[str] = None   # the umarell cell the plan runs in (robot1 / robot2, or N arms)
    # rollout
    t: Optional[np.ndarray] = None
    q: Optional[np.ndarray] = None
    weld_events: list = field(default_factory=list)
    at_home: bool = True
    # plan
    plan: Optional[dict] = None
    run_dir: Optional[Path] = None
    meta: dict = field(default_factory=dict)


def _rel(p) -> str:
    p = Path(p).resolve()
    return str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p)


def cell_robot_ids(cell_id: str) -> tuple:
    """The arms of a umarell cell (robots.yaml): robot1 / robot2 in that order for a two-arm cell (env.cell.ROBOT_IDS,
    as before), else the cell's own ids in robots.yaml order (N-arm cells, e.g. robot1..robot4)."""
    from env.cell import ROBOT_IDS, load_cell
    rids = tuple(load_cell(cell_id).robots)
    return tuple(ROBOT_IDS) if set(rids) == set(ROBOT_IDS) else rids


def cell_home(cell_id: str, robot_ids=None) -> np.ndarray:
    """(R, 6) HOME of the cell's arms in ``robot_ids`` order (default robot1 / robot2)."""
    from env.cell import ROBOT_IDS, load_cell
    c = load_cell(cell_id)
    return np.stack([c.robots[r].home for r in (ROBOT_IDS if robot_ids is None else robot_ids)])


# ───────────────────────────────────────── input validation ─────────────────────────────────────────
def _bad(msg: str):
    from tools.btexport.core import Unsupported
    raise Unsupported(f"malformed input: {msg}")


def check_arrays(t, q, ctx: str, n_joints: int = 6) -> None:
    """A rollout / timeline: finite, strictly increasing times; finite (N, R, n_joints) positions."""
    t = np.asarray(t, float)
    q = np.asarray(q, float)
    if t.ndim != 1 or len(t) < 2:
        _bad(f"{ctx}: need >= 2 time samples, got shape {t.shape}")
    if q.ndim != 3 or q.shape[0] != len(t) or q.shape[2] != n_joints:
        _bad(f"{ctx}: positions shape {q.shape}, expected ({len(t)}, R, {n_joints})")
    if not np.all(np.isfinite(t)):
        _bad(f"{ctx}: non-finite time stamps")
    if not np.all(np.diff(t) > 0):
        _bad(f"{ctx}: time stamps not strictly increasing (first at index {int(np.argmax(~(np.diff(t) > 0)))})")
    if not np.all(np.isfinite(q)):
        _bad(f"{ctx}: non-finite joint positions")


def _num(v, ctx, *, lo=None, strict=False, default=None):
    if v is None:
        return default
    try:
        x = float(v)
    except (TypeError, ValueError):
        _bad(f"{ctx}: not a number: {v!r}")
    if not np.isfinite(x) or (lo is not None and (x <= lo if strict else x < lo)):
        _bad(f"{ctx}: {v!r} (must be finite and {'>' if strict else '>='} {lo})")
    return x


def check_plan_input(env: dict, n_joints: int = 6, robot_ids=None) -> dict:
    """Validate a plan envelope before anything is computed from it (P3U round 2: malformed plans raised tracebacks
    or were exported with NaN): every ExecuteTrajectory / MoveHome trajectory reference resolves on the blackboard
    and is non-empty, every waypoint has ``n_joints`` finite positions and a finite time_s, times strictly increase
    within a trajectory (and start >= 0), velocity_scale > 0, hold_duration_s / start_delay_s finite >= 0, robot_ns
    among ``robot_ids`` (when given). Raises Unsupported (CLI exit 2) listing the first problems."""
    from player.bt_plan import node_kind, port
    tree = env.get("behaviour_tree") if isinstance(env, dict) else None
    if not isinstance(tree, dict) or not isinstance(tree.get("root"), dict):
        _bad("no behaviour_tree.root")
    bb = tree.get("blackboard") or {}
    if not isinstance(bb, dict):
        _bad("blackboard is not a mapping")
    probs, n_leaf = [], 0

    def traj_of(node, op):
        """-> (resolved, value): resolved False when a {key} reference is missing (reported here)."""
        ref = port(node, "trajectory")
        if isinstance(ref, str) and len(ref) > 2 and ref[0] == "{" and ref[-1] == "}":
            k = ref[1:-1].strip()
            if k not in bb:
                probs.append(f"{op}: blackboard key {k!r} missing")
                return False, None
            return True, bb[k]
        return True, ref

    def check_traj(traj, op):
        if not isinstance(traj, list) or not traj:
            probs.append(f"{op}: empty trajectory")
            return
        prev = None
        for i, w in enumerate(traj):
            if not isinstance(w, dict) or "positions" not in w or "time_s" not in w:
                probs.append(f"{op}[{i}]: waypoint without positions / time_s")
                return
            pos = w["positions"]
            if not isinstance(pos, (list, tuple)) or len(pos) != n_joints:
                probs.append(f"{op}[{i}]: {len(pos) if isinstance(pos, (list, tuple)) else type(pos).__name__} "
                             f"joint positions, expected {n_joints}")
                return
            try:
                pa = np.asarray(pos, float)
                ts = float(w["time_s"])
            except (TypeError, ValueError):
                probs.append(f"{op}[{i}]: non-numeric waypoint")
                return
            if not (np.all(np.isfinite(pa)) and np.isfinite(ts)):
                probs.append(f"{op}[{i}]: non-finite position / time_s")
                return
            if (i == 0 and ts < 0) or (prev is not None and not ts > prev):
                probs.append(f"{op}[{i}]: time_s {ts} not strictly increasing (previous {prev})")
                return
            prev = ts

    def walk(n):
        nonlocal n_leaf
        k = node_kind(n)
        if k in ("ExecuteTrajectory", "MoveHome"):
            n_leaf += 1
            op = f"{port(n, 'robot_ns')}/{port(n, 'operation_id') if k == 'ExecuteTrajectory' else 'home'}"
            rid = port(n, "robot_ns")
            if robot_ids is not None and rid not in robot_ids:
                probs.append(f"{op}: robot_ns {rid!r} not among {list(robot_ids)}")
            if k == "ExecuteTrajectory":
                try:
                    _num(port(n, "velocity_scale", 1.0), f"{op}: velocity_scale", lo=0.0, strict=True)
                    _num(port(n, "hold_duration_s", 0.0), f"{op}: hold_duration_s", lo=0.0)
                    _num(port(n, "start_delay_s", 0.0), f"{op}: start_delay_s", lo=0.0)
                except ValueError as e:
                    probs.append(str(e).removeprefix("malformed input: "))
                ok, tr = traj_of(n, op)
                if ok:
                    check_traj(tr, op)
            else:
                ref = port(n, "trajectory")
                if ref not in (None, "", []):          # MoveHome without a trajectory = umarell's blind home
                    ok, tr = traj_of(n, op)
                    if ok:
                        check_traj(tr, op)
        for c in n.get("children") or []:
            if isinstance(c, dict):
                walk(c)
    walk(tree["root"])
    if probs:
        _bad(f"{len(probs)} problem(s) in the plan: " + "; ".join(probs[:4]) + (" ..." if len(probs) > 4 else ""))
    return {"leaves": n_leaf}


def check_source(src) -> None:
    """Validate a resolved Source (rollout arrays or plan) before export (raises Unsupported)."""
    if src.kind == "rollout":
        check_arrays(src.t, src.q, src.spec)
        for e in src.weld_events or []:
            if len(e) < 4 or e[0] not in src.robot_ids or not all(np.isfinite(float(x)) for x in e[2:4]):
                _bad(f"{src.spec}: weld event {e!r}")
    elif src.plan is not None:
        check_plan_input(src.plan, robot_ids=tuple(src.robot_ids))


# ───────────────────────────────────────── controllers ─────────────────────────────────────────
class StatelessController:
    """expert.stateless.StatelessExpert (the DAgger teacher; a function of the observation)."""

    def __init__(self, env):
        from expert.stateless import StatelessExpert
        self.ex = StatelessExpert(env)
        self.stats = {}

    def reset(self, obs):
        self.ex.reset()

    def act(self, obs):
        return self.ex.act(obs)


class ResidualController:
    """Residual policy on the rate expert (rl/residual_env.py) as a per-step controller of its own env:
    ``act`` returns exactly the env action ``ResidualEnv.step`` would apply."""

    def __init__(self, ckpt: str, cell: str):
        import torch
        import evaluate_residual as ER
        ck = torch.load(ckpt, map_location="cpu", weights_only=False)
        spec = json.loads(json.dumps(ck["spec"]))
        spec.setdefault("env", {})["cell_id"] = cell
        self.renv = ER._make(spec)
        self.pol = ER._policy(ckpt, spec)
        self.env = self.renv.env
        self.stats = {"checkpoint": ckpt}
        self._first = True

    def reset(self, obs):
        self.renv.base.reset()
        self.renv._obs = self.renv._features(obs)
        self._first = True

    def act(self, obs):
        r = self.renv
        if not self._first:
            r._obs = r._features(obs)
        self._first = False
        u = np.clip(np.asarray(self.pol(r._obs), float).reshape(2, -1), -1, 1)
        bj, br = r.bounds
        sigma_cmd = np.clip(1.0 + br * u[:, 6], 0.0, 1.0 + br)
        a, _ = r.base.execute(sigma_cmd, bj * u[:, :6], r._gate.copy(), r.rcfg.rate_delta)
        return np.asarray(a, float).reshape(-1)


def run_controller(spec: str, cell: str, *, home_tail: str = "expert", home_tol: float = 1e-4,
                   max_steps: int = 3000, safety_filter: bool = False) -> Source:
    """Canonical rollout of a controller with execute.py's machinery (same env kwargs, controllers, home tail)."""
    import execute as X
    kind, _, arg = spec.partition(":")
    t0 = time.time()
    if kind == "residual":
        ctrl = ResidualController(arg, cell)
        env = ctrl.env
    else:
        env_kw = X.controller_env_kwargs("expert" if kind == "stateless" else kind, arg if kind != "stateless" else "")
        env_kw.pop("cell_id", None)
        env = X.make_env(cell, **env_kw)
        ctrl = StatelessController(env) if kind == "stateless" else X.build_controller(kind, arg, env)
    opts = {"canonical": True}
    if isinstance(ctrl, X.TimelineController):
        env.cfg.assignment = ctrl.queues or X.queues_from_timeline(ctrl.tl)
        opts["q0"] = ctrl.tl.q[0]
    n_max = len(ctrl.tl.q) + 10 if isinstance(ctrl, X.TimelineController) else max_steps
    sf = X.SafetyFilter(env, enabled=True) if safety_filter else None
    rec = X.rollout(env, ctrl, reset_options=opts, max_steps=n_max, sfilter=sf, on_collision="stop",
                    home_tail=home_tail, home_tol=home_tol)
    from env.cell import ROBOT_IDS
    meta = {"controller": spec, "welds_done": rec["welds_done"], "n_welds": rec["n_welds"],
            "success": rec["success"], "collision": rec["collision"], "collision_steps": rec["collision_steps"],
            "t_last_weld_s": rec["t_last_weld_s"], "t_end_s": round(float(rec["t_end_s"]), 3),
            "home_tail": {k: rec["home_tail"].get(k) for k in ("mode", "steps", "at_home", "duration_s", "home_err_rad")},
            "rollout_wall_s": round(time.time() - t0, 1), "queues": rec["queues"]}
    cst = getattr(ctrl, "stats", None) or {}
    for k in ("lookahead", "lookahead_models", "lookahead_note"):
        if cst.get(k) is not None:
            meta[k] = cst[k]
    return Source(kind="rollout", cell_id=cell, robot_ids=tuple(ROBOT_IDS), home=np.asarray(env.home, float).reshape(2, 6),
                  spec=spec, umarell_cell=cell, t=rec["t"], q=rec["q"], weld_events=rec["weld_events"],
                  at_home=bool(rec["home_tail"]["at_home"]), meta=meta)


# ───────────────────────────────────────── files ─────────────────────────────────────────
def from_timeline(path, cell: Optional[str], spec: str, weld_events=None, home_tol: float = 1e-4,
                  scene=None) -> Source:
    """A Timeline / NTimeline npz. Two robots (robot1 / robot2): a rollout in the umarell cell ``cell``. Any other
    robot count (or ``scene`` given): a rollout in the trajgen scene named by ``scene`` or by the timeline's
    ``meta.source`` / ``meta.scene`` (e.g. tools/ctrl's smoothed trajgen motions, meta.source =
    trajgen/samples/<name>), cut by tools.btexport.nsync (umarell profile) or retimed (--backend retime). Without
    ``cell`` and ``scene``: the scene of the meta if it names one (a two-robot umarell-parity scene -> its umarell
    cell), else welding_cell_2_small."""
    p = Path(path)
    from trajgen.ntimeline import NTimeline
    if p.exists() and p.suffix == ".npz":
        try:
            tl0 = NTimeline.load(p)
        except Exception:  # noqa: BLE001  (not an NTimeline file: the paths below report it)
            tl0 = None
        if tl0 is not None and str((tl0.meta or {}).get("clock", "")) == EXECUTOR_CLOCK:
            # round 2 (docs/n2_contract.md 3.2.3): an env v2 timeline on the BT executor clock -> its legs 1:1
            check_arrays(tl0.t, tl0.q, f"{p}")
            return from_executor_timeline(p, tl0, spec, scene=scene)
    if (p.parent / "plan.json").exists() and (p.parent / "report.json").exists():
        return from_tgrun(p.parent, spec)
    tl = NTimeline.load(p)
    check_arrays(tl.t, tl.q, f"{p}")
    if scene is None and cell is None:
        sd = _scene_of_meta(tl.meta or {})
        if sd is not None:
            from trajgen.scene import load_scene
            sc = load_scene(sd / "scene.yaml")
            if len(tl.robot_ids) == 2 and sc.umarell_cell and getattr(sc, "umarell_parity", False) and \
                    set(tl.robot_ids) == {"robot1", "robot2"}:
                cell = sc.umarell_cell            # a umarell-parity scene: its umarell cell (validators apply)
            else:
                scene = sd
    if len(tl.robot_ids) != 2 or scene is not None:
        return _scene_timeline(p, tl, spec, scene, home_tol)
    cell = cell or "welding_cell_2_small"
    q = np.asarray(tl.q, float)
    ev = [tuple(e) for e in (weld_events if weld_events is not None else tl.weld_events)]
    home = cell_home(cell)
    from env.cell import ROBOT_IDS
    if tuple(tl.robot_ids) != tuple(ROBOT_IDS):
        order = [list(tl.robot_ids).index(r) for r in ROBOT_IDS]
        q = q[:, order]
    at_home = bool(np.abs(q[-1] - home).max() < home_tol)
    return Source(kind="rollout", cell_id=cell, robot_ids=tuple(ROBOT_IDS), home=np.asarray(q[0]), spec=spec,
                  umarell_cell=cell, t=np.asarray(tl.t, float), q=q, weld_events=ev, at_home=at_home,
                  meta={"timeline": _rel(p), "timeline_meta": {k: v for k, v in (tl.meta or {}).items()
                                                               if isinstance(v, (str, int, float, bool))}})


EXECUTOR_CLOCK = "executor"      # NTimeline meta["clock"] of timelines on the BT executor clock (envn v2)
GATE_EPS_S = 1e-6


def from_executor_timeline(p: Path, tl, spec: str, scene=None, gates: bool = True) -> Source:
    """An N-robot timeline on the BT executor clock (``meta.clock == "executor"``, envn v2, docs/n2_contract.md
    4.6) -> a ``plan`` Source whose tree runs the timeline's legs 1:1 with their own timing (no re-timing, both
    profiles): per robot a Sequence of its legs (ExecuteTrajectory, MoveHome for kind "home"), dispatched when the
    previous one completes (t1 + hold_s + the 1.0 s executor margin). A dispatch later than that becomes a stationary
    prefix of the leg. ``gates``: like trajgen.coord.legs_from_tracks, a leg that begins with a stationary run waits
    for the other robots' operations completed before its motion starts (WaitForRobot + Delay, nominally exact; the
    prefix is then dropped), so the plan follows a late robot instead of the clock. Raises Unsupported if a leg is
    dispatched before the previous one completes (not executor-consistent)."""
    from dataclasses import replace as _rep

    from player.bt_plan import COMPLETION_MARGIN_S
    from player.timeline import Leg
    from tools.btexport.core import Unsupported
    from trajgen.btplan import plan_from_sequences
    base = _scene_timeline(p, tl, spec, scene, 1e-3)
    rids = list(base.robot_ids)
    if not tl.legs:
        raise Unsupported(f"{p}: an executor-clock timeline without legs")
    margin = COMPLETION_MARGIN_S
    out_legs, deps, rows = [], {}, []
    # pass 1: executor-consistent legs (stationary prefix where the dispatch is later than the completion)
    per = {}
    for rid in rids:
        mine = sorted([l for l in tl.legs if l.robot == rid], key=lambda l: l.t0)
        prev_done, L2 = 0.0, []
        for l in mine:
            if l.t0 < prev_done - GATE_EPS_S:
                raise Unsupported(f"{p}: {rid} leg {l.op_id} is dispatched at {l.t0:.4f} s, before its previous leg "
                                  f"completes on the executor clock ({prev_done:.4f} s)")
            times, pos, ph = np.asarray(l.times, float), np.asarray(l.positions, float), np.asarray(l.phase)
            if l.t0 > prev_done + GATE_EPS_S or (len(times) and times[0] > l.t0 + GATE_EPS_S):
                t_new = prev_done
                times = np.concatenate([[t_new], times]) if times[0] > t_new + GATE_EPS_S else times
                pos = np.vstack([pos[:1], pos]) if len(times) > len(l.times) else pos
                ph = np.concatenate([np.array(["wait"], dtype="<U16"), ph]) if len(times) > len(l.times) else ph
            else:
                t_new = l.t0
            home = l.kind == "home"
            hold = 0.0 if home else float(l.hold_s)
            leg = Leg(robot=rid, op_id=str(l.op_id), kind="home" if home else ("weld" if l.kind == "weld" else "move"),
                      t0=float(t_new), t1=float(times[-1]), times=times, positions=pos,
                      phase=np.asarray(ph, dtype="<U16"), hold_s=hold, vscale=1.0,
                      operation_type="" if home else ("weld" if l.kind == "weld" else "move"),
                      node_type="MoveHome" if home else "ExecuteTrajectory",
                      bb_key=f"traj_{rid}_{l.op_id}")
            leg.t_done = leg.t1 + hold + margin
            src_done = float(getattr(l, "t_done", 0.0) or 0.0)
            rows.append(abs(src_done - leg.t_done) if src_done else 0.0)
            L2.append(leg)
            prev_done = leg.t_done
        per[rid] = L2
    # pass 2: gates at the motion start of legs that begin with a stationary run (trajgen.coord.legs_from_tracks)
    done = {r: [(l.t_done, l) for l in per[r] if l.kind != "home"] for r in rids}
    for rid in rids:
        prev_disp = 0.0
        for l in per[rid]:
            Q = l.positions
            mv = np.flatnonzero(np.abs(np.diff(Q, axis=0)).max(1) > 1e-9) if len(Q) > 1 else np.zeros(0, int)
            k0 = int(mv[0]) if len(mv) else 0
            t_mv = float(l.times[k0])
            dd = []
            if gates and k0 > 0 and t_mv > l.t0 + GATE_EPS_S:
                for rj in rids:
                    if rj == rid:
                        continue
                    prev = [x for x in done[rj] if prev_disp - GATE_EPS_S <= x[0] <= t_mv + GATE_EPS_S]
                    if prev:
                        tdj, lj = max(prev, key=lambda x: x[0])
                        dd.append((rj, lj.op_id, round(t_mv - tdj, 9)))
            if dd:
                l2 = _rep(l, t0=t_mv, times=l.times[k0:], positions=Q[k0:], phase=l.phase[k0:])
                l2.t_done = l.t_done
                deps[(rid, l.op_id)] = dd
                out_legs.append(l2)
                prev_disp = t_mv
            else:
                out_legs.append(l)
                prev_disp = l.t0
    md = {"generator": str((tl.meta or {}).get("tag") or (tl.meta or {}).get("controller") or "executor timeline"),
          "export_mode": "executor_timeline", "source_timing": True,
          "sync": "per-robot sequences on the executor clock" + (" with WaitForRobot gates" if deps else "")}
    plan = plan_from_sequences(base.cell_id, out_legs, rids, deps, tree_id=f"tl_{base.cell_id}", metadata_extra=md)
    meta = dict(base.meta)
    meta.update(source_timing=True, timeline_clock=EXECUTOR_CLOCK, n_gates=sum(len(v) for v in deps.values()),
                source_t_done_max_dev_s=round(float(max(rows, default=0.0)), 6))
    for k in ("success_env", "success", "controller"):
        v = (tl.meta or {}).get(k)
        if isinstance(v, (str, int, float, bool)):
            meta[k] = v
    if "success" not in meta and isinstance((tl.meta or {}).get("success_env"), bool):
        meta["success"] = bool(tl.meta["success_env"])
    return Source(kind="plan", cell_id=base.cell_id, robot_ids=base.robot_ids, home=base.home, spec=spec,
                  scene=base.scene, task=base.task, umarell_cell=base.umarell_cell, plan=plan, meta=meta)


def _scene_of_meta(meta: dict) -> Optional[Path]:
    for k in ("scene", "source"):
        v = (meta or {}).get(k)
        if not isinstance(v, str) or not v:
            continue
        for c in (Path(v), ROOT / v):
            d = c.parent if c.suffix == ".yaml" else c
            if (d / "scene.yaml").exists():
                return d
            if (d / "report.json").exists():                      # a trajgen run dir: its scene
                try:
                    sp = Path(json.load(open(d / "report.json"))["scene"])
                    sp = sp if sp.is_absolute() else ROOT / sp
                    return sp.parent if sp.suffix == ".yaml" else sp
                except Exception:  # noqa: BLE001
                    pass
    return None


def _scene_timeline(p: Path, tl, spec: str, scene, home_tol: float) -> Source:
    """N-robot (or explicitly scene-bound) timeline -> a scene rollout Source (P3U round 2: tools/ctrl's smoothed
    3-4-robot trajgen motions were refused)."""
    from tools.btexport.core import Unsupported
    sd = Path(scene) if scene is not None else _scene_of_meta(tl.meta or {})
    if sd is not None and not sd.is_absolute() and not sd.exists():
        sd = ROOT / sd
    if sd is not None and sd.suffix == ".yaml":
        sd = sd.parent
    if sd is None or not (sd / "scene.yaml").exists():
        raise Unsupported(f"{p}: a {len(tl.robot_ids)}-robot timeline needs its trajgen scene: none found in "
                          f"meta.source / meta.scene ({(tl.meta or {}).get('source')!r}); pass --scene <scene dir> "
                          "or export the trajgen run dir (tgrun:)")
    from trajgen.scene import load_scene
    sc = load_scene(sd / "scene.yaml")
    rids = tuple(r.id for r in sc.robots)
    if set(rids) != set(tl.robot_ids):
        raise Unsupported(f"{p}: timeline robots {list(tl.robot_ids)} != scene {sd} robots {list(rids)}")
    order = [list(tl.robot_ids).index(r) for r in rids]
    q = np.asarray(tl.q, float)[:, order]
    home = np.stack([r.home for r in sc.robots])
    tp = sd / "task.json"
    if not tp.exists():
        raise Unsupported(f"{sd}: no task.json next to scene.yaml")
    ev = [tuple(e) for e in tl.weld_events]
    ucell = sc.umarell_cell if (sc.umarell_cell and getattr(sc, "umarell_parity", False)) else None
    return Source(kind="rollout", cell_id=sc.cell_id, robot_ids=rids, home=home, spec=spec, scene=sd / "scene.yaml",
                  task=tp, umarell_cell=ucell, t=np.asarray(tl.t, float), q=q, weld_events=ev,
                  at_home=bool(np.abs(q[-1] - home).max() < max(home_tol, 1e-3)),
                  meta={"timeline": _rel(p), "scene": _rel(sd / "scene.yaml"), "task": _rel(tp),
                        "generator": str((tl.meta or {}).get("tag") or "timeline"),
                        "timeline_meta": {k: v for k, v in (tl.meta or {}).items()
                                          if isinstance(v, (str, int, float, bool))}})


def from_exec(path, spec: str) -> Source:
    res = json.load(open(path))
    ro = res.get("rollout") or {}
    tl = ROOT / ro["timeline"]
    s = from_timeline(tl, res["cell"], spec)
    s.at_home = bool((ro.get("home_tail") or {}).get("at_home", s.at_home))
    s.meta.update({"exec_result": _rel(path), "controller": res.get("controller"),
                   "exec_export_mode": res.get("export_mode")})
    return s


def _tree_robots(tree: dict) -> list:
    out = []

    def walk(n):
        p = n.get("ports") or {}
        if p.get("robot_ns") and p["robot_ns"] not in out:
            out.append(p["robot_ns"])
        for c in n.get("children") or []:
            walk(c)
    walk(tree.get("root") or {})
    return out


def from_plan(plan_or_path, cell: Optional[str], spec: str) -> Source:
    from player.bt_plan import load_plan
    from tools.btexport.core import Unsupported
    env = load_plan(str(plan_or_path)) if isinstance(plan_or_path, (str, Path)) else plan_or_path
    if not isinstance(env, dict) or not isinstance(env.get("behaviour_tree"), dict):
        raise Unsupported(f"{plan_or_path}: not a umarell plan envelope (no behaviour_tree)")
    tree = env["behaviour_tree"]
    if not (tree.get("root") or {}).get("children") and (tree.get("root") or {}).get("type") in (None, "Sequence"):
        raise Unsupported(f"{plan_or_path}: the plan's root has no children (empty plan)")
    check_plan_input(env)
    cid = cell or env.get("cell_id") or tree.get("cell_id")
    from env.cell import CELLS
    robots = _tree_robots(tree)
    md = tree.get("metadata") or {}
    umarell_cell = cid is not None and (CELLS / str(cid) / "agentic_planner" / "config" / "robots.yaml").exists()
    # the cell's own arms (robots.yaml): robot1 / robot2, or any N-arm umarell-style cell (robot1..robot4, ...)
    cell_rids = cell_robot_ids(str(cid)) if umarell_cell else ()
    if set(robots) - set(cell_rids) or not umarell_cell:
        # an N-robot / exported-cell plan (trajgen): its scene defines the robots -> the trajgen run dir
        cands = []
        if isinstance(plan_or_path, (str, Path)):
            cands.append(Path(plan_or_path).resolve().parent)
        sc = (md.get("source") or {}).get("scene") or md.get("scene")
        for d in cands:
            if (d / "report.json").exists() and (d / "plan.json").exists():
                s = from_tgrun(d, spec)
                if isinstance(plan_or_path, (str, Path)) and Path(plan_or_path).resolve() != (d / "plan.json").resolve():
                    s.plan = json.loads(json.dumps(env))
                s.meta["plan_file"] = _rel(plan_or_path)
                return s
        why = (f"robots {sorted(set(robots) - set(cell_rids))} are not arms of cell {cid!r} ({list(cell_rids)})"
               if umarell_cell else f"{cid!r} is not a umarell cell (no agentic_planner/config/robots.yaml)")
        raise Unsupported(f"plan with robots {robots} in cell {cid!r}: not a umarell-cell plan ({why}) and no trajgen "
                          f"run (report.json + plan.json) next to it" + (f" (metadata scene {sc})" if sc else "") +
                          "; export its run directory with --from tgrun:<run dir>")
    return Source(kind="plan", cell_id=cid, robot_ids=cell_rids, home=cell_home(cid, cell_rids), spec=spec,
                  umarell_cell=cid, plan=json.loads(json.dumps(env)),
                  meta={"plan_file": _rel(plan_or_path) if isinstance(plan_or_path, (str, Path)) else "dict",
                        "generator": "umarell" if str(env.get("tree_id", "")).startswith("bt_") else
                        (env["behaviour_tree"].get("metadata") or {}).get("generator", "unknown")})


def from_tgrun(run, spec: str) -> Source:
    run = Path(run)
    rep = json.load(open(run / "report.json"))
    if not (run / "plan.json").exists():
        raise SystemExit(f"{run}: no plan.json (trajgen status {rep.get('status')})")
    plan = json.load(open(run / "plan.json"))
    sp = Path(rep["scene"]); sp = sp if sp.is_absolute() else ROOT / sp
    tp = Path(rep["task"]); tp = tp if tp.is_absolute() else ROOT / tp
    from trajgen.scene import load_scene
    sc = load_scene(sp)
    rids = tuple(r.id for r in sc.robots)
    home = np.stack([r.home for r in sc.robots])
    ucell = sc.umarell_cell if (sc.umarell_cell and getattr(sc, "umarell_parity", False)) else None
    md = plan["behaviour_tree"].get("metadata") or {}
    return Source(kind="plan", cell_id=plan.get("cell_id") or sc.cell_id, robot_ids=rids, home=home, spec=spec,
                  scene=sp, task=tp, umarell_cell=ucell, plan=plan, run_dir=run,
                  meta={"trajgen_run": _rel(run), "trajgen_status": rep.get("status"), "generator": "trajgen",
                        "trajgen_speed": md.get("speed"), "coordination": md.get("coordination"),
                        "scene": _rel(sp), "task": _rel(tp)})


def sample_scene(name: str) -> Path:
    for base in (ROOT / "trajgen" / "samples" / name, ROOT / "scenes" / name, ROOT / "scenes" / "suite" / name, Path(name)):
        if (base / "scene.yaml").exists() or (base.suffix == ".yaml" and base.exists()):
            return base
    raise SystemExit(f"trajgen: no scene {name!r} (trajgen/samples, scenes/, scenes/suite/ or a path)")


def run_trajgen(arg: str, work: Path, profile: str) -> Source:
    """``trajgen:<scene>[,<extra args>]``: run python -m trajgen into ``work/<scene name>[_<speed>]`` unless a
    complete run with the same command line is there already. ``profile=full`` adds ``--speed full`` when the
    generator supports it (track TGPERF); the plan is then exported unchanged."""
    name, _, extra = arg.partition(",")
    sp = sample_scene(name)
    xargs = shlex.split(extra.replace(";", " ")) if extra else []
    speed = None
    if profile == "full" and "--speed" not in xargs:
        hp = subprocess.run([sys.executable, "-m", "trajgen", "--help"], cwd=str(ROOT), capture_output=True, text=True)
        if "full" in hp.stdout.split("--speed", 1)[-1][:200]:
            xargs += ["--speed", "full"]
            speed = "full"
        else:
            speed = "unavailable"
    out = work / f"{Path(sp).name}{'_full' if speed == 'full' else ''}"
    cmd = [sys.executable, "-m", "trajgen", "--scene", str(sp), "--out", str(out)] + xargs
    stamp = out / "btexport_cmd.json"
    # cache key: the command AND the md5 of scene.yaml / task.json / every robot URDF + SRDF (an in-place URDF edit
    # re-plans) and the robot override in effect
    from tools.btexport.limits import inputs_fingerprint
    from robotmodel.override import active_override, dumps
    key = {"cmd": cmd[1:], "inputs": inputs_fingerprint(scene=sp), "robot_override": dumps(active_override())}
    if not ((out / "plan.json").exists() and stamp.exists() and json.load(open(stamp)) == key):
        out.mkdir(parents=True, exist_ok=True)
        t0 = time.time()
        p = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True, env=_pyenv())
        (out / "btexport_trajgen.log").write_text(p.stdout[-20000:] + "\n--- stderr ---\n" + p.stderr[-20000:])
        if not (out / "plan.json").exists():
            raise SystemExit(f"trajgen failed (exit {p.returncode}), see {out / 'btexport_trajgen.log'}")
        json.dump(key, open(stamp, "w"))
        (out / "btexport_wall_s.txt").write_text(f"{time.time() - t0:.1f}\n")
    s = from_tgrun(out, f"trajgen:{arg}")
    s.meta["trajgen_cmd"] = " ".join(cmd[1:])
    s.meta["trajgen_full_speed"] = speed
    return s


def _pyenv():
    import os
    e = dict(os.environ)
    e["PYTHONPATH"] = str(ROOT) + (":" + e["PYTHONPATH"] if e.get("PYTHONPATH") else "")
    return e


def resolve(spec: str, *, cell: Optional[str] = None, work: Path = ROOT / "data" / "btexport" / "work",
            profile: str = "umarell", home_tail: str = "expert", home_tol: float = 1e-4, max_steps: int = 3000,
            safety_filter: bool = False, scene=None) -> Source:
    """``scene``: the trajgen scene of a ``timeline:`` source (N-robot timelines; default: the timeline's meta)."""
    src = _resolve(spec, cell=cell, work=work, profile=profile, home_tail=home_tail, home_tol=home_tol,
                   max_steps=max_steps, safety_filter=safety_filter, scene=scene)
    check_source(src)
    return src


def _resolve(spec: str, *, cell, work, profile, home_tail, home_tol, max_steps, safety_filter, scene) -> Source:
    kind, _, arg = spec.partition(":")
    if scene is not None and kind != "timeline" and Path(spec).suffix != ".npz":
        from tools.btexport.core import Unsupported
        raise Unsupported("--scene together with --from is only for timeline: sources (the scene of an N-robot "
                          "timeline)")
    if kind in CONTROLLER_KINDS:
        return run_controller(spec, cell or "welding_cell_2_small", home_tail=home_tail, home_tol=home_tol,
                              max_steps=max_steps, safety_filter=safety_filter)
    if kind == "umarell":
        import execute as X
        s = from_plan(X.umarell_plan_file(arg), cell or "welding_cell_2_small", spec)
        return s
    if kind == "trajgen":
        return run_trajgen(arg, Path(work), profile)
    if kind == "tgrun":
        return from_tgrun(arg, spec)
    if kind == "timeline":
        return from_timeline(arg, cell, spec, scene=scene)
    if kind == "exec":
        return from_exec(arg, spec)
    if kind == "plan":
        return from_plan(arg, cell, spec)
    p = Path(spec)
    if p.is_dir():
        return from_tgrun(p, f"tgrun:{spec}")
    if p.suffix == ".npz":
        return from_timeline(p, cell, f"timeline:{spec}", scene=scene)
    if p.suffix == ".json":
        d = json.load(open(p))
        if "rollout" in d and "controller" in d:
            return from_exec(p, f"exec:{spec}")
        if "behaviour_tree" in d or "root" in d:
            return from_plan(p, cell, f"plan:{spec}")
    from tools.btexport.core import Unsupported
    raise Unsupported(f"cannot classify --from {spec!r}")
