#!/usr/bin/env python3
"""Validate an exported plan with umarell's OWN loaders, validators and executor (run in .venv-umarell).

    .venv-umarell/bin/python -m tools.btexport.umarell_check --run <validation dir> --repo data/btexport/umarell_repo
        [--out report.json] [--no-executor] [--no-validators] [--mesh-audit] [--executor-xml]

``<validation dir>`` is written by ``tools.btexport.validate.prepare`` (project venv): plan.json (the export),
plan.xml (the separate BTCPP v4 file), timeline.npz (the exporter's replay of the plan on the executor clock),
check.json {cell_id, umarell_cell, scene, task, robots}. The umarell repo is a scratch copy (never third_party/).

Checks, all with umarell's code from the scratch copy:
  load          the plan is stored in the cell's plans/ dir and read back with plan_store.get_plan (identical tree);
                plan_store._count_execute == num_operations
  structure     behaviour_tree.validate_behaviour_tree (known node kinds, non-empty trajectories, unique operation ids,
                monotonic time stamps); no warnings allowed
  xml           the .xml file is well-formed, == btcpp.to_xml(tree) == tree["xml"]; root BTCPP_format="4",
                main_tree_to_execute, one <BehaviorTree>; every element is a node kind umarell's executor implements
                (behaviour_tree._KNOWN_KINDS) and carries only ports umarell declares (btcpp.LEAF_PORTS for the custom
                leaves; Parallel success_count / failure_count; Delay delay_msec); <TreeNodesModel> == btcpp's;
                every {key} reference resolves in the blackboard; btcpp.from_xml(xml, blackboard) gives the JSON root
                back (ids aside); btcpp.to_btcpp is idempotent on it
  schedule      trajectory_quality.schedule_metrics (umarell's own makespan estimate)
  validators    two-robot umarell cells: scripts/umarell_validate.py (validate_trajectory_set per step, full-timeline
                inter_robot_clearance, full_eval, validate_approach per weld, obstacle checks, optional hpp-fcl mesh
                audit) on the ORIGINAL cell copy; other scenes: scripts/trajgen_umarell_check.py export_cell + validate
                (an N-robot umarell cell exported from the scene: inter_robot_clearance every pair, validate_trajectory_set
                per leg window, validate_approach, _trajectory_collision_info, FK cross-check)
  executor      agents/cell/bt_executor.BehaviourTreeExecutor runs the plan in real time (20 Hz ticks) on a fake
                runtime that tracks the dispatched trajectories; dispatch times vs the replay, success, e-stops; the
                motion re-assembled from the actual dispatch times is checked with rk.inter_robot_clearance;
                ``--executor-xml``: the same on the tree parsed back from the .xml file (btcpp.from_xml), i.e. the
                XML export alone (+ its blackboard JSON) is executable
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def _r(x, n=4):
    return None if x is None else round(float(x), n)


def _strip(n):
    if not isinstance(n, dict):
        return n
    return {"type": n.get("type"), "name": n.get("name") or None, "ports": n.get("ports") or {},
            "children": [_strip(c) for c in n.get("children") or []]}


def setup(repo: Path):
    repo = repo.resolve()
    if repo == (ROOT / "third_party" / "sample-umarell").resolve():
        raise SystemExit("refusing to use third_party/sample-umarell (read-only); pass a scratch copy")
    os.environ["UMARELL_REPO"] = str(repo)
    os.environ["CELL_BASE_PATH"] = str(repo / "cells")
    os.environ["CELL_DATA_MODE"] = "local"
    for p in (str(ROOT / "scripts"), str(ROOT), str(repo)):
        if p in sys.path:
            sys.path.remove(p)
        sys.path.insert(0, p)
    import pinocchio as pin
    if not hasattr(pin.Frame, "parentJoint"):
        pin.Frame.parentJoint = property(lambda s: s.parent)


# ───────────────────────────────────────── load / structure / xml ─────────────────────────────────────────
def load_check(plan: dict, cell_id: str) -> tuple[dict, dict]:
    from agents.trajectory.tools import plan_store
    from utils import cell_layout
    tid = plan["tree_id"]
    d = Path(cell_layout.plans_dir(cell_id))
    d.mkdir(parents=True, exist_ok=True)
    p = d / f"{tid}.json"
    stored = dict(plan, cell_id=cell_id)
    stored["behaviour_tree"] = dict(plan["behaviour_tree"], cell_id=cell_id)
    p.write_text(json.dumps(stored))
    loaded = plan_store.get_plan(cell_id, tid)
    listed = [x for x in plan_store.list_plans(cell_id) if x["tree_id"] == tid]
    out = {"plans_dir": str(d), "get_plan_loaded": loaded is not None,
           "identical": loaded == stored["behaviour_tree"],
           "listed_status": listed[0]["status"] if listed else None,
           "count_execute": plan_store._count_execute(loaded or {}), "num_operations": plan.get("num_operations")}
    out["pass"] = bool(out["get_plan_loaded"] and out["identical"] and out["listed_status"] == "ok"
                       and out["count_execute"] == out["num_operations"])
    return out, loaded or {}


def robot_model_check(plan: dict, cell_id: str, repo: Path) -> dict:
    """The plan's metadata.robot_model (tools.btexport) against the umarell cell it would run in: per robot the URDF
    content (md5), base position, yaw, tip_link and tcp_offset of the cell's robots.yaml / cell.yaml. A plan exported
    for another robot (robot override) is refused here: in umarell's cell it would run on the cell's own robot with
    its welds off. Plans without the record (umarell's own, older exports) pass with ``recorded`` False."""
    import hashlib
    import yaml
    md = (plan.get("behaviour_tree") or {}).get("metadata") or {}
    rm = md.get("robot_model")
    if not isinstance(rm, dict) or not rm.get("robots"):
        return {"pass": True, "recorded": False}
    ap = Path(repo) / "cells" / cell_id / "agentic_planner"
    if not (ap / "config" / "robots.yaml").exists():
        return {"pass": True, "recorded": True, "note": f"no umarell cell {cell_id} in the scratch repo"}
    arms = yaml.safe_load(open(ap / "config" / "robots.yaml"))["robots"]["arms"]
    man = yaml.safe_load(open(ap / "cell.yaml"))
    urdfs = {r["id"]: Path(repo) / "cells" / cell_id / r["urdf"] for r in man.get("robots", [])}
    diffs = {}
    for rid, e in rm["robots"].items():
        c = arms.get(rid)
        if c is None:
            diffs[rid] = ["robot not in the cell"]
            continue
        d = []
        u = urdfs.get(rid)
        if u is not None and u.exists():
            if e.get("kinematics_md5"):          # geometry (joints, collision shapes); <limit> edits do not count
                try:
                    from robotmodel.urdf_robot import kinematics_md5
                    if kinematics_md5(u) != e["kinematics_md5"]:
                        d.append(f"URDF kinematics / collision geometry differ ({e.get('urdf')})")
                except Exception as ex:  # noqa: BLE001
                    d.append(f"cannot read the cell URDF: {ex!r}")
            elif e.get("urdf_md5") and hashlib.md5(u.read_bytes()).hexdigest() != e["urdf_md5"]:
                d.append(f"URDF content differs ({e.get('urdf')})")
        if e.get("base") is not None and not np.allclose(e["base"], c.get("base_position"), atol=1e-6):
            d.append(f"base {e['base']} vs {c.get('base_position')}")
        if e.get("yaw") is not None and abs(float(e["yaw"]) - float(c.get("base_orientation_yaw", 0.0))) > 1e-6:
            d.append(f"yaw {e['yaw']} vs {c.get('base_orientation_yaw', 0.0)}")
        if e.get("tcp_offset") is not None and c.get("tcp_offset") is not None and \
                not np.allclose(e["tcp_offset"], c["tcp_offset"], atol=1e-6):
            d.append(f"tcp_offset {e['tcp_offset']} vs {c['tcp_offset']}")
        if e.get("tip_link") and c.get("tip_link") and e["tip_link"] != c["tip_link"]:
            d.append(f"tip_link {e['tip_link']} vs {c['tip_link']}")
        if e.get("home_rad") is not None and c.get("home_pose_deg") is not None and \
                not np.allclose(e["home_rad"], np.deg2rad(c["home_pose_deg"]), atol=1e-6):
            d.append("home pose differs")
        if d:
            diffs[rid] = d
    return {"pass": not diffs, "recorded": True, "mismatches": diffs,
            "rule": "metadata.robot_model must match the cell's robots (URDF md5, base, yaw, tcp_offset, tip_link, "
                    "home): a plan for another robot model is not valid in this cell"}


def structure_check(tree: dict) -> dict:
    from agents.trajectory.tools.behaviour_tree import validate_behaviour_tree
    fn = getattr(validate_behaviour_tree, "func", None) or validate_behaviour_tree
    v = fn(tree)
    return {"valid": v["valid"], "errors": v["errors"][:10], "warnings": v["warnings"][:10],
            "execute_count": v["execute_count"], "n_operation_ids": len(v["operation_ids"]),
            "pass": bool(v["valid"] and not v["warnings"])}


CONTROL_PORTS = {"Sequence": set(), "SequenceWithMemory": set(), "Parallel": {"success_count", "failure_count"},
                 "Delay": {"delay_msec"}}


def xml_check(tree: dict, xml_text: str) -> dict:
    from agents.trajectory.tools import btcpp
    from agents.trajectory.tools.behaviour_tree import _KNOWN_KINDS
    out = {}
    try:
        root = ET.fromstring(xml_text)
        out["well_formed"] = True
    except ET.ParseError as e:
        return {"well_formed": False, "error": str(e), "pass": False}
    out["equals_tree_xml"] = xml_text == tree.get("xml")
    out["equals_btcpp_to_xml"] = xml_text == btcpp.to_xml(tree)
    bts = root.findall("BehaviorTree")
    out["root_ok"] = (root.tag == "root" and root.get("BTCPP_format") == "4" and len(bts) == 1
                      and root.get("main_tree_to_execute") == bts[0].get("ID") and len(list(bts[0])) == 1)
    bad_kind, bad_port, refs, kinds = [], [], [], {}
    for el in bts[0].iter() if bts else []:
        if el is bts[0]:
            continue
        kinds[el.tag] = kinds.get(el.tag, 0) + 1
        if el.tag not in _KNOWN_KINDS:
            bad_kind.append(el.tag)
            continue
        allowed = CONTROL_PORTS.get(el.tag)
        if allowed is None:
            allowed = {k for k in btcpp.LEAF_PORTS[el.tag] if k != "_kind"}
        for k, v in el.attrib.items():
            if k != "name" and k not in allowed:
                bad_port.append(f"{el.tag}.{k}")
            if isinstance(v, str) and len(v) > 2 and v[0] == "{" and v[-1] == "}" and '"' not in v:
                refs.append(v[1:-1])
    bb = tree.get("blackboard") or {}
    out["node_kinds"] = kinds
    out["unknown_node_kinds"] = sorted(set(bad_kind))
    out["undeclared_ports"] = sorted(set(bad_port))
    out["unresolved_refs"] = sorted({r for r in refs if r not in bb})
    out["n_blackboard_refs"] = len(refs)
    m = root.find("TreeNodesModel")
    shape = lambda e: (e.tag, dict(e.attrib), [shape(c) for c in e])
    out["tree_nodes_model_ok"] = m is not None and shape(m) == shape(btcpp.tree_nodes_model())
    parsed = btcpp.from_xml(xml_text, bb)
    out["from_xml_root_equals_json"] = _strip(parsed["root"]) == _strip(tree["root"])
    conv = btcpp.to_btcpp(tree)
    out["to_btcpp_idempotent"] = _strip(conv["root"]) == _strip(tree["root"]) and conv.get("xml") == tree.get("xml")
    out["pass"] = bool(out["equals_tree_xml"] and out["equals_btcpp_to_xml"] and out["root_ok"] and not bad_kind
                       and not bad_port and not out["unresolved_refs"] and out["tree_nodes_model_ok"]
                       and out["from_xml_root_equals_json"] and out["to_btcpp_idempotent"])
    return out


def schedule_check(tree: dict) -> dict:
    try:
        from agents.trajectory.tools.trajectory_quality import schedule_metrics
        m = schedule_metrics(tree)
        return {k: m.get(k) for k in ("makespan_s", "n_arms", "parallel_fraction", "busy_s", "idle_s") if k in m} | \
            {"keys": sorted(m)[:20]}
    except Exception as e:  # noqa: BLE001
        return {"error": repr(e)}


# ───────────────────────────────────────── executor dry run ─────────────────────────────────────────
class FakeRuntime:
    """Duck-typed runtime for BehaviourTreeExecutor (any robot ids): tracks dispatched trajectories with the robot
    agent's velocity scaling and the sim controller's linear sampling (scripts/umarell_validate.py, N robots)."""

    def __init__(self, robot_ids, home):
        from agents.trajectory.tools.time_parameterization import _sample_at_time
        self._sample = _sample_at_time
        self.rids = list(robot_ids)
        self.t0 = time.monotonic()
        self.lock = threading.Lock()
        self.cmd = {r: (0.0, [{"positions": [float(v) for v in home[i]], "time_s": 0.0}]) for i, r in enumerate(self.rids)}
        self.log, self.estops = [], []

    def now(self):
        return time.monotonic()

    def robots(self):
        return list(self.rids)

    def latest_status(self, rid):
        with self.lock:
            t_d, traj = self.cmd[rid]
        q = self._sample(traj, self.now() - self.t0 - t_d)
        return {"joint_positions": [float(v) for v in q], "ee_position": [0.0, 0.0, 0.0]}

    def dispatch(self, rid, task):
        t = self.now() - self.t0
        vs = float(task.get("velocity_scale", 1.0)) or 1.0
        traj = [dict(p, time_s=float(p["time_s"]) / vs) for p in task["trajectory"]]
        with self.lock:
            self.cmd[rid] = (t, traj)
        self.log.append({"t": t, "robot": rid, "op": task.get("operation_id") or "home", "traj": traj,
                         "hold": float(task.get("hold_duration_s") or 0.0)})

    def estop(self, reason):
        self.estops.append(reason)

    def clear_estop(self, reason="cleared"):
        pass

    def shutdown(self):
        pass


def executor_dry_run(plan: dict, cell_id: str, robot_ids, home, legs) -> dict:
    from agents.cell.bt_executor import BehaviourTreeExecutor, ExecConfig
    from agents.trajectory.tools import robot_kinematics as rk
    from trajgen_umarell_check import composite
    cfg = ExecConfig.for_cell(cell_id)
    rt = FakeRuntime(robot_ids, home)
    ex = BehaviourTreeExecutor(rt, monitor=None, cfg=cfg)
    t0 = time.monotonic()
    res = ex.execute(plan["behaviour_tree"])
    wall = time.monotonic() - t0
    starts = {}
    for l in legs:
        starts.setdefault((l["robot"], l["op_id"]), []).append(l["t0"])
    seen, rows = {}, []
    for e in rt.log:
        k = (e["robot"], e["op"])
        i = seen.get(k, 0)
        seen[k] = i + 1
        lst = sorted(starts.get(k) or [])
        exp = lst[i] if i < len(lst) else None
        rows.append({"robot": e["robot"], "op": e["op"], "t_dispatch_s": _r(e["t"], 3), "replay_t0_s": _r(exp, 3),
                     "lag_s": _r(e["t"] - exp, 3) if exp is not None else None})
    lags = [r["lag_s"] for r in rows if r["lag_s"] is not None]
    out = {"exec_config": {k: getattr(cfg, k) for k in ("tick_hz", "completion_margin_s", "home_time_s",
                                                        "home_return_time_s")},
           "result": res, "wall_s": _r(wall, 2), "n_dispatches": len(rows), "n_legs": len(legs),
           "max_lag_s": _r(max(lags), 3) if lags else None, "min_lag_s": _r(min(lags), 3) if lags else None,
           "estops": rt.estops, "undispatched": [k for k, v in starts.items() if seen.get(k, 0) < len(v)]}
    # the motion as actually dispatched: every trajectory from its dispatch time, arm holds its last pose after
    if len(robot_ids) >= 2:
        dl = []
        for e in rt.log:
            tr = e["traj"]
            dl.append({"robot": e["robot"], "op_id": e["op"], "t0": e["t"], "t_done": e["t"] + tr[-1]["time_s"],
                       "t1": e["t"] + tr[-1]["time_s"], "times": np.array([e["t"] + p["time_s"] for p in tr]),
                       "positions": np.array([p["positions"] for p in tr], float),
                       "phase": [str(p.get("phase", "")) for p in tr]})
        T = max([l["t1"] for l in dl] + [0.0]) + 1.0
        comp = {rid: composite([l for l in dl if l["robot"] == rid], home[i], T) for i, rid in enumerate(robot_ids)}
        c = rk.inter_robot_clearance(cell_id, comp)
        out["dispatched_motion_clearance"] = {k: c.get(k) for k in ("collision_free", "min_clearance_m",
                                                                    "samples_checked", "worst_pair") if k in c}
    out["pass"] = bool(res.get("success") and not rt.estops and not out["undispatched"]
                       and (out.get("dispatched_motion_clearance") or {"collision_free": True}).get("collision_free"))
    return out


# ───────────────────────────────────────── validators ─────────────────────────────────────────
def validators_umarell_cell(run: Path, cell_id: str, mesh: bool) -> dict:
    import umarell_validate as UV
    out_json = run / "umarell_validate.json"
    argv = [str(run / "plan.json"), "--cell", cell_id, "--out", str(out_json)] + (["--mesh-audit"] if mesh else [])
    t0 = time.time()
    rep = UV.main(argv)
    v = dict(rep["verdict"])
    ap = rep["approach"]
    gate = bool(v.get("arm_arm_collision_free_full_timeline")) and bool(v.get("per_step_collision_free")) \
        and bool(v.get("obstacles_free")) and ap["n_matches"] == ap["task_welds"] and not ap["missing"] \
        and not ap.get("duplicates")
    n_jl = sum(len(rep["structure"][r]["joint_limit_violations"] or []) for r in rep["structure"])
    gate = gate and n_jl == 0              # umarell's own joint-limit count (P3U fix: it was reported, not gated)
    s = {"tool": "scripts/umarell_validate.py (original umarell cell copy)", "report": str(out_json),
         "wall_s": _r(time.time() - t0, 1), "verdict": v,
         "inter_robot_clearance_min_m": rep["full_timeline"].get("min_clearance_m"),
         "per_step_min_clearance_m": (rep.get("per_step_summary") or {}).get("min_clearance_m"),
         "dense_min_m": rep["dense"]["min_clearance_m"],
         "validate_approach": f"{ap['n_matches']}/{ap['task_welds']}", "approach_missing": ap["missing"],
         "obstacles_free": rep["obstacles"]["all_free"],
         "joint_speed_max_frac_urdf": max(rep["structure"][r]["max_joint_speed_fraction_of_urdf"] for r in rep["structure"]),
         "joint_limit_violations": n_jl,
         "pass": bool(gate)}
    if mesh:
        s["mesh_self_collision_free"] = v.get("mesh_self_collision_free")
        s["pass_incl_mesh"] = bool(gate and v.get("mesh_self_collision_free"))
    return s


def export_scene_cell(run: Path, repo: Path) -> dict:
    import trajgen_umarell_check as TU
    return TU.export_cell(run, repo)


def validators_exported_cell(run: Path, ex: dict) -> dict:
    import trajgen_umarell_check as TU
    t0 = time.time()
    out = TU.validate(run, ex)
    keep = {k: out.get(k) for k in ("cell_id", "robots", "unsupported", "composite_vs_timeline_max_rad", "pass")}
    keep["approximations"] = [a.get("what") for a in out.get("approximations") or []]
    keep["export_fk_check"] = (out.get("export_fk_check") or {}).get("pass")
    keep["inter_robot_clearance"] = {k: (out.get("inter_robot_clearance") or {}).get(k)
                                     for k in ("collision_free", "min_clearance_m", "samples_checked")}
    keep["dense_clearance"] = {k: (out.get("dense_clearance") or {}).get(k) for k in ("collision_free", "min_clearance_m")}
    vl = out.get("validate_trajectory_set_per_leg") or {}
    keep["validate_trajectory_set_per_leg"] = f"{vl.get('n_free')}/{vl.get('n')}" if vl else None
    keep["validate_trajectory_set_full"] = (out.get("validate_trajectory_set_full") or {}).get("collision_free")
    va = out.get("validate_approach") or {}
    keep["validate_approach"] = f"{va.get('n_matches')}/{va.get('task_welds')}"
    keep["approach_failures"] = (va.get("failures") or [])[:5]
    keep["obstacles_free"] = (out.get("obstacles") or {}).get("all_free")
    keep["obstacle_breaches"] = [(b["robot"], b["op"]) for b in (out.get("obstacles") or {}).get("breaches", [])][:10]
    keep["tool"] = "scripts/trajgen_umarell_check.py export_cell + validate (N-robot umarell cell from the scene)"
    keep["wall_s"] = _r(time.time() - t0, 1)
    (run / "umarell_cell_check.json").write_text(json.dumps(out, indent=1, default=str))
    return keep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--repo", default=str(ROOT / "data" / "btexport" / "umarell_repo"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-executor", action="store_true")
    ap.add_argument("--no-validators", action="store_true")
    ap.add_argument("--mesh-audit", action="store_true")
    ap.add_argument("--executor-xml", action="store_true",
                    help="also run the executor on the tree parsed back from the .xml file (btcpp.from_xml + blackboard)")
    a = ap.parse_args(argv)
    run, repo = Path(a.run).resolve(), Path(a.repo).resolve()
    setup(repo)
    info = json.load(open(run / "check.json"))
    plan = json.load(open(run / "plan.json"))
    xml_text = (run / "plan.xml").read_text()
    rids = info["robots"]
    rep = {"plan": str(run / "plan.json"), "tree_id": plan["tree_id"], "repo": str(repo), "robots": rids}
    t0 = time.time()
    if info.get("umarell_cell"):
        cell_id = info["umarell_cell"]
        if not a.no_validators:
            rep["validators"] = validators_umarell_cell(run, cell_id, a.mesh_audit)
    else:
        ex = export_scene_cell(run, repo)
        cell_id = ex["cell_id"]
        if not a.no_validators:
            rep["validators"] = validators_exported_cell(run, ex)
    rep["cell_id"] = cell_id
    if info.get("umarell_cell"):
        rep["robot_model"] = robot_model_check(plan, cell_id, repo)
    rep["load"], loaded = load_check(plan, cell_id)
    rep["structure"] = structure_check(loaded or plan["behaviour_tree"])
    rep["xml"] = xml_check(plan["behaviour_tree"], xml_text)
    rep["schedule"] = schedule_check(plan["behaviour_tree"])
    if not a.no_executor:
        z = np.load(run / "timeline.npz", allow_pickle=False)
        from trajgen_independent_check import legs_of
        legs = legs_of(z)
        home = np.asarray(z["home"], float)
        rep["executor"] = executor_dry_run(dict(plan, cell_id=cell_id), cell_id, rids, home, legs)
    if a.executor_xml:
        from agents.trajectory.tools import btcpp
        z = np.load(run / "timeline.npz", allow_pickle=False)
        from trajgen_independent_check import legs_of
        parsed = btcpp.from_xml(xml_text, plan["behaviour_tree"].get("blackboard") or {})
        parsed.update(tree_id=plan["tree_id"], cell_id=cell_id)
        rep["executor_xml"] = executor_dry_run({"cell_id": cell_id, "behaviour_tree": parsed}, cell_id, rids,
                                               np.asarray(z["home"], float), legs_of(z))
        rep["executor_xml"]["source"] = "btcpp.from_xml(plan.xml, blackboard)"
    parts = (["robot_model"] if "robot_model" in rep else []) + ["load", "structure", "xml"] + \
        (["validators"] if "validators" in rep else []) + \
            (["executor"] if "executor" in rep else []) + (["executor_xml"] if "executor_xml" in rep else [])
    rep["pass_by_part"] = {p: bool(rep[p].get("pass")) for p in parts}
    rep["pass"] = all(rep["pass_by_part"].values())
    rep["wall_s"] = _r(time.time() - t0, 1)
    out = Path(a.out) if a.out else run / "umarell_check.json"
    out.write_text(json.dumps(rep, indent=1, default=str))
    print(f"[btexport umarell_check] {plan['tree_id']} on {cell_id}: pass={rep['pass']} {rep['pass_by_part']} "
          f"({rep['wall_s']} s)", flush=True)
    return 0 if rep["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
