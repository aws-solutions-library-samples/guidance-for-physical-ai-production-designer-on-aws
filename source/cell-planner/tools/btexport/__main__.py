"""CLI: export any tool's motion as a umarell behaviour-tree plan (JSON envelope + separate BTCPP v4 XML).

    PYTHONPATH=. .venv/bin/python -m tools.btexport --from <spec> [--cell welding_cell_2_small] --out plan.json
        [--xml plan.xml] [--profile umarell|full] [--backend native|retime] [--retime-cut sync|windows]
        [--tree-id ID] [--validate [--no-executor] [--mesh-audit] [--executor-xml]] [--dense-check]
        [--urdf U] [--robot-base RID=X,Y,Z[,YAW]] [--robot-accel A1,..,A6] [--robot-override SPEC]
        [--report report.json] [--vass off|plan|fifo [--vass-nodes] [--vass-required]]

Robot kinematics and limits: the URDF of each robot (joint limits incl. <limit velocity>), accelerations from the
override / robots.yaml / scene.yaml, else the KR240 REVIEW default. --urdf / --robot-* (robotmodel.override) move the
controllers, the retimer and the export to another URDF / base / acceleration limits together (umarell's validators
are then skipped if the geometry changed: they load the cell's own URDF).

--from specs (tools/btexport/sources.py): expert[:assign] | stateless | hier:<ckpt>[,la=auto|<mode>] |
hier_la:<ckpt> | residual:<ckpt> | e2e:<ckpt> | ppo:<ckpt> (run first) | umarell:<plan> |
trajgen:<scene|sample>[,<args>] (run first) | tgrun:<run dir> | timeline:<npz> (N robots: scene from meta.source or
--scene) | exec:<results/exec_*.json> | plan:<plan.json> | a bare path.
--scene alone is an alias of --from trajgen:<scene>.

Export checks (export_pass): executor timing, legs at rest at every pause, real weld holds, unique ids, the
absolute URDF limits (segment velocity <= v_max, 0.05 s acceleration <= a_max x 1.05, positions inside
<limit lower upper>; fails even when the source already broke them), weld operations slow (full profile), every
task weld present (--allow-partial to accept a partial source).

Exit codes: 0 exported (and, with --validate, every umarell check passed); 1 exported but a check failed;
2 unsupported / malformed input (bad spec, missing or corrupt file or URDF, empty plan, a plan with unresolved
blackboard keys / empty trajectories / non-finite or non-increasing waypoints / wrong joint count / velocity_scale
<= 0, invalid override or parameter, an override naming a robot that is not in the cell, a robot override on a
trajgen scene source); nothing is written then.

Robot override and umarell's validators: they are skipped only when the override changes the robot geometry (URDF
content, base, yaw, tip_link, tcp_offset, home); an acceleration-only override keeps them. The report's
``robot_override.applied`` lists what changed per robot; the plan records it in metadata.robot_model.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.btexport", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="src", default=None, help="source spec (see above)")
    ap.add_argument("--scene", default=None, help="shortcut for --from trajgen:<scene>; with --from timeline:<npz>: "
                                                  "the trajgen scene of an N-robot timeline (default: its meta.source)")
    ap.add_argument("--cell", default=None, help="umarell cell for controllers / umarell plans / timelines "
                                                 "(default welding_cell_2_small)")
    ap.add_argument("--out", required=True, help="plan JSON (umarell plan_store envelope)")
    ap.add_argument("--xml", default=None, help="BTCPP v4 XML file (default: <out> with .xml)")
    ap.add_argument("--profile", choices=["umarell", "full"], default="umarell",
                    help="umarell: the source's timing (rollouts: execute.py sync_rest, 0.49 a_max envelope); full: "
                         "non-weld motion at the URDF velocity limits with acceleration ramps, weld ops slow")
    ap.add_argument("--tree-id", default=None)
    ap.add_argument("--report", default=None, help="export report JSON (default: <out> with .report.json)")
    ap.add_argument("--work", default=str(ROOT / "data" / "btexport" / "work"), help="trajgen runs of trajgen: specs")
    ap.add_argument("--retime-work", default=None,
                    help="retime backend: work dir for tools.retime (kept; its cache applies). Default: a unique dir "
                         "data/btexport/work/retime_<tree id>_<pid>_<random> per export, removed afterwards")
    ap.add_argument("--keep-work", action="store_true", help="keep the unique retime work dir")
    ap.add_argument("--home-tail", choices=["expert", "controller", "off"], default="expert")
    ap.add_argument("--home-tol", type=float, default=1e-4)
    ap.add_argument("--max-steps", type=int, default=3000)
    ap.add_argument("--safety-filter", action="store_true")
    ap.add_argument("--acc-frac-full", type=float, default=1.0, help="full profile: acceleration limit x a_max")
    ap.add_argument("--weld-vfrac", type=float, default=0.28, help="full profile: weld-operation joint speed x v_max")
    ap.add_argument("--weld-radius", type=float, default=0.10, help="full profile: weld-operation TCP radius (m)")
    ap.add_argument("--full-backend", choices=["warp", "retime"], default="warp",
                    help="(version-1 name) full profile: warp = this package's common-clock time scaling; retime = "
                         "same as --backend retime")
    ap.add_argument("--backend", choices=["native", "retime"], default=None,
                    help="native: umarell profile = the source's timing, full = this package's warp; retime: track "
                         "SPEED's python -m tools.retime --profile <profile> first (jerk-limited ramps, weld ops "
                         "slow, sync waits, any number of robots). Default native; gated N-robot plans need retime "
                         "in the full profile")
    ap.add_argument("--retime-cut", choices=["sync", "windows"], default="sync",
                    help="retime backend: sync = per-weld leaves, all arms at rest at every executor pause (umarell's "
                         "validators apply; never cuts inside another arm's weld operation, falls back to windows "
                         "when a weld offers no such instant; +1.5 s pause per weld); windows = tools.retime.bt, the "
                         "retimed clock exactly, leaves may hold several welds (fastest full-speed export)")
    ap.add_argument("--retime-mode", choices=["auto", "independent", "coupled"], default="auto")
    ap.add_argument("--dense-check", action="store_true",
                    help="our dense 400 Hz safety re-check of the exported plan vs its source (tools.btexport.dense)")
    ap.add_argument("--validate", action="store_true", help="run umarell's loaders / validators / executor on it")
    ap.add_argument("--no-executor", action="store_true")
    ap.add_argument("--mesh-audit", action="store_true")
    ap.add_argument("--executor-xml", action="store_true", help="--validate: also run umarell's executor on the tree "
                                                                    "parsed back from the .xml file alone")
    ap.add_argument("--repo", default=str(ROOT / "data" / "btexport" / "umarell_repo"))
    ap.add_argument("--weld-afrac", type=float, default=None,
                    help="full warp: weld-operation acceleration x a_max (e.g. 0.0784 = umarell's weld rule 0.28^2; "
                         "default: full acceleration; on kinked rollout paths this cap makes the zones very slow)")
    ap.add_argument("--weld-source-cap", action="store_true",
                    help="full warp: never faster than the source inside a weld operation (on top of the caps)")
    ap.add_argument("--no-limits-gate", action="store_true",
                    help="diagnostics only: the absolute URDF limit check (velocity / acceleration / position) is "
                         "reported but does not fail the export")
    ap.add_argument("--allow-partial", action="store_true",
                    help="a source that did not complete every weld of its task may pass the export checks")
    ap.add_argument("--vass", choices=["off", "plan", "fifo", "priority"], default="off",
                    help="round 3 (docs/r3_vass.md): VASS PLC interlocks -- derive the zone table of the exported plan "
                         "and annotate every leg (interlocks port, metadata.vass, <out stem>.zones.json); off = the "
                         "plan exactly as before")
    ap.add_argument("--vass-nodes", action="store_true",
                    help="with --vass: also RequestInterlock / ReleaseInterlock nodes around the legs (VASS-aware "
                         "executors only; today's executors refuse unknown nodes)")
    ap.add_argument("--vass-required", action="store_true",
                    help="with --vass: metadata.vass.required = true (a VASS-aware executor refuses to run it without "
                         "a matching PLC)")
    from robotmodel.override import add_cli_args, apply_cli_args
    add_cli_args(ap)
    a = ap.parse_args(argv)
    for name, v in (("--acc-frac-full", a.acc_frac_full), ("--weld-vfrac", a.weld_vfrac),
                    ("--weld-afrac", a.weld_afrac)):
        if v is not None and not (0.0 < v <= 1.0):
            ap.error(f"{name} must be in (0, 1], got {v}")
    if not a.weld_radius > 0:
        ap.error(f"--weld-radius must be > 0, got {a.weld_radius}")
    tl_scene = None
    if a.src and a.scene:
        if not (a.src.startswith("timeline:") or a.src.endswith(".npz")):
            ap.error("--scene with --from is only for timeline: sources (the scene of an N-robot timeline)")
        tl_scene = a.scene
    elif not a.src and not a.scene:
        ap.error("give one of --from / --scene")
    spec = a.src or f"trajgen:{a.scene}"

    from tools.btexport import core as C
    from tools.btexport import sources as S
    out = Path(a.out)
    xml = Path(a.xml) if a.xml else out.with_suffix(".xml")
    rpath = Path(a.report) if a.report else out.with_name(out.stem + ".report.json")
    import xml.etree.ElementTree as ET
    from robotmodel.override import active_override
    from tools.btexport import limits as L
    input_errors = (FileNotFoundError, json.JSONDecodeError, ValueError, KeyError, IndexError, TypeError,
                    ET.ParseError)

    def _input_error(e) -> int:
        if isinstance(e, C.Unsupported):
            print(f"[btexport] unsupported: {e}", file=sys.stderr)
        elif isinstance(e, SystemExit):
            print(f"[btexport] input error: {e.code}", file=sys.stderr)
        else:
            print(f"[btexport] input error: {type(e).__name__}: {str(e).splitlines()[0] if str(e) else ''}",
                  file=sys.stderr)
        return 2

    # input / validation errors -> one line + exit 2 (the documented contract: 1 = exported but a check failed)
    eff = None
    try:
        apply_cli_args(a)
        kind = spec.partition(":")[0]
        if active_override() is not None and kind in ("trajgen", "tgrun"):
            raise C.Unsupported("a robot override is not applied to trajgen scene sources (the scene's "
                                "robots[i].urdf / joints define the robots): point the scene at the URDF and re-plan")
        if active_override() is not None and kind in S.CONTROLLER_KINDS:
            _check_override_robots(L.override_effect(a.cell or "welding_cell_2_small"), C)   # before the rollout
        src = S.resolve(spec, cell=a.cell, work=Path(a.work), profile=a.profile, home_tail=a.home_tail,
                        home_tol=a.home_tol, max_steps=a.max_steps, safety_filter=a.safety_filter, scene=tl_scene)
        if active_override() is not None:
            if src.scene is not None and not src.umarell_cell:
                raise C.Unsupported("a robot override is not applied to trajgen scene sources (the scene's "
                                    "robots[i].urdf / joints define the robots): point the scene at the URDF")
            eff = L.override_effect(src.cell_id)
            _check_override_robots(eff, C)
            if not eff["applied"]:
                print("[btexport] warning: the robot override changes nothing in this cell (same URDF content, "
                      "base, TCP, home and limits)", flush=True)
    except SystemExit as e:
        if isinstance(e.code, int) or e.code is None:
            raise
        return _input_error(e)
    except (C.Unsupported,) + input_errors as e:
        return _input_error(e)
    try:
        plan, rep = C.export(src, profile=a.profile, tree_id=a.tree_id or out.stem, out=out, xml=xml,
                             acc_frac_full=a.acc_frac_full, weld_vfrac=a.weld_vfrac, weld_radius=a.weld_radius,
                             full_backend=a.full_backend, backend=a.backend, retime_cut=a.retime_cut,
                             retime_mode=a.retime_mode, weld_source_cap=a.weld_source_cap, weld_afrac=a.weld_afrac,
                             limits_gate=not a.no_limits_gate, allow_partial=a.allow_partial,
                             work=Path(a.retime_work) if a.retime_work else None, keep_work=a.keep_work,
                             vass=a.vass, vass_nodes=a.vass_nodes, vass_required=a.vass_required)
    except SystemExit as e:
        if isinstance(e.code, int) or e.code is None:
            raise
        return _input_error(e)
    except (C.Unsupported,) + input_errors as e:
        return _input_error(e)
    t = rep["timing"]
    ch = rep["checks"]
    lim = ch["limits"]
    print(f"[btexport] {spec} -> {out} + {xml}: profile {a.profile}, {plan['num_operations']} operations, "
          f"motion {t['makespan_motion_s']} s, executor end {t['executor_end_s']} s, export checks "
          f"{'pass' if rep['export_pass'] else 'FAIL'}", flush=True)
    print(f"[btexport] URDF limits: segment v {lim['segment_vel_frac_vmax_max']} x v_max, a {lim['dense_acc_frac_amax_max']}"
          f" x a_max (0.05 s), joint-limit excess {lim['joint_limit_excess_rad']} rad -> "
          f"{'pass' if lim['pass'] else 'FAIL'}" + (" (inherited from the source)" if any(lim['inherited'].values())
                                                   else ""), flush=True)
    for k in ("timing", "rest", "welds", "weld_rule", "complete"):
        if ch.get(k) is not None and not ch[k].get("pass", True):
            print(f"[btexport] check {k} FAILED: " + json.dumps({kk: vv for kk, vv in ch[k].items()
                                                                  if kk in ("max_speed_frac_vmax", "cap", "missing",
                                                                            "max_end_error_mm", "min_hold_s",
                                                                            "n_legs_not_at_rest", "not_at_rest",
                                                                            "metric")}),
                  flush=True)
    if rep.get("vass"):
        v = rep["vass"]
        print(f"[btexport] vass {v.get('mode')}: " + (f"{v.get('zones')} zones, derive {v.get('derive_s')} s, interlocked "
              f"nominal run complete={v.get('nominal_complete')} overhead={v.get('nominal_overhead')}" if v.get("ok")
              else f"FAILED ({v.get('code')}): {str(v.get('error') or v.get('problems'))[:200]}"), flush=True)
    for w in rep.get("warnings") or []:
        print(f"[btexport] warning: {w}", flush=True)
    ok = rep["export_pass"]
    if active_override() is not None:
        # the override as actually applied per robot (P3U round 2), next to the raw spec
        rep["robot_override"] = {"spec": active_override(), "applied": eff}
    if a.dense_check:
        from tools.btexport import dense as D
        rep["dense"] = D.plan_check(plan, src)
        ok = ok and rep["dense"]["valid"]
        print(f"[btexport] dense check: {'VALID' if rep['dense']['valid'] else 'INVALID'} "
              f"{rep['dense']['new_or_worse'] or ''}", flush=True)
    n_cell = src.scene is None and set(src.robot_ids) != {"robot1", "robot2"}
    if a.validate and n_cell:
        # umarell's validators / executor dry run (tools.btexport.umarell_check) are two-arm: not a failure
        why = (f"--validate skipped: umarell's validators are two-arm (robot1 / robot2), this cell has "
               f"{len(src.robot_ids)} arms {list(src.robot_ids)}; the dense check (--dense-check) covers every pair")
        rep["umarell"] = {"pass": None, "skipped": why}
        rep.setdefault("warnings", []).append(why)
        print(f"[btexport] warning: {why}", flush=True)
    elif a.validate:
        from tools.btexport import validate as V
        run = rpath.with_name(out.stem + "_umarell")
        V.prepare(run, plan, src)
        # umarell's validators load the cell's own robot: skipped only when the override changes the geometry
        # (URDF / base / TCP / home); an acceleration-only override keeps them
        geo = bool(eff and eff.get("geometry_changed"))
        r = V.run_check(run, repo=Path(a.repo), executor=not a.no_executor, mesh_audit=a.mesh_audit,
                        executor_xml=a.executor_xml, validators=not geo)
        if geo:
            r["validators_skipped"] = ("robot override changes the robot geometry (" + ", ".join(
                f"{rid}: {sorted(k for k in e if k in L.GEOMETRY_KEYS)}" for rid, e in eff["robots"].items()
                if e.get("geometry_changed")) + "): umarell's validators load the cell's own URDF")
            print(f"[btexport] umarell validators skipped ({r['validators_skipped']}); load / structure / xml / "
                  "executor still run", flush=True)
        rep["umarell"] = V.summary(r)
        rep["umarell_report"] = str(run / "umarell_check.json")
        ok = ok and bool(r.get("pass"))
        print(f"[btexport] umarell checks: {json.dumps(rep['umarell'])}", flush=True)
    rpath.parent.mkdir(parents=True, exist_ok=True)
    rpath.write_text(json.dumps(rep, indent=1, default=str))
    return 0 if ok else 1


def _check_override_robots(eff, C):
    if eff and eff.get("unknown_robots"):
        raise C.Unsupported(f"robot override names robot(s) {eff['unknown_robots']} that are not in the cell "
                            f"(robots: {eff['cell_robots']}; use '*' for all)")


if __name__ == "__main__":
    sys.exit(main())
