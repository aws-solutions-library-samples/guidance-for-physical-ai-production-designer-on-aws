"""CLI: python -m tools.retime --timeline X.npz [--cell ID | --scene DIR] --profile full|umarell --out Y.npz

Retimes any controller's timeline (player Timeline / trajgen NTimeline npz, or a umarell BT plan json) to a speed
profile: transit at the URDF joint velocity limits (x ``--v-scale``) with acceleration / jerk ramps, weld
operations (approach descent, hold, lift-off) at the weld speed; one shared clock with synchronisation waits where
the new relative timing would break the arm-arm clearance; dense re-check of the result. Writes the retimed
NTimeline (Y.npz, 250 Hz, with qd / qdd / labels) and a JSON report (Y.json). docs/p3_speed.md.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def main(argv=None):
    from tools.retime.context import dumps, guess_context, load_context
    from tools.retime.core import RetimeCfg, check_result, dynamics, load_source, retime, save, summary
    ap = argparse.ArgumentParser(prog="python -m tools.retime", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timeline", required=True, help="Timeline / NTimeline npz, or a umarell BT plan json")
    ap.add_argument("--cell", help="umarell cell id (default: from the timeline's meta)")
    ap.add_argument("--scene", help="trajgen scene dir or scene.yaml (default: from the meta / sample dir)")
    ap.add_argument("--profile", default="full", choices=("full", "umarell"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--mode", default="auto", choices=("auto", "independent", "coupled"))
    ap.add_argument("--v-scale", type=float, help="transit velocity fraction of the URDF limit (full: 1.0)")
    ap.add_argument("--a-scale", type=float, help="transit acceleration fraction (full: 1.0)")
    ap.add_argument("--weld-v", type=float, help="weld-operation velocity fraction (default 0.28 = umarell 0.7 x 0.4)")
    ap.add_argument("--weld-a", type=float, help="weld-operation acceleration fraction (default weld_v^2 = 0.078)")
    ap.add_argument("--approach-offset", type=float, help="weld zone: TCP within this of the weld (default 0.10 m)")
    ap.add_argument("--hold-s", type=float, default=0.5)
    ap.add_argument("--no-jerk", action="store_true", help="no jerk limit (plain TOPP, bang-bang acceleration)")
    ap.add_argument("--j-frac", type=float, help="jerk limit in a_max per second (full 10, umarell 5)")
    ap.add_argument("--accel", help="override acceleration limits, 6 comma-separated rad/s^2 (all robots)")
    ap.add_argument("--jerk", help="override jerk limits, 6 comma-separated rad/s^3 (all robots)")
    ap.add_argument("--check-hz", type=float, default=400.0)
    ap.add_argument("--no-check", action="store_true")
    ap.add_argument("--bt-out", help="also export a umarell behaviour tree (plan JSON + .xml), tools/retime/bt.py")
    a = ap.parse_args(argv)
    src = load_source(a.timeline)
    cell, scene = a.cell, a.scene
    if cell is None and scene is None:
        cell, scene = guess_context(src.meta, a.timeline)
        if cell is None and scene is None:
            ap.error("cannot tell the cell / scene from the timeline: give --cell or --scene")
    from robotmodel.override import active_override
    if scene is not None and active_override() is not None:
        print("tools.retime: a robot override (KWR_ROBOT_OVERRIDE) is not applied to trajgen scenes (the scene's "
              "robots[i].urdf / joints define the robots); point the scene at the URDF instead. For other "
              "acceleration limits use --accel.", file=sys.stderr)
        return 2
    vec = lambda s: None if s is None else [float(x) for x in s.split(",")]   # noqa: E731
    ctx = load_context(cell=cell, scene=scene, accel=vec(a.accel), jerk=vec(a.jerk))
    cfg = RetimeCfg(profile=a.profile, mode=a.mode, v_scale=a.v_scale, a_scale=a.a_scale, weld_v=a.weld_v,
                    weld_a=a.weld_a,
                    approach_offset_m=a.approach_offset, hold_s=a.hold_s, jerk=not a.no_jerk, j_frac=a.j_frac)
    res = retime(src, ctx, cfg)
    dyn = dynamics(res)
    chk = None if a.no_check else check_result(res, hz=a.check_hz)
    rep = summary(res, chk, dyn)
    rep["limits"] = ctx.limits_report()
    rep["cfg"] = cfg.to_dict()
    out = Path(a.out)
    save(res, out, report={k: rep[k] for k in ("makespan_s", "last_weld_s", "mode", "profile")})
    if a.bt_out:
        from tools.retime.bt import export_bt
        from trajgen.ntimeline import NTimeline
        cid = ctx.name if ctx.kind == "cell" else (getattr(ctx.scene, "umarell_cell", None) or ctx.scene.cell_id)
        rep["bt_export"] = export_bt(NTimeline.load(out), cid, a.bt_out,
                                     meta=dict(source_timeline=str(a.timeline), profile=a.profile, mode=res.mode))
        print(f"BT export: {rep['bt_export']}")
    out.with_suffix(".json").write_text(dumps(rep))
    s = rep.get("safety", {})
    print(f"{a.timeline}: {res.mode}, profile {a.profile}: last weld {rep['last_weld_s']} s, end {rep['makespan_s']} s "
          f"(source {rep['source_last_weld_s']} / {rep['source_end_s']} s), sync waits {rep['sync']}, "
          f"v/a max {dyn['all']['v_ratio_max']}/{dyn['all']['a_ratio_max']}, "
          f"safety {'VALID' if s.get('valid') else ('not checked' if chk is None else 'INVALID')} "
          f"{s.get('new_or_worse') or ''} -> {out}")
    return 0 if (chk is None or chk["valid"]) else 3


if __name__ == "__main__":
    sys.exit(main())
