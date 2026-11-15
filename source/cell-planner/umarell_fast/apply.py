#!/usr/bin/env python3
"""Create a writable umarell working copy for an umarell-fast variant and apply its patches.

    python3 umarell_fast/apply.py                          # -> data/umarell_fast/repo_U1_dc60669/ (newest base, U patches)
    python3 umarell_fast/apply.py --with-upstream-fixes    # -> data/umarell_fast/repo_U1_dc60669_fix/ (set v9)
    python3 umarell_fast/apply.py --base 76ea13a --with-upstream-fixes   # -> repo_U1_76ea13a_fix/ (set v8)
    python3 umarell_fast/apply.py --base eccb1bc           # -> data/umarell_fast/repo_U1/ (old base, U patches)
    python3 umarell_fast/apply.py --with-upstream-fixes --upstream-fixes-set v5   # -> repo_U1_fix/ (old base, set v5)
    python3 umarell_fast/apply.py --with-upstream-fixes --upstream-fixes-set v4   # -> repo_U1_fix_v4/ (old base)
    python3 umarell_fast/apply.py --with-upstream-fixes --upstream-fixes-set v1   # -> repo_U1_fix_v1/ (old set)
    python3 umarell_fast/apply.py --variant U1 --dest /path/to/copy --force

Three umarell bases (``BASES``), each a read-only, never modified copy with its own fix and hook patch folders:

* ``eccb1bc`` (old base): ``third_party/sample-umarell``, fixes ``docs/upstream/patches``, hooks
  ``umarell_fast/patches``; fix sets v1 .. v5 / v4r3, all byte-for-byte as before (same patches, same destinations
  ``repo_<V>``, ``repo_<V>_fix`` = v5, ``repo_<V>_fix_<set>``).
* ``76ea13a`` (umarell HEAD 2026-10-23 .. 2026-11-03): ``third_party/sample-umarell-76ea13a``, fixes
  ``docs/upstream/patches_76ea13a`` (series.txt there), hooks ``umarell_fast/patches_76ea13a``; fix set v8 (since
  2026-10-28; v7 2026-10-27, v6 before). Destinations ``repo_<V>_76ea13a``, ``repo_<V>_76ea13a_fix`` (= v8),
  ``.._fix_v7``, ``.._fix_v6``.
* ``dc60669`` (umarell HEAD since 2026-11-03, the default since 2026-11-04): ``third_party/sample-umarell-dc60669``,
  fixes ``docs/upstream/patches_dc60669`` (series.txt there: set v9 = v8 rebased, 26 dropped because umarell now
  appends the trajectory index itself), hooks ``umarell_fast/patches_dc60669`` (the 76ea13a hooks, unchanged but
  for line numbers). Destinations ``repo_<V>_dc60669``, ``repo_<V>_dc60669_fix`` (= v9).
* ``08d5a39`` (umarell HEAD since 2026-11-04, PR #13; opt-in: ``--base 08d5a39``): ``third_party/sample-umarell-08d5a39``,
  fixes ``docs/upstream/patches_08d5a39`` (set v10 = v9 rebased, 22v3 adjusted to 13d446b's Numba box kernel), hooks
  ``umarell_fast/patches_08d5a39`` (U1_01 context only, U1_02, U3_02; U3_01 dropped: 08d5a39 parses robots.yaml once
  per content itself, ``cell_layout.load_yaml_text``). Destinations ``repo_<V>_08d5a39``, ``repo_<V>_08d5a39_fix``.

Which base: ``--base`` if given, else the base of ``--upstream-fixes-set`` (default v9, so dc60669) -- WITH OR WITHOUT
``--with-upstream-fixes``: ``apply.py`` alone builds the newest base, ``--upstream-fixes-set v5`` alone the old one.
``--base eccb1bc --with-upstream-fixes`` applies that base's default set (v5); a set of another base is an error.

Steps:
1. rsync the base (``BASES[base].src``) to the destination, without .git / __pycache__ / venvs / test caches. The
   copy contains umarell's ``cells/`` (robot meshes and the GEO_FIXTURE_OP1000 fixture CAD): ``data/umarell_fast/``
   is gitignored and must never be synced to S3.
2. With ``--with-upstream-fixes``: apply the verified correctness fixes of the base's fix folder in order.
   ``--upstream-fixes-set v9`` (default since 2026-11-04, base dc60669) = v8 on dc60669: 04v3, 22v3 (the tool's
   workpiece rule on umarell's ``tool_contact`` boxes: dc60669 reads ``is_workpiece`` as ``tool_contact`` and checks the
   arm against it, but exempted the tool spheres anywhere on the box and never checked the tool hulls), 20v2, 23v2,
   21v5, 24v2 (on dc60669's exact triangle / voxel test), 25v2 (``--refine-points`` beside dc60669's
   ``--weld-targets``), 27; 26 dropped (dc60669 appends ``trajectory_index.jsonl`` and writes through per-writer
   temp files).
   ``--upstream-fixes-set v8`` (default since 2026-10-28, base 76ea13a) = v7 + 27 Hermite fallback at rest (without
   TOPP-RA, the arm64 hosts: every timed path starts and ends at rest).
   ``--upstream-fixes-set v7`` (default 2026-10-27 .. 2026-10-28, base 76ea13a) = v6 + 24 voxel boundary inward, 25
   near-weld refinement (converter ``--refine-points``), 26 trajectory index race.
   ``--upstream-fixes-set v6`` (default 2026-10-23 .. 2026-10-27, base 76ea13a) = 04v3 hull backstop, 22v2 no workpiece dip,
   20v2 driver continuous, 23v2 commanded acceleration, 21v5 VASS interlocks (umarell 76ea13a ported 01-03, 05-19
   and the link-sphere part of 04v2: docs/upstream/patches_76ea13a/series.txt). A missing patch file is reported by
   name before anything is copied.
   Old base eccb1bc: ``--upstream-fixes-set v5`` (default 2026-10-18 .. 2026-10-23, round 4) = v4 with 21 v3
   (21_vass_interlocks.diff is now 21 v4: the executor / PLC liveness fixes, the fast zone derivation with the exact
   pair re-ordering, splices on the joint_trajectory_controller's clock, the dropped-goal fix) + 23 commanded
   acceleration (TOPP-RA legs timed so that fix 20's driver command stays within the acceleration limits).
   ``--upstream-fixes-set v4r3`` = set v4 exactly as shipped in round 3 (21 v2, ``21v2_vass_interlocks.diff``);
   destination ``repo_<V>_fix_v4r3``.
   ``--upstream-fixes-set v4`` (default 2026-10-17 .. 2026-10-18, round 3) = v3 + 20 driver continuous (velocities /
   accelerations forwarded, simulators interpolate like joint_trajectory_controller), 21 VASS interlocks over zenoh
   (zone derivation, PLC emulator, supervised legs; default OFF: robots.yaml ``vass:`` / ``UMARELL_VASS_*``), 22 no
   workpiece dip (tool hulls / spheres never inside a workpiece or fixture box except the weld contact). 21 needs
   ``eclipse-zenoh`` in the interpreter for the bus (it falls back to an in-process bus without it).
   ``--upstream-fixes-set v3`` = v2 + 16 replan start gap, 17 blocked lift-off drag,
   18v2 spline stays on the validated path, 19 home lift-off blocked. ``--upstream-fixes-set v2``: 01 self-collision, 02 tool spheres, 03 tool vs
   floor/obstacles, 05 departure lift-off, 06 fixture per-step check, 07 entry check, 04v2 link-sphere envelope
   (hull backstop), 12 IK spin/tilt, 13 joint-limit gate, 14 dense self-collision, 15 agent-leg lift-off.
   ``--upstream-fixes-set v1``: the old set 01/02/03/05/06/07 only. 02, 03 and 04v2 also edit the cells'
   robots.yaml / obstacles.yaml of the copy.
3. Apply the base's ``<hooks>/<variant>_*.diff`` (our own hook patches, default-off behaviour) in name order.
   76ea13a: U1_01 agent hooks, U1_02 effort / 1 h cache TTL (the prompt cache itself is upstream), U3_01 robots.yaml
   parse cache (the model cache is upstream), U3_02 OMPL seed; U2 has none (U2_01 was ported upstream).
4. Write ``UF_APPLIED.json`` (base, sources, patch list with sha256) into the copy.

The overlay code (``umarell_fast.<variant>``) is NOT copied: drivers put the project root on ``sys.path``.
Uses ``patch -p1 --forward --batch`` (the copy is inside this git repo, so ``git apply`` would resolve paths against
the wrong root). Fails loudly on any rejected hunk.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
# the old base's names (eccb1bc), kept for callers of this module (u3/__main__.py, tests)
SRC = ROOT / "third_party" / "sample-umarell"
UPSTREAM = ROOT / "docs" / "upstream" / "patches"
OURS = ROOT / "umarell_fast" / "patches"
BASES = {
    "eccb1bc": SimpleNamespace(head="eccb1bc", commit="eccb1bc7ee6d3c1df14f6d455bba4f50ab49d62f", src=SRC,
                               fixes=UPSTREAM, ours=OURS, default_set="v5", suffix=""),
    "76ea13a": SimpleNamespace(head="76ea13a", commit="76ea13aafdeede10be11936cf0830613e8717907",
                               src=ROOT / "third_party" / "sample-umarell-76ea13a",
                               fixes=ROOT / "docs" / "upstream" / "patches_76ea13a",
                               ours=ROOT / "umarell_fast" / "patches_76ea13a", default_set="v8", suffix="_76ea13a"),
    "dc60669": SimpleNamespace(head="dc60669", commit="dc60669576cca17d2ca6b58713dcd0f2b4201b39",
                               src=ROOT / "third_party" / "sample-umarell-dc60669",
                               fixes=ROOT / "docs" / "upstream" / "patches_dc60669",
                               ours=ROOT / "umarell_fast" / "patches_dc60669", default_set="v9", suffix="_dc60669"),
    # umarell HEAD since 2026-11-04 (PR #13, perf/body-weld-2-planning); opt-in (--base 08d5a39 / UMARELL_BASE)
    "08d5a39": SimpleNamespace(head="08d5a39", commit="08d5a390a498cef123cfe36becfc45ee61876cd9",
                               src=ROOT / "third_party" / "sample-umarell-08d5a39",
                               fixes=ROOT / "docs" / "upstream" / "patches_08d5a39",
                               ours=ROOT / "umarell_fast" / "patches_08d5a39", default_set="v10", suffix="_08d5a39"),
}
UPSTREAM_FIX_SETS = {
    "v1": ("01_self_collision", "02_tool_spheres", "03_tool_floor_obstacles", "05_departure_drag",
           "06_fixture_blind_spot", "07_entry_check_zero_motion"),
}
UPSTREAM_FIX_SETS["v2"] = UPSTREAM_FIX_SETS["v1"] + (
    "04v2_link_spheres_envelope", "12_ik_spin_tilt", "13_joint_limits", "14_dense_self_collision",
    "15_agent_leg_liftoff")
UPSTREAM_FIX_SETS["v3"] = UPSTREAM_FIX_SETS["v2"] + (
    "16_replan_start_gap", "17_blocked_liftoff_drag", "18v2_spline_leaves_validated_path", "19_home_liftoff_blocked")
UPSTREAM_FIX_SETS["v4"] = UPSTREAM_FIX_SETS["v3"] + (
    "20_driver_continuous", "21_vass_interlocks", "22_no_workpiece_dip")  # 21 needs 20 (robot_agent.py)
# round 4 (2026-10-18, INTEGRATE): 21_vass_interlocks.diff is 21 v3 (U21FIX executor / PLC + DERIVE derivation + the
# ROS track's JTC-clock splice); v4r3 = set v4 as shipped in round 3 (21 v2); v5 = v4 (with 21 v3) + 23
UPSTREAM_FIX_SETS["v4r3"] = UPSTREAM_FIX_SETS["v3"] + (
    "20_driver_continuous", "21v2_vass_interlocks", "22_no_workpiece_dip")
UPSTREAM_FIX_SETS["v5"] = UPSTREAM_FIX_SETS["v4"] + ("23_commanded_acceleration",)  # 23 needs 20 (fjt_trajectory)
# 2026-10-23: base 76ea13a (umarell ported 01-03, 05-19 and 04v2's link spheres); only what is still missing,
# rebased: docs/upstream/patches_76ea13a/series.txt
UPSTREAM_FIX_SETS["v6"] = ("04v3_hull_backstop", "22v2_no_workpiece_dip", "20v2_driver_continuous",
                           "23v2_commanded_acceleration", "21v5_vass_interlocks")
# 2026-10-27: v7 = v6 + 24 voxel faces on a grid plane go inward, 25 near-weld refinement (converter
# --refine-points), 26 trajectory_index.json store race
UPSTREAM_FIX_SETS["v7"] = UPSTREAM_FIX_SETS["v6"] + ("24_voxel_boundary_inward", "25_near_weld_refinement",
                                                     "26_trajectory_index_race")
# 2026-10-28: v8 = v7 + 27 the Hermite fallback (no TOPP-RA: arm64) starts and ends every timed path at rest
UPSTREAM_FIX_SETS["v8"] = UPSTREAM_FIX_SETS["v7"] + ("27_hermite_rest_ends",)   # = patches_76ea13a/series.txt
# 2026-11-04: base dc60669 (= patches_dc60669/series.txt): v8 rebased; 22, 24, 25 re-done on dc60669's tool_contact
# boxes / exact voxel test / --weld-targets converter, 26 dropped (dc60669 fixed the index race its own way)
UPSTREAM_FIX_SETS["v9"] = ("04v3_hull_backstop", "22v3_no_workpiece_dip", "20v2_driver_continuous",
                           "23v2_commanded_acceleration", "21v5_vass_interlocks", "24v2_voxel_boundary_inward",
                           "25v2_near_weld_refinement", "27_hermite_rest_ends")
# 2026-11-05: base 08d5a39 (= patches_08d5a39/series.txt): v9 rebased, same patches and order; 22v3 adjusted (the
# compiled box kernel of 13d446b knows only upstream's blanket tool_contact exemption: not used under the workpiece rule)
# + 28: upstream's own d749dc5 (perf/body-weld-2-planning, not in main): the context compaction inside the model step
# instead of a graph node -- 08d5a39's node costs 3 recursion steps per turn and long LLM plans hit LangGraph's
# recursion limit (bench 2026-11-05: body_weld_4x8 / body_weld_2x8, ompl-compound, GRAPH_RECURSION_LIMIT)
UPSTREAM_FIX_SETS["v10"] = UPSTREAM_FIX_SETS["v9"] + ("28_context_compaction_in_model_step",)
FIX_SET_BASE = {s: ("08d5a39" if s == "v10" else "dc60669" if s == "v9" else
                    "76ea13a" if s in ("v6", "v7", "v8") else "eccb1bc")
                for s in UPSTREAM_FIX_SETS}
DEFAULT_FIX_SET = "v9"
UPSTREAM_FIXES = ("01", "02", "03", "05", "06", "07")  # old name, old meaning (the v1 numbers); use fix_diffs()


def fix_diffs(fix_set: str = DEFAULT_FIX_SET) -> list:
    """The fix diffs of a fix set (in its base's fix folder), in application order."""
    return [BASES[FIX_SET_BASE[fix_set]].fixes / f"{stem}.diff" for stem in UPSTREAM_FIX_SETS[fix_set]]


def hook_diffs(variant: str, base: str = "dc60669") -> list:
    """Our own hook patches of a variant for a base, in application order."""
    return sorted(BASES[base].ours.glob(f"{variant}_*.diff"))


HEAD = "eccb1bc"  # the old base (module constant kept for callers); a copy's real base: UF_APPLIED.json umarell_head


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def apply_patch(dest: Path, diff: Path) -> None:
    r = subprocess.run(["patch", "-p1", "--forward", "--batch", "--no-backup-if-mismatch", "-d", str(dest),
                        "-i", str(diff)], capture_output=True, text=True)
    if r.returncode != 0 or list(dest.rglob("*.rej")):
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit(f"patch failed: {diff.name}")
    print(f"[apply] {diff.name}: " + " | ".join(l for l in r.stdout.splitlines() if "offset" in l or "fuzz" in l)
          if ("offset" in r.stdout or "fuzz" in r.stdout) else f"[apply] {diff.name}: ok")


def main(argv=None) -> Path:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--variant", default="U1")
    ap.add_argument("--base", choices=sorted(BASES), default=None,
                    help="umarell base (default: the base of --upstream-fixes-set, i.e. dc60669 unless an older set "
                         "v6..v8 (76ea13a) / v1..v5 / v4r3 (eccb1bc) is named; also without --with-upstream-fixes)")
    ap.add_argument("--with-upstream-fixes", action="store_true")
    ap.add_argument("--upstream-fixes-set", choices=sorted(UPSTREAM_FIX_SETS), default=None,
                    help="which fix set --with-upstream-fixes applies (default: the base's default set, v9 on "
                         "dc60669 = v8 rebased without 26, v8 on 76ea13a = v7 + 27, v7 = v6 + 24/25/26, v6 = 04v3/22v2/20v2/23v2/21v5, v5 on eccb1bc; on eccb1bc: v5 = v4 with 21 v3 + 23; "
                         "v4r3 = v4 as shipped in round 3 (21 v2); v4 = v3 + 20/21/22; v3 = v2 + 16/17/18v2/19; "
                         "v2 = 01-07 + 04v2/12-15; v1 = 01/02/03/05/06/07)")
    ap.add_argument("--dest", default=None)
    ap.add_argument("--src", default=None,
                    help="the plain umarell checkout of --base to copy (default: the base's third_party/ tree)")
    ap.add_argument("--force", action="store_true", help="delete an existing destination first")
    a = ap.parse_args(argv)
    base = a.base or FIX_SET_BASE[a.upstream_fixes_set or DEFAULT_FIX_SET]
    B = BASES[base]
    fix_set = a.upstream_fixes_set or B.default_set
    if FIX_SET_BASE[fix_set] != base:
        raise SystemExit(f"fix set {fix_set} is for base {FIX_SET_BASE[fix_set]}, not --base {base}")
    src = Path(a.src).resolve() if a.src else B.src
    if not src.is_dir():
        raise SystemExit(f"umarell base {base} not found: {src}")
    diffs = (fix_diffs(fix_set) if a.with_upstream_fixes else []) + hook_diffs(a.variant, base)
    missing = [d for d in diffs if not d.is_file()]
    if missing:
        raise SystemExit(f"fix set {fix_set} (base {base}): missing patch file(s): "
                         + ", ".join(str(d.relative_to(ROOT)) for d in missing))
    name = f"repo_{a.variant}{B.suffix}" + ("_fix" if a.with_upstream_fixes else "")
    if a.with_upstream_fixes and fix_set != B.default_set:
        name += f"_{fix_set}"  # repo_<V>_fix_v1 / _v3 / _v4: an older set never overwrites the default copy
    dest = Path(a.dest) if a.dest else ROOT / "data" / "umarell_fast" / name
    if dest.exists():
        if not a.force:
            raise SystemExit(f"{dest} exists (use --force to recreate)")
        shutil.rmtree(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["rsync", "-a", "--exclude", ".git", "--exclude", "__pycache__", "--exclude", ".venv*",
                    "--exclude", ".hypothesis", "--exclude", ".pytest_cache", f"{src}/", f"{dest}/"], check=True)
    applied = []
    for diff in diffs:
        apply_patch(dest, diff)
        applied.append({"patch": str(diff.relative_to(ROOT)), "sha256_16": sha(diff)})
    (dest / "UF_APPLIED.json").write_text(json.dumps({
        "variant": a.variant, "source": str(src.relative_to(ROOT)) if src.is_relative_to(ROOT) else str(src),
        "umarell_head": B.head,
        "umarell_commit": B.commit, "base": base,
        "with_upstream_fixes": a.with_upstream_fixes,
        "upstream_fixes_set": fix_set if a.with_upstream_fixes else None, "patches": applied,
        "overlay": f"umarell_fast.{a.variant.lower()} (imported from {ROOT}, not copied)",
        "created": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}, indent=1))
    print(f"[apply] working copy ready: {dest}")
    return dest


if __name__ == "__main__":
    main()
