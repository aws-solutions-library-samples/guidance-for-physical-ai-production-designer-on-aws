#!/usr/bin/env python3
"""Acceleration repair of a U3 plan before btexport checks it: re-times the junctions where a leg's motion jumps.

    python3 -I u3_retime.py <plan.json> [--robots-yaml <robots.yaml>] [--out <plan.json>]

Why: on the arm64 hosts umarell's venv has no TOPP-RA (no arm64 wheel; deploy/requirements-planner.txt), so
umarell's time_parameterization falls back to ``_hermite_spline``. That spline gives every knot-to-knot segment the
time ``max(|dq|, 0.05) / velocity_scale`` but a Catmull-Rom tangent ``0.5 (q[i+1] - q[i-1])`` in per-SEGMENT units:
where a short segment (a lift-off step: 0.05 rad floor, 0.167 s) meets a long one (the transit after it: ~0.7 rad,
2.3 s), the long segment's tangent is applied to the short one and the spline overshoots BACKWARDS in the short
segment. bt_1792079345_jamijam2, robot1 W04 at t = 1.0833 s of the leg (the last lift-off step out of W02, right
before the transit): a1 and a6 swing -0.0306 rad (-0.24767 -> -0.27830 -> -0.24767) in 2 x 0.083 s, i.e.
0.135 x v_max but 4.76 x a_max, and btexport's limits check fails ("inherited from the source"); robot2 W03 has the
same sample. Every U3 plan with a lift-off before a long transit has it.

What: per leg (blackboard trajectory), the replayed motion (umarell's executor interpolates the waypoints linearly,
holds the first / last waypoint before / after the leg) is checked with btexport's own metric (0.05 s second
difference; here on a 1 kHz grid, so no 200 Hz grid phase can see more). Around the worst sample above ``target``
x a_max the waypoints strictly inside a window [t_a, t_b] are replaced by one cubic per joint that starts and ends
with the replayed position and velocity at t_a and t_b (C1, so the window's ends add no jump), sampled every
<= 0.05 s. The shortest window that brings the whole affected span under the target, keeps every joint as
monotone as it was (no new reversal), stays under v_max and keeps the leg's first / last segment and its weld
approach untouched is used; repeated until the leg passes or no window does. Nothing else changes: leg start /
end times and poses (so the tree's step timing and both arms' schedules), the weld approach, the hold. A window
moves the motion by the overshoot it removes and a few mrad besides (``max_change_rad``: the largest change against
the replayed original, the overshoot included); btexport's dense check (collision, clearance, rule W, welds) then
checks the re-timed plan like any other.

Standard library only (run with the router's python3).
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import re
import sys
from pathlib import Path
from typing import Optional

H = 0.05                 # btexport's finite-difference step (s): dense_dynamics h, limits_check
HZ = 1000.0              # evaluation grid (btexport: 200 Hz; finer, so every grid phase is covered)
STEP = 0.05              # longest step between the new waypoints of a window (s)
QUANTUM = 1e-4           # umarell's time_s rounding (s)
TARGET = 0.95            # re-time a leg where the acceleration exceeds this fraction of a_max
MAX_WINDOW_S = 2.5
MAX_REPAIRS_PER_LEG = 12
KR240_ACC = [3.0, 3.0, 3.0, 6.0, 6.0, 8.0]          # btexport's default (umarell robots.yaml "REVIEW" value)


# ─────────────────────────────────────── the replayed motion and its metric ───────────────────────────────────────
def _q(times: list, col: list, t: float) -> float:
    """One joint of the replayed leg at time t: linear between waypoints, held before / after."""
    if t <= times[0]:
        return col[0]
    if t >= times[-1]:
        return col[-1]
    i = bisect.bisect_right(times, t) - 1
    t0, t1 = times[i], times[i + 1]
    if t1 <= t0:
        return col[i + 1]
    return col[i] + (col[i + 1] - col[i]) * (t - t0) / (t1 - t0)


def acc_ratio(times: list, pos: list, a_max: list, t_lo: Optional[float] = None, t_hi: Optional[float] = None,
              hz: float = HZ, h: float = H) -> tuple:
    """max over [t_lo, t_hi] of |q(t+h) - 2 q(t) + q(t-h)| / h^2 / a_max (btexport's dense acceleration metric).
    Returns (ratio, t, joint index)."""
    lo = times[0] - h if t_lo is None else t_lo
    hi = times[-1] + h if t_hi is None else t_hi
    n = max(1, int(math.ceil((hi - lo) * hz)))
    best = (0.0, lo, 0)
    for j in range(len(pos[0])):
        col = [p[j] for p in pos]
        am = a_max[j] if j < len(a_max) else a_max[-1]
        for k in range(n + 1):
            t = lo + (hi - lo) * k / n
            a = abs(_q(times, col, t + h) - 2.0 * _q(times, col, t) + _q(times, col, t - h)) / (h * h) / am
            if a > best[0]:
                best = (a, t, j)
    return best


def seg_vel_ratio(times: list, pos: list, v_max: Optional[list]) -> float:
    if not v_max:
        return 0.0
    r = 0.0
    for i in range(len(times) - 1):
        dt = times[i + 1] - times[i]
        if dt <= 1e-9:
            continue
        for j in range(len(pos[0])):
            r = max(r, abs(pos[i + 1][j] - pos[i][j]) / dt / v_max[j])
    return r


# ─────────────────────────────────────────────── the window blend ───────────────────────────────────────────────
def _slope(times: list, pos: list, i: int, k: int, j: int) -> float:
    dt = times[k] - times[i]
    return (pos[k][j] - pos[i][j]) / dt if dt > 1e-9 else 0.0


def blend(times: list, pos: list, i0: int, i1: int, step: float = STEP) -> tuple:
    """New (times, positions) of the waypoints strictly between i0 and i1: per joint the cubic Hermite from
    (t[i0], q[i0], v_in) to (t[i1], q[i1], v_out), v_in = the replayed velocity just before t[i0] (0 at the leg's
    start: the arm rests there), v_out = the one just after t[i1] (0 at its end)."""
    ta, tb = times[i0], times[i1]
    T = tb - ta
    m = max(1, int(math.ceil(T / step - 1e-9)))
    dof = len(pos[0])
    va = [_slope(times, pos, i0 - 1, i0, j) if i0 > 0 else 0.0 for j in range(dof)]
    vb = [_slope(times, pos, i1, i1 + 1, j) if i1 < len(times) - 1 else 0.0 for j in range(dof)]
    nt, npos = [], []
    for k in range(1, m):
        t = round(ta + T * k / m, 4)
        if not (ta + QUANTUM / 2 < t < tb - QUANTUM / 2) or (nt and t <= nt[-1]):
            continue
        s = (t - ta) / T
        h00, h10 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s
        h01, h11 = -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
        nt.append(t)
        npos.append([round(h00 * pos[i0][j] + h10 * T * va[j] + h01 * pos[i1][j] + h11 * T * vb[j], 6)
                     for j in range(dof)])
    return nt, npos, va, vb


def _monotone_ok(qa: float, qb: float, va: float, vb: float, seq: list, tol: float = 2e-5) -> bool:
    """A joint whose motion over the window is one-directional (its end-to-end change and both end velocities
    agree in sign; a joint at the same position at both ends that leaves in the direction it came in counts too) must
    stay so: the blend may not add a reversal (the overshoot this repairs is one)."""
    d = qb - qa
    if abs(d) < 1e-6:
        if va * vb < 0 or abs(va) + abs(vb) < 1e-9:
            return True
        d = va + vb
    sgn = 1.0 if d > 0 else -1.0
    if va * sgn < -1e-9 or vb * sgn < -1e-9:
        return True
    return all((b - a) * sgn >= -tol for a, b in zip(seq[:-1], seq[1:]))


def _deviation(times: list, pos: list, nt: list, npos: list) -> float:
    """Largest joint distance of the new waypoints from the replayed original motion at the same instants."""
    dev = 0.0
    for t, q in zip(nt, npos):
        for j in range(len(q)):
            dev = max(dev, abs(q[j] - _q(times, [p[j] for p in pos], t)))
    return dev


def repair_leg(traj: list, a_max: list, v_max: Optional[list] = None, target: float = TARGET,
               max_window_s: float = MAX_WINDOW_S) -> tuple:
    """Re-time the junctions of one leg (a list of {"positions", "time_s", ...}) whose replayed acceleration exceeds
    ``target`` x a_max. Returns (new trajectory, report); the trajectory is ``traj`` itself when nothing changed."""
    pts = [dict(p) for p in traj]
    if len(pts) < 3 or not all(isinstance(p.get("positions"), list) and "time_s" in p for p in pts):
        return traj, {"changed": False, "reason": "not a timed trajectory"}
    times = [float(p["time_s"]) for p in pts]
    if any(b < a for a, b in zip(times[:-1], times[1:])):
        return traj, {"changed": False, "reason": "time_s not monotonic"}
    before = acc_ratio(times, [p["positions"] for p in pts], a_max)
    rep = {"changed": False, "acc_before": round(before[0], 3), "t_before": round(before[1], 4),
           "joint_before": f"a{before[2] + 1}", "windows": []}
    if before[0] <= target:
        rep["acc_after"] = rep["acc_before"]
        return traj, rep
    v_lim = seg_vel_ratio(times, [p["positions"] for p in pts], v_max)
    for _ in range(MAX_REPAIRS_PER_LEG):
        times = [float(p["time_s"]) for p in pts]
        pos = [list(map(float, p["positions"])) for p in pts]
        worst = acc_ratio(times, pos, a_max)
        if worst[0] <= target:
            break
        tv, n = worst[1], len(pts)
        ph_v = pts[min(max(bisect.bisect_right(times, tv) - 1, 0), n - 1)].get("phase")
        cands = []
        for i0 in range(0, n - 1):
            for i1 in range(i0 + 1, n):
                ta, tb = times[i0], times[i1]
                if not (ta <= tv + H and tb >= tv - H) or tb - ta > max_window_s or tb - ta < H:
                    continue
                inner = range(i0 + 1, i1)
                # the leg's rest at its ends (btexport's rest check) and a weld's straight approach stay as they are,
                # unless the jump itself is in there
                protected = (i0 == 0 or i1 == n - 1) + sum(
                    1 for i in inner if pts[i].get("phase") == "approach" and ph_v != "approach")
                cands.append((protected, tb - ta, abs((ta + tb) / 2 - tv), i0, i1))
        cands.sort()
        best = None
        for protected, T, _, i0, i1 in cands:
            if best is not None and best[0] <= target and (protected, T) > best[5]:
                break
            nt, npos, va, vb = blend(times, pos, i0, i1)
            t2 = times[:i0 + 1] + nt + times[i1:]
            p2 = pos[:i0 + 1] + npos + pos[i1:]
            r = acc_ratio(t2, p2, a_max, times[i0] - H - 0.002, times[i1] + H + 0.002)[0]
            if r > target and best is not None and r >= best[0]:
                continue
            if v_max and seg_vel_ratio(t2[i0:i0 + len(nt) + 2], p2[i0:i0 + len(nt) + 2], v_max) > max(1.0, v_lim):
                continue
            if not all(_monotone_ok(pos[i0][j], pos[i1][j], va[j], vb[j],
                                    [pos[i0][j]] + [q[j] for q in npos] + [pos[i1][j]]) for j in range(len(va))):
                continue
            if best is None or r < best[0] or (r <= target and best[0] > target):
                best = (r, i0, i1, nt, npos, (protected, T))
            if r <= target:
                break
        if best is None or best[0] >= worst[0] - 1e-6:
            rep["unrepaired"] = {"acc": round(worst[0], 3), "t": round(tv, 4), "joint": f"a{worst[2] + 1}"}
            break
        r, i0, i1, nt, npos, _ = best
        dev = _deviation(times, pos, nt, npos)
        keep = {k: v for k, v in pts[i0].items() if k not in ("positions", "time_s")}
        new = [dict(keep, positions=q, time_s=t) for t, q in zip(nt, npos)]
        for k, pt in enumerate(new):     # the phase of the original waypoint the new one replaces in time
            j = min(max(bisect.bisect_right(times, pt["time_s"]) - 1, i0), i1 - 1)
            if "phase" in pts[j]:
                pt["phase"] = pts[j]["phase"]
        rep["windows"].append({"t_a": times[i0], "t_b": times[i1], "replaced": i1 - i0 - 1, "inserted": len(new),
                               "acc_window": round(r, 3), "acc_was": round(worst[0], 3), "at": f"a{worst[2] + 1}",
                               "max_change_rad": round(dev, 5)})
        pts = pts[:i0 + 1] + new + pts[i1:]
        rep["changed"] = True
    times = [float(p["time_s"]) for p in pts]
    after = acc_ratio(times, [p["positions"] for p in pts], a_max)
    rep["acc_after"] = round(after[0], 3)
    if not rep["changed"]:
        return traj, rep
    return pts, rep


# ──────────────────────────────────────────────────── a plan ────────────────────────────────────────────────────
def robot_limits(robots_yaml: Optional[Path]) -> dict:
    """{robot ns: {"a_max": [...], "v_max": [...] | None}} from a umarell robots.yaml (``robots: arms: <ns>:`` with
    one-line ``joint_acceleration_limits`` / ``joint_velocity_limits`` lists, as umarell and Prepare Cell write
    them); a robot without accelerations gets btexport's default."""
    out: dict = {}
    if robots_yaml is None or not Path(robots_yaml).is_file():
        return out
    arm_indent, cur = None, None
    for raw in Path(robots_yaml).read_text(errors="replace").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        ind = len(line) - len(line.lstrip())
        key = line.strip()
        if key.startswith("arms:"):
            arm_indent, cur = ind, None
            continue
        if arm_indent is None:
            continue
        if ind <= arm_indent:
            arm_indent, cur = None, None
            continue
        m = re.match(r"([A-Za-z0-9_\-]+):\s*$", key)
        if m and (cur is None or ind <= cur[1]):
            cur = (m.group(1), ind)
            out.setdefault(cur[0], {"a_max": None, "v_max": None})
            continue
        m = re.match(r"(joint_acceleration_limits|joint_velocity_limits):\s*\[([^\]]*)\]", key)
        if m and cur is not None:
            try:
                vals = [float(v) for v in m.group(2).split(",") if v.strip()]
            except ValueError:
                continue
            out[cur[0]]["a_max" if m.group(1).startswith("joint_acc") else "v_max"] = vals
    return out


def _trajectory_refs(node, out: dict) -> None:
    ports = node.get("ports") or {}
    tr, ns = ports.get("trajectory"), ports.get("robot_ns")
    if isinstance(tr, str) and tr.startswith("{") and tr.endswith("}") and ns:
        out.setdefault(tr[1:-1], ns)
    for c in node.get("children") or []:
        if isinstance(c, dict):
            _trajectory_refs(c, out)


def retime_plan(plan: dict, limits: Optional[dict] = None, target: float = TARGET) -> dict:
    """Repair every blackboard trajectory of a umarell plan (in place). Returns {"changed", "legs": {key: report}}."""
    limits = limits or {}
    bt = plan.get("behaviour_tree") if isinstance(plan, dict) else None
    if not isinstance(bt, dict) or not isinstance(bt.get("blackboard"), dict):
        return {"changed": False, "legs": {}, "reason": "no behaviour_tree blackboard"}
    refs: dict = {}
    if isinstance(bt.get("root"), dict):
        _trajectory_refs(bt["root"], refs)
    out = {"changed": False, "legs": {}, "target_frac_amax": target}
    for key, traj in bt["blackboard"].items():
        if not isinstance(traj, list) or not traj or not isinstance(traj[0], dict) or "positions" not in traj[0]:
            continue
        ns = refs.get(key)
        lim = limits.get(ns) or {}
        dof = len(traj[0]["positions"])
        a_max = lim.get("a_max") or KR240_ACC
        a_max = (list(a_max) + [a_max[-1]] * dof)[:dof]
        new, rep = repair_leg(traj, a_max, lim.get("v_max"), target)
        rep["robot"] = ns
        out["legs"][key] = rep
        if rep.get("changed"):
            bt["blackboard"][key] = new
            out["changed"] = True
    return out


def summary_lines(info: dict) -> list:
    lines = []
    for key, r in (info.get("legs") or {}).items():
        if r.get("changed"):
            w = r["windows"]
            lines.append(f"{key}: {len(w)} junction(s) re-timed ("
                         + "; ".join(f"{x['t_a']:.4f}-{x['t_b']:.4f} s, {x['at']} {x['acc_was']} -> {x['acc_window']}"
                                     f" x a_max, moved <= {x['max_change_rad']} rad" for x in w)
                         + f"); leg {r['acc_before']} -> {r['acc_after']} x a_max")
        if r.get("unrepaired"):
            u = r["unrepaired"]
            lines.append(f"{key}: still {u['acc']} x a_max at {u['t']} s ({u['joint']}): no window repairs it")
    return lines


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("plan")
    ap.add_argument("--robots-yaml", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--target", type=float, default=TARGET)
    a = ap.parse_args(argv)
    plan = json.loads(Path(a.plan).read_text())
    info = retime_plan(plan, robot_limits(Path(a.robots_yaml) if a.robots_yaml else None), a.target)
    for line in summary_lines(info):
        print(line)
    if a.out:
        Path(a.out).write_text(json.dumps(plan))
    print(json.dumps({k: v for k, v in info.items() if k != "legs"}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
