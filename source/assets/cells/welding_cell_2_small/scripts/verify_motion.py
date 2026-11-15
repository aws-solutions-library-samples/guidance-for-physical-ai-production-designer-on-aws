#!/usr/bin/env python3
"""Verify the backend actually moves the robots, and moves them CORRECTLY.

Drives a running cell-runner: rewind, play, then sample the telemetry the 3D
viewer renders and compare each sample against the pose the compiled schedule
says the robot should be in at that instant. Leaves playback paused and rewound.

"Joints changed" only proves something moved. Comparing against the schedule
proves it moved along the planned trajectory.

Usage:  python3 scripts/verify_motion.py [port] [seconds]
"""
from __future__ import annotations

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CELL = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from selftest import rpc, ws_connect
from trajectory_player import Schedule, load_plan, max_joint_error, robot_ids_from_cell_yaml

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 9902
WATCH_S = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
INTERVAL = 0.5
# The backend samples on its own 10 ms cycle and we round-trip over a socket, so
# our observation time and its playhead differ slightly. Tolerance is generous
# enough to absorb that but far tighter than any real divergence.
TOL_RAD = 0.15

sock = ws_connect("127.0.0.1", PORT)
counter = [0]


def call(verb, params=None):
    counter[0] += 1
    req = {"id": counter[0], "type": verb}
    if params is not None:
        req["params"] = params
    return rpc(sock, req)


def die(msg):
    print(f"FAIL: {msg}")
    sock.close()
    sys.exit(1)


# ── make sure a plan is loaded, and learn which one ──────────────────────────
state = call("trajectory_state").get("result") or {}
if not state.get("loaded"):
    plans = (call("trajectory_list").get("result") or {}).get("plans") or []
    if not plans:
        die("no plan loaded and none found in trajectories/")
    loaded = call("trajectory_load", {"path": plans[0]["path"]})
    if not loaded.get("ok"):
        die(f"trajectory_load failed: {loaded}")
    state = call("trajectory_state").get("result") or {}
    print(f"loaded {plans[0]['path']}")

plan_rel = state.get("plan")
print(f"plan     : {plan_rel}  (tree {state.get('tree_id')})")
print(f"duration : {state.get('duration')} s")

# ── build the same schedule locally, to predict the expected pose ────────────
bt, _meta = load_plan(os.path.join(CELL, plan_rel))
schedule = Schedule(robot_ids_from_cell_yaml(CELL))
schedule.build(bt["root"])
print(f"local schedule: {len(schedule.segments)} segments, {schedule.duration:.2f} s")
if abs(schedule.duration - (state.get("duration") or 0)) > 0.02:
    die(f"backend duration {state.get('duration')} != local {schedule.duration:.2f}")
print("backend and local schedules agree on duration\n")

# ── rewind, then play ────────────────────────────────────────────────────────
call("trajectory_set_speed", {"speed": 1.0})
call("trajectory_set_loop", {"loop": False})
call("trajectory_stop")
at_zero = call("trajectory_state").get("result") or {}
if at_zero.get("t", 1) > 1e-6 or at_zero.get("playing"):
    die(f"stop did not rewind/pause: {at_zero}")
print("rewound to 0 and paused")

call("trajectory_play")
print("playing — sampling the telemetry the 3D viewer renders\n")

hdr = f"{'t (s)':>7}  {'phase':<8} {'op':<6}  {'max err vs plan':>15}  robot1 j0 j1 j2"
print(hdr)
print("-" * len(hdr))

deadline = time.time() + WATCH_S
worst = 0.0
samples = 0
moved = set()
while time.time() < deadline:
    ts = call("trajectory_state").get("result") or {}
    st = call("state").get("result") or {}
    if not ts.get("playing"):
        print("(playback stopped)")
        break
    t = ts.get("t") or 0.0
    krl = {e["robot_id"]: e["joints"] for e in (st.get("krl") or [])}
    rows = {r["robot_id"]: r for r in (ts.get("robots") or [])}

    # Compare every robot's reported joints against the locally predicted pose.
    err = 0.0
    for robot, actual in krl.items():
        expected = schedule._pose_at(robot, t)
        err = max(err, max_joint_error(expected, actual))
        moved.add(tuple(round(v, 6) for v in actual))
    worst = max(worst, err)
    samples += 1

    r1 = rows.get("robot1", {})
    j = krl.get("robot1", [0] * 6)
    print(f"{t:>7.2f}  {str(r1.get('phase')):<8} {str(r1.get('op_id')):<6}  "
          f"{err:>15.5f}  " + " ".join(f"{v:+.4f}" for v in j[:3]))
    time.sleep(INTERVAL)

call("trajectory_pause")
call("trajectory_stop")

print()
print(f"samples                     : {samples}")
print(f"distinct joint vectors seen  : {len(moved)}")
print(f"worst deviation from the plan: {worst:.5f} rad (tolerance {TOL_RAD})")
print()
ok = True
if samples < 3:
    print("FAIL: too few samples to judge"); ok = False
if len(moved) < 3:
    print("FAIL: joints did not change — the robots are not moving"); ok = False
else:
    print("PASS: joints changed over time — the robots are moving")
if worst > TOL_RAD:
    print(f"FAIL: telemetry diverges from the planned trajectory by {worst:.5f} rad"); ok = False
else:
    print("PASS: telemetry tracks the planned trajectory within tolerance")
print()
print("left paused and rewound to 0")
sock.close()
sys.exit(0 if ok else 1)
