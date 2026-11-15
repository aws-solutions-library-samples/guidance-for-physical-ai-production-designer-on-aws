#!/usr/bin/env python3
"""Diagnose 'status says paused but the robot is moving'.

Samples the live backend without changing anything: reads `trajectory_state` and
the `state` telemetry the 3D viewer renders, and reports whether the reported
`playing` flag agrees with whether the joints are actually changing.
"""
from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from selftest import rpc, ws_connect

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 9902
SAMPLES = 8
INTERVAL = 0.4

sock = ws_connect("127.0.0.1", PORT)
counter = [0]


def call(verb, params=None):
    counter[0] += 1
    req = {"id": counter[0], "type": verb}
    if params is not None:
        req["params"] = params
    return rpc(sock, req)


ts = call("trajectory_state").get("result") or {}
print("trajectory_state:")
print(json.dumps({k: v for k, v in ts.items() if k != "robots"}, indent=2))
print()

print(f"{'#':>2}  {'traj.playing':>12} {'traj.t':>8} {'state.running':>13}  robot1 j0..j2")
rows = []
for i in range(SAMPLES):
    t_state = call("trajectory_state").get("result") or {}
    s_state = call("state").get("result") or {}
    krl = (s_state.get("krl") or [{}])[0]
    joints = krl.get("joints") or []
    rows.append((t_state.get("playing"), t_state.get("t"), s_state.get("running"),
                 tuple(round(v, 5) for v in joints[:6])))
    print(f"{i:>2}  {str(t_state.get('playing')):>12} {t_state.get('t'):>8} "
          f"{str(s_state.get('running')):>13}  "
          + " ".join(f"{v:+.4f}" for v in joints[:3]))
    time.sleep(INTERVAL)

print()
playing_flags = {r[0] for r in rows}
t_moved = rows[-1][1] != rows[0][1]
joints_moved = len({r[3] for r in rows}) > 1

print(f"reported playing : {playing_flags}")
print(f"playhead moved   : {t_moved}  ({rows[0][1]} -> {rows[-1][1]})")
print(f"joints changed   : {joints_moved}")
print()
if playing_flags == {False} and joints_moved:
    print("=> CONTRADICTION: reported paused, but the telemetry joints are changing.")
elif playing_flags == {False} and t_moved:
    print("=> CONTRADICTION: reported paused, but the playhead is advancing.")
elif playing_flags == {False} and not joints_moved:
    print("=> CONSISTENT: paused and joints are frozen — any visible motion is")
    print("   coming from the viewer, not the backend telemetry.")
else:
    print("=> playing; joints changing as expected.")

sock.close()
