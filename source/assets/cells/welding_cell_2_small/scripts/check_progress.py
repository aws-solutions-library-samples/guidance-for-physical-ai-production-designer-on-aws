#!/usr/bin/env python3
"""Check what the Trajectory Controller's PROGRESS column actually reports.

Loads a plan, samples `trajectory_state` while paused and then while playing, and
prints exactly the values the panel renders per robot.

Usage:  python3 scripts/check_progress.py [port]
"""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from selftest import rpc, ws_connect

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 9902
sock = ws_connect("127.0.0.1", PORT)
n = [0]


def call(verb, params=None):
    n[0] += 1
    req = {"id": n[0], "type": verb}
    if params is not None:
        req["params"] = params
    return rpc(sock, req)


def row(state):
    """Render the panel's Playback + Robots values from one state payload."""
    t = state.get("t") or 0.0
    dur = state.get("duration") or 0.0
    bar = (t / dur * 100) if dur else 0.0
    cells = []
    for r in state.get("robots") or []:
        pct = round((r.get("progress") or 0) * 100)
        cells.append(f"{r['robot_id']}: {str(r.get('phase')):<6} "
                     f"{str(r.get('op_id') or '-'):<5} {pct:>3}%")
    return f"t={t:6.2f}  bar={bar:5.1f}%  " + " | ".join(cells)


call("trajectory_unload")
plans = (call("trajectory_list")["result"]["plans"])
call("trajectory_load", {"path": plans[0]["path"]})
call("trajectory_stop")

print("=== PAUSED at t=0 (panel status: 'ready') ===")
for _ in range(3):
    print("  " + row(call("trajectory_state")["result"]))
    time.sleep(0.3)

print()
print("=== PLAYING ===")
call("trajectory_play")
for _ in range(24):
    print("  " + row(call("trajectory_state")["result"]))
    time.sleep(0.5)

call("trajectory_pause")
call("trajectory_stop")
call("trajectory_unload")
sock.close()
