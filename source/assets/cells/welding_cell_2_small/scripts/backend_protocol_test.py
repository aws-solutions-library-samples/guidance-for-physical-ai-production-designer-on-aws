#!/usr/bin/env python3
"""Exercise the `trajectory_*` protocol against a running cell-runner.

Unlike `selftest.py` (which drives the stdlib Python player), this points at the
real Rust backend, so it verifies the shipped implementation: the schedule
compiled in `trajectory.rs`, the joints overlay in `server.rs`, and the collision
report produced at load time.

Usage:
  # start the backend first, e.g.
  #   cell-runner --server --cell <cell> --env <cell> --scene scene.yaml --port 9979
  python3 scripts/backend_protocol_test.py [port]
"""
from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from selftest import rpc, ws_connect          # reuse the RFC6455 client

CELL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 9902

results: list[tuple[str, str, str]] = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    results.append(("PASS" if cond else "FAIL", name, detail))
    return bool(cond)


def main() -> int:
    sock = ws_connect("127.0.0.1", PORT)
    check("WS handshake to the cell-runner", True)

    counter = [0]

    def call(verb: str, params: dict | None = None) -> dict:
        counter[0] += 1
        req: dict = {"id": counter[0], "type": verb}
        if params is not None:
            req["params"] = params
        return rpc(sock, req)

    # Establish our own precondition rather than assuming the backend is idle:
    # anything that ran earlier (verify_motion.py, or a Trajectory Controller
    # session) may have left a plan loaded, which would make the "KRL mode"
    # baseline below a trajectory pose instead.
    call("trajectory_unload")

    # ── baseline: the KRL motion source ───────────────────────────────────
    st = call("trajectory_state")
    check("trajectory_state works with nothing loaded",
          st.get("ok") is True and st["result"]["loaded"] is False,
          json.dumps(st)[:200])

    base = call("state")
    check("state responds in KRL mode", base.get("ok") is True)
    # The KRL-mode joints, captured BEFORE any load, are what `unload` must
    # restore. (Read after loading they would be the trajectory's t=0 pose.)
    krl_baseline = (base["result"].get("krl") or [{}])[0].get("joints")
    krl = base["result"].get("krl") or []
    check("state carries 2 robots with 6 joints",
          len(krl) == 2 and all(len(k.get("joints", [])) == 6 for k in krl),
          json.dumps(krl)[:200] if krl else "no krl array")
    check("state embeds an unloaded trajectory block",
          base["result"].get("trajectory", {}).get("loaded") is False)

    listing = call("trajectory_list")
    plans = (listing.get("result") or {}).get("plans") or []
    check("trajectory_list finds a plan", len(plans) >= 1, json.dumps(listing)[:200])
    check("listed plan carries tree_id + operations",
          bool(plans and plans[0].get("tree_id") and plans[0].get("operations")),
          json.dumps(plans[:1])[:200])

    # ── refusals ──────────────────────────────────────────────────────────
    check("play before load is refused", call("trajectory_play").get("ok") is False)
    check("traversal is refused",
          call("trajectory_load", {"path": "../../../etc/passwd"}).get("ok") is False)
    check("missing plan is refused",
          call("trajectory_load", {"path": "trajectories/nope.json"}).get("ok") is False)

    # ── load ──────────────────────────────────────────────────────────────
    loaded = call("trajectory_load", {"path": plans[0]["path"]})
    check("trajectory_load succeeds", loaded.get("ok") is True, json.dumps(loaded)[:300])
    res = loaded.get("result") or {}
    check("load reports 17 segments / 15 operations",
          res.get("segments") == 17 and res.get("operations") == 15,
          f"segments={res.get('segments')} operations={res.get('operations')}")
    check("load reports the 91.97 s duration",
          abs((res.get("duration_s") or 0) - 91.97) < 0.02, str(res.get("duration_s")))
    check("load reports both robots", res.get("robots") == ["robot1", "robot2"],
          str(res.get("robots")))
    check("load has no schedule warnings", res.get("warnings") == [],
          str(res.get("warnings")))

    collision = res.get("collision") or {}
    check("collision check ran", collision.get("checked") is True,
          json.dumps(collision)[:300])
    check("collision result carries a verdict", "in_collision" in collision,
          json.dumps(collision)[:300])
    print("\n  collision: " + json.dumps(collision)[:400] + "\n")

    # ── start / stop ──────────────────────────────────────────────────────
    call("trajectory_stop")
    s0 = call("trajectory_state")["result"]
    check("stop leaves it paused at 0",
          s0["playing"] is False and s0["t"] < 1e-6, json.dumps(s0)[:160])

    j_home = call("state")["result"]["krl"][0]["joints"]
    call("trajectory_play")
    time.sleep(1.5)
    s1 = call("trajectory_state")["result"]
    check("play advances the playhead",
          s1["playing"] and s1["t"] > 0.5, json.dumps(s1)[:200])
    check("state reports per-robot phase",
          len(s1["robots"]) == 2 and all(r.get("phase") for r in s1["robots"]),
          json.dumps(s1["robots"])[:250])

    j_moving = call("state")["result"]["krl"][0]["joints"]
    check("trajectory joints overlay the telemetry the 3D viewer renders",
          j_home != j_moving, f"{j_home} -> {j_moving}")
    check("state.running reflects playback",
          call("state")["result"]["running"] is True)

    call("trajectory_pause")
    p0 = call("trajectory_state")["result"]["t"]
    time.sleep(0.6)
    p1 = call("trajectory_state")["result"]["t"]
    check("pause holds the playhead", abs(p1 - p0) < 1e-6, f"{p0} -> {p1}")

    check("set_loop applies",
          call("trajectory_set_loop", {"loop": True})["result"]["loop"] is True)
    check("set_speed applies",
          abs(call("trajectory_set_speed", {"speed": 3.0})["result"]["speed"] - 3.0) < 1e-9)
    check("seek positions the playhead",
          abs(call("trajectory_seek", {"t": 46.0})["result"]["t"] - 46.0) < 0.3)

    # ── unload restores KRL ───────────────────────────────────────────────
    un = call("trajectory_unload")
    check("unload succeeds",
          un.get("ok") is True and un["result"]["was_loaded"] is True)
    after = call("state")["result"]
    check("unload restores the KRL telemetry untouched",
          after["krl"][0]["joints"] == krl_baseline,
          f"{after['krl'][0]['joints']} vs {krl_baseline}")
    check("unload reports nothing loaded",
          after.get("trajectory", {}).get("loaded") is False)

    # ── inline plan (nothing on disk) ─────────────────────────────────────
    plan_path = os.path.join(CELL, "trajectories", plans[0]["name"])
    with open(plan_path, encoding="utf-8") as fh:
        doc = json.load(fh)
    inline = call("trajectory_load", {"plan": doc})
    check("an inline plan loads", inline.get("ok") is True, json.dumps(inline)[:200])
    check("an inline plan has no file path",
          (inline.get("result") or {}).get("plan") is None)
    call("trajectory_unload")

    sock.close()

    width = max(len(name) for _, name, _ in results)
    for status, name, detail in results:
        line = f"  [{status}] {name.ljust(width)}"
        if detail and status == "FAIL":
            line += f"  {detail}"
        print(line)
    failed = sum(1 for s, _, _ in results if s == "FAIL")
    print(f"\n{len(results) - failed}/{len(results)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
