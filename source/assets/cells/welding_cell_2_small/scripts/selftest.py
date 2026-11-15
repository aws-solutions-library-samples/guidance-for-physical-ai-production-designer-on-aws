#!/usr/bin/env python3
"""Self-test for trajectory_player: exercises exactly what the rerun-viewer does.

Starts the player on an ephemeral port, then:
  1. fetches every static asset the viewer requests (scene, cell.yaml, URDFs,
     the meshes named by each URDF at the configured LOD);
  2. performs a real RFC6455 handshake against /ws and polls `state`, validating
     the response against live_client.rs's parsing requirements;
  3. checks the playhead advances and the control verbs work.

Stdlib only. Run:  python3 scripts/selftest.py
"""
from __future__ import annotations

import base64
import json
import os
import re
import socket
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
CELL = os.path.dirname(HERE)

PASS, FAIL = "PASS", "FAIL"
results: list[tuple[str, str, str]] = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    results.append((PASS if cond else FAIL, name, detail))
    return cond


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ── websocket client (minimal RFC6455, mirrors tungstenite's client usage) ──

def ws_connect(host: str, port: int, path: str = "/ws") -> socket.socket:
    sock = socket.create_connection((host, port), timeout=10)
    key = base64.b64encode(os.urandom(16)).decode()
    req = (
        f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\n"
        f"Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
    )
    sock.sendall(req.encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            raise RuntimeError("handshake: connection closed")
        buf += chunk
    head = buf.split(b"\r\n\r\n")[0].decode(errors="replace")
    if "101" not in head.split("\r\n")[0]:
        raise RuntimeError(f"handshake failed: {head.splitlines()[0]}")
    return sock


def ws_send_text(sock: socket.socket, text: str) -> None:
    payload = text.encode()
    mask = os.urandom(4)
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    n = len(payload)
    hdr = bytearray([0x81])
    if n < 126:
        hdr.append(0x80 | n)
    elif n < 65536:
        hdr.append(0x80 | 126)
        hdr += struct.pack(">H", n)
    else:
        hdr.append(0x80 | 127)
        hdr += struct.pack(">Q", n)
    sock.sendall(bytes(hdr) + mask + masked)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise RuntimeError("closed mid-frame")
        buf += chunk
    return buf


def ws_recv_text(sock: socket.socket) -> str:
    b0, b1 = _recv_exact(sock, 2)
    length = b1 & 0x7F
    if length == 126:
        length = struct.unpack(">H", _recv_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack(">Q", _recv_exact(sock, 8))[0]
    if b1 & 0x80:                       # server must not mask, but be tolerant
        mask = _recv_exact(sock, 4)
        data = _recv_exact(sock, length)
        return bytes(b ^ mask[i % 4] for i, b in enumerate(data)).decode()
    return _recv_exact(sock, length).decode()


def rpc(sock: socket.socket, req: dict) -> dict:
    ws_send_text(sock, json.dumps(req))
    return json.loads(ws_recv_text(sock))


# ── static asset expectations, derived from the URDFs like the viewer does ──

def urdf_meshes(urdf_path: str, lod: str) -> list[str]:
    """Mesh paths referenced by a URDF, LOD-swapped as the viewer does."""
    with open(urdf_path, encoding="utf-8", errors="replace") as fh:
        xml = fh.read()
    out = []
    for m in re.findall(r'filename="([^"]+)"', xml):
        rel = m.lstrip("./")
        if "hull" in os.path.basename(rel).lower():
            continue                    # collision LOD, not fetched for visual/low
        if lod == "low":
            rel = rel.replace("meshes/", "meshes_lowres/", 1)
        out.append(rel)
    return sorted(set(out))


def main() -> int:
    port = free_port()
    base = f"http://127.0.0.1:{port}"
    proc = subprocess.Popen(
        [sys.executable, os.path.join(HERE, "trajectory_player.py"),
         "--cell", CELL, "--port", str(port), "--loop", "--no-plan"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        # wait for the port to accept
        deadline = time.time() + 20
        while time.time() < deadline:
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                    break
            except OSError:
                if proc.poll() is not None:
                    out, err = proc.communicate()
                    print("player died:\n", out, err)
                    return 1
                time.sleep(0.2)
        else:
            print("player did not start")
            return 1

        # 1. static assets ------------------------------------------------
        def get(path: str) -> tuple[int, bytes]:
            try:
                with urllib.request.urlopen(f"{base}{path}", timeout=15) as r:
                    return r.status, r.read()
            except urllib.error.HTTPError as e:
                return e.code, b""

        for path in ("/env/scene.yaml", "/env/cell.yaml"):
            code, body = get(path)
            check(f"GET {path}", code == 200 and len(body) > 0, f"status={code}")

        # scene_id mapping must be present and complete
        _, cell_yaml = get("/env/cell.yaml")
        txt = cell_yaml.decode()
        check("cell.yaml maps both scene ids",
              "kr240r2900__sticktool" in txt and "kr240r2900__sticktool_2" in txt)

        # the only URDF the scene references (this cell has no fixture)
        urdfs = {
            "robot": "robots/kr240r2900__sticktool/kr240r2900__sticktool.urdf",
        }
        for label, rel in urdfs.items():
            code, body = get(f"/env/{rel}")
            check(f"GET {label} URDF", code == 200 and b"<robot" in body,
                  f"status={code}")

        # every mesh each URDF names, at the extension's default LOD ("low")
        for label, rel in urdfs.items():
            urdf_abs = os.path.join(CELL, rel)
            urdf_dir = os.path.dirname(rel)
            meshes = urdf_meshes(urdf_abs, "low")
            missing = []
            for mesh_rel in meshes:
                code, body = get(f"/env/{urdf_dir}/{mesh_rel}")
                if code != 200 or not body:
                    missing.append(mesh_rel)
            check(f"{label}: all {len(meshes)} low-LOD meshes served",
                  not missing,
                  f"missing {len(missing)}: {missing[:3]}" if missing else "")

        code, body = get("/env/floorplans/hall-a-floor-plan/hall-a-floor-plan.obj")
        check("GET floor plan OBJ", code == 200 and len(body) > 0, f"status={code}")

        check("path traversal refused", get("/env/../cell.yaml")[0] != 200)

        # 2. websocket telemetry ------------------------------------------
        sock = ws_connect("127.0.0.1", port)
        check("WS handshake to /ws", True)

        resp = rpc(sock, {"id": 1, "type": "state"})
        check("state responds ok", resp.get("ok") is True, json.dumps(resp)[:200])
        result = resp.get("result") or {}

        # exactly what live_client.rs::update_state reads
        check("result.running is bool", isinstance(result.get("running"), bool))
        check("result.cycle is int", isinstance(result.get("cycle"), int))
        krl = result.get("krl")
        robots = result.get("robots")
        check("result.krl is a list of 2", isinstance(krl, list) and len(krl) == 2)
        check("result.robots is a list of 2",
              isinstance(robots, list) and len(robots) == 2)

        ids = [k.get("robot_id") for k in krl or []]
        check("krl robot_ids are robot1/robot2", ids == ["robot1", "robot2"], str(ids))
        for k in krl or []:
            j = k.get("joints")
            ok = (isinstance(j, list) and len(j) == 6
                  and all(isinstance(v, (int, float)) for v in j))
            check(f"{k.get('robot_id')}: joints is 6 numbers", ok, str(j)[:80])
        for r in robots or []:
            check(f"{r.get('robot_id')}: phase is a string",
                  isinstance(r.get("phase"), str))

        # 3. legacy cell-runner transport, over a loaded plan ------------
        # The player starts with --no-plan, so load one first; these verbs keep
        # cell-runner's original meaning (start/stop/reset/set_speed).
        first = (rpc(sock, {"id": 14, "type": "trajectory_list"})["result"]
                 ["plans"][0]["path"])
        check("load for legacy transport checks",
              rpc(sock, {"id": 15, "type": "trajectory_load",
                         "params": {"path": first}}).get("ok") is True)

        def traj(i):
            return rpc(sock, {"id": i, "type": "state"})["result"]["trajectory"]

        rpc(sock, {"id": 16, "type": "start"})
        t0 = traj(17)["t"]
        time.sleep(1.2)
        t1 = traj(18)["t"]
        check("playhead advances", t1 > t0, f"{t0} -> {t1}")

        j_a = rpc(sock, {"id": 19, "type": "state"})["result"]["krl"][0]["joints"]
        time.sleep(0.7)
        j_b = rpc(sock, {"id": 191, "type": "state"})["result"]["krl"][0]["joints"]
        check("robot1 joints change over time", j_a != j_b)

        rpc(sock, {"id": 6, "type": "stop"})
        ta = traj(7)["t"]
        time.sleep(0.6)
        tb = traj(8)["t"]
        check("stop freezes the playhead", abs(tb - ta) < 1e-6, f"{ta} -> {tb}")

        rpc(sock, {"id": 9, "type": "reset"})
        check("reset returns to 0", traj(10)["t"] < 1e-6)

        sp = rpc(sock, {"id": 11, "type": "set_speed",
                        "params": {"speed": 4.0}})["result"]["speed"]
        check("set_speed applies", abs(sp - 4.0) < 1e-9, str(sp))

        seek = rpc(sock, {"id": 12, "type": "trajectory_seek",
                          "params": {"t": 46.0}})
        check("seek positions the playhead",
              abs(seek["result"]["t"] - 46.0) < 0.2, str(seek["result"]["t"]))

        bad = rpc(sock, {"id": 13, "type": "nonsense"})
        check("unknown request type is refused", bad.get("ok") is False)

        # 4. trajectory_* motion-source protocol ---------------------------
        listing = rpc(sock, {"id": 20, "type": "trajectory_list"})
        plans = (listing.get("result") or {}).get("plans") or []
        check("trajectory_list finds a plan", len(plans) >= 1, str(listing)[:160])
        check("listed plan carries a tree_id",
              bool(plans and plans[0].get("tree_id")))

        rpc(sock, {"id": 21, "type": "trajectory_unload"})
        st = rpc(sock, {"id": 22, "type": "trajectory_state"})["result"]
        check("unload reports nothing loaded", st.get("loaded") is False)

        # With nothing loaded the viewer must still get 6 joints per robot.
        krl_idle = rpc(sock, {"id": 23, "type": "state"})["result"]["krl"]
        check("state still valid with no plan loaded",
              len(krl_idle) == 2 and all(len(k["joints"]) == 6 for k in krl_idle))

        # play with nothing loaded is an error, not a crash
        nope = rpc(sock, {"id": 24, "type": "trajectory_play"})
        check("play with no plan is refused", nope.get("ok") is False)
        check("connection survives a refused request",
              rpc(sock, {"id": 25, "type": "trajectory_state"}).get("ok") is True)

        loaded = rpc(sock, {"id": 26, "type": "trajectory_load",
                            "params": {"path": plans[0]["path"]}})
        res = loaded.get("result") or {}
        check("trajectory_load succeeds", loaded.get("ok") is True, str(loaded)[:160])
        check("load reports duration + segments",
              res.get("duration_s", 0) > 0 and res.get("segments", 0) > 0, str(res)[:160])
        check("load reports both robots", res.get("robots") == ["robot1", "robot2"],
              str(res.get("robots")))
        check("load reports no schedule warnings", res.get("warnings") == [],
              str(res.get("warnings")))
        check("load reports a collision block", isinstance(res.get("collision"), dict))

        check("traversal is refused on load",
              rpc(sock, {"id": 27, "type": "trajectory_load",
                         "params": {"path": "../../../etc/passwd"}}).get("ok") is False)

        # start / stop, the transport the panel drives
        rpc(sock, {"id": 28, "type": "trajectory_stop"})
        s0 = rpc(sock, {"id": 29, "type": "trajectory_state"})["result"]
        check("stop rewinds and pauses",
              s0["playing"] is False and s0["t"] < 1e-6, str(s0)[:120])

        rpc(sock, {"id": 30, "type": "trajectory_play"})
        time.sleep(1.0)
        s1 = rpc(sock, {"id": 31, "type": "trajectory_state"})["result"]
        check("play advances the playhead", s1["playing"] is True and s1["t"] > 0,
              str(s1)[:120])
        check("state reports per-robot op + phase",
              len(s1.get("robots", [])) == 2
              and all("phase" in r and "op_id" in r for r in s1["robots"]),
              str(s1.get("robots"))[:160])

        rpc(sock, {"id": 32, "type": "trajectory_pause"})
        p0 = rpc(sock, {"id": 33, "type": "trajectory_state"})["result"]["t"]
        time.sleep(0.5)
        p1 = rpc(sock, {"id": 34, "type": "trajectory_state"})["result"]["t"]
        check("pause holds the playhead", abs(p1 - p0) < 1e-6, f"{p0} -> {p1}")

        lp = rpc(sock, {"id": 35, "type": "trajectory_set_loop",
                        "params": {"loop": True}})["result"]
        check("set_loop applies", lp.get("loop") is True)
        sp = rpc(sock, {"id": 36, "type": "trajectory_set_speed",
                        "params": {"speed": 3.0}})["result"]
        check("trajectory_set_speed applies", abs(sp.get("speed", 0) - 3.0) < 1e-9)

        # the 3D viewer's own view of playback must move too
        rpc(sock, {"id": 37, "type": "trajectory_play"})
        ja = rpc(sock, {"id": 38, "type": "state"})["result"]["krl"][0]["joints"]
        time.sleep(0.6)
        jb = rpc(sock, {"id": 39, "type": "state"})["result"]["krl"][0]["joints"]
        check("state joints track playback", ja != jb)
        check("state embeds the trajectory block",
              isinstance(rpc(sock, {"id": 40, "type": "state"})["result"]
                         .get("trajectory"), dict))

        # HTTP RPC alternative
        req = json.dumps({"id": 99, "type": "state"}).encode()
        r = urllib.request.Request(f"{base}/rpc", data=req,
                                   headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(r, timeout=10) as resp2:
            check("HTTP /rpc mirrors the WS protocol",
                  json.loads(resp2.read()).get("ok") is True)

        sock.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()

    width = max(len(n) for _, n, _ in results)
    for status, name, detail in results:
        line = f"  [{status}] {name.ljust(width)}"
        if detail and status == FAIL:
            line += f"  {detail}"
        print(line)
    failed = sum(1 for s, _, _ in results if s == FAIL)
    print(f"\n{len(results) - failed}/{len(results)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
