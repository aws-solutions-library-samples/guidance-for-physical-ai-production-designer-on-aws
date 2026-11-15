#!/usr/bin/env python3
"""
trajectory_player — replay umarell BehaviourTree plans as cell telemetry.

An alternative motion source to cell-runner's KRL interpreter. Where cell-runner
derives joint angles by interpreting KRL programs (PTP/LIN Cartesian targets ->
IK -> OTG), this serves the ALREADY-PLANNED, collision-free joint trajectories
produced by umarell's trajectory agent.

It is a drop-in for cell-runner from the rerun-viewer's point of view, speaking
the same two interfaces on one port:

  GET  /env/<path>   static assets (scene.yaml, cell.yaml, URDFs, meshes)
  WS   /ws           NDJSON request/response telemetry protocol

so a client of the cell-runner works unchanged against it: it polls
{"type":"state"} and reads result.krl[i].joints (6 pre-solved joint angles,
radians). (The retired Kiro extension's rerun-viewer was such a client.)

Stdlib only — no pip install required.

Usage
-----
  python3 scripts/trajectory_player.py                     # cell dir = parent of scripts/
  python3 scripts/trajectory_player.py --loop --speed 2.0
  python3 scripts/trajectory_player.py --plan trajectories/other.json --port 9902
  python3 scripts/trajectory_player.py --dump-schedule      # print timeline, don't serve
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import ipaddress
import json
import math
import os
import posixpath
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional


def loopback_origin(origin: str) -> bool:
    """Whether an Origin header (http[s]://host[:port]) names this machine: localhost, 127.0.0.0/8 or [::1]."""
    for scheme in ("http://", "https://"):
        if origin.startswith(scheme):
            rest = origin[len(scheme):]
            break
    else:
        return False
    if rest.startswith("["):
        host, sep, port = rest[1:].partition("]")
        if not sep:
            return False
    else:
        host, _, tail = rest.partition(":")
        port = ":" + tail if ":" in rest else ""
    if port and not (port.startswith(":") and 1 < len(port) <= 6 and port[1:].isascii() and port[1:].isdigit()):
        return False
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False

# ── Execution constants (mirror umarell agents/cell/bt_executor.py ExecConfig) ──
HOME_TIME_S = 2.5           # interlock pre-move duration
COMPLETION_MARGIN_S = 1.0   # settle slack after a trajectory ends
DEFAULT_INTERLOCK_TOL = 0.15
HOME_JOINTS = [0.0, -1.57, 2.2, 0.0, -0.8, 0.0]

WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

_TWO_PI = 2.0 * math.pi


def angular_diff(x: float, y: float) -> float:
    """Smallest absolute difference between two joint angles, treating values a
    full turn apart as the same physical configuration (umarell _angular_diff)."""
    d = abs(float(x) - float(y)) % _TWO_PI
    return min(d, _TWO_PI - d)


def max_joint_error(a, b) -> float:
    n = min(len(a), len(b))
    return max((angular_diff(a[i], b[i]) for i in range(n)), default=0.0)


def interpolate(traj: list[dict], t: float) -> list[float]:
    """Joint configuration at trajectory-local time ``t`` (linear, clamped).

    Matches umarell's interpolate_trajectory: before the first waypoint returns
    the first pose, after the last returns the last (so the arm holds its final
    pose until the next segment begins).
    """
    if not traj:
        return list(HOME_JOINTS)
    if t <= float(traj[0]["time_s"]):
        return [float(v) for v in traj[0]["positions"]]
    if t >= float(traj[-1]["time_s"]):
        return [float(v) for v in traj[-1]["positions"]]
    for k in range(len(traj) - 1):
        t0 = float(traj[k]["time_s"])
        t1 = float(traj[k + 1]["time_s"])
        if t0 <= t <= t1:
            a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            p0, p1 = traj[k]["positions"], traj[k + 1]["positions"]
            n = min(len(p0), len(p1))
            return [float(p0[i]) + a * (float(p1[i]) - float(p0[i])) for i in range(n)]
    return [float(v) for v in traj[-1]["positions"]]


# ────────────────────────────── schedule model ──────────────────────────────

@dataclass
class Segment:
    """One robot motion occupying an absolute wall-clock window."""
    robot: str
    t0: float
    traj: list[dict]          # time_s relative to t0, velocity-scaled
    op_id: str
    op_type: str
    hold_s: float = 0.0
    node_id: str = ""

    @property
    def duration(self) -> float:
        return float(self.traj[-1]["time_s"]) if self.traj else 0.0

    @property
    def t_end(self) -> float:
        return self.t0 + self.duration

    @property
    def t_done(self) -> float:
        return self.t_end + self.hold_s


class Schedule:
    """Absolute timeline built from a BehaviourTree.

    The plan's trajectories already carry TOPP-RA intra-node timing, and the
    planner resolved inter-robot conflicts into explicit INTERLOCK_WAIT gates.
    So instead of re-running the live gate loop, we resolve each gate ONCE at
    build time against the depended-on robot's already-scheduled motion. The
    result is a deterministic, scrubbable timeline.
    """

    def __init__(self, robots: list[str]):
        self.robots = list(robots)
        self.segments: list[Segment] = []
        self.by_robot: dict[str, list[Segment]] = {r: [] for r in robots}
        self._last_q: dict[str, list[float]] = {r: list(HOME_JOINTS) for r in robots}
        # Per-robot pose BEFORE that robot's first segment. Filled in by
        # _adopt_preroll_poses() from the plan itself; HOME_JOINTS is only the
        # fallback for a robot the plan never moves.
        self.home: dict[str, list[float]] = {}
        self.warnings: list[str] = []

    def home_of(self, robot: str) -> list[float]:
        return list(self.home.get(robot, HOME_JOINTS))

    # -- construction ------------------------------------------------------
    def _emit(self, seg: Segment) -> None:
        self.segments.append(seg)
        self.by_robot.setdefault(seg.robot, []).append(seg)
        if seg.traj:
            self._last_q[seg.robot] = [float(v) for v in seg.traj[-1]["positions"]]

    def build(self, root: dict) -> float:
        end = self._schedule(root, 0.0)
        for segs in self.by_robot.values():
            segs.sort(key=lambda s: s.t0)
        self.duration = end
        self._adopt_preroll_poses()
        return end

    def _adopt_preroll_poses(self) -> None:
        """Park each robot, before its first segment, in the configuration that
        segment STARTS from — not in a generic home pose.

        Robots rarely all start moving at t=0 (an INTERLOCK_WAIT commonly holds
        one arm back), and a plan's trajectories begin from whatever pose the
        planner homed the arms to, which is not necessarily this cell's home.
        Using a generic home for the gap made a waiting arm sit in one pose and
        then TELEPORT to the plan's starting pose the instant its first segment
        began. The generic home still applies to a robot the plan never moves.
        """
        for robot, segs in self.by_robot.items():
            if segs and segs[0].traj:
                self.home[robot] = [float(v) for v in segs[0].traj[0]["positions"]]

    def _schedule(self, node: dict, t0: float) -> float:
        ntype = (node.get("type") or "").upper()
        handler = {
            "SEQUENCE": self._seq,
            "PARALLEL": self._par,
            "EXECUTE": self._execute,
            "INTERLOCK_WAIT": self._interlock,
            "HOME": self._home,
            "SAFETY_GATE": lambda n, t: t,   # no-op for playback
        }.get(ntype)
        if handler is None:
            self.warnings.append(f"unknown node type {ntype!r} (id={node.get('id')}) — skipped")
            return t0
        return handler(node, t0)

    def _seq(self, node: dict, t0: float) -> float:
        t = t0
        for child in node.get("children") or []:
            t = self._schedule(child, t)
        return t

    def _par(self, node: dict, t0: float) -> float:
        children = node.get("children") or []
        if not children:
            return t0
        # Scheduled in declaration order so an INTERLOCK_WAIT in a later branch
        # can resolve against an earlier branch's already-placed motion.
        return max(self._schedule(c, t0) for c in children)

    def _execute(self, node: dict, t0: float) -> float:
        robot = node.get("robot_ns")
        traj = node.get("precomputed_trajectory") or []
        op_id = node.get("operation_id") or "op"
        if not traj:
            self.warnings.append(f"EXECUTE {op_id} has no trajectory — skipped")
            return t0

        t = t0 + float(node.get("start_delay_s", 0.0) or 0.0)

        # Optional interlock pre-move to a safe intermediate configuration.
        itgt = node.get("interlock_target") or {}
        if node.get("has_interlock") and itgt.get("joint_positions"):
            il = [float(v) for v in itgt["joint_positions"]]
            self._emit(Segment(
                robot=robot, t0=t,
                traj=[{"positions": list(self._last_q.get(robot, HOME_JOINTS)), "time_s": 0.0},
                      {"positions": il, "time_s": HOME_TIME_S}],
                op_id=f"{op_id}:interlock", op_type="move",
                node_id=node.get("id", ""),
            ))
            t += HOME_TIME_S

        vscale = float(node.get("velocity_scale", 1.0) or 1.0)
        scaled = [{"positions": [float(v) for v in w["positions"]],
                   "time_s": float(w["time_s"]) / vscale} for w in traj]
        hold_s = float(node.get("hold_duration_s", 0.0) or 0.0)
        self._emit(Segment(
            robot=robot, t0=t, traj=scaled, op_id=op_id,
            op_type=node.get("operation_type", "weld"),
            hold_s=hold_s, node_id=node.get("id", ""),
        ))
        return t + scaled[-1]["time_s"] + hold_s + COMPLETION_MARGIN_S

    def _home(self, node: dict, t0: float) -> float:
        robot = node.get("robot_ns")
        traj = node.get("precomputed_trajectory") or []
        if not traj:
            traj = [{"positions": list(self._last_q.get(robot, HOME_JOINTS)), "time_s": 0.0},
                    {"positions": list(HOME_JOINTS), "time_s": 7.0}]
        scaled = [{"positions": [float(v) for v in w["positions"]],
                   "time_s": float(w["time_s"])} for w in traj]
        self._emit(Segment(robot=robot, t0=t0, traj=scaled, op_id="home",
                           op_type="move", node_id=node.get("id", "")))
        return t0 + scaled[-1]["time_s"] + COMPLETION_MARGIN_S

    def _interlock(self, node: dict, t0: float) -> float:
        """Resolve a joint-space gate to the absolute time it opens."""
        t = t0
        for dep in node.get("depends_on_joints") or []:
            rid = dep.get("robot")
            target = dep.get("joint_positions")
            tol = float(dep.get("tolerance_rad", DEFAULT_INTERLOCK_TOL))
            if not rid or not target:
                continue
            opens = self._gate_time(rid, [float(v) for v in target], tol, t0)
            if opens is None:
                # No scheduled motion reaches the target: fall back to the
                # depended-on robot finishing everything placed so far.
                segs = self.by_robot.get(rid) or []
                opens = max((s.t_done for s in segs), default=t0)
                self.warnings.append(
                    f"INTERLOCK_WAIT {node.get('id')}: {rid} never reaches the awaited "
                    f"configuration within {tol} rad — gated on its last motion instead"
                )
            t = max(t, opens)
        # Cartesian gates need FK, which playback does not synthesize; they are
        # reported rather than silently ignored.
        for dep in node.get("depends_on_positions") or []:
            self.warnings.append(
                f"INTERLOCK_WAIT {node.get('id')}: Cartesian gate on "
                f"{dep.get('robot')} ignored (no FK in playback)"
            )
        return t

    def _pose_at(self, robot: str, t: float) -> list[float]:
        """``robot``'s joint configuration at absolute time ``t``.

        Between segments the arm holds the previous segment's final pose, which
        is what `interpolate`'s clamping gives us. Before its first segment it
        sits at HOME.
        """
        active: Optional[Segment] = None
        for seg in self.by_robot.get(robot) or []:      # kept sorted by t0
            if seg.t0 <= t:
                active = seg
            else:
                break
        if active is None:
            return self.home_of(robot)
        return interpolate(active.traj, t - active.t0)

    def _gate_time(self, robot: str, target: list[float], tol: float,
                   not_before: float) -> Optional[float]:
        """Earliest time >= ``not_before`` at which ``robot`` is within ``tol``
        of ``target``.

        Mirrors umarell's live `_wait_joints`, which starts polling when the
        INTERLOCK_WAIT node runs and returns on the first sample within
        tolerance. Two cases matter:

          * the arm is ALREADY holding a matching pose when the gate opens
            (it finished the awaited motion earlier and has not moved since) —
            caught by testing ``not_before`` itself;
          * the arm passes through the pose later — caught by testing every
            scheduled waypoint, which is dense enough to bound the error.
        """
        segs = sorted(self.by_robot.get(robot) or [], key=lambda s: s.t0)
        # Keep by_robot sorted for _pose_at, which relies on ascending t0.
        self.by_robot[robot] = segs

        if max_joint_error(target, self._pose_at(robot, not_before)) <= tol:
            return not_before

        candidates: list[float] = []
        for seg in segs:
            if seg.t_done < not_before:
                continue
            for w in seg.traj:
                t_abs = seg.t0 + float(w["time_s"])
                if t_abs > not_before:
                    candidates.append(t_abs)
            if seg.t_end > not_before:
                candidates.append(seg.t_end)     # start of the hold window

        for t_abs in sorted(candidates):
            if max_joint_error(target, self._pose_at(robot, t_abs)) <= tol:
                return t_abs
        return None

    # -- sampling ----------------------------------------------------------
    def sample(self, t: float) -> dict[str, dict]:
        """Per-robot state at absolute time ``t``."""
        out: dict[str, dict] = {}
        for robot in self.robots:
            segs = self.by_robot.get(robot) or []
            active: Optional[Segment] = None
            for seg in segs:
                if seg.t0 <= t:
                    active = seg
                else:
                    break
            if active is None:
                out[robot] = {
                    "joints": self.home_of(robot), "phase": "idle",
                    "op_id": None, "op_type": None, "progress": 0.0,
                }
                continue
            local = t - active.t0
            dur = active.duration
            if local < dur:
                phase = active.op_type or "moving"
            elif local < dur + active.hold_s:
                phase = "hold"
            else:
                phase = "idle"
            out[robot] = {
                "joints": interpolate(active.traj, local),
                "phase": phase,
                "op_id": active.op_id,
                "op_type": active.op_type,
                "progress": 1.0 if dur <= 0 else max(0.0, min(1.0, local / dur)),
            }
        return out

    def describe(self) -> str:
        lines = [f"schedule: {len(self.segments)} segments, "
                 f"{self.duration:.2f}s total, robots={', '.join(self.robots)}"]
        for seg in sorted(self.segments, key=lambda s: (s.t0, s.robot)):
            lines.append(
                f"  {seg.t0:8.2f} -> {seg.t_done:8.2f}s  {seg.robot:<8} "
                f"{seg.op_type:<6} {seg.op_id:<20} "
                f"({len(seg.traj):4d} wp, hold {seg.hold_s:.1f}s)"
            )
        for w in self.warnings:
            lines.append(f"  WARN {w}")
        return "\n".join(lines)


# ───────────────────────────── playback clock ─────────────────────────────

class Player:
    """Wall-clock playhead over a Schedule, with cell-runner-style controls."""

    def __init__(self, schedule: Schedule, loop: bool = False,
                 speed: float = 1.0, autostart: bool = True):
        self._s = schedule
        self._loop = loop
        self._speed = max(0.01, float(speed))
        self._lock = threading.Lock()
        self._running = autostart
        self._pos = 0.0                    # playhead, seconds into the schedule
        self._wall = time.monotonic()
        self._cycle = 0

    def _advance(self) -> float:
        now = time.monotonic()
        if self._running:
            self._pos += (now - self._wall) * self._speed
            end = self._s.duration
            if self._pos >= end:
                if self._loop and end > 0:
                    self._pos = self._pos % end
                    self._cycle += 1
                else:
                    self._pos = end
                    self._running = False
        self._wall = now
        return self._pos

    def snapshot(self) -> dict:
        with self._lock:
            t = self._advance()
            states = self._s.sample(t)
            return {
                "t": t, "duration": self._s.duration, "running": self._running,
                "cycle": self._cycle, "speed": self._speed, "states": states,
            }

    def start(self) -> None:
        with self._lock:
            self._advance()
            if self._pos >= self._s.duration:
                self._pos = 0.0
            self._running = True

    def stop(self) -> None:
        with self._lock:
            self._advance()
            self._running = False

    def reset(self) -> None:
        with self._lock:
            self._pos = 0.0
            self._cycle = 0
            self._wall = time.monotonic()

    def seek(self, t: float) -> None:
        with self._lock:
            self._advance()
            self._pos = max(0.0, min(float(t), self._s.duration))

    def set_speed(self, speed: float) -> float:
        with self._lock:
            self._advance()
            self._speed = max(0.01, min(float(speed), 100.0))
            return self._speed

    def set_loop(self, loop: bool) -> bool:
        with self._lock:
            self._advance()
            self._loop = bool(loop)
            return self._loop

    @property
    def looping(self) -> bool:
        return self._loop


# ──────────────────────────── runtime session ────────────────────────────

class Session:
    """Owns the loaded plan and playhead, and supports runtime load/unload.

    The cell-runner equivalent is `MotionMode`: with nothing loaded the cell
    reports every robot at HOME (so the 3D viewer still renders the arms), and
    loading a plan switches the motion source to trajectory playback.
    """

    def __init__(self, cell_dir: str, robots: list[str], speed: float = 1.0,
                 loop: bool = False, autostart: bool = True):
        self.cell_dir = cell_dir
        self.robots = list(robots)
        self._speed = max(0.01, float(speed))
        self._loop = bool(loop)
        self._autostart = bool(autostart)
        self._lock = threading.RLock()
        self.plan_rel: Optional[str] = None
        self.meta: dict = {}
        self.schedule: Optional[Schedule] = None
        self.player: Optional[Player] = None

    # -- discovery ---------------------------------------------------------
    def plans_dir(self) -> str:
        return os.path.join(self.cell_dir, "trajectories")

    def list_plans(self) -> dict:
        tdir = self.plans_dir()
        plans = []
        if os.path.isdir(tdir):
            for name in sorted(os.listdir(tdir)):
                if not name.endswith(".json"):
                    continue
                full = os.path.join(tdir, name)
                entry = {"name": name, "path": f"trajectories/{name}",
                         "size_bytes": os.path.getsize(full)}
                try:
                    with open(full, encoding="utf-8") as fh:
                        doc = json.load(fh)
                    bt = doc.get("behaviour_tree") or doc
                    entry["tree_id"] = doc.get("tree_id") or bt.get("tree_id")
                    entry["cell_id"] = doc.get("cell_id") or bt.get("cell_id")
                    entry["operations"] = doc.get("num_operations")
                except (OSError, ValueError):
                    entry["error"] = "unreadable or not valid JSON"
                plans.append(entry)
        return {"plans": plans, "loaded": self.plan_rel}

    def _resolve(self, rel: str) -> str:
        """Resolve a plan path inside the cell dir, refusing traversal."""
        candidate = os.path.realpath(
            rel if os.path.isabs(rel) else os.path.join(self.cell_dir, rel))
        root = os.path.realpath(self.cell_dir)
        if not (candidate == root or candidate.startswith(root + os.sep)):
            raise ValueError(f"plan path escapes the cell directory: {rel}")
        if not os.path.isfile(candidate):
            raise ValueError(f"plan not found: {rel}")
        return candidate

    # -- lifecycle ---------------------------------------------------------
    def load(self, rel: Optional[str] = None,
             plan: Optional[dict] = None) -> dict:
        """Compile a plan into a schedule and make it the active motion source."""
        with self._lock:
            if plan is not None:
                bt = plan.get("behaviour_tree") or plan
                if "root" not in bt:
                    raise ValueError("inline plan has no behaviour tree root")
                meta = {
                    "tree_id": plan.get("tree_id") or bt.get("tree_id") or "inline",
                    "cell_id": plan.get("cell_id") or bt.get("cell_id"),
                    "num_operations": plan.get("num_operations"),
                }
                rel_name = None
            else:
                if not rel:
                    raise ValueError("either 'path' or 'plan' is required")
                full = self._resolve(rel)
                bt, meta = load_plan(full, self.cell_dir)
                rel_name = os.path.relpath(full, self.cell_dir).replace(os.sep, "/")

            robots = list(self.robots)
            for r in robots_in_plan(bt["root"]):
                if r not in robots:
                    robots.append(r)

            schedule = Schedule(robots)
            schedule.build(bt["root"])

            self.schedule = schedule
            self.meta = meta
            self.plan_rel = rel_name
            self.player = Player(schedule, loop=self._loop, speed=self._speed,
                                 autostart=self._autostart)
            return self.summary()

    def unload(self) -> dict:
        with self._lock:
            self.schedule = None
            self.player = None
            self.plan_rel = None
            self.meta = {}
            return {"loaded": False}

    # -- introspection -----------------------------------------------------
    def summary(self) -> dict:
        with self._lock:
            if not self.schedule:
                return {"loaded": False}
            ops = sorted({s.op_id for s in self.schedule.segments
                          if s.op_type != "move"})
            return {
                "loaded": True,
                "plan": self.plan_rel,
                "tree_id": self.meta.get("tree_id"),
                "cell_id": self.meta.get("cell_id"),
                "duration_s": round(self.schedule.duration, 3),
                "segments": len(self.schedule.segments),
                "operations": len(ops),
                "robots": list(self.schedule.robots),
                "warnings": list(self.schedule.warnings),
                # The Python player has no collision backend; the Rust
                # cell-runner fills this in from its scene checker.
                "collision": {"checked": False,
                              "reason": "collision checking requires the "
                                        "cell-runner backend"},
            }

    def robot_states(self) -> dict[str, dict]:
        """Per-robot state now; HOME for every robot when nothing is loaded."""
        with self._lock:
            if self.player is None:
                return {r: {"joints": list(HOME_JOINTS), "phase": "idle",
                            "op_id": None, "op_type": None, "progress": 0.0}
                        for r in self.robots}
            return self.player.snapshot()["states"]

    def playback(self) -> dict:
        with self._lock:
            if self.player is None:
                return {"loaded": False, "playing": False, "t": 0.0,
                        "duration": 0.0, "speed": self._speed,
                        "loop": self._loop, "cycle": 0}
            snap = self.player.snapshot()
            return {
                "loaded": True, "playing": snap["running"],
                "t": round(snap["t"], 3), "duration": round(snap["duration"], 3),
                "speed": snap["speed"], "loop": self.player.looping,
                "cycle": snap["cycle"],
            }

    def state(self) -> dict:
        """The `trajectory_state` payload the controller panel polls."""
        with self._lock:
            out = self.playback()
            out["plan"] = self.plan_rel
            out["tree_id"] = self.meta.get("tree_id")
            out["robots"] = [
                {"robot_id": rid, "phase": st["phase"], "op_id": st["op_id"],
                 "op_type": st["op_type"], "progress": round(st["progress"], 4),
                 "joints": [round(v, 6) for v in st["joints"]]}
                for rid, st in self.robot_states().items()
            ]
            return out

    # -- transport ---------------------------------------------------------
    def play(self) -> None:
        with self._lock:
            if self.player is None:
                raise ValueError("no trajectory loaded")
            self.player.start()

    def pause(self) -> None:
        with self._lock:
            if self.player is not None:
                self.player.stop()

    def stop(self) -> None:
        """Pause and rewind to the start."""
        with self._lock:
            if self.player is not None:
                self.player.stop()
                self.player.reset()

    def seek(self, t: float) -> None:
        with self._lock:
            if self.player is None:
                raise ValueError("no trajectory loaded")
            self.player.seek(t)

    def set_speed(self, speed: float) -> float:
        with self._lock:
            self._speed = max(0.01, min(float(speed), 100.0))
            if self.player is not None:
                return self.player.set_speed(self._speed)
            return self._speed

    def set_loop(self, loop: bool) -> bool:
        with self._lock:
            self._loop = bool(loop)
            if self.player is not None:
                return self.player.set_loop(self._loop)
            return self._loop


# ─────────────────────────── telemetry protocol ───────────────────────────

def build_state_result(session: Session) -> dict:
    """The `state` result, shaped exactly as the rerun-viewer's live_client
    expects (see rerun-viewer/src/live_client.rs::update_state).

    Required per robot in `krl`: `robot_id` and a 6-element `joints` array —
    the render loop's only input. `position` (Cartesian) is null because
    playback does not synthesize FK; live_client treats it as optional.

    With nothing loaded every robot reports HOME, so the 3D viewer still shows
    assembled arms rather than an empty scene.
    """
    pb = session.playback()
    krl, robots = [], []
    for robot_id, st in session.robot_states().items():
        joints = [float(v) for v in st["joints"]][:6]
        joints += [0.0] * (6 - len(joints))     # live_client requires exactly 6
        krl.append({
            "robot_id": robot_id,
            "position": None,
            "joints": joints,
            "active_folge": 11 if st["phase"] != "idle" else 0,
            "motion_progress": st["progress"],
            "operation_id": st["op_id"],
        })
        robots.append({
            "robot_id": robot_id,
            "phase": st["phase"],
            "model": "kr240_r2900_ultra",
            "joints": joints,
        })
    return {
        "connected": True,
        "running": pb["playing"],
        "cycle": pb["cycle"],
        "krl": krl,
        "robots": robots,
        # Playback extras (ignored by the viewer, used by the controller panel).
        "trajectory": session.state(),
    }


def dispatch(req: dict, session: Session) -> dict:
    """Handle one request, mirroring cell-runner's WS command surface.

    `state` / `start` / `stop` / `reset` / `set_speed` keep cell-runner's
    existing meaning. The `trajectory_*` verbs are the trajectory motion-source
    control surface the Trajectory Controller panel drives.
    """
    rid = req.get("id", 0)
    rtype = req.get("type", "")
    params = req.get("params") or {}

    def ok(result):
        return {"id": rid, "ok": True, "result": result}

    def err(msg):
        return {"id": rid, "ok": False, "error": str(msg)}

    try:
        # ── cell-runner compatible verbs ──────────────────────────────────
        if rtype == "state":
            return ok(build_state_result(session))
        if rtype in ("start", "run"):
            session.play()
            return ok(build_state_result(session))
        if rtype == "stop":
            session.pause()
            return ok(build_state_result(session))
        if rtype == "reset":
            session.stop()
            return ok(build_state_result(session))
        if rtype == "set_speed":
            return ok({"speed": session.set_speed(params.get("speed", 1.0))})
        if rtype == "unsubscribe":
            return ok({"unsubscribed": True})

        # ── trajectory motion source ──────────────────────────────────────
        if rtype == "trajectory_list":
            return ok(session.list_plans())
        if rtype == "trajectory_load":
            return ok(session.load(params.get("path"), params.get("plan")))
        if rtype == "trajectory_unload":
            return ok(session.unload())
        if rtype == "trajectory_play":
            session.play()
            return ok(session.state())
        if rtype == "trajectory_pause":
            session.pause()
            return ok(session.state())
        if rtype == "trajectory_stop":
            session.stop()
            return ok(session.state())
        if rtype == "trajectory_seek":
            session.seek(params.get("t", 0.0))
            return ok(session.state())
        if rtype == "trajectory_set_speed":
            session.set_speed(params.get("speed", 1.0))
            return ok(session.state())
        if rtype == "trajectory_set_loop":
            session.set_loop(bool(params.get("loop", False)))
            return ok(session.state())
        if rtype == "trajectory_state":
            return ok(session.state())
    except ValueError as exc:
        return err(exc)
    except Exception as exc:                      # never kill the connection
        return err(f"{type(exc).__name__}: {exc}")

    return err(f"unknown request type: {rtype}")


# ──────────────────────────── websocket framing ────────────────────────────

class WSClosed(Exception):
    pass


def ws_accept_key(client_key: str) -> str:
    # RFC 6455 4.2.2 fixes SHA-1 for Sec-WebSocket-Accept: a handshake check, not a security function.
    # nosemgrep: python.lang.security.insecure-hash-algorithms.insecure-hash-algorithm-sha1
    digest = hashlib.sha1((client_key + WS_MAGIC).encode("ascii"), usedforsecurity=False).digest()
    return base64.b64encode(digest).decode("ascii")


def ws_read_frame(rfile) -> tuple[int, bytes]:
    """Read one frame; returns (opcode, payload). Handles client masking."""
    hdr = rfile.read(2)
    if len(hdr) < 2:
        raise WSClosed()
    b0, b1 = hdr[0], hdr[1]
    opcode = b0 & 0x0F
    masked = bool(b1 & 0x80)
    length = b1 & 0x7F
    if length == 126:
        ext = rfile.read(2)
        if len(ext) < 2:
            raise WSClosed()
        length = struct.unpack(">H", ext)[0]
    elif length == 127:
        ext = rfile.read(8)
        if len(ext) < 8:
            raise WSClosed()
        length = struct.unpack(">Q", ext)[0]
    mask = rfile.read(4) if masked else b""
    if masked and len(mask) < 4:
        raise WSClosed()
    payload = b""
    while len(payload) < length:
        chunk = rfile.read(length - len(payload))
        if not chunk:
            raise WSClosed()
        payload += chunk
    if masked:
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return opcode, payload


def ws_write_frame(wfile, opcode: int, payload: bytes) -> None:
    """Write one unmasked (server-side) frame."""
    header = bytearray([0x80 | opcode])
    n = len(payload)
    if n < 126:
        header.append(n)
    elif n < 65536:
        header.append(126)
        header += struct.pack(">H", n)
    else:
        header.append(127)
        header += struct.pack(">Q", n)
    wfile.write(bytes(header) + payload)
    wfile.flush()


# ───────────────────────────── HTTP + WS server ─────────────────────────────

CONTENT_TYPES = {
    ".yaml": "application/x-yaml", ".yml": "application/x-yaml",
    ".json": "application/json", ".urdf": "application/xml",
    ".xml": "application/xml", ".obj": "text/plain",
    ".mtl": "text/plain", ".stl": "model/stl",
    ".glb": "model/gltf-binary", ".gltf": "model/gltf+json",
    ".dae": "model/vnd.collada+xml", ".png": "image/png",
    ".jpg": "image/jpeg", ".jpeg": "image/jpeg",
}


def make_handler(cell_dir: str, session: Session, verbose: bool):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "trajectory-player/1.0"

        def log_message(self, fmt, *args):
            if verbose:
                sys.stderr.write("[http] " + (fmt % args) + "\n")

        # -- static /env/<path> -------------------------------------------
        def _resolve(self, url_path: str) -> Optional[str]:
            """Map /env/<rel> to a file inside cell_dir, refusing traversal.

            `/env/cell.yaml` is an ALIAS, not a path. The viewer is a separate
            application that fetches that URL to read the scene-id map
            (`fetch_scene_id_map`), and we do not ship it — so the URL is the
            contract even though the file has since moved into
            `agentic_planner/`. Resolving the alias through `cell_yaml_path`
            means the served bytes are the manifest this player itself reads;
            hard-coding either location would have made the two disagree.
            """
            path = url_path.split("?", 1)[0].split("#", 1)[0]
            if not path.startswith("/env/"):
                return None
            rel = posixpath.normpath(path[len("/env/"):])
            # Traversal is refused BEFORE the alias, so `/env/../cell.yaml` is
            # still a 404 rather than an aliased read.
            if rel.startswith("..") or rel.startswith("/") or rel == ".":
                return None
            if rel == "cell.yaml":
                full = os.path.realpath(cell_yaml_path(cell_dir))
            else:
                full = os.path.realpath(os.path.join(cell_dir, *rel.split("/")))
            root = os.path.realpath(cell_dir)
            if not (full == root or full.startswith(root + os.sep)):
                return None
            return full if os.path.isfile(full) else None

        def do_GET(self):
            if self._is_ws_upgrade():
                return self._serve_ws()
            full = self._resolve(self.path)
            if full is None:
                return self._send_plain(404, "not found")
            try:
                with open(full, "rb") as fh:
                    body = fh.read()
            except OSError as exc:
                return self._send_plain(500, f"read error: {exc}")
            ctype = CONTENT_TYPES.get(os.path.splitext(full)[1].lower(),
                                      "application/octet-stream")
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self._send_cors()
            self.end_headers()
            self.wfile.write(body)

        def do_HEAD(self):
            full = self._resolve(self.path)
            if full is None:
                return self._send_plain(404, "not found", body=False)
            self.send_response(200)
            self.send_header("Content-Length", str(os.path.getsize(full)))
            self.end_headers()

        def do_POST(self):
            """JSON-over-HTTP alternative to the WS channel (handy for scripting)."""
            if self.path.rstrip("/") not in ("/rpc", "/api"):
                return self._send_plain(404, "not found")
            try:
                n = int(self.headers.get("Content-Length") or 0)
                req = json.loads(self.rfile.read(n) or b"{}")
            except (ValueError, json.JSONDecodeError) as exc:
                return self._send_json(400, {"ok": False, "error": str(exc)})
            return self._send_json(200, dispatch(req, session))

        def _send_cors(self):
            # CORS for pages served from this machine only (as cell-runner): another site's page open in
            # the same browser gets no Access-Control-Allow-Origin, so it can't read the cell's files.
            origin = self.headers.get("Origin", "")
            if loopback_origin(origin):
                self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")

        def _send_plain(self, code: int, msg: str, body: bool = True):
            payload = msg.encode()
            self.send_response(code)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            if body:
                self.wfile.write(payload)

        def _send_json(self, code: int, obj: dict):
            payload = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self._send_cors()
            self.end_headers()
            self.wfile.write(payload)

        # -- websocket ----------------------------------------------------
        def _is_ws_upgrade(self) -> bool:
            return (self.headers.get("Upgrade", "").lower() == "websocket"
                    and "upgrade" in self.headers.get("Connection", "").lower())

        def _serve_ws(self):
            key = self.headers.get("Sec-WebSocket-Key")
            if not key:
                return self._send_plain(400, "missing Sec-WebSocket-Key")
            self.send_response(101)
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", ws_accept_key(key))
            self.end_headers()
            peer = self.client_address
            sys.stderr.write(f"[ws] client connected {peer}\n")
            try:
                while True:
                    opcode, payload = ws_read_frame(self.rfile)
                    if opcode == 0x8:                       # close
                        ws_write_frame(self.wfile, 0x8, b"")
                        break
                    if opcode == 0x9:                       # ping -> pong
                        ws_write_frame(self.wfile, 0xA, payload)
                        continue
                    if opcode == 0xA:                       # pong
                        continue
                    if opcode not in (0x1, 0x2):
                        continue
                    try:
                        req = json.loads(payload.decode("utf-8") or "{}")
                    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                        resp = {"id": 0, "ok": False, "error": f"bad json: {exc}"}
                    else:
                        resp = dispatch(req, session)
                    ws_write_frame(self.wfile, 0x1,
                                   json.dumps(resp).encode("utf-8"))
            except (WSClosed, BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                sys.stderr.write(f"[ws] client disconnected {peer}\n")

    return Handler


# ────────────────────────── plan / cell.yaml loading ──────────────────────────

# The planning agents' workspace inside a cell. The manifest is generated by their
# converter, so it lives in here; cells converted before that keep it at the cell
# root. This player reads whichever is present — see `cell_yaml_path`.
AGENT_ROOT_NAME = "agentic_planner"


def cell_yaml_path(cell_dir: str) -> str:
    """The cell manifest: the workspace copy when present, else the cell root.

    Deliberately a probe rather than a constant. The manifest is written by
    `scripts/convert_isaac_scene.py`, which now puts it under
    `agentic_planner/`; a cell converted before that move still has it at the
    root, and this player has to open both without being told which.

    Returns the workspace path even when neither exists, so a caller reporting a
    missing manifest names the place a fresh conversion would put it.
    """
    agent = os.path.join(cell_dir, AGENT_ROOT_NAME, "cell.yaml")
    root = os.path.join(cell_dir, "cell.yaml")
    if os.path.isfile(agent):
        return agent
    return root if os.path.isfile(root) else agent


def robot_ids_from_cell_yaml(cell_dir: str) -> list[str]:
    """Telemetry robot ids from cell.yaml's `robots: [{id, scene_id}]`.

    Parsed with a tiny hand-rolled scan so the player stays stdlib-only (PyYAML
    is not guaranteed present). Only the two keys we need are extracted.
    """
    path = cell_yaml_path(cell_dir)
    ids: list[str] = []
    if not os.path.isfile(path):
        return ids
    in_robots = False
    with open(path, encoding="utf-8") as fh:
        for raw in fh:
            line = raw.rstrip("\n")
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            # A top-level key ends the robots block.
            if not line[0].isspace():
                in_robots = stripped.startswith("robots:")
                continue
            if not in_robots:
                continue
            if stripped.startswith("- id:"):
                ids.append(stripped.split(":", 1)[1].strip())
    return ids


def robots_in_plan(root: dict) -> list[str]:
    """Robot namespaces referenced anywhere in the tree, in first-seen order."""
    seen: list[str] = []

    def walk(node: dict) -> None:
        ns = node.get("robot_ns")
        if ns and ns not in seen:
            seen.append(ns)
        for child in node.get("children") or []:
            walk(child)

    walk(root)
    return seen


def load_plan(path: str, cell_dir: Optional[str] = None) -> tuple[dict, dict]:
    """Load an umarell plan file; returns (behaviour_tree, meta).

    With ``cell_dir``, the path must resolve (symlinks followed) to a regular file inside that
    directory, else ValueError: the player's own requests pass their cell directory.
    """
    if cell_dir is not None:
        root = os.path.realpath(cell_dir)
        path = os.path.realpath(path)
        if not (path == root or path.startswith(root + os.sep)):
            raise ValueError("plan path escapes the cell directory")
        if not os.path.isfile(path):
            raise ValueError("plan not found")
    with open(path, encoding="utf-8") as fh:
        doc = json.load(fh)
    # Plans are saved either wrapped ({tree_id, behaviour_tree: {...}}) or bare.
    bt = doc.get("behaviour_tree") or doc
    if "root" not in bt:
        raise SystemExit(f"{path}: no behaviour tree root found")
    meta = {
        "tree_id": doc.get("tree_id") or bt.get("tree_id") or "unknown",
        "cell_id": doc.get("cell_id") or bt.get("cell_id") or "unknown",
        "num_operations": doc.get("num_operations"),
    }
    return bt, meta


def default_plan(cell_dir: str, explicit: Optional[str]) -> str:
    if explicit:
        return explicit if os.path.isabs(explicit) else os.path.join(cell_dir, explicit)
    tdir = os.path.join(cell_dir, "trajectories")
    if not os.path.isdir(tdir):
        raise SystemExit(f"no trajectories/ directory in {cell_dir}; pass --plan")
    plans = sorted(f for f in os.listdir(tdir) if f.endswith(".json"))
    if not plans:
        raise SystemExit(f"no .json plans in {tdir}; pass --plan")
    return os.path.join(tdir, plans[-1])


# ──────────────────────────────────── main ────────────────────────────────────

def main(argv: Optional[list[str]] = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(
        description="Replay umarell BehaviourTree plans as cell-runner-compatible telemetry.")
    ap.add_argument("--cell", default=os.path.dirname(here),
                    help="cell directory (default: parent of scripts/)")
    ap.add_argument("--plan", default=None,
                    help="plan JSON (default: newest in <cell>/trajectories)")
    ap.add_argument("--no-plan", action="store_true",
                    help="start with nothing loaded; the Trajectory Controller "
                         "panel loads a plan over the wire")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9902,
                    help="cell-runner port the rerun-viewer connects to (default 9902)")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--loop", action="store_true", help="restart when the plan ends")
    ap.add_argument("--paused", action="store_true",
                    help="start paused (send a `start` request to begin)")
    ap.add_argument("--dump-schedule", action="store_true",
                    help="print the resolved timeline and exit")
    ap.add_argument("-v", "--verbose", action="store_true", help="log HTTP requests")
    args = ap.parse_args(argv)

    cell_dir = os.path.abspath(args.cell)
    if not os.path.isdir(cell_dir):
        raise SystemExit(f"cell directory not found: {cell_dir}")

    # Telemetry ids come from cell.yaml when present (that mapping is what the
    # viewer resolves scene ids to).
    session = Session(cell_dir, robot_ids_from_cell_yaml(cell_dir),
                      speed=args.speed, loop=args.loop,
                      autostart=not args.paused)

    # `--no-plan` starts empty: the Trajectory Controller panel then drives
    # trajectory_list / trajectory_load itself, which is the normal flow once
    # the panel is in use. Otherwise preload a plan so the player is useful on
    # its own (and so --dump-schedule has something to describe).
    if args.no_plan:
        print(f"cell dir  : {cell_dir}")
        print(f"no plan loaded - {len(session.list_plans()['plans'])} available "
              f"in trajectories/; load one with trajectory_load")
    else:
        plan_path = default_plan(cell_dir, args.plan)
        session.load(os.path.relpath(plan_path, cell_dir))
        if not session.robots:
            session.robots = list(session.schedule.robots)
        print(f"plan      : {session.plan_rel}")
        print(f"tree_id   : {session.meta['tree_id']}  "
              f"(cell {session.meta['cell_id']}, "
              f"{session.meta['num_operations']} operations)")
        print(f"cell dir  : {cell_dir}")
        print(session.schedule.describe())

    if args.dump_schedule:
        return 0

    handler = make_handler(cell_dir, session, args.verbose)

    class Server(ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    httpd = Server((args.host, args.port), handler)
    base = f"http://{args.host}:{args.port}"
    print()
    print(f"serving   : {base}/env/...   (static assets)")
    print(f"telemetry : ws://{args.host}:{args.port}/ws   (cell-runner state protocol)")
    print(f"playback  : speed {args.speed}x, loop {'on' if args.loop else 'off'}, "
          f"{'paused' if args.paused else 'running'}")
    print()
    print("Ctrl-C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping.")
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
