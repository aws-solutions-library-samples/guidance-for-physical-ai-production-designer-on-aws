#!/usr/bin/env python3
"""FactoryOS planner router: one MCP server for the chat agent, many planners behind it.

    python3 -I server.py --bind 127.0.0.1:8893 --cell-planner <src> --python <venv>/bin/python \
        --umarell <umarell checkout> --config <factoryos.config.json> [--work DIR] [--cells DIR]

The Kiro agent `factoryos-planner` (deploy/kiro/factoryos-planner.json) plans every cell through this
server (docs/tasks/cell-planner-integration.md). It routes a request either to

* the umarell LLM planner (the native planner MCP server on 127.0.0.1:8890, started through the
  backend's Agentic Planning if it is down), or
* one of factoryos-cell-planner's planners, mainly its learned policies (planners.json): a pinned
  `python -m tools.btexport --from <spec>` run as a subprocess, whose plan is published into the cell's
  agentic_planner/plans/ like umarell's own plans, or
* umarell_u3: the cell planner's U3 (umarell's own tools with the upstream fixes, driven by a deterministic
  orchestrator, no LLM; umarell_u3.py runs it in the umarell venv, then btexport checks its plan), or
* umarell_jam: the cell planner's cache-based umarell weld planner (umarell_jam.py: seconds from a warm component
  cache; with options.fallback, the default, a job it cannot plan falls back to umarell_u3 in the same job).

Jobs run in parallel, for any cell (not only the active one): FACTORYOS_PLANNER_WORKERS subprocess jobs at once
(default max(2, CPUs)), each with its own job dir and cell view; FACTORYOS_UMARELL_WORKERS umarell plans at once
(default 2) in the one umarell planner MCP (its plan_trajectories takes the cell per call and plans each in its own
thread). Two umarell_u3 jobs on the same cell take turns (U3 stages the cell in its shared patched copy), and so do
two umarell_jam jobs, and a umarell_jam job whose fallback is on with a umarell_u3 job.

umarell_u3's timeout and ETA follow the U3 workers a job gets (planners.json's timeout_s / typical_s are for
ref_workers = 8; u3_timeout / u3_eta): 4 x on a 2-vCPU workspace. Below FACTORYOS_BURST_MIN_CPUS CPUs (default 8), or
with options.remote true, a umarell_u3 job runs remotely instead (burst.py: the same command line on an ephemeral
arm64 instance of infra/lib/planner-burst.ts, its job dir downloaded and published as a local job's); when that can't
start, the job plans here with the scaled timeout and plan_status says why (remote.fallback). FACTORYOS_BURST=off:
only options.remote true plans remotely; always: every umarell_u3 job. burst.py is optional: without it (or without
the burst_instance.py it ships to the instance) every job plans here.

Transport: the plain JSON-RPC over HTTP subset umarell's utils/mcp_http.py serves to Kiro (`initialize`,
`ping`, `tools/list`, `tools/call`, `notifications/*`) on POST /mcp, and GET /health. Standard library
only (no venv): the planners run in their own venv (--python).
"""
from __future__ import annotations

import argparse
import errno
import hashlib
import heapq
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import types
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:   # -I: the script's folder is not on sys.path
    sys.path.insert(0, str(HERE))
try:
    import burst  # noqa: E402 - remote ("burst") U3 jobs on an ephemeral EC2 instance
    HAVE_BURST = True
except ModuleNotFoundError as e:
    if e.name != "burst":
        raise
    # No burst.py next to this file: no remote planning, every umarell_u3 job plans on this host (FACTORYOS_BURST is
    # ignored; options.remote true gets plan_status's remote.fallback "not installed"). The stand-in has what the
    # router calls before it knows there is no config.
    HAVE_BURST = False

    class _BurstUnavailable(Exception):
        """burst.BurstUnavailable, without burst.py."""

    def _no_burst_config():
        raise _BurstUnavailable("not installed on this host: no burst.py")

    burst = types.SimpleNamespace(BurstUnavailable=_BurstUnavailable, load_config=_no_burst_config, Aws=None)
SERVER_NAME = "factoryos-planner-router"
SERVER_VERSION = "1"
PLANS_DIR = "agentic_planner/plans"
TASKS_DIR = "agentic_planner/tasks"
# Learned policies first, best first (routing rule 2 of the task document).
POLICY_ORDER = ("hier_la_fu", "hier_q", "ppo_pure", "ppo_v3", "ppo_v4", "ppo_2t")
U3 = "umarell_u3"     # the cell planner's U3 (kind "rules"): umarell_u3.py
# umarell_u3's options.speed: "max" (the default since 2026-10-29) = U3 at the robots' limits (umarell_u3.py --speed
# max: transit 1.0, weld stroke 0.28, the run's ceiling lifted to 1.0) and btexport's full profile (the robots' joint
# limits: the tighter of the URDF and the cell's robots.yaml from the robot sidecar; weld operations <= 0.28 v_max;
# never the umarell profile); "normal" (alias
# "default", the name U3's own preset has) = U3's planning speeds (transit 0.3, the cell's 0.7 ceiling) and btexport's
# umarell profile (U3's own timing): the slower plans from before. profile "umarell" alone asks for "normal" too;
# profile full with speed "normal" re-times U3's normal-speed plan only.
U3_SPEEDS = ("max", "normal", "default")
U3_DEFAULT_SPEED = "max"
JAM = "umarell_jam"   # the cell planner's cache-based umarell weld planner (kind "rules"): umarell_jam.py
# umarell_jam's own options (plan's options; planners.json has their defaults). A job's timeout: umarell_jam's
# --timeout + JAM_MARGIN_S (the shared cache pull, its setup, btexport's export and dense check) + with the fallback
# on umarell_u3's timeout for the U3 workers the job gets.
JAM_OPTIONS = ("welds", "robots", "timeout_s", "repair_budget_s", "prepare_budget_s", "fallback")
JAM_MARGIN_S = 300.0
JAM_FALLBACK_MARK = "falling back to umarell_u3"   # umarell_jam.py's log line: the job's phase is the fallback
KEEP_JOB_DIRS = 20
CANCEL_GRACE_S = 10.0
UMARELL_START_WAIT_S = 90.0
PREPARE_WAIT_S = 300.0
PREPARED_MARKER = "agentic_planner/cell.yaml"   # what Prepare Cell writes (as the backend's `plans.prepared`)
PREPARED_INPUTS = "agentic_planner/inputs.json"  # what it was prepared from (the backend's cell_resolve InputHashes)
MANIFEST = "factoryos-cell.json"   # a cell's manifest: format 2 {"kind": "reference", "fitted_scene": "<scene name>"}
SCENE_FILE = "scene.yaml"          # a Robo Fitter scene is the folder <scenes>/<name>/scene.yaml, referred to by <name>
VIEW_DIR = ".view"                 # a reference cell's view (the backend's cell_resolve::materialise_view)
SRDF_DIR = "agentic_planner/srdf"  # a reference cell's SRDFs (the backend's cell_resolve::SRDF_DIR), linked into its view
UMARELL_POLL_S = 2.0          # the umarell jobs' status poll (frees their slots)
UMARELL_LOST_POLLS = 5        # polls umarell does not know a job before it is failed (umarell restarted)
UMARELL_ETA_S = 600.0         # an umarell plan without typical_s in planners.json
CPUS = os.cpu_count() or 2
# umarell_u3's timeout_s / typical_s in planners.json are for ref_workers (default 8) U3 workers: a job with fewer
# (a small workspace, or jobs sharing its CPUs) gets a timeout longer in proportion (u3_timeout, at most timeout_max_s)
# and an ETA to match; a remote job (burst.py) adds remote_setup_s for the instance's boot and install.
U3_REF_WORKERS = 8
U3_TIMEOUT_MAX_S = 4 * 3600.0
BURST_SETUP_S = 420.0
BURST_MIN_CPUS = 8        # FACTORYOS_BURST_MIN_CPUS: auto remote planning below this many CPUs


def env_int(name: str, default: int) -> int:
    """A positive integer from the environment, else `default`."""
    try:
        value = int((os.environ.get(name) or "").strip() or 0)
    except ValueError:
        value = 0
    return value if value > 0 else default


def u3_timeout(base_s: float, workers: int, ref: int = U3_REF_WORKERS, cap_s: float = U3_TIMEOUT_MAX_S) -> float:
    """umarell_u3's timeout with `workers` U3 workers: planners.json's (for `ref` workers) x ref / workers when fewer,
    never shorter, at most cap_s (4800 s for 8 workers: 14400 s, the cap, on a 2-vCPU workspace)."""
    w = max(1, int(workers or 1))
    return round(min(max(cap_s, base_s), base_s * max(1.0, ref / w)), 1)


def u3_eta(typical_s: float, workers: int, ref: int = U3_REF_WORKERS) -> float:
    """umarell_u3's expected planning time with `workers` U3 workers: typical_s x ref / workers, at least half of
    typical_s (U3's legs do not all run in parallel)."""
    w = max(1, int(workers or 1))
    return round(typical_s * max(0.5, ref / w), 1)


class ToolError(Exception):
    """A tool's error result ({"error", "hint"}), relayed to the agent rather than raised to the client."""

    def __init__(self, error: str, hint: str = "", **extra):
        super().__init__(error)
        self.result = {"error": error, "hint": hint, **extra}


# ───────────────────────────────────────── helpers ─────────────────────────────────────────
def planner_cell_id(cell: str) -> str:
    """The agentic planner's id for a FactoryOS cell folder (the backend's cells.rs planner_cell_id)."""
    cid = "".join(c if c.isascii() and c.isalnum() else "_" for c in cell)
    return f"cell_{cid}" if cid[:1].isdigit() else cid


def cell_root(cell_dir: str) -> Path:
    """The cell folder of a configured cellDir (the backend's cells.rs cell_root: not `cellrunner/`, not a reference
    cell's view `.view/`)."""
    p = Path(cell_dir)
    return p.parent if p.name in ("cellrunner", VIEW_DIR) else p


def read_manifest(root: Path) -> dict:
    """The cell's factoryos-cell.json ({} if none)."""
    try:
        m = json.loads((root / MANIFEST).read_text())
    except (OSError, ValueError):
        return {}
    return m if isinstance(m, dict) else {}


def is_reference_cell(root: Path) -> bool:
    """A cell that references its Robo Fitter scene (cells reference, never copy): no scene.yaml of its own (that,
    a legacy self-contained cell, wins) and a format 2 manifest naming the scene."""
    if (root / "scene.yaml").is_file() or (root / "scene.factoryos.yaml").is_file():
        return False
    m = read_manifest(root)
    return bool(m.get("fitted_scene")) and int(m.get("format") or 1) >= 2 and m.get("kind") != "copy"


def scene_name(ref: Optional[str]) -> Optional[str]:
    """The Robo Fitter scene name a reference gives (the backend's scene_store::name_of): a name (letters, digits,
    _, -, .; not starting with .), or a legacy `<name>.yaml`/`.yml` (the extension stripped). None otherwise."""
    ref = str(ref or "").strip()
    for ext in (".yaml", ".yml"):
        if ref.endswith(ext):
            ref = ref[: -len(ext)]
            break
    if not ref or ref.startswith(".") or re.fullmatch(r"[A-Za-z0-9_.\-]+", ref) is None:
        return None
    return ref


def scene_asset(base: Path, rel: str) -> Path:
    """A scene's asset path `rel` from its folder `base` (the backend's scene_store::asset_path): a relative path
    that does not exist from a scene folder (<scenes>/<name>/) but does from the scenes folder (the old flat
    layout's base) is that one."""
    p = Path(rel)
    if p.is_absolute():
        return p
    primary = base / p
    if not primary.exists() and base.parent.name == "scenes" and (base.parent / p).exists():
        return base.parent / p
    return primary


def clean_cell_name(scene: str) -> str:
    """A cell name from a scene name, or a legacy scene file name (the backend's cell_resolve::clean_name)."""
    stem = scene_name(scene) or Path(scene).stem
    name = "".join(c if c.isascii() and c.isalnum() else "_" for c in stem).strip("_")
    return name or "cell"


def valid_name(name: str) -> bool:
    return bool(name) and ".." not in name and re.fullmatch(r"[A-Za-z0-9_.\-]+", name) is not None


_SHA_CACHE: dict = {}


def sha256_file(path: Path) -> Optional[str]:
    try:
        st = path.stat()
    except OSError:
        return None
    key = (str(path), st.st_mtime_ns, st.st_size)
    if key not in _SHA_CACHE:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        _SHA_CACHE[key] = h.hexdigest()
    return _SHA_CACHE[key]


def write_json_atomic(path: Path, value: Any) -> None:
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with open(tmp, "w") as f:
        json.dump(value, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


def http_json(url: str, body: Any = None, timeout: float = 10.0) -> Any:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method="GET" if body is None else "POST",
                                 headers={"Content-Type": "application/json", "Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        text = resp.read().decode() or "null"
    return json.loads(text)


# ───────────────────────────────────────── products: welds in the world frame ─────────────────────────────────────────
# A Product & Task product (`product.json`, factoryos_protocol::product) holds its operations in the product's own
# frame (m, z up; a box stands on its origin) with the outward surface normal. A cell places it as a scene.yaml
# fixture (the Fitter copies product.json next to the fixture's URDF): position x, y, z (m) and orientation a, b, c
# (degrees) as R = Rz(a)·Ry(b)·Rx(c) (the Fitter's reach/task.rs fixture_pose). In the world: p_w = R·p + t, and the
# approach (into the surface) is -R·normal. The tool orientation follows the training cell's task programs: the tool
# axis is -R[:,0] of the orientation, top-down (approach -z) is {x: 0, y: -0.7071, z: 0, w: 0.7071}.
TOP_DOWN = {"x": 0.0, "y": -0.7071, "z": 0.0, "w": 0.7071}
MAX_TOOL_AXIS_DEG = 10.0      # save_task_program: the orientation's tool axis vs approach_vector
INSIDE_EPS_M = 1e-3           # a position deeper than this inside a non-workpiece box is refused
WORKPIECE_TOL_M = 0.01        # task_program_from_products marks the boxes within this of a weld as is_workpiece
WORKPIECE_NEAR_M = 0.12       # ... and the boxes of that fixture's voxel shell within this (the tool's 0.05 m spheres
                              # + 0.05 m margin keep clear of every other box: its neighbouring voxels block the weld)
REFINE_RADIUS_M = 0.25        # ... and of its fine shell (`<fixture>_f<n>`, Prepare Cell's near-weld refinement) within this


def _dot(u, v):
    return sum(a * b for a, b in zip(u, v))


def _cross(u, v):
    return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]


def _unit(v):
    n = _dot(v, v) ** 0.5
    return [a / n for a in v] if n > 1e-12 else None


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(r, v):
    return [_dot(r[i], v) for i in range(3)]


def euler_zyx_deg(a: float, b: float, c: float) -> list:
    """Rz(a)·Ry(b)·Rx(c), degrees (scene.yaml's orientation a, b, c)."""
    ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
    cb, sb = math.cos(math.radians(b)), math.sin(math.radians(b))
    cc, sc = math.cos(math.radians(c)), math.sin(math.radians(c))
    rz = [[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]]
    ry = [[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]]
    rx = [[1.0, 0.0, 0.0], [0.0, cc, -sc], [0.0, sc, cc]]
    return _matmul(rz, _matmul(ry, rx))


def quat_to_matrix(q: dict) -> list:
    x, y, z, w = (float(q.get(k, 0.0)) for k in "xyzw")
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-9:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def matrix_to_quat(r: list) -> dict:
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = (t + 1.0) ** 0.5 * 2
        w, x, y, z = s / 4, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = (1.0 + r[0][0] - r[1][1] - r[2][2]) ** 0.5 * 2
        w, x, y, z = (r[2][1] - r[1][2]) / s, s / 4, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = (1.0 + r[1][1] - r[0][0] - r[2][2]) ** 0.5 * 2
        w, x, y, z = (r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, s / 4, (r[1][2] + r[2][1]) / s
    else:
        s = (1.0 + r[2][2] - r[0][0] - r[1][1]) ** 0.5 * 2
        w, x, y, z = (r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, s / 4
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return {k: round(v, 4) + 0.0 for k, v in zip("xyzw", (x, y, z, w))}


def tool_axis(q: dict) -> list:
    """The tool axis of a task program orientation: -R[:,0]."""
    r = quat_to_matrix(q)
    return [-r[0][0], -r[1][0], -r[2][0]]


def orientation_for(approach: list) -> dict:
    """The orientation whose tool axis (-R[:,0]) is `approach`: the top-down orientation turned by the shortest
    rotation from -z to `approach` (so approach -z gives exactly TOP_DOWN)."""
    a = _unit(approach) or [0.0, 0.0, -1.0]
    down = [0.0, 0.0, -1.0]
    axis, cos = _cross(down, a), _dot(down, a)
    sin = _dot(axis, axis) ** 0.5
    if sin < 1e-9:
        turn = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] if cos > 0 else \
            [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]   # from below: 180° about x
    else:
        k = [v / sin for v in axis]
        ang = math.atan2(sin, cos)
        kx = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]]
        kx2 = _matmul(kx, kx)
        turn = [[(1.0 if i == j else 0.0) + math.sin(ang) * kx[i][j] + (1 - math.cos(ang)) * kx2[i][j]
                 for j in range(3)] for i in range(3)]
    return matrix_to_quat(_matmul(turn, quat_to_matrix(TOP_DOWN)))


def angle_deg(u: list, v: list) -> float:
    u, v = _unit(u), _unit(v)
    return math.degrees(math.acos(max(-1.0, min(1.0, _dot(u, v)))))


# A YAML subset for the files the backend writes (scene.yaml, obstacles.yaml): block mappings and lists, flow lists
# of scalars, scalars, comments. The router has no PyYAML (standard library only).
def _yaml_scalar(s: str) -> Any:
    s = s.strip()
    if s.startswith("[") and s.endswith("]"):
        return [_yaml_scalar(v) for v in s[1:-1].split(",") if v.strip()]
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "\"'":
        return s[1:-1]
    low = s.lower()
    if low in ("true", "false"):
        return low == "true"
    if low in ("null", "~", ""):
        return None
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        return s


def _strip_comment(line: str) -> str:
    quote = None
    for i, ch in enumerate(line):
        if quote:
            quote = None if ch == quote else quote
        elif ch in "\"'":
            quote = ch
        elif ch == "#" and (i == 0 or line[i - 1] in " \t"):
            return line[:i].rstrip()
    return line.rstrip()


def parse_yaml(text: str) -> Any:
    lines = []
    for raw in text.splitlines():
        line = _strip_comment(raw)
        if line.strip():
            lines.append([len(line) - len(line.lstrip(" ")), line.strip()])

    def block(i: int, indent: int) -> tuple[Any, int]:
        if i >= len(lines):
            return None, i
        if lines[i][1].startswith("- ") or lines[i][1] == "-":
            out = []
            while i < len(lines) and lines[i][0] == indent and (lines[i][1].startswith("- ") or lines[i][1] == "-"):
                rest = lines[i][1][2:].strip()
                if not rest:
                    value, i = block(i + 1, lines[i + 1][0]) if i + 1 < len(lines) else (None, i + 1)
                elif re.match(r"^[^\[\"'][^:]*:(\s|$)", rest):   # "- key: value": a mapping at indent + 2
                    lines[i] = [indent + 2, rest]
                    value, i = block(i, indent + 2)
                else:
                    value, i = _yaml_scalar(rest), i + 1
                out.append(value)
            return out, i
        out = {}
        while i < len(lines) and lines[i][0] == indent and not lines[i][1].startswith("- "):
            key, _, rest = lines[i][1].partition(":")
            key, rest = key.strip().strip("\"'"), rest.strip()
            i += 1
            if rest:
                out[key] = _yaml_scalar(rest)
            elif i < len(lines) and (lines[i][0] > indent or (lines[i][0] == indent and lines[i][1].startswith("- "))):
                out[key], i = block(i, lines[i][0])
            else:
                out[key] = None
        return out, i

    value, _ = block(0, lines[0][0]) if lines else (None, 0)
    return value


def obstacle_boxes(text: str) -> list:
    """The box obstacles of an obstacles.yaml: [{id, center, half_extents, is_workpiece}]."""
    try:
        doc = parse_yaml(text) or {}
    except (IndexError, ValueError):
        return []
    out = []
    for o in (doc.get("obstacles") if isinstance(doc, dict) else None) or []:
        if not isinstance(o, dict) or o.get("type", "box") != "box":
            continue
        c, h = o.get("center"), o.get("half_extents")
        if isinstance(c, list) and isinstance(h, list) and len(c) == 3 and len(h) == 3:
            try:
                out.append({"id": str(o.get("id", "")), "center": [float(v) for v in c],
                            "half_extents": [abs(float(v)) for v in h], "is_workpiece": o.get("is_workpiece") is True})
            except (TypeError, ValueError):
                continue
    return out


def box_depth(box: dict, p: list) -> float:
    """How deep `p` is inside the (axis-aligned) box: > 0 inside, <= 0 on or outside."""
    return min(h - abs(x - c) for x, c, h in zip(p, box["center"], box["half_extents"]))


def shell_of(box_id: str) -> Optional[str]:
    """The fixture of a voxel-shell box id: the converter's coarse `<fixture>_v<n>` or its near-weld refinement's
    fine `<fixture>_f<n>`, else None."""
    m = re.fullmatch(r"(.+)_[vf]\d+", str(box_id))
    return m.group(1) if m else None


def shell_near_m(box_id: str) -> Optional[float]:
    """How far from a weld the boxes of the welded fixture's shell are workpiece: the coarse shell (`_v`) within
    WORKPIECE_NEAR_M, the fine one (`_f`) within REFINE_RADIUS_M; None for a box of no shell."""
    m = re.fullmatch(r"(.+)_([vf])\d+", str(box_id))
    return None if not m else REFINE_RADIUS_M if m.group(2) == "f" else WORKPIECE_NEAR_M


def workpiece_ids(boxes: list, points: list) -> set:
    """The ids of the boxes (not yet is_workpiece) the welds at `points` are on: within WORKPIECE_TOL_M of one, and
    of the same fixture's voxel shell within WORKPIECE_NEAR_M (coarse `_v` boxes) or REFINE_RADIUS_M (fine `_f`
    boxes) (the backend's product_tasks::workpiece_ids)."""
    out = set()
    for p in points:
        on = [b for b in boxes if box_depth(b, p) >= -WORKPIECE_TOL_M]
        shells = {shell_of(b["id"]) for b in on} - {None}
        out |= {b["id"] for b in on}
        out |= {b["id"] for b in boxes
                if shell_of(b["id"]) in shells and box_depth(b, p) >= -shell_near_m(b["id"])}
    return {i for i in out if not next(b for b in boxes if b["id"] == i)["is_workpiece"]}


def mark_workpieces(text: str, ids: set) -> str:
    """obstacles.yaml with `is_workpiece: true` on the obstacles `ids` (text-edited: comments and layout stay)."""
    lines = text.splitlines(keepends=True)
    out, i = [], 0
    item = re.compile(r"^(\s*)-\s+id:\s*[\"']?([^\"'#\s]+)[\"']?\s*(#.*)?$")
    while i < len(lines):
        m = item.match(lines[i].rstrip("\n"))
        if not m or m.group(2) not in ids:
            out.append(lines[i])
            i += 1
            continue
        indent = len(m.group(1))
        j = i + 1
        while j < len(lines) and (not lines[j].strip() or lines[j].lstrip().startswith("#")
                                  or len(lines[j]) - len(lines[j].lstrip(" ")) > indent) \
                and not item.match(lines[j].rstrip("\n")):
            j += 1
        blk = lines[i:j]
        last = max(k for k, line in enumerate(blk) if line.strip() and not line.lstrip().startswith("#"))
        note = "   # task_program_from_products: the product's welds are on it (tool tip may touch)\n"
        for k, line in enumerate(blk):
            if re.match(r"^\s*is_workpiece:", line):
                blk[k] = re.sub(r"is_workpiece:.*$", "is_workpiece: true" + note.rstrip("\n"), line.rstrip("\n")) + "\n"
                break
        else:
            body = (blk[last] if blk[last].endswith("\n") else blk[last] + "\n")
            blk[last] = body + " " * (indent + 2) + "is_workpiece: true" + note
        out.extend(blk)
        i = j
    return "".join(out)


def place_product(product: dict, fixture: dict) -> dict:
    """A product's point operations in the world frame by its scene.yaml `fixture`: [{id, task, position,
    approach_vector, orientation, group}], the paths it skipped, and its box (world AABB, for axis-aligned poses)."""
    pos, ori = fixture.get("position") or {}, fixture.get("orientation") or {}
    t = [float(pos.get(k) or 0.0) for k in "xyz"]
    r = euler_zyx_deg(*(float(ori.get(k) or 0.0) for k in "abc"))
    welds, skipped = [], []
    for task in product.get("tasks") or []:
        for op in task.get("operations") or []:
            target = op.get("target") or {}
            if target.get("kind") != "point":
                skipped.append(f"{task.get('name')}/{op.get('id')} ({target.get('kind')})")
                continue
            p = [float(v) for v in target.get("position") or [0, 0, 0]]
            n = _unit([float(v) for v in target.get("normal") or [0, 0, 1]]) or [0.0, 0.0, 1.0]
            world = [round(a + b, 6) + 0.0 for a, b in zip(_apply(r, p), t)]
            approach = [round(-v, 6) + 0.0 for v in _apply(r, n)]
            welds.append({"id": str(op.get("id")), "task": task.get("name"), "process": task.get("process"),
                          "position": world, "approach_vector": approach, "orientation": orientation_for(approach),
                          "group": op.get("group") or "structural",
                          "priority": int((op.get("params") or {}).get("priority", 1))})
    return {"welds": welds, "skipped_paths": skipped, "pose": {"position": t, "orientation_deg": {
        k: float(ori.get(k) or 0.0) for k in "abc"}, "convention": "R = Rz(a)·Ry(b)·Rx(c), p_world = R·p + position"}}


# ───────────────────────────────────────── settings / state ─────────────────────────────────────────
@dataclass
class Settings:
    cell_planner: Path                 # factoryos-cell-planner source tree (read-only is fine)
    python: Path                       # its venv's python
    umarell: Path                      # the umarell checkout (its cells/<cell_id>/agentic_planner: the training copy)
    config: Optional[Path]             # factoryos.config.json (cellsDir, cellDir = the active cell)
    work: Path                         # job dirs and the cell views
    cells: Optional[Path] = None       # overrides the config's cellsDir
    planners_file: Path = HERE / "planners.json"
    umarell_url: str = "http://127.0.0.1:8890/mcp"
    backend_url: str = "http://127.0.0.1"
    commit: str = ""
    umarell_python: Optional[Path] = None  # umarell's venv python (U3); default <umarell>/.venv-planner/bin/python
    workers: Optional[int] = None      # subprocess jobs at once (default $FACTORYOS_PLANNER_WORKERS, max(2, CPUs))
    umarell_workers: Optional[int] = None  # umarell plans at once (default $FACTORYOS_UMARELL_WORKERS, 2)


@dataclass
class Job:
    job_id: str
    tree_id: str
    planner: str
    cell: str
    cell_id: str
    cell_dir: Path
    profile: str
    spec: str
    cmd: list
    timeout_s: float
    eta_s: float
    dir: Path
    task: str
    task_file: Optional[Path] = None  # the cell's task program for the view (None: the checkout's canonical one)
    speed: Optional[str] = None       # umarell_u3: options.speed (U3_SPEEDS)
    status: str = "queued"            # queued | planning | done | failed | cancelled
    phase: str = "queued"
    submitted_s: float = field(default_factory=time.time)
    started_s: Optional[float] = None
    ended_s: Optional[float] = None
    exit_code: Optional[int] = None
    plan_path: Optional[str] = None
    export: dict = field(default_factory=dict)
    error: Optional[str] = None
    hint: Optional[str] = None
    proc: Optional[subprocess.Popen] = None
    cancel: bool = False
    kind: str = "btexport"            # btexport: a subprocess job | umarell: a plan in the umarell planner MCP
    umarell_args: dict = field(default_factory=dict)   # umarell: plan_trajectories' arguments
    umarell: dict = field(default_factory=dict)        # umarell: its last get_plan_status
    submitted: bool = False           # umarell: plan_trajectories accepted it (tree_id is umarell's)
    misses: int = 0                   # umarell: polls in a row it did not know the job
    view: Optional[Path] = None       # btexport: the job's own cell view (UMARELL_REPO)
    timeout_base_s: float = 0.0       # umarell_u3: planners.json's timeout_s / typical_s (for ref_workers)
    eta_base_s: float = 0.0
    u3_workers: Optional[int] = None  # umarell_u3: the U3 workers it runs with (FACTORY_OS_U3_WORKERS)
    remote: bool = False              # umarell_u3: runs on a planner-burst instance (burst.py)
    jam: dict = field(default_factory=dict)     # umarell_jam: timeout_s (its own), fallback {enabled, why, used}
    burst: dict = field(default_factory=dict)   # remote planning: why, instance, phase, fallback (plan_status)
    launcher: Any = None              # remote: its burst.Launcher


class Router:
    def __init__(self, s: Settings):
        self.s = s
        self.catalog = json.loads(s.planners_file.read_text())
        self.planners = {p["name"]: p for p in self.catalog["planners"]}
        self.training = self.catalog.get("training_cells", {})
        self.canonical_task = self.catalog.get("canonical_task", "car_door_spot_welding")
        self.jobs: dict[str, Job] = {}
        self.alias: dict[str, str] = {}   # umarell tree_id -> job_id of a job that was queued before umarell had it
        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)   # a job queued / ended
        self.prepare_lock = threading.Lock()       # the backend runs one Prepare Cell at a time
        self.reserved: set[str] = set()            # tree ids of live jobs (router and umarell plans)
        self.commit = s.commit or self._commit()
        if s.umarell_python is None:
            s.umarell_python = s.umarell / ".venv-planner" / "bin" / "python"
        self.workers = s.workers or env_int("FACTORYOS_PLANNER_WORKERS", max(2, CPUS))
        self.umarell_workers = s.umarell_workers or env_int("FACTORYOS_UMARELL_WORKERS", 2)
        # Remote planning (burst.py): FACTORYOS_BURST auto (default: below FACTORYOS_BURST_MIN_CPUS CPUs, or when
        # options.remote asks), always, off (options.remote true still asks for it); off without burst.py.
        self.burst_mode = (os.environ.get("FACTORYOS_BURST") or "auto").strip().lower() if HAVE_BURST else "off"
        self.burst_min_cpus = env_int("FACTORYOS_BURST_MIN_CPUS", BURST_MIN_CPUS)
        self.burst_load = burst.load_config        # tests stub these two
        self.burst_aws = burst.Aws
        self._burst_cfg: tuple = (0.0, None, "")   # (read at, config, why not)
        self._burst_slots: Optional[threading.BoundedSemaphore] = None
        shutil.rmtree(s.work / "view", ignore_errors=True)   # the views of a previous run's jobs
        self.threads = [threading.Thread(target=self._work, name=f"planner-job-{i}", daemon=True)
                        for i in range(self.workers)]
        self.threads.append(threading.Thread(target=self._umarell_loop, name="umarell-jobs", daemon=True))
        for t in self.threads:
            t.start()

    # ---------------------------------------------------------------- environment
    def _commit(self) -> str:
        pin = self.s.cell_planner / ".cell-planner.txt"
        if pin.is_file():
            parts = pin.read_text().split()
            if parts:
                return parts[0]
        try:
            out = subprocess.run(["git", "-C", str(self.s.cell_planner), "rev-parse", "--short=7", "HEAD"],
                                 capture_output=True, text=True, timeout=5)
            return out.stdout.strip() or "unknown"
        except (OSError, subprocess.SubprocessError):
            return "unknown"

    def _config(self) -> dict:
        if not self.s.config:
            return {}
        try:
            return json.loads(self.s.config.read_text())
        except (OSError, ValueError):
            return {}

    def _resolve_path(self, value: str) -> str:
        value = (value or "").strip()
        if value.startswith("~"):
            value = os.path.expanduser(value)
        if value and not os.path.isabs(value) and self.s.config:
            value = str(self.s.config.parent / value)
        return value

    def cells_dir(self) -> Optional[Path]:
        if self.s.cells:
            return self.s.cells
        cfg = self._config()
        configured = self._resolve_path(cfg.get("cellsDir", ""))
        if configured:
            return Path(configured)
        cell = self._resolve_path(cfg.get("cellDir", ""))
        return cell_root(cell).parent if cell else None

    def fitted_dir(self) -> Path:
        """The catalogue's scenes folder (fittedRobotsDir, else <assetRoot>/scenes), as the backend's: a configured
        .../scenes (the old default, migrated into scenes/ by the backend) counts as unset."""
        cfg = self._config()
        configured = self._resolve_path(cfg.get("fittedRobotsDir", ""))
        if configured and Path(configured).name != "scenes":
            return Path(configured)
        return Path(self._resolve_path(cfg.get("assetRoot", "") or ".")) / "scenes"

    def fitted_scene_file(self, scene: Optional[str]) -> Path:
        """The file of Robo Fitter scene `scene` (a name, or a legacy `<name>.yaml`): <scenes>/<name>/scene.yaml; a
        flat <scenes>/<name>.yaml not migrated yet (the backend migrates it at its start) while that is all there is."""
        fitted = self.fitted_dir()
        name = scene_name(scene) or str(scene)
        folder = fitted / name / SCENE_FILE
        if not folder.is_file():
            for ext in (".yaml", ".yml"):
                if (fitted / f"{name}{ext}").is_file():
                    return fitted / f"{name}{ext}"
        return folder

    def scene_path(self, root: Path) -> Path:
        """The cell's scene: a legacy cell's own scene.yaml; a reference cell's view scene.yaml (its paths into the
        view's links to the catalogue robots and products), else the Robo Fitter scene it points at."""
        if not is_reference_cell(root):
            return root / "scene.yaml"
        view = root / VIEW_DIR / "scene.yaml"
        if view.is_file():
            return view
        return self.fitted_scene_file(read_manifest(root).get("fitted_scene"))

    def robots_dir(self, root: Path) -> Optional[Path]:
        """The cell's robot packages with their meshes: a legacy cell's own robots/, a reference cell's view robots/
        (links to the catalogue's packages)."""
        for robots in (root / "robots", root / VIEW_DIR / "robots"):
            if self._has_meshes(robots):
                return robots
        return None

    def cell_for_scene(self, scene: str) -> Optional[str]:
        """The cell of Robo Fitter scene `scene` (as the backend's cell_resolve::cell_for_scene): a cell folder that
        references it (or was copied from it), reference cells first, then the one named after it; made (only its
        manifest, format 2) when there is none. None: no such scene, or no cells folder."""
        scene = scene_name(scene)
        if not scene or not self.fitted_scene_file(scene).is_file():
            return None
        cells = self.cells_dir()
        if not cells:
            return None
        found = self.existing_cell_for_scene(scene)
        if found:
            return found
        base = clean_cell_name(scene)
        i = 1
        while True:
            name = base if i == 1 else f"{base}_{i}"
            d = cells / name
            if not d.exists() or (d.is_dir() and not any(d.iterdir())):
                break
            i += 1
        d.mkdir(parents=True, exist_ok=True)
        write_json_atomic(d / MANIFEST, {"format": 2, "kind": "reference", "fitted_scene": scene,
                                         "cell_id": planner_cell_id(name)})
        print(f"[router] cell {name}: made for the Robo Fitter scene {scene} (references it; nothing copied)",
              file=sys.stderr, flush=True)
        return name

    def existing_cell_for_scene(self, scene: str) -> Optional[str]:
        """The cell folder that references Robo Fitter scene `scene` (or was copied from it): reference cells first,
        then the one named after it (cell_for_scene's choice, never making one); None if there is none."""
        cells = self.cells_dir()
        scene = scene_name(scene)
        if not cells or not cells.is_dir() or not scene:
            return None
        base = clean_cell_name(scene)
        made = []
        for d in sorted(cells.iterdir()):
            m = read_manifest(d) if d.is_dir() and not d.name.startswith(".") else {}
            if scene_name(m.get("fitted_scene")) == scene:
                made.append((not (int(m.get("format") or 1) >= 2 and m.get("kind") != "copy"), d.name != base, d.name))
        return sorted(made)[0][2] if made else None

    def active_cell(self) -> Optional[tuple[str, Path]]:
        cell = self._resolve_path(self._config().get("cellDir", ""))
        if not cell:
            return None
        root = cell_root(cell)
        return root.name, root

    def resolve_cell(self, cell: Optional[str]) -> tuple[str, Path]:
        """A FactoryOS cell folder name (default: the active cell) -> (name, folder)."""
        active = self.active_cell()
        if not cell:
            if not active:
                raise ToolError("no active cell", "Pick the cell in the Behaviour Tree Executor's Scene, or pass `cell`.")
            return active
        if not valid_name(cell):
            raise ToolError(f"invalid cell name {cell!r}", "Use the cell's folder name, e.g. welding_cell_2_small.")
        if active and active[0] == cell:
            return active
        cells = self.cells_dir()
        if not cells or not (cells / cell).is_dir():
            # A Robo Fitter scene (jam2, or the legacy jam2.yaml) without a cell yet: its cell (a folder for its
            # outputs that references the scene) is made now.
            made = self.cell_for_scene(cell) if cells else None
            if made:
                return made, cells / made
            raise ToolError(f"no cell {cell!r} in the cells folder {cells}", "list_plans / the viewer's Scene list the cells.")
        return cell, cells / cell

    # ---------------------------------------------------------------- the cell view and the gate
    def training_scene(self, root: Path) -> Path:
        """The scene the training cell check compares: a legacy cell's own scene.yaml; a reference cell's Robo Fitter
        scene (the source, scenes/<name>/scene.yaml; not the view's rewrite)."""
        if not is_reference_cell(root):
            return root / "scene.yaml"
        return self.fitted_scene_file(read_manifest(root).get("fitted_scene"))

    @staticmethod
    def source_sha(rel: str, path: Path, root: Path) -> Optional[str]:
        """The sha256 of cell file `rel` (at `path`) for the training cell check. A reference cell fos-migrate-cells
        imported (manifest `imported`) counts its scene as the scene it had while the Robo Fitter scene is still what
        the migration wrote: only the asset paths were rewritten (to the catalogue's packages, which hold the very
        files), so the policies' training cell stays their training cell."""
        got = sha256_file(path)
        if rel == "scene.yaml" and got and is_reference_cell(root):
            imported = read_manifest(root).get("imported")
            if isinstance(imported, dict) and imported.get("scene_sha256") and got == imported.get("fitted_sha256"):
                return imported["scene_sha256"]
        return got

    def cell_sources(self, cell_id: str, root: Path) -> dict[str, Path]:
        """The files a policy reads for `cell_id`, by their path in the cell view (see planners.json)."""
        ucell = self.s.umarell / "cells" / cell_id
        robots = self.robots_dir(root) or ucell / "robots"
        out = {}
        for rel in self.training.get(cell_id, {}):
            if rel == "scene.yaml":
                out[rel] = self.training_scene(root)
            elif rel.startswith("robots/"):
                out[rel] = robots / rel[len("robots/"):]
            else:
                out[rel] = ucell / rel
        return out

    @staticmethod
    def _has_meshes(robots: Path) -> bool:
        return robots.is_dir() and any((d / "meshes").is_dir() for d in robots.iterdir() if d.is_dir())

    def build_view(self, cell_id: str, root: Path, task_file: Optional[Path] = None,
                   key: Optional[str] = None) -> Path:
        """`<work>/view/<key>/cells/<cell_id>`: the umarell checkout's cell (its agentic_planner/: the training
        copy) with the FactoryOS cell's robots/ (the meshes; umarell's archive has only the URDFs). UMARELL_REPO for the
        planner. Every job builds its own (key: its job id; default the cell id), so jobs running at the same time,
        on the same cell too, never swap a view another job plans in; symlinks only, dropped when the job ends."""
        ucell = self.s.umarell / "cells" / cell_id
        key = key or cell_id
        base = self.s.work / "view" / key
        tmp = self.s.work / "view" / f".{key}.{uuid.uuid4().hex[:8]}"
        cdir = tmp / "cells" / cell_id
        cdir.mkdir(parents=True)
        own = not ucell.is_dir() or task_file is not None
        if own:
            ap = cdir / "agentic_planner"
            (ap / "tasks").mkdir(parents=True)
            for entry in sorted((root / "agentic_planner").iterdir()):
                if entry.name not in ("tasks", "plans", "logs", "recordings"):
                    os.symlink(entry, ap / entry.name)
            tf = task_file or self.task_file(root, None)
            if tf:
                os.symlink(tf, ap / "tasks" / f"{self.canonical_task}.json")
            if is_reference_cell(root):
                # A reference cell: its scene and the catalogue's robots / the products, laid out as a legacy cell's
                # (the backend's view .view/: scene.yaml with view-relative paths, robots/, fixtures/, floorplans/).
                for src, rel in self.reference_layout(root):
                    if not (cdir / rel).exists():
                        os.symlink(src, cdir / rel)
            for entry in sorted(root.iterdir()):
                if (entry.name not in ("agentic_planner", "robots") and not entry.name.startswith(VIEW_DIR)
                        and not (cdir / entry.name).exists()):
                    os.symlink(entry, cdir / entry.name)
        robots = self.robots_dir(root)
        for entry in sorted(ucell.iterdir()) if ucell.is_dir() and not own else []:
            if entry.name == "robots" and robots:
                continue
            os.symlink(entry, cdir / entry.name)
        if robots and not (cdir / "robots").exists():
            os.symlink(robots, cdir / "robots")
        # The checkout's other cells: a policy reads its own training cell for reference data (action scaling,
        # the robots' acceleration limits, feature normalisation) while it plans and is checked in this one.
        if ucell.parent.is_dir():
            for other in sorted(ucell.parent.iterdir()):
                if other.is_dir() and other.name != cell_id and not (tmp / "cells" / other.name).exists():
                    os.symlink(other, tmp / "cells" / other.name)
        # Shared asset folders next to the cells (fixture meshes for the planner's visual-only fixture).
        for shared in ("urdf-fixture-collection", "urdf-dressed-robots", "floor-plans"):
            for src in (self.s.umarell / "cells" / shared, root.parent / shared):
                if src.is_dir():
                    os.symlink(src, tmp / "cells" / shared)
                    break
        # Swap in: move the old view aside first (a rename, so a view another job builds at the same time can't
        # leave `base` half-deleted and non-empty under os.replace), then drop it.
        for _ in range(5):
            if base.exists() or base.is_symlink():
                old = base.with_name(f".{base.name}.old.{uuid.uuid4().hex[:8]}")
                try:
                    os.replace(base, old)
                except FileNotFoundError:
                    pass
                else:
                    shutil.rmtree(old, ignore_errors=True)
            try:
                os.replace(tmp, base)
                return base
            except OSError as err:
                if err.errno not in (errno.ENOTEMPTY, errno.EEXIST):
                    raise
        raise ToolError(f"could not install the planner's view of {cell_id} at {base} (another job keeps replacing it)")

    def reference_layout(self, root: Path) -> list:
        """[(source, name in the cell view)] for a reference cell: its view's scene.yaml, robots/, fixtures/ and
        floorplans/ when the backend built one; else the Robo Fitter scene itself as scene.yaml (its paths are
        relative to scenes/, so it is linked by its real path) and robots/ of the scene's robot packages."""
        view = root / VIEW_DIR
        if (view / "scene.yaml").is_file():
            return [(view / n, n) for n in ("scene.yaml", "robots", "fixtures", "floorplans") if (view / n).exists()]
        scene = self.scene_path(root)
        return [(scene.resolve(), "scene.yaml")] if scene.is_file() else []

    def gate(self, planner: dict, cell: str, root: Path) -> tuple[bool, str, list]:
        """(available, why not, warnings) of a policy / scripted planner for the FactoryOS cell `cell`."""
        if planner["kind"] == "llm":
            return True, "", []
        cid = planner_cell_id(cell)
        trained = cid in planner.get("cells", [])
        if not self.s.python.exists() or not (self.s.cell_planner / "tools" / "btexport").is_dir():
            return False, "factoryos-cell-planner is not installed on this host (provision.sh)", []
        if planner["kind"] == "rules":
            return self._gate_jam(planner, cell, root) if planner["name"] == JAM else self._gate_u3(planner, cell, root)
        if planner["name"] == "trajgen" and \
                not (self.s.cell_planner / "trajgen" / "samples" / cid / "expected" / "plan.json").is_file():
            return False, f"trajgen has no sample for {cid} (a live run: options.allow_slow, 20-30 min)", []
        if planner.get("checkpoint") and not (self.s.cell_planner / planner["checkpoint"]).is_file():
            return False, f"checkpoint {planner['checkpoint']} is missing from the cell planner", []
        if not trained:
            # Every planner plans every cell (a policy: one it has a weld IK table for); the export checks judge it.
            if planner["name"] != "trajgen" and not self.task_file(root, None) \
                    and not (self.s.umarell / "cells" / cid / TASKS_DIR).is_dir():
                return False, (f"cell {cell} has no task program (agentic_planner/tasks/*.json, the weld targets): "
                               "write one first (Agentic Planning: Write Task Program)"), []
            table = "weld_ik.json" if cid == "welding_cell_2_small" else f"weld_ik_{cid}.json"
            if planner["kind"] == "policy" and not (self.s.cell_planner / "data" / table).is_file():
                # The policies' env reads which robot reaches which weld from it (rl/hier/instances.table_path):
                # without one no weld has an IK branch and the rollout crashes (IndexError) instead of planning.
                return False, (f"{planner['name']} needs the weld IK table data/{table}, which only its training "
                               f"cell(s) {', '.join(planner.get('cells', []))} have; umarell_u3 or umarell plan any "
                               "cell"), []
            return True, "", [f"cell {cell} is outside {planner['name']}'s training cell(s) "
                              f"{', '.join(planner.get('cells', []))}: the export checks decide"]
        want = self.training.get(cid)
        if not want:
            return False, f"no training checksums pinned for {cid} (planners.json)", []
        if not (self.s.umarell / "cells" / cid / "agentic_planner").is_dir():
            return False, f"the umarell checkout {self.s.umarell} has no cell {cid}", []
        bad = []
        for rel, path in self.cell_sources(cid, root).items():
            got = self.source_sha(rel, path, root)
            if got != want[rel]:
                bad.append(f"{rel} ({'missing' if got is None else 'changed'})")
        warnings = []
        if bad:
            warnings.append(f"cell {cell} differs from the cell the policies were trained on: {', '.join(bad)}; "
                            "they plan the cell's own prepared model (Prepare Cell) and task program, outside their "
                            "training: the export checks decide")
            return True, "", warnings
        prepared = root / "agentic_planner" / "config" / "robots.yaml"
        trained = want.get("agentic_planner/config/robots.yaml")
        if prepared.is_file() and sha256_file(prepared) != trained:
            warnings.append("the cell's own agentic_planner/config/robots.yaml (Prepare Cell) differs from the training "
                            "copy; with the canonical program the policies use the training copy, whose scene.yaml "
                            "and robots match this cell (a task program of the cell's own: its prepared model)")
        return True, "", warnings

    def training_cell_unchanged(self, cid: str, root: Path) -> bool:
        """Every file the policies were trained on is unchanged in the cell view (planners.json training_cells)."""
        want = self.training.get(cid)
        return bool(want) and all(self.source_sha(rel, path, root) == want[rel]
                                  for rel, path in self.cell_sources(cid, root).items())

    def u3_own_cell(self, cid: str, root: Path) -> bool:
        """U3 plans the FactoryOS cell itself (its prepared agentic_planner/) rather than umarell's patched copy of the
        cell: umarell has no such cell, or the FactoryOS cell's layout (scene.yaml) is not umarell's."""
        if not (self.s.umarell / "cells" / cid / "agentic_planner").is_dir():
            return True
        want = self.training.get(cid, {}).get("scene.yaml")
        return want is not None and self.source_sha("scene.yaml", self.training_scene(root), root) != want

    def _gate_u3(self, planner: dict, cell: str, root: Path) -> tuple[bool, str, list]:
        cid = planner_cell_id(cell)
        if not (self.s.cell_planner / "umarell_fast" / "u3").is_dir():
            return False, f"the installed cell planner ({self.commit}) has no umarell_fast/u3", []
        if not self.s.umarell_python or not self.s.umarell_python.exists():
            return False, f"umarell's venv {self.s.umarell_python} is missing (provision.sh)", []
        if not shutil.which("patch") or not shutil.which("rsync"):
            return False, "U3 needs the patch and rsync programs (provision.sh installs them)", []
        warnings = []
        if self.u3_own_cell(cid, root):
            if not self.task_file(root, None) and not (self.s.umarell / "cells" / cid / TASKS_DIR).is_dir():
                return False, (f"cell {cell} has no task program (agentic_planner/tasks/*.json, the weld targets): "
                               "write one first (Agentic Planning: Write Task Program)"), []
            what = "umarell has no such cell" if not (self.s.umarell / "cells" / cid).is_dir() \
                else "its layout (scene.yaml) differs from umarell's"
            warnings.append(f"U3 plans cell {cell} from its own prepared agentic_planner/ ({what}): the upstream code "
                            f"fixes apply, the tool spheres / hull margin of the patched cells "
                            f"{', '.join(planner.get('cells', []))} do not; the export checks decide")
        return True, "", warnings

    def _gate_jam(self, planner: dict, cell: str, root: Path) -> tuple[bool, str, list]:
        if not (HERE / "umarell_jam.py").is_file():
            return False, "this router has no umarell_jam.py", []
        if not (self.s.cell_planner / "umarell_jam" / "__main__.py").is_file():
            return False, f"the installed cell planner ({self.commit}) has no umarell_jam", []
        py = self.jam_python()
        if not py.exists():
            return False, f"umarell_jam's python {py} is missing (provision-cell-planner.sh: jam-venv)", []
        if not self.task_file(root, None):
            return False, (f"cell {cell} has no task program of its own (agentic_planner/tasks/*.json, the weld "
                           "targets): umarell_jam plans the cell's own programs; write one first (Agentic Planning: "
                           "Write Task Program)"), []
        import umarell_jam  # noqa: E402 - next to this file (checked above)
        warnings = []
        missing = self._jam_runtime(py)
        if missing:
            warnings.append(f"umarell_jam's numerical runtime is incomplete in {py} ({missing}): it needs the "
                            "session's stack (pin 2.7.0, hpp-fcl 2.4.4, numpy 1.26.4, toppra 0.6.3; its cache keys "
                            "depend on it), so it will fail to set up (then, with options.fallback, umarell_u3 plans)")
        if umarell_jam.jam_source(self.s.umarell.resolve()) is None:
            bases = " / ".join(umarell_jam.JAM_BASES)
            warnings.append(f"no plain umarell {bases} on this host ({self.s.umarell}-<{bases}>: "
                            "deploy/provision.sh installs it from deploy/umarell-jam.pin's "
                            "commit, or the installer's --umarell-jam-tarball, on a public install; from "
                            "deploy/umarell-jam-base.txt's archive on the internal servers): umarell_jam may fail to "
                            "set up (then, with options.fallback, umarell_u3 plans the job)")
        if not umarell_jam.cache_entries(self.jam_root()):
            warnings.append("umarell_jam's component cache on this workspace is empty (not prepared, and nothing "
                            "pulled from the catalogue yet): every weld is a miss, repaired within the repair budget "
                            "or planned by the umarell_u3 fallback")
        return True, "", warnings

    def jam_python(self) -> Path:
        """umarell_jam's python: FACTORY_OS_JAM_PYTHON, else its own venv (provision-cell-planner.sh's jam-venv, next to
        the cell planner's src/), else umarell's venv (which lacks the session's numerical stack: the gate warns)."""
        over = os.environ.get("FACTORY_OS_JAM_PYTHON", "").strip()
        if over:
            return Path(over)
        venv = self.s.cell_planner.parent / "jam-venv"
        if (venv / "bin" / "python").exists():
            return venv / "bin" / "python"
        return Path(self.s.umarell_python or "/nonexistent")

    def _jam_runtime(self, py: Path) -> str:
        """'' when ``import hppfcl, toppra`` works in ``py``, else what failed (cached per interpreter and mtime)."""
        try:
            key = (str(py), py.stat().st_mtime_ns)
        except OSError:
            return "no interpreter"
        cache = self.__dict__.setdefault("_jam_runtime_cache", {})
        if key not in cache:
            try:
                r = subprocess.run([str(py), "-c", "import hppfcl, toppra"], capture_output=True, text=True, timeout=60,
                                   stdin=subprocess.DEVNULL)
                cache[key] = "" if r.returncode == 0 else \
                    ((r.stderr.strip().splitlines() or [f"exit {r.returncode}"])[-1][:200])
            except (OSError, subprocess.TimeoutExpired) as exc:
                cache[key] = f"{type(exc).__name__}: {exc}"
        return cache[key]

    def jam_root(self) -> Path:
        """umarell_jam's writable root (its component cache, patched umarell copy, runs): kept across jobs."""
        return Path(os.environ.get("FACTORY_OS_JAM_ROOT") or self.s.work / "umarell_jam")

    def data_root(self, root: Path) -> Path:
        """The FactoryOS data root umarell_jam reads (cells/, catalogue/, products/): FACTORY_OS_JAM_WORKSPACE, else
        the cells folder's parent (/var/lib/factoryos on a workspace)."""
        over = os.environ.get("FACTORY_OS_JAM_WORKSPACE", "").strip()
        if over:
            return Path(over)
        cells = self.cells_dir()
        return (cells or root.parent).resolve().parent

    @staticmethod
    def task_file(root: Path, task: Optional[str]) -> Optional[Path]:
        """The cell's task program `task` (agentic_planner/tasks/<task>.json), else its only / newest one."""
        tasks = root / TASKS_DIR
        task = (task or "").strip()
        if task and task != "(none yet)" and valid_name(task):
            f = tasks / f"{task}.json"
            return f if f.is_file() else None
        files = sorted(tasks.glob("*.json"), key=lambda f: f.stat().st_mtime) if tasks.is_dir() else []
        return files[-1] if files else None

    def is_canonical(self, task: Optional[str], root: Path, cid: str) -> bool:
        """The task program the policies plan: none picked, or the canonical program unchanged."""
        task = (task or "").strip()
        if task in ("", "(none yet)"):
            return True
        if not valid_name(task):
            return False
        f = root / TASKS_DIR / f"{task}.json"
        if f.is_file():
            want = self.training.get(cid, {}).get(f"{TASKS_DIR}/{self.canonical_task}.json")
            return want is not None and sha256_file(f) == want
        return task == self.canonical_task

    # ---------------------------------------------------------------- tools: catalogue
    def _entry(self, p: dict, cell: Optional[tuple[str, Path]], full: bool = False) -> dict:
        e = {k: p.get(k) for k in ("name", "kind", "title", "cells", "robots", "fixture", "typical_s", "verified",
                                   "notes")}
        if cell:
            ok, why, warnings = self.gate(p, *cell)
            e["available"] = ok
            e["why_unavailable"] = why or None
            if warnings:
                e["warnings"] = warnings
        if full:
            e.update(spec=p.get("from"), checkpoint=p.get("checkpoint") or None, default_profile=p.get("profile"),
                     flags=p.get("flags", []), timeout_s=p.get("timeout_s"),
                     cell_planner_commit=self.commit if p["kind"] != "llm" else None,
                     limitations=self._limitations(p))
        return e

    @staticmethod
    def _limitations(p: dict) -> list:
        if p["kind"] == "llm":
            return ["takes minutes; calls Bedrock", "runs in the umarell planner (Agentic Planning's simulation)"]
        if p["name"] == JAM:
            return ["no free-text constraints (the cell's own task program as is, or options.welds / options.robots "
                    "subsets)",
                    "exported in the umarell profile only (planned and timed at the cell limits)",
                    "seconds only from a prepared component cache (python -m umarell_jam prepare, offline); a miss is "
                    "repaired within options.repair_budget_s, else no plan (exit 3) or, with options.fallback (the "
                    "default), umarell_u3 plans the job: minutes",
                    f"validated in {', '.join(p.get('cells', []))} (umarell's cells with the tool spheres); "
                    "other cells: the export checks decide"]
        if p["kind"] == "rules":
            return ["no free-text constraints (the task program as is; use umarell for constraints)",
                    "takes minutes on this host (umarell's OMPL, one worker per CPU)",
                    f"validated in {', '.join(p.get('cells', []))} (umarell's cells with the tool spheres); "
                    "other cells: the export checks decide (btexport checks every arm pair; its --validate is two-arm only)"]
        out = ["plans the canonical car_door_spot_welding program only (ignores another task program)",
               f"only for the cell(s) it was trained on: {', '.join(p.get('cells', []))} (checksums in planners.json)"]
        if p["kind"] == "policy":
            out.append("never beats umarell's best plan at matched limits")
        return out

    def _cell_or_none(self, cell: Optional[str]) -> Optional[tuple[str, Path]]:
        try:
            return self.resolve_cell(cell)
        except ToolError:
            if cell:
                raise
            return None

    def list_planners(self, cell: Optional[str] = None) -> dict:
        c = self._cell_or_none(cell)
        return {"cell": c[0] if c else None, "planners": [self._entry(p, c) for p in self.catalog["planners"]]}

    def describe_planner(self, name: str, cell: Optional[str] = None) -> dict:
        p = self.planners.get(name or "")
        if not p:
            raise ToolError(f"unknown planner {name!r}", "list_planners names them: " + ", ".join(self.planners))
        return self._entry(p, self._cell_or_none(cell), full=True)

    # ---------------------------------------------------------------- tools: routing
    _UMARELL_WORDS = re.compile(r"\b(llm|umarell|agentic|explain|repair|why|constraint|constraints|avoid|only|subset|"
                                r"except|without|order|sequence|priority|custom)\b", re.I)
    _POLICY_WORDS = re.compile(r"\b(policy|policies|learned|rl|reinforcement|fast|fastest|quick|quickly)\b", re.I)
    _FILLER_WORDS = re.compile(r"\b(please|plan|planning|planner|the|this|that|my|a|an|it|cell|with|using|use|via|"
                               r"by|for|me|now|run|go|auto|default|best|recommended|ok|okay|and)\b", re.I)
    _AUTO_WORDS = re.compile(r"^\s*(plan( the| this)?( cell)?|auto|default|best|recommended)?\s*[.!]?\s*$", re.I)

    def recommend_planner(self, cell: Optional[str] = None, task: Optional[str] = None,
                          request: Optional[str] = None, planner: Optional[str] = None,
                          profile: Optional[str] = None) -> dict:
        name, root = self.resolve_cell(cell)
        cid = planner_cell_id(name)
        canonical = self.is_canonical(task, root, cid)
        gates = {p: self.gate(self.planners[p], name, root) for p in self.planners}
        avail = {p for p, (ok, _, _) in gates.items() if ok}
        explicit = planner if planner and planner != "auto" else None
        text = (request or "").strip()
        if not explicit and text:
            named = [p for p in self.planners if re.search(rf"\b{re.escape(p)}\b", text, re.I)]
            if len(named) == 1:
                explicit = named[0]
        out = {"cell": name, "cell_id": cid, "task": (task or "").strip() or None, "canonical_task": canonical,
               "candidates": [], "recommended": None, "ambiguous": False}

        def cand(p, score, reason):
            entry = {"name": p, "score": score, "reason": reason, "available": p in avail}
            v = self.planners[p].get("verified") or {}
            if v.get("end_s"):
                entry["verified_end_s"] = v["end_s"]
                entry["verified_profile"] = v.get("profile")
            if p not in avail:
                entry["why_unavailable"] = gates[p][1]
            out["candidates"].append(entry)

        policies = [p for p in POLICY_ORDER if p in avail]
        u3 = U3 if U3 in avail else None
        # the canonical program in a policy's own, unchanged training cell: what the policies are verified for
        fit = canonical and self.training_cell_unchanged(cid, root) \
            and any(cid in self.planners[p].get("cells", []) for p in policies)
        if explicit:                                                         # rule 1
            if explicit not in self.planners:
                raise ToolError(f"unknown planner {explicit!r}", "list_planners names them: " + ", ".join(self.planners))
            if explicit in avail:
                cand(explicit, 1.0, "asked for explicitly")
                out["recommended"] = explicit
                out["reason"] = "the planner you asked for"
                return out
            why = gates[explicit][1]
            cand(explicit, 0.0, why)
            alt = policies[0] if policies else u3 or "umarell"
            cand(alt, 0.9, "available for this cell and task")
            out["ambiguous"] = True
            out["question"] = f"{explicit} can't plan this: {why}. Use {alt} instead?"
            return out
        intent = ""
        if text:
            intent = "umarell" if self._UMARELL_WORDS.search(text) else "policy" if self._POLICY_WORDS.search(text) \
                else "" if self._AUTO_WORDS.match(text) else "free"
        u3_reason = "umarell's own planning tools with the upstream fixes, deterministic (no LLM): any task program"
        if not policies:                                                     # rule 5
            why = "no learned policy fits this cell" + (f" ({gates['hier_la_fu'][1]})" if "hier_la_fu" in gates else "")
            if u3 and intent != "umarell":
                cand(u3, 1.0, f"{why}; {u3_reason}")
                cand("umarell", 0.6, "LLM planner: free-text constraints, takes minutes, calls Bedrock")
            else:
                cand("umarell", 1.0, why)
            for p in POLICY_ORDER:
                if p in self.planners and self.planners[p].get("cells") and cid in self.planners[p]["cells"]:
                    cand(p, 0.0, gates[p][1])
            out["recommended"] = u3 if u3 and intent != "umarell" else "umarell"
            out["reason"] = "the cell is out of the policies' training distribution"
            return out
        if intent == "umarell":
            cand("umarell", 1.0, "your request asks for constraints / explanations only the LLM planner handles (as operations / constraints)")
            cand(policies[0], 0.6, "the best learned policy, if the canonical program as is will do")
            if u3:
                cand(u3, 0.5, f"{u3_reason}, if the task program as is will do")
            out["recommended"] = "umarell"
            out["reason"] = "free-text constraints"
            return out
        if u3 and not fit:                                                   # outside the policies' training
            cand(u3, 1.0, f"the task program or the cell is outside the learned policies' training; {u3_reason}")
            for i, p in enumerate(policies):
                cand(p, round(0.8 - 0.1 * i, 2), "learned policy outside its training: the export checks decide")
            cand("umarell", 0.3, "LLM planner: any task and constraints, takes minutes")
            out["recommended"] = u3
            out["reason"] = "outside the learned policies' training: U3 plans the task program itself"
            if intent == "free":
                out["ambiguous"] = True
                out["question"] = (f"Your request has free text {u3} ignores. Plan with {u3} (the task program as is, "
                                   f"{self._verified(u3)}) or with umarell (applies your wish as task operations / "
                                   "constraints, takes minutes)?")
            return out
        order = list(policies)
        if profile == "full" and "hier_q" in order:                          # hier_q: verified in the full profile
            order.remove("hier_q")
            order.insert(0, "hier_q")
        reasons = {
            "hier_la_fu": "learned policy; the only one that passes every check in the umarell profile",
            "hier_q": "learned policy; fastest (full profile), unverified in the umarell profile",
            "ppo_pure": "learned PPO policy; slower, largest clearance",
            "ppo_v3": "learned PPO policy; slower",
            "ppo_v4": "learned PPO policy; slowest",
            "ppo_2t": "learned PPO policy for the fixture cell",
        }
        for i, p in enumerate(order):
            cand(p, round(1.0 - 0.1 * i, 2), reasons.get(p, "learned policy"))
        if u3:
            cand(u3, 0.5, f"{u3_reason}; takes minutes on this host")
        if "trajgen" in avail:
            cand("trajgen", 0.4, "scripted generator's precomputed sample (instant)")
        cand("umarell", 0.3, "LLM planner: any task and constraints, takes minutes")
        out["recommended"] = order[0]
        out["reason"] = "canonical program in the training cell: a learned policy plans it in under a minute"
        if intent == "free":                                                 # rule 6: a policy and umarell both fit
            out["ambiguous"] = True
            out["question"] = (
                f"Your request has free text a learned policy ignores. Plan with {order[0]} "
                f"(canonical program, {self._verified(order[0])}) or with umarell (applies your wish as task operations / constraints, takes minutes)?")
        return out

    def _has_wish(self, request: Optional[str]) -> bool:
        """Does the request carry a wish beyond 'plan (the cell) with <planner>'?"""
        text = (request or "").strip()
        if not text or self._AUTO_WORDS.match(text):
            return False
        for p in self.planners:
            text = re.sub(rf"\b{re.escape(p)}\b", " ", text, flags=re.I)
        text = self._FILLER_WORDS.sub(" ", text)
        return bool(re.sub(r"[\W_]+", "", text))

    def _verified(self, p: str) -> str:
        v = self.planners[p].get("verified") or {}
        if not v.get("end_s"):
            return "not verified here"
        return f"{v['end_s']} s, {v.get('profile')} profile" + (", all checks pass" if v.get("export_pass") else "")

    # ---------------------------------------------------------------- tools: planning
    def plan(self, planner: str = "auto", cell: Optional[str] = None, task: Optional[str] = None,
             profile: Optional[str] = None, request: Optional[str] = None, options: Optional[dict] = None) -> dict:
        # Any cell: the job reads that cell's folder and publishes into its plans (not necessarily the active cell's,
        # which the viewer's Behaviour Tree step shows). Jobs run in parallel (FACTORYOS_PLANNER_WORKERS).
        options = options or {}
        asked_profile = profile
        name, root = self.resolve_cell(cell)
        cid = planner_cell_id(name)
        if not planner or planner == "auto":
            rec = self.recommend_planner(name, task, request, None, profile)
            if rec["ambiguous"] or not rec["recommended"]:
                raise ToolError("ambiguous: ask the user which planner to use", rec.get("question", ""),
                                candidates=rec["candidates"])
            planner = rec["recommended"]
        p = self.planners.get(planner)
        if not p:
            raise ToolError(f"unknown planner {planner!r}", "list_planners names them: " + ", ".join(self.planners))
        if options.get("speed") is not None and options["speed"] not in U3_SPEEDS:
            raise ToolError(f"unknown options.speed {options['speed']!r}", "max (the default) or normal")
        if options.get("speed") is not None and p["kind"] != "rules":
            raise ToolError(f"options.speed is umarell_u3's and umarell_jam's, not {planner}'s",
                            "for a policy / trajgen plan at the URDF limits use profile full")
        if options.get("remote") is not None and not isinstance(options["remote"], bool):
            raise ToolError(f"options.remote is true or false, not {options['remote']!r}",
                            "true: plan on a planner-burst instance; false: on this workspace; leave it out: auto")
        if options.get("remote") is not None and p["name"] != U3:
            raise ToolError(f"options.remote is umarell_u3's, not {planner}'s", "the other planners run here")
        jam_opts = self._jam_options(p, options)
        if p["kind"] == "llm":
            if self._has_wish(request) and not (options.get("operations") or options.get("constraints")
                                                 or options.get("request_applied")):
                # umarell's plan_trajectories takes operations + constraints only: free text would be dropped.
                raise ToolError(
                    "umarell can't take the request as free text: it would plan the whole task program and ignore it",
                    "Turn the wish into options.operations (the task program's operations, filtered / reordered as "
                    "asked) and/or options.constraints {max_parallel, trajectory_backend}, or save a task program "
                    "with save_task_program and plan that (then options.request_applied=true). If the wish can't be "
                    "expressed that way, tell the user before planning.", request=request)
            return self._plan_umarell(name, root, cid, task, options)
        ok, why, _ = self.gate(p, name, root)
        if not ok and p["name"] == "trajgen" and options.get("allow_slow") and why.startswith("trajgen has no sample"):
            ok = True                                    # a live run needs no sample
        if not ok:
            raise ToolError(f"{planner} is not available for cell {name}: {why}",
                            "recommend_planner lists what fits; umarell plans any cell")
        trained = cid in p.get("cells", [])
        # A policy's training cell that changed (its scene, e.g. the release's reference welding_cell_2_small with its
        # fixture, or its robots) is planned and checked in the cell's own view too: umarell's training copy (its
        # agentic_planner/: obstacles, program) is no longer this cell. U3 / umarell_jam decide that themselves.
        changed = trained and p["kind"] != "rules" and not self.training_cell_unchanged(cid, root)
        if not (self.s.umarell / "cells" / cid).is_dir() or is_reference_cell(root) or changed:
            # A cell the checkout lacks: the policies read its own prepared agentic_planner/config. A reference cell
            # also needs its view (its scene and the catalogue's robots), which preparing brings up to date.
            self._ensure_prepared(name, root)
        task_file = None
        if not trained or not self.is_canonical(task, root, cid) or changed:
            # Every planner plans every task program: the cell's own, as the planner's task (a changed training cell
            # without one of its own: the canonical program, planned against the cell's own prepared model).
            task_file = self.task_file(root, task)
            canon = self.s.umarell / "cells" / cid / TASKS_DIR / f"{self.canonical_task}.json"
            if task_file is None and changed and self.is_canonical(task, root, cid) and canon.is_file():
                task_file = canon
            if task_file is None and p["name"] != "trajgen":
                raise ToolError(f"cell {name} has no task program {task or ''}".rstrip(),
                                "save_task_program (describe_cell for the fixtures) or Write Task Program")
        profile = profile or p.get("profile", "umarell")
        if profile not in ("umarell", "full"):
            raise ToolError(f"unknown profile {profile!r}", "umarell or full")
        speed = options.get("speed")
        if p["name"] == JAM:
            # umarell_jam plans at the cell limits whatever the speed (recorded), and its per-robot-sequence trees
            # export in the umarell profile only (btexport's full profile refuses them; they are already timed at the
            # cell limits)
            speed = "normal" if speed == "default" else (speed or U3_DEFAULT_SPEED)
            if asked_profile == "full":
                raise ToolError("umarell_jam's plans are exported in the umarell profile only",
                                "drop profile: they are planned and timed at the cell limits already (btexport's "
                                "full profile refuses their per-robot-sequence trees)")
            profile = "umarell"
        elif p["kind"] == "rules":
            # max (the default): U3 plans at the robots' limits and btexport times to them (full). U3's own timing of
            # a max-speed plan is not dynamically valid where umarell has no TOPP-RA (aarch64: its Hermite fallback,
            # measured 7 x a_max on body_weld_3), so max is never exported in the umarell profile; asking for that
            # profile (or speed normal) gives the normal-speed plan.
            speed = "normal" if speed == "default" else speed
            speed = speed or ("normal" if asked_profile == "umarell" else U3_DEFAULT_SPEED)
            if speed == "max" and asked_profile == "umarell":
                raise ToolError("options.speed max is exported in the full profile only",
                                "drop profile (or pass profile full), or options.speed normal for the slower plan in "
                                "the umarell profile: umarell's own timing of a max-speed plan breaks "
                                "the acceleration limits on hosts without TOPP-RA")
            if speed == "max":
                profile = "full"
            elif not asked_profile:
                profile = "umarell"                      # normal: the plan in U3's own timing, as before
        spec, timeout_s, eta_s = p["from"], float(p.get("timeout_s", 600)), float(p.get("typical_s", 60))
        if p["name"] == "trajgen":
            if options.get("allow_slow"):
                spec, timeout_s, eta_s = p["from_live"], float(p["live_timeout_s"]), float(p["live_typical_s"])
            elif profile == "full":
                spec = p["from_full"]
            spec = spec.replace("{cell_id}", cid)
        u3_own = None
        if p["kind"] == "rules" and p["name"] != JAM:
            u3_own = self.u3_own_cell(cid, root)
            if u3_own:
                self._ensure_prepared(name, root)
                if task_file is None:
                    canon = self.s.umarell / "cells" / cid / TASKS_DIR / f"{self.canonical_task}.json"
                    task_file = self.task_file(root, task) or (canon if canon.is_file() else None)
                if task_file is None:
                    raise ToolError(f"cell {name} has no task program {task or ''}".rstrip(),
                                    "save_task_program (describe_cell for the fixtures) or Write Task Program")
        remote = self._remote_choice(options) if p["name"] == U3 else None
        jam_task = None
        if p["name"] == JAM:
            # umarell_jam plans the FactoryOS cell itself (the data root's cells/<cell>: its prepared agentic_planner/)
            # and only the cell's own task programs (its component keys follow them; never umarell's copy of the
            # canonical program). The job's view is the cell's own too, as btexport checks the plan in it.
            self._ensure_prepared(name, root)
            jam_task = task_file or self.task_file(root, task)
            if jam_task is None:
                raise ToolError(f"cell {name} has no task program {task or ''}".rstrip(),
                                "umarell_jam plans the cell's own task programs (agentic_planner/tasks): "
                                "save_task_program (describe_cell for the fixtures) or Write Task Program")
            task_file = jam_task
            jam_opts["fallback"] = self._jam_fallback(p, name, root, options, jam_opts)
            if jam_opts["fallback"]["enabled"]:
                u3_own = self.u3_own_cell(cid, root)
            timeout_s = jam_opts["timeout_s"] + JAM_MARGIN_S
        with self.cv:
            tree_id = self._reserve_tree_id(root, cid)
            jdir = self.s.work / "jobs" / tree_id
            out = jdir / f"{tree_id}.json"
            cmd = [str(self.s.python), "-m", "tools.btexport", "--from", spec, *p.get("flags", []),
                   "--profile", profile, "--cell", cid, "--tree-id", tree_id, "--out", str(out),
                   "--work", str(jdir / "tg"), "--report", str(jdir / f"{tree_id}.report.json"),
                   "--repo", str(jdir / "umarell")]
            if p["name"] == JAM and jam_opts["welds"]:
                # A requested weld subset is complete by request: btexport's `complete` check (every weld of the
                # task) failed it and the plan was published unverified although every other check passed. The
                # report keeps the missing welds and allow_partial: true.
                cmd.append("--allow-partial")
            if p["name"] == JAM:
                # umarell_jam.py plans with umarell_jam in umarell's venv (with the fallback: umarell_u3.py's U3),
                # then runs this btexport line on the plan
                fb = jam_opts["fallback"]
                cmd = [sys.executable, "-I", str(HERE / "umarell_jam.py"), "--cell-planner", str(self.s.cell_planner),
                       "--umarell", str(self.s.umarell), "--python", str(self.jam_python()),
                       "--root", str(self.jam_root()), "--workspace", str(self.data_root(root)),
                       "--cell-id", name, "--cell-dir", str(root), "--task-file", str(jam_task),
                       "--speed", "max" if speed == "max" else "normal",
                       *(["--welds", ",".join(jam_opts["welds"])] if jam_opts["welds"] else []),
                       *(["--robots", ",".join(jam_opts["robots"])] if jam_opts["robots"] else []),
                       "--timeout", f"{jam_opts['timeout_s']:g}",
                       *(["--repair-budget", f"{jam_opts['repair_budget_s']:g}"]
                         if jam_opts["repair_budget_s"] is not None else []),
                       *(["--prepare-budget", f"{jam_opts['prepare_budget_s']:g}"]
                         if jam_opts["prepare_budget_s"] is not None else []),
                       "--progress-timeout", f"{jam_opts['progress_timeout_s']:g}",
                       *(["--fallback-u3", "--u3-root", str(self.s.work / "umarell_u3"),
                          "--u3-python", str(self.s.umarell_python), "--u3-cell-id", cid,
                          *(["--u3-own-cell"] if u3_own else [])] if fb["enabled"] else []),
                       "--job-dir", str(jdir), "--", *cmd]
            elif u3_own is not None:
                # umarell_u3.py plans with U3 in umarell's venv, then runs this btexport line on U3's plan
                cmd = [sys.executable, "-I", str(HERE / "umarell_u3.py"), "--cell-planner", str(self.s.cell_planner),
                       "--umarell", str(self.s.umarell), "--python", str(self.s.umarell_python),
                       "--root", str(self.s.work / "umarell_u3"), "--cell-id", cid, "--cell-dir", str(root),
                       *(["--own-cell"] if u3_own else []),
                       *(["--task-file", str(task_file)] if task_file else []),
                       *(["--speed", speed] if speed == "max" else []),
                       "--job-dir", str(jdir), "--", *cmd]
            job = Job(job_id=tree_id, tree_id=tree_id, planner=p["name"], cell=name, cell_id=cid, cell_dir=root,
                      profile=profile, spec=spec, cmd=cmd, timeout_s=timeout_s, eta_s=eta_s, dir=jdir,
                      task=task_file.stem if task_file else self.canonical_task, task_file=task_file,
                      speed=speed if p["kind"] == "rules" else None)
            if remote is not None:
                self._u3_times(job, p, remote)
            if p["name"] == JAM:
                job.jam = {"timeout_s": jam_opts["timeout_s"], "fallback": {**jam_opts["fallback"], "used": False},
                           **{k: jam_opts[k] for k in ("welds", "robots", "repair_budget_s", "prepare_budget_s")
                              if jam_opts[k] is not None}}
                self._jam_times(job)
            self.jobs[tree_id] = job
            out = self._started(job)
            self.cv.notify_all()
        if job.remote:
            threading.Thread(target=self._remote_job, args=(job,), name=f"burst-{tree_id}", daemon=True).start()
        return out

    _JAM_ID = re.compile(r"^[A-Za-z0-9_.:-]+$")

    def _jam_options(self, p: dict, options: dict) -> dict:
        """umarell_jam's options checked, with planners.json's defaults; {} for another planner (which gets none)."""
        if p["name"] != JAM:
            for k in JAM_OPTIONS:
                if options.get(k) is not None:
                    raise ToolError(f"options.{k} is umarell_jam's, not {p['name']}'s",
                                    "plan with planner umarell_jam for weld / robot subsets and its budgets")
            return {}
        out: dict = {}
        for k in ("welds", "robots"):
            v = options.get(k)
            if isinstance(v, str):
                v = [x.strip() for x in v.split(",") if x.strip()]
            if v is not None and (not isinstance(v, list) or not v
                                  or not all(isinstance(x, str) and self._JAM_ID.match(x) for x in v)):
                raise ToolError(f"options.{k} is a non-empty list of {k[:-1]} ids, not {options.get(k)!r}",
                                "the task program's weld ids (describe_cell) / the cell's robot names")
            out[k] = list(dict.fromkeys(v)) if v else None

        def seconds(k: str, default, minimum: float = 0.0, maximum: float = U3_TIMEOUT_MAX_S):
            v = options.get(k)
            if v is None:
                return default
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not minimum <= v <= maximum:
                raise ToolError(f"options.{k} is a number of seconds in [{minimum:g}, {maximum:g}], not {v!r}",
                                f"leave it out for planners.json's default ({default})")
            return float(v)

        out["repair_budget_s"] = seconds("repair_budget_s", p.get("repair_budget_s"))
        out["prepare_budget_s"] = seconds("prepare_budget_s", p.get("prepare_budget_s"))
        need = (out["repair_budget_s"] or 0.0) + (out["prepare_budget_s"] or 0.0) + 120.0
        out["timeout_s"] = seconds("timeout_s", max(float(p.get("plan_timeout_s") or 600), need), 1.0)
        out["progress_timeout_s"] = float(p.get("progress_timeout_s") or 300)
        if options.get("fallback") is not None and not isinstance(options["fallback"], bool):
            raise ToolError(f"options.fallback is true or false, not {options['fallback']!r}",
                            "true (the default): umarell_u3 plans the job when umarell_jam has no plan; false: "
                            "no plan then (exit 3)")
        return out

    def _jam_fallback(self, p: dict, name: str, root: Path, options: dict, jam: dict) -> dict:
        """Whether a umarell_jam job falls back to umarell_u3 when it has no plan: {"enabled", "why"}."""
        asked = options.get("fallback")
        if asked is False or (asked is None and not p.get("fallback", True)):
            return {"enabled": False, "why": "options.fallback false" if asked is False
                    else "planners.json's umarell_jam fallback is off"}
        why = None
        if jam.get("welds") or jam.get("robots"):
            why = "umarell_u3 plans the whole task program with every robot, not an options.welds / robots subset"
        elif U3 not in self.planners:
            why = "no umarell_u3 in planners.json"
        else:
            ok, gate_why, _ = self.gate(self.planners[U3], name, root)
            if not ok:
                why = f"umarell_u3 is not available: {gate_why}"
        if why and asked:
            raise ToolError(f"options.fallback true, but {why}",
                            "drop options.welds / options.robots, or pass options.fallback false")
        if why:
            return {"enabled": False, "why": why}
        return {"enabled": True, "why": "options.fallback true" if asked else "the default"}

    def _jam_times(self, job: Job, workers: Optional[int] = None) -> None:
        """(Under the lock.) A umarell_jam job's timeout: umarell_jam's own + JAM_MARGIN_S, and with the fallback on
        umarell_u3's for the U3 workers it gets (at plan: as if it started now)."""
        job.timeout_s = job.jam["timeout_s"] + JAM_MARGIN_S
        if job.jam["fallback"]["enabled"]:
            ref, cap, _ = self._u3_ref(U3)
            job.u3_workers = workers or self._u3_workers_now(starting=True)
            job.timeout_s += u3_timeout(float(self.planners[U3].get("timeout_s", 4800)), job.u3_workers, ref, cap)

    def _jam_fell_back(self, job: Job) -> None:
        """(At run.) umarell_jam.py's log says it falls back to umarell_u3: the phase, and U3's ETA from now."""
        fb = job.jam.get("fallback") or {}
        if not fb.get("enabled") or fb.get("used"):
            return
        try:
            text = (job.dir / "btexport.log").read_text(errors="replace")
        except OSError:
            return
        if JAM_FALLBACK_MARK not in text:
            return
        with self.lock:
            fb["used"] = True
            job.phase = "fallback: umarell_u3"
            job.eta_s = self._elapsed(job) + u3_eta(float(self.planners[U3].get("typical_s", 1600)),
                                                    job.u3_workers or 1, self._u3_ref(U3)[0])

    def _started(self, job: Job) -> dict:
        """plan's answer for a new job (under the lock)."""
        st = self._queue_info(job)
        if job.kind == "btexport":
            # it starts now if a worker is free and nothing queued before it can run first
            free = st["workers"]["planner"] - len(st["running"]) - len(st["queued_behind"])
            # (a job queued before it that it takes turns with counts, too: a worker may not have taken that one yet)
            status = "planning" if free > 0 and not self._blocked(job, ahead=True) else "queued"
        else:
            status = job.status
        if job.remote:
            status = job.status if job.status != "queued" or len(st["running"]) >= self._cap("remote") else "planning"
        out = {"job_id": job.job_id, "tree_id": job.tree_id or None, "planner": job.planner, "cell": job.cell,
               "profile": job.profile or None, **({"speed": job.speed} if job.speed else {}), "status": status, **st,
               **self._timing(job),
               "poll": "plan_status(job_id) until status is done / failed / cancelled; jobs lists every job"}
        active = self.active_cell()
        if active and active[0] != job.cell:
            out["note"] = (f"cell {job.cell} is not the active cell {active[0]}: the plan is saved in {job.cell}'s "
                           "plans (the Behaviour Tree step shows the active cell's)")
        return out

    def _reserve_tree_id(self, root: Path, cid: str) -> str:
        t = int(time.time())
        while True:
            tid = f"bt_{t}_{cid}"
            if tid not in self.reserved and not (root / PLANS_DIR / f"{tid}.json").exists():
                self.reserved.add(tid)
                return tid
            t += 1

    TERMINAL = ("done", "failed", "cancelled", "paused")   # paused: umarell stopped it resumable
    LIVE = ("queued", "planning")

    # ---------------------------------------------------------------- scheduling (under the lock)
    def _pool(self, job: Job) -> str:
        return "umarell" if job.kind == "umarell" else "remote" if job.remote else "planner"

    def _cap(self, pool: str) -> int:
        if pool == "remote":
            cfg = self._burst_cfg[1]
            return max(1, cfg.max_concurrent if cfg else 2)
        return self.umarell_workers if pool == "umarell" else self.workers

    def _cell_turns(self, job: Job) -> set:
        """What a local job stages per cell: "u3" (umarell_u3, and a umarell_jam job whose fallback is on: U3's cell
        in its shared patched copy), "jam" (umarell_jam's runs of the cell in its root)."""
        if job.remote:
            return set()             # a remote job stages its cell on its own instance
        if job.planner == JAM:
            return {"jam"} | ({"u3"} if (job.jam.get("fallback") or {}).get("enabled") else set())
        return {"u3"} if self.planners.get(job.planner, {}).get("kind") == "rules" else set()

    def _blocked(self, job: Job, ahead: bool = False) -> bool:
        """A queued job that must wait although a worker is free: umarell_u3 stages its cell in the patched umarell
        copy all U3 jobs share, so two U3 jobs on the same cell take turns (umarell_jam jobs likewise, and with a U3
        job when their fallback may run U3). ahead: a job of those queued before it blocks it too (plan's answer:
        that one starts first, though no worker has taken it yet)."""
        mine = self._cell_turns(job)
        return bool(mine) and any(
            j is not job and j.cell_id == job.cell_id and mine & self._cell_turns(j)
            and (j.status == "planning" or (ahead and j.status == "queued" and j.submitted_s <= job.submitted_s))
            for j in self.jobs.values())

    def _elapsed(self, job: Job, now: Optional[float] = None) -> float:
        if not job.started_s:
            return 0.0
        return (job.ended_s or now or time.time()) - job.started_s

    def _queue_info(self, job: Job) -> dict:
        """running / queued_behind / workers and the job's ETA (eta_s: until it is done; starts_in_s while queued),
        by laying the queued jobs of its pool onto the workers as they free up (planners.json typical_s)."""
        pool, now = self._pool(job), time.time()
        live = [j for j in self.jobs.values() if self._pool(j) == pool and j.status in self.LIVE]
        running = [j for j in live if j.status == "planning"]
        queued = sorted((j for j in live if j.status == "queued"), key=lambda j: j.submitted_s)
        out = {"running": [j.job_id for j in running if j is not job],
               "queued_behind": [j.job_id for j in queued if j.submitted_s < job.submitted_s and j is not job],
               "workers": {"planner": self.workers, "umarell": self.umarell_workers}}
        if job.status == "planning":
            out["eta_s"] = round(max(job.eta_s - self._elapsed(job, now), 0.0), 1)
            return out
        if job.status != "queued":
            out["eta_s"] = None
            return out
        free = sorted(max(j.eta_s - self._elapsed(j, now), 0.0) for j in running)
        slots = free + [0.0] * max(self._cap(pool) - len(free), 0)
        heapq.heapify(slots)
        for j in queued:
            start = heapq.heappop(slots)
            heapq.heappush(slots, start + j.eta_s)
            if j is job:
                out.update(starts_in_s=round(start, 1), eta_s=round(start + j.eta_s, 1))
                break
        return out

    def jobs_list(self, cell: Optional[str] = None, active_only: bool = False) -> dict:
        """Every job the router knows (since it started), newest first."""
        with self.lock:
            rows = []
            for j in sorted(self.jobs.values(), key=lambda j: j.submitted_s, reverse=True):
                if (cell and j.cell != cell) or (active_only and j.status not in self.LIVE):
                    continue
                q = self._queue_info(j)
                rows.append({"job_id": j.job_id, "tree_id": j.tree_id, "cell": j.cell, "planner": j.planner,
                             "status": j.status, "phase": j.phase, "elapsed_s": round(self._elapsed(j), 1),
                             "eta_s": q.get("eta_s"), **({"starts_in_s": q["starts_in_s"]} if "starts_in_s" in q else {}),
                             "plan_path": j.plan_path, "error": j.error, **({"remote": True} if j.remote else {})})
            counts = {st: sum(1 for j in self.jobs.values() if j.status == st) for st in self.LIVE}
        return {"workers": {"planner": self.workers, "umarell": self.umarell_workers},
                "planning": counts["planning"], "queued": counts["queued"], "jobs": rows}

    def plan_status(self, job_id: str, wait_s: float = 0) -> dict:
        """The job's status; with wait_s, held until its status or phase changes, it ends, or wait_s passed
        (so an agent polling it makes a few calls per minute, not one every 2 s)."""
        first = self._plan_status_once(job_id)
        deadline = time.time() + max(0.0, min(float(wait_s or 0), 60.0))
        key = lambda st: (st.get("status"), (st.get("progress") or {}).get("phase"))  # noqa: E731
        cur = first
        while cur.get("status") not in self.TERMINAL and time.time() < deadline:
            time.sleep(1.0 if job_id in self.jobs else 3.0)
            cur = self._plan_status_once(job_id)
            if key(cur) != key(first):
                break
        return cur

    def _plan_status_once(self, job_id: str) -> dict:
        with self.lock:
            job = self.jobs.get(self.alias.get(job_id or "", job_id or ""))
            if job and job.kind == "btexport":
                return self._status(job)
            ask = job is not None and job.submitted
        if job:                                   # an umarell job: umarell's own status, while it has the job
            if ask:
                try:
                    self._umarell_seen(job, self._umarell_call("get_plan_status", {"tree_id": job.tree_id}))
                except ToolError:
                    pass                          # its last known status
            with self.lock:
                return self._status_umarell(job)
        if (job_id or "").startswith("bt_"):      # planned through multi-robots-agents directly
            st = self._umarell_call("get_plan_status", {"tree_id": job_id})
            return {"job_id": job_id, "tree_id": job_id, "planner": "umarell", **st}
        raise ToolError(f"no job {job_id!r}", "plan returns the job_id; jobs lists them (kept until the router restarts)")

    def _status_umarell(self, job: Job) -> dict:
        out = {"job_id": job.job_id, "tree_id": job.tree_id if job.submitted else None, "planner": "umarell",
               "cell": job.cell, **job.umarell, "status": job.status,
               "progress": {"phase": job.phase, "elapsed_s": round(self._elapsed(job), 1)},
               **self._queue_info(job)}
        if job.plan_path:
            out["plan_path"] = job.plan_path
        if job.error:
            out["error"] = job.error
            out["hint"] = job.hint or ""
        return out

    def _status(self, job: Job) -> dict:
        elapsed = self._elapsed(job)
        q = self._queue_info(job)
        out = {"job_id": job.job_id, "tree_id": job.tree_id, "planner": job.planner, "cell": job.cell,
               "profile": job.profile, **({"speed": job.speed} if job.speed else {}), "status": job.status,
               "progress": {"phase": job.phase, "elapsed_s": round(elapsed, 1),
                            "eta_s": q.get("eta_s") if job.status in self.LIVE else None},
               "plan_path": job.plan_path, "export": job.export or None, "log_tail": self._log_tail(job), **q,
               **self._timing(job)}
        if job.status == "queued" and self._blocked(job):
            out["waiting_for"] = ("another umarell_u3 job on this cell (U3 jobs on one cell take turns)"
                                  if job.planner != JAM else
                                  "another umarell_jam job on this cell, or a umarell_u3 job when this job's fallback "
                                  "is on (they take turns on one cell)")
        if job.error:
            out["error"] = job.error
        if job.hint or job.error:
            out["hint"] = job.hint or ""
        return out

    @staticmethod
    def _log_tail(job: Job, n: int = 12) -> list:
        try:
            lines = (job.dir / "btexport.log").read_text(errors="replace").splitlines()
        except OSError:
            return []
        keep = [l for l in lines if l.startswith(("[btexport]", "[u3]", "[jam]")) or "Error" in l or "error" in l]
        return [l[:300] for l in (keep or lines)[-n:]]

    def cancel_plan(self, job_id: str) -> dict:
        proc, tree = None, None
        with self.lock:
            job = self.jobs.get(self.alias.get(job_id or "", job_id or ""))
            if job and job.status == "queued":
                job.status = job.phase = "cancelled"
                job.ended_s = time.time()
                self.reserved.discard(job.tree_id)
                self.cv.notify_all()
                return {"cancelled": True, "job_id": job_id}
            if job and job.status == "planning":
                job.cancel = True
                proc = job.proc
                tree = job.tree_id if job.kind == "umarell" and job.submitted else None
            elif job:
                return {"cancelled": False, "job_id": job_id, "status": job.status}
        if not job:
            if (job_id or "").startswith("bt_"):
                r = self._umarell_call("cancel_planning", {"tree_id": job_id, "reason": "cancelled by the user"})
                return {"cancelled": bool(r.get("cancelled", r.get("ok", True))), "job_id": job_id, "umarell": r}
            raise ToolError(f"no job {job_id!r}")
        if tree:        # umarell stops at its next step boundary; its status says cancelled then
            r = self._umarell_call("cancel_planning", {"tree_id": tree, "reason": "cancelled by the user"})
            return {"cancelled": bool(r.get("cancelled", r.get("ok", True))), "job_id": job_id, "umarell": r}
        if proc is not None:
            self._terminate(proc)
        return {"cancelled": True, "job_id": job_id}

    @staticmethod
    def _terminate(proc: subprocess.Popen) -> None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            return
        try:
            proc.wait(CANCEL_GRACE_S)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    # ---------------------------------------------------------------- tools: the cell and its task program
    _VEC = re.compile(r"(center|half_extents):\s*\[([^\]]*)\]")

    def describe_cell(self, cell: Optional[str] = None) -> dict:
        """Where the cell is and what is in it, for deriving a task program (its weld targets)."""
        name, root = self.resolve_cell(cell)
        cid = planner_cell_id(name)
        read = lambda f: f.read_text()[:20000] if f.is_file() else None  # noqa: E731
        obstacles = root / "agentic_planner" / "config" / "obstacles.yaml"
        boxes, cur = [], {}
        for key, vec in self._VEC.findall(read(obstacles) or ""):
            try:
                cur[key] = [float(v) for v in vec.split(",")]
            except ValueError:
                continue
            if "center" in cur and "half_extents" in cur:
                boxes.append(cur)
                cur = {}
        bbox = None
        if boxes:
            lo = [min(b["center"][i] - b["half_extents"][i] for b in boxes) for i in range(3)]
            hi = [max(b["center"][i] + b["half_extents"][i] for b in boxes) for i in range(3)]
            bbox = {"min": [round(v, 4) for v in lo], "max": [round(v, 4) for v in hi]}
        tasks = sorted(f.stem for f in (root / TASKS_DIR).glob("*.json")) if (root / TASKS_DIR).is_dir() else []
        products = self.cell_products(root)
        n_welds = sum(len(p["welds"]) for p in products)
        sources = self.task_sources(root)
        derived = [t["file"] for t in sources if t["from_product"]]
        if derived:
            hint = (f"the cell's task programs come from its products (the backend derives one per Product & Task "
                    f"task): {', '.join(derived)}; plan one of them. The user edits the welds in Product & Task "
                    "(the backend re-derives the files); never edit or replace them yourself")
        elif tasks:
            hint = "the cell has task programs: plan one of them"
        elif n_welds:
            hint = (f"no task program yet: call task_program_from_products to write the {n_welds} welds of the "
                    "cell's products (below) as its task program; never invent other weld targets")
        else:
            hint = ("no task program and no product with welds: tell the user and ask before deriving weld targets "
                    "from the fixtures")
        return {"cell": name, "cell_id": cid, "cell_dir": str(root),
                "products": products, "product_welds": n_welds, "next": hint,
                "note": "The cell's files are here (cell_dir), not in the umarell checkout's cells/.",
                "scene_yaml": read(self.scene_path(root)),
                "scene_file": str(self.scene_path(root)),
                "kind": "reference" if is_reference_cell(root) else "legacy",
                "obstacles_yaml": read(obstacles),
                "fixture_boxes_bbox": bbox,
                "task_programs": tasks,
                "task_sources": sources,
                "task_program_format": {
                    "operations": [{"id": "W01", "type": "weld", "position": [0.0, 0.0, 1.35],
                                    "approach_vector": [0.0, 0.0, -1.0],
                                    "orientation": {"x": 0.0, "y": -0.7071, "z": 0.0, "w": 0.7071},
                                    "weld_orientation": {"x": 0.0, "y": -0.7071, "z": 0.0, "w": 0.7071},
                                    "sequence_group": "structural", "priority": 1}],
                    "note": "positions in the world frame (m) on the fixture's surface the robots reach; "
                            "approach_vector points into the surface; orientation = the tool pointing along it: its "
                            "tool axis -R[:,0] is approach_vector (the default above is top-down; leave orientation "
                            "out and save_task_program derives it); save_task_program fills defaults and refuses "
                            "an orientation more than 10° off approach_vector or a position inside an obstacle box "
                            "that is not is_workpiece"}}

    def cell_products(self, root: Path) -> list:
        """The cell's fixtures that are Product & Task products (a product.json beside the fixture's URDF, or in the
        products folder under the URDF's name), with their point operations in the world frame."""
        scene_file = self.scene_path(root)
        try:
            scene = parse_yaml(scene_file.read_text()) or {}
        except (OSError, IndexError, ValueError):
            return []
        base = scene_file.parent     # a reference cell's paths start at its scene's folder scenes/<name>/ (or its view)
        fixtures = scene.get("fixtures") if isinstance(scene, dict) else None
        products_dir = Path(self._resolve_path(self._config().get("productsDir") or "products"))
        out = []
        for f in fixtures if isinstance(fixtures, list) else []:
            if not isinstance(f, dict) or not f.get("urdf"):
                continue
            urdf = scene_asset(base, str(f["urdf"]))
            for file in (urdf.parent / "product.json", products_dir / urdf.stem / "product.json"):
                try:
                    product = json.loads(file.read_text())
                except (OSError, ValueError):
                    continue
                placed = place_product(product, f)
                geometry = product.get("geometry") or {}
                out.append({"fixture_id": str(f.get("id") or urdf.stem), "product": product.get("name") or urdf.stem,
                            "product_file": str(file), "geometry": geometry, **placed})
                break
        return out

    @staticmethod
    def task_sources(root: Path) -> list:
        """Where each of the cell's task programs comes from: the backend derives one per product task
        (`<fixture>__<task>.json`, with a `source` {product, task, fixture, product_sha256}); others are hand-written."""
        out = []
        tasks = root / TASKS_DIR
        for f in sorted(tasks.glob("*.json")) if tasks.is_dir() else []:
            try:
                program = json.loads(f.read_text())
            except (OSError, ValueError):
                program = {}
            source = program.get("source") if isinstance(program, dict) else None
            entry = {"file": f.stem, "operations": len(program.get("operations") or []) if isinstance(program, dict) else 0}
            if isinstance(source, dict) and source.get("product"):
                entry.update(from_product=True, product=source.get("product"), task=source.get("task"),
                             fixture=source.get("fixture"))
            else:
                entry["from_product"] = False
            out.append(entry)
        return out

    def save_task_program(self, operations: list, cell: Optional[str] = None, name: str = "spot_welding",
                          description: str = "") -> dict:
        """Write `<cell>/agentic_planner/tasks/<name>.json` (the template: the canonical program's settings)."""
        return self._save_program(operations, cell, name, description)

    def _save_program(self, operations: list, cell: Optional[str], name: str, description: str,
                      workpieces: frozenset = frozenset()) -> dict:
        """save_task_program, with the obstacles `workpieces` counted as is_workpiece (about to be marked)."""
        cname, root = self.resolve_cell(cell)
        if not valid_name(name):
            raise ToolError(f"invalid task program name {name!r}", "letters, digits, _ and -")
        if not isinstance(operations, list) or not operations:
            raise ToolError("operations must be a non-empty list", "see describe_cell's task_program_format")
        top = dict(TOP_DOWN)
        obstacles = root / "agentic_planner" / "config" / "obstacles.yaml"
        boxes = obstacle_boxes(obstacles.read_text()) if obstacles.is_file() else []
        ops, seen = [], set()
        for i, op in enumerate(operations, 1):
            if not isinstance(op, dict):
                raise ToolError(f"operation {i} is not an object", "see describe_cell's task_program_format")
            pos = op.get("position")
            if not (isinstance(pos, list) and len(pos) == 3 and all(isinstance(v, (int, float)) for v in pos)):
                raise ToolError(f"operation {i}: position must be [x, y, z] in m", "")
            oid = str(op.get("id") or f"W{i:02d}")
            if oid in seen:
                raise ToolError(f"duplicate operation id {oid}", "")
            seen.add(oid)
            av = op.get("approach_vector") or [0.0, 0.0, -1.0]
            if not (isinstance(av, list) and len(av) == 3 and all(isinstance(v, (int, float)) for v in av)) \
                    or _unit(av) is None:
                raise ToolError(f"operation {oid}: approach_vector must be a non-zero [x, y, z] (into the surface)", "")
            ori = op.get("orientation") or (top if not op.get("approach_vector") else orientation_for(av))
            for key, q in (("orientation", ori), ("weld_orientation", op.get("weld_orientation"))):
                if q is None:
                    continue
                try:
                    axis = tool_axis(q)
                except (AttributeError, TypeError, ValueError):
                    raise ToolError(f"operation {oid}: {key} must be a quaternion {{x, y, z, w}}", "") from None
                off = angle_deg(axis, av)
                if off > MAX_TOOL_AXIS_DEG:
                    raise ToolError(
                        f"operation {oid}: {key}'s tool axis (-R[:,0] = {[round(v, 3) for v in axis]}) is "
                        f"{off:.0f}° off approach_vector {[round(float(v), 3) for v in av]} (max {MAX_TOOL_AXIS_DEG:.0f}°)",
                        f"leave {key} out and it is derived from approach_vector ({orientation_for(av)}), or fix "
                        "approach_vector; for a cell with products use task_program_from_products")
            for box in boxes:
                if not (box["is_workpiece"] or box["id"] in workpieces) and box_depth(box, pos) > INSIDE_EPS_M:
                    raise ToolError(
                        f"operation {oid}: position {pos} is {box_depth(box, pos):.3f} m inside obstacle "
                        f"{box['id']!r} (center {box['center']}, half_extents {box['half_extents']}), which is not "
                        "is_workpiece",
                        "weld points lie on the part's surface, not inside it: take them from the cell's products "
                        "(describe_cell's products, task_program_from_products) or put them on the box's surface")
            ops.append({"id": oid, "type": op.get("type", "weld"), "position": [float(v) for v in pos],
                        "approach_vector": [float(v) for v in av], "orientation": ori,
                        "sequence_group": op.get("sequence_group", "structural"),
                        "priority": int(op.get("priority", 1)),
                        "weld_orientation": op.get("weld_orientation") or ori})
        template = {}
        canon = self.s.umarell / "cells" / "welding_cell_2_small" / TASKS_DIR / f"{self.canonical_task}.json"
        try:
            template = json.loads(canon.read_text())
        except (OSError, ValueError):
            pass
        program = {k: v for k, v in template.items() if k != "operations"}
        program.update(program=name, cell_id=planner_cell_id(cname), version="1.0.0",
                       description=description or f"{len(ops)} spot welds (written through the planner router)",
                       operations=ops)
        program.setdefault("trajectory_backend", "ompl")
        out = root / TASKS_DIR / f"{name}.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(program, indent=2) + "\n")
        os.replace(tmp, out)
        return {"cell": cname, "task": name, "path": str(out), "operations": len(ops),
                "next": f"plan with task {name!r} (recommend_planner first)"}

    def task_program_from_products(self, cell: Optional[str] = None, name: Optional[str] = None) -> dict:
        """The cell's task program from its products' Product & Task welds (world frame by the fixtures' poses);
        marks the obstacles.yaml boxes the welds are on as is_workpiece."""
        cname, root = self.resolve_cell(cell)
        products = [p for p in self.cell_products(root) if p["welds"]]
        derived = [t for t in self.task_sources(root) if t["from_product"]]
        if derived and not name:
            return self._derived_result(cname, root, products, derived)
        if not products:
            raise ToolError(f"cell {cname!r} has no product with welds (no scene.yaml fixture with a product.json "
                            "that has point operations)",
                            "tell the user; derive weld targets from the fixtures only after they agree")
        ids = [w["id"] for p in products for w in p["welds"]]
        unique = len(set(ids)) == len(ids)
        ops = []
        for p in products:
            for w in p["welds"]:
                ops.append({"id": w["id"] if unique else f"{p['fixture_id']}_{w['id']}", "type": "weld",
                            "position": w["position"], "approach_vector": w["approach_vector"],
                            "orientation": w["orientation"], "weld_orientation": w["orientation"],
                            "sequence_group": w["group"], "priority": w.get("priority", 1)})
        processes = {w["process"] for p in products for w in p["welds"]}
        stem = "_".join(dict.fromkeys(p["product"] for p in products))
        name = name or (f"{stem}_spot_welding" if processes == {"spotWeld"} else f"{stem}_task")
        obstacles = root / "agentic_planner" / "config" / "obstacles.yaml"
        text = obstacles.read_text() if obstacles.is_file() else ""
        mark = sorted(workpiece_ids(obstacle_boxes(text), [o["position"] for o in ops]))
        tasks = ", ".join(f"{p['product']} at fixture {p['fixture_id']} ({len(p['welds'])} welds)" for p in products)
        out = self._save_program(ops, cname, name, f"From Product & Task: {tasks}; world frame by the fixtures' "
                                 "scene.yaml poses", frozenset(mark))
        if mark:
            tmp = obstacles.with_name(f".{obstacles.name}.tmp")
            tmp.write_text(mark_workpieces(text, set(mark)))
            os.replace(tmp, obstacles)
        out.update(products=[{k: p[k] for k in ("fixture_id", "product", "product_file", "pose", "skipped_paths")}
                             for p in products],
                   welds=[{"id": o["id"], "position": o["position"], "approach_vector": o["approach_vector"],
                           "orientation": o["orientation"]} for o in ops],
                   workpiece_boxes=mark,
                   obstacles_note=(f"marked {', '.join(mark)} is_workpiece in agentic_planner/config/obstacles.yaml "
                                   "(the boxes the welds are on); Prepare Cell regenerating obstacles.yaml drops it"
                                   if mark else "no obstacle box needed marking as is_workpiece"))
        return out

    def _derived_result(self, cname: str, root: Path, products: list, derived: list) -> dict:
        """task_program_from_products for a cell whose task programs the backend derived from its products: those
        (nothing written), with the obstacle boxes the welds are on marked is_workpiece if they are not yet."""
        obstacles = root / "agentic_planner" / "config" / "obstacles.yaml"
        text = obstacles.read_text() if obstacles.is_file() else ""
        mark = sorted(workpiece_ids(obstacle_boxes(text), [w["position"] for p in products for w in p["welds"]]))
        if mark:
            tmp = obstacles.with_name(f".{obstacles.name}.tmp")
            tmp.write_text(mark_workpieces(text, set(mark)))
            os.replace(tmp, obstacles)
        first = derived[0]
        return {"cell": cname, "task": first["file"], "path": str(root / TASKS_DIR / f"{first['file']}.json"),
                "operations": first["operations"], "source": "backend",
                "tasks": [{k: t[k] for k in ("file", "product", "task", "fixture", "operations")} for t in derived],
                "products": [{k: p[k] for k in ("fixture_id", "product", "product_file", "pose", "skipped_paths")}
                             for p in products],
                "workpiece_boxes": mark,
                "note": "the backend derived these task programs from the cell's products (one per Product & Task "
                        "task, kept up to date when the product is saved); nothing was written. The user changes the "
                        "welds in Product & Task.",
                "next": f"plan with task {first['file']!r} (recommend_planner first)"
                        + (f"; the other tasks: {', '.join(t['file'] for t in derived[1:])}" if len(derived) > 1 else "")}

    def list_plans(self, cell: Optional[str] = None) -> dict:
        name, root = self.resolve_cell(cell)
        plans = []
        for f in sorted((root / PLANS_DIR).glob("*.json")) if (root / PLANS_DIR).is_dir() else []:
            try:
                v = json.loads(f.read_text())
            except (OSError, ValueError):
                continue
            if not isinstance(v, dict) or "behaviour_tree" not in v:
                continue
            meta = v.get("planner_meta") or {}
            plans.append({"path": f"{PLANS_DIR}/{f.name}", "tree_id": v.get("tree_id", f.stem),
                          "planner": v.get("planner") or "umarell", "created_s": v.get("created_s"),
                          "num_operations": v.get("num_operations"), "status": v.get("status"),
                          "end_s": (meta.get("report") or {}).get("executor_end_s"), "profile": meta.get("profile")})
        plans.sort(key=lambda p: p.get("created_s") or 0, reverse=True)
        return {"cell": name, "plans": plans}

    # ---------------------------------------------------------------- the job workers
    def _next_job(self) -> Job:
        """The oldest queued subprocess job that may run now (under the lock; waits for one)."""
        while True:
            for job in sorted(self.jobs.values(), key=lambda j: j.submitted_s):
                if job.kind == "btexport" and job.status == "queued" and not job.remote and not self._blocked(job):
                    job.status, job.phase, job.started_s = "planning", "starting", time.time()
                    return job
            self.cv.wait()

    def _work(self) -> None:
        """One of the `workers` threads: runs subprocess jobs, one at a time each."""
        while True:
            with self.cv:
                job = self._next_job()
            try:
                self._run(job)
            except Exception as e:  # noqa: BLE001 - a job never takes the worker down
                with self.lock:
                    job.status, job.phase = "failed", "failed"
                    job.error, job.hint = f"router error: {type(e).__name__}: {e}", "see the router's journal"
            finally:
                self._cleanup(job)

    def _cleanup(self, job: Job) -> None:
        """A job's worker is done with it (it ended, or it was handed on)."""
        with self.cv:
            job.ended_s = job.ended_s or time.time()
            job.proc = None
            self.reserved.discard(job.tree_id)
            self.cv.notify_all()
        if job.view is not None:
            shutil.rmtree(job.view, ignore_errors=True)   # symlinks only: the cell's files stay
        self._prune()

    def _job_env(self, job: Job) -> dict:
        """A new job dir and cell view for the job, and its environment (a remote job's too: burst.py runs the same
        command line in it)."""
        shutil.rmtree(job.dir, ignore_errors=True)
        job.dir.mkdir(parents=True)
        view = job.view = self.build_view(job.cell_id, job.cell_dir, job.task_file, key=job.job_id)
        env = {k: v for k, v in os.environ.items() if k in ("PATH", "HOME", "LANG", "LC_ALL", "TMPDIR", "USER")}
        env.update(PYTHONPATH=".", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", PYTHONDONTWRITEBYTECODE="1",
                   MUJOCO_GL="disable", PYTHONUNBUFFERED="1", UMARELL_REPO=str(view))
        if job.spec.startswith("trajgen:"):
            env["TRAJGEN_WORKERS"] = "2"
        if job.planner == JAM:   # umarell_jam.py's knobs (FACTORY_OS_JAM_CACHE_S3, _UMARELL, _ROOT, ...)
            env.update({k: v for k, v in os.environ.items() if k.startswith("FACTORY_OS_JAM_")})
        if self.planners.get(job.planner, {}).get("kind") == "rules" and os.environ.get("FACTORY_OS_CATALOGUE_CACHE_PUSH"):
            # the opt-in save of the planner's cache to the catalogue after a successful job (catalogue_cache.py)
            env["FACTORY_OS_CATALOGUE_CACHE_PUSH"] = os.environ["FACTORY_OS_CATALOGUE_CACHE_PUSH"]
        (job.dir / "cmd.json").write_text(json.dumps({"cmd": job.cmd, "cwd": str(self.s.cell_planner),
                                                      "UMARELL_REPO": str(view), "remote": job.remote}, indent=1))
        return env

    def _run(self, job: Job) -> None:
        env = self._job_env(job)
        if job.planner == JAM:
            # its umarell_u3 fallback (if it comes to that) shares the CPUs as a U3 job does: its part of the timeout
            if job.jam["fallback"]["enabled"]:
                with self.lock:
                    workers = self._u3_workers_now()
                    self._jam_times(job, workers)
                env["FACTORY_OS_U3_WORKERS"] = str(workers)
        elif self.planners[job.planner]["kind"] == "rules":
            # U3 runs one OMPL worker per CPU: share the CPUs with the jobs running beside it; the timeout and the
            # ETA follow the workers it gets (u3_timeout)
            with self.lock:
                workers = self._u3_workers_now()
                self._scale_local(job, workers)
            env["FACTORY_OS_U3_WORKERS"] = str(workers)
        with open(job.dir / "btexport.log", "wb") as log:
            with self.lock:
                if job.cancel:
                    job.status = job.phase = "cancelled"
                    return
                job.proc = subprocess.Popen(job.cmd, cwd=str(self.s.cell_planner), env=env, stdout=log,
                                            stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
                job.phase = "planning"
            proc = job.proc
            deadline = job.started_s + job.timeout_s
            timed_out = False
            while proc.poll() is None:
                if time.time() > deadline:
                    timed_out = True
                    self._terminate(proc)
                    break
                if job.planner == JAM:
                    self._jam_fell_back(job)
                tail = self._log_tail(job, 3)
                if any("URDF limits" in l or "dense check" in l or "exporting and checking" in l for l in tail):
                    job.phase = "checking"
                time.sleep(0.5)
            code = proc.wait()
        self._finish(job, code, timed_out)

    def _finish(self, job: Job, code: int, timed_out: bool) -> None:
        """The ended run's result (a local or a remote one): its report, the plan published, or why not."""
        job.exit_code = code
        report = self._report(job)
        with self.lock:
            job.export = {"exit_code": code, **report}
            if job.planner == JAM and job.jam and ((report.get("jam") or {}).get("fallback") or {}).get("used"):
                job.jam["fallback"]["used"] = True
            if job.cancel:
                job.status = job.phase = "cancelled"
                return
            if timed_out:
                job.status, job.phase = "failed", "failed"
                job.error = f"timed out after {job.timeout_s:.0f} s" + (
                    f" ({job.u3_workers} U3 workers{' on ' + str(job.burst.get('instance_type')) if job.remote else ''})"
                    if job.u3_workers else "")
                job.hint = "nothing was published" + (
                    "; options.remote true plans it on a bigger instance" if job.planner == U3 and job.u3_workers
                    and not job.remote and job.u3_workers < self._u3_ref(job.planner)[0] else "")
                return
        # btexport's documented exit 1 is "exported but a check failed"; an uncaught exception also exits 1,
        # possibly after C.export wrote the plan JSON. The report file is written last, so only exit 1 WITH a
        # report is a failed check; exit 1 without one is a crash and nothing is published.
        crashed = code == 1 and not (job.dir / f"{job.tree_id}.report.json").is_file()
        if crashed:
            with self.lock:
                job.status, job.phase = "failed", "failed"
                last = self._log_tail(job, 1)
                job.error = "btexport exit 1 without a report (crashed): " + (last[0] if last else "no output")
                job.hint = self._srdf_hint(job) or "nothing was published: btexport crashed before its checks finished; see log_tail"
            return
        if code not in (0, 1):
            with self.lock:
                job.status, job.phase = "failed", "failed"
                last = self._log_tail(job, 1)
                if job.planner == JAM and code in (2, 3, 4):
                    fb = (job.jam.get("fallback") or {})
                    jam = job.export.get("jam") or {}
                    # umarell_jam's own reason when it had no plan (jam.json, untruncated: its diagnosis -- the
                    # missing welds and the cause they share -- is past the 300 characters of a log line), else the
                    # last log line (a refusal, a setup failure, the fallback's outcome).
                    detail = (f"umarell_jam {jam.get('status')}: {jam['reason']}"[:700]
                              if code == 3 and not fb.get("used") and jam.get("reason") else
                              (last[0] if last else "no output"))
                    job.error = {2: "umarell_jam refused the request: ", 3: "no complete plan: ",
                                 4: "umarell_jam setup failed: "}[code] + detail
                    if code == 2:
                        job.hint = "nothing was published; check options.welds / options.robots against the cell"
                    elif fb.get("used"):
                        job.hint = ("nothing was published: umarell_jam had no plan and its umarell_u3 fallback found "
                                    "none either; see log_tail (umarell plans any cell with constraints)")
                    elif (jam.get("diagnosis") or {}).get("blocker"):
                        blocker = jam["diagnosis"]["blocker"]
                        job.hint = ("nothing was published; every failed attempt hit the same cause: " + blocker[:240]
                                    + (". The robot model collides with itself at HOME: check the cell's dressed "
                                       "robot (catalogue urdf-dressed-robots) and its SRDF; a cache or a larger "
                                       "budget cannot help" if "at HOME" in blocker else ""))
                    else:
                        job.hint = ("nothing was published"
                                    + (f" (cache {jam['cache'].get('misses')} miss(es))"
                                       if isinstance(jam.get("cache"), dict) else "")
                                    + f"; fallback off: {fb.get('why') or 'not asked'}. Plan with umarell_u3 "
                                    "(minutes), or prepare umarell_jam's cache for this cell (python -m umarell_jam "
                                    "prepare, offline)")
                    return
                if self.planners[job.planner]["kind"] == "rules" and code in (3, 4):
                    job.error = ("U3 found no complete plan: " if code == 3 else "U3 setup failed: ") + \
                        (last[0] if last else "no output")
                    job.hint = "nothing was published; see log_tail (umarell plans any cell with constraints)"
                    return
                job.error = f"btexport exit {code}: " + (last[0] if last else "no output")
                job.hint = self._srdf_hint(job) or "nothing was published (2: unsupported / malformed input, else a crash; see log_tail)"
            return
        job.phase = "publishing"
        path = self._publish(job, report, passed=(code == 0))
        with self.lock:
            job.plan_path = path
            job.status, job.phase = "done", "done"
            if code == 1:
                job.hint = ("published with status 'unverified': an export check failed (export.failed_checks); "
                            "say so before it is used")

    def _srdf_hint(self, job: Job) -> Optional[str]:
        """The hint for a run that failed on a robot's SRDF: Prepare Cell makes the SRDFs (a reference cell's in its
        agentic_planner/srdf/, linked beside the URDFs in its view .view/robots/), never Robo Dresser, and the
        catalogue's urdf-dressed-robots/ holds none; None when the log names no SRDF."""
        if not any("srdf" in l.lower() for l in self._log_tail(job, 40)):
            return None
        return ("nothing was published: a robot's SRDF is missing, so the cell's prepared model or its view is out of "
                "date (not a Robo Dresser or catalogue problem: the SRDF lives in the cell's agentic_planner/srdf/, "
                "never beside the catalogue's URDF). Prepare Cell for this cell (Agentic Planning, or POST "
                "/api/v1/agentic/prepare {\"cell\": ...}), then plan again")

    def _report(self, job: Job) -> dict:
        try:
            r = json.loads((job.dir / f"{job.tree_id}.report.json").read_text())
        except (OSError, ValueError):
            # no export (umarell_jam had no plan): what umarell_jam.py says it did, if it ran
            try:
                jam = json.loads((job.dir / "jam.json").read_text()) if job.planner == JAM else None
            except (OSError, ValueError):
                jam = None
            return {"jam": jam} if isinstance(jam, dict) else {}
        timing = r.get("timing") or {}
        checks = r.get("checks") or {}
        dense = r.get("dense") or {}
        lim = checks.get("limits") or {}
        failed = [k for k, v in checks.items() if isinstance(v, dict) and v.get("pass") is False]
        if dense and not dense.get("valid", True):
            failed.append("dense")
        return {"passed": bool(r.get("export_pass")) and (not dense or bool(dense.get("valid"))),
                "failed_checks": failed, "end_s": timing.get("executor_end_s"),
                "executor_end_s": timing.get("executor_end_s"), "makespan_motion_s": timing.get("makespan_motion_s"),
                "last_weld_done_s": timing.get("last_weld_done_s"),
                "min_clearance_m": dense.get("sphere_min_m"), "dense_valid": dense.get("valid") if dense else None,
                "acc_frac_amax_max": lim.get("dense_acc_frac_amax_max"),
                "vel_frac_vmax_max": lim.get("segment_vel_frac_vmax_max"),
                "warnings": (r.get("warnings") or [])[:5],
                **({"u3": r["u3"]} if isinstance(r.get("u3"), dict) else {}),
                **({"jam": r["jam"]} if isinstance(r.get("jam"), dict) else {})}

    def _publish(self, job: Job, report: dict, passed: bool) -> str:
        """Copy the plan JSON (only it: no report / XML side files) into the cell's plans, atomically."""
        plan = json.loads((job.dir / f"{job.tree_id}.json").read_text())
        plans = job.cell_dir / PLANS_DIR
        plans.mkdir(parents=True, exist_ok=True)
        with self.lock:      # the id check and the write at once: another job may publish into this cell now
            return self._publish_locked(job, report, passed, plan, plans)

    def _publish_locked(self, job: Job, report: dict, passed: bool, plan: dict, plans: Path) -> str:
        tid = job.tree_id
        t = int(tid.split("_")[1])
        # e.g. umarell saved a plan in the same second; never an id another live job (or umarell plan) holds
        while (plans / f"{tid}.json").exists() or (tid != job.tree_id and tid in self.reserved):
            t += 1
            tid = f"bt_{t}_{job.cell_id}"
        plan["tree_id"] = tid
        if isinstance(plan.get("behaviour_tree"), dict):
            plan["behaviour_tree"]["tree_id"] = tid
        plan["cell_id"] = job.cell_id
        if not isinstance(plan.get("created_s"), (int, float)) or plan["created_s"] <= 0:
            plan["created_s"] = time.time()        # trajgen samples carry 0: the plan list sorts by it
        if not passed:
            plan["status"] = "unverified"
        plan["planner"] = job.planner
        plan["task"] = job.task
        p = self.planners[job.planner]
        plan["planner_meta"] = {
            "router": f"{SERVER_NAME} {SERVER_VERSION}", "source_spec": job.spec,
            "checkpoint": p.get("checkpoint") or None, "profile": job.profile, "cell_planner_commit": self.commit,
            "elapsed_s": round((job.ended_s or time.time()) - (job.started_s or time.time()), 1),
            "export_exit": job.exit_code, "report": report, "job_id": job.job_id,
        }
        inputs = self.prepared_inputs(job.cell_dir)
        if inputs:
            plan["planner_meta"]["inputs"] = inputs
        write_json_atomic(plans / f"{tid}.json", plan)
        if tid != job.tree_id:
            self.reserved.discard(job.tree_id)
            self.reserved.add(tid)
        job.tree_id = tid
        return f"{PLANS_DIR}/{tid}.json"

    @staticmethod
    def prepared_inputs(root: Path) -> Optional[dict]:
        """What the cell's prepared planner model was made from (agentic_planner/inputs.json: its scene's, URDFs',
        products' hashes, written by Prepare Cell): a plan records it as planner_meta.inputs, and is stale (the
        backend says why) once the cell's inputs differ."""
        try:
            inputs = json.loads((root / PREPARED_INPUTS).read_text())
        except (OSError, ValueError):
            return None
        return inputs if isinstance(inputs, dict) and inputs.get("combined") else None

    def stamp_inputs(self, root: Path, plan_rel: str) -> None:
        """Record the prepared inputs in a plan umarell saved itself (planner_meta.inputs), unless it has some."""
        inputs = self.prepared_inputs(root)
        f = root / plan_rel
        if not inputs or not f.is_file():
            return
        try:
            plan = json.loads(f.read_text())
            meta = plan.setdefault("planner_meta", {}) if isinstance(plan, dict) else None
            if not isinstance(meta, dict) or meta.get("inputs"):
                return
            meta["inputs"] = inputs
            write_json_atomic(f, plan)
        except (OSError, ValueError) as e:
            print(f"[router] recording the inputs of {f}: {e}", file=sys.stderr, flush=True)

    def _prune(self) -> None:
        jobs = self.s.work / "jobs"
        if not jobs.is_dir():
            return
        dirs = sorted((d for d in jobs.iterdir() if d.is_dir()), key=lambda d: d.stat().st_mtime, reverse=True)
        with self.lock:
            live = {j.dir for j in self.jobs.values() if j.status in ("queued", "planning")}
        for d in dirs[KEEP_JOB_DIRS:]:
            if d not in live:
                shutil.rmtree(d, ignore_errors=True)

    # ---------------------------------------------------------------- umarell_u3: timing and remote planning
    def _u3_workers_now(self, starting: bool = False) -> int:
        """(Under the lock.) The U3 workers a local job gets now: FACTORY_OS_U3_WORKERS, else the CPUs shared by the
        local subprocess jobs planning (starting: and this one, not planning yet)."""
        fixed = env_int("FACTORY_OS_U3_WORKERS", 0)
        if fixed:
            return fixed
        busy = sum(1 for j in self.jobs.values() if j.kind == "btexport" and j.status == "planning" and not j.remote)
        return max(1, CPUS // max(busy + (1 if starting else 0), 1))

    def _u3_ref(self, planner: str) -> tuple[int, float, float]:
        p = self.planners[planner]
        return (int(p.get("ref_workers") or U3_REF_WORKERS), float(p.get("timeout_max_s") or U3_TIMEOUT_MAX_S),
                float(p.get("remote_setup_s") or BURST_SETUP_S))

    def _scale_local(self, job: Job, workers: int) -> None:
        ref, cap, _ = self._u3_ref(job.planner)
        job.u3_workers = workers
        job.timeout_s = u3_timeout(job.timeout_base_s, workers, ref, cap)
        job.eta_s = u3_eta(job.eta_base_s, workers, ref)

    def _scale_remote(self, job: Job, cpus: int) -> None:
        ref, cap, setup = self._u3_ref(job.planner)
        job.u3_workers = cpus
        job.timeout_s = u3_timeout(job.timeout_base_s, cpus, ref, cap) + setup
        job.eta_s = u3_eta(job.eta_base_s, cpus, ref) + setup

    def _u3_times(self, job: Job, p: dict, remote: dict) -> None:
        """(Under the lock, at plan.) A umarell_u3 job's timeout and ETA for the workers it will get, and where it runs."""
        job.timeout_base_s, job.eta_base_s = job.timeout_s, job.eta_s
        job.remote = bool(remote.get("use"))
        job.burst = {k: v for k, v in remote.items() if k != "use" and v is not None}
        cfg = self._burst_cfg[1]
        if job.remote and cfg is not None:
            self._scale_remote(job, burst.vcpus(cfg.instance_types[0]) or 32)
        else:
            job.remote = False
            self._scale_local(job, self._u3_workers_now(starting=True))

    def _timing(self, job: Job) -> dict:
        """plan / plan_status: the effective timeout (and a umarell_u3 job's U3 workers, where it plans)."""
        if job.kind != "btexport":
            return {}
        out = {"timeout_s": job.timeout_s}
        if job.planner == JAM and job.jam:
            fb = job.jam["fallback"]
            out["fallback"] = dict(fb)
            out["timeout_note"] = (f"umarell_jam's {job.jam['timeout_s']:.0f} s + {JAM_MARGIN_S:.0f} s (setup, export)"
                                   + (f" + umarell_u3's fallback timeout for {job.u3_workers} U3 workers"
                                      if fb["enabled"] else ""))
            if fb["enabled"]:
                out["u3_workers"] = job.u3_workers
        if job.timeout_base_s:
            out["u3_workers"] = job.u3_workers
            if job.timeout_s != job.timeout_base_s:
                out["timeout_note"] = (f"planners.json's {job.timeout_base_s:.0f} s is for "
                                       f"{self._u3_ref(job.planner)[0]} U3 workers; this job has {job.u3_workers}"
                                       + (" on a remote instance, plus its boot and install" if job.remote else ""))
        if job.burst or job.remote:
            out["remote"] = {"used": job.remote, **job.burst}
            if job.burst.get("fallback"):
                out["remote_note"] = (f"{job.burst['fallback']}: planning on this workspace instead "
                                      f"({job.u3_workers} U3 workers, timeout {job.timeout_s:.0f} s)")
        return out

    def _remote_choice(self, options: dict) -> dict:
        """Whether a umarell_u3 job plans on a planner-burst instance: {"use", "why", "fallback"}; {} = here, as always.
        options.remote true / false decides; else FACTORYOS_BURST (auto: below FACTORYOS_BURST_MIN_CPUS CPUs)."""
        asked = options.get("remote")
        if asked is False:
            return {"use": False, "why": "options.remote false"}
        mode = self.burst_mode
        auto = mode == "always" or (mode not in ("off", "0", "false", "no") and CPUS < self.burst_min_cpus)
        if asked is None and not auto:
            return {}
        why = ("options.remote true" if asked else "FACTORYOS_BURST=always" if mode == "always"
               else f"this workspace has {CPUS} CPUs (< {self.burst_min_cpus}, FACTORYOS_BURST_MIN_CPUS)")
        cfg, err = self._burst_config()
        if cfg is None:
            return {"use": False, "why": why, "fallback": f"remote planning is not available ({err})"}
        return {"use": True, "why": why}

    def _burst_config(self) -> tuple[Optional["burst.BurstConfig"], str]:
        """The planner-burst config (burst.load_config), re-read every 5 min (2 min while there is none)."""
        t, cfg, err = self._burst_cfg
        if time.time() - t < (300 if cfg else 120):
            return cfg, err
        try:
            cfg, err = self.burst_load(), ""
        except burst.BurstUnavailable as e:
            cfg, err = None, str(e)
        self._burst_cfg = (time.time(), cfg, err)
        if cfg is not None and self._burst_slots is None:
            self._burst_slots = threading.BoundedSemaphore(max(1, cfg.max_concurrent))
        return cfg, err

    def _remote_job(self, job: Job) -> None:
        """A remote umarell_u3 job's thread (not a planner worker: the instance plans): at most max_concurrent at once;
        when remote planning fails before the job ran, the job is queued for the local workers."""
        slots = self._burst_slots or threading.BoundedSemaphore(2)
        handed_on = False
        with slots:
            with self.cv:
                if job.status != "queued" or job.cancel:
                    return                      # cancelled while it waited (cancel_plan did the bookkeeping)
                job.status, job.phase, job.started_s = "planning", "uploading", time.time()
            try:
                self._run_remote(job)
            except burst.BurstUnavailable as e:
                handed_on = self._fall_back(job, str(e))
            except Exception as e:  # noqa: BLE001 - a job never takes the router down
                with self.lock:
                    job.status, job.phase = "failed", "failed"
                    job.error, job.hint = f"router error: {type(e).__name__}: {e}", "see the router's journal"
            finally:
                if not handed_on:
                    self._cleanup(job)

    def _fall_back(self, job: Job, why: str) -> bool:
        """Remote planning failed before the job ran: queue it for the local workers (with the capacity-scaled timeout).
        False: it was cancelled meanwhile."""
        # The remote attempt's view goes first: once queued, a local worker builds the job's view at the same path
        # (keyed by the job id), and removing it after the hand-on deleted the view that worker plans in.
        if job.view is not None:
            shutil.rmtree(job.view, ignore_errors=True)
            job.view = None
        print(f"[router] {job.job_id}: remote planning failed ({why}); planning locally", file=sys.stderr, flush=True)
        with self.cv:
            job.burst["fallback"] = f"remote planning failed: {why}"[:700]
            job.launcher = None
            if job.cancel:
                job.status = job.phase = "cancelled"
                return False
            job.remote = False
            job.status = job.phase = "queued"
            job.started_s = job.ended_s = job.exit_code = None
            job.export = {}
            self._scale_local(job, self._u3_workers_now(starting=True))
            self.cv.notify_all()
        return True

    def _run_remote(self, job: Job) -> None:
        cfg, err = self._burst_config()
        if cfg is None:
            raise burst.BurstUnavailable(err)
        env = self._job_env(job)
        launcher = job.launcher = burst.Launcher(cfg, self.burst_aws(cfg.region), tree_id=job.tree_id)
        job.burst["s3"] = launcher.prefix
        try:
            self._remote_upload(job, cfg, launcher, env)
            if job.cancel:
                with self.lock:
                    job.status = job.phase = "cancelled"
                return
            job.phase = "launching"
            launcher.launch(job.dir, int(job.timeout_s // 60) + 30)
            with self.lock:
                job.burst.update(instance_id=launcher.instance_id, instance_type=launcher.instance_type,
                                 market=launcher.market)
                if launcher.attempts:
                    job.burst["launch_attempts"] = launcher.attempts[-6:]
                self._scale_remote(job, burst.vcpus(launcher.instance_type or "") or 32)
                job.phase = "booting"
            st, timed_out = self._remote_wait(job, cfg, launcher)
            if st is None:
                return                          # cancelled
            if timed_out:
                launcher.terminate()
                self._finish(job, -15, True)
                return
            if st.get("exit_code") is None:
                raise burst.BurstUnavailable(f"the instance's {st.get('stage') or 'setup'} failed: "
                                             f"{st.get('error') or 'see its instance.log'}")
            job.phase = "downloading"
            launcher.fetch_result(job.dir)
            job.burst.update({k: st[k] for k in ("setup_s", "plan_s", "cpus") if st.get(k) is not None})
            self._finish(job, int(st["exit_code"]), False)
        finally:
            launcher.terminate()                # it powers itself off; this is the backstop

    def _remote_upload(self, job: Job, cfg: "burst.BurstConfig", launcher: "burst.Launcher", env: dict) -> None:
        """inputs.tgz (the view, the cell resolved, what the cell references by absolute path, the router's U3
        scripts) and job.json to the job's S3 prefix."""
        umarell_py = self.s.umarell_python or self.s.umarell / ".venv-planner" / "bin" / "python"
        pins = burst.pins(self.s.cell_planner, self.s.python, self.s.umarell, umarell_py)
        installed = [self.s.cell_planner, self.s.umarell, self.s.umarell.with_name(self.s.umarell.name + "-base"),
                     self.s.python.parent.parent, umarell_py.parent.parent]
        ip = burst.plan_inputs(job.view, job.cell_dir, installed, job.task_file, cmd=job.cmd, exclude=[job.dir])
        summary = ip.summary()
        job.burst["inputs_mb"] = summary["mb"]
        if summary["skipped"]:
            job.burst["inputs_skipped"] = summary["skipped"][:5]
        if summary["referenced"]:
            job.burst["inputs_referenced"] = [f"{r['path']} ({r['files']} files, {r['mb']} MB)"
                                              for r in summary["referenced"][:8]]
        if summary["missing"]:
            job.burst["inputs_missing"] = summary["missing"][:5]
        for line in ip.log_lines():
            print(f"[router] {job.job_id}: remote inputs: {line}", file=sys.stderr, flush=True)
        tgz = job.dir.parent / f".{job.tree_id}.inputs.tgz"
        try:
            burst.write_inputs(ip, tgz)
            spec = burst.job_spec(cfg, launcher.key, job.tree_id, job.dir, job.cmd, str(self.s.cell_planner), env,
                                  pins, [self.s.work, job.dir], cell_dir=job.cell_dir, required=ip.required())
            launcher.upload(tgz, spec)
        finally:
            tgz.unlink(missing_ok=True)

    def _remote_wait(self, job: Job, cfg: "burst.BurstConfig", launcher: "burst.Launcher") -> tuple[Optional[dict], bool]:
        """Poll the instance's status.json (heartbeat, progress, btexport.log) until it is there: (status, False);
        ({}, True) past the job's timeout; (None, False) cancelled. BurstUnavailable: the instance ended without one,
        or its progress.json (uploaded every 20 s) did not change for cfg.progress_timeout_s (it was rebooted, hung or
        lost its job: the router then terminates it and plans locally instead of waiting for the timeout)."""
        polls = 0
        seen, changed = None, time.time()
        while True:
            if job.cancel:
                with self.lock:
                    job.status = job.phase = "cancelled"
                return None, False
            if time.time() > (job.started_s or time.time()) + job.timeout_s:
                return {}, True
            launcher.heartbeat()
            st = launcher.status()
            if st is not None:
                return st, False
            prog = launcher.progress()
            phase = prog.get("phase")
            if phase in ("installing", "planning", "checking"):
                job.phase = phase
                job.burst["phase"] = phase
            mark = (prog.get("t"), prog.get("elapsed_s"), phase)
            if mark != seen:
                seen, changed = mark, time.time()
            elif time.time() - changed > cfg.progress_timeout_s:
                tail = launcher.instance_log().strip().splitlines()[-3:]
                raise burst.BurstUnavailable(
                    f"instance {launcher.instance_id} stopped reporting: no progress for {time.time() - changed:.0f} s "
                    f"(phase {phase or 'booting'})" + (f" ({' | '.join(tail)[-300:]})" if tail else ""))
            launcher.fetch_log(job.dir / "btexport.log")
            polls += 1
            if polls % 4 == 0:
                state = launcher.state()
                if state in ("shutting-down", "terminated", "stopping", "stopped"):
                    st = launcher.status()
                    if st is not None:
                        return st, False
                    tail = launcher.instance_log().strip().splitlines()[-3:]
                    raise burst.BurstUnavailable(f"instance {launcher.instance_id} {state} before the job finished"
                                                 + (f" ({' | '.join(tail)[-300:]})" if tail else ""))
            end = time.time() + cfg.poll_s
            while time.time() < end and not job.cancel:
                time.sleep(min(0.5, cfg.poll_s))

    # ---------------------------------------------------------------- the umarell route
    def _umarell_call(self, tool: str, arguments: dict, timeout: float = 30.0) -> dict:
        body = {"jsonrpc": "2.0", "id": uuid.uuid4().hex[:8], "method": "tools/call",
                "params": {"name": tool, "arguments": arguments}}
        try:
            r = http_json(self.s.umarell_url, body, timeout)
        except (urllib.error.URLError, OSError, ValueError) as e:
            raise ToolError(f"umarell planner not reachable ({e})", "Start simulation in Agentic Planning") from None
        if r.get("error"):
            raise ToolError(f"umarell {tool}: {r['error'].get('message')}")
        content = (r.get("result") or {}).get("content") or []
        text = content[0].get("text") if content else "{}"
        try:
            value = json.loads(text)
        except ValueError:
            value = {"text": text}
        return value if isinstance(value, dict) else {"result": value}

    def _umarell_health(self) -> bool:
        url = self.s.umarell_url.rsplit("/mcp", 1)[0] + "/health"
        try:
            with urllib.request.urlopen(url, timeout=3) as resp:
                return resp.status == 200
        except (urllib.error.URLError, OSError):
            return False

    def _ensure_umarell(self) -> None:
        if self._umarell_health():
            return
        try:
            http_json(self.s.backend_url.rstrip("/") + "/api/v1/agentic/start", {}, 15)
        except (urllib.error.URLError, OSError, ValueError):
            pass
        deadline = time.time() + UMARELL_START_WAIT_S
        while time.time() < deadline:
            if self._umarell_health():
                return
            time.sleep(2)
        raise ToolError("umarell planner not running", "Start simulation in Agentic Planning")

    def _ensure_prepared(self, name: str, root: Path) -> None:
        """umarell plans from the FactoryOS cell's agentic_planner/ (cell.yaml, config/), which Prepare Cell writes:
        without it every run_trajectory_planner fails on the robot lookup and umarell refuses the tree. The
        viewer's Plan Cell prepares first; a chat request or a headless call may not, so prepare the cell here (the
        backend's Prepare Cell for that cell, the same converter) and wait for it. The backend prepares one cell
        at a time: jobs take turns, and one the viewer started for another cell is waited out first."""
        if self._prepared_ok(name, root):
            return
        with self.prepare_lock:
            if self._prepared_ok(name, root):
                return
            self._prepare(name, root)

    def _prepared_ok(self, name: str, root: Path) -> bool:
        """Prepared, not stale, and (a reference cell) its view .view/ built for what it was prepared from: the job
        views link the view's scene.yaml, robots/ and fixtures/, so a missing view (e.g. a restored backup, which
        leaves views out) or one of other inputs is made again by preparing the cell."""
        if not (root / PREPARED_MARKER).is_file():
            return False
        if is_reference_cell(root) and not self.view_current(root):
            return False
        return not self.prepared_stale(name)

    @classmethod
    def view_current(cls, root: Path) -> bool:
        """Whether a reference cell's view was built for the inputs its planner model was prepared from (the view's
        stamp .view/.inputs, the backend's, against agentic_planner/inputs.json) and links the cell's SRDFs."""
        inputs = cls.prepared_inputs(root)
        try:
            stamp = (root / VIEW_DIR / ".inputs").read_text().strip()
        except OSError:
            return False
        if not (inputs and stamp == inputs.get("combined") and (root / VIEW_DIR / "scene.yaml").is_file()):
            return False
        # Each of the cell's SRDFs beside its URDF in the view's robots/ (a view built before Prepare Cell made
        # them links the catalogue's package, which has none: the planner finds no SRDF).
        srdfs = sorted((root / SRDF_DIR).glob("*.srdf")) if (root / SRDF_DIR).is_dir() else []
        robots = root / VIEW_DIR / "robots"
        return all(any(robots.glob(f"*/{s.name}")) for s in srdfs)

    def prepared_stale(self, name: str) -> Optional[str]:
        """Why the cell's prepared planner model is stale (its scene, a robot's URDF or a product changed since
        Prepare Cell), as the backend's cell list says; None if it is not, or the backend can't tell."""
        try:
            info = http_json(self.s.backend_url.rstrip("/") + "/api/v1/cell/cells", None, 5) or {}
        except (urllib.error.URLError, OSError, ValueError):
            return None
        for e in info.get("entries") or [] if isinstance(info, dict) else []:
            if isinstance(e, dict) and e.get("name") == name and (e.get("prepared_stale") or e.get("preparedStale")):
                return e.get("stale_reason") or e.get("staleReason") or "its inputs changed"
        return None

    def _prepare(self, name: str, root: Path) -> None:
        base = self.s.backend_url.rstrip("/") + "/api/v1/agentic"
        active = self.active_cell()
        print(f"[router] {name} is not prepared: running Prepare Cell", file=sys.stderr, flush=True)

        def start() -> dict:
            try:
                return http_json(base + "/prepare", {"cell": name}, 15) or {}
            except (urllib.error.URLError, OSError, ValueError) as e:
                raise ToolError(f"cell {name} is not prepared and Prepare Cell could not start: {e}",
                                "Press Prepare Cell in Agentic Planning, then plan again.") from None

        st = start()
        if "prepareCell" not in st and (not active or active[0] != name):
            # a backend whose Prepare Cell takes no cell: it would prepare the active cell
            raise ToolError(f"cell {name} is not prepared for umarell (no {PREPARED_MARKER})",
                            "Open the cell (step 1 Load & Play or Robo Fitter -> Open cell), press Prepare Cell in "
                            "Agentic Planning, then plan again.")
        deadline = time.time() + PREPARE_WAIT_S
        asked = 1
        while True:
            state, whose = st.get("prepare"), st.get("prepareCell")
            if whose and whose != name:
                # another cell's Prepare Cell (the viewer's) was running: ask again once it is over
                if state != "preparing":
                    if asked > 5:
                        raise ToolError(f"Prepare Cell for {name} did not start (the backend keeps preparing {whose})",
                                        "Wait for Prepared in Agentic Planning, then plan again.")
                    st, asked = start(), asked + 1
                    continue
            else:
                if state == "failed":
                    raise ToolError(f"Prepare Cell failed for {name}: {st.get('prepareError')}",
                                    "See the Agentic Planning log; fix the cell's scene and press Prepare Cell.")
                if state != "preparing" and (root / PREPARED_MARKER).is_file():
                    return
                if state == "done":
                    raise ToolError(f"Prepare Cell finished but {name} still has no {PREPARED_MARKER}",
                                    "See the Agentic Planning log.")
            if time.time() > deadline:
                raise ToolError(f"Prepare Cell for {name} did not finish in {PREPARE_WAIT_S:.0f} s",
                                "Wait for Prepared in Agentic Planning, then plan again.")
            time.sleep(2)
            try:
                st = http_json(base + "/status", None, 10) or {}
            except (urllib.error.URLError, OSError, ValueError):
                st = {"prepare": "preparing", "prepareCell": whose}

    def _plan_umarell(self, name: str, root: Path, cid: str, task: Optional[str], options: dict) -> dict:
        operations = options.get("operations")
        if not operations:
            task = (task or "").strip()
            if task in ("", "(none yet)"):
                task = self.canonical_task
            if not valid_name(task):
                raise ToolError(f"invalid task program name {task!r}")
            for f in (root / TASKS_DIR / f"{task}.json", self.s.umarell / "cells" / cid / TASKS_DIR / f"{task}.json"):
                if f.is_file():
                    try:
                        operations = json.loads(f.read_text()).get("operations")
                    except (OSError, ValueError):
                        operations = None
                    break
        if not operations:
            raise ToolError(f"no task program {task!r} for cell {name}",
                            "Call multi-robots-agents' list_task_programs / plan_trajectories directly, or save one.")
        self._ensure_prepared(name, root)
        self._ensure_umarell()
        args = {"cell_id": cid, "operations": operations}
        for k in ("constraints", "model"):
            if options.get(k):
                args[k] = options[k]
        eta = float(self.planners["umarell"].get("typical_s") or UMARELL_ETA_S)
        with self.cv:
            job = Job(job_id=f"um_{uuid.uuid4().hex[:10]}", tree_id="", planner="umarell", cell=name, cell_id=cid,
                      cell_dir=root, profile="", spec="", cmd=[], timeout_s=0.0, eta_s=eta,
                      dir=self.s.work / "jobs" / "umarell", task="(operations)" if options.get("operations") else task,
                      kind="umarell", umarell_args=args)
            busy = [j for j in self.jobs.values() if j.kind == "umarell" and j.status in self.LIVE]
            now = len(busy) < self.umarell_workers
            if now:     # a free slot: submit at once, so a refusal is this call's error
                job.status, job.phase, job.started_s = "planning", "submitting", time.time()
            self.jobs[job.job_id] = job
            self.cv.notify_all()
        if now:
            try:
                self._submit_umarell(job, rekey=True)
            except ToolError:
                with self.cv:
                    self.jobs.pop(job.job_id, None)
                    self.cv.notify_all()
                raise
        with self.lock:
            out = self._started(job)
        out.update(model=job.umarell.get("model"),
                   poll="plan_status(job_id); multi-robots-agents' get_plan_events / get_plan_quality for details")
        return out

    def _submit_umarell(self, job: Job, rekey: bool = False) -> None:
        """plan_trajectories for `job`. umarell names a plan bt_<second>_<cell> when it gets it: the two ids it may
        pick are reserved while it is asked (and its plans of this cell this second are waited out), so two plans
        never get one id: not two umarell plans, not an umarell and a router plan. A job that was submitted at
        once is re-keyed to umarell's tree id (its job_id, as before); a queued one keeps its job_id."""
        cid = job.cell_id
        deadline = time.time() + 15
        while True:
            with self.lock:
                t = int(time.time())
                ids = [f"bt_{t}_{cid}", f"bt_{t + 1}_{cid}"]
                if not any(i in self.reserved or (job.cell_dir / PLANS_DIR / f"{i}.json").exists() for i in ids):
                    self.reserved.update(ids)
                    break
            if time.time() > deadline:
                raise ToolError(f"umarell: no free plan id for cell {cid}", "plan again in a few seconds")
            time.sleep(0.2)
        try:
            r = self._umarell_call("plan_trajectories", job.umarell_args, timeout=60)
        finally:
            with self.lock:
                self.reserved.difference_update(ids)
        if not r.get("accepted", True) or not r.get("tree_id"):
            raise ToolError(f"umarell refused the plan: {r.get('error', r)}", r.get("hint", ""))
        tree = r["tree_id"]
        with self.cv:
            job.tree_id, job.submitted, job.misses = tree, True, 0
            job.umarell = {k: v for k, v in r.items() if k not in ("accepted", "poll")}
            job.phase = "planning"
            self.reserved.add(tree)
            if rekey and self.jobs.get(job.job_id) is job:
                # submitted at once: the job is known by umarell's tree id (what plan returned before)
                del self.jobs[job.job_id]
                job.job_id = tree
                self.jobs[tree] = job
            else:
                self.alias[tree] = job.job_id
            cancel = job.cancel
            self.cv.notify_all()
        if cancel:
            self._umarell_call("cancel_planning", {"tree_id": tree, "reason": "cancelled by the user"})

    def _umarell_seen(self, job: Job, st: dict) -> None:
        """umarell's get_plan_status of `job`: its status (terminal: the slot is free)."""
        with self.cv:
            if job.status not in self.LIVE:
                return
            status = st.get("status")
            if status in ("planning", "queued", "running"):
                job.umarell, job.misses = st, 0
                job.phase = str(st.get("phase") or "planning")
                return
            if status not in self.TERMINAL:
                job.misses += 1           # umarell does not know it (restarted?)
                if job.misses >= UMARELL_LOST_POLLS:
                    job.status = job.phase = "failed"
                    job.error = f"umarell no longer knows plan {job.tree_id} ({st.get('error') or status})"
                    job.hint = "the umarell planner restarted? plan again"
                    job.ended_s = time.time()
                    self.reserved.discard(job.tree_id)
                    self.cv.notify_all()
                return
            job.umarell, job.misses = st, 0
            job.status = job.phase = status
            job.ended_s = time.time()
            if status == "done":
                job.plan_path = f"{PLANS_DIR}/{job.tree_id}.json"
                self.stamp_inputs(job.cell_dir, job.plan_path)
            elif status == "failed":
                job.error = st.get("error") or "umarell failed"
                job.hint = st.get("note") or st.get("hint") or ""
            self.reserved.discard(job.tree_id)
            self.cv.notify_all()

    def _umarell_loop(self) -> None:
        """Follows the umarell jobs (frees their slots when umarell is done) and submits the queued ones."""
        while True:
            with self.cv:
                live = [j for j in self.jobs.values() if j.kind == "umarell" and j.status in self.LIVE]
                running = [j for j in live if j.status == "planning" and j.submitted]
                if not live:
                    self.cv.wait()
                    continue
            for job in running:
                try:
                    self._umarell_seen(job, self._umarell_call("get_plan_status", {"tree_id": job.tree_id}))
                except ToolError:
                    pass
                except Exception as e:  # noqa: BLE001 - never takes the loop down
                    print(f"[router] umarell status of {job.job_id}: {e}", file=sys.stderr, flush=True)
            while True:
                with self.cv:
                    busy = sum(1 for j in self.jobs.values() if j.kind == "umarell" and j.status == "planning")
                    queued = sorted((j for j in self.jobs.values() if j.kind == "umarell" and j.status == "queued"),
                                    key=lambda j: j.submitted_s)
                    if not queued or busy >= self.umarell_workers:
                        break
                    job = queued[0]
                    job.status, job.phase, job.started_s = "planning", "submitting", time.time()
                try:
                    self._ensure_umarell()
                    self._submit_umarell(job)
                except Exception as e:  # noqa: BLE001
                    with self.cv:
                        job.status = job.phase = "failed"
                        job.ended_s = time.time()
                        r = e.result if isinstance(e, ToolError) else {"error": f"{type(e).__name__}: {e}"}
                        job.error, job.hint = r.get("error"), r.get("hint") or ""
                        self.cv.notify_all()
            with self.cv:
                self.cv.wait(UMARELL_POLL_S)

    def health(self) -> dict:
        with self.lock:
            busy = [j.job_id for j in self.jobs.values() if j.status == "planning"]
            queued = [j.job_id for j in self.jobs.values() if j.status == "queued"]
        return {"ok": True, "server": SERVER_NAME, "version": SERVER_VERSION, "cell_planner": str(self.s.cell_planner),
                "cell_planner_commit": self.commit, "python": str(self.s.python),
                "python_present": self.s.python.exists(), "umarell": str(self.s.umarell),
                "active_cell": (self.active_cell() or (None,))[0], "planning": busy, "queued": queued,
                "workers": {"planner": self.workers, "umarell": self.umarell_workers}}


    # ---------------------------------------------------------------- Robo Fitter reachability
    def _backend_post(self, path: str, body: dict, timeout: float) -> Any:
        """POST to the FactoryOS backend; its error answers (4xx/5xx, text) become tool errors."""
        try:
            return http_json(self.s.backend_url.rstrip("/") + path, body, timeout)
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace").strip()
            raise ToolError(detail or f"HTTP {e.code}", f"the Physical AI Production Designer backend answered {e.code} on {path}") from None
        except (urllib.error.URLError, OSError) as e:
            raise ToolError(f"the Physical AI Production Designer backend is not reachable: {e}",
                            "Is factoryos-backend running? (systemctl status factoryos-backend)") from None

    @staticmethod
    def _scene_body(scene: Optional[str], cell: Optional[str]) -> dict:
        if not scene and not cell:
            raise ToolError("give scene (a Robo Fitter scene's name or a scene YAML's absolute path) or cell",
                            "The Robo Fitter's scenes are the folders of the catalogue's scenes/, by name.")
        if cell and not valid_name(cell):
            raise ToolError(f"invalid cell name {cell!r}", "Use the cell's folder name.")
        body: dict = {}
        if scene:
            body["scene_file"] = scene
        if cell:
            body["cell"] = cell
        return body

    def check_reachability(self, scene: Optional[str] = None, cell: Optional[str] = None, mode: str = "quick",
                           show_in_fitter: bool = False, source: str = "auto", chained: Optional[str] = None) -> dict:
        """Every robot of a Robo Fitter scene on every weld of its task, with the cell planner's IK (the backend's
        /api/v1/reach/check): which robot reaches which weld, with which joints, and why not. ``chained`` (mode
        full, a prepared cell): each weld also from its predecessor's end in an order -- ``program`` (the cell's
        task program, each weld on its nearest robot whose approach is clear from HOME), a plan's tree id or
        ``latest`` (that plan's order per robot) -- see :meth:`_check_chained`."""
        if mode not in ("quick", "full"):
            raise ToolError(f"mode {mode!r}: quick or full")
        if source not in ("auto", "product", "cell_program"):
            raise ToolError(f"source {source!r}: auto, product or cell_program")
        chained = str(chained).strip() if chained not in (None, False, "") else None
        if chained is not None and mode != "full":
            raise ToolError(f"chained {chained!r} needs mode full",
                            "The chained check walks the planner's approach predicate from each weld's predecessor: "
                            "mode full runs that predicate.")
        if chained is not None and chained not in ("program", "latest") and not valid_name(chained):
            raise ToolError(f"chained {chained!r}: program, latest or a plan's tree id")
        body = {**self._scene_body(scene, cell), "mode": mode, "source": source, "show_in_fitter": bool(show_in_fitter)}
        report = self._backend_post("/api/v1/reach/check", body, REACH_TIMEOUT_S)
        if isinstance(report, dict) and report.get("welds") is not None:
            report["approach"] = self._check_approach(report, scene, cell, mode, chained)
        return {"table": reach_table(report), "report": report}

    # ---------------------------------------------------------------- the planner's approach check (mode full)
    def _approach_cell(self, scene: Optional[str], cell: Optional[str]) -> tuple[Optional[tuple[str, Path]], str]:
        """The existing cell whose prepared planner model is the checked layout, or (None, why not). Never makes a
        cell (check_reachability is read-only)."""
        cells = self.cells_dir()
        if not cells:
            return None, "no cells folder"
        named = None
        if scene:
            p = Path(scene)
            fitted = self.fitted_dir().resolve()
            if p.is_absolute() and p.name == SCENE_FILE and p.parent.parent.resolve() == fitted:
                named = self.existing_cell_for_scene(p.parent.name)  # a Robo Fitter scene by its path
            elif p.is_absolute() and p.parent.resolve() == fitted:
                named = self.existing_cell_for_scene(p.name)       # a flat scene file of the old layout
            elif p.is_absolute():
                root = cell_root(str(p.parent))                    # a cell's own scene.yaml (or its view's)
                named = root.name if root.parent.resolve() == cells.resolve() else None
            else:
                named = self.existing_cell_for_scene(scene)
            if named is None:
                return None, f"scene {scene} is no cell's layout (a cell made from it and prepared is)"
            if cell and cell != named:
                return None, f"scene {scene} is not cell {cell}'s layout"
        name = cell or named
        if not name or not (cells / name).is_dir():
            return None, f"no cell {name}" if name else "no cell"
        return (name, cells / name), ""

    def _check_approach(self, report: dict, scene: Optional[str], cell: Optional[str], mode: str,
                        chained: Optional[str] = None) -> dict:
        """mode full on a prepared cell: every robot the IK says reaches a weld is checked with umarell's own
        terminal-approach predicate (approach_check.py: the weld-pose search, the planner's IK, its straight
        Cartesian descent against the cell's obstacle boxes, as umarell_u3 plans it); a blocked stroke turns the
        attempt into approach_blocked and the weld's status follows. The report's `approach`: what was checked."""
        if mode != "full":
            return {"checked": False, "why": "quick mode skips the approach stroke (mode full checks it with the "
                                             "planner's own predicate)"}
        found, why = self._approach_cell(scene, cell)
        if found is None:
            return {"checked": False, "why": f"{why}: the approach stroke was not checked"}
        name, root = found
        if not (root / PREPARED_MARKER).is_file():
            return {"checked": False, "cell": name,
                    "why": f"cell {name} is not prepared (Prepare Cell; plan prepares it): the approach stroke was "
                           "not checked"}
        stale = self.prepared_stale(name)
        if stale:
            return {"checked": False, "cell": name,
                    "why": f"cell {name}'s prepared planner model is stale ({stale}; Prepare Cell again): the "
                           "approach stroke was not checked"}
        if not self.s.umarell_python or not self.s.umarell_python.exists():
            return {"checked": False, "cell": name,
                    "why": f"umarell's venv {self.s.umarell_python} is missing: the approach stroke was not checked"}
        ids = cell_robot_ids(root)
        orients = task_orientations(root)
        pairs, unmapped = approach_pairs(report, ids, orients)
        out: dict = {"checked": True, "cell": name, "predicate": "umarell (umarell_u3's terminal approach)",
                     "strokes_m": list(APPROACH_STROKES_M), "pairs": len(pairs)}
        if unmapped:
            out["unmapped_robots"] = sorted(unmapped)
        if not pairs:
            out.update(blocked=0, elapsed_s=0.0)
            return out
        t0 = time.time()
        job = {"cell_id": planner_cell_id(name), "pairs": pairs, "strokes_m": list(APPROACH_STROKES_M),
               "approach_velocity_scale": APPROACH_VELOCITY_SCALE,
               "workers": env_int("FACTORYOS_APPROACH_WORKERS", CPUS)}
        try:
            lines = self._run_approach_check(root, job)
        except ToolError as e:
            return {**out, "checked": False, "why": f"the approach check failed: {e.result.get('error')}"}
        outside = {p["weld"] for p in pairs if not p.get("in_program")}
        out.update(apply_approach(report, [l for l in lines if not l.get("chained")], {v: k for k, v in ids.items()},
                                  outside))
        if outside:
            out["not_in_program"] = sorted(outside)
        if chained is not None:
            out["chained"] = self._check_chained(report, root, name, ids, chained, job)
        out["elapsed_s"] = round(time.time() - t0, 1)
        return out

    def _check_chained(self, report: dict, root: Path, name: str, ids: dict, chained: str, job: dict) -> dict:
        """The chained check (approach_check.py's chains): umarell_u3 plans a weld from where the robot's previous
        weld ended -- the weld-pose search, the IK and the straight descent warm-started there (another wrist /
        elbow branch than from HOME), after the on-axis lift-off out of the previous seam. Each robot's welds are
        walked in ``chained`` order (:func:`chain_order`): ``program`` -- the task program's order, each weld on
        the robot nearest to it whose approach is clear from HOME (what the reach check just found; umarell_u3's
        own allocation may differ), a plan's tree id or ``latest`` -- the order that plan runs its welds per robot.
        A weld that is clear from HOME but blocked chained is reported distinctly on its robot's attempt
        (``chained``: approach_blocked_chained / liftoff_blocked_chained, with the predecessor); the weld's
        status is not changed (from HOME it is reached: a different order, or a return to HOME first, plans it)."""
        scene_of = {v: k for k, v in ids.items()}
        ops = program_ops(root)
        try:
            chains, how = chain_order(report, root, chained, ids, ops)
        except ToolError as e:
            return {"checked": False, "order": chained, "why": e.result.get("error")}
        out: dict = {"checked": True, "order": chained, "how": how,
                     "chains": [{"robot": c["robot"], "welds": [w["weld"] for w in c["welds"]]} for c in chains]}
        if not chains:
            return {**out, "clear": 0, "blocked_chained": 0}
        cjob = {k: v for k, v in job.items() if k != "pairs"}
        cjob.update(chains=chains, retreats_m=list(APPROACH_RETREATS_M),
                    workers=max(1, min(len(chains), int(job.get("workers") or 1))))
        try:
            lines = [l for l in self._run_approach_check(root, cjob) if l.get("chained")]
        except ToolError as e:
            return {**out, "checked": False, "why": f"the chained check failed: {e.result.get('error')}"}
        out.update(apply_chained(report, lines, scene_of))
        return out

    def _run_approach_check(self, root: Path, job: dict) -> list:
        """approach_check.py in umarell's venv on the cell (a link to it under its planner id, as umarell reads
        $CELL_BASE_PATH/<cell id>): its NDJSON lines; the pairs it did not finish in time are left out."""
        base = Path(tempfile.mkdtemp(prefix="fos-approach-"))
        try:
            (base / job["cell_id"]).symlink_to(root.resolve(), target_is_directory=True)
            env = {k: v for k, v in os.environ.items() if k in ("PATH", "HOME", "LANG", "LC_ALL", "TMPDIR", "USER")}
            env.update(CELL_BASE_PATH=str(base), OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
                       PYTHONDONTWRITEBYTECODE="1", MUJOCO_GL="disable")
            cmd = [str(self.s.umarell_python), "-I", str(HERE / "approach_check.py"), "--umarell", str(self.s.umarell)]
            proc = subprocess.Popen(cmd, cwd=str(self.s.umarell), env=env, stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
            try:
                stdout, stderr = proc.communicate(json.dumps(job), timeout=APPROACH_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self._terminate(proc)
                stdout, stderr = proc.communicate()
            lines = []
            for raw in (stdout or "").splitlines():
                try:
                    v = json.loads(raw)
                except ValueError:
                    continue
                if isinstance(v, dict) and v.get("error"):
                    raise ToolError(str(v["error"]))
                if isinstance(v, dict) and "robot" in v:
                    lines.append(v)
            if proc.returncode not in (0, None) and not lines:
                raise ToolError(f"exit {proc.returncode}: {(stderr or '').strip()[-400:]}")
            return lines
        finally:
            shutil.rmtree(base, ignore_errors=True)

    def suggest_layout(self, scene: Optional[str] = None, cell: Optional[str] = None, max_move_m: Optional[float] = None,
                       write_as: Optional[str] = None) -> dict:
        """Robot moves (and pedestals) that bring the unreached welds into reach, checked against the reach domes;
        with write_as, the moved scene as a new scene (a new name; never overwrites one)."""
        body = self._scene_body(scene, cell)
        if max_move_m is not None:
            body["max_move_m"] = float(max_move_m)
        if write_as:
            if not scene_name(write_as):
                raise ToolError(f"write_as {write_as!r}: a new scene name (letters, digits, _, - and .; no folders)")
            body["write_as"] = write_as
        result = self._backend_post("/api/v1/reach/suggest", body, 120)
        moves = [m.get("text", "") for m in result.get("moves", [])]
        return {"moves": moves, "written": result.get("written"),
                "table": reach_table(result.get("report") or {}), "scene": result.get("scene"),
                "hint": "The moves are checked against the reach domes only: run check_reachability on the new scene."}

    # ---------------------------------------------------------------- deployability of an export
    @staticmethod
    def _deploy_body(cell: Optional[str], build: Optional[str]) -> dict:
        body: dict = {}
        if cell:
            if not valid_name(cell):
                raise ToolError(f"invalid cell name {cell!r}", "Use the cell's folder name.")
            body["cell"] = cell
        if build:
            if not re.fullmatch(r"[0-9]{8}T[0-9]{9}Z-[A-Za-z0-9_.-]+", build):
                raise ToolError(f"invalid export id {build!r}",
                                "An export id is <UTC time>-<plan>, e.g. 20261016T174822044Z-bt_1792156523_scene_fit; "
                                "leave it out for the cell's newest export.")
            body["build"] = build
        return body

    def check_deployability(self, cell: Optional[str] = None, build: Optional[str] = None) -> dict:
        """The backend's deployability check of an export (read-only, seconds): every rule pass / warn / fail with
        its hint and fixes."""
        report = self._backend_post("/api/v1/infra/deploy/check", self._deploy_body(cell, build), DEPLOY_CHECK_TIMEOUT_S)
        return {"deployable": bool(report.get("ok")), "build": report.get("build"), "cell": report.get("cell"),
                "table": deploy_table(report), "report": report}

    def fix_deployability(self, cell: Optional[str] = None, build: Optional[str] = None,
                          rules: Optional[list] = None) -> dict:
        """Apply the failing rules' fixes (Hardware Configuration saved with a backup, export options), export the plan
        again with the same options and check the new export. Changes settings: only with the user's consent."""
        body = self._deploy_body(cell, build)
        if rules is not None:
            if not isinstance(rules, list) or not all(isinstance(r, str) and re.fullmatch(r"[a-z_]+\.[a-z_]+", r)
                                                      for r in rules):
                raise ToolError(f"rules {rules!r}: a list of rule ids, e.g. [\"cell.safety\", \"cell.network\"]")
            body["rules"] = rules
        result = self._backend_post("/api/v1/infra/deploy/fix", body, DEPLOY_FIX_TIMEOUT_S)
        report = result.get("report") or {}
        out = {"changes": result.get("changes") or [], "backup": result.get("backup"), "new_build": result.get("build"),
               "deployable": bool(report.get("ok")), "table": deploy_table(report) if report else "", "result": result}
        if result.get("error"):
            out["error"] = result["error"]
        return out

REACH_TIMEOUT_S = 630.0   # the backend's IK limit (600 s) and some
APPROACH_TIMEOUT_S = float(os.environ.get("FACTORYOS_APPROACH_TIMEOUT_S") or 420.0)  # the planner's approach check
APPROACH_STROKES_M = (0.12, 0.2, 0.08)   # umarell_u3's approach offset, then its recovery offsets (U3's order)
APPROACH_VELOCITY_SCALE = 0.25           # umarell_u3's approach speed (the stroke's speed margin)
APPROACH_RETREATS_M = (0.08, 0.05, 0.03)  # umarell_u3's shorter lift-offs for a liftoff_blocked leg (its order)
DEPLOY_CHECK_TIMEOUT_S = 180.0   # plan2cell cell check reads the KUKA VCs' $config.dat over SSM
DEPLOY_FIX_TIMEOUT_S = 600.0     # a re-export and two checks


def deploy_table(report: dict) -> str:
    """A deployability report as text: the verdict, then one line per rule; the problems with hint, fixes and details."""
    if not report:
        return "no report"
    rules = report.get("rules") or []
    mark = {"pass": "✓", "skip": "·", "warn": "⚠", "fail": "✗"}
    fails = [r for r in rules if r.get("status") == "fail"]
    warns = [r for r in rules if r.get("status") == "warn"]
    verdict = ("deployable" + (f", {len(warns)} warning(s)" if warns else "")) if not fails else \
        f"NOT deployable: {len(fails)} problem(s), {len(warns)} warning(s)"
    lines = [f"export {report.get('build')} of cell {report.get('cell')} (PLAN_ID {report.get('planId')}, profile "
             f"{report.get('profile')}): {verdict}"]
    if report.get("target"):
        lines.append(f"target: {report['target']}")
    for r in rules:
        lines.append(f"{mark.get(r.get('status'), '?')} {r.get('title')} [{r.get('id')}]: {r.get('message')}")
        if r.get("status") in ("warn", "fail"):
            if r.get("generatorBug"):
                lines.append(f"    generator: {r['generatorBug']}")
            if r.get("hint"):
                lines.append(f"    fix: {r['hint']}")
            for f in r.get("fixes") or []:
                if f.get("kind") == "hardware":
                    lines.append(f"    auto-fix: Hardware Configuration {f.get('field')} = {json.dumps(f.get('value'))}")
                elif f.get("kind") == "export":
                    lines.append(f"    auto-fix: export option {f.get('option')} = {json.dumps(f.get('value'))}")
                else:
                    lines.append(f"    auto-fix: export again ({f.get('reason')})")
            for d in (r.get("details") or [])[:8]:
                lines.append(f"    {d}")
    return "\n".join(lines)


def reach_classify(robots: int, reached: int) -> str:
    """A weld's status (the backend's reach::classify): green when every robot of a one-robot scene or at least two
    robots reach it, orange with one, red with none."""
    if robots == 0:
        return "unknown"
    if reached == 0:
        return "no_robot"
    if robots == 1 or reached >= 2:
        return "all_robots"
    return "one_robot"


def cell_robot_ids(root: Path) -> dict:
    """The prepared cell's robots: the scene's robot id (the reach report's robot_id) -> the cell's (robots.yaml,
    umarell's robot_ns), from agentic_planner/cell.yaml."""
    try:
        manifest = parse_yaml((root / PREPARED_MARKER).read_text()) or {}
    except (OSError, ValueError):
        return {}
    out = {}
    for r in manifest.get("robots") or [] if isinstance(manifest, dict) else []:
        if isinstance(r, dict) and r.get("id"):
            out[str(r.get("scene_id") or r["id"])] = str(r["id"])
    return out


def task_orientations(root: Path) -> list:
    """Every weld of the cell's task programs: (program, op id, position, orientation)."""
    out = []
    tasks = root / TASKS_DIR
    for f in sorted(tasks.glob("*.json")) if tasks.is_dir() else []:
        try:
            ops = json.loads(f.read_text()).get("operations") or []
        except (OSError, ValueError, AttributeError):
            continue
        for o in ops:
            ori = o.get("weld_orientation") or o.get("orientation") if isinstance(o, dict) else None
            if isinstance(ori, dict) and isinstance(o.get("position"), list) and len(o["position"]) == 3:
                out.append((f.stem, str(o.get("id")), [float(v) for v in o["position"]], ori))
    return out


def weld_orientation(weld: dict, orients: list) -> tuple[dict, bool]:
    """A reach report weld's tool orientation, as the planner gets it: its task program operation's (same program
    and id, else the same id or position within 5 mm), else the one derived from its approach (orientation_for);
    and whether the weld is one of the cell's program welds."""
    wid, task, pos = str(weld.get("id")), weld.get("task"), weld.get("position") or [0.0, 0.0, 0.0]
    near = lambda q: max(abs(a - b) for a, b in zip(q, pos)) <= 0.005  # noqa: E731
    for test in (lambda t, i, q: t == task and i == wid, lambda t, i, q: i == wid and near(q),
                 lambda t, i, q: near(q)):
        for t, i, q, ori in orients:
            if test(t, i, q):
                return ori, True
    return orientation_for(weld.get("approach") or [0.0, 0.0, -1.0]), False


def approach_pairs(report: dict, ids: dict, orients: list) -> tuple[list, set]:
    """approach_check.py's pairs: every attempt the IK reached (robot -> the cell's id; in_program: the weld is
    one of the prepared cell's program welds), and the robots of the report the cell does not have."""
    pairs, unmapped = [], set()
    for w in report.get("welds") or []:
        weld = w.get("weld") or {}
        for a in w.get("attempts") or []:
            if not a.get("reachable"):
                continue
            rid = ids.get(str(a.get("robot_id")))
            if rid is None:
                unmapped.add(str(a.get("robot_id")))
                continue
            ori, known = weld_orientation(weld, orients)
            pairs.append({"robot": rid, "weld": str(weld.get("id")), "position": weld.get("position"),
                          "approach_vector": weld.get("approach"), "orientation": ori, "in_program": known})
    return pairs, unmapped


def program_ops(root: Path) -> dict:
    """The weld operations of the cell's task programs by id (the first program that has the id):
    {id: {"weld", "position", "approach_vector", "orientation", "program", "index"}}."""
    out: dict = {}
    tasks = root / TASKS_DIR
    for f in sorted(tasks.glob("*.json")) if tasks.is_dir() else []:
        try:
            ops = json.loads(f.read_text()).get("operations") or []
        except (OSError, ValueError, AttributeError):
            continue
        for i, o in enumerate(ops):
            if not isinstance(o, dict) or str(o.get("id")) in out:
                continue
            ori = o.get("weld_orientation") or o.get("orientation")
            pos, av = o.get("position"), o.get("approach_vector") or [0.0, 0.0, -1.0]
            if isinstance(pos, list) and len(pos) == 3 and isinstance(ori, dict):
                out[str(o["id"])] = {"weld": str(o["id"]), "position": [float(v) for v in pos],
                                     "approach_vector": [float(v) for v in av], "orientation": ori,
                                     "program": f.stem, "index": i}
    return out


def robot_bases(root: Path) -> dict:
    """The cell's robots' base positions from its robots.yaml ({robot id: [x, y, z]}), by a line scan (the file is
    large; only `<robot>:` keys and their `base_position: [x, y, z]` are read)."""
    try:
        text = (root / "agentic_planner" / "config" / "robots.yaml").read_text()
    except OSError:
        return {}
    keys: dict = {}
    out: dict = {}
    for raw in text.splitlines():
        line = _strip_comment(raw)
        m = re.match(r"^(\s*)([A-Za-z0-9_.-]+):\s*$", line)
        if m:
            keys[len(m.group(1))] = m.group(2)
            continue
        m = re.match(r"^(\s*)base_position:\s*\[([^\]]*)\]", line)
        if m and (len(m.group(1)) - 2) in keys:
            try:
                out.setdefault(keys[len(m.group(1)) - 2], [float(v) for v in m.group(2).split(",")])
            except ValueError:
                continue
    return out


def plan_weld_order(plan: Any) -> dict:
    """{robot_ns: [operation id, ...]}: the weld legs of a saved plan (its behaviour tree's ExecuteTrajectory nodes
    with operation_type weld) in the order the tree runs them."""
    out: dict = {}

    def walk(node):
        if isinstance(node, dict):
            ports = node.get("ports") if isinstance(node.get("ports"), dict) else {}
            if node.get("type") == "ExecuteTrajectory" and ports.get("operation_type") == "weld":
                ns, op = ports.get("robot_ns"), ports.get("operation_id")
                if ns and op and op not in out.setdefault(str(ns), []):
                    out[str(ns)].append(str(op))
            for v in node.values():
                if isinstance(v, (dict, list)):
                    walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)

    walk(plan.get("behaviour_tree", plan) if isinstance(plan, dict) else plan)
    return out


def chain_order(report: dict, root: Path, chained: str, ids: dict, ops: dict) -> tuple[list, str]:
    """approach_check.py's chains for ``chained`` (see Router._check_chained) and how they were made."""
    if chained == "program":
        bases = robot_bases(root)
        by_id = {}
        for w in report.get("welds") or []:
            wid = str((w.get("weld") or {}).get("id"))
            by_id.setdefault(wid, w)
            by_id.setdefault(str((w.get("weld") or {}).get("label")), w)
        per: dict = {}
        for op in sorted(ops.values(), key=lambda o: (o["program"], o["index"])):
            w = by_id.get(op["weld"])
            if w is None:
                continue
            clear = [ids.get(str(a.get("robot_id"))) for a in w.get("attempts") or [] if a.get("reachable")]
            clear = [r for r in clear if r]
            if not clear:
                continue
            p = op["position"]

            def dist(r):
                b = bases.get(r)
                return math.hypot(p[0] - b[0], p[1] - b[1]) if b else float("inf")

            per.setdefault(min(clear, key=lambda r: (dist(r), r)), []).append(op)
        return ([{"robot": r, "welds": ws} for r, ws in sorted(per.items())],
                "the task program's order; each weld on the nearest robot whose approach is clear from HOME")
    plans = root / PLANS_DIR
    if chained == "latest":
        files = sorted(plans.glob("*.json"), key=lambda f: f.stat().st_mtime) if plans.is_dir() else []
        files = [f for f in files if not f.name.startswith(".")]
        if not files:
            raise ToolError("the cell has no saved plan (chained latest)")
        path = files[-1]
    else:
        path = plans / f"{chained}.json"
        if not path.is_file():
            raise ToolError(f"no plan {chained} in {PLANS_DIR}/")
    try:
        order = plan_weld_order(json.loads(path.read_text()))
    except (OSError, ValueError) as e:
        raise ToolError(f"plan {path.stem} cannot be read: {e}") from None
    chains = [{"robot": ns, "welds": [ops[o] for o in welds if o in ops]} for ns, welds in sorted(order.items())]
    return [c for c in chains if c["welds"]], f"plan {path.stem}'s order per robot"


def chained_detail(line: dict) -> str:
    """A chained weld's outcome in one line, e.g. `after W01: approach_blocked_chained: link_5 vs ... (0.12 m
    stroke)`."""
    after = f"after {line['predecessor']}" if line.get("predecessor") else "from HOME"
    if line.get("clear"):
        return f"{after}: clear ({line.get('stroke_m')} m stroke)"
    if line.get("reason_code") == "liftoff_blocked":
        return f"{after}: liftoff_blocked: {line.get('reason') or 'the lift-off out of the previous weld is blocked'}"
    return f"{after}: {approach_detail(line)}"


def apply_chained(report: dict, lines: list, scene_ids: dict) -> dict:
    """The chained check's lines into the reach report: each weld's attempt of its chain's robot carries `chained`
    ({predecessor, clear, reason (approach_blocked_chained | liftoff_blocked_chained | approach_blocked: blocked
    from HOME too), detail}); a weld blocked only chained is listed in the weld's `chained_blocked`. Returns the
    counts for the report's `approach.chained`."""
    by = {}
    for w in report.get("welds") or []:
        weld = w.get("weld") or {}
        for k in (weld.get("id"), weld.get("label")):
            if k is not None:
                by.setdefault(str(k), w)
    clear = blocked = liftoff = home_too = errors = 0
    failures = []
    for line in lines:
        w = by.get(str(line.get("weld")))
        rid = scene_ids.get(line["robot"], line["robot"])
        a = next((x for x in (w or {}).get("attempts") or [] if str(x.get("robot_id")) == str(rid)), None)
        from_home = bool(a and a.get("reachable"))
        info = {k: line.get(k) for k in ("predecessor", "clear", "stroke_m", "reason_code", "blocker", "liftoff",
                                          "search", "elapsed_s") if line.get(k) is not None}
        if line.get("clear") is True:
            clear += 1
            reason = None
        elif line.get("clear") is False:
            if not from_home:
                home_too += 1
                reason = "approach_blocked"
            elif line.get("reason_code") == "liftoff_blocked":
                liftoff += 1
                reason = "liftoff_blocked_chained"
            else:
                blocked += 1
                reason = "approach_blocked_chained"
        else:
            errors += 1
            reason = "error"
        detail = chained_detail(line) if line.get("clear") is not None else f"error: {line.get('reason')}"
        rec = {"robot": rid, **info, "reason": reason, "detail": detail}
        if a is not None:
            a["chained"] = {k: v for k, v in rec.items() if k != "robot"}
        if w is not None:
            w["chained"] = rec
            if reason in ("approach_blocked_chained", "liftoff_blocked_chained"):
                w["chained_blocked"] = rec
        if reason not in (None, "error"):
            failures.append({"weld": line.get("weld"), "robot": rid, "robot_ns": line["robot"],
                             "predecessor": line.get("predecessor"), "reason": reason})
    return {"clear": clear, "blocked_chained": blocked, "liftoff_blocked_chained": liftoff,
            "blocked_from_home_too": home_too, "errors": errors, "failures": failures}


def approach_detail(line: dict) -> str:
    """A blocked stroke in one line, e.g. `link_5 vs body_weld_3_f0026 (workpiece) at stroke step 2/6, -0.9 cm`."""
    b = line.get("blocker") or {}
    code = line.get("reason_code")
    at = f" at stroke step {b['step']}/{b.get('steps', 6)}" if b.get("step") else ""
    cm = f", {100 * float(b['clearance_m']):+.1f} cm" if b.get("clearance_m") is not None else ""
    if b.get("object_id") and b.get("type") == "self":
        text = f"the arm folds into itself ({b['object_id']}){at}{cm}"
    elif b.get("object_id"):
        kind = " (workpiece)" if b.get("is_workpiece") else ""
        text = f"{b.get('link') or 'the arm'} vs {b['object_id']}{kind}{at}{cm}"
    elif code == "approach_unreachable" or b.get("type") == "ik":
        text = f"no IK solution at the weld's lean{at}"
    else:
        text = line.get("reason") or code or "blocked"
    strokes = [s.get("stroke_m") for s in line.get("strokes") or []]
    if strokes:
        text += f" ({strokes[0]} m stroke" + (f"; {', '.join(str(x) for x in strokes[1:])} m blocked too"
                                              if len(strokes) > 1 else "") + ")"
    se = line.get("search") or {}
    if se.get("status") == "none_usable":
        text += f"; tool spin/tilt searched: none of {se.get('candidates')} poses usable"
    return text


#: A blocked weld that is not a program weld of the prepared cell: Prepare Cell builds the near-weld obstacle boxes
#: (<fixture>_f*, workpiece) only around the program's welds, elsewhere the product is the coarse voxel shell.
NOT_IN_PROGRAM = ("not a weld of the prepared cell's task program: the obstacle model around it is the coarse "
                  "voxel shell (Prepare Cell with it in the program, then check again)")


def apply_approach(report: dict, lines: list, scene_ids: dict, outside: Optional[set] = None) -> dict:
    """The approach check's lines into the reach report: a blocked attempt is not reachable (reason approach_blocked,
    its detail), every checked attempt carries `approach`, each weld's reached_by and status are re-derived.
    `outside`: the weld ids that are not program welds of the prepared cell. Returns the counts for the report's
    `approach`."""
    outside = outside or set()
    by = {(scene_ids.get(l["robot"], l["robot"]), str(l.get("weld"))): l for l in lines}
    blocked = clear = errors = unchecked = 0
    for w in report.get("welds") or []:
        wid = str((w.get("weld") or {}).get("id"))
        for a in w.get("attempts") or []:
            if not a.get("reachable"):
                continue
            line = by.get((str(a.get("robot_id")), wid))
            if line is None:
                unchecked += 1
                a["approach"] = {"checked": False}
                continue
            info = {k: line.get(k) for k in ("clear", "stroke_m", "reason_code", "blocker", "search", "strokes",
                                              "elapsed_s") if line.get(k) is not None}
            if line.get("clear") is True:
                clear += 1
                a["approach"] = {"checked": True, **info}
            elif line.get("clear") is False:
                blocked += 1
                detail = approach_detail(line) + (f"; {NOT_IN_PROGRAM}" if wid in outside else "")
                a.update(reachable=False, reasons=["approach_blocked"],
                         detail=f"approach_blocked: {detail}" + (f"; IK: {a['detail']}" if a.get("detail") else ""))
                a["approach"] = {"checked": True, **info, "detail": detail}
            else:
                errors += 1
                a["approach"] = {"checked": False, "error": line.get("reason")}
        attempts = w.get("attempts") or []
        w["reached_by"] = [a.get("robot_id") for a in attempts if a.get("reachable")]
        w["status"] = reach_classify(len(attempts), len(w["reached_by"]))
    return {"clear": clear, "blocked": blocked, "errors": errors, "unchecked": unchecked}


def reach_table(report: dict) -> str:
    """A reach report as a short text table: per weld its status, and per robot reached (joints) or why not."""
    welds = report.get("welds") or []
    if not welds:
        return "no welds" + (f" ({'; '.join(report.get('notes') or [])})" if report.get("notes") else "")
    count = {s: sum(1 for w in welds if w.get("status") == s) for s in ("all_robots", "one_robot", "no_robot")}
    lines = [f"{len(welds)} welds ({report.get('source')}, {report.get('method')}, {report.get('mode')}, "
             f"{report.get('elapsed_s', 0):.0f} s): {count['all_robots']} green, {count['one_robot']} orange, "
             f"{count['no_robot']} red"]
    ap = report.get("approach") or {}
    if ap.get("checked"):
        more = "".join(f", {ap[k]} {k}" for k in ("errors", "unchecked") if ap.get(k))
        lines.append(f"approach: the {ap.get('pairs', 0)} reached weld x robot pairs checked with the planner's own "
                     f"terminal-approach predicate ({ap.get('predicate', 'umarell')}, strokes "
                     f"{', '.join(str(x) for x in ap.get('strokes_m') or [])} m) in {ap.get('elapsed_s', 0):.0f} s: "
                     f"{ap.get('clear', 0)} clear, {ap.get('blocked', 0)} approach_blocked{more}")
    elif ap:
        lines.append(f"approach: not checked ({ap.get('why')})")
    ch = ap.get("chained") or {}
    if ch.get("checked"):
        n = sum(len(c.get("welds") or []) for c in ch.get("chains") or [])
        more = "".join(f", {ch[k]} {k}" for k in ("blocked_from_home_too", "errors") if ch.get(k))
        lines.append(f"chained ({ch.get('order')}: {ch.get('how')}): {n} welds, each from its predecessor's end: "
                     f"{ch.get('clear', 0)} clear, {ch.get('blocked_chained', 0)} approach_blocked_chained, "
                     f"{ch.get('liftoff_blocked_chained', 0)} liftoff_blocked_chained{more}")
        for c in ch.get("chains") or []:
            lines.append(f"  {c['robot']}: {' -> '.join(c.get('welds') or [])}")
    elif ch:
        lines.append(f"chained: not checked ({ch.get('why')})")
    colour = {"all_robots": "green", "one_robot": "orange", "no_robot": "red", "unknown": "grey"}
    for w in welds:
        cells = []
        for a in w.get("attempts") or []:
            if a.get("reachable"):
                q = ", ".join(f"{v:.1f}" for v in a.get("joints_deg") or [])
                stroke = (a.get("approach") or {}).get("stroke_m")
                cells.append(f"{a['robot_id']}: reached" + (f" (q° {q})" if q else "")
                             + (f", approach clear ({stroke} m stroke)" if stroke else ""))
            elif "approach_blocked" in (a.get("reasons") or []):
                cells.append(f"{a['robot_id']}: approach_blocked: {(a.get('approach') or {}).get('detail', '')}")
            else:
                why = ", ".join(a.get("reasons") or ["no_solution"])
                cells.append(f"{a['robot_id']}: {why} (dome margin {a.get('margin_m', 0):+.3f} m)")
        weld = w.get("weld") or {}
        cw = w.get("chained")
        if cw:
            reason = cw.get("reason")
            cells.append(f"chained {cw['robot']} {cw.get('detail')}" if reason is None or reason == "approach_blocked"
                         else f"chained {cw['robot']} {reason} {cw.get('detail')} (clear from HOME)")
        lines.append(f"{weld.get('id')} [{colour.get(w.get('status'), w.get('status'))}]: " + "; ".join(cells))
    for s in report.get("suggestions") or []:
        lines.append(f"suggestion: {s.get('text')}"
                     + (" (from the IK, before the approach check)" if ap.get("blocked") else ""))
    for n in report.get("notes") or []:
        lines.append(f"note: {n}")
    if report.get("error"):
        lines.append(f"error: {report['error']}")
    return "\n".join(lines)


# ───────────────────────────────────────── MCP tools ─────────────────────────────────────────
_CELL = {"type": "string", "description": "FactoryOS cell folder name (default: the active cell)"}
TOOLS = [
    {"name": "list_planners",
     "description": "The planners this router can run (umarell LLM planner, umarell_u3 (umarell's tools with the "
                    "upstream fixes, no LLM), umarell_jam (the same from a prepared component cache: seconds; "
                    "falls back to umarell_u3 on a miss), factoryos-cell-planner's learned policies and trajgen), with their verified numbers and whether each is available for the cell.",
     "inputSchema": {"type": "object", "properties": {"cell": _CELL}}},
    {"name": "describe_planner",
     "description": "One planner in detail: its btexport spec, checkpoint, default profile and limitations.",
     "inputSchema": {"type": "object", "properties": {"name": {"type": "string"}, "cell": _CELL},
                     "required": ["name"]}},
    {"name": "recommend_planner",
     "description": "Which planner fits the cell, task program and request. Returns ranked candidates, `recommended`, "
                    "and `ambiguous` plus a `question` to ask the user when the choice is theirs. Call it first.",
     "inputSchema": {"type": "object", "properties": {
         "cell": _CELL,
         "task": {"type": "string", "description": "task program name (agentic_planner/tasks/<task>.json)"},
         "request": {"type": "string", "description": "the user's request in their words (constraints, wishes)"},
         "planner": {"type": "string", "description": "a planner the user asked for, or auto"},
         "profile": {"type": "string", "enum": ["umarell", "full"]}}}},
    {"name": "plan",
     "description": "Start planning. planner: auto (only when recommend_planner was not ambiguous) or a name from "
                    "list_planners. Returns a job_id at once (status planning or queued, running / queued_behind, "
                    "workers, eta_s); follow it with plan_status. Any cell, not only the active one, and several "
                    "plans at once (other cells, other planners, the same cell too): the router runs them in "
                    "parallel up to its workers and queues the rest. The finished plan is saved in that cell's "
                    "agentic_planner/plans/ for the Behaviour Tree Executor. timeout_s is the job's effective timeout: "
                    "umarell_u3's grows when it gets fewer than 8 U3 workers (x 8 / u3_workers; a 2-vCPU workspace: "
                    "4 x), and eta_s with it. umarell_u3 on a workspace with fewer than 8 CPUs (or options.remote "
                    "true) plans on an ephemeral 32-vCPU instance instead (remote: used, instance_type; the plan is "
                    "published as a local job's, a few minutes of boot / install included in eta_s); if that cannot "
                    "start, it plans here and remote.fallback / remote_note say why: tell the user.",
     "inputSchema": {"type": "object", "properties": {
         "planner": {"type": "string"}, "cell": _CELL, "task": {"type": "string"},
         "profile": {"type": "string", "enum": ["umarell", "full"],
                     "description": "policies: umarell = umarell's timing envelope (default for most), full = URDF "
                                    "velocity limits (hier_q's default); umarell_u3: full (default) = options.speed max, "
                                    "umarell = options.speed normal"},
         "request": {"type": "string"},
         "options": {"type": "object", "description": "allow_slow (trajgen live run, 20-30 min); umarell: "
                                                      "operations, constraints, model, request_applied (umarell "
                                                      "gets no free text: encode `request` as operations / "
                                                      "constraints or a saved task program); umarell_u3: speed "
                                                      "max | normal (default max = the robots' maximum speed: U3 "
                                                      "plans transit at 1.0 x the joint velocity limits, the weld "
                                                      "stroke at 0.28, and the plan is timed to the robots' joint "
                                                      "limits (the tighter of the URDF and the cell's robots.yaml: "
                                                      "measured accelerations, datasheet speeds), btexport profile "
                                                      "full; normal = U3's slower planning speeds (transit 0.3) in "
                                                      "umarell's own timing, profile umarell: only when the user "
                                                      "asks for a slower / normal-speed / conservative plan); "
                                                      "umarell_u3: remote true | false (default: auto, remote on "
                                                      "a workspace with fewer than 8 CPUs): true = plan on an "
                                                      "ephemeral big instance (for a big cell from a small "
                                                      "workspace), false = always on this workspace; "
                                                      "umarell_jam: always profile umarell (planned and timed "
                                                      "at the cell limits; speed max | normal is recorded), "
                                                      "welds / robots (lists "
                                                      "of ids: plan only these; no fallback then), timeout_s, "
                                                      "repair_budget_s (time to repair cache misses, default "
                                                      "300), prepare_budget_s, fallback true | false (default "
                                                      "true: when umarell_jam has no plan, e.g. a cache miss not "
                                                      "repaired in budget, umarell_u3 plans the job in the same "
                                                      "job: minutes, the phase says 'fallback: umarell_u3' and "
                                                      "eta_s grows; tell the user; false: no plan, then ask)"}},
         "required": ["planner"]}},
    {"name": "plan_status",
     "description": "A planning job's status (queued | planning | done | failed | cancelled), progress, the saved "
                    "plan's path and its export checks, the jobs running beside it / queued before it and its ETA "
                    "(eta_s; starts_in_s while queued), the effective timeout_s (umarell_jam: fallback {enabled, "
                    "why, used} and export.jam {status, reason, cache {hits, misses}}; umarell_u3: u3_workers, and remote "
                    "{used, instance_type, phase, fallback} with remote_note when remote planning fell back to this "
                    "workspace; phases launching / booting / installing / planning / checking / downloading). Waits up to wait_s (default 20, max 60) for a change of "
                    "status or phase before it answers: call it again right away, without a remark, while nothing "
                    "changed. Following several jobs: give the others a short wait_s (e.g. 5) or use jobs.",
     "inputSchema": {"type": "object", "properties": {"job_id": {"type": "string"},
                                                      "wait_s": {"type": "number"}}, "required": ["job_id"]}},
    {"name": "jobs",
     "description": "Every planning job the router knows (since it started), newest first: job_id, cell, planner, "
                    "status, phase, elapsed_s, eta_s (starts_in_s while queued), plan_path, error; and the workers "
                    "(planner: subprocess jobs at once, umarell: umarell plans at once). One call to follow several "
                    "jobs started together.",
     "inputSchema": {"type": "object", "properties": {
         "cell": {"type": "string", "description": "only this cell's jobs"},
         "active_only": {"type": "boolean", "description": "only queued / planning jobs"}}}},
    {"name": "cancel_plan",
     "description": "Cancel a planning job.",
     "inputSchema": {"type": "object", "properties": {"job_id": {"type": "string"}}, "required": ["job_id"]}},
    {"name": "describe_cell",
     "description": "The cell's folder (cell_dir: its files are there, not in the umarell checkout), scene.yaml "
                    "(robots, fixtures), obstacles.yaml, the fixtures' bounding box, its task programs, its products "
                    "(the fixtures that are Product & Task products, with their welds in the world frame) and the "
                    "task program format. task_sources says which task programs the backend derived from the "
                    "products (<fixture>__<task>, from_product true): those are the cell's tasks, the user edits their "
                    "welds in Product & Task. A cell without a task program gets it from task_program_from_products.",
     "inputSchema": {"type": "object", "properties": {"cell": _CELL}}},
    {"name": "task_program_from_products",
     "description": "The cell's task program from the Product & Task welds of its products (the scene.yaml "
                    "fixtures that are products), placed in the world frame by the fixtures' poses, with the tool "
                    "orientation from each weld's normal; marks the obstacle boxes the welds are on as is_workpiece. "
                    "When the backend already derived the task programs from the products (<fixture>__<task>.json, "
                    "one per product task) it returns those and writes nothing; otherwise (or with name) it writes "
                    "one program of all the products' welds. For a cell without a task program, ALWAYS call this "
                    "first. name defaults to <product>_spot_welding.",
     "inputSchema": {"type": "object", "properties": {
         "cell": _CELL, "name": {"type": "string", "description": "program name (default <product>_spot_welding)"}}}},
    {"name": "save_task_program",
     "description": "Save a task program (the weld targets) for the cell as agentic_planner/tasks/<name>.json, for "
                    "every planner (policies, trajgen, umarell). operations: [{id, position [x,y,z] m, "
                    "approach_vector, orientation {x,y,z,w}, ...}] (defaults filled; orientation derived from "
                    "approach_vector when left out; see describe_cell). Refuses an orientation whose tool axis is "
                    "more than 10° off approach_vector and a position inside a non-workpiece obstacle box. For a "
                    "cell with products use task_program_from_products instead.",
     "inputSchema": {"type": "object", "properties": {
         "cell": _CELL, "name": {"type": "string", "description": "program name (default spot_welding)"},
         "description": {"type": "string"},
         "operations": {"type": "array", "items": {"type": "object"}}},
         "required": ["operations"]}},
    {"name": "list_plans",
     "description": "The cell's saved plans (newest first) with the planner that made each.",
     "inputSchema": {"type": "object", "properties": {"cell": _CELL}}},
    {"name": "check_reachability",
     "description": "Robo Fitter: check whether the robots of a scene reach every weld of its task, with the cell "
                    "planner's IK (each robot moved to each weld). Returns a table (weld x robot: reached with its "
                    "joints, or the reason: out_of_reach, joint_limits, self_collision, floor, pose_tolerance, "
                    "weld_inside_product, tool_hits_product, approach_blocked; and the margin to the reach dome) "
                    "plus the report. quick: the weld pose only, ~1 s per robot and weld (the approach stroke is NOT "
                    "checked: a weld it reports may still be refused by the planner). full: with the IK's retract, "
                    "~20 s per robot and weld, and, for a prepared cell (cell, or a scene a prepared cell is made "
                    "from), every reached weld x robot again with the planner's own terminal-approach predicate "
                    "(umarell_u3's: tool spin/tilt search, its IK, the straight on-axis stroke into the weld, "
                    "strokes 0.12/0.2/0.08 m, against the cell's obstacle boxes with the arm's link envelope): "
                    "approach_blocked names what blocks it, e.g. `link_5 vs body_weld_3_f0026 (workpiece) at "
                    "stroke step 2/6, -0.9 cm` (the clearance below the required radius + margin), and that robot "
                    "no longer counts as reaching the weld (the green / orange / red counts follow). ~20 s per "
                    "reached pair (~60 s when blocked), in parallel; an unprepared or stale cell: not checked (the "
                    "table says so). Backend up to 10 min, approach check up to 7 min.",
     "inputSchema": {"type": "object", "properties": {
         "scene": {"type": "string", "description": "a Robo Fitter scene's name (e.g. jam2: scenes/jam2/scene.yaml) "
                                                    "or a scene YAML's absolute path (a cell's scene.yaml)"},
         "cell": {"type": "string", "description": "a cell folder name: its scene.yaml (or, with scene, the cell "
                                                   "whose task program to check)"},
         "mode": {"type": "string", "enum": ["quick", "full"]},
         "source": {"type": "string", "enum": ["auto", "product", "cell_program"],
                    "description": "the welds: the products placed in the scene, or the cell's task program "
                                   "(auto: the program when the scene is a cell's)"},
         "chained": {"type": "string",
                     "description": "mode full on a prepared cell: also check each weld from its predecessor's end "
                                    "pose, as umarell_u3 plans it (lift-off out of the previous seam, then the "
                                    "stroke warm-started from the previous weld's end). `program`: the task "
                                    "program's order, each weld on its nearest robot whose approach is clear from "
                                    "HOME; a plan's tree id or `latest`: that plan's order per robot. A weld clear "
                                    "from HOME but blocked chained is reported as approach_blocked_chained / "
                                    "liftoff_blocked_chained with its predecessor (~20 s per weld, chains in "
                                    "parallel)."},
         "show_in_fitter": {"type": "boolean", "description": "also show the result in the Robo Fitter"}}}},
    {"name": "check_deployability",
     "description": "Control Code: can an export (generated PLC + KRL code) be deployed to, and run on, the virtual cell? "
                    "Read-only, seconds: the rules of the cell's template; on the legacy templates (CODESYS head PLC, KUKA "
                    "VCs): KRL names against the KUKA VCs' $config.dat (KSS error 2137), the safety settings "
                    "(safety module / bypass / External: ERR_SAFETY_TIMEOUT), the head's network and gateway, the robots' "
                    "stations and IPs against the controllers, CODESYS robot simulators on KUKA addresses, job number and "
                    "profile, PLAN_ID, the cell's state and the plan's status. Returns a table (✓ / ⚠ / ✗ per rule with the "
                    "fix: which setting, where in Physical AI Production Designer; auto-fix lines where fix_deployability can do it) and the report.",
     "inputSchema": {"type": "object", "properties": {
         "cell": _CELL,
         "build": {"type": "string", "description": "an export id (<UTC time>-<plan>, e.g. "
                                                    "20261016T174822044Z-bt_1792156523_scene_fit); default: the newest"}}}},
    {"name": "fix_deployability",
     "description": "Apply the fixes of the failing deployability rules (the auto-fix lines of check_deployability): "
                    "Hardware Configuration values saved (the previous hardware.json kept as a backup), export options; "
                    "then the plan is exported again with the same options and the new export checked. Changes the cell's "
                    "settings and makes a new export (never deploys or touches the cell): only with the user's consent. "
                    "Returns the changes, the new export id and its check.",
     "inputSchema": {"type": "object", "properties": {
         "cell": _CELL,
         "build": {"type": "string", "description": "the export to fix (default: the newest)"},
         "rules": {"type": "array", "items": {"type": "string"},
                   "description": "only these rules' fixes (ids as in the table, e.g. cell.safety); default: all failing"}}}},
    {"name": "suggest_layout",
     "description": "Robo Fitter: robot moves (and pedestal heights) that bring the welds no robot reaches, then those "
                    "only one reaches, inside the robots' reach domes (robots kept 1.5 m apart, moved at most "
                    "max_move_m, default 1). write_as: save the moved scene as a NEW scene, by name (an "
                    "existing name is refused, nothing is overwritten). Check the new scene with check_reachability.",
     "inputSchema": {"type": "object", "properties": {
         "scene": {"type": "string"}, "cell": {"type": "string"},
         "max_move_m": {"type": "number"},
         "write_as": {"type": "string", "description": "new scene name, e.g. jam2-fit"}}}},
]
_TOOL_ARGS = {t["name"]: set(t["inputSchema"]["properties"]) for t in TOOLS}
_METHODS = {"jobs": "jobs_list"}     # tools whose Router method has another name (Router.jobs: the job table)


def call_tool(router: Router, name: str, arguments: dict) -> tuple[dict, bool]:
    """(result, is_error) of a tools/call."""
    if name not in _TOOL_ARGS:
        return {"error": f"unknown tool {name!r}", "hint": "tools/list"}, True
    args = {k: v for k, v in (arguments or {}).items() if k in _TOOL_ARGS[name] and v is not None}
    if name == "plan_status":
        args.setdefault("wait_s", 20)
    if name not in ("plan_status", "jobs"):
        print(f"[router] {name} {json.dumps(args, default=str)[:500]}", file=sys.stderr, flush=True)
    try:
        return getattr(router, _METHODS.get(name, name))(**args), False
    except ToolError as e:
        return e.result, True
    except Exception as e:  # noqa: BLE001 - relayed to the agent, never a crash
        return {"error": f"{type(e).__name__}: {e}", "hint": "see the router's journal"}, True


def handle_rpc(router: Router, body: Any) -> tuple[int, Optional[dict], dict]:
    """(HTTP status, JSON-RPC response or None, extra headers)."""
    if not isinstance(body, dict):
        return 400, {"jsonrpc": "2.0", "id": None, "error": {"code": -32600, "message": "Invalid Request"}}, {}
    rid, method, params = body.get("id"), body.get("method") or "", body.get("params") or {}
    if method == "initialize":
        result = {"protocolVersion": params.get("protocolVersion") or "2024-11-05",
                  "capabilities": {"tools": {"listChanged": False}},
                  "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION}}
        return 200, {"jsonrpc": "2.0", "id": rid, "result": result}, {"Mcp-Session-Id": uuid.uuid4().hex}
    if method == "ping":
        return 200, {"jsonrpc": "2.0", "id": rid, "result": {}}, {}
    if method.startswith("notifications/"):
        return 202, None, {}
    if method == "tools/list":
        return 200, {"jsonrpc": "2.0", "id": rid, "result": {"tools": TOOLS}}, {}
    if method == "tools/call":
        value, is_error = call_tool(router, params.get("name"), params.get("arguments") or {})
        content = [{"type": "text", "text": json.dumps(value, default=str)}]
        return 200, {"jsonrpc": "2.0", "id": rid, "result": {"content": content, "isError": is_error}}, {}
    return 404, {"jsonrpc": "2.0", "id": rid, "error": {"code": -32601, "message": f"Method not found: {method}"}}, {}


def make_handler(router: Router):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = f"{SERVER_NAME}/{SERVER_VERSION}"

        def _send(self, status: int, value: Optional[dict], headers: Optional[dict] = None) -> None:
            data = b"" if value is None else json.dumps(value, default=str).encode()
            self.send_response(status)
            if value is not None:
                self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            for k, v in (headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):  # noqa: N802
            if self.path.split("?")[0] == "/health":
                self._send(200, router.health())
            elif self.path.split("?")[0] == "/mcp":
                self._send(405, {"error": "use POST (no SSE stream)"}, {"Allow": "POST"})
            else:
                self._send(404, {"error": "not found"})

        def do_DELETE(self):  # noqa: N802 - MCP session end
            self._send(200 if self.path.split("?")[0] == "/mcp" else 404, {})

        def do_POST(self):  # noqa: N802
            if self.path.split("?")[0] != "/mcp":
                self._send(404, {"error": "not found"})
                return
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"null")
            except (ValueError, OSError):
                self._send(400, {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "Parse error"}})
                return
            status, value, headers = handle_rpc(router, body)
            self._send(status, value, headers)

        def log_message(self, fmt, *args):  # one line per request on stderr (the journal)
            sys.stderr.write(f"[router] {self.address_string()} {fmt % args}\n")

    return Handler


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="127.0.0.1:8893")
    ap.add_argument("--cell-planner", required=True, help="factoryos-cell-planner source tree")
    ap.add_argument("--python", required=True, help="the cell planner venv's python")
    ap.add_argument("--umarell", required=True, help="the umarell checkout (agenticRepoDir)")
    ap.add_argument("--umarell-python", default=None,
                    help="umarell's venv python for U3 (default <umarell>/.venv-planner/bin/python)")
    ap.add_argument("--config", default=None, help="factoryos.config.json (cellsDir / cellDir)")
    ap.add_argument("--cells", default=None, help="the cells folder (default: the config's cellsDir)")
    ap.add_argument("--work", default="/var/lib/factoryos/cell-planner/work")
    ap.add_argument("--planners", default=str(HERE / "planners.json"))
    ap.add_argument("--umarell-url", default="http://127.0.0.1:8890/mcp")
    ap.add_argument("--backend-url", default="http://127.0.0.1")
    ap.add_argument("--commit", default="", help="the cell planner's commit (default: <src>/.cell-planner.txt / git)")
    a = ap.parse_args(argv)
    host, _, port = a.bind.rpartition(":")
    work = Path(a.work)
    work.mkdir(parents=True, exist_ok=True)
    s = Settings(cell_planner=Path(a.cell_planner).resolve(), python=Path(a.python), umarell=Path(a.umarell),
                 config=Path(a.config) if a.config else None, work=work, cells=Path(a.cells) if a.cells else None,
                 planners_file=Path(a.planners), umarell_url=a.umarell_url, backend_url=a.backend_url, commit=a.commit,
                 umarell_python=Path(a.umarell_python) if a.umarell_python else None)
    router = Router(s)
    httpd = ThreadingHTTPServer((host or "127.0.0.1", int(port)), make_handler(router))
    httpd.daemon_threads = True

    def stop(*_):
        threading.Thread(target=httpd.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    print(f"[router] {SERVER_NAME} on http://{host}:{port}/mcp; cell planner {s.cell_planner} ({router.commit}), "
          f"umarell {s.umarell}, work {work}", file=sys.stderr, flush=True)
    try:
        httpd.serve_forever()
    finally:
        with router.lock:
            procs = [j.proc for j in router.jobs.values() if j.proc is not None]
            launchers = [j.launcher for j in router.jobs.values() if j.launcher is not None and j.status == "planning"]
        for p in procs:
            Router._terminate(p)
        for launcher in launchers:      # remote jobs: their instances (else their watchdogs end them)
            launcher.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
