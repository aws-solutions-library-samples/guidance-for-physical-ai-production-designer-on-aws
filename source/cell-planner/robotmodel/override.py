"""Process-wide robot override: point env/, the policies and execute.py at another URDF and/or base pose.

    from robotmodel.override import set_robot_override
    set_robot_override({"robots": {"*": {"urdf": "data/p3_urdf/kr240_mod/kr240_mod.urdf"},
                                   "robot2": {"base": [-1.18, 1.04, 0.0], "yaw": 0.1}}})
    # every later env.cell.load_cell(...) (env, expert, PPO / hier / residual / e2e controllers) uses it

Also via the environment (inherited by subprocess / multiprocessing workers):
    KWR_ROBOT_OVERRIDE='{"robots": {"*": {"urdf": "..."}}}'   or   KWR_ROBOT_OVERRIDE=path/to/override.{json,yaml}

Per-robot keys (``"*"`` = every robot, a robot id overrides ``"*"``):
    urdf         path to the URDF (meshes resolved relative to it); its joint limits replace robots.yaml's
    base         [x, y, z] world position of the URDF root;  yaw  rad about +z
    home_deg     [6] home pose (deg)
    tip_link     TCP parent link (default: robots.yaml tip_link, else the child of the deepest movable joint)
    tcp_offset   [3] TCP in the tip_link frame (default: robots.yaml)
    accel        [6] rad/s^2 acceleration limits (URDFs carry none; default robots.yaml, else the per-model default)
    jerk         [6] rad/s^3 (optional, informational)
    limits       "urdf" (default with an override urdf) | "config" (keep robots.yaml's lower/upper/velocity)
Top-level ``label``: free text, recorded in results.
"""
from __future__ import annotations

import json
import math
import os
from functools import lru_cache
from pathlib import Path

ENV_VAR = "KWR_ROBOT_OVERRIDE"
LIMITS_ENV = "KWR_LIMITS_SOURCE"     # "auto" (default: URDF, robots.yaml copies where equal) | "config" | "urdf"
_KEYS = {"urdf", "base", "yaw", "home_deg", "tip_link", "tcp_offset", "accel", "jerk", "limits"}


def _norm_robot(d: dict, ctx: str) -> dict:
    bad = set(d) - _KEYS
    if bad:
        raise ValueError(f"robot override {ctx}: unknown keys {sorted(bad)} (allowed: {sorted(_KEYS)})")
    out = dict(d)
    if out.get("urdf"):
        p = Path(out["urdf"]).expanduser()
        out["urdf"] = str(p.resolve())
        if not p.exists():
            raise FileNotFoundError(f"robot override {ctx}: URDF not found: {p}")
    for k, n in (("base", 3), ("home_deg", 6), ("tcp_offset", 3), ("accel", None), ("jerk", None)):
        if out.get(k) is not None:
            out[k] = [float(x) for x in out[k]]
            if n is not None and len(out[k]) != n:
                raise ValueError(f"robot override {ctx}.{k}: expected {n} values")
            if k in ("accel", "jerk") and not all(math.isfinite(x) and x > 0 for x in out[k]):
                raise ValueError(f"robot override {ctx}.{k}: every value must be finite and > 0, got {out[k]}")
    if out.get("yaw") is not None:
        out["yaw"] = float(out["yaw"])
    if out.get("limits") not in (None, "urdf", "config"):
        raise ValueError(f"robot override {ctx}.limits: 'urdf' or 'config'")
    return out


def normalize(spec) -> dict | None:
    """dict | JSON text | path to .json/.yaml | None -> canonical dict (absolute URDF paths) or None."""
    if spec is None or spec == "" or spec == {}:
        return None
    if isinstance(spec, (str, Path)):
        s = str(spec).strip()
        if s.startswith("{"):
            spec = json.loads(s)
        else:
            p = Path(s)
            txt = p.read_text()
            if p.suffix in (".yaml", ".yml"):
                import yaml
                spec = yaml.safe_load(txt)
            else:
                spec = json.loads(txt)
    if not isinstance(spec, dict):
        raise ValueError("robot override: expected a mapping")
    rob = spec.get("robots", {}) or {}
    out = {"robots": {str(k): _norm_robot(v or {}, str(k)) for k, v in sorted(rob.items())}}
    if spec.get("label"):
        out["label"] = str(spec["label"])
    return out if out["robots"] else None


def dumps(ov: dict | None) -> str:
    return "" if not ov else json.dumps(ov, sort_keys=True)


@lru_cache(maxsize=32)
def _parse_env(s: str) -> dict | None:
    return normalize(s)


def active_override() -> dict | None:
    """The override in effect for this process (from KWR_ROBOT_OVERRIDE), or None."""
    s = os.environ.get(ENV_VAR, "").strip()
    return _parse_env(s) if s else None


def set_robot_override(spec) -> dict | None:
    """Activate an override for this process and its children (sets KWR_ROBOT_OVERRIDE). None clears it."""
    ov = normalize(spec)
    if ov is None:
        os.environ.pop(ENV_VAR, None)
    else:
        os.environ[ENV_VAR] = dumps(ov)
    return ov


def clear_robot_override() -> None:
    os.environ.pop(ENV_VAR, None)


def override_for(ov: dict | None, rid: str) -> dict:
    """Merged per-robot entry ("*" then the robot id)."""
    if not ov:
        return {}
    r = ov.get("robots", {})
    return {**(r.get("*") or {}), **(r.get(rid) or {})}


# ----------------------------------------------------------------------------------------------- CLI helpers
def add_cli_args(ap) -> None:
    """--urdf / --robot-base / --robot-accel / --robot-override (all default off)."""
    g = ap.add_argument_group("robot model override (docs/p3_urdf.md; default: the cell's own URDF)")
    g.add_argument("--urdf", default=None, help="URDF for every robot (or 'rid=path', repeatable via comma); joint "
                   "limits incl. <limit velocity> are read from it")
    g.add_argument("--robot-base", action="append", default=[], metavar="RID=X,Y,Z[,YAW]",
                   help="world base pose of one robot (repeatable)")
    g.add_argument("--robot-accel", default=None, metavar="A1,..,A6", help="acceleration limits rad/s^2 (all robots)")
    g.add_argument("--robot-override", default=None, help="override spec: JSON text or a .json/.yaml file")


def spec_from_cli(a) -> dict | None:
    spec = normalize(a.robot_override) if getattr(a, "robot_override", None) else None
    spec = {"robots": dict((spec or {}).get("robots", {}))} | ({"label": spec["label"]} if spec and "label" in spec else {})
    rob = spec["robots"]
    if getattr(a, "urdf", None):
        for part in str(a.urdf).split(","):
            rid, _, path = part.rpartition("=")
            rob.setdefault(rid or "*", {})["urdf"] = path
    for b in getattr(a, "robot_base", []) or []:
        rid, _, vals = b.partition("=")
        v = [float(x) for x in vals.split(",")]
        if len(v) not in (3, 4):
            raise ValueError(f"--robot-base {b}: expected RID=X,Y,Z[,YAW]")
        e = rob.setdefault(rid, {})
        e["base"] = v[:3]
        if len(v) == 4:
            e["yaw"] = v[3]
    if getattr(a, "robot_accel", None):
        rob.setdefault("*", {})["accel"] = [float(x) for x in a.robot_accel.split(",")]
    return normalize(spec)


def apply_cli_args(a) -> dict | None:
    """Activate the override given on the command line (no-op without any of the flags)."""
    ov = spec_from_cli(a)
    if ov is not None:
        set_robot_override(ov)
    return ov
