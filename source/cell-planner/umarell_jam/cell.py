"""Strict real-cell import and dependency provenance, separate geometry/timing keys."""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import json

import yaml

from . import CHECKER_VERSION, RETIMER_VERSION
from .store import digest


class CellLoadError(ValueError):
    pass


class Cell:
    def __init__(self, workspace: str | Path, cell_id: str):
        self.workspace = Path(workspace).resolve()
        if "/" in cell_id or cell_id in {"", ".", ".."}:
            raise CellLoadError("invalid cell id")
        self.cell_id = cell_id
        self.root = self.workspace / "cells" / cell_id
        self.files: dict[str, str] = {}
        self.asset_files: dict[str, str] = {}
        self.manifest = self.yaml(self.root / "agentic_planner/cell.yaml")
        self.config = self.yaml(self.root / "agentic_planner/config/robots.yaml")
        self.obstacles = self.yaml(self.root / "agentic_planner/config/obstacles.yaml")
        self.scene = self.yaml(self.root / ".view/scene.yaml")
        self.robots = {}
        for group in self.config.get("robots", {}).values():
            for rid, cfg in group.items():
                if rid in self.robots:
                    raise CellLoadError(f"duplicate robot {rid}")
                self.robots[rid] = cfg
        if not self.robots:
            raise CellLoadError("no configured robots")
        declarations = {r["id"]: r for r in self.manifest["robots"]}
        if set(declarations) != set(self.robots):
            raise CellLoadError("manifest/config robot set mismatch")
        scene_robots = {r["robot_id"]: r for r in self.scene["robots"]}
        if {r["scene_id"] for r in declarations.values()} != set(scene_robots):
            raise CellLoadError("scene/config robot set mismatch")
        self.robot_urdfs = {}
        self.srdfs = {}
        for rid, cfg in self.robots.items():
            decl = declarations[rid]
            scene = scene_robots[decl["scene_id"]]
            urdf = self.resolve(decl["urdf"], self.root)
            self.robot_urdfs[rid] = urdf
            self.srdfs[rid] = self.resolve(decl["srdf"], self.root)
            self.record(self.srdfs[rid], asset=True)
            xml = self.urdf(urdf)
            actual = self.urdf(self.resolve(scene["dressed_urdf"], self.root / ".view"))
            # Hash canonical XML independent of mesh absolute staging paths.
            if ET.tostring(xml) != ET.tostring(actual):
                raise CellLoadError(f"{rid}: .view and declared dressed URDF differ")
            links = {el.attrib["name"] for el in xml.findall("link")}
            if cfg.get("tip_link") not in links:
                raise CellLoadError(f"{rid}: missing declared TCP parent")
            for axis, x in zip("xyz", cfg["base_position"]):
                if abs(x - scene["position"][axis]) > 1e-6:
                    raise CellLoadError(f"{rid}: scene/base position mismatch")
            orient = scene["orientation"]
            if abs(orient.get("b", 0)) + abs(orient.get("c", 0)) > 1e-8:
                raise CellLoadError(f"{rid}: tilted base needs full transform support")
            if abs(math.radians(orient["a"]) - cfg["base_orientation_yaw"]) > 1e-6:
                raise CellLoadError(f"{rid}: scene/base rotation mismatch")
            if cfg["home_pose_deg"] != scene["home_pose_deg"]:
                raise CellLoadError(f"{rid}: scene/configured HOME mismatch")
            joints = [j for j in xml.findall("joint") if j.get("type") in {"revolute", "prismatic"}]
            cfg["joint_order"] = [j.get("name") for j in joints]
            for i, j in enumerate(joints):
                limit = j.find("limit").attrib
                # Effective limits are the intersection, never a rounded relaxation.
                cfg["joint_limits_lower"][i] = max(cfg["joint_limits_lower"][i], float(limit["lower"]))
                cfg["joint_limits_upper"][i] = min(cfg["joint_limits_upper"][i], float(limit["upper"]))
                cfg["joint_velocity_limits"][i] = min(cfg["joint_velocity_limits"][i], float(limit["velocity"]))
            for name in ("joint_limits_lower", "joint_limits_upper", "joint_velocity_limits",
                         "joint_acceleration_limits"):
                if len(cfg.get(name, [])) != len(joints):
                    raise CellLoadError(f"{rid}: incomplete {name}")
        self.fixtures = []
        for f in self.scene.get("fixtures", []):
            p = self.resolve(f["urdf"], self.root / ".view")
            self.urdf(p)
            self.fixtures.append({**f, "resolved_urdf": str(p)})
        if not self.fixtures:
            raise CellLoadError("missing fixture/workpiece geometry")
        geometry = {
            "assets": self.asset_files, "scene": self.scene,
            "robots": {r: {k: v for k, v in c.items()
                          if k not in {"joint_velocity_limits", "joint_acceleration_limits",
                                       "joint_jerk_limits"}} for r, c in self.robots.items()},
            "collision": self.config.get("collision"), "obstacles": self.obstacles,
        }
        self.geometry_hash = digest(geometry)
        self.limits_hash = digest({"robots": {r: {k: v for k, v in c.items()
                                         if k.startswith("joint_")} for r, c in self.robots.items()},
                                   "planning": self.config.get("planning")})
        self.programs = {}
        self.program_errors = {}
        self.program_keys = {}
        for path in sorted((self.root / "agentic_planner/tasks").glob("*.json")):
            d = json.loads(self.record(path))
            pid = d["program"]
            if pid in self.programs:
                raise CellLoadError(f"duplicate program {pid}")
            self.programs[pid] = d
            self.program_errors[pid] = self.check_program(d)
            geometric_ops = [{k: v for k, v in o.items() if k in
                              {"id", "type", "position", "orientation", "weld_orientation", "approach_vector"}}
                             for o in d["operations"]]
            params = d.get("extensions", {}).get("weld_params", {})
            self.program_keys[pid] = {
                "geometry": digest({"cell": self.geometry_hash, "operations": geometric_ops,
                                    "approach": params.get("approach_offset_z"),
                                    "retract": params.get("retract_offset_z")}),
                "process": digest(d), "limits": self.limits_hash,
                "checker": CHECKER_VERSION, "retimer": RETIMER_VERSION,
            }
        if not self.programs:
            raise CellLoadError("no programs")

    def resolve(self, ref: str, parent: Path) -> Path:
        if ref.startswith("/var/lib/factoryos/"):
            p = self.workspace / ref.removeprefix("/var/lib/factoryos/")
        else:
            p = Path(ref)
            if not p.is_absolute():
                p = parent / p
        # Resolve original absolute fixture symlinks through the supplied mapping.
        parts = p.parts
        cur = Path(parts[0])
        for part in parts[1:]:
            cur = cur / part
            if cur.is_symlink():
                target = cur.readlink()
                if str(target).startswith("/var/lib/factoryos/"):
                    cur = self.workspace / str(target).removeprefix("/var/lib/factoryos/")
        p = cur.resolve()
        if not p.is_relative_to(self.workspace):
            raise CellLoadError(f"asset escapes declared workspace: {ref} -> {p}")
        if not p.is_file():
            raise CellLoadError(f"missing geometry/config dependency: {p}")
        return p

    def record(self, path: Path, *, asset=False) -> bytes:
        if not path.is_file():
            raise CellLoadError(f"missing dependency: {path}")
        data = path.read_bytes()
        name = str(path.relative_to(self.workspace))
        self.files[name] = hashlib.sha256(data).hexdigest()
        if asset:
            self.asset_files[name] = self.files[name]
        return data

    def yaml(self, path):
        return yaml.load(self.record(path), Loader=getattr(yaml, "CSafeLoader", yaml.SafeLoader))

    def urdf(self, path: Path):
        root = ET.fromstring(self.record(path, asset=True))
        for mesh in root.findall(".//mesh"):
            p = self.resolve(mesh.attrib["filename"], path.parent)
            self.record(p, asset=True)
            # OBJ material/texture files are part of the import, including visual assets.
            if p.suffix == ".obj":
                for line in p.read_text(errors="replace").splitlines():
                    if line.startswith("mtllib "):
                        mtl = self.resolve(line[7:].strip(), p.parent)
                        self.record(mtl, asset=True)
                        for material in mtl.read_text().splitlines():
                            if material.startswith(("map_Kd ", "map_Ka ", "map_Bump ", "bump ")):
                                self.record(self.resolve(material.split(maxsplit=1)[1], mtl.parent), asset=True)
        return root

    def check_program(self, d):
        errors = []
        ops = d["operations"]
        if len({o["id"] for o in ops}) != len(ops):
            errors.append("duplicate operation ids")
        if any(o.get("type") != "weld" for o in ops):
            errors.append("unsupported non-weld operation")
        ext = d.get("extensions", {})
        strategy = ext.get("strategy", {})
        groups = strategy.get("sequence_groups_order", [])
        used = {o.get("sequence_group") for o in ops}
        if not used <= set(groups) or len(groups) != len(set(groups)):
            errors.append(f"sequence_groups_order {groups} contradicts operation groups {sorted(used)}")
        cap = strategy.get("max_concurrent_robots", len(self.robots))
        if not isinstance(cap, int) or isinstance(cap, bool) or not 1 <= cap <= len(self.robots):
            errors.append("invalid max_concurrent_robots")
        params = ext.get("weld_params", {})
        phases = params.get("weld_phases", [])
        if [p.get("phase") for p in phases] != ["approach", "contact", "squeeze", "weld", "hold", "retract"]:
            errors.append("missing/contradictory process phases")
        for p in phases:
            if p.get("type") in {"wait", "weld_pulse"} and (
                    not isinstance(p.get("duration_s"), (float, int)) or p["duration_s"] < 0):
                errors.append("invalid process dwell")
            if "speed_percentage" in p and not 0 < p["speed_percentage"] <= 100:
                errors.append("invalid process speed")
        pulse = [p.get("duration_s") for p in phases if p.get("type") == "weld_pulse"]
        if pulse != [params.get("welding_duration_s")]:
            errors.append("contradictory welding duration")
        for op in ops:
            if op.get("orientation") != op.get("weld_orientation", op.get("orientation")):
                errors.append(f"{op['id']}: contradictory orientations")
        return errors

    def manifest_export(self):
        return {"schema_version": "umarell-cell-v1", "cell_id": self.cell_id,
                "scene_name": self.scene["name"], "geometry_hash": self.geometry_hash,
                "limits_hash": self.limits_hash, "files": self.files,
                "robots": self.robots, "programs": {
                    p: {"weld_ids": [o["id"] for o in d["operations"]],
                        "keys": self.program_keys[p], "errors": self.program_errors[p]}
                    for p, d in self.programs.items()}}

    def unchanged(self):
        """Content verification, including same-size edits with preserved mtimes."""
        for name, expected in self.files.items():
            p = self.workspace / name
            if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != expected:
                return False
        actual_programs = {p.stem for p in (self.root / "agentic_planner/tasks").glob("*.json")}
        return actual_programs == set(self.programs)
