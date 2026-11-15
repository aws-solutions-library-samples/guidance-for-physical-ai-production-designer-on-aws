"""Explicit supported model contract for the supplied Umarell controller."""
import xml.etree.ElementTree as ET

from .cell import CellLoadError


def validate_capacity(cell):
    if cell.config.get("collision", {}).get("speed_margin_full_speed_m", 0) != 0:
        raise CellLoadError("speed-dependent static margins require phase-specific recertification")
    for rid, path in cell.robot_urdfs.items():
        root = ET.parse(path).getroot()
        joints = [j for j in root.findall("joint") if j.get("type") != "fixed"]
        if len(joints) != 6 or any(j.get("type") != "revolute" for j in joints):
            raise CellLoadError(f"{rid}: this release supports six bounded revolute joints per arm")
        names = [j.get("name") for j in joints]
        if names != cell.robots[rid]["joint_order"]:
            raise CellLoadError(f"{rid}: controller/import joint order mismatch")
