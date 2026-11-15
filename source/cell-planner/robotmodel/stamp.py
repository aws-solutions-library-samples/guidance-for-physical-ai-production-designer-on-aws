"""Robot-model stamps of precomputed per-robot data (IK tables, P3U round 2, docs/p3_urdf.md section 9).

data/weld_ik*.json (Track X) hold IK solutions of the cell's own KR240. They drive the env's greedy split, the hier
InstanceSolver, the scripted expert and the curricula. ``stamp`` records the robot model they were solved for (URDF
content md5, base, yaw, tip_link, tcp_offset per robot) in a sidecar ``<table>.robot_model.json`` (the table files
themselves are not rewritten); ``check_table`` warns (once per file and model) when the
robot model in effect (robot override, an edited URDF in UMARELL_REPO) differs from it, so a stale table is never
used silently. Warning only: the default behaviour is unchanged.
"""
from __future__ import annotations

import hashlib
import json
import warnings
from pathlib import Path

import numpy as np

_WARNED: set = set()
GEOMETRY = ("urdf_md5", "base", "yaw", "tip_link", "tcp_offset")


def _md5(p) -> str | None:
    p = Path(p)
    return hashlib.md5(p.read_bytes()).hexdigest() if p.is_file() else None


def robot_stamp(cell_id: str) -> dict:
    """{rid: {urdf, urdf_md5, base, yaw, tip_link, tcp_offset}} of the robot model in effect for ``cell_id``."""
    from env.cell import load_cell
    c = load_cell(cell_id)
    return {rid: {"urdf": Path(rc.urdf).name, "urdf_md5": _md5(rc.urdf),
                  "base": [round(float(x), 9) for x in rc.base], "yaw": round(float(rc.yaw), 9),
                  "tip_link": rc.tip_link, "tcp_offset": [round(float(x), 9) for x in rc.tcp_offset]}
            for rid, rc in c.robots.items()}


def mismatches(stamp: dict, cur: dict) -> dict:
    out = {}
    for rid, e in (stamp or {}).items():
        c = cur.get(rid)
        if c is None:
            out[rid] = ["robot not in the cell"]
            continue
        d = []
        for k in GEOMETRY:
            a, b = e.get(k), c.get(k)
            if a is None or b is None:
                continue
            same = (a == b) if isinstance(a, str) else bool(np.allclose(np.asarray(a, float), np.asarray(b, float),
                                                                         atol=1e-6))
            if not same:
                d.append(k)
        if d:
            out[rid] = d
    return out


def sidecar(path) -> Path:
    p = Path(path)
    return p.with_name(p.stem + ".robot_model.json")


def check_table(path, cell_id: str, what: str = "IK table") -> dict:
    """Warn if the table at ``path`` (data/weld_ik*.json) was solved for another robot model than the one in effect
    for ``cell_id`` (its sidecar stamp). -> {"stamped": bool, "mismatches": {...}}."""
    try:
        sc = sidecar(path)
        st = json.load(open(sc)) if sc.exists() else None
        cur = robot_stamp(cell_id)
    except Exception as e:  # noqa: BLE001  (a check must never break a loader)
        return {"stamped": None, "error": repr(e)}
    from robotmodel.override import active_override, dumps
    key = (str(path), cell_id, dumps(active_override()), json.dumps(cur, sort_keys=True))
    if st is None:
        if active_override() is not None and key not in _WARNED:
            _WARNED.add(key)
            warnings.warn(f"{path}: {what} has no robot-model stamp (solved for the cell's own robots); a robot "
                          f"override is active, so its solutions may not fit (re-solve it under the override)",
                          UserWarning, stacklevel=3)
        return {"stamped": False, "mismatches": {}}
    mm = mismatches(st.get("robots", {}), cur)
    if mm and key not in _WARNED:
        _WARNED.add(key)
        warnings.warn(f"{path}: {what} was solved for another robot model than the one in effect for {cell_id} "
                      f"({mm}); its IK solutions / reachability may be wrong (re-solve it: scripts/weld_ik_table.py)",
                      UserWarning, stacklevel=3)
    return {"stamped": True, "mismatches": mm}


def stamp(path, cell_id: str, note: str = "") -> dict:
    """Write the sidecar stamp of a table: the stock cell's robots (no override active)."""
    from robotmodel.override import active_override
    if active_override() is not None:
        raise RuntimeError("stamp the stock robot model: clear KWR_ROBOT_OVERRIDE first")
    rec = {"table": Path(path).name, "table_md5": _md5(path), "cell_id": cell_id, "robots": robot_stamp(cell_id),
           "note": note}
    sidecar(path).write_text(json.dumps(rec, indent=1))
    return rec
