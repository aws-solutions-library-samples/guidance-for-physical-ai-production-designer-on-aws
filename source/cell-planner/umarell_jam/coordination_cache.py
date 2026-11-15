"""Conservative continuous pair certificates over independent trajectory clocks."""
from __future__ import annotations

import hashlib
from pathlib import Path
import time

import hppfcl as fcl
import numpy as np

from .geometry import separation, collision
from .store import digest
from .trajectory import Trajectory
from .validation import compatible


def dependencies(cell, a, b):
    order = sorted([(a["robot"], a["certificate"]), (b["robot"], b["certificate"])],
                   key=lambda v: (v[0], v[1]["trajectory_hash"]))
    return {"geometry": cell.geometry_hash, "margin": cell.config["collision"]["min_clearance"],
            "paths": [{"robot": r, "certificate": digest(c)} for r, c in order],
            "checker": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def accepts(cell, a, b, store):
    if a["robot"] == b["robot"]:
        return False
    if compatible(a["certificate"], b["certificate"], cell.config["collision"]["min_clearance"]):
        return True
    cached = store.get("coordination", dependencies(cell, a, b))
    return bool(cached and cached.get("status") == "certified")


def prepare(cell, geometry, a, b, store, budget=20.):
    dep = dependencies(cell, a, b)
    def build():
        start = time.monotonic()
        deadline = start + budget
        if compatible(a["certificate"], b["certificate"], dep["margin"]):
            return {"status": "certified", "proof": "whole-path swept AABB separation", "wall_s": 0.}
        pa, pb = Trajectory(a["points"]), Trajectory(b["points"])
        va, vb = np.max(abs(pa.segment_extrema(1)), axis=1), np.max(abs(pb.segment_extrema(1)), axis=1)
        la, lb = geometry.levers[a["robot"]], geometry.levers[b["robot"]]

        def travel(path, velocity, lever, lo, hi):
            first = max(0, int(np.searchsorted(path.times, lo, side="right")) - 1)
            last = min(len(velocity), int(np.searchsorted(path.times, hi, side="left")) + 1)
            return float(np.max(velocity[first:last], axis=0) @ lever) * (hi-lo)/2

        stack = [(0., pa.duration, 0., pb.duration)]
        rectangles = 0
        while stack:
            if time.monotonic() >= deadline:
                return {"status": "timeout_or_unknown", "reason": "pair_bound_deadline",
                        "rectangles": rectangles, "wall_s": time.monotonic()-start}
            a0, a1, b0, b1 = stack.pop()
            ta, tb = (a0+a1)/2, (b0+b1)/2
            da, db = travel(pa, va, la, a0, a1), travel(pb, vb, lb, b0, b1)
            aa = geometry.place(a["robot"], pa.sample(ta))[0]
            bb = geometry.place(b["robot"], pb.sample(tb))[0]
            clear = True
            for ba in aa:
                for body_b in bb:
                    bound = separation(ba, body_b)
                    displacement = (0. if ba["fixed"] else da) + (0. if body_b["fixed"] else db)
                    if bound > displacement + dep["margin"] + 1e-8:
                        continue
                    if collision(ba, body_b, dep["margin"]) is not None:
                        return {"status": "conflict", "reason": "independent_clock_collision_witness",
                                "times_s": [ta, tb], "pair": [ba["name"], body_b["name"]],
                                "wall_s": time.monotonic()-start}
                    distance = fcl.distance(ba["shape"], ba["tf"], body_b["shape"], body_b["tf"],
                                            fcl.DistanceRequest(), fcl.DistanceResult())
                    if distance <= displacement + dep["margin"] + 1e-8:
                        clear = False
            rectangles += 1
            if clear:
                continue
            if max(a1-a0, b1-b0) < 1e-6:
                return {"status": "timeout_or_unknown", "reason": "pair_bound_unresolved",
                        "intervals_s": [[a0, a1], [b0, b1]], "rectangles": rectangles}
            if da >= db:
                stack.extend([(a0, ta, b0, b1), (ta, a1, b0, b1)])
            else:
                stack.extend([(a0, a1, b0, tb), (a0, a1, tb, b1)])
        return {"status": "certified", "proof": "continuous two-dimensional clock subdivision with FCL distance bounds",
                "rectangles": rectangles, "wall_s": time.monotonic()-start}
    return store.obtain("coordination", dep, build)
