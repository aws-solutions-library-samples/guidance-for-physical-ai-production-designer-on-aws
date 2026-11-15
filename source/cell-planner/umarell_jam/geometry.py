"""Full dressed robot / fixture geometry backstop, with explicit electrode contacts.

Native collision hulls are used for links and self/robot pairs. The tool's actual
separate triangle meshes backstop workpiece contact, so a convex envelope across
the gun opening cannot be mistaken for an electrode. No fixture is removed.
"""
from __future__ import annotations

from itertools import combinations
from pathlib import Path
import time

import numpy as np
import pinocchio as pin
import hppfcl as fcl
from scipy.spatial.transform import Rotation

from .cell import Cell

_LOCAL_BOUNDS = {}


def transform(position, orientation):
    return pin.SE3(Rotation.from_euler("ZYX", [orientation.get(k, 0) for k in "abc"],
                                      degrees=True).as_matrix(),
                   np.array([position[k] for k in "xyz"], dtype=float))


def bounds(shape, pose):
    cached = _LOCAL_BOUNDS.get(id(shape))
    if cached is None or cached[0] is not shape:
        shape.computeLocalAABB()
        cached = (shape, np.array(shape.aabb_local.min_), np.array(shape.aabb_local.max_))
        _LOCAL_BOUNDS[id(shape)] = cached
        if len(_LOCAL_BOUNDS) > 4096:
            _LOCAL_BOUNDS.pop(next(iter(_LOCAL_BOUNDS)))
    _, lo, hi = cached
    center, half = (lo + hi) / 2, (hi - lo) / 2
    center = pose.rotation @ center + pose.translation
    extent = np.abs(pose.rotation) @ half
    return center - extent, center + extent


def placed(name, shape, pose, **meta):
    lo, hi = bounds(shape, pose)
    return {"name": name, "shape": shape, "tf": fcl.Transform3f(pose.rotation, pose.translation),
            "lo": lo, "hi": hi, **meta}


def separation(a, b):
    return float(np.linalg.norm(np.maximum(np.maximum(a["lo"] - b["hi"], b["lo"] - a["hi"]), 0)))


def collision(a, b, margin=0.0):
    if separation(a, b) > margin + 1e-9:
        return None
    req = fcl.CollisionRequest()
    req.enable_contact = True
    req.num_max_contacts = 64
    req.security_margin = margin
    result = fcl.CollisionResult()
    fcl.collide(a["shape"], a["tf"], b["shape"], b["tf"], req, result)
    if not result.isCollision():
        return None
    return {"pair": [a["name"], b["name"]],
            "contacts": [{"position": list(map(float, p.pos)),
                          "depth_m": float(p.penetration_depth)} for p in result.getContacts()]}


class Geometry:
    def __init__(self, cell: Cell):
        self.cell = cell
        self.arms = {}
        self.fixtures = []
        self.queries = 0
        self._placements = {}
        for rid, cfg in cell.robots.items():
            u = cell.robot_urdfs[rid]
            model = pin.buildModelFromUrdf(str(u))
            geom = pin.buildGeomFromUrdf(model, str(u), pin.GeometryType.COLLISION,
                                        package_dirs=[str(u.parent)])
            for go in geom.geometryObjects:
                go.geometry.buildConvexRepresentation(False)
                go.geometry = go.geometry.convex
            geom.addAllCollisionPairs()
            pin.removeCollisionPairs(model, geom, str(cell.srdfs[rid]))
            visual = pin.buildGeomFromUrdf(model, str(u), pin.GeometryType.VISUAL,
                                          package_dirs=[str(u.parent)])
            # Only the distal electrode patch may contact the selected workpiece.
            # Partition triangles without deleting or simplifying any geometry.
            # The rest of tip.obj remains an ordinary collision body.
            tool_parts = {}
            for i, go in enumerate(visual.geometryObjects):
                if not go.meshPath.endswith("/tip.obj"):
                    continue
                tcp_frame = model.frames[model.getFrameId(cfg["tip_link"])]
                if tcp_frame.parent != go.parentJoint:
                    raise ValueError("electrode and TCP must share the dressed tool joint")
                tcp_offset = pin.SE3(np.eye(3), np.array(cfg.get("tcp_offset", [0., 0., 0.])))
                target_local = (go.placement.inverse() * tcp_frame.placement * tcp_offset).translation
                vertices = np.array(go.geometry.vertices())
                triangles = np.array([[go.geometry.tri_indices(j)[k] for k in range(3)]
                                      for j in range(go.geometry.num_tris)])
                allowed = np.all(np.linalg.norm(vertices[triangles] - target_local, axis=2) <= .02, axis=1)
                parts = []
                for mask, electrode in ((allowed, True), (~allowed, False)):
                    faces = triangles[mask]
                    if not len(faces):
                        continue
                    used, remap = np.unique(faces, return_inverse=True)
                    part_vertices, faces = vertices[used], remap.reshape(-1, 3)
                    shape = fcl.BVHModelOBBRSS()
                    shape.beginModel(len(faces), len(part_vertices))
                    vv, ff = fcl.StdVec_Vec3f(), fcl.StdVec_Triangle()
                    for vertex in part_vertices:
                        vv.append(vertex)
                    for face in faces:
                        ff.append(fcl.Triangle(*map(int, face)))
                    shape.addSubModel(vv, ff)
                    shape.endModel()
                    parts.append((shape, electrode))
                tool_parts[i] = parts
            cfgworld = pin.SE3(Rotation.from_euler("z", cfg["base_orientation_yaw"]).as_matrix(),
                              np.array(cfg["base_position"], dtype=float))
            self.arms[rid] = {"m": model, "d": model.createData(), "g": geom,
                              "gd": pin.GeometryData(geom), "visual": visual,
                              "vd": pin.GeometryData(visual), "world": cfgworld,
                              "tcp": model.getFrameId(cfg["tip_link"]), "tool_parts": tool_parts}
        for fixture in cell.fixtures:
            u = Path(fixture["resolved_urdf"])
            model = pin.buildModelFromUrdf(str(u))
            geom = pin.buildGeomFromUrdf(model, str(u), pin.GeometryType.COLLISION,
                                        package_dirs=[str(u.parent)])
            if geom.ngeoms == 0:
                raise ValueError(f"fixture lacks collision geometry: {u}")
            data, gd = model.createData(), pin.GeometryData(geom)
            pin.forwardKinematics(model, data, np.zeros(model.nq))
            pin.updateGeometryPlacements(model, data, geom, gd)
            world = transform(fixture["position"], fixture["orientation"])
            for i, go in enumerate(geom.geometryObjects):
                self.fixtures.append(placed(f"{fixture['id']}/{Path(go.meshPath).name}",
                                            go.geometry, world * gd.oMg[i]))
        self.fixture_lo = np.array([f["lo"] for f in self.fixtures])
        self.fixture_hi = np.array([f["hi"] for f in self.fixtures])
        self.levers = {rid: self._levers(rid) for rid in self.arms}

    def _levers(self, rid):
        arm = self.arms[rid]
        model = arm["m"]
        radii = {}
        for g in (arm["g"], arm["visual"]):
            for go in g.geometryObjects:
                go.geometry.computeLocalAABB()
                lo, hi = np.asarray(go.geometry.aabb_local.min_), np.asarray(go.geometry.aabb_local.max_)
                radius = float(np.linalg.norm(np.maximum(abs(lo), abs(hi)))
                               + np.linalg.norm(go.placement.translation))
                radii[go.parentJoint] = max(radii.get(go.parentJoint, 0), radius)
        levers = np.zeros(model.nq)
        for j in range(1, model.njoints):
            if model.joints[j].nq != 1:
                raise ValueError("swept bounds require one-dimensional joints")
            for k, radius in radii.items():
                i, chain = k, 0.
                while i not in (0, j):
                    chain += np.linalg.norm(model.jointPlacements[i].translation)
                    i = model.parents[i]
                if i == j:
                    levers[model.joints[j].idx_q] = max(levers[model.joints[j].idx_q], chain + radius)
        return levers

    def interval_clear(self, states, displacement, contacts=None):
        """Lipschitz swept bound around the midpoint of an interval.

        displacement[r] bounds the travel of ANY point of arm r from that
        midpoint. Exact FCL distances refine inconclusive AABB lower bounds.
        False means subdivide, never a claim that a collision exists.
        """
        contacts = contacts or {}
        arms = {}

        def separated(a, b, need):
            if separation(a, b) > need + 1e-9:
                return True
            result = fcl.DistanceResult()
            value = fcl.distance(a["shape"], a["tf"], b["shape"], b["tf"],
                                 fcl.DistanceRequest(), result)
            return value > need + 1e-9

        for rid, q in states.items():
            bodies, tools, _ = self.place(rid, q)
            arms[rid] = bodies
            move = displacement[rid]
            for cp in self.arms[rid]["g"].collisionPairs:
                if not separated(bodies[cp.first], bodies[cp.second], 2 * move):
                    return False
            for body in bodies:
                if not body["fixed"] and body["lo"][2] <= move:
                    return False
            for body in [b for b in bodies if not b["tool"]] + tools:
                if body.get("electrode") and rid in contacts:
                    target = np.array(contacts[rid])
                    furthest = np.linalg.norm(np.maximum(abs(body["lo"] - target), abs(body["hi"] - target)))
                    if furthest + move < self.cell.config["collision"]["tool_contact_radius"]:
                        continue
                lower = np.linalg.norm(np.maximum(np.maximum(
                    body["lo"] - self.fixture_hi, self.fixture_lo - body["hi"]), 0), axis=1)
                for k in np.flatnonzero(lower <= move + 1e-9):
                    if not separated(body, self.fixtures[int(k)], move):
                        return False
        margin = float(self.cell.config.get("collision", {}).get("min_clearance", 0))
        for a, b in combinations(arms, 2):
            for ba in arms[a]:
                for bb in arms[b]:
                    if not separated(ba, bb, displacement[a] + displacement[b] + margin):
                        return False
        return True

    def place(self, rid, q):
        key = tuple(q)
        cached = self._placements.get(rid)
        if cached is not None and cached[0] == key:
            return cached[1]
        arm = self.arms[rid]
        m, d, g, gd, world = (arm[k] for k in ("m", "d", "g", "gd", "world"))
        pin.forwardKinematics(m, d, np.array(q, dtype=float))
        pin.updateFramePlacements(m, d)
        pin.updateGeometryPlacements(m, d, g, gd)
        pin.updateGeometryPlacements(m, d, arm["visual"], arm["vd"])
        bodies = [placed(f"{rid}/{go.name}", go.geometry, world * gd.oMg[i],
                          fixed=go.parentJoint == 0, tool="/tool/" in go.meshPath)
                  for i, go in enumerate(g.geometryObjects)]
        tools = []
        for i, go in enumerate(arm["visual"].geometryObjects):
            if "/tool/" not in go.meshPath:
                continue
            parts = arm["tool_parts"].get(i, [(go.geometry, False)])
            for shape, electrode in parts:
                tools.append(placed(f"{rid}/{Path(go.meshPath).name}", shape,
                                    world * arm["vd"].oMg[i], electrode=electrode))
        tcp = world * d.oMf[arm["tcp"]]
        offset = np.array(self.cell.robots[rid].get("tcp_offset", [0, 0, 0]))
        tcp = tcp * pin.SE3(np.eye(3), offset)
        out = (bodies, tools, tcp)
        self._placements[rid] = (key, out)
        return out

    def fixture_check(self, bodies, tools, target=None, *, contact=False):
        # The convex tool decomposition is conservative; the separate actual
        # meshes check the gun opening and electrode semantics against workpieces.
        for body in [b for b in bodies if not b["tool"]] + tools:
            near = np.flatnonzero(np.all(body["lo"] <= self.fixture_hi + 1e-9, axis=1)
                                  & np.all(body["hi"] >= self.fixture_lo - 1e-9, axis=1))
            for j in near:
                hit = collision(body, self.fixtures[int(j)])
                if hit is None:
                    continue
                allow = (contact and target is not None and body.get("electrode")
                         and hit["contacts"] and all(
                             np.linalg.norm(np.array(p["position"]) - target)
                             <= self.cell.config["collision"]["tool_contact_radius"]
                             for p in hit["contacts"]))
                if not allow:
                    return {"kind": "fixture_or_tool", **hit}
        return None

    def check(self, states: dict, contacts: dict | None = None):
        """All installed arms, including inactive ones; no missing state allowed."""
        self.queries += 1
        if set(states) != set(self.arms):
            raise ValueError("full scene validation needs every installed arm")
        contacts = contacts or {}
        placed_arms = {}
        for rid, q in states.items():
            cfg = self.cell.robots[rid]
            if np.any(np.array(q) < cfg["joint_limits_lower"]) or np.any(np.array(q) > cfg["joint_limits_upper"]):
                return {"kind": "joint_limits", "robot": rid}
            bodies, tools, tcp = self.place(rid, q)
            placed_arms[rid] = bodies
            arm = self.arms[rid]
            for cp in arm["g"].collisionPairs:
                hit = collision(bodies[cp.first], bodies[cp.second])
                if hit:
                    return {"kind": "self", "robot": rid, **hit}
            # Floor mount is the sole fixed-base contact exemption.
            for body in bodies:
                if not body["fixed"] and body["lo"][2] < -1e-8:
                    return {"kind": "floor", "body": body["name"], "min_z": float(body["lo"][2])}
            target = np.array(contacts[rid]) if rid in contacts else None
            hit = self.fixture_check(bodies, tools, target, contact=target is not None)
            if hit:
                return {"robot": rid, **hit}
        margin = float(self.cell.config.get("collision", {}).get("min_clearance", 0))
        for a, b in combinations(placed_arms, 2):
            for ba in placed_arms[a]:
                for bb in placed_arms[b]:
                    hit = collision(ba, bb, margin)
                    if hit:
                        return {"kind": "robot_pair", **hit}
        return None

    def target_tool_check(self, rid, position, orientation, *, contact=True):
        """A cheap IK-independent necessary check; failures are candidate failures."""
        bodies, tools, tcp = self.place(rid, np.zeros(self.arms[rid]["m"].nq))
        desired = pin.SE3(Rotation.from_quat([orientation[k] for k in "xyzw"]).as_matrix(),
                         np.array(position, dtype=float))
        delta = desired * tcp.inverse()
        moved = []
        for tool in tools:
            old = pin.SE3(tool["tf"].getRotation(), tool["tf"].getTranslation())
            moved.append(placed(tool["name"], tool["shape"], delta * old,
                                electrode=tool["electrode"]))
        return self.fixture_check([], moved, np.array(position), contact=contact)
