"""Weld IK for any scene robot: ``expert.ik.WeldIK``'s solver (DLS, 5-DOF spin sweep, tilt, straight strokes) on a
per-robot Pinocchio model built from the scene (not from an umarell cell).

Differences to ``WeldIK`` (all overrides; the solver code itself is inherited):
  * one Pinocchio model per scene robot: URDF, tip_link, TCP offset and rotation from the scene; the IK frame is
    R_tcp @ C with C @ [-1, 0, 0] = tool.axis, so WeldIK's "tool axis = -x, spin about x" holds for any tool axis;
  * the base placement is a full rotation (``RobotSpec.base_R``), not only a yaw;
  * self collision: the robot's own ``disabled_pairs`` (scene), not the SRDF next to the URDF;
  * joint limits: the scene's limits (the DLS projects onto them);
  * floor: hull rule against the scene's floor z (and a umarell-style frame-origin rule floor.z - 0.05);
  * pose tolerances: the scene's ``tolerances`` (position, tool axis).
"""
from __future__ import annotations

import numpy as np
import pinocchio as pin

from expert.ik import IKResult, WeldIK, pose_errors
from trajgen.robot import internal_tcp_rotation
from trajgen.scene import Scene

TCP_FRAME = "welding_tcp"


class _SceneArm:
    """Pinocchio model + convex collision model of one scene robot (attributes as expert.ik._PinArm)."""

    def __init__(self, robot, C: np.ndarray, disabled_pairs=None):
        urdf = str(robot.urdf)
        self.model = pin.buildModelFromUrdf(urdf)
        self.cmodel = pin.buildGeomFromUrdf(self.model, urdf, pin.GeometryType.COLLISION, None, str(robot.urdf.parent))
        m = self.model
        ft = m.getFrameId(robot.tool.tip_link)
        if ft >= m.nframes:
            raise ValueError(f"{robot.id}: tip_link {robot.tool.tip_link} not a Pinocchio frame")
        parent = m.frames[ft]
        placement = parent.placement * pin.SE3(np.asarray(robot.tool.tcp_R, float) @ C, np.asarray(robot.tool.tcp_offset, float))
        self.tcp_id = m.addFrame(pin.Frame(TCP_FRAME, parent.parentJoint, ft, placement, pin.FrameType.OP_FRAME))
        self.data = m.createData()
        self.hull_vertices = []
        for go in self.cmodel.geometryObjects:
            g = go.geometry
            g.buildConvexRepresentation(False)
            verts = np.array(g.convex.points(), float) if hasattr(g.convex, "points") else None
            if verts is None or verts.ndim != 2:
                verts = np.array([g.vertices(i) for i in range(g.num_vertices)], float)
            self.hull_vertices.append(verts)
            go.geometry = g.convex
        self.cmodel.addAllCollisionPairs()
        disabled = {frozenset(p) for p in (robot.coll.disabled_pairs if disabled_pairs is None else disabled_pairs)}
        link_of = [m.frames[go.parentFrame].name for go in self.cmodel.geometryObjects]
        drop = [(int(cp.first), int(cp.second)) for cp in self.cmodel.collisionPairs
                if link_of[cp.first] == link_of[cp.second] or frozenset((link_of[cp.first], link_of[cp.second])) in disabled]
        for i, j in drop:
            self.cmodel.removeCollisionPair(pin.CollisionPair(i, j))
        self.gdata = pin.GeometryData(self.cmodel)
        self.lo = np.asarray(robot.q_lo, float).copy()
        self.hi = np.asarray(robot.q_hi, float).copy()
        self.n_frames_umarell = m.nframes            # frames 3.. = base_link, joints, links, TCP

    def fk(self, q):
        pin.framesForwardKinematics(self.model, self.data, np.asarray(q, float))
        M = self.data.oMf[self.tcp_id]
        return M.translation.copy(), M.rotation.copy()


class _SceneKin:
    def __init__(self, arms):
        self.arms = arms

    def tcp_local(self, rid, q):
        return self.arms[rid].fk(q)

    def jac_local(self, rid, q):
        a = self.arms[rid]
        return pin.computeFrameJacobian(a.model, a.data, np.asarray(q, float), a.tcp_id, pin.LOCAL)


class _RobotView:
    """What WeldIK reads from ``self.cell.robots[rid]`` (base, yaw, home, limits, urdf, tcp_offset)."""

    def __init__(self, r):
        self.rid, self.base, self.yaw, self.home = r.id, np.asarray(r.base_xyz, float), float(r.yaw), np.asarray(r.home, float)
        self.R = np.asarray(r.base_R, float)
        self.q_lo, self.q_hi = np.asarray(r.q_lo, float), np.asarray(r.q_hi, float)
        self.urdf, self.tcp_offset = r.urdf, np.asarray(r.tool.tcp_offset, float)


class SceneIK(WeldIK):
    """``WeldIK`` on a scene (any number of robots, any base orientation, any tool axis)."""

    def __init__(self, scene: Scene, self_margin: float = 0.005, margin_target: float = 0.010, strict_rot: bool = False):
        self.scene = scene
        self.C = internal_tcp_rotation(scene.tool_axis_local)
        self.robots = {r.id: _RobotView(r) for r in scene.robots}
        self.pin = _SceneKin({r.id: _SceneArm(r, self.C, scene.disabled_pairs_of(r.id)) for r in scene.robots})
        self.kin = self.pin
        self.cell = None
        self.self_margin, self.floor_margin = self_margin, 0.0
        self.margin_target, self.strict_rot = margin_target, strict_rot
        self.rng = np.random.default_rng(0)
        tol = scene.tolerances
        self.pos_tol, self.axis_tol = float(tol.position), float(tol.tool_axis_deg)

    # ------------------------------------------------------------------ frames
    def robot(self, rid):
        return self.robots[rid]

    def to_internal(self, R_tcp_world):
        """Scene TCP rotation -> IK frame rotation (tool axis = -x)."""
        return np.asarray(R_tcp_world, float) @ self.C

    def to_scene(self, R_int):
        return np.asarray(R_int, float) @ self.C.T

    def to_local(self, rid, p, R):
        r = self.robot(rid)
        return r.R.T @ (np.asarray(p, float) - r.base), r.R.T @ R

    def tcp_world(self, rid, q):
        r = self.robot(rid)
        p, R = self.kin.tcp_local(rid, q)
        return r.base + r.R @ p, r.R @ R

    def default_seeds(self, rid, n_random=200, target_world=None):
        r = self.robot(rid)
        seeds = [r.home.copy(), np.array([0.0, -1.0, 1.5, 0.0, -0.5, 0.0]),
                 np.array([0.5, -1.0, 1.5, 0.0, -0.5, 0.0]), np.array([-0.5, -1.0, 1.5, 0.0, -0.5, 0.0])]
        lo, hi = r.q_lo, r.q_hi
        if target_world is not None:
            d = r.R.T @ (np.asarray(target_world, float) - r.base)            # target in the base frame
            th = np.arctan2(d[1], d[0])
            for a1 in (-th, -th + np.pi, -th - np.pi, th, th + np.pi, th - np.pi):
                for a2, a3 in ((-1.2, 1.3), (-0.6, 0.6), (-1.8, 2.0), (-0.4, -1.5), (-1.6, -0.8)):
                    for a5 in (-1.2, 1.2):
                        for a4 in (-np.pi, 0.0, np.pi):
                            s = np.array([a1, a2, a3, a4, a5, 0.0])
                            if np.all(s[:3] >= lo[:3]) and np.all(s[:3] <= hi[:3]) and \
                                    not any(np.abs(s - x).max() < 1e-9 for x in seeds):
                                seeds.append(s)
        rng = self.rng
        for _ in range(n_random):
            seeds.append(rng.uniform(lo, hi))
        return seeds

    # ------------------------------------------------------------------ checks
    def floor_z(self, rid, q) -> float:
        """Lowest world z of the collision-hull vertices (base_link excluded), minus the scene floor z."""
        a = self.pin.arms[rid]
        r = self.robot(rid)
        pin.updateGeometryPlacements(a.model, a.data, a.cmodel, a.gdata, np.asarray(q, float))
        zmin = np.inf
        for go, M, V in zip(a.cmodel.geometryObjects, a.gdata.oMg, a.hull_vertices):
            if go.parentJoint == 0:
                continue
            W = (V @ M.rotation.T + M.translation) @ r.R.T + r.base
            zmin = min(zmin, float(W[:, 2].min()))
        fl = self.scene.floor
        return zmin - (fl.z if fl.enabled else -np.inf)

    def umarell_floor_ok(self, rid, q) -> bool:
        """umarell's rule relative to the scene floor: frame origins from base_link on at z >= floor.z - 0.05."""
        fl = self.scene.floor
        if not fl.enabled:
            return True
        a = self.pin.arms[rid]
        r = self.robot(rid)
        pin.framesForwardKinematics(a.model, a.data, np.asarray(q, float))
        z = [(r.R @ a.data.oMf[i].translation)[2] + r.base[2] for i in range(3, a.model.nframes)]
        return bool(min(z) >= fl.z - 0.05)

    def check(self, rid, q, p_target=None, R_target=None) -> IKResult:
        """``R_target`` in the IK frame (tool axis -x), as everywhere inside WeldIK."""
        r = self.robot(rid)
        q = np.asarray(q, float)
        reasons = []
        if np.any(q < r.q_lo - 1e-9) or np.any(q > r.q_hi + 1e-9):
            reasons.append("joint_limits")
        sd = self.self_distance(rid, q)
        if sd < self.self_margin:
            reasons.append(f"self_collision({sd:.4f})")
        fz = self.floor_z(rid, q)
        fl = self.scene.floor
        if fl.enabled and fl.hull_rule and fz < self.floor_margin:
            reasons.append(f"floor({fz:.3f})")
        uf = self.umarell_floor_ok(rid, q)
        if not uf:
            reasons.append("umarell_floor")
        err = {}
        if p_target is not None:
            p, R = self.tcp_world(rid, q)
            err = pose_errors(p, R, p_target, R_target)
            ok = err["pos_m"] <= self.pos_tol and err["tool_axis_deg"] <= self.axis_tol
            if self.strict_rot:
                ok = ok and err["rot_deg"] <= self.axis_tol
            if not ok:
                reasons.append("pose_tolerance")
        return IKResult(q=q, err=err, valid=not reasons, reasons=reasons, self_dist=sd, floor_z=fz,
                        umarell_floor_ok=uf)
