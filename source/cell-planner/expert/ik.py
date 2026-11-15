"""Damped-least-squares IK for the KR240 + stick-tool TCP, with validity checks.

Backend: Pinocchio 4 (RL venv) loading the cell's dressed URDF, with the TCP frame
attached to ``link_6`` at ``tcp_offset`` exactly like umarell's ``utils/arm_model.add_tcp_frame``.
Kinematics are computed in the robot base frame; the world pose is ``base + Rz(yaw) @ local``
(the same convention as umarell's ``run_ik`` / ``validate_approach``).

The kinematic backend is swappable (``WeldIK(kin_backend=...)``): any object with
``tcp_local(rid, q) -> (p (3,), R (3,3))`` and ``jac_local(rid, q) -> J (6,6)`` (LOCAL frame
[v; w]) works; ``PinKinematics`` is the default.

Validity of a configuration (``check``):
  * joint limits (robots.yaml limits = URDF limits),
  * self-collision on the convex hulls (coal via ``pin.computeDistances``), adjacent pairs
    from the SRDF disabled, required min distance ``self_margin`` (default 0.005 m),
  * floor: every hull vertex at z >= ``floor_margin`` (default 0.0 m) -- stricter than umarell's
    own floor rule (frame spheres idx>2 at z >= -0.05), which is reported as well,
  * pose tolerances of umarell ``validate_approach``: tool-axis angle <= 5 deg (this is what
    umarell actually gates on), position <= 10 mm, plus the full geodesic rotation error which we
    also require <= 5 deg (stricter; DESIGN.md wording). Solutions are solved to ~1e-9.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property

import numpy as np
import pinocchio as pin

from env.cell import APPROACH_TOL_DEG, ORI_TOL_DEG, POS_TOL_M, Cell, Weld, load_cell, srdf_of

APPROACH_OFFSET_M = 0.10   # umarell run_trajectory_planner(approach_offset_m=0.1)
TCP_FRAME = "welding_tcp"


def quat_xyzw_to_R(q) -> np.ndarray:
    """Same formula as umarell ``pinocchio._quat_to_R`` (normalised)."""
    x, y, z, w = (float(v) for v in q)
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tool_axis(R: np.ndarray) -> np.ndarray:
    a = -R[:, 0]
    return a / (np.linalg.norm(a) + 1e-9)


def pose_errors(p, R, target_p, target_R, approach=None, p_prev=None) -> dict:
    """umarell ``validate_approach`` error definitions.

    ``tool_axis_deg``: angle between -R[:,0] and -Rexp[:,0] (umarell's gate, <= 5 deg);
    ``rot_deg``: geodesic angle of R^T Rexp (umarell reports it, does not gate on it);
    ``pos_m``: ||p - target||; ``approach_deg`` only when ``p_prev`` (start of the last
    ``approach_span_m`` of travel) is given: angle between (p - p_prev) and the approach vector.
    """
    a, e = tool_axis(R), tool_axis(target_R)
    axis = float(np.degrees(np.arccos(np.clip(a @ e, -1.0, 1.0))))
    Rrel = R.T @ target_R
    rot = float(np.degrees(np.arccos(np.clip((np.trace(Rrel) - 1.0) / 2.0, -1.0, 1.0))))
    out = {"pos_m": float(np.linalg.norm(np.asarray(p) - target_p)), "tool_axis_deg": axis, "rot_deg": rot,
           "approach_deg": None}
    if approach is not None and p_prev is not None:
        mv = np.asarray(p) - np.asarray(p_prev)
        nm = np.linalg.norm(mv)
        if nm > 1e-6:
            av = np.asarray(approach, float) / (np.linalg.norm(approach) + 1e-9)
            out["approach_deg"] = float(np.degrees(np.arccos(np.clip(mv / nm @ av, -1.0, 1.0))))
    return out


def axis_angle_R(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def spin_tilt_R(R, spin_deg=0.0, tilt_deg=0.0, tilt_dir_deg=0.0):
    """R . Rt . Rs: spin about the TCP x axis (the tool axis is -x, so it is unchanged) and tilt
    the tool axis by ``tilt_deg`` about the TCP-frame axis (0, cos d, sin d)."""
    Rs = axis_angle_R([1.0, 0.0, 0.0], np.deg2rad(spin_deg))
    d = np.deg2rad(tilt_dir_deg)
    Rt = axis_angle_R([0.0, np.cos(d), np.sin(d)], np.deg2rad(tilt_deg)) if tilt_deg else np.eye(3)
    return R @ Rt @ Rs


def within_tolerance(err: dict, strict_rot: bool = False) -> bool:
    ok = err["pos_m"] <= POS_TOL_M and err["tool_axis_deg"] <= ORI_TOL_DEG
    if strict_rot:
        ok = ok and err["rot_deg"] <= ORI_TOL_DEG
    if err.get("approach_deg") is not None:
        ok = ok and err["approach_deg"] <= APPROACH_TOL_DEG
    return ok


class _PinArm:
    """One Pinocchio model + collision model per robot (both robots share the URDF)."""

    def __init__(self, robot):
        urdf = str(robot.urdf)
        self.model = pin.buildModelFromUrdf(urdf)
        self.cmodel = pin.buildGeomFromUrdf(self.model, urdf, pin.GeometryType.COLLISION, None, str(robot.urdf.parent))
        m = self.model
        f6 = m.getFrameId(getattr(robot, "tip_link", "") or "like_kr240r2900_link_6")   # TCP parent (docs/p3_urdf.md)
        parent = m.frames[f6]
        placement = parent.placement * pin.SE3(np.eye(3), np.asarray(robot.tcp_offset, float))
        self.tcp_id = m.addFrame(pin.Frame(TCP_FRAME, parent.parentJoint, f6, placement, pin.FrameType.OP_FRAME))
        self.data = m.createData()
        # Convex hulls -> coal convex objects (the meshes are already hulls).
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
        pin.removeCollisionPairs(m, self.cmodel, str(srdf_of(robot)))
        self.gdata = pin.GeometryData(self.cmodel)
        self.lo = m.lowerPositionLimit.copy()
        self.hi = m.upperPositionLimit.copy()

    def fk(self, q):
        pin.framesForwardKinematics(self.model, self.data, np.asarray(q, float))
        M = self.data.oMf[self.tcp_id]
        return M.translation.copy(), M.rotation.copy()


class PinKinematics:
    """Default kinematic backend (Pinocchio)."""

    def __init__(self, cell: Cell):
        self.arms = {rid: _PinArm(r) for rid, r in cell.robots.items()}

    def tcp_local(self, rid, q):
        return self.arms[rid].fk(q)

    def jac_local(self, rid, q):
        a = self.arms[rid]
        return pin.computeFrameJacobian(a.model, a.data, np.asarray(q, float), a.tcp_id, pin.LOCAL)


@dataclass
class IKResult:
    q: np.ndarray
    err: dict
    valid: bool
    reasons: list
    self_dist: float
    floor_z: float
    umarell_floor_ok: bool
    spin_deg: float = 0.0        # rotation about the tool axis applied to weld_orientation
    tilt_deg: float = 0.0        # tool-axis tilt (<= 5 deg tolerance) applied to weld_orientation
    tilt_dir_deg: float = 0.0    # direction of the tilt axis in the TCP y-z plane
    R_target: np.ndarray | None = None   # the solved TCP rotation (world)
    thin: bool = False           # valid but hull self-distance < WeldIK.margin_target


class WeldIK:
    def __init__(self, cell: Cell | str = "welding_cell_2_small", kin_backend=None,
                 self_margin: float = 0.005, floor_margin: float = 0.0, margin_target: float = 0.010,
                 strict_rot: bool = False):
        self.cell = load_cell(cell) if isinstance(cell, str) else cell
        self.pin = PinKinematics(self.cell)
        self.kin = kin_backend or self.pin
        self.self_margin, self.floor_margin = self_margin, floor_margin
        # margin_target: preferred hull self-distance; strict_rot: also require the geodesic
        # rotation error <= 5 deg (umarell/env only gate the tool axis -> default False)
        self.margin_target, self.strict_rot = margin_target, strict_rot
        self.rng = np.random.default_rng(0)

    # ------------------------------------------------------------------ frames
    def robot(self, rid):
        return self.cell.robots[rid]

    def to_local(self, rid, p, R):
        r = self.robot(rid)
        Rz = rot_z(-r.yaw)
        return Rz @ (np.asarray(p, float) - r.base), Rz @ R

    def tcp_world(self, rid, q):
        r = self.robot(rid)
        p, R = self.kin.tcp_local(rid, q)
        Rz = rot_z(r.yaw)
        return r.base + Rz @ p, Rz @ R

    def weld_target(self, weld: Weld, offset: float = 0.0):
        """World target pose; ``offset`` > 0 = pre-weld standoff back along -approach_vector."""
        av = np.asarray(weld.approach, float)
        av = av / (np.linalg.norm(av) + 1e-9)
        return weld.pos - av * offset, quat_xyzw_to_R(weld.quat_xyzw)

    # ------------------------------------------------------------------ solver
    def _dls(self, rid, q, pl, Rl, iters=200, tol=1e-9, lam0=1e-3):
        """Levenberg-Marquardt DLS on the SE3 log error, projected onto the joint box."""
        a = self.pin.arms[rid]
        lo, hi = a.lo, a.hi
        Md = pin.SE3(Rl, pl)
        q = np.clip(np.asarray(q, float).copy(), lo, hi)
        lam = lam0
        p, R = self.kin.tcp_local(rid, q)
        e = pin.log(pin.SE3(R, p).actInv(Md)).vector
        en = float(e @ e)
        for _ in range(iters):
            if en < tol * tol:
                break
            J = self.kin.jac_local(rid, q)
            Jl = pin.Jlog6(Md.actInv(pin.SE3(R, p)))   # d e / d q = -Jlog6(Md^-1 M) J
            Je = Jl @ J
            dq = np.linalg.solve(Je.T @ Je + lam * np.eye(6), Je.T @ e)
            qn = np.clip(q + dq, lo, hi)
            pn, Rn = self.kin.tcp_local(rid, qn)
            en_new_v = pin.log(pin.SE3(Rn, pn).actInv(Md)).vector
            en_new = float(en_new_v @ en_new_v)
            if en_new < en:
                q, p, R, e, en = qn, pn, Rn, en_new_v, en_new
                lam = max(lam * 0.3, 1e-9)
            else:
                lam *= 10.0
                if lam > 1e6:
                    break
        return q, np.sqrt(en)

    def default_seeds(self, rid, n_random=200, target_world=None):
        r = self.robot(rid)
        seeds = [r.home.copy(), np.array([0.0, -1.0, 1.5, 0.0, -0.5, 0.0]),
                 np.array([0.5, -1.0, 1.5, 0.0, -0.5, 0.0]), np.array([-0.5, -1.0, 1.5, 0.0, -0.5, 0.0])]
        lo, hi = r.q_lo, r.q_hi
        # Structured seeds: base yaw towards / away from the target, elbow up/down, wrist flips.
        if target_world is not None:
            d = np.asarray(target_world) - r.base
            th = np.arctan2(d[1], d[0]) - r.yaw
            # KUKA a1 sign convention: joint_a1 has rpy (pi,0,pi) => a1 = -atan2.
            for a1 in (-th, -th + np.pi, -th - np.pi, th):
                for a2, a3 in ((-1.2, 1.3), (-0.6, 0.6), (-1.8, 2.0), (-0.4, -1.5), (-1.6, -0.8)):
                    for a5 in (-1.2, 1.2):
                        for a4 in (-np.pi, 0.0, np.pi):
                            s = np.array([a1, a2, a3, a4, a5, 0.0])
                            if np.all(s[:3] >= lo[:3]) and np.all(s[:3] <= hi[:3]):
                                seeds.append(s)
        span_lo = lo.copy(); span_hi = hi.copy()
        rng = self.rng
        for _ in range(n_random):
            seeds.append(rng.uniform(span_lo, span_hi))
        return seeds

    def solve_pose(self, rid, p_world, R_world, seeds=None, tol=1e-9, max_err=1e-6):
        """All converged configurations (unfiltered, not clustered) for a world TCP pose."""
        pl, Rl = self.to_local(rid, p_world, R_world)
        if seeds is None:
            seeds = self.default_seeds(rid, target_world=p_world)
        out = []
        for s in seeds:
            q, en = self._dls(rid, s, pl, Rl, tol=tol)
            if en < max_err:
                out.append(q)
        return out

    def solve(self, rid, weld: Weld, seeds=None, offset: float = 0.0, valid_only=True, cluster_tol=0.15):
        """Distinct solutions (``list[IKResult]``) for a weld (or its pre-weld if offset>0)."""
        p, R = self.weld_target(weld, offset)
        qs = self.solve_pose(rid, p, R, seeds)
        res = [self.check(rid, q, p, R) for q in cluster(qs, cluster_tol)]
        if valid_only:
            res = [r for r in res if r.valid]
        return res

    def solve_axis(self, rid, weld: Weld | None = None, offset: float = 0.0, p=None, R=None, seeds=None,
                   hints=None, spin_step: float = 5.0, max_spin: float | None = None,
                   tilts=(2.0, 3.0, 4.0, 4.5), tilt_dirs=tuple(range(0, 360, 30)), cluster_tol=0.15,
                   valid_only=True, approach=None, standoff: float = 0.10, min_standoff: float = 0.045,
                   n_steps: int = 20, extra_check=None, spin_window: float | None = None):
        """5-DOF weld IK: position + tool axis; the spin about the tool axis is free (umarell's
        validate_approach and the env only gate the tool axis), and the axis may tilt by up to
        ``max(tilts)`` (< 5 deg tolerance) if no spin works.

        Branches: all 6D solutions at spin 0 (``solve_pose`` with ``seeds``) or the ``hints``
        [(q, spin, tilt, dir)] (e.g. from data/weld_ik.json for a perturbed weld). Each branch is swept
        over spin by warm-started continuation. Per branch the preferred candidate is the smallest
        |spin| (then tilt) whose hull self-distance >= ``margin_target``; with ``approach`` given, the
        straight retract to the pre-weld standoff (fixed orientation, umarell ``_cartesian_descent``
        reversed) must also stay valid for >= ``min_standoff`` and the margin is taken as the minimum
        along that path. Without a comfortable candidate: the largest margin (``thin``); tilts are
        tried only then. ``extra_check(q) -> bool`` (e.g. the fixture rule) is applied to every
        configuration. Returns distinct ``IKResult``s; ``.path`` / ``.standoff_m`` / ``.path_min_sd``
        are set when ``approach`` is given. Pose errors are vs the unspun ``R``.
        """
        if weld is not None:
            p, R = self.weld_target(weld, offset)
            if approach is True:
                approach = weld.approach
        p, R = np.asarray(p, float), np.asarray(R, float)
        lim = 180.0 if max_spin is None else float(max_spin)
        av = None if approach is None else np.asarray(approach, float) / (np.linalg.norm(approach) + 1e-12)

        def evaluate(q, spin, tilt=0.0, tdir=0.0):
            c = self.check(rid, q, p, R)
            if c.valid and extra_check is not None and not extra_check(q):
                c.valid = False
                c.reasons.append("extra_check")
            c.spin_deg, c.tilt_deg, c.tilt_dir_deg = float(spin), float(tilt), float(tdir)
            c.R_target = spin_tilt_R(R, spin, tilt, tdir)
            return c

        def solve_at(q0, spin, tilt=0.0, tdir=0.0):
            q, en = self.refine(rid, q0, p, spin_tilt_R(R, spin, tilt, tdir))
            if en > 1e-6 or np.abs(q - q0).max() > 0.35:
                return None
            return q

        def with_path(c):
            """Attach the retract path; returns (usable, full, margin)."""
            if av is None:
                return c.valid, True, c.self_dist
            if not c.valid:
                return False, False, -np.inf
            path, ok, info = self.cartesian_path(rid, c.q, p, p - av * standoff, c.R_target, n_steps=n_steps)
            n_ok, sd = 0, c.self_dist
            for k in range(1, len(path)):
                if np.abs(path[k] - path[k - 1]).max() > 0.2:
                    break
                ck = self.check(rid, path[k])
                if not ck.valid or (extra_check is not None and not extra_check(path[k])):
                    break
                n_ok, sd = k, min(sd, ck.self_dist)
            c.path = path[: n_ok + 1]
            c.standoff_m = standoff * n_ok / n_steps
            c.path_min_sd = sd
            return c.standoff_m >= min_standoff - 1e-9, n_ok == n_steps, sd

        def pick(cands):
            ok = [c for c in cands if c.valid]
            # preference: comfortable margin at the weld pose, small spin/tilt
            ok.sort(key=lambda c: (c.self_dist < self.margin_target, c.tilt_deg, abs(c.spin_deg), -c.self_dist))
            best, best_key = None, None
            for c in ok[:60]:
                usable, full, m = with_path(c)
                if not usable:
                    continue
                if m >= self.margin_target and full:
                    return c, True
                key = (m >= self.margin_target, full, m)
                if best is None or key > best_key:
                    best, best_key = c, key
            return best, False

        branches = []          # list of (q, spin, tilt, dir) starting points
        if hints:
            for h in hints:
                qh, sp, tl, td = (list(h) + [0.0, 0.0, 0.0])[:4]
                q = solve_at(np.asarray(qh, float), sp, tl, td)
                if q is not None:
                    branches.append((q, float(sp), float(tl), float(td)))
        if not branches:
            qs = self.solve_pose(rid, p, R, seeds)
            branches = [(q, 0.0, 0.0, 0.0) for q in cluster(qs, cluster_tol)]
        out = []
        for q0, sp0, tl0, td0 in branches:
            cands = [evaluate(q0, sp0, tl0, td0)]
            if tl0 == 0.0:
                for sgn in (1.0, -1.0):
                    q, sp = q0, sp0
                    while True:
                        sp = sp + sgn * spin_step
                        if abs(sp) > lim + 1e-9 or (spin_window is not None and abs(sp - sp0) > spin_window + 1e-9):
                            break
                        qn = solve_at(q, sp)
                        if qn is None:
                            break
                        q = qn
                        cands.append(evaluate(q, sp))
            best, comfy = pick(cands)
            if not comfy and tilts:
                pool = sorted(cands, key=lambda c: -c.self_dist)[:6]
                tc = []
                for c in pool:
                    for tl in tilts:
                        for td in tilt_dirs:
                            qn = solve_at(c.q, c.spin_deg, tl, td)
                            if qn is not None:
                                tc.append(evaluate(qn, c.spin_deg, tl, td))
                b2, comfy2 = pick(tc)
                if b2 is not None:
                    m1 = -np.inf if best is None else getattr(best, "path_min_sd", best.self_dist)
                    m2 = getattr(b2, "path_min_sd", b2.self_dist)
                    if best is None or comfy2 or (m1 < self.margin_target and m2 > m1):
                        best = b2
            if best is None:
                best = max(cands, key=lambda c: c.self_dist)
                best.valid = False if av is not None else best.valid
                if av is not None:
                    best.reasons = best.reasons + ["no_valid_descent"]
                    best.path, best.standoff_m, best.path_min_sd = None, 0.0, -np.inf
            m = getattr(best, "path_min_sd", best.self_dist)
            best.thin = bool(best.valid and m < self.margin_target)
            out.append(best)
        res = []
        for c in sorted(out, key=lambda c: (not c.valid, c.thin, abs(c.spin_deg) + c.tilt_deg)):
            if all(np.abs(c.q - r.q).max() > cluster_tol for r in res):
                res.append(c)
        if valid_only:
            res = [r for r in res if r.valid]
        return res

    def refine(self, rid, q_seed, p_world, R_world):
        pl, Rl = self.to_local(rid, p_world, R_world)
        return self._dls(rid, q_seed, pl, Rl)

    # ------------------------------------------------------------------ checks
    def self_distance(self, rid, q) -> float:
        a = self.pin.arms[rid]
        pin.computeDistances(a.model, a.data, a.cmodel, a.gdata, np.asarray(q, float))
        return float(min(r.min_distance for r in a.gdata.distanceResults)) if a.gdata.distanceResults else np.inf

    def mesh_self_distance(self, rid, q) -> tuple[float, str]:
        """Self distance on the full VISUAL meshes (BVH, slow ~10-100 ms): tells whether a hull
        self-collision is real or a convex-hull artefact. Returns (min distance, pair name);
        distances of penetrating BVH pairs are reported as 0 / negative by coal."""
        a = self.pin.arms[rid]
        if not hasattr(a, "vmodel"):
            robot = self.robot(rid)
            a.vmodel = pin.buildGeomFromUrdf(a.model, str(robot.urdf), pin.GeometryType.VISUAL, None,
                                             str(robot.urdf.parent))
            a.vmodel.addAllCollisionPairs()
            pin.removeCollisionPairs(a.model, a.vmodel, str(srdf_of(robot)))
            a.vdata = pin.GeometryData(a.vmodel)
        q = np.asarray(q, float)
        pin.computeCollisions(a.model, a.data, a.vmodel, a.vdata, q, False)
        best, name = np.inf, ""
        for k, (cp, cr) in enumerate(zip(a.vmodel.collisionPairs, a.vdata.collisionResults)):
            n = f"{a.vmodel.geometryObjects[cp.first].name}|{a.vmodel.geometryObjects[cp.second].name}"
            if cr.isCollision():
                return 0.0, n
        pin.computeDistances(a.model, a.data, a.vmodel, a.vdata, q)
        for cp, dr in zip(a.vmodel.collisionPairs, a.vdata.distanceResults):
            if dr.min_distance < best:
                best = dr.min_distance
                name = f"{a.vmodel.geometryObjects[cp.first].name}|{a.vmodel.geometryObjects[cp.second].name}"
        return float(best), name

    def self_collides(self, rid, q) -> bool:
        a = self.pin.arms[rid]
        return bool(pin.computeCollisions(a.model, a.data, a.cmodel, a.gdata, np.asarray(q, float), False))

    def floor_z(self, rid, q) -> float:
        """Lowest hull vertex z in the world (base z added)."""
        a = self.pin.arms[rid]
        pin.updateGeometryPlacements(a.model, a.data, a.cmodel, a.gdata, np.asarray(q, float))
        zmin = np.inf
        for go, M, V in zip(a.cmodel.geometryObjects, a.gdata.oMg, a.hull_vertices):
            if go.parentJoint == 0:
                continue  # base link sits on the floor by design
            z = (V @ M.rotation.T + M.translation)[:, 2]
            zmin = min(zmin, float(z.min()))
        return zmin + float(self.robot(rid).base[2])

    def umarell_floor_ok(self, rid, q) -> bool:
        """umarell's rule: frame origins idx > 2 (pin frame list incl. TCP) at z >= -0.05."""
        a = self.pin.arms[rid]
        pin.framesForwardKinematics(a.model, a.data, np.asarray(q, float))
        z = [a.data.oMf[i].translation[2] + self.robot(rid).base[2] for i in range(3, a.model.nframes)]
        return bool(min(z) >= -0.05)

    def check(self, rid, q, p_target=None, R_target=None) -> IKResult:
        r = self.robot(rid)
        q = np.asarray(q, float)
        reasons = []
        if np.any(q < r.q_lo - 1e-9) or np.any(q > r.q_hi + 1e-9):
            reasons.append("joint_limits")
        sd = self.self_distance(rid, q)
        if sd < self.self_margin:
            reasons.append(f"self_collision({sd:.4f})")
        fz = self.floor_z(rid, q)
        if fz < self.floor_margin:
            reasons.append(f"floor({fz:.3f})")
        uf = self.umarell_floor_ok(rid, q)
        if not uf:
            reasons.append("umarell_floor")
        err = {}
        if p_target is not None:
            p, R = self.tcp_world(rid, q)
            err = pose_errors(p, R, p_target, R_target)
            if not within_tolerance(err, self.strict_rot):
                reasons.append("pose_tolerance")
        return IKResult(q=q, err=err, valid=not reasons, reasons=reasons, self_dist=sd, floor_z=fz,
                        umarell_floor_ok=uf)

    # ------------------------------------------------------------------ paths
    def cartesian_path(self, rid, q_start, p0, p1, R, n_steps=10):
        """Straight TCP line p0 -> p1 at fixed orientation, warm-started IK (like umarell
        ``_cartesian_descent``, 6 steps there; finer here). Returns (qs (n+1,6), ok, info)."""
        qs = [np.asarray(q_start, float)]
        worst = 0.0
        for i in range(1, n_steps + 1):
            a = i / n_steps
            p = (1 - a) * np.asarray(p0) + a * np.asarray(p1)
            q, en = self.refine(rid, qs[-1], p, R)
            if en > 1e-6:
                return np.array(qs), False, {"reason": f"ik_fail step {i} err {en:.2e}"}
            worst = max(worst, float(np.abs(q - qs[-1]).max()))
            qs.append(q)
        qs = np.array(qs)
        bad = [i for i, q in enumerate(qs) if not self.check(rid, q).valid]
        ok = not bad and worst < 0.2
        return qs, ok, {"max_step_rad": worst, "invalid_steps": bad}


def cluster(qs, tol=0.15):
    """Greedy clustering by max-abs joint distance; keeps the first representative."""
    reps = []
    for q in qs:
        if all(np.abs(q - r).max() > tol for r in reps):
            reps.append(np.asarray(q, float))
    return reps
