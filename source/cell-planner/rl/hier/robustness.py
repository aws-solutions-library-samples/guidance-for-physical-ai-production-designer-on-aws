"""Weld-pose and self-distance margins of a rollout under tracking noise (Track H, verifier finding m2/m3).

For every weld of a rollout, at the configuration where the arm arrives on the weld:
  * nominal tool-axis error (deg, vs the weld's weld_orientation tool axis) and position error (mm);
  * fail rates of the umarell gate (tool axis <= 5 deg, position <= 10 mm) under
      - orientation noise: random rotation vector, N(0, 0.5 deg) per axis, applied to the TCP frame;
      - position noise: N(0, 1 mm) and N(0, 2 mm) per axis;
      - joint noise: U(-0.005, 0.005) rad per joint (FK; axis and position gate reported separately);
  * hull self distance (coal, URDF convex hulls) at the weld and its minimum under the joint noise.
Plus the minimum hull self distance of each arm over the whole rollout (every 20 Hz sample).
"""
from __future__ import annotations

import numpy as np

from env.cell import ROBOT_IDS, load_cell

AXIS_TOL, POS_TOL = 5.0, 0.010


def _rotvec_R(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def weld_margins(cell_id, arrivals, task_pos, n=2000, n_joint=200, seed=0):
    """arrivals: [(rid, wid, q (6,))]; task_pos {wid: (3,)} (the episode's weld positions)."""
    from env.rewards import quat_xyzw_to_mat
    from expert.ik import WeldIK
    cell = load_cell(cell_id)
    ik = WeldIK(cell)
    by = {w.wid: w for w in cell.welds}
    rng = np.random.default_rng(seed)
    out = []
    for rid, wid, q in arrivals:
        q = np.asarray(q, float)
        Rw = quat_xyzw_to_mat(np.asarray(by[wid].quat_xyzw, float)[None])[0]
        a_nom = -Rw[:, 0]
        p, R = ik.tcp_world(rid, q)
        pw = np.asarray(task_pos[wid], float)

        def axis_err(Rm):
            return np.degrees(np.arccos(np.clip(-Rm[:, 0] @ a_nom, -1, 1)))
        e_axis, e_pos = float(axis_err(R)), float(np.linalg.norm(p - pw))
        rv = rng.normal(0, np.radians(0.5), (n, 3))
        f_ori = float(np.mean([axis_err(_rotvec_R(v) @ R) > AXIS_TOL for v in rv]))
        f_p1 = float(np.mean(np.linalg.norm(p - pw + rng.normal(0, 0.001, (n, 3)), axis=1) > POS_TOL))
        f_p2 = float(np.mean(np.linalg.norm(p - pw + rng.normal(0, 0.002, (n, 3)), axis=1) > POS_TOL))
        fj, fjp, sdj = 0, 0, np.inf
        for _ in range(n_joint):
            qj = q + rng.uniform(-0.005, 0.005, 6)
            pj, Rj = ik.tcp_world(rid, qj)
            fj += axis_err(Rj) > AXIS_TOL
            fjp += np.linalg.norm(pj - pw) > POS_TOL
            sdj = min(sdj, ik.self_distance(rid, qj))
        out.append(dict(robot=rid, weld=wid, axis_err_deg=e_axis, pos_err_mm=1000 * e_pos,
                        self_dist_mm=1000 * ik.self_distance(rid, q), fail_ori_0p5deg=f_ori, fail_pos_1mm=f_p1,
                        fail_pos_2mm=f_p2, fail_joint_axis_0p005rad=fj / n_joint,
                        fail_joint_pos_0p005rad=fjp / n_joint, min_self_dist_joint_noise_mm=1000 * sdj))
    return out


def rollout_self_distance(cell_id, q):
    """Min coal hull self distance per arm over a rollout q (N,2,6) -> {rid: (min m, sample index)}."""
    from expert.ik import WeldIK
    ik = WeldIK(load_cell(cell_id))
    res = {}
    for i, rid in enumerate(ROBOT_IDS):
        d = np.array([ik.self_distance(rid, x) for x in np.asarray(q)[:, i]])
        res[rid] = (float(d.min()), int(d.argmin()))
    return res


def summary(margins):
    m = margins
    return dict(max_axis_err_deg=max(x["axis_err_deg"] for x in m), max_pos_err_mm=max(x["pos_err_mm"] for x in m),
                min_weld_self_dist_mm=min(x["self_dist_mm"] for x in m),
                worst_fail_ori_0p5deg=max(x["fail_ori_0p5deg"] for x in m),
                mean_fail_ori_0p5deg=float(np.mean([x["fail_ori_0p5deg"] for x in m])),
                worst_fail_pos_2mm=max(x["fail_pos_2mm"] for x in m),
                worst_fail_joint_axis=max(x["fail_joint_axis_0p005rad"] for x in m),
                worst_fail_joint_pos=max(x["fail_joint_pos_0p005rad"] for x in m),
                min_self_dist_joint_noise_mm=min(x["min_self_dist_joint_noise_mm"] for x in m),
                p_any_weld_fails_ori=1 - float(np.prod([1 - x["fail_ori_0p5deg"] for x in m])))
