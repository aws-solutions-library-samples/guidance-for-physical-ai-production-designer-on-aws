"""Weld-pose check and reward, vectorised and shared by the env and the HER relabeller.

Goal vectors (per robot, last axis 12):
    achieved = [tcp_pos (3), tcp_rot6 (6), entry_dir (3)]
    desired  = [weld_pos (3), weld_rot6 (6), approach_vector (3)]
rot6 = first two columns of the rotation matrix (column-major: R[:,0], R[:,1]).
entry_dir = unit TCP displacement over the last >= ``approach_span`` metres of travel towards the
current pose (zeros when the TCP has not moved), i.e. what umarell's ``validate_approach``
compares with ``approach_vector``. HER may relabel ``desired`` with any achieved vector: the
first 9 entries become the goal pose, the last 3 can stay the original approach vector.

Pose check (= umarell ``validate_approach``, agents/trajectory/tools/pinocchio.py):
    position error  |p - p_w|                         <= 10 mm
    "orientation"   angle(-R[:,0], -R_w[:,0]) (tool axis lean)   <= 5 deg   (umarell's gate)
    entry direction angle(entry_dir, approach_vector) <= 15 deg  (skipped if no motion)
umarell also *reports* the full geodesic rotation error but does not gate on it; set
``RewardConfig.rot_tol_deg`` to also require it (stricter than umarell).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .cell import APPROACH_TOL_DEG, ORI_TOL_DEG, POS_TOL_M

GOAL_DIM = 12


@dataclass
class RewardConfig:
    pos_tol: float = POS_TOL_M
    axis_tol_deg: float = ORI_TOL_DEG
    rot_tol_deg: float | None = None          # None = umarell (axis only); 5.0 = also full rotation
    entry_tol_deg: float | None = APPROACH_TOL_DEG  # None = do not check the entry direction
    dwell_steps: int = 10                     # executor hold_duration_s 0.5 s at 20 Hz
    step_penalty: float = -0.001
    hold_reward: float = 0.02                 # per robot per step inside tolerance (dwell progress)
    weld_reward: float = 1.0                  # per completed weld
    completion_bonus: float = 2.0             # all welds of the episode done
    # |collision_penalty| should exceed the worst cumulative step penalty.
    collision_penalty: float = -5.0
    # potential shaping on the pose error to the current goal: phi = -(pos + rot_weight*axis_rad)
    shaping_coef: float = 1.0
    shaping_kind: str = "linear"              # "linear" | "log" (log adds gradient near 0)
    shaping_rot_weight: float = 0.3           # metres per radian of tool-axis error
    shaping_log_scale: float = 0.01           # metres; log kind: phi = -(e + s*log(1 + e/s)), 2x slope at 0
    shaping_clip: float = 2.0                 # cap on the error entering phi (m)
    # Track G: extra weight on the TCP's distance from the weld's approach line (through the weld along
    # approach_vector): phi -= perp_weight * perp. Rewards lining up and descending straight (entry gate).
    shaping_perp_weight: float = 0.0
    # arm-arm clearance shaping: penalty ramps from 0 at clearance_start to -clearance_coef at 0.20 m
    clearance_coef: float = 0.0
    clearance_start: float = 0.35
    clearance_limit: float = 0.20
    # hull self-distance shaping (Track G; needs EnvConfig.self_dist, off by default): per robot, penalty
    # ramps from 0 at self_start to -self_coef at 0 m of the min hull self-distance (obs robot_self_dist[:, 0])
    self_coef: float = 0.0
    self_start: float = 0.03


class RewardOut(NamedTuple):
    reward: np.ndarray        # (...)
    terminated: np.ndarray    # (...) bool: collision or all welds done
    dwell: np.ndarray         # (..., R) consecutive in-tolerance steps after this step (0 on completion)
    completed: np.ndarray     # (..., R) bool: this robot's goal weld completed this step
    match: np.ndarray         # (..., R) bool: pose within tolerance this step
    err: dict                 # pose_errors(achieved, desired)


# ----------------------------------------------------------------- geometry helpers
def _cross(a, b):
    return np.stack([a[..., 1] * b[..., 2] - a[..., 2] * b[..., 1],
                     a[..., 2] * b[..., 0] - a[..., 0] * b[..., 2],
                     a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]], -1)


def rot6_to_mat(r6: np.ndarray) -> np.ndarray:
    """(..., 6) -> (..., 3, 3), Gram-Schmidt (exact for rot6 taken from a rotation)."""
    r6 = np.asarray(r6, np.float64)
    a, b = r6[..., :3], r6[..., 3:6]
    c0 = a / np.maximum(np.sqrt((a * a).sum(-1, keepdims=True)), 1e-12)
    b = b - (c0 * b).sum(-1, keepdims=True) * c0
    c1 = b / np.maximum(np.sqrt((b * b).sum(-1, keepdims=True)), 1e-12)
    return np.stack([c0, c1, _cross(c0, c1)], axis=-1)


def mat_to_rot6(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R)
    return np.concatenate([R[..., :, 0], R[..., :, 1]], -1)


def quat_xyzw_to_mat(q: np.ndarray) -> np.ndarray:
    """Same formula as umarell ``_quat_to_R`` (normalised), batched."""
    q = np.asarray(q, np.float64)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def _angle(u, v):
    """Angle (rad) between unit vectors, batched."""
    return np.arccos(np.clip(np.sum(u * v, -1), -1.0, 1.0))


def rotvec_from_mat(R: np.ndarray) -> np.ndarray:
    """Axis-angle vector (..., 3) of rotation matrices (..., 3, 3) (log map)."""
    R = np.asarray(R, np.float64)
    cos = np.clip((np.trace(R, axis1=-2, axis2=-1) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(cos)
    v = np.stack([R[..., 2, 1] - R[..., 1, 2], R[..., 0, 2] - R[..., 2, 0], R[..., 1, 0] - R[..., 0, 1]], -1)
    s = np.sin(ang)
    small = s < 1e-6
    f = np.where(small, 0.5, ang / (2.0 * np.where(small, 1.0, s)))
    out = v * f[..., None]
    # near pi: use the diagonal (rare; the arms never need it for control features)
    near_pi = small & (cos < 0)
    if np.any(near_pi):
        d = np.sqrt(np.clip((np.diagonal(R, axis1=-2, axis2=-1) + 1.0) / 2.0, 0, None))
        sgn = np.where(v >= 0, 1.0, -1.0)
        out = np.where(near_pi[..., None], d * sgn * np.pi, out)
    return out


# ----------------------------------------------------------------- pose errors
def pose_errors(achieved: np.ndarray, desired: np.ndarray) -> dict:
    """Errors between achieved and desired goal vectors (..., 12). Angles in radians.

    Returns pos (m), axis (tool-axis lean, umarell's orientation gate), rot (geodesic),
    entry (angle of the entry direction to the approach vector; 0 when there is no motion),
    has_entry (bool).
    """
    a = np.asarray(achieved, np.float64)
    d = np.asarray(desired, np.float64)
    pos = np.linalg.norm(a[..., :3] - d[..., :3], axis=-1)
    Ra, Rd = rot6_to_mat(a[..., 3:9]), rot6_to_mat(d[..., 3:9])
    axis = _angle(Ra[..., :, 0], Rd[..., :, 0])
    tr = np.einsum("...ij,...ij->...", Ra, Rd)  # trace(Ra^T Rd)
    rot = np.arccos(np.clip((tr - 1.0) / 2.0, -1.0, 1.0))
    e = a[..., 9:12]
    en = np.linalg.norm(e, axis=-1)
    has_entry = en > 0.5
    av = d[..., 9:12] / np.maximum(np.linalg.norm(d[..., 9:12], axis=-1, keepdims=True), 1e-9)
    entry = np.where(has_entry, _angle(e / np.maximum(en, 1e-9)[..., None], av), 0.0)
    dp = a[..., :3] - d[..., :3]
    perp = np.linalg.norm(dp - (dp * av).sum(-1, keepdims=True) * av, axis=-1)
    return dict(pos=pos, axis=axis, rot=rot, entry=entry, has_entry=has_entry, perp=perp)


def pose_match(cfg: RewardConfig, achieved, desired, err: dict | None = None) -> np.ndarray:
    err = pose_errors(achieved, desired) if err is None else err
    ok = (err["pos"] <= cfg.pos_tol) & (err["axis"] <= np.deg2rad(cfg.axis_tol_deg))
    if cfg.rot_tol_deg is not None:
        ok &= err["rot"] <= np.deg2rad(cfg.rot_tol_deg)
    if cfg.entry_tol_deg is not None:
        ok &= err["entry"] <= np.deg2rad(cfg.entry_tol_deg)
    return ok


def _phi(cfg: RewardConfig, err: dict) -> np.ndarray:
    e = err["pos"] + cfg.shaping_rot_weight * err["axis"]
    if getattr(cfg, "shaping_perp_weight", 0.0):
        e = e + cfg.shaping_perp_weight * err["perp"]
    e = np.minimum(e, cfg.shaping_clip)
    if cfg.shaping_kind == "log":
        s = cfg.shaping_log_scale
        return -(e + s * np.log1p(e / s))
    return -e


def self_penalty(cfg: RewardConfig, self_dist) -> np.ndarray:
    """self_dist (..., R) min hull self-distance per robot (m) -> (...) penalty (summed over robots)."""
    if not getattr(cfg, "self_coef", 0.0) or self_dist is None:
        return 0.0
    d = np.asarray(self_dist, np.float64)
    x = np.clip((cfg.self_start - d) / cfg.self_start, 0.0, 1.0)
    return -cfg.self_coef * x.sum(-1)


def clearance_penalty(cfg: RewardConfig, clearance) -> np.ndarray:
    if not cfg.clearance_coef or clearance is None:
        return 0.0
    c = np.asarray(clearance, np.float64)
    x = np.clip((cfg.clearance_start - c) / (cfg.clearance_start - cfg.clearance_limit), 0.0, 1.0)
    return -cfg.clearance_coef * x


# ----------------------------------------------------------------- reward
def compute_reward(cfg: RewardConfig, achieved, desired, goal_mask, dwell, collision,
                   n_left=None, achieved_prev=None, clearance=None, self_dist=None) -> RewardOut:
    """Vectorised reward for one transition.

    achieved      (..., R, 12) after the step;  desired (..., R, 12) current goal per robot
    goal_mask     (..., R) 1 if the robot has a goal weld
    dwell         (..., R) consecutive in-tolerance steps *before* this step
    collision     (...) 0/1
    n_left        (...) open welds before this step (enables completion bonus + termination); None = skip
    achieved_prev (..., R, 12) before the step (for potential shaping); None = no shaping
    clearance     (...) arm-arm clearance after the step (for clearance shaping); None = skip
    self_dist     (..., R) min hull self-distance per robot after the step (self shaping); None = skip
    """
    goal_mask = np.asarray(goal_mask) > 0.5
    err = pose_errors(achieved, desired)
    match = pose_match(cfg, achieved, desired, err) & goal_mask
    dwell_next = np.where(match, np.asarray(dwell) + 1, 0)
    completed = match & (dwell_next >= cfg.dwell_steps)
    dwell_next = np.where(completed, 0, dwell_next)
    collision = np.asarray(collision, np.float64)

    r = (cfg.step_penalty + cfg.hold_reward * match.sum(-1) + cfg.weld_reward * completed.sum(-1)
         + cfg.collision_penalty * collision)
    if cfg.shaping_coef and achieved_prev is not None:
        err0 = pose_errors(achieved_prev, desired)
        same = goal_mask & ~completed  # goal unchanged across the transition
        r = r + cfg.shaping_coef * np.where(same, _phi(cfg, err) - _phi(cfg, err0), 0.0).sum(-1)
    r = r + clearance_penalty(cfg, clearance) + self_penalty(cfg, self_dist)

    terminated = collision > 0.5
    if n_left is not None:
        n_left = np.asarray(n_left)
        all_done = (n_left > 0) & (n_left - completed.sum(-1) <= 0)
        r = r + cfg.completion_bonus * all_done
        terminated = terminated | all_done
    return RewardOut(np.asarray(r, np.float32), np.asarray(terminated), dwell_next.astype(np.int64),
                     completed, match, err)


# ----------------------------------------------------------------- HER helpers
def achieved_from_obs(obs: dict) -> np.ndarray:
    """(..., R, 12) achieved goal from an observation dict (works on batched obs)."""
    return np.concatenate([obs["robot_tcp_pos"], obs["robot_tcp_rot6"], obs["robot_entry"]], -1)


def desired_from_obs(obs: dict) -> np.ndarray:
    return np.concatenate([obs["robot_goal"], obs["robot_goal_approach"]], -1)


def goal_features(tcp_pos, tcp_rot6, goal9, goal_mask=None):
    """Goal-relative obs features (recompute these when relabelling goals).

    Returns (goal_err (..., 6), goal_errmag (..., 3)):
      goal_err   = [R_tcp^T (p_goal - p_tcp), rotvec(R_tcp^T R_goal)]  (TCP frame)
      goal_errmag = [pos err (m), tool-axis err (rad), geodesic rot err (rad)]
    """
    R = rot6_to_mat(tcp_rot6)
    Rg = rot6_to_mat(goal9[..., 3:9])
    dp = np.asarray(goal9[..., :3], np.float64) - np.asarray(tcp_pos, np.float64)
    p_loc = np.einsum("...ji,...j->...i", R, dp)
    Rrel = np.einsum("...ji,...jk->...ik", R, Rg)
    rv = rotvec_from_mat(Rrel)
    mag = np.stack([np.linalg.norm(dp, axis=-1), _angle(R[..., :, 0], Rg[..., :, 0]),
                    np.linalg.norm(rv, axis=-1)], -1)
    err = np.concatenate([p_loc, rv], -1)
    if goal_mask is not None:
        m = (np.asarray(goal_mask) > 0.5)[..., None]
        err, mag = np.where(m, err, 0.0), np.where(m, mag, 0.0)
    return err.astype(np.float32), mag.astype(np.float32)


def dwell_from_matches(match: np.ndarray, dwell0=None) -> np.ndarray:
    """Consecutive-in-tolerance counts along the last axis (time) *before* each step.

    match (..., L) bool -> dwell (..., L): dwell[t] = number of consecutive True in match[:t]
    ending at t-1 (plus dwell0 if the run started before the window). Resetting on completion is
    left to the caller (relabelled goals rarely complete twice).
    """
    m = np.asarray(match, bool)
    L = m.shape[-1]
    idx = np.arange(1, L + 1)
    last_false = np.maximum.accumulate(np.where(~m, idx, 0), axis=-1)  # 1-based index of last False
    run = idx - last_false  # run length ending at t (inclusive)
    if dwell0 is not None:
        run = np.where(last_false == 0, run + np.asarray(dwell0)[..., None], run)
    prev = np.concatenate([np.zeros_like(run[..., :1]) if dwell0 is None else
                           np.asarray(dwell0)[..., None] * np.ones_like(run[..., :1]), run[..., :-1]], -1)
    return prev
