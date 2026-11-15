"""Certain-material wedge rule W(r, tol) (docs/workpiece_decision.md §1, §5); EnvConfig(wedge=True), off by default.

For every episode weld k (position p, unit approach vector a, tool axis t = -R_weld[:, 0]) the TCP x of either arm
violates the rule when |x - p| < radius and min((x - p)·a, (x - p)·t) > tol (more than tol inside the wedge
{rel·a > 0 and rel·t > 0}). This holds for all welds, both arms, and the whole cycle. The check is pure numpy (no
MuJoCo geoms) and samples the Cartesian TCP segment between consecutive collision sub-steps at `sweep` points.

Option (off by default) ``WedgeCfg.electrode_offsets`` = (s1, s2, ...) m: the rule is also applied to the points
TCP + s * R_tcp[:, 0] up the electrode axis (the tool axis is -R_tcp[:, 0]; e.g. (0.01, 0.02, 0.03), as
scripts/verify_ppo3_safety_dense_electrode.py, docs/verification_ppo_v3.md V3-3). Each electrode point is swept
like the TCP (linear between its positions at consecutive sub-steps). Contacts are labelled ``<robot>/e<mm>mm``;
the reported depth is the max over the TCP and the electrode points. ``margin`` (the obs) stays TCP-only.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class WedgeCfg:
    radius: float = 0.020     # m, ball around each weld point
    tol: float = 0.003        # m, depth > tol is a violation
    sweep: int = 4            # Cartesian sub-points between consecutive collision sub-steps
    margin_max: float = 0.05  # m, cap of the obs margin
    electrode_offsets: tuple = ()   # m up the electrode axis (TCP + s * R_tcp[:, 0]); () = TCP only (default)

    def to_dict(self):
        return asdict(self)


class Wedge:
    def __init__(self, cfg: WedgeCfg | dict | None = None, robot_ids=("robot1", "robot2")):
        self.cfg = cfg if isinstance(cfg, WedgeCfg) else WedgeCfg(**(cfg or {}))
        self.rids = robot_ids
        self.P = np.zeros((0, 3))
        self.A = np.zeros((0, 3))
        self.T = np.zeros((0, 3))
        self.ids = []

    def layout(self, pos, approach, tool_axis, ids):
        """pos/approach/tool_axis (n, 3) of the episode tasks (task_mask = 1), ids: task ids (weld names)."""
        self.P = np.asarray(pos, float).reshape(-1, 3)
        A = np.asarray(approach, float).reshape(-1, 3)
        self.A = A / np.maximum(np.linalg.norm(A, axis=-1, keepdims=True), 1e-12)
        T = np.asarray(tool_axis, float).reshape(-1, 3)
        self.T = T / np.maximum(np.linalg.norm(T, axis=-1, keepdims=True), 1e-12)
        self.ids = list(ids)

    def _dep(self, X):
        """X (m, 3) -> dist (m, n), dep (m, n)."""
        rel = X[:, None] - self.P[None]
        return np.linalg.norm(rel, axis=-1), np.minimum((rel * self.A[None]).sum(-1), (rel * self.T[None]).sum(-1))

    @property
    def electrode(self) -> bool:
        return len(self.cfg.electrode_offsets) > 0

    def check(self, x_prev, x, R_prev=None, R=None):
        """x_prev, x (2, 3) TCP positions -> depth (2,) (max dep inside the ball, -inf if none), contacts.
        R_prev, R (2, 3, 3) TCP rotations: required only with ``electrode_offsets``."""
        c = self.cfg
        depth = np.full(2, -np.inf)
        contacts = []
        if not len(self.P):
            return depth, contacts
        f = np.arange(1, max(1, c.sweep) + 1) / max(1, c.sweep)
        pts = [("tip", 0.0)]
        if self.electrode:
            if R is None or R_prev is None:
                raise ValueError("WedgeCfg.electrode_offsets needs the TCP rotations (R_prev, R)")
            pts += [(f"e{round(s * 1e3):g}mm", float(s)) for s in c.electrode_offsets]
        for i in range(2):
            for lab, s in pts:
                a0, a1 = (x_prev[i], x[i]) if s == 0.0 else (x_prev[i] + s * R_prev[i][:, 0], x[i] + s * R[i][:, 0])
                X = a0[None] + f[:, None] * (a1 - a0)[None]
                dist, dep = self._dep(X)
                inside = dist < c.radius
                if inside.any():
                    depth[i] = max(depth[i], float(dep[inside].max()))
                viol = inside & (dep > c.tol)
                for k in np.flatnonzero(viol.any(0)):
                    contacts.append((f"{self.rids[i]}/{lab}", f"wedge/{self.ids[k]}", -float(dep[viol[:, k], k].max())))
        return depth, contacts

    def margin(self, x):
        """x (2, 3) -> (2,) clip(min over welds with dist < 2 radius of (tol - dep), +-margin_max), else margin_max."""
        c = self.cfg
        out = np.full(2, c.margin_max)
        if not len(self.P):
            return out
        dist, dep = self._dep(np.asarray(x, float).reshape(2, 3))
        for i in range(2):
            m = dist[i] < 2 * c.radius
            if m.any():
                out[i] = np.clip((c.tol - dep[i][m]).min(), -c.margin_max, c.margin_max)
        return out
