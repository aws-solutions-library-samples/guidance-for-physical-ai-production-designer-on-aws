"""Optional workpiece model for KukaWeldEnv (``EnvConfig(workpiece=True)``; off by default). Track PPO v2.

welding_cell_2_small has no workpiece: the 15 welds float in free space, so nothing stops a tool from
approaching a weld from *below* the sheet (docs/verification_ppo.md I-4). This module adds a per-weld panel
patch (a "no-go half-space" just below each weld plane) and checks it against both arms. It is a collision
kind of its own (``"workpiece"``), and it terminates the episode like the others.

Geometry (task json `car_door_spot_welding.json`, welding_cell_2_small):

* All approach vectors are [0, 0, -1]. 12 welds sit on z = 1.35 m. W10 (+5 cm), W14 (+12.5 cm) and W15 (+15 cm)
  were moved up to clear the fixture, whose top is at z = 1.5 m.
* `weld_orientation`: "the two outer columns lean 45 degrees so the tool tips OUTWARD over the edge; the
  central column stays vertical". So the outer-column welds are on the panel's edge flanges, and the
  sheet lies on the *inner* side of them. W13 and W10 lean like the left column, W15 like the right one.
* A flat slab under the whole grid is therefore not plausible. A 45-degree tool's electrode bar dips ~6 mm
  below the tip ~5.6 cm outward of the weld, and the moved welds sit 5-15 cm above their neighbours. The
  model uses local patches instead:
  * **tip zone** (electrode tip + a 5 mm tip sphere centred 5 mm behind the TCP along the tool axis):
    |dx|, |dy| <= `tip_half` (6 cm) around each episode weld, from `below` (3 mm) to `below + thickness`
    (5 cm) under the weld plane. In words: the tip may not go more than 3 mm below the weld plane within
    ~6 cm lateral of any weld.
  * **hull patch** (all hulls of both arms except the electrode cap, whose convex hull protrudes ~4 mm beyond
    the TCP and is replaced by the tip sphere): a box `hull_half` (4 cm) around each weld, same depth range,
    restricted to the sheet side for leaning welds (it ends `edge` = 1 cm outward of the weld point). This
    is a MuJoCo box geom (contype/conaffinity 0, so it never creates contacts and leaves every other verdict
    unchanged), checked with `mj_geomDistance`.
* Penetration deeper than `tol` (0.1 mm) is a collision.
* Neighbouring welds stay feasible: W10 vs W11 are 10 cm apart and 5 cm apart in height; W11's vertical
  electrode bar (half width 4.4 cm) clears W10's 4 cm hull patch. Position noise can still make rare
  variants infeasible (two welds pushed within ~6 cm and at different heights); scripts/workpiece_audit.py
  reports this rate for the scripted expert.

``distances()`` returns per-arm minimum signed distances [tool (tip + tool hulls), rest of the arm], capped at
`dist_max`. The env exposes them as the obs ``robot_wp_dist`` (2, 2).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import mujoco
import numpy as np

N_BOX = 16
FAR = np.array([0.0, 0.0, -50.0])


@dataclass
class WorkpieceCfg:
    below: float = 0.003        # m: sheet top under the weld point
    thickness: float = 0.05     # m: patch depth
    tip_half: float = 0.06      # m: lateral half width of the tip zone
    hull_half: float = 0.04     # m: lateral half width of the hull patch
    edge: float = 0.01          # m: a leaning weld's flange ends this far outward of the weld point
    lean_tol: float = 0.2       # |tool axis x| above this = leaning weld (outer columns lean 45 deg)
    tip_radius: float = 0.005   # m: tip sphere (centre tip_radius behind the TCP)
    tol: float = 1e-4           # m: penetration deeper than this is a collision
    dist_max: float = 0.10      # m: distance cap (obs)

    def to_dict(self):
        return asdict(self)


def add_workpiece_geoms(spec: mujoco.MjSpec, n: int = N_BOX):
    """Add `n` contact-free box geoms named workpiece/box<k> to the world body (placed per episode)."""
    for k in range(n):
        g = spec.worldbody.add_geom()
        g.name = f"workpiece/box{k:02d}"
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [0.01, 0.01, 0.01]
        g.pos = FAR
        g.contype = 0
        g.conaffinity = 0
        g.group = 4
        g.rgba = [0.75, 0.75, 0.8, 0.6]


def _box_sdf(p, ctr, half):
    """p (3,), ctr/half (B,3) -> (B,) signed distance."""
    q = np.abs(p[None] - ctr) - half
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)


class Workpiece:
    def __init__(self, model: mujoco.MjModel, geom_owner: np.ndarray, tcp_site, cfg: WorkpieceCfg | dict | None = None):
        self.cfg = cfg if isinstance(cfg, WorkpieceCfg) else WorkpieceCfg(**(cfg or {}))
        self.m = model
        self.box = np.array([model.geom(f"workpiece/box{k:02d}").id for k in range(N_BOX)])
        self.tcp_site = np.asarray(tcp_site)
        self.arm_geoms, self.is_tool, self.cap = [], [], []
        for i in range(2):
            gs = [g for g in range(model.ngeom) if geom_owner[g] == i and model.geom_contype[g]]
            tool = [g for g in gs if model.body(model.geom_bodyid[g]).name.endswith("link_6")]
            cap = min(tool, key=lambda g: model.geom_rbound[g])   # electrode cap hull -> tip sphere
            gs = [g for g in gs if g != cap]
            self.cap.append(cap)
            self.arm_geoms.append(np.array(gs))
            self.is_tool.append(np.array([g in tool for g in gs]))
        self._fromto = np.zeros(6)
        self.n = 0

    # ------------------------------------------------------------------ layout
    def layout(self, weld_pos: np.ndarray, tool_axis: np.ndarray):
        """weld_pos (n, 3), tool_axis (n, 3) (= -R_weld[:, 0]) of this episode's welds."""
        c, m = self.cfg, self.m
        P = np.asarray(weld_pos, float).reshape(-1, 3)[:N_BOX]
        A = np.asarray(tool_axis, float).reshape(-1, 3)[:N_BOX]
        self.n = len(P)
        zc = P[:, 2] - c.below - c.thickness / 2
        self.tip_ctr = np.stack([P[:, 0], P[:, 1], zc], -1)
        self.tip_half = np.tile([c.tip_half, c.tip_half, c.thickness / 2], (self.n, 1))
        hc, hh = self.tip_ctr.copy(), np.tile([c.hull_half, c.hull_half, c.thickness / 2], (self.n, 1))
        for k in range(self.n):
            ax = A[k, 0]
            if abs(ax) > c.lean_tol:          # tool tips towards +x (ax > 0): it hangs over the edge at -x
                lo, hi = (P[k, 0] - c.edge, P[k, 0] + c.hull_half) if ax > 0 else (P[k, 0] - c.hull_half, P[k, 0] + c.edge)
                hc[k, 0], hh[k, 0] = (lo + hi) / 2, (hi - lo) / 2
        self.hull_ctr, self.hull_half = hc, hh
        for k, g in enumerate(self.box):
            if k < self.n:
                m.geom_pos[g], m.geom_size[g] = hc[k], hh[k]
                m.geom_rbound[g] = float(np.linalg.norm(hh[k]))
            else:
                m.geom_pos[g], m.geom_size[g] = FAR, [0.01, 0.01, 0.01]
                m.geom_rbound[g] = 0.02

    def describe(self):
        return dict(tip=[dict(center=c.round(4).tolist(), half=h.round(4).tolist()) for c, h in zip(self.tip_ctr, self.tip_half)],
                    hull=[dict(center=c.round(4).tolist(), half=h.round(4).tolist()) for c, h in zip(self.hull_ctr, self.hull_half)])

    # ------------------------------------------------------------------ queries
    def distances(self, d: mujoco.MjData, want_contacts: bool = True):
        """-> dist (2, 2) [tool, arm] min signed distance (capped), contacts [(body, zone, depth)]."""
        c, m = self.cfg, self.m
        out = np.full((2, 2), c.dist_max)
        pen = []
        if not self.n:
            return out, pen
        r = c.tip_radius
        for i in range(2):
            s = self.tcp_site[i]
            ctr = d.site_xpos[s] + r * d.site_xmat[s].reshape(3, 3)[:, 0]
            sdf = _box_sdf(ctr, self.tip_ctr, self.tip_half) - r
            k = int(np.argmin(sdf))
            out[i, 0] = min(out[i, 0], float(sdf[k]))
            if want_contacts and sdf[k] < -c.tol:
                pen.append((m.body(m.site_bodyid[s]).name + "/tip", f"workpiece/tip{k:02d}", float(sdf[k])))
            gs = self.arm_geoms[i]
            xp, rb = d.geom_xpos[gs], m.geom_rbound[gs]
            q = np.abs(xp[:, None] - self.hull_ctr[None]) - self.hull_half[None]       # (G,B,3)
            lb = np.linalg.norm(np.maximum(q, 0), axis=-1) - rb[:, None]                # distance lower bound
            for gi, bi in np.argwhere(lb < c.dist_max):
                g = int(gs[gi])
                dist = mujoco.mj_geomDistance(m, d, g, int(self.box[bi]), c.dist_max, self._fromto)
                col = 0 if self.is_tool[i][gi] else 1
                out[i, col] = min(out[i, col], dist)
                if want_contacts and dist < -c.tol:
                    pen.append((m.body(m.geom_bodyid[g]).name, f"workpiece/box{bi:02d}", float(dist)))
        return out, pen
