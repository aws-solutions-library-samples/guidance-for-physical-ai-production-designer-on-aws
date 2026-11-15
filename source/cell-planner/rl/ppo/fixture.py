"""Fixture distance field for the PPO 2t track (welding_cell_2t, 737 GEO_FIXTURE_OP1000 boxes).

The env's verdict stays umarell's exact box rule (env.kinematics.SphereModel.obstacle_terms, every sub-step). For the
policy's *observation* and a dense shaping term we need the same quantity many times per step, cheaply:

* `FixtureField`: a signed distance field to the union of the cell's boxes on a 2 cm grid (exact box SDF at the grid
  points, min over boxes), plus its gradient, trilinearly interpolated. Cached in runs/ppo_2t_assets/.
  `margin(rid, S)` = sdf(sphere centre) - sphere radius - box margin, i.e. umarell's box-rule margin
  (`dist(centre, box) - (r + 0.05)`) up to the grid interpolation error (tests: |err| < 1 cm where it matters).
* `FixtureFeatures`: per robot, the k spheres (umarell's box-checked spheres, `box_idx_u`) with the smallest margin:
  tanh(m / 5 cm), clip(m / 30 cm), unit gradient (3) and sphere position relative to the base / 2 m (3), both in the
  robot's base frame -> (B, R, 8k) robot-node features (rl.ppo.features FeatureSpec.fixture_k).
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = ROOT / "runs" / "ppo_2t_assets"
RES = 0.02
PAD = 0.45
F_PER = 8


def _box_sdf_grid(P, C, H, chunk=8192):
    """P (N,3) -> min over boxes of the exact box SDF (outside: Euclidean distance, inside: -depth)."""
    out = np.empty(len(P))
    for s in range(0, len(P), chunk):
        d = np.abs(P[s:s + chunk, None, :] - C[None]) - H[None]          # (n,B,3)
        outside = np.linalg.norm(np.maximum(d, 0.0), axis=-1)
        inside = np.minimum(d.max(-1), 0.0)
        out[s:s + chunk] = (outside + inside).min(-1)
    return out


class FixtureField:
    def __init__(self, cell_id: str = "welding_cell_2t", res: float = RES, pad: float = PAD):
        from env.cell import load_cell
        from env.kinematics import SphereModel
        self.sm = SphereModel(load_cell(cell_id))
        C, H = self.sm.box_center, self.sm.box_half
        if len(C) == 0:
            raise ValueError(f"{cell_id} has no obstacle boxes")
        self.margin_box = float(np.max(self.sm.box_margin))   # 0.05 for every box in welding_cell_2t
        lo = (C - H).min(0) - pad
        hi = (C + H).max(0) + pad
        lo[2] = max(lo[2], -0.1)
        key = hashlib.md5(np.concatenate([C.ravel(), H.ravel(), [res, pad]]).tobytes()).hexdigest()[:10]
        path = ASSET_DIR / f"fixture_sdf_{cell_id}_{key}.npz"
        if path.exists():
            z = np.load(path)
            self.lo, self.res, self.sdf = z["lo"], float(z["res"]), z["sdf"]
        else:
            n = np.ceil((hi - lo) / res).astype(int) + 1
            axes = [lo[k] + res * np.arange(n[k]) for k in range(3)]
            G = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
            self.lo, self.res = lo, res
            self.sdf = _box_sdf_grid(G, C, H).reshape(*n).astype(np.float32)
            ASSET_DIR.mkdir(parents=True, exist_ok=True)
            tmp = path.with_suffix(".tmp.npz")
            np.savez_compressed(tmp, lo=lo, res=res, sdf=self.sdf)
            tmp.replace(path)
        self.n = np.array(self.sdf.shape)
        self.grad = np.stack(np.gradient(self.sdf, self.res), -1).astype(np.float32)   # (nx,ny,nz,3)
        self.hi = self.lo + self.res * (self.n - 1)

    def query(self, P):
        """P (..., 3) -> sdf (...), gradient (..., 3) (trilinear; outside the grid: clamped + distance to it)."""
        P = np.asarray(P, np.float64)
        sh = P.shape[:-1]
        P = P.reshape(-1, 3)
        Pc = np.clip(P, self.lo, self.hi - 1e-9)
        extra = np.linalg.norm(P - Pc, axis=-1)
        u = (Pc - self.lo) / self.res
        i0 = np.floor(u).astype(int)
        i0 = np.minimum(i0, self.n - 2)
        f = u - i0
        d = np.zeros(len(P))
        g = np.zeros((len(P), 3))
        for dx in (0, 1):
            wx = f[:, 0] if dx else 1 - f[:, 0]
            for dy in (0, 1):
                wy = f[:, 1] if dy else 1 - f[:, 1]
                for dz in (0, 1):
                    wz = f[:, 2] if dz else 1 - f[:, 2]
                    w = wx * wy * wz
                    ix, iy, iz = i0[:, 0] + dx, i0[:, 1] + dy, i0[:, 2] + dz
                    d += w * self.sdf[ix, iy, iz]
                    g += w[:, None] * self.grad[ix, iy, iz]
        d = d + extra
        return d.reshape(sh), g.reshape(*sh, 3)

    def sphere_margins(self, rid: str, q):
        """q (..., 6) -> margins (..., M), gradients (..., M, 3), centres (..., M, 3) of the box-checked spheres."""
        a = self.sm._arm[rid]
        S = self.sm.spheres(rid, q)[..., a.box_idx_u, :]
        r = a.obst_radii[a.box_idx_u]
        d, g = self.query(S)
        return d - r - self.margin_box, g, S

    def min_margin(self, q12):
        """q (..., 2, 6) -> (..., 2) min box-rule margin per robot (approximate; the env check is exact)."""
        from env.cell import ROBOT_IDS
        q12 = np.asarray(q12, np.float64)
        return np.stack([self.sphere_margins(r, q12[..., i, :])[0].min(-1) for i, r in enumerate(ROBOT_IDS)], -1)


_FIELDS = {}


def field(cell_id: str = "welding_cell_2t") -> FixtureField:
    if cell_id not in _FIELDS:
        _FIELDS[cell_id] = FixtureField(cell_id)
    return _FIELDS[cell_id]


def fixture_features(fx: FixtureField, q, base, k: int):
    """q (B, 2, 6), base (B, 2, 4) [x, y, z, yaw] -> (B, 2, 8k) float32 (see module doc)."""
    from env.cell import ROBOT_IDS
    q = np.asarray(q, np.float64)
    base = np.asarray(base, np.float64)
    B = q.shape[0]
    out = np.zeros((B, 2, F_PER * k), np.float32)
    for i, rid in enumerate(ROBOT_IDS):
        m, g, S = fx.sphere_margins(rid, q[:, i])                       # (B,M), (B,M,3), (B,M,3)
        idx = np.argsort(m, axis=1)[:, :k]
        mk = np.take_along_axis(m, idx, 1)
        gk = np.take_along_axis(g, idx[..., None], 1)
        Sk = np.take_along_axis(S, idx[..., None], 1)
        gk = gk / np.maximum(np.linalg.norm(gk, axis=-1, keepdims=True), 1e-6)
        c, s_ = np.cos(base[:, i, 3]), np.sin(base[:, i, 3])
        Rinv = np.stack([np.stack([c, s_, 0 * c], -1), np.stack([-s_, c, 0 * c], -1),
                         np.stack([0 * c, 0 * c, 1 + 0 * c], -1)], -2)            # (B,3,3) world -> base
        gl = np.einsum("bij,bkj->bki", Rinv, gk)
        pl = np.einsum("bij,bkj->bki", Rinv, Sk - base[:, None, i, :3]) / 2.0
        f = np.concatenate([np.tanh(mk / 0.05)[..., None], np.clip(mk / 0.3, -1, 1)[..., None], gl, pl], -1)
        out[:, i] = f.reshape(B, -1)
    return out
