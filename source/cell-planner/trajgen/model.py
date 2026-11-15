"""The scene's collision / validity model, as one object the generator and the verifier share (any number of robots).

    m = SceneModel(scene, task)
    m.static_ok(rid, q)               # one configuration: every single-arm rule of the scene (planning gate)
    m.static_margins(rid, Q)          # (N,) margins per rule (floor spheres, floor hulls, boxes, rule W) for reports
    m.pair_clearance(ra, rb, Qa, Qb)  # (N,) corrected-sphere arm-arm clearance (gate: > collision.arm_arm.gate_m)
    m.mj_pair_contact(ra, rb, Qa, Qb) # (N,) bool MuJoCo hull contact between two arms
    m.tcp(rid, Q)                     # (N,3), (N,3,3) world TCP (scene TCP frame)

Rules (scenes/SCHEMA.md):
  * joint limits;
  * hull self-collision: coal signed distance (trajgen.ik.SceneIK, the robot's disabled pairs removed) >=
    ``self_margin`` for planning (default 5 mm, stricter than the scene's 0) and MuJoCo hull penetration for
    verification;
  * floor: corrected sphere rule (every link sphere after base_link, fill, TCP and every tool sphere, own radius,
    ``z - r >= floor.z + margin``) AND hull rule (every hull vertex except base_link at z >= floor.z);
  * boxes: umarell's box rule extended to the tool spheres (spheres index >= 3, distance from the centre to the
    box < r + collision_margin is a violation; workpiece boxes skip the TCP and electrode spheres) AND the hull rule
    (URDF collision hulls vs the boxes, MuJoCo signed distance >= obstacles.box_hull.min_distance_m; workpiece boxes
    too with ``obstacles.box_hull.workpiece: true``, every hull except the electrode hull(s));
  * rule W, per weld (scene default or the weld's ``rule_w``): TCP within ``radius`` of the weld and more than ``tol``
    inside its wedge;
  * electrode off-weld rule (``obstacles.box_hull.electrode_off_weld``, default off; docs/n2_contract.md 3.1): while
    the TCP is farther than ``radius`` from every weld of the task, electrode hull(s) - workpiece box signed distance
    (MuJoCo, ``MjWorld.electrode_distance``) >= ``min_distance`` (screened by the electrode hulls' bounding spheres);
    with the round-3 no-dip rule (``trajgen.nodip``, ``obstacles.box_hull.no_dip``) everywhere, except contacts
    inside the contact exemption (TCP within ``radius`` = 5 mm of a weld, contact point within 10 mm, depth
    <= 1.5 mm), and every other hull vs the workpiece boxes too;
  * arm vs arm, every robot pair: corrected-sphere clearance > gate (pair radius for every sphere) AND hull distance
    >= ``collision.arm_arm.hull_min_distance_m`` (``hull_backstop``; the planners hold it over the +-skew window); optional umarell parity: umarell's own 35-sphere tool grid on the SCENE's
    robot geometry (bases, homes, URDF) when the scene declares an umarell cell.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from trajgen.checks import SceneGeometry
from trajgen.scene import Scene, Task, bind_task

ROOT = Path(__file__).resolve().parents[1]
BOXFIELD_VERSION = 2


class _Parity:
    """umarell's arm-arm gate parameters (warning_distance, collision_radius_m)."""

    def __init__(self, cell_id, warning_distance, pair_radius):
        self.cell_id, self.warning_distance, self.pair_radius = cell_id, float(warning_distance), float(pair_radius)


class SceneModel:
    box_field_cache = True      # store / reuse the box distance field in data/trajgen/cache (scenegen: off)

    def __init__(self, scene: Scene, task: Task, *, self_margin: float = 0.005, margin_target: float = 0.015,
                 box_tool_spheres: bool = True):
        """``box_tool_spheres=False``: umarell's own box rule (link spheres from base_link on + the TCP sphere; the
        tool spheres are only checked arm-vs-arm, robots.yaml tool_collision) -- the boxes are then guarded against
        the tool by the box-hull rule (``obstacles.box_hull``) alone."""
        from trajgen.ik import SceneIK
        from trajgen.mjscene import MjWorld

        self.scene, self.task = scene, bind_task(scene, task)
        self.rids = scene.robot_ids
        self.n = len(self.rids)
        self.geo = SceneGeometry(scene, task)
        self.rm = self.geo.rm
        self.kin = self.rm
        self.ik = SceneIK(scene, self_margin=self_margin, margin_target=margin_target)
        self.self_margin = float(self_margin)
        self.robots = {r.id: r for r in scene.robots}
        # ---- box rule: per robot index set (link spheres 3.. + every tool sphere) and radii
        self.box_idx, self.box_r, self.wp_exempt = {}, {}, {}
        for rid, rm in self.rm.items():
            idx = np.concatenate([np.arange(3, rm.n_link), rm.n_link + np.arange(rm.n_tool if box_tool_spheres else 0)])
            self.box_idx[rid] = idx
            self.box_r[rid] = rm.sphere_r[idx]
            ex = [rm.tcp_frame] + ([rm.n_link + rm.electrode] if rm.electrode >= 0 else [])
            self.wp_exempt[rid] = np.isin(idx, ex)
        B = scene.boxes
        self.box_c = np.array([b.center for b in B], float).reshape(-1, 3)
        self.box_h = np.array([b.half_extents for b in B], float).reshape(-1, 3)
        self.box_R = np.array([np.eye(3) if b.R is None else b.R for b in B], float).reshape(-1, 3, 3)
        self.box_rot = any(b.R is not None for b in B)
        self.box_m = np.array([b.margin for b in B], float)
        self.box_wp = np.array([b.is_workpiece for b in B], bool)
        self.box_ids = [b.id for b in B]
        self.box_hull_wp = bool(scene.box_hull.enabled and scene.box_hull.workpiece)   # NR 1.5: cubes too
        self.box_hull_on = bool(scene.box_hull.enabled and any((not b.is_workpiece) or self.box_hull_wp for b in B))
        self.box_hull_min = float(scene.box_hull.min_distance)
        # ---- rule W, per weld
        self.W_ids = [w.id for w in task.welds]
        self.W_P = np.array([w.position for w in task.welds], float).reshape(-1, 3)
        self.W_A = np.array([w.approach for w in task.welds], float).reshape(-1, 3)
        self.W_T = np.array([w.tool_axis for w in task.welds], float).reshape(-1, 3)
        rw = [w.rule_w_for(scene) for w in task.welds]
        self.W_on = np.array([x.enabled for x in rw], bool)
        self.W_rad = np.array([x.radius for x in rw], float)
        self.W_tol = np.array([x.tol for x in rw], float)
        self.wedge_on = bool(self.W_on.any())
        self.w_radius, self.w_tol = float(scene.workpiece.radius), float(scene.workpiece.tol)   # scene defaults
        self.mjw = MjWorld(scene)
        self._field = None
        # ---- electrode off-weld rule (round 2; off unless the scene enables it)
        er = scene.box_hull.electrode
        self.electrode_rule = er
        self.electrode_enabled = bool(er.enabled)                       # the verifier reports the check
        self.electrode_on = bool(er.enabled and self.box_wp.any())     # something to check
        self.nodip = bool(self.electrode_on and getattr(er, "nodip", False))   # round 3 no-dip rule (trajgen.nodip)
        self._elec_geo = {}
        if self.electrode_on:
            self._wp_c, self._wp_h = self.box_c[self.box_wp], self.box_h[self.box_wp]
            self._wp_R = self.box_R[self.box_wp]
            self._wp_idx = np.flatnonzero(self.box_wp)
            for rid in self.rids:
                self.electrode_geometry(rid)                            # fails early without an electrode hull
        # ---- umarell parity (only when the scene declares an umarell cell and every robot has umarell's tool grid)
        self.umarell = None
        if scene.umarell_cell and all(rm.um_tool is not None for rm in self.rm.values()):
            try:
                from env.cell import load_cell
                c = load_cell(scene.umarell_cell)
                self.umarell = _Parity(scene.umarell_cell, c.collision.get("warning_distance", 0.20),
                                       c.collision.get("collision_radius_m", 0.12))
            except Exception:  # noqa: BLE001
                self.umarell = None

    @property
    def parity_info(self) -> dict:
        if self.umarell is None:
            return {"available": False, "reason": "scene declares no umarell cell" if not self.scene.umarell_cell
                    else f"umarell cell {self.scene.umarell_cell} has no tool grid for every robot"}
        return {"available": True, "cell_id": self.umarell.cell_id, "gate_m": self.umarell.warning_distance,
                "pair_radius_m": self.umarell.pair_radius,
                "geometry": "the scene's own robots (URDF, bases, homes) with umarell's tool_collision grid"}

    # ------------------------------------------------------------------ kinematics
    def tcp(self, rid, Q):
        return self.rm[rid].tcp(np.asarray(Q, float))

    def spheres(self, rid, Q):
        return self.rm[rid].spheres(np.asarray(Q, float))

    def pair_spheres(self, rid, Q):
        """Distinct sphere centres for the arm-arm gate (all with pair_radius)."""
        return self.rm[rid].spheres(np.asarray(Q, float))[..., self.rm[rid].pair_idx, :]

    def umarell_spheres(self, rid, Q):
        rm = self.rm[rid]
        return rm.umarell_spheres(np.asarray(Q, float))[..., rm.um_pair_idx, :]

    # ------------------------------------------------------------------ boxes
    def _box_dist(self, P):
        """(n, B) distance from points to the box surfaces (0 inside)."""
        rel = P[:, None, :] - self.box_c[None]
        if self.box_rot:
            rel = np.einsum("nbi,bij->nbj", rel, self.box_R)
        return np.sqrt((np.maximum(np.abs(rel) - self.box_h, 0.0) ** 2).sum(-1))

    def _box_exact(self, P, r, ex, chunk: int = 2048):
        """Exact min over boxes of dist(P, box) - r - margin for points P (n, 3)."""
        out = np.empty(len(P))
        for k in range(0, len(P), chunk):
            p = P[k:k + chunk]
            g = self._box_dist(p) - r[k:k + chunk, None] - self.box_m[None]
            if self.box_wp.any():
                g[np.ix_(ex[k:k + chunk], self.box_wp)] = np.inf
            out[k:k + chunk] = g.min(1)
        return out

    def _box_field(self):
        """Distance from grid points to the union of the boxes (2.5 cm grid over the boxes' bounding box + pad),
        cached in data/trajgen/cache. The cache key covers the boxes (incl. rotations), the pad, the step and the grid
        origin; the stored origin / step / shape are checked on load (round-1 issue: a field built for another
        margin was indexed with the wrong origin)."""
        if self._field is not None:
            return self._field
        step = 0.025
        rmax = max(float(r.max()) for r in self.box_r.values())
        pad = float(rmax + self.box_m.max() + 0.15)
        ext = np.abs(self.box_R) @ self.box_h[..., None] if self.box_rot else self.box_h[..., None]
        ext = ext[..., 0]
        lo = np.round((self.box_c - ext).min(0) - pad, 6)
        hi = np.round((self.box_c + ext).max(0) + pad, 6)
        axes = [np.arange(lo[i], hi[i] + step, step) for i in range(3)]
        shape = tuple(len(a) for a in axes)
        h = hashlib.sha256()
        for a in (self.box_c, self.box_h, self.box_R, lo, np.array([step, pad, BOXFIELD_VERSION], float)):
            h.update(np.ascontiguousarray(a, float).tobytes())
        from trajgen.cachedir import CACHE_DIR
        cache = CACHE_DIR / f"boxfield_{h.hexdigest()[:16]}.npz"
        F = None
        if self.box_field_cache and cache.exists():
            try:
                z = np.load(cache)
                if z["F"].shape == shape and np.allclose(z["lo"], lo) and float(z["step"]) == step:
                    F = z["F"]
            except Exception:  # noqa: BLE001  (truncated / concurrent file: rebuild)
                F = None
        if F is None:
            G = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
            F = np.empty(len(G))
            for k in range(0, len(G), 4096):
                F[k:k + 4096] = self._box_dist(G[k:k + 4096]).min(1)
            F = F.reshape(shape).astype(np.float32)
            if self.box_field_cache:
                import os
                cache.parent.mkdir(parents=True, exist_ok=True)
                tmp = cache.with_name(f"{cache.stem}.{os.getpid()}.tmp.npz")   # atomic (concurrent processes)
                np.savez_compressed(tmp, F=F, lo=lo, step=step, pad=pad)
                os.replace(tmp, cache)
        self._field = (F, lo, step, pad)
        return self._field

    def box_margin(self, rid, Q, S=None, brute: bool = False) -> np.ndarray:
        """(N,) min over checked spheres and boxes of dist(centre, box) - r - margin; +inf without boxes.

        A precomputed distance field (2.5 cm grid, nearest grid point, error <= half a cell diagonal) screens the
        spheres; every sphere whose screened margin is below 5 cm gets the exact distance to all boxes, so the
        result is exact wherever it is < 5 cm (in particular its sign); larger values are lower bounds accurate to
        ~2 cm. ``brute=True``: exact for every sphere (reference for the tests)."""
        Q = np.asarray(Q, float).reshape(-1, 6)
        if not len(self.box_c):
            return np.full(len(Q), np.inf)
        S = self.spheres(rid, Q) if S is None else S
        idx = self.box_idx[rid]
        M = len(idx)
        P = S[:, idx].reshape(-1, 3)
        r = np.tile(self.box_r[rid], len(Q))
        ex = np.tile(self.wp_exempt[rid], len(Q))
        if brute:
            return self._box_exact(P, r, ex).reshape(len(Q), M).min(1)
        F, lo, step, pad = self._box_field()
        ii = np.rint((P - lo) / step).astype(int)
        inside = np.all((ii >= 0) & (ii < np.array(F.shape)), axis=1)
        f = np.full(len(P), pad)                                     # outside the grid: farther than pad
        jj = ii[inside]
        f[inside] = F[jj[:, 0], jj[:, 1], jj[:, 2]]
        err = 0.5 * step * np.sqrt(3.0)
        g = f - err - r - self.box_m.max()
        near = g < 0.05
        if near.any():
            g[near] = self._box_exact(P[near], r[near], ex[near])
        return g.reshape(len(Q), M).min(1)

    def box_hull_margin(self, rid, Q) -> np.ndarray:
        """(N,) MuJoCo hull-vs-box signed distance - box_hull.min_distance (+inf without boxes / rule off).
        Values >= 0 pass; positive values are capped at the detection band."""
        Q = np.asarray(Q, float).reshape(-1, 6)
        if not self.box_hull_on:
            return np.full(len(Q), np.inf)
        return self.mjw.box_distance(rid, Q) - self.box_hull_min

    # ------------------------------------------------------------------ electrode off-weld rule
    def electrode_geometry(self, rid):
        """(frames (H,), centres (H, 3) in those joint frames, radii (H,)) of bounding spheres of the robot's
        electrode hull(s) (``trajgen.mjscene.electrode_hull_vertices``; centre = vertex bounding-box middle)."""
        if rid not in self._elec_geo:
            from trajgen.mjscene import electrode_hull_vertices
            rm = self.rm[rid]
            K, C, Rr = [], [], []
            for link, name, V in electrode_hull_vertices(self.robots[rid]):
                if link not in rm.chain["fixed_links"]:
                    raise ValueError(f"{rid}: electrode hull {name} is on link {link}, not on the arm chain")
                k, Tl = rm.chain["fixed_links"][link]
                W = V @ Tl[:3, :3].T + Tl[:3, 3]
                c = 0.5 * (W.min(0) + W.max(0))
                K.append(int(k))
                C.append(c)
                Rr.append(float(np.linalg.norm(W - c, axis=1).max()) + 1e-6)
            if not K:
                raise ValueError(f"{rid}: electrode off-weld rule needs the electrode hull(s) (tool.electrode_hulls or "
                                 "the tip-link hull containing the TCP); none found")
            self._elec_geo[rid] = (np.array(K, int), np.array(C, float), np.array(Rr, float))
        return self._elec_geo[rid]

    def electrode_centres(self, rid, Q) -> np.ndarray:
        """(N, H, 3) world centres of the electrode bounding spheres (radii: electrode_geometry(rid)[2])."""
        K, C, _ = self.electrode_geometry(rid)
        Rs, ps = self.rm[rid].joint_poses(np.asarray(Q, float).reshape(-1, 6))
        return np.einsum("nhij,hj->nhi", Rs[:, K], C) + ps[:, K]

    def weld_distance(self, X) -> np.ndarray:
        """(N,) distance from points X to the nearest weld of the task (+inf without welds)."""
        X = np.asarray(X, float).reshape(-1, 3)
        if not len(self.W_P):
            return np.full(len(X), np.inf)
        return np.linalg.norm(X[:, None] - self.W_P[None], axis=-1).min(1)

    def electrode_distance(self, rid, Q, exact_below: float | None = None):
        """(d (N,), box (N,)): signed distance electrode hull(s) - nearest workpiece box and that box's index into
        scene.boxes (-1 when no workpiece box is within the band). Exact (MuJoCo, capped at ``mjw.elec_band``) where
        the bounding-sphere lower bound is below ``exact_below`` (default: the band); elsewhere that lower bound
        (>= exact_below), box -1."""
        Q = np.asarray(Q, float).reshape(-1, 6)
        N = len(Q)
        if not self.electrode_on:
            return np.full(N, np.inf), np.full(N, -1, int)
        band = self.mjw.elec_band
        thr = band if exact_below is None else min(float(exact_below), band)
        X = self.electrode_centres(rid, Q)                                  # (N, H, 3)
        r = self.electrode_geometry(rid)[2]
        rel = X[:, :, None, :] - self._wp_c[None, None]                     # (N, H, B, 3)
        if self.box_rot:
            rel = np.einsum("nhbi,bij->nhbj", rel, self._wp_R)
        dc = np.sqrt((np.maximum(np.abs(rel) - self._wp_h, 0.0) ** 2).sum(-1))   # centre to box (0 inside)
        lb = (dc - r[None, :, None]).min(axis=(1, 2))                       # (N,) lower bound where > 0
        d = lb.copy()
        box = np.full(N, -1, int)
        need = np.flatnonzero(lb < max(thr, 0.0) + 1e-9)
        if len(need):
            de, bx = self.mjw.electrode_distance(rid, Q[need])
            d[need] = np.minimum(de, band)
            box[need] = bx
        return d, box

    def electrode_margin(self, rid, Q, exact_below: float = 0.0, return_box: bool = False, welds=None,
                         min_distance: float | None = None):
        """(N,) electrode off-weld margin: electrode - workpiece signed distance - ``min_distance`` where the TCP is
        farther than ``radius`` from every weld (< 0 violates), +inf where it is within (or the rule is off). Exact
        wherever it is below ``exact_below`` (lower bounds above). ``return_box``: also the box index (-1 none).

        No-dip rule (round 3, ``self.nodip``, trajgen.nodip): the margin is checked everywhere; only contacts inside
        the contact exemption of a weld in ``welds`` (weld ids or indices into the task's welds; None = every weld of
        the task, () = none, e.g. transits) pass, per contact: max(distance - min_distance, min(distance +
        contact_depth, contact_point_radius - |contact point - weld|, radius - |TCP - weld|)) >= 0 (``nodip_margin``).
        ``min_distance`` overrides the rule's (the verifier passes 0: any penetration fails)."""
        Q = np.asarray(Q, float).reshape(-1, 6)
        out = np.full(len(Q), np.inf)
        box = np.full(len(Q), -1, int)
        if self.electrode_on and len(Q):
            er = self.electrode_rule
            md = er.min_distance if min_distance is None else float(min_distance)
            if self.nodip:
                return self.nodip_margin(rid, Q, welds=welds, min_distance=md, exact_below=exact_below,
                                         return_box=return_box)
            off = np.flatnonzero(self.weld_distance(self.tcp(rid, Q)[0]) > er.radius)
            if len(off):
                d, b = self.electrode_distance(rid, Q[off], exact_below=md + exact_below)
                out[off] = d - md
                box[off] = b
        return (out, box) if return_box else out

    def weld_indices(self, welds=None) -> np.ndarray:
        """Indices into the task's welds (``W_P``) of ``welds``: None = all, else weld ids or integer indices."""
        if welds is None:
            return np.arange(len(self.W_P))
        out = []
        for w in welds:
            if isinstance(w, (int, np.integer)):
                out.append(int(w))
            else:
                out.append(self.W_ids.index(w))
        return np.array(out, int)

    def nodip_margin(self, rid, Q, welds=None, min_distance: float | None = None, exact_below: float = 0.0,
                     return_box: bool = False):
        """(N,) no-dip electrode margin (trajgen.nodip; >= 0 passes): min over the electrode-workpiece contacts of
        max(distance - min_distance, max over the exempting welds w of min(distance + contact_depth,
        contact_point_radius - |contact point - p_w|, radius - |TCP - p_w|)); without a contact within the band the
        bounding-sphere lower bound of the distance - min_distance. Exempting welds: ``welds`` (None = every weld of
        the task, () = none). ``return_box``: also the box index of the binding contact (-1 none)."""
        Q = np.asarray(Q, float).reshape(-1, 6)
        er = self.electrode_rule
        md = er.min_distance if min_distance is None else float(min_distance)
        out = np.full(len(Q), np.inf)
        box = np.full(len(Q), -1, int)
        if not self.electrode_on or not len(Q):
            return (out, box) if return_box else out
        d, b = self.electrode_distance(rid, Q, exact_below=md + exact_below)
        out = d - md
        box = b.copy()
        Wi = self.weld_indices(welds)
        neg = np.flatnonzero(out < 0)
        if len(Wi) and len(self.W_P) and len(neg):
            P = self.tcp(rid, Q[neg])[0]                                     # only where the exemption matters
            Wp = self.W_P[Wi]
            dist = np.full((len(Q), len(Wi)), np.inf)
            dist[neg] = np.linalg.norm(P[:, None] - Wp[None], axis=-1)       # (N, W)
            cand = np.flatnonzero((dist <= er.radius).any(1) & (out < 0))
            for k in cand:
                near = np.flatnonzero(dist[k] <= er.radius)
                dc, pc, bc = self.mjw.electrode_contacts(rid, Q[k])
                if not len(dc):
                    continue
                gw = np.minimum(np.minimum(dc[:, None] + er.contact_depth,
                                           er.contact_point_radius - np.linalg.norm(pc[:, None] - Wp[near][None], axis=-1)),
                                er.radius - dist[k, near][None])               # (C, near)
                g = np.maximum(dc - md, gw.max(1))                               # (C,)
                j = int(np.argmin(g))
                out[k] = min(float(g[j]), float(self.mjw.elec_band - md))
                box[k] = int(bc[j])
        return (out, box) if return_box else out

    def electrode_ok(self, rid, Q, welds=None) -> np.ndarray:
        """(N,) bool: the electrode off-weld rule holds (True everywhere when the rule is off). ``welds``: the
        no-dip rule's exempting welds (None = every weld, () = none)."""
        return self.electrode_margin(rid, Q, welds=welds) >= 0.0

    # ------------------------------------------------------------------ rule W
    def wedge_depth(self, X) -> np.ndarray:
        """(N,) max wedge depth min((x-p).a, (x-p).t) over welds whose rule W is on and |x-p| < radius (-inf if
        none)."""
        X = np.asarray(X, float).reshape(-1, 3)
        if not self.wedge_on or not len(self.W_P):
            return np.full(len(X), -np.inf)
        rel = X[:, None] - self.W_P[None]
        dist = np.linalg.norm(rel, axis=-1)
        dep = np.minimum((rel * self.W_A).sum(-1), (rel * self.W_T).sum(-1))
        return np.where(self.W_on & (dist < self.W_rad), dep, -np.inf).max(1)

    def wedge_excess(self, X) -> np.ndarray:
        """(N,) max over welds of (wedge depth - the weld's tol); > 0 violates rule W."""
        X = np.asarray(X, float).reshape(-1, 3)
        if not self.wedge_on or not len(self.W_P):
            return np.full(len(X), -np.inf)
        rel = X[:, None] - self.W_P[None]
        dist = np.linalg.norm(rel, axis=-1)
        dep = np.minimum((rel * self.W_A).sum(-1), (rel * self.W_T).sum(-1)) - self.W_tol
        return np.where(self.W_on & (dist < self.W_rad), dep, -np.inf).max(1)

    # ------------------------------------------------------------------ single-arm rules (vectorised)
    def static_margins(self, rid, Q, hull_box: bool = False, electrode: bool = False) -> dict:
        """Vectorised single-arm margins (< 0 violates; wedge_excess > 0 violates). ``electrode``: also the electrode
        off-weld margin (key "electrode", only when the scene's rule is on)."""
        Q = np.asarray(Q, float).reshape(-1, 6)
        S = self.spheres(rid, Q)
        r = self.robots[rid]
        p, _ = self.tcp(rid, Q)
        fl = self.scene.floor
        out = {"joint": np.minimum((Q - r.q_lo).min(1), (r.q_hi - Q).min(1)),
               "floor_sphere": self.geo.floor_sphere_margin(rid, Q) if fl.sphere_rule else np.full(len(Q), np.inf),
               "floor_hull": self.geo.floor_hull_min_z(rid, Q) if fl.hull_rule else np.full(len(Q), np.inf),
               "box": self.box_margin(rid, Q, S),
               "wedge_depth": self.wedge_depth(p), "wedge_excess": self.wedge_excess(p)}
        if hull_box:
            out["box_hull"] = self.box_hull_margin(rid, Q)
        if electrode and self.electrode_on:
            out["electrode"] = self.electrode_margin(rid, Q)
        return out

    def _hull_box_ok(self, rid, Q, box):
        """Hull-vs-box rule for the rows whose sphere box margin is small enough for a hull to reach a box."""
        ok = np.ones(len(Q), bool)
        if not self.box_hull_on:
            return ok
        near = np.flatnonzero(box < 0.30)
        if len(near):
            ok[near] = self.box_hull_margin(rid, Q[near]) >= 0
        return ok

    def static_ok_batch(self, rid, Q, self_check: bool = True, welds=None) -> np.ndarray:
        """(N,) bool: every single-arm rule; the coal self-distance only for rows passing the rest. ``welds``: the
        no-dip rule's exempting welds (None = every weld of the task, () = none: transits)."""
        Q = np.asarray(Q, float).reshape(-1, 6)
        m = self.static_margins(rid, Q)
        ok = (m["joint"] >= -1e-9) & (m["floor_sphere"] >= 0) & (m["floor_hull"] >= 0) & (m["box"] >= 0) \
            & (m["wedge_excess"] <= 0)
        if ok.any() and self.box_hull_on:
            k = np.flatnonzero(ok)
            ok[k] = self._hull_box_ok(rid, Q[k], m["box"][k])
        if ok.any() and self.electrode_on:
            k = np.flatnonzero(ok)
            ok[k] = self.electrode_ok(rid, Q[k], welds=welds)
        if self_check:
            for i in np.flatnonzero(ok):
                if self.ik.self_distance(rid, Q[i]) < self.self_margin:
                    ok[i] = False
        return ok

    def static_ok(self, rid, q) -> bool:
        return bool(self.static_ok_batch(rid, np.asarray(q, float)[None])[0])

    def static_reasons(self, rid, q) -> list:
        """Names of the single-arm rules a configuration violates (for diagnostics)."""
        q = np.asarray(q, float)[None]
        m = self.static_margins(rid, q, hull_box=self.box_hull_on, electrode=self.electrode_on)
        out = [k for k in ("joint", "floor_sphere", "floor_hull", "box", "box_hull") if k in m and m[k][0] < -1e-9]
        if "electrode" in m and m["electrode"][0] < 0:
            out.append("electrode_nodip" if self.nodip else "electrode_off_weld")
        if m["wedge_excess"][0] > 0:
            out.append("rule_W")
        if self.ik.self_distance(rid, q[0]) < self.self_margin:
            out.append("self_collision")
        return out

    def extra_check(self, rid):
        """Callable for ``WeldIK.solve_axis(extra_check=...)``: the scene rules SceneIK.check does not cover."""
        def f(q):
            Q = np.asarray(q, float)[None]
            m = self.static_margins(rid, Q)
            ok = bool(m["floor_sphere"][0] >= 0 and m["box"][0] >= 0 and m["wedge_excess"][0] <= 0)
            if ok and self.box_hull_on:
                ok = bool(self._hull_box_ok(rid, Q, m["box"])[0])
            if ok and self.electrode_on:
                ok = bool(self.electrode_ok(rid, Q)[0])
            return ok
        return f

    # Cartesian resolution of the segment checks (round-2 finding: at 0.02 rad near full reach the TCP moves ~34 mm
    # between check points, more than the 20 mm rule-W wedge radius)
    seg_dx = 0.01           # max displacement of any sphere centre (TCP, link and tool spheres) between check points
    seg_dx_w = 0.002        # max TCP displacement between check points within reach of a rule-W wedge
    seg_w_margin = 0.001    # rule W in transits: wedge excess <= -margin at the check points (1-Lipschitz depth:
                            # covers the 2 mm spacing, so the continuous segment stays within tol)

    def segment_points(self, rid, q0, q1, res: float = 0.02):
        """(Q_joint, Q_dense): the joint-resolution points of q0 -> q1 (excluding q0, max joint step ``res``) and
        the same segment subdivided until no sphere centre moves more than ``seg_dx`` and, near a weld whose rule W
        is on, the TCP not more than ``seg_dx_w`` between consecutive points (excluding q0)."""
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        n = max(1, int(np.ceil(np.abs(q1 - q0).max() / res)))
        Q = q0 + (q1 - q0) * (np.arange(0, n + 1) / n)[:, None]
        S = self.spheres(rid, Q)
        step = np.linalg.norm(np.diff(S, axis=0), axis=-1).max(1)                   # (n,)
        sub = np.maximum(1, np.ceil(step / self.seg_dx - 1e-9)).astype(int)
        if self.wedge_on:
            p, _ = self.tcp(rid, Q)
            st = np.linalg.norm(np.diff(p, axis=0), axis=-1)
            on = self.W_on
            dw = np.linalg.norm(p[:, None] - self.W_P[None, on], axis=-1) - self.W_rad[None, on]  # (n+1, n_on)
            near = (np.minimum(dw[:-1], dw[1:]) <= st[:, None] + 1e-9).any(1) if on.any() else np.zeros(n, bool)
            sub = np.where(near, np.maximum(sub, np.ceil(st / self.seg_dx_w - 1e-9).astype(int)), sub)
        idx = np.repeat(np.arange(n), sub)
        start = np.cumsum(sub) - sub
        frac = (np.arange(len(idx)) - start[idx] + 1) / sub[idx]
        Qd = Q[idx] + (Q[idx + 1] - Q[idx]) * frac[:, None]
        return Q[1:], Qd

    def segment_ok(self, rid, q0, q1, res: float = 0.02) -> bool:
        """Straight joint segment q0 -> q1 (excluding q0) valid: every single-arm rule on the Cartesian-bounded points
        of ``segment_points`` (rule W with ``seg_w_margin``), hull self-distance at max-joint resolution ``res``."""
        Qj, Qd = self.segment_points(rid, q0, q1, res)
        # transits: no weld-contact exemption under the no-dip rule (trajgen.nodip)
        ok = self.static_ok_batch(rid, Qd, self_check=False, welds=() if self.nodip else None)
        if not ok.all():
            return False
        if self.wedge_on:
            p, _ = self.tcp(rid, Qd)
            if (self.wedge_excess(p) > -self.seg_w_margin).any():
                return False
        return all(self.ik.self_distance(rid, q) >= self.self_margin for q in Qj)

    def self_distance(self, rid, Q) -> np.ndarray:
        Q = np.asarray(Q, float).reshape(-1, 6)
        return np.array([self.ik.self_distance(rid, q) for q in Q])

    # ------------------------------------------------------------------ arm vs arm
    def pair_clearance(self, ra, rb, Qa=None, Qb=None) -> np.ndarray:
        """Corrected-sphere clearance of robots ra, rb. (Legacy two-robot form ``pair_clearance(Q1, Q2)`` too.)"""
        if Qa is None and Qb is None:            # pair_clearance(Q1, Q2) for the first two robots
            return self.geo.pair_clearance_r(self.rids[0], self.rids[1], ra, rb)
        return self.geo.pair_clearance_r(ra, rb, Qa, Qb)

    @staticmethod
    def pair_clearance_spheres(S1, S2, pair_radius: float) -> np.ndarray:
        """(N,) min centre distance - 2 r from precomputed sphere arrays (N, n, 3) / (N, m, 3) (Gram trick)."""
        c = 0.5 * (S1.mean(1, keepdims=True) + S2.mean(1, keepdims=True))
        A, B = S1 - c, S2 - c
        d2 = (A * A).sum(-1)[:, :, None] + (B * B).sum(-1)[:, None, :] - 2.0 * np.einsum("nik,njk->nij", A, B)
        return np.sqrt(np.maximum(d2.reshape(len(d2), -1).min(1), 0.0)) - 2.0 * pair_radius

    def umarell_pair_clearance(self, ra, rb, Qa, Qb, chunk: int = 512) -> np.ndarray:
        Qa, Qb = np.asarray(Qa, float).reshape(-1, 6), np.asarray(Qb, float).reshape(-1, 6)
        out = np.empty(len(Qa))
        for s in range(0, len(Qa), chunk):
            out[s:s + chunk] = self.pair_clearance_spheres(self.umarell_spheres(ra, Qa[s:s + chunk]),
                                                           self.umarell_spheres(rb, Qb[s:s + chunk]),
                                                           self.umarell.pair_radius)
        return out

    def hull_extent(self, rid) -> float:
        """Upper bound of the distance from any collision-hull vertex to the nearest pair-check sphere centre of the
        same robot (exact up to sampling: vertices are rigid in their joint frames, evaluated at 8 configurations,
        +1 cm). Two arms' hulls can touch only if their sphere clearance < extent_a + extent_b - 2 pair_radius."""
        if not hasattr(self, "_hext"):
            self._hext = {}
        if rid not in self._hext:
            rm = self.rm[rid]
            Q = np.random.default_rng(1).uniform(rm.q_lo, rm.q_hi, (8, 6))
            Rs, ps = rm.joint_poses(Q)
            S = self.pair_spheres(rid, Q)
            best = 0.0
            for k, V in rm.hulls.items():
                W = np.einsum("qij,nj->qni", Rs[:, k], V) + ps[:, k, None, :]
                d = np.linalg.norm(W[:, :, None] - S[:, None], axis=-1).min(-1)
                best = max(best, float(d.max()))
            self._hext[rid] = best + 0.01
        return self._hext[rid]

    def sweep_radii(self, rid, kind: str = "hull") -> np.ndarray:
        """(6,) R_j: an upper bound of the distance from joint j's axis to any point that joint j moves (``hull``:
        every collision-hull vertex of the frames after joint j, by the chain bound sum of the rigid joint-to-joint
        offsets + the vertex radius in its frame; ``spheres``: every pair-check sphere centre, sampled over 256
        configurations + 2 cm). A joint-space step dq moves any such point by at most sum_j |dq_j| R_j (round-4
        finding: inter-sample bound of the 100 Hz hull checks)."""
        if not hasattr(self, "_swr"):
            self._swr = {}
        key = (rid, kind)
        if key not in self._swr:
            rm = self.rm[rid]
            off = np.linalg.norm(rm._To[:, :3, 3], axis=1)             # |p_{i+1} - p_i| in frame i, rigid
            Rj = np.zeros(6)
            if kind == "hull":
                vr = {k: float(np.linalg.norm(V, axis=1).max()) for k, V in rm.hulls.items()}
                for j in range(6):                 # joint j rotates frames k >= j + 1 about the origin of frame j + 1
                    best = 0.0
                    for k, r in vr.items():
                        if k >= j + 1:
                            best = max(best, float(off[j + 1:k].sum()) + r)
                    Rj[j] = best
            else:
                Q = np.random.default_rng(2).uniform(rm.q_lo, rm.q_hi, (256, 6))
                _, ps = rm.joint_poses(Q)
                S = self.pair_spheres(rid, Q)
                for j in range(6):
                    Rj[j] = float(np.linalg.norm(S - ps[:, j + 1, None, :], axis=-1).max()) + 0.02
            self._swr[key] = Rj
        return self._swr[key]

    def step_margin(self, rid, Q, kind: str = "hull") -> np.ndarray:
        """(N,) per sample of the piecewise-linear joint path Q: the largest point displacement (``sweep_radii``
        bound) over its two adjacent intervals. The continuous distance between two such paths within half an
        interval of a sample pair is >= the sampled distance - (e_a + e_b) / 2."""
        Q = np.asarray(Q, float).reshape(-1, 6)
        if len(Q) < 2:
            return np.zeros(len(Q))
        D = np.abs(np.diff(Q, axis=0)) @ self.sweep_radii(rid, kind)
        e = np.empty(len(Q))
        e[0], e[-1] = D[0], D[-1]
        e[1:-1] = np.maximum(D[:-1], D[1:])
        return e

    def hull_cover(self, rid):
        """Spheres that together contain every collision hull of the robot (the MuJoCo pair geoms): per hull, its
        PCA-oriented bounding box of the vertices, cut along the long axis into near-cubic pieces, each piece's
        circumscribed sphere. Returns (frame index (H,), centre in that frame (H, 3), radius (H,))."""
        if not hasattr(self, "_hcov"):
            self._hcov = {}
        if rid not in self._hcov:
            from trajgen.robot import collision_hulls
            rm = self.rm[rid]
            K, C, Rr = [], [], []
            for ln, Vs in collision_hulls(str(rm.urdf)).items():
                if ln not in rm.chain["fixed_links"]:
                    continue
                k, Tl = rm.chain["fixed_links"][ln]
                for V in Vs:
                    W = np.asarray(V, float) @ Tl[:3, :3].T + Tl[:3, 3]
                    c0 = W.mean(0)
                    _, _, Vt = np.linalg.svd(W - c0, full_matrices=False)
                    P = (W - c0) @ Vt.T
                    lo, hi = P.min(0), P.max(0)
                    ext = hi - lo
                    n = int(max(1, np.ceil(ext[0] / max(ext[1], 0.12))))
                    for i in range(n):
                        a0, a1 = lo[0] + ext[0] * i / n, lo[0] + ext[0] * (i + 1) / n
                        mid = np.array([0.5 * (a0 + a1), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])])
                        K.append(k)
                        C.append(c0 + Vt.T @ mid)
                        Rr.append(0.5 * np.linalg.norm([a1 - a0, ext[1], ext[2]]) + 1e-4)
            self._hcov[rid] = (np.array(K, int), np.array(C, float).reshape(-1, 3), np.array(Rr, float))
        return self._hcov[rid]

    def hull_groups(self, rid):
        """Per joint frame one sphere containing every hull cover sphere rigid in that frame: (frames (F,), centre in
        the frame (F, 3), radius (F,))."""
        if not hasattr(self, "_hgrp"):
            self._hgrp = {}
        if rid not in self._hgrp:
            K, C, Rr = self.hull_cover(rid)
            fr, cs, rs = [], [], []
            for k in np.unique(K):
                Ck, Rk = C[K == k], Rr[K == k]
                c0 = 0.5 * ((Ck - Rk[:, None]).min(0) + (Ck + Rk[:, None]).max(0))
                fr.append(int(k))
                cs.append(c0)
                rs.append(float((np.linalg.norm(Ck - c0, axis=1) + Rk).max()))
            self._hgrp[rid] = (np.array(fr, int), np.array(cs, float), np.array(rs, float))
        return self._hgrp[rid]

    def hull_group_centres(self, rid, Q) -> np.ndarray:
        """(N, F, 3) world centres of ``hull_groups`` at configurations Q (radii: hull_groups(rid)[2])."""
        F, C, _ = self.hull_groups(rid)
        Rs, ps = self.rm[rid].joint_poses(np.asarray(Q, float).reshape(-1, 6))
        return np.einsum("nfij,fj->nfi", Rs[:, F], C) + ps[:, F]

    def hull_lower_bound(self, ra, rb, Qa, Qb, chunk: int = 512) -> np.ndarray:
        """(N,) lower bound of the robot-robot hull distance (cover spheres of ``hull_cover``): MuJoCo is needed only
        where it is below the threshold of interest."""
        Qa, Qb = np.asarray(Qa, float).reshape(-1, 6), np.asarray(Qb, float).reshape(-1, 6)
        out = np.empty(len(Qa))
        cov = []
        for r in (ra, rb):
            K, C, Rr = self.hull_cover(r)
            cov.append((K, C, Rr))
        for s0 in range(0, len(Qa), chunk):
            X = []
            for (K, C, Rr), r, Q in zip(cov, (ra, rb), (Qa[s0:s0 + chunk], Qb[s0:s0 + chunk])):
                Rs, ps = self.rm[r].joint_poses(Q)
                X.append(np.einsum("nhij,hj->nhi", Rs[:, K], C) + ps[:, K])
            A, B = X
            d = np.linalg.norm(A[:, :, None] - B[:, None], axis=-1) - cov[0][2][None, :, None] - cov[1][2][None, None]
            out[s0:s0 + chunk] = d.reshape(len(A), -1).min(1)
        return out

    def mj_pair_contact(self, ra, rb, Qa, Qb, thr: float = 0.0) -> np.ndarray:
        """(N,) bool: robot-robot hull distance < thr (0: penetration; ``hull_min``: the scene's hull buffer)."""
        return self.mjw.pair_contact(ra, rb, Qa, Qb, thr)

    def mj_pair_min_distance(self, ra, rb, Qa, Qb) -> np.ndarray:
        """(N,) robot-robot signed hull distance, capped at ``mjw.pair_band`` (hull_min + 5 cm). (MuJoCo's own
        bounding-sphere broadphase makes far configurations cheap, ~15 us per configuration.)"""
        return self.mjw.pair_min_distance(ra, rb, Qa, Qb)

    @property
    def hull_min(self) -> float:
        """collision.arm_arm.hull_min_distance_m when the hull backstop is on, else 0."""
        c = self.scene.collision
        return float(c.hull_min_distance) if c.hull_backstop else 0.0

    def mj_pair_distance(self, ra, rb, qa, qb, distmax: float = 0.5) -> float:
        return self.mjw.pair_distance(ra, rb, qa, qb, distmax)

    def mj_single_contacts(self, rid, q) -> dict:
        return self.mjw.single_contacts(rid, q)
