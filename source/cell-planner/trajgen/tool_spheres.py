"""Tool collision spheres fitted to the URDF collision hulls, with the <origin> rotations applied.

umarell's ``tool_collision`` block (robots.yaml) was built from hull vertices placed with the
``<origin xyz>`` only; the ``rpy`` was dropped (docs/selfcollision_verification.md §B). Placed
correctly, its 35 spheres cover 72.9 % of the hull vertices and 21 of them sit in empty space.

This module rebuilds the set from the URDF itself:

* hulls are the ``<collision>`` meshes of the mount link (``link_6``): the flange hull and the five
  tool hulls ``World_hull_0..4``, each transformed by its full ``<origin xyz rpy>``
  (URDF convention R = Rz(y) Ry(p) Rx(r), the same as ``env.kinematics.rpy_matrix``);
* two radius classes: the slim stick (electrode ``World_hull_0`` + bar ``World_hull_4``, radial
  <= 0.051 m) gets ``r_slim`` spheres, the bulky parts (flange hull, adapter discs
  ``World_hull_1/2``, RAH plate ``World_hull_3``) get ``r_bulk`` spheres;
* greedy set cover: candidate centres are points inside the hulls (so every centre is in metal),
  targets are all hull vertices plus even surface samples; spheres are added until every target of
  the group is within its group radius.

Run ``.venv/bin/python -m trajgen.tool_spheres`` to print the set and write
``results/trajgen_tool_spheres.json`` (deterministic, seed 0, about 20 s, one process).
"""
from __future__ import annotations

import json
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]

SLIM_HULLS = ("World_hull_0", "World_hull_4")                       # electrode + 80 mm bar
BULK_HULLS = ("link_6_hull_0", "World_hull_1", "World_hull_2", "World_hull_3")  # flange, discs, plate


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF rpy -> R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


@dataclass
class Hull:
    name: str              # mesh file stem, e.g. "World_hull_3"
    mesh: object           # trimesh.Trimesh in the link frame (origin xyz + rpy applied)
    vertices: np.ndarray   # (n, 3) link-frame vertices

    def __post_init__(self):
        from scipy.spatial import ConvexHull
        self._eq = ConvexHull(self.vertices).equations       # (F, 4): n.x + d <= 0 inside

    def contains(self, P) -> np.ndarray:
        """Inside the convex hull of the vertices (all URDF collision meshes here are convex)."""
        P = np.asarray(P, float).reshape(-1, 3)
        return (P @ self._eq[:, :3].T + self._eq[:, 3] <= 1e-9).all(1)

    @property
    def lo(self):
        return self.vertices.min(0)

    @property
    def hi(self):
        return self.vertices.max(0)


def link_hulls(urdf: str | Path, link_suffix: str = "link_6", apply_rpy: bool = True) -> list[Hull]:
    """The ``<collision>`` meshes of the URDF link whose name ends with ``link_suffix``, placed in
    that link's frame. ``apply_rpy=False`` reproduces umarell's mistake (xyz only)."""
    import trimesh

    urdf = Path(urdf)
    root = ET.parse(urdf).getroot()
    out = []
    for link in root.findall("link"):
        if not link.get("name", "").endswith(link_suffix):
            continue
        for c in link.findall("collision"):
            o = c.find("origin")
            xyz = np.array((o.get("xyz") if o is not None else "0 0 0").split(), float)
            rpy = np.array((o.get("rpy") if o is not None else "0 0 0").split(), float)
            fn = c.find("geometry/mesh").get("filename")
            m = trimesh.load(os.path.join(urdf.parent, fn), force="mesh", process=False)
            T = np.eye(4)
            T[:3, :3] = rpy_matrix(*rpy) if apply_rpy else np.eye(3)
            T[:3, 3] = xyz
            m.apply_transform(T)
            name = Path(fn).stem.replace("like_kr240r2900_", "")
            out.append(Hull(name, m, np.asarray(m.vertices, float)))
    if not out:
        raise ValueError(f"no collision geoms on a link ending with {link_suffix!r} in {urdf}")
    return out


def _targets(hulls: list[Hull], n_surface: int, rng) -> tuple[np.ndarray, np.ndarray]:
    """(vertices, surface samples) of a hull group."""
    import trimesh

    V = np.concatenate([h.vertices for h in hulls])
    area = np.array([h.mesh.area for h in hulls])
    S = []
    for h, a in zip(hulls, area):
        k = max(50, int(round(n_surface * a / area.sum())))
        pts, _ = trimesh.sample.sample_surface(h.mesh, k, seed=int(rng.integers(1 << 31)))
        S.append(pts)
    return V, np.concatenate(S)


def _interior(hulls: list[Hull], n: int, rng) -> np.ndarray:
    """Points inside the hulls (rejection sampling per hull, plus hull centroids)."""
    pts = []
    for h in hulls:
        lo, hi = h.lo, h.hi
        k = max(200, int(n * h.mesh.volume / sum(g.mesh.volume for g in hulls)))
        got = [h.mesh.centroid[None]]
        tries = 0
        while sum(len(g) for g in got) < k and tries < 50:
            P = rng.uniform(lo, hi, (4 * k, 3))
            got.append(P[h.contains(P)])
            tries += 1
        pts.append(np.concatenate(got)[:k])
    return np.concatenate(pts)


def greedy_cover(targets: np.ndarray, candidates: np.ndarray, radius: float) -> np.ndarray:
    """Greedy set cover: indices of candidates so that every target is within ``radius``."""
    D = np.linalg.norm(targets[:, None, :] - candidates[None, :, :], axis=-1) <= radius   # (T, C)
    if not D.any(1).all():
        raise ValueError("some targets cannot be covered by any candidate")
    left = np.ones(len(targets), bool)
    chosen = []
    while left.any():
        gain = D[left].sum(0)
        c = int(np.argmax(gain))
        chosen.append(c)
        left &= ~D[:, c]
    # prune: drop any sphere whose targets are all covered by the others (latest first)
    for c in list(chosen[::-1]):
        rest = [k for k in chosen if k != c]
        if rest and D[:, rest].any(1).all():
            chosen = rest
    return np.array(chosen, int)


def fit(urdf: str | Path, r_slim: float = 0.07, r_bulk: float = 0.12, n_surface: int = 20000,
        n_interior: int = 4000, seed: int = 0) -> dict:
    """Fit the corrected tool sphere set. Returns {"centers" (n,3), "radii" (n,), "groups" [...]}."""
    rng = np.random.default_rng(seed)
    hulls = {h.name: h for h in link_hulls(urdf)}
    centers, radii, groups = [], [], []
    for gname, names, r in (("slim", SLIM_HULLS, r_slim), ("bulk", BULK_HULLS, r_bulk)):
        hs = [hulls[n] for n in names]
        V, S = _targets(hs, n_surface // 2, rng)
        T = np.concatenate([V, S])
        rf = r - 0.0005                     # fit 0.5 mm tight so 0.1 mm rounding of the centres stays covered
        if gname == "slim":
            # the stick is straight along -x: evenly spaced spheres on its centre line, the
            # smallest count that covers every target (ends exactly at the hull extremes)
            yz = 0.5 * (V[:, 1:].min(0) + V[:, 1:].max(0))
            x0, x1 = V[:, 0].max(), V[:, 0].min()
            for n in range(2, 64):
                cs = np.column_stack([np.linspace(x0, x1, n), np.full(n, yz[0]), np.full(n, yz[1])])
                if coverage(cs, np.full(n, rf), T)["covered_frac"] == 1.0:
                    break
            else:
                raise ValueError("slim group not coverable on its centre line")
        else:
            C = np.concatenate([_interior(hs, n_interior, rng), V])
            cs = C[greedy_cover(T, C, rf)]
        cs = cs[np.lexsort((cs[:, 2], cs[:, 1], -cs[:, 0]))]          # order: flange -> tip (x descending)
        centers.append(np.round(cs, 4)); radii += [r] * len(cs); groups += [gname] * len(cs)
    centers = np.concatenate(centers)
    return {"centers": centers, "radii": np.array(radii, float), "groups": groups}


def coverage(centers, radii, points) -> dict:
    """Fraction of points within (own) radius of at least one sphere, and the worst gap.

    gap(p) = min_k (|p - c_k| - r_k): <= 0 means covered. ``worst_gap_m`` is the largest
    centre distance among points that the covering criterion treats as worst, reported both
    as the signed excess over the radius and as the raw distance to the nearest centre."""
    P = np.asarray(points, float)
    C = np.asarray(centers, float).reshape(-1, 3)
    r = np.asarray(radii, float).reshape(-1)
    d = np.linalg.norm(P[:, None, :] - C[None, :, :], axis=-1)           # (P, K)
    excess = (d - r[None]).min(1)
    k = np.argmin(d - r[None], axis=1)
    return {"n_points": int(len(P)), "covered_frac": float((excess <= 1e-9).mean()),
            "worst_excess_m": float(excess.max()),
            "worst_gap_m": float(d[np.arange(len(P)), k].max()),
            "worst_point": P[int(np.argmax(excess))].round(4).tolist()}


def centers_in_metal(centers, hulls: list[Hull]) -> np.ndarray:
    """Distance of each centre to the nearest hull (0 if inside); outside, the distance to the
    nearest of 2000 surface samples + vertices per hull (an upper bound, a few mm coarse)."""
    import trimesh

    C = np.asarray(centers, float).reshape(-1, 3)
    out = np.full(len(C), np.inf)
    for h in hulls:
        pts = np.concatenate([h.vertices, trimesh.sample.sample_surface(h.mesh, 2000, seed=0)[0]])
        dd = np.linalg.norm(C[:, None] - pts[None], axis=-1).min(1)
        out = np.minimum(out, np.where(h.contains(C), 0.0, dd))
    return out


def report(urdf, centers, radii, n_surface: int = 20000, seed: int = 1, link: str = "link_6") -> dict:
    """Coverage of a sphere set against all hulls of the link whose name ends with ``link`` (default link_6;
    vertices and surface samples), per hull and overall. Uses each sphere's own radius."""
    rng = np.random.default_rng(seed)
    hulls = link_hulls(urdf, link)
    V, S = _targets(hulls, n_surface, rng)
    out = {"vertices": coverage(centers, radii, V), "vertices_and_surface": coverage(centers, radii, np.concatenate([V, S])),
           "per_hull": {}, "n_spheres": int(len(np.asarray(centers).reshape(-1, 3)))}
    for h in hulls:
        out["per_hull"][h.name] = coverage(centers, radii, h.vertices)["covered_frac"]
    dm = centers_in_metal(centers, hulls)
    out["n_centres_outside_metal_gt_radius"] = int((dm > np.asarray(radii)).sum())
    out["max_centre_distance_to_metal_m"] = float(dm.max())
    return out


def main():
    from trajgen.scene import load_original  # local import: scene imports this module

    orig = load_original("welding_cell_2_small")
    urdf = orig.robots[0].urdf
    res = fit(urdf)
    C, r = res["centers"], res["radii"]
    Cr = np.round(C, 4)
    new = report(urdf, Cr, r)
    from env.kinematics import tool_offsets

    old_c = tool_offsets(orig.raw["tool_collision"])
    old = report(urdf, old_c, np.full(len(old_c), 0.12))
    old_xyz_only = coverage(old_c, np.full(len(old_c), 0.12),
                            np.concatenate([h.vertices for h in link_hulls(urdf, apply_rpy=False)
                                            if h.name.startswith("World_hull")]))
    out = {"corrected": {"centers": Cr.tolist(), "radii": r.tolist(), "groups": res["groups"], "coverage": new},
           "umarell_original": {"n": int(len(old_c)), "radius": 0.12, "coverage_urdf_correct": old,
                                "coverage_tool_hulls_xyz_only_rpy_ignored": old_xyz_only}}
    p = ROOT / "results" / "trajgen_tool_spheres.json"
    p.parent.mkdir(exist_ok=True)
    json.dump(out, open(p, "w"), indent=1)
    print(json.dumps({k: v for k, v in new.items()}, indent=1))
    print("old:", json.dumps(old["vertices"]), old["n_centres_outside_metal_gt_radius"])
    for g, c, rr in zip(res["groups"], Cr, r):
        print(f"      - [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]   # {g} r={rr}")
    print("wrote", p)


if __name__ == "__main__":
    main()
