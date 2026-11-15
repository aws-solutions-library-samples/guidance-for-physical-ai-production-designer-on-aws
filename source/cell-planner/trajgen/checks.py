"""Collision geometry of a scene (scenes/SCHEMA.md): link spheres + corrected tool spheres, floor rules,
arm-vs-arm clearance. Also evaluates the ORIGINAL umarell rules for comparison.

    from trajgen.scene import load_scene, load_task
    from trajgen.checks import SceneGeometry
    g = SceneGeometry(load_scene("scenes/welding_cell_2_small"), load_task("scenes/welding_cell_2_small/task.json"))
    g.spheres("robot1", q)            # (..., n, 3) world centres: 42 link spheres (umarell order) + tool spheres
    g.floor_sphere_margin("robot1", q)  # (...)  min(z - r) - (floor.z + margin); < 0 violates
    g.floor_hull_min_z("robot1", q)     # (...)  lowest collision-hull vertex (base_link excluded) - floor.z
    g.pair_clearance(q1, q2)            # (...)  min centre distance - 2 * pair_radius (gate: > arm_arm_gate)

``python -m trajgen.checks`` writes ``results/trajgen_errata_check.json``: old vs corrected rules on the saved
canonical timelines of the project (umarell plans, expert, policies). One process, about 1 min.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from trajgen.scene import Scene, Task

ROOT = Path(__file__).resolve().parents[1]
LINKS = ("link_1", "link_2", "link_3", "link_4", "link_5", "link_6")


class SceneGeometry:
    """Sphere and hull-vertex rules of a scene, for any number of robots (``trajgen.robot.RobotModel`` per robot)."""

    def __init__(self, scene: Scene, task: Task | None = None):
        from trajgen.robot import RobotModel

        self.scene = scene
        self.rids = scene.robot_ids
        um = _umarell_tools(scene)
        self.rm = {r.id: RobotModel(r, umarell_tool=um.get(r.id)) for r in scene.robots}
        self.kin = self.rm
        a = self.rm[self.rids[0]]
        self.n_link = a.n_link                                              # 42 for the KR240
        self.base_frame = a.base_frame
        self.tcp_frame = a.tcp_frame
        self.is_scene = scene.origin == "scene"

    def n_link_of(self, rid):
        return self.rm[rid].n_link

    # ------------------------------------------------------------------ spheres
    def spheres(self, rid: str, q) -> np.ndarray:
        """(..., n_link + n_tool, 3): link spheres + the robot's tool spheres (umarell's grid for load_original)."""
        return self.rm[rid].spheres(np.asarray(q, float))

    def pair_clearance_r(self, ra: str, rb: str, qa, qb, chunk: int = 512) -> np.ndarray:
        """min centre distance over all sphere pairs of robots ra, rb - 2 * pair_radius."""
        qa, qb = np.broadcast_arrays(np.asarray(qa, float), np.asarray(qb, float))
        lead = qa.shape[:-1]
        fa, fb = qa.reshape(-1, 6), qb.reshape(-1, 6)
        ia, ib = self.rm[ra].pair_idx, self.rm[rb].pair_idx
        out = np.empty(len(fa))
        for s in range(0, len(fa), chunk):
            A = self.spheres(ra, fa[s:s + chunk])[:, ia]
            B = self.spheres(rb, fb[s:s + chunk])[:, ib]
            d = np.linalg.norm(A[:, :, None] - B[:, None], axis=-1).min((1, 2))
            out[s:s + chunk] = d - 2 * self.scene.collision.pair_radius
        return out.reshape(lead)

    def pair_clearance(self, q1, q2, chunk: int = 512) -> np.ndarray:
        """Two-robot convenience: clearance between the scene's first two robots."""
        return self.pair_clearance_r(self.rids[0], self.rids[1], q1, q2, chunk)

    # ------------------------------------------------------------------ floor
    def floor_sphere_margin(self, rid: str, q) -> np.ndarray:
        """Corrected sphere rule: min over checked spheres of (z - r) - (floor.z + margin). < 0 = violation.
        Checked: every link-frame sphere after base_link, fill, TCP and (scene origin) every tool sphere."""
        fl = self.scene.floor
        rm = self.rm[rid]
        S = self.spheres(rid, q)
        idx = np.arange(rm.base_frame + 1, rm.n_link)
        if self.is_scene:
            idx = np.concatenate([idx, rm.n_link + np.arange(rm.n_tool)])
        if not fl.enabled:
            return np.full(np.shape(q)[:-1], np.inf)
        return (S[..., idx, 2] - rm.sphere_r[idx]).min(-1) - (fl.z + fl.margin)

    def umarell_floor_margin(self, rid: str, q) -> np.ndarray:
        """umarell's actual predicate: centres of spheres[3:42] (no tool) at z >= -0.05, no radius."""
        S = self.spheres(rid, q)
        return S[..., 3:self.rm[rid].n_link, 2].min(-1) + 0.05

    def floor_hull_min_z(self, rid: str, q) -> np.ndarray:
        """Lowest world z of any collision-hull vertex of links 1..6 (tool hulls are on link_6) - floor.z."""
        fl = self.scene.floor
        z = self.rm[rid].hull_min_z(np.asarray(q, float))
        return z - (fl.z if fl.enabled else -np.inf)


def _umarell_tools(scene: Scene) -> dict:
    """{rid: umarell tool_collision block} when the scene declares a umarell cell (parity model), else {}.
    The robot's umarell id is ``robots[i].umarell_id`` (default: its own id, else the umarell cell's robot in the
    same list position)."""
    if not scene.umarell_cell:
        return {}
    try:
        from env.cell import load_cell
        cell = load_cell(scene.umarell_cell)
    except Exception:  # noqa: BLE001
        return {}
    ids = list(cell.robots)
    out = {}
    for i, r in enumerate(scene.robots):
        uid = r.umarell_id or (r.id if r.id in cell.robots else (ids[i] if i < len(ids) else ids[0]))
        if uid in cell.robots:
            out[r.id] = cell.robots[uid].tool_collision
    return out


TIMELINES = {
    "umarell_bt_1790851970": "data/timelines/umarell_bt_1790851970.npz",
    "umarell_bt_1790848575": "data/timelines/umarell_bt_1790848575.npz",
    "expert_canonical": "data/demos/expert_timeline_canonical_expert_2_small.npz",
    "hier_canonical": "data/timelines/policy_hier_canonical.npz",
    "residual_canonical": "data/timelines/policy_residual_canonical.npz",
    "ppo_v1_canonical": "data/timelines/policy_ppo_canonical.npz",
    "ppo_v3_canonical": "data/timelines/policy_ppo_v3_canonical.npz",
    "e2e_canonical": "data/timelines/policy_e2e_canonical.npz",
}


def link_sphere_coverage(g: SceneGeometry, n_q: int = 64, seed: int = 0) -> dict:
    """How well umarell's link spheres (r = link_radius, frames + fill, unchanged in the corrected scene)
    cover the arm's own link hulls (links 1..5), over random configurations. Informational."""
    from trajgen.tool_spheres import link_hulls

    rid = g.rids[0]
    kin = g.kin[rid]
    rng = np.random.default_rng(seed)
    Q = rng.uniform(kin.q_lo, kin.q_hi, (n_q, 6))
    S = g.spheres(rid, Q)[:, 3:g.n_link]                          # (n_q, 39, 3)
    F = kin.link_transforms(Q)                                    # base_link, link_1 .. link_6
    out = {}
    for k, ln in enumerate(LINKS[:5], start=1):
        V = np.concatenate([h.vertices for h in link_hulls(g.scene.robots[0].urdf, ln)])
        W = np.einsum("qij,nj->qni", F[:, k, :3, :3], V) + F[:, k, None, :3, 3]
        d = np.linalg.norm(W[:, :, None] - S[:, None], axis=-1).min(-1)          # (n_q, n_v)
        out[ln] = {"covered_frac": float((d <= g.scene.collision.link_radius).mean()), "worst_gap_m": float(d.max()),
                   "hull_radial_extent_m": float(np.linalg.norm(V - V.mean(0), axis=1).max())}
    return out


def main():
    from trajgen.scene import load_original, load_original_task, load_scene, load_task
    from trajgen.tool_spheres import report

    sdir = ROOT / "scenes" / "welding_cell_2_small"
    new = SceneGeometry(load_scene(sdir), load_task(sdir / "task.json"))
    old = SceneGeometry(load_original(), load_original_task())
    urdf = new.scene.robots[0].urdf
    res = {"tool_sphere_coverage": {
        "corrected": report(urdf, new.scene.collision.tool_spheres.centers, new.scene.collision.tool_spheres.radii),
        "umarell_original": report(urdf, old.scene.collision.tool_spheres.centers, old.scene.collision.tool_spheres.radii)}}
    res["link_sphere_coverage_info"] = link_sphere_coverage(new)
    # floor rules on 20 000 uniform random joint configurations of robot1 (ERRATA E3/E4)
    kin = new.kin["robot1"]
    Q = np.random.default_rng(1).uniform(kin.q_lo, kin.q_hi, (20000, 6))
    um = old.umarell_floor_margin("robot1", Q) >= 0
    tool_z = new.spheres("robot1", Q)[:, new.n_link:, 2].min(1)
    hull_z = new.floor_hull_min_z("robot1", Q)
    res["random_configs_floor"] = {
        "n": int(len(Q)), "umarell_rule_pass_frac": float(um.mean()),
        "umarell_pass_but_tool_sphere_centre_below_floor": int(((tool_z < 0) & um).sum()),
        "umarell_pass_but_hull_below_floor": int(((hull_z < 0) & um).sum()),
        "corrected_sphere_rule_pass_frac": float((new.floor_sphere_margin("robot1", Q) >= 0).mean()),
        "corrected_pass_but_hull_below_floor": int(((hull_z < 0) & (new.floor_sphere_margin("robot1", Q) >= 0)).sum())}
    tl = {}
    for name, f in TIMELINES.items():
        p = ROOT / f
        if not p.exists():
            continue
        d = np.load(p, allow_pickle=True)
        q = d["q"]
        row = {"n_samples": int(len(q)), "t_end_s": float(d["t"][-1])}
        for k, rid in enumerate(new.rids):
            row[rid] = {
                "umarell_floor_margin_min_m": float(old.umarell_floor_margin(rid, q[:, k]).min()),
                "corrected_floor_sphere_margin_min_m": float(new.floor_sphere_margin(rid, q[:, k]).min()),
                "corrected_floor_sphere_violations": int((new.floor_sphere_margin(rid, q[:, k]) < 0).sum()),
                "hull_min_z_m": float(new.floor_hull_min_z(rid, q[:, k]).min()),
                "tcp_min_z_m": float(new.kin[rid].tcp(q[:, k])[0][:, 2].min())}
        c_old = old.pair_clearance(q[:, 0], q[:, 1]); c_new = new.pair_clearance(q[:, 0], q[:, 1])
        row["arm_arm_clearance_min_m"] = {"umarell_tool_spheres": float(c_old.min()), "corrected_tool_spheres": float(c_new.min()),
                                          "samples_below_gate_umarell": int((c_old <= 0.20).sum()),
                                          "samples_below_gate_corrected": int((c_new <= new.scene.collision.arm_arm_gate).sum())}
        tl[name] = row
    res["timelines"] = tl
    out = ROOT / "results" / "trajgen_errata_check.json"
    json.dump(res, open(out, "w"), indent=1)
    print(json.dumps(res, indent=1))
    print("wrote", out)


if __name__ == "__main__":
    main()
