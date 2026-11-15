"""Weld instances: the IK branches the scheduler can choose from (Track H).

A *weld instance* is one weld of the program with a concrete position (canonical, or with the env's
+-3 cm noise). For every robot that can reach it, ``solve_branches`` returns up to ``K`` IK branches,
each with the weld configuration and the straight Cartesian descent pre-weld -> weld (0.10 m standoff
along ``-approach_vector``, shorter >= 0.045 m where the branch needs it), computed with Track X's
``ScriptedExpert.weld_options`` (5-DoF IK, hull self-collision + floor checked along the descent).

Branches are sorted by the expert's joint-travel cost from HOME; the first ``K`` are kept.

IK is slow (0.1-1.8 s per weld and robot), so training samples weld noise from a precomputed **pool**
of ``M`` offsets per weld (offset 0 = canonical). Episodes combine offsets independently per weld,
so the training set still has M^15 distinct programs; held-out evaluation solves fresh offsets
(``InstanceSolver.solve`` on the fly, other seeds).
"""
from __future__ import annotations

import pickle
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from env.cell import ROBOT_IDS, ROOT, load_cell

K_BRANCH = 3


@dataclass
class Branch:
    q_weld: np.ndarray        # (6,)
    desc: np.ndarray          # (n,6) pre-weld -> weld joint samples (IK path, not time-resampled)
    cost_home: float
    self_dist: float          # coal hull self distance at the weld configuration (m)
    standoff: float           # m
    tilt_deg: float = float("nan")       # tool-axis error vs the weld's nominal tool axis (deg); annotate()
    path_min_sd: float = float("nan")    # min coal hull self distance along the descent (m); annotate()

    @property
    def q_pre(self):
        return self.desc[0]


def table_path(cell_id):
    return ROOT / "data" / ("weld_ik.json" if cell_id == "welding_cell_2_small" else f"weld_ik_{cell_id}.json")


def branch_ok(b: Branch, max_tilt_deg=None, min_self_dist=None):
    """Branch-quality constraint (None = off): tool-axis tilt <= max_tilt_deg and hull self distance
    (at the weld and along the descent) >= min_self_dist."""
    if max_tilt_deg is not None and not (b.tilt_deg <= max_tilt_deg + 1e-6):
        return False
    if min_self_dist is not None and not (min(b.self_dist, b.path_min_sd) >= min_self_dist - 1e-9):
        return False
    return True


class InstanceSolver:
    """Wraps ScriptedExpert.weld_options (one per process; builds a throw-away env for dt/scales).
    With a fixture cell (welding_cell_2t) the expert checks umarell's box rule along the descent and uses
    the fixture-aware table data/weld_ik_welding_cell_2t.json."""

    def __init__(self, cell_id="welding_cell_2_small", k=K_BRANCH):
        from env.weld_env import KukaWeldEnv, load_reach
        from expert.scripted import ScriptedExpert
        self.env = KukaWeldEnv(cell_id=cell_id)
        self.ex = ScriptedExpert(self.env, table_path=table_path(cell_id))
        self.cell = load_cell(cell_id)
        self.k = k
        # robots with at least one usable table solution per weld (comfortable or thin): data/weld_ik.json
        tab = self.ex.table or {}
        self.reach_all = {}
        for w in self.cell.welds:
            ws = tab.get("welds", {}).get(w.wid)
            self.reach_all[w.wid] = [r for r in ROBOT_IDS
                                     if ws is not None and any(s["usable"] for s in ws["robots"][r]["solutions"])]
        self.home = {r: self.cell.robots[r].home for r in ROBOT_IDS}
        self.by_id = {w.wid: w for w in self.cell.welds}

    def solve(self, wid: str, pos, rid: str) -> list[Branch]:
        if rid not in self.reach_all[wid]:
            return []
        from env.rewards import quat_xyzw_to_mat
        w = self.by_id[wid]
        R = quat_xyzw_to_mat(np.asarray(w.quat_xyzw, float)[None])[0]
        opts = self.ex.weld_options(rid, np.asarray(pos, float), R, w.approach, self.home[rid])
        out = []
        for cost, qw, desc in opts[: self.k]:
            sd = float(self.ex.ik.self_distance(rid, qw))
            p_pre = self.ex.ik.tcp_world(rid, desc[0])[0]
            b = Branch(np.asarray(qw, float), np.asarray(desc, float), float(cost), sd,
                       float(np.linalg.norm(p_pre - np.asarray(pos))))
            out.append(self.annotate(b, rid, wid))
        return out

    def annotate(self, b: Branch, rid, wid):
        """Fill tilt_deg (tool axis vs the weld's nominal axis) and path_min_sd (descent self distance)."""
        from env.rewards import quat_xyzw_to_mat
        Rw = quat_xyzw_to_mat(np.asarray(self.by_id[wid].quat_xyzw, float)[None])[0]
        _, Rt = self.ex.ik.tcp_world(rid, b.q_weld)
        c = float(np.clip(np.dot(-Rt[:, 0], -Rw[:, 0]), -1, 1))
        b.tilt_deg = float(np.degrees(np.arccos(c)))
        b.path_min_sd = float(min(self.ex.ik.self_distance(rid, q) for q in b.desc))
        return b

    def solve_weld(self, wid, pos):
        return {rid: self.solve(wid, pos, rid) for rid in ROBOT_IDS}


# ---------------------------------------------------------------------------- pool
def pool_path(cell_id="welding_cell_2_small", tag="train"):
    return ROOT / "runs" / "hier_pool" / f"pool_{tag}_{cell_id}.pkl"


def _pool_job(args):
    cell_id, wid, m, pos = args
    s = _solver(cell_id)
    return wid, m, pos, s.solve_weld(wid, pos)


_SOLVER = {}


def _solver(cell_id):
    if cell_id not in _SOLVER:
        _SOLVER[cell_id] = InstanceSolver(cell_id)
    return _SOLVER[cell_id]


def build_pool(cell_id="welding_cell_2_small", m_offsets=32, seed=12345, noise=0.03, workers=3, tag="train",
               out=None, verbose=True):
    """{wid: [(pos (3,), {rid: [Branch]}) for m in range(M)]}; m = 0 is the canonical position."""
    from multiprocessing import Pool
    cell = load_cell(cell_id)
    rng = np.random.default_rng(seed)
    jobs = []
    for w in cell.welds:
        offs = rng.uniform(-noise, noise, (m_offsets, 3))
        offs[0] = 0.0
        for m in range(m_offsets):
            jobs.append((cell_id, w.wid, m, np.asarray(w.pos, float) + offs[m]))
    t0 = time.time()
    res = []
    with Pool(workers) as p:
        for i, r in enumerate(p.imap_unordered(_pool_job, jobs, chunksize=4)):
            res.append(r)
            if verbose and (i + 1) % 50 == 0:
                print(f"pool {i + 1}/{len(jobs)} {time.time() - t0:.0f}s", flush=True)
    pool = {w.wid: [None] * m_offsets for w in cell.welds}
    for wid, m, pos, br in res:
        pool[wid][m] = (pos, br)
    out = Path(out or pool_path(cell_id, tag))
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp")
    with open(tmp, "wb") as f:
        pickle.dump({"cell_id": cell_id, "seed": seed, "noise": noise, "M": m_offsets, "pool": pool}, f)
    tmp.replace(out)
    return pool


def load_pool(cell_id="welding_cell_2_small", tag="train"):
    from robotmodel.override import active_override
    if active_override() is not None:   # docs/p3_urdf.md: the pool's IK branches belong to the cell's own URDF
        raise RuntimeError("robot override active: runs/hier_pool/* holds IK branches of the cell's own URDF; use "
                           "InstanceSolver (fresh IK) or rebuild the pool with build_pool(..., out=...) under the override")
    with open(pool_path(cell_id, tag), "rb") as f:
        return pickle.load(f)["pool"]


def _annotate_job(args):
    cell_id, wid, m, rid, bs = args
    s = _solver(cell_id)
    return wid, m, rid, [s.annotate(b, rid, wid) for b in bs]


def annotate_pool(cell_id="welding_cell_2_small", tag="train", workers=3):
    """Add tilt_deg / path_min_sd to an existing pool file (in place, atomic)."""
    from multiprocessing import get_context
    p = pool_path(cell_id, tag)
    d = pickle.load(open(p, "rb"))
    jobs = [(cell_id, wid, m, rid, bs) for wid, lst in d["pool"].items() for m, (pos, br) in enumerate(lst)
            for rid, bs in br.items() if bs]
    with get_context("spawn").Pool(workers) as pl:
        for wid, m, rid, bs in pl.imap_unordered(_annotate_job, jobs, chunksize=16):
            d["pool"][wid][m][1][rid] = bs
    tmp = p.with_suffix(".tmp")
    pickle.dump(d, open(tmp, "wb"))
    tmp.replace(p)
    return d["pool"]
