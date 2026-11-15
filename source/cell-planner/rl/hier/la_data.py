"""Rollout datasets for the learned lookahead (phase 3, track G HIERLA, docs/p3_hierla.md).

The engineered lookahead (``features.FeatureBuilder``) runs an exact model-based forward check of every candidate
option: the umarell sphere clearance against the other arm's committed primitive (time-aligned at 20 Hz, stride 2),
the floor margin of the deciding arm's spheres, and MuJoCo hull self / floor contacts along the direct transit.
This module records **supervised labels** for learned replacements of that check while episodes of
``HierWeldEnv`` are played by a mixture of behaviour policies (greedy-safe heuristic at several temperatures,
noisy heuristic, uniform random, and the trained hier_u / hier_nl / hier_q schedulers with epsilon-greedy).

Three datasets per shard (labels come from the env's own geometric models, i.e. the same SphereModel / MuJoCo
hulls that score the episodes):

* ``pair``: (q_robot1, q_robot2) -> umarell sphere clearance (m). Pairs are the time-aligned (candidate, other arm)
  configurations the lookahead evaluates (all with clearance < 0.40 m, 20 % of the rest), plus Gaussian
  perturbations (sigma 0.05 rad) of kept pairs.
* ``arm``: (robot index, q) -> floor margin of the floor spheres (m), MuJoCo hull self/floor contact (0/1) and depth
  (m), on the transit configurations (waypoints + midpoints) the lookahead checks, plus perturbed and uniformly random
  configurations (hull contacts are rare along real transits).
* ``opt``: one row per (decision, valid option) for the option-level predictor: candidate path and the other arm's
  committed future sampled every ``OPT_DT`` s over ``OPT_N`` samples (clamped at their ends, i.e. rest afterwards),
  the transit sampled at ``OPT_NT`` fractions, lengths, and the engineered outputs (cmin, tmin, cend, fmin, hit,
  depth) as labels.

Training seeds avoid the reserved held-out / OOD / eval-pool seeds (train_hier.RESERVED); programs come from the
training pool (``runs/hier_pool/pool_train_*``), training randomisation (start noise 0.1 rad, pool offsets +-3 cm).

    PYTHONPATH=. .venv/bin/python -m rl.hier.la_data --out runs/hier_la_data/train --workers 150 --episodes 150 \
        --seed 1000
"""
from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OPT_N, OPT_STEP = 24, 5          # option samples: every 5 control steps (0.25 s) -> 6 s horizon
OPT_NT = 12                      # transit samples (fractions 0..1) for the hull contact head
KEEP_FAR = 0.2                   # keep-probability of pairs with clearance >= 0.40 m
RESERVED = [(50000, 61000), (900000, 1000000)]


def train_seed(rng):
    while True:
        s = int(rng.integers(2 ** 31))
        if not any(a <= s < b for a, b in RESERVED):
            return s


def aligned_indices(n_c, n_o, stride):
    """Time indices evaluated by FeatureBuilder.clearance_batch for one candidate of length n_c vs the other arm's
    future of length n_o (identical to the engineered lookahead)."""
    L = max(n_c, n_o)
    js = np.unique(np.concatenate([np.arange(0, L, stride), [L - 1]]))
    return np.minimum(js, n_c - 1), np.minimum(js, n_o - 1)


def opt_samples(P, n=OPT_N, step=OPT_STEP):
    P = np.asarray(P, float)
    idx = np.minimum(np.arange(n) * step, len(P) - 1)
    return P[idx]


def transit_samples(q0, path, n=OPT_NT):
    a = np.vstack([np.asarray(q0, float)[None], np.asarray(path, float)]) if len(path) else np.asarray(q0, float)[None]
    idx = np.round(np.linspace(0, len(a) - 1, n)).astype(int)
    return a[idx]


def hull_contacts(e, data, i, pts):
    """Per configuration of arm i (other arm at HOME): (hit 0/1, depth m) of hull self / floor contacts, the rule of
    FeatureBuilder.self_hit."""
    import mujoco
    o = 1 - i
    data.qpos[e.qadr[o]] = e.home[o]
    hit = np.zeros(len(pts), np.float32)
    dep = np.zeros(len(pts), np.float32)
    own = e.geom_owner
    for k, x in enumerate(pts):
        data.qpos[e.qadr[i]] = x
        mujoco.mj_kinematics(e.model, data)
        mujoco.mj_collision(e.model, data)
        n = data.ncon
        if n == 0:
            continue
        dist = data.contact.dist[:n]
        g = data.contact.geom[:n]
        oa, ob = own[g[:, 0]], own[g[:, 1]]
        bad = ((oa == i) & (ob == i)) | ((oa == i) & (ob == -2)) | ((oa == -2) & (ob == i))
        bad &= dist < -e.cfg.contact_tol
        if bad.any():
            hit[k] = 1.0
            dep[k] = float(-dist[bad].min())
    return hit, dep


def make_recorder(henv, rng):
    """FeatureBuilder subclass that computes the engineered lookahead exactly like the parent and records labels."""
    from env.cell import ROBOT_IDS
    from env.kinematics import FLOOR_Z_MIN

    from .features import FeatureBuilder

    class Recorder(FeatureBuilder):
        def __init__(self, h):
            super().__init__(h)
            self.rec = dict(pair_q1=[], pair_q2=[], pair_clr=[], arm_r=[], arm_q=[], arm_floor=[], arm_hit=[],
                            arm_depth=[], opt=[])
            self.rng = rng
            self._last = None

        def clearance_batch(self, i, paths, other_future):
            cmin, tmin, cend, fmin = super().clearance_batch(i, paths, other_future)
            # labels: recompute the aligned pairs (exactly as the parent) and keep a subsample
            qi, qo = [], []
            for P in paths:
                a, b = aligned_indices(len(P), len(other_future), self.stride)
                qi.append(P[a]); qo.append(other_future[b])
            qi, qo = np.concatenate(qi), np.concatenate(qo)
            q1, q2 = (qi, qo) if i == 0 else (qo, qi)
            clr = self.sm.pair_clearance(q1, q2)
            keep = (clr < 0.40) | (self.rng.uniform(size=len(clr)) < KEEP_FAR)
            r = self.rec
            r["pair_q1"].append(q1[keep]); r["pair_q2"].append(q2[keep]); r["pair_clr"].append(clr[keep])
            # perturbed copies of a share of the kept pairs (local coverage of the clearance field)
            k2 = np.flatnonzero(keep)
            k2 = k2[self.rng.uniform(size=len(k2)) < 0.3]
            if len(k2):
                p1 = q1[k2] + self.rng.normal(0, 0.05, (len(k2), 6))
                p2 = q2[k2] + self.rng.normal(0, 0.05, (len(k2), 6))
                p1 = np.clip(p1, self.q_lo[0], self.q_hi[0]); p2 = np.clip(p2, self.q_lo[1], self.q_hi[1])
                r["pair_q1"].append(p1); r["pair_q2"].append(p2); r["pair_clr"].append(self.sm.pair_clearance(p1, p2))
            # floor margin of the deciding arm's floor spheres on a subsample of its aligned configs
            ks = self.rng.uniform(size=len(qi)) < 0.15
            if ks.any():
                S = self.sm.spheres(ROBOT_IDS[i], qi[ks])
                r["arm_r"].append(np.full(int(ks.sum()), i, np.int8)); r["arm_q"].append(qi[ks])
                r["arm_floor"].append(S[:, self.floor_idx[i], 2].min(-1) - FLOOR_Z_MIN)
                r["arm_hit"].append(np.full(int(ks.sum()), np.nan, np.float32))      # not labelled (no MuJoCo here)
                r["arm_depth"].append(np.full(int(ks.sum()), np.nan, np.float32))
            self._last = dict(i=i, paths=paths, other=other_future, cmin=cmin, tmin=tmin, cend=cend, fmin=fmin)
            return cmin, tmin, cend, fmin

        def self_hit(self, i, q0, path):
            v = super().self_hit(i, q0, path)
            if len(path) == 0:
                return v
            if self.rng.uniform() < 0.35:
                a = np.vstack([np.asarray(q0, float)[None], path])
                pts = np.concatenate([a[1:], 0.5 * (a[1:] + a[:-1])])
                n = len(pts)
                extra = pts[self.rng.integers(0, n, max(1, n // 4))] + self.rng.normal(0, 0.05, (max(1, n // 4), 6))
                extra = np.clip(extra, self.q_lo[i], self.q_hi[i])
                uni = self.rng.uniform(self.q_lo[i], self.q_hi[i], (max(1, n // 8), 6))
                allp = np.concatenate([pts, extra, uni])
                hit, dep = hull_contacts(self.e, self.data, i, allp)
                S = self.sm.spheres(ROBOT_IDS[i], allp)
                r = self.rec
                r["arm_r"].append(np.full(len(allp), i, np.int8)); r["arm_q"].append(allp)
                r["arm_floor"].append(S[:, self.floor_idx[i], 2].min(-1) - FLOOR_Z_MIN)
                r["arm_hit"].append(hit); r["arm_depth"].append(dep)
            return v

        def build(self):
            obs = super().build()
            L = self._last
            if L is None:
                return obs
            h = self.h
            i = L["i"]
            acts = np.flatnonzero(obs["mask"])
            q_i = self.e.q[i].copy()
            for k, a in enumerate(acts):
                P = L["paths"][k]
                p = h.make_primitive(i, int(a))
                n_tr = p.info.get("n_transit", len(p.wps)) if p.kind != "wait" else 0
                hit, depth = self.self_hit_cached(i, q_i, p.wps[:n_tr]) if n_tr > 0 else (0.0, 0.0)
                self.rec["opt"].append(dict(
                    i=i, act=int(a), kind={"weld": 0, "wait": 1, "home": 2}[p.kind], q0=q_i,
                    cs=opt_samples(P), os=opt_samples(L["other"]), ts=transit_samples(q_i, p.wps[:n_tr]),
                    n_c=len(P), n_o=len(L["other"]), n_tr=n_tr,
                    y=np.array([L["cmin"][k], L["tmin"][k], L["cend"][k], L["fmin"][k], hit, depth], np.float32)))
            self._last = None
            return obs

        def self_hit_cached(self, i, q0, path):
            return FeatureBuilder.self_hit(self, i, q0, path)

    return Recorder(henv)


# ---------------------------------------------------------------------------------------------------- behaviour
def behaviour(name, rng, nets):
    from .heuristic import heuristic_action
    from .features import blind
    from .model import greedy_action
    if name.startswith("heur"):
        temp = float(name[4:])
        return lambda obs: heuristic_action(obs, rng=rng, temp=temp)
    if name.startswith("noisy"):
        p = float(name[5:])
        return lambda obs: int(rng.choice(np.flatnonzero(obs["mask"]))) if rng.uniform() < p \
            else heuristic_action(obs, rng=rng)
    if name == "random":
        return lambda obs: int(rng.choice(np.flatnonzero(obs["mask"])))
    net_name, eps = name.split("@")
    eps = float(eps)
    net = nets[net_name]
    see = blind if net_name == "nl" else (lambda o: o)

    def pol(obs):
        if rng.uniform() < eps:
            return int(rng.choice(np.flatnonzero(obs["mask"])))
        return greedy_action(net, see(obs), thr=0.3)
    return pol


MIX = [("heur0", 1), ("heur0.3", 2), ("heur1", 2), ("noisy0.15", 2), ("noisy0.5", 1), ("random", 1),
       ("u@0.1", 3), ("u@0.3", 1), ("nl@0.1", 2), ("q@0.1", 2)]
CKPTS = dict(u="runs/hier_u/selected_u75000.pt", nl="runs/hier_nl/selected_nl37750.pt", q="runs/hier_q/selected_q76500.pt")


def _flush(rec, out, rank, k, meta):
    def cat(key, dt=np.float32):
        xs = rec[key]
        return np.concatenate(xs).astype(dt) if xs else np.zeros((0,), dt)
    o = rec["opt"]
    d = dict(pair_q1=cat("pair_q1"), pair_q2=cat("pair_q2"), pair_clr=cat("pair_clr"),
             arm_r=cat("arm_r", np.int8), arm_q=cat("arm_q"), arm_floor=cat("arm_floor"), arm_hit=cat("arm_hit"),
             arm_depth=cat("arm_depth"))
    if o:
        d.update(opt_i=np.array([x["i"] for x in o], np.int8), opt_kind=np.array([x["kind"] for x in o], np.int8),
                 opt_q0=np.stack([x["q0"] for x in o]).astype(np.float32),
                 opt_cs=np.stack([x["cs"] for x in o]).astype(np.float32),
                 opt_os=np.stack([x["os"] for x in o]).astype(np.float32),
                 opt_ts=np.stack([x["ts"] for x in o]).astype(np.float32),
                 opt_n=np.array([[x["n_c"], x["n_o"], x["n_tr"]] for x in o], np.int32),
                 opt_y=np.stack([x["y"] for x in o]).astype(np.float32))
    p = Path(out) / f"shard_{rank:04d}_{k:03d}.npz"
    tmp = p.with_suffix(".tmp.npz")
    np.savez_compressed(tmp, **d, meta=np.array(str(meta)))
    tmp.replace(p)
    for v in rec.values():
        v.clear()


def worker(rank, args):
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    import torch
    torch.set_num_threads(1)
    from .hier_env import HierConfig, HierWeldEnv
    from .instances import load_pool
    from .model import HierQNet
    rng = np.random.default_rng([args.seed, rank])
    pool = load_pool(args.cell, tag="train")
    envs = {False: HierWeldEnv(HierConfig(cell_id=args.cell, exclude_offset0=True, p_canonical=0.0), pool=pool),
            True: HierWeldEnv(HierConfig(cell_id=args.cell, exclude_offset0=True, p_canonical=0.0, max_tilt_deg=3.5,
                                         min_self_dist=0.015), pool=pool)}
    recs = {}
    for c, e in envs.items():
        e.feat = make_recorder(e, rng)
        recs[c] = e.feat.rec
    nets = {}
    for k, path in CKPTS.items():
        sd = torch.load(ROOT / path, map_location="cpu", weights_only=False)
        n = HierQNet(**sd["cfg"]); n.load_state_dict(sd["net"]); n.eval()
        nets[k] = n
    names = [m for m, w in MIX for _ in range(w)]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    k_shard, t0 = 0, time.time()
    stats = []
    for ep in range(args.episodes):
        name = names[int(rng.integers(len(names)))]
        constrained = name.startswith("q@") or (not name.startswith(("u@", "nl@")) and rng.uniform() < 0.3)
        env = envs[constrained]
        pol = behaviour(name, rng, nets)
        seed = train_seed(rng)
        obs = env.reset(seed=seed)
        while True:
            obs, r, done, info = env.step(pol(obs))
            if done:
                break
        stats.append((name, bool(info["success"]), None if info["collision"] is None else info["collision"]["kind"]))
        if (ep + 1) % args.shard_episodes == 0 or ep + 1 == args.episodes:
            for c in (False, True):
                if recs[c]["opt"] or recs[c]["pair_clr"]:
                    _flush(recs[c], out, rank, k_shard, dict(rank=rank, constrained=c, stats=stats[-args.shard_episodes:]))
                    k_shard += 1
            print(f"[{rank}] ep {ep + 1}/{args.episodes} {time.time() - t0:.0f}s", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--episodes", type=int, default=20, help="per worker")
    ap.add_argument("--shard-episodes", type=int, default=25)
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--rank0", type=int, default=0)
    ap.add_argument("--cell", default="welding_cell_2_small")
    a = ap.parse_args()
    import multiprocessing as mp
    ps = [mp.get_context("spawn").Process(target=worker, args=(a.rank0 + r, a)) for r in range(a.workers)]
    for p in ps:
        p.start()
    for p in ps:
        p.join()


if __name__ == "__main__":
    main()
