"""Evaluate residual policies (Track RR): canonical 15-weld task + N held-out randomised variants.

    python evaluate_residual.py --checkpoint runs/res_full_s0/best.pt --episodes 100 --workers 3 \
        --ablation --out results/res_full_s0.json [--save-timeline data/timelines/policy_residual_canonical.npz]
    python evaluate_residual.py --zero --base noyield --episodes 100 --out results/res_base_noyield.json

For each controller: the deterministic policy (u = actor(s)) and, with --ablation, the same wrapper with
the residual zeroed (u = 0, i.e. exactly the base controller). Held-out seeds 50000.. (training used
seeds >= 10**6, training evals 90000..). Collision verdicts are the env's own per-step checks (umarell
sphere gate <= 0.20 m arm-arm + umarell floor/box rules + MuJoCo hull contacts: self, floor, arm-arm);
the canonical rollout is re-checked offline on a 4x finer time grid (``verify_canonical``) and with the
umarell-replica 25 Hz ``inter_robot_clearance``.
"""
from __future__ import annotations

import os

# numpy/OpenBLAS in the env workers and the learner: one thread each (torch threads are set
# explicitly); otherwise OpenBLAS starts one thread per core in every process.
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse
import json
import time
from multiprocessing import get_context
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent


def _make(spec):
    from rl.residual_pool import make_res_env
    return make_res_env(spec)


def _policy(ckpt, spec=None):
    if ckpt is None:
        from rl.residual_eval import zero_policy
        return zero_policy
    if isinstance(ckpt, str) and ckpt.startswith("heur:"):
        from rl.residual_eval import heuristic_yield_policy
        return heuristic_yield_policy(float(ckpt.split(":")[1]))
    if isinstance(ckpt, str) and ckpt.startswith("const:"):   # hand-set constant clock rate, no learning
        sig = float(ckpt.split(":")[1])
        rb = float((spec or {}).get("residual", {}).get("rate_bound", 1.0))

        def const(obs, _u=(sig - 1.0) / rb):
            u = np.zeros((2, 7), np.float32)
            u[:, 6] = _u
            return u
        return const
    import torch
    torch.set_num_threads(1)
    from rl.residual_eval import agent_policy
    from rl.residual_models import load_residual_agent
    agent, _ = load_residual_agent(ckpt)
    return agent_policy(agent)


def _job(args):
    spec, ckpt, seeds, canonical = args
    from rl.residual_eval import rollout
    renv = _make(spec)
    pol = _policy(ckpt, spec)
    out = []
    if canonical:
        r = rollout(renv, pol, options={"canonical": True}, record=True)
        r["seed"] = "canonical"
        out.append(r)
    for s in seeds:
        r = rollout(renv, pol, seed=int(s))
        r["seed"] = int(s)
        out.append(r)
    return out


def run(spec, ckpt, seeds, workers):
    chunks = [list(c) for c in np.array_split(np.asarray(seeds, int), max(1, workers))]
    jobs = [(spec, ckpt, c, i == 0) for i, c in enumerate(chunks)]
    if workers <= 1:
        res = [_job(j) for j in jobs]
    else:
        with get_context("forkserver").Pool(workers) as p:
            res = p.map(_job, jobs)
    eps = [e for part in res for e in part]
    can = [e for e in eps if e["seed"] == "canonical"][0]
    return can, [e for e in eps if e["seed"] != "canonical"]


def verify_canonical(spec, q, sub=4):
    """Offline re-check of a joint trajectory: env.check_collisions at sub x finer interpolation +
    umarell-replica inter_robot_clearance (25 Hz grid)."""
    renv = _make(spec)
    env = renv.env
    kinds, min_clr = {}, np.inf
    pts = [(1 - f) * q[k] + f * q[k + 1] for k in range(len(q) - 1) for f in np.arange(sub) / sub] + [q[-1]]
    for x in pts:
        c = env.check_collisions(x)
        min_clr = min(min_clr, c["min_clearance"])
        for kk in c["kinds"]:
            kinds[kk] = kinds.get(kk, 0) + 1
    t = np.arange(len(q)) * env.dt
    irc = env.spheres.inter_robot_clearance({"robot1": (t, q[:, 0]), "robot2": (t, q[:, 1])})
    return dict(samples=len(pts), collision_samples_by_kind=kinds, min_clearance_fine=float(min_clr),
                umarell_inter_robot_clearance_min=float(irc.get("min_clearance_raw", irc["min_clearance_m"])),
                umarell_collision_free=bool(irc["collision_free"]))


def clean(e):
    return {k: v for k, v in e.items() if k not in ("t", "q", "weld_events")}


def select(run_dir, n_val, workers, out, min_steps=25000):
    """Model selection on validation seeds 91000.. (disjoint from the held-out 50000..): every kept
    checkpoint of a run; rank by canonical success without collision, then validation
    success - 2 * collision rate, then canonical makespan."""
    import torch
    from rl.residual_eval import summarize
    run_dir = Path(run_dir)
    cands = sorted(run_dir.glob("ckpt_[0-9]*.pt"), key=lambda p: int(p.stem.split("_")[1]))
    cands += [p for p in (run_dir / "best.pt", run_dir / "final.pt") if p.exists()]
    rows = []
    cache = {}
    if Path(out).exists():   # reuse rows of checkpoints already evaluated (same file, same steps)
        try:
            for r in json.load(open(out))["rows"]:
                cache[(Path(r["ckpt"]).name, r["steps"])] = r
        except Exception:  # noqa: BLE001
            cache = {}
    for c in cands:
        ck = torch.load(c, map_location="cpu", weights_only=False)
        if ck["steps"] < min_steps:      # the untrained initial actor (~ the expert) is not a candidate
            continue
        if (c.name, ck["steps"]) in cache and c.name not in ("best.pt", "final.pt"):
            rows.append(cache[(c.name, ck["steps"])])
            continue
        can, val = run(ck["spec"], str(c), list(range(91000, 91000 + n_val)), workers)
        s = summarize(val)
        ok = bool(can["success"] and not can["collision"])
        key = (ok, s["success_rate"] - 2 * s["collision_rate"], -(can["makespan_s"] or 1e9))
        rows.append(dict(ckpt=str(c), steps=ck["steps"], canonical=clean(can), val=s, key=list(key)))
        print(f"{c.name}: steps {ck['steps']} canonical ok {ok} makespan {can['makespan_s']} | val succ "
              f"{s['success_rate']:.2f} coll {s['collision_rate']:.2f} makespan {s['mean_makespan_success_s']}", flush=True)
    best = max(rows, key=lambda r: tuple(r["key"])) if rows else None
    json.dump(dict(run=str(run_dir), n_val=n_val, rows=rows, selected=best and best["ckpt"]), open(out, "w"),
              indent=1, default=float)
    print("selected:", best and best["ckpt"])
    return best


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--select-run", help="model selection over the kept checkpoints of runs/<name> (validation seeds)")
    p.add_argument("--n-val", type=int, default=30)
    p.add_argument("--checkpoint")
    p.add_argument("--zero", action="store_true", help="base controller only (u = 0)")
    p.add_argument("--const-sigma", type=float, default=None, help="baseline: constant clock rate, no learning")
    p.add_argument("--heuristic", type=float, default=None, help="baseline: hand-written yield rule with threshold (m)")
    p.add_argument("--base", default=None, choices=[None, "full", "noyield"])
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--seed0", type=int, default=50000)
    p.add_argument("--workers", type=int, default=3)
    p.add_argument("--ablation", action="store_true", help="also run the same wrapper with the residual zeroed")
    p.add_argument("--out", required=True)
    p.add_argument("--save-timeline")
    p.add_argument("--label", default=None)
    a = p.parse_args()
    if a.select_run:
        select(a.select_run, a.n_val, a.workers, a.out)
        return

    if a.checkpoint:
        import torch
        ck = torch.load(a.checkpoint, map_location="cpu", weights_only=False)
        spec = ck["spec"]
        meta = dict(checkpoint=a.checkpoint, steps=ck["steps"], updates=ck["agent"]["n_updates"])
    else:
        spec = dict(reward=dict(step_penalty=-0.005), env={}, residual=dict(base=a.base or "full"), canonical_frac=0.0)
        meta = {}
    if a.base:
        spec["residual"]["base"] = a.base
    seeds = list(range(a.seed0, a.seed0 + a.episodes))
    out = dict(spec=spec, meta=meta, seeds=[seeds[0], seeds[-1]] if seeds else [])
    runs = []
    if a.checkpoint and not a.zero:
        runs.append(("policy", a.checkpoint))
    if a.heuristic is not None:
        runs.append((f"heuristic_{a.heuristic:g}", f"heur:{a.heuristic}"))
    if a.const_sigma is not None:
        runs.append((f"const_sigma_{a.const_sigma:g}", f"const:{a.const_sigma}"))
    if a.zero or a.ablation:
        runs.append(("zero_residual", None))
    timelines = {}
    for name, ck in runs:
        t0 = time.time()
        can, rnd = run(spec, ck, seeds, a.workers)
        from rl.residual_eval import summarize
        out[name] = dict(canonical=clean(can), random=summarize(rnd), episodes=[clean(e) for e in rnd],
                         wall_s=time.time() - t0)
        out[name]["canonical"]["verify"] = verify_canonical(spec, can["q"])
        timelines[name] = can
        c, s = out[name]["canonical"], out[name]["random"]
        s = s or dict(n=0, success_rate=float("nan"), collision_rate=float("nan"), collision_kinds={},
                      weld_frac=float("nan"), mean_makespan_success_s=None)
        print(f"{name}: canonical welds {c['welds_done']}/15 coll {c['collision_kind']} makespan {c['makespan_s']} "
              f"min clr {c['min_clearance']:.3f} sigma {c['sigma_mean']:.3f} res_joint {c['res_joint_mean']:.4f} | "
              f"random({s['n']}): success {s['success_rate']:.2f} coll {s['collision_rate']:.2f} {s['collision_kinds']} "
              f"weld frac {s['weld_frac']:.3f} makespan {s['mean_makespan_success_s']}  ({time.time() - t0:.0f}s)",
              flush=True)
        print("   verify:", c["verify"], flush=True)
    # paired comparison policy vs zero residual on the episodes both completed
    if "policy" in out and "zero_residual" in out:
        P = {e["seed"]: e for e in out["policy"]["episodes"]}
        Z = {e["seed"]: e for e in out["zero_residual"]["episodes"]}
        both = [s for s in P if P[s]["success"] and Z[s]["success"]]
        if both:
            d = np.array([P[s]["makespan_s"] - Z[s]["makespan_s"] for s in both])
            out["paired"] = dict(n_both_success=len(both), makespan_diff_mean_s=float(d.mean()),
                                 makespan_ratio_mean=float(np.mean([P[s]["makespan_s"] / Z[s]["makespan_s"] for s in both])),
                                 frac_faster=float((d < -1e-9).mean()))
            print("paired:", out["paired"])
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=1, default=float)
    print("wrote", a.out)
    if a.save_timeline:
        from player.timeline import timeline_from_arrays
        name = "policy" if "policy" in timelines else "zero_residual"
        can = timelines[name]
        c = out[name]["canonical"]
        tl = timeline_from_arrays(can["t"], can["q"], weld_events=can["weld_events"],
                                  meta=dict(controller=a.label or f"residual RL ({spec['residual']['base']} base)",
                                            source=a.checkpoint, makespan_motion_s=c["makespan_s"],
                                            welds_done=c["welds_done"], collision=c["collision"],
                                            min_clearance=c["min_clearance"], steps=c["steps"]),
                                  op_prefix="RESIDUAL")
        tl.save(a.save_timeline)
        print("timeline ->", a.save_timeline)


if __name__ == "__main__":
    main()
