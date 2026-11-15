"""Learned lookahead features for the hierarchical scheduler (phase 3, track G HIERLA, docs/p3_hierla.md).

``LearnedFeatureBuilder`` is ``features.FeatureBuilder`` with ``lookahead`` replaced by a learned predictor
(``rl.hier.la_model``). The engineered geometric check (SphereModel sphere distances, floor spheres, MuJoCo hull
contacts) is **never called** in these modes; the graph features keep the same columns and semantics
(cand 15..22 / spec 5..10: predicted min clearance, <= 0.20 / <= 0.25 flags, time of the minimum, final clearance,
hull-contact probability, depth, floor flag), so ``HierQNet`` and the heuristic run unchanged.

Modes:
  ``engineered``  the exact model-based check (features.FeatureBuilder, unchanged)
  ``field``       variant A: PairNet clearance field + ArmNet (floor margin, hull contact) on the time-aligned
                  configurations of the executor's plans (stride 2, like the engineered check), min/max-pooled
  ``field_dense`` variant A at stride 1 (every 20 Hz step; the engineered check uses stride 2)
  ``opt``         variant B: OptNet, one direct prediction per option
  ``distill``     variant D (rl/hier/la_dqn.py): no predictor; the scheduler network zeroes the lookahead columns
                  itself and is trained with an auxiliary loss to predict them (distillation into the GNN)

Activation (also in spawned worker processes): ``install(mode, model_dir)`` patches ``HierWeldEnv.obs`` so that new
envs use the learned builder; ``install_from_env()`` reads ``HIER_LA=<mode>:<model_dir>`` from the environment.
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import torch

from env.cell import ROBOT_IDS

from .features import WARN, FeatureBuilder, blind
from .la_data import OPT_N, OPT_STEP, aligned_indices, opt_samples, transit_samples

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODELS = "runs/hier_la_models"
_NETS = {}


def _nets(model_dir, kinds):
    from .la_model import load_net
    out = {}
    for k in kinds:
        p = str(Path(model_dir) if Path(model_dir).is_absolute() else ROOT / model_dir) + f"/{k}.pt"
        if p not in _NETS:
            _NETS[p] = load_net(p, k)
        out[k] = _NETS[p]
    return out


class LearnedFeatureBuilder(FeatureBuilder):
    def __init__(self, henv, mode="field", model_dir=DEFAULT_MODELS, stride=None):
        super().__init__(henv, stride=stride or (1 if mode == "field_dense" else 2))
        self.mode = mode
        kinds = {"field": ("pair", "arm"), "field_dense": ("pair", "arm"), "opt": ("opt",)}.get(mode, ())
        self.nets = _nets(model_dir, kinds) if kinds else {}
        self.calls = 0

    @torch.no_grad()
    def lookahead(self, i, q_i, prs, fut_o):
        self.calls += 1
        n = len(prs)
        if self.mode in ("engineered", "distill"):     # distill: exact features in the obs, the network blinds itself
            return super().lookahead(i, q_i, prs, fut_o)
        if n == 0:
            z = np.zeros(0)
            return z, z, z, z, z, z
        n_tr = [p.info.get("n_transit", len(p.wps)) if p.kind != "wait" else 0 for p in prs]
        if self.mode == "opt":
            return self._opt(i, q_i, prs, fut_o, n_tr)
        return self._field(i, q_i, prs, fut_o, n_tr)

    def _field(self, i, q_i, prs, fut_o, n_tr):
        pair, arm = self.nets["pair"], self.nets["arm"]
        qi, qo, seg = [], [], []
        for c, p in enumerate(prs):
            a, b = aligned_indices(len(p.wps), len(fut_o), self.stride)
            qi.append(p.wps[a]); qo.append(fut_o[b]); seg.append(np.full(len(a), c))
        qi, qo, seg = np.concatenate(qi), np.concatenate(qo), np.concatenate(seg)
        Qi = torch.as_tensor(qi, dtype=torch.float32)
        Qo = torch.as_tensor(qo, dtype=torch.float32)
        clr = (pair(Qi, Qo) if i == 0 else pair(Qo, Qi)).numpy()
        # transit points (waypoints + midpoints) for the hull contact head, same points as FeatureBuilder.self_hit
        tp, tseg = [], []
        for c, (p, nt) in enumerate(zip(prs, n_tr)):
            if nt > 0:
                a = np.vstack([np.asarray(q_i, float)[None], p.wps[:nt]])
                pts = np.concatenate([a[1:], 0.5 * (a[1:] + a[:-1])])
                tp.append(pts); tseg.append(np.full(len(pts), c))
        allq = np.concatenate([qi] + tp) if tp else qi
        o = arm(torch.as_tensor(allq, dtype=torch.float32), torch.full((len(allq),), i, dtype=torch.long)).numpy()
        fl = o[:len(qi), 0]
        n = len(prs)
        cmin, tmin, cend, fmin = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
        bounds = np.flatnonzero(np.diff(seg)) + 1
        for c, (a, b) in enumerate(zip(np.r_[0, bounds], np.r_[bounds, len(seg)])):
            cc = clr[a:b]
            k = int(np.argmin(cc))
            cmin[c], tmin[c], cend[c] = cc[k], k / max(1, b - a - 1), cc[-1]
            fmin[c] = fl[a:b].min()
        hits, depths = np.zeros(n), np.zeros(n)
        if tp:
            ot = o[len(qi):]
            ts = np.concatenate(tseg)
            ph = 1.0 / (1.0 + np.exp(-ot[:, 1]))
            for c in np.unique(ts):
                m = ts == c
                hits[c] = float(ph[m].max())
                hd = ot[m, 2][ph[m] > 0.5]
                depths[c] = float(max(0.0, hd.max())) if len(hd) else 0.0
        return cmin, tmin, cend, fmin, hits, depths

    def _opt(self, i, q_i, prs, fut_o, n_tr):
        net = self.nets["opt"]
        n = len(prs)
        cs = np.stack([opt_samples(p.wps, net.n, OPT_STEP) for p in prs])
        os_ = np.repeat(opt_samples(fut_o, net.n, OPT_STEP)[None], n, 0)
        ts = np.stack([transit_samples(q_i, p.wps[:nt], net.nt) for p, nt in zip(prs, n_tr)])
        kind = np.array([{"weld": 0, "wait": 1, "home": 2}[p.kind] for p in prs])
        nn_ = np.array([[len(p.wps), len(fut_o), nt] for p, nt in zip(prs, n_tr)])
        T = lambda x, dt=torch.float32: torch.as_tensor(np.asarray(x), dtype=dt)  # noqa: E731
        o = net(T(np.full(n, i), torch.long), T(kind, torch.long), T(np.repeat(np.asarray(q_i)[None], n, 0)), T(cs),
                T(os_), T(ts), T(nn_, torch.long)).numpy()
        hit = 1.0 / (1.0 + np.exp(-o[:, 4]))
        hit = np.where(np.asarray(n_tr) > 0, hit, 0.0)
        depth = np.where(hit > 0.5, np.maximum(o[:, 5], 0.0), 0.0)
        return o[:, 0], np.clip(o[:, 1], 0, 1), o[:, 2], o[:, 3], hit, depth


# ---------------------------------------------------------------------------------------------------- install
_STATE = {}


def install(mode, model_dir=DEFAULT_MODELS):
    """Make every HierWeldEnv (created or reset later in this process) use ``mode``'s lookahead."""
    from . import hier_env as HE
    if "orig_obs" not in _STATE:
        _STATE["orig_obs"] = HE.HierWeldEnv.obs
    _STATE.update(mode=mode, model_dir=model_dir)
    if mode == "engineered_orig":
        HE.HierWeldEnv.obs = _STATE["orig_obs"]
        return
    if mode == "distill":            # variant D: engineered features stay in the obs, the network blinds itself
        from .la_dqn import install_distill
        install_distill()
        os.environ["HIER_LA"] = f"{mode}:{model_dir}"
        return

    def obs(self):
        if not isinstance(self.feat, LearnedFeatureBuilder) or self.feat.mode != _STATE["mode"]:
            self.feat = LearnedFeatureBuilder(self, _STATE["mode"], _STATE["model_dir"])
        return self.feat.build()
    HE.HierWeldEnv.obs = obs
    os.environ["HIER_LA"] = f"{mode}:{model_dir}"


def install_from_env():
    v = os.environ.get("HIER_LA")
    if v:
        mode, _, md = v.partition(":")
        install(mode, md or DEFAULT_MODELS)
    return v


def eval_init(o, record):
    """evaluate_hier pool initializer that first installs the learned lookahead (HIER_LA env var)."""
    import torch as _t
    _t.set_num_threads(1)
    install_from_env()
    import evaluate_hier as EH
    getattr(EH, "_orig_init", EH._init)(o, record)     # parent: EH._init is this function (p3_hierla_eval.py)


__all__ = ["LearnedFeatureBuilder", "install", "install_from_env", "eval_init", "blind", "ROBOT_IDS"]
