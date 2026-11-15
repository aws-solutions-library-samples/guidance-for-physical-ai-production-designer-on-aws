"""Learned lookahead predictors (phase 3, track G HIERLA, docs/p3_hierla.md).

Two learned replacements of the engineered geometric lookahead (``features.FeatureBuilder.clearance_batch`` +
``self_hit``), trained supervised on the rollout datasets of ``rl.hier.la_data``:

* **field** (variant A): a learned *clearance field* ``PairNet(q_robot1, q_robot2) -> umarell sphere clearance`` and a
  learned *arm model* ``ArmNet(robot, q) -> (floor margin, hull self/floor contact logit, depth)``. At a decision they
  are evaluated on the same time-aligned configurations the engineered check uses (the executor's candidate
  waypoints vs the other arm's committed waypoints, stride 2) and min/max-pooled over time. The networks replace
  the forward kinematics, the 77+77 sphere distances and the MuJoCo hull contacts; the time alignment (which
  configurations co-occur) is the executor's plan.
* **option** (variant B): one network per option, ``OptNet(candidate path sampled every 0.25 s, other arm's committed
  future at the same times, transit samples, lengths) -> (cmin, tmin, cend, fmin, hit logit, depth)``: a direct
  per-option risk / clearance predictor with no pairing or pooling structure given.

Joint inputs are encoded as [(q - mid) / half, sin q, cos q] per arm (18 values).
"""
from __future__ import annotations

import glob
import json
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from env.cell import ROBOT_IDS

ROOT = Path(__file__).resolve().parents[2]
_LIM = {}


def joint_limits(cell_id="welding_cell_2_small"):
    if cell_id not in _LIM:
        from env.weld_env import EnvConfig, KukaWeldEnv
        e = KukaWeldEnv(EnvConfig(cell_id=cell_id))
        _LIM[cell_id] = (e.q_lo.astype(np.float32), e.q_hi.astype(np.float32))
    return _LIM[cell_id]


class JointEnc(nn.Module):
    """q (..., 6) of robot r -> (..., 18)."""

    def __init__(self, lo, hi):
        super().__init__()
        lo, hi = torch.as_tensor(np.asarray(lo), dtype=torch.float32), torch.as_tensor(np.asarray(hi), dtype=torch.float32)
        self.register_buffer("qmid", 0.5 * (lo + hi))       # (2,6)
        self.register_buffer("qhalf", 0.5 * (hi - lo))

    def forward(self, q, r):
        """q (..., 6); r int (robot index) or a long tensor broadcastable to q[..., 0]."""
        mid, half = self.qmid[r], self.qhalf[r]
        return torch.cat([(q - mid) / half, torch.sin(q), torch.cos(q)], -1)


def mlp(i, h, o, n):
    layers, d = [], i
    for _ in range(n):
        layers += [nn.Linear(d, h), nn.SiLU()]
        d = h
    layers.append(nn.Linear(d, o))
    return nn.Sequential(*layers)


class PairNet(nn.Module):
    """(q_robot1, q_robot2) -> sphere clearance (m)."""

    def __init__(self, lo, hi, hidden=512, layers=4):
        super().__init__()
        self.cfg = dict(hidden=hidden, layers=layers)
        self.enc = JointEnc(lo, hi)
        self.f = mlp(36, hidden, 1, layers)

    def forward(self, q1, q2):
        return self.f(torch.cat([self.enc(q1, 0), self.enc(q2, 1)], -1)).squeeze(-1)


class ArmNet(nn.Module):
    """(robot, q) -> [floor margin (m), hull self/floor contact logit, contact depth (m)]."""

    def __init__(self, lo, hi, hidden=256, layers=4):
        super().__init__()
        self.cfg = dict(hidden=hidden, layers=layers)
        self.enc = JointEnc(lo, hi)
        self.f = mlp(18 + 2, hidden, 3, layers)

    def forward(self, q, r):
        """q (N,6), r (N,) long."""
        oh = F.one_hot(r.long(), 2).float()
        return self.f(torch.cat([self.enc(q, r.long()), oh], -1))


class OptNet(nn.Module):
    """Option-level predictor. Inputs (batch B):
    i (B,) deciding arm, kind (B,) 0 weld / 1 wait / 2 home, q0 (B,6), cs (B,N,6) candidate samples, os (B,N,6) other
    arm samples, ts (B,NT,6) transit samples, n (B,3) lengths [n_c, n_o, n_tr] in control steps.
    Outputs (B,6): [cmin (m), tmin (0..1), cend (m), fmin (m), hit logit, depth (m)]."""

    def __init__(self, lo, hi, n=24, nt=12, hidden=256, layers=3):
        super().__init__()
        self.cfg = dict(n=n, nt=nt, hidden=hidden, layers=layers)
        self.enc = JointEnc(lo, hi)
        self.n, self.nt = n, nt
        H = hidden
        self.tok = mlp(36 + 2 + 1, H, H, 2)             # per time sample: deciding arm, other arm, flags, time
        self.conv = nn.Conv1d(H, H, 3, padding=1)
        self.tr = mlp(18 * nt + 2, H, H, 2)               # transit (hull contact)
        self.glob = mlp(18 + 3 + 2 + 3, H, H, 1)
        self.head = mlp(4 * H, 2 * H, 6, layers)

    def forward(self, i, kind, q0, cs, os, ts, n):
        B, N = cs.shape[:2]
        i = i.long()
        o = 1 - i
        ii = i[:, None].expand(B, N)
        oo = o[:, None].expand(B, N)
        t = torch.arange(N, device=cs.device, dtype=torch.float32)[None].expand(B, N) / N
        x = torch.cat([self.enc(cs, ii), self.enc(os, oo), F.one_hot(i, 2).float()[:, None].expand(B, N, 2),
                       t[..., None]], -1)
        h = self.tok(x)                                    # (B,N,H)
        h = h + torch.relu(self.conv(h.transpose(1, 2))).transpose(1, 2)
        hmin = -torch.logsumexp(-h, 1)
        hmean = h.mean(1)
        tt = self.tr(torch.cat([self.enc(ts, i[:, None].expand(B, ts.shape[1])).reshape(B, -1),
                                F.one_hot(i, 2).float()], -1))
        nf = torch.log1p(n.float()) / 5.0
        g = self.glob(torch.cat([self.enc(q0, i), F.one_hot(kind.long(), 3).float(), F.one_hot(i, 2).float(), nf], -1))
        return self.head(torch.cat([hmin, hmean, tt, g], -1))


# ---------------------------------------------------------------------------------------------------- data
def _keep_pairs(c, rng, frac):
    """Subsample pairs: all near the thresholds (0.10-0.45 m) with prob min(1, 4*frac), the rest with prob frac."""
    p = np.where((c > 0.10) & (c < 0.45), min(1.0, 4 * frac), frac)
    return rng.uniform(size=len(c)) < p


def load_shards(pattern, keys, max_files=None, frac=None, seed=0, arm_frac=None):
    """Concatenate shard arrays. ``frac``: per-shard subsample of the pair (and arm / opt) rows."""
    files = sorted(glob.glob(pattern))[:max_files]
    out = {k: [] for k in keys}
    rng = np.random.default_rng(seed)
    for f in files:
        d = np.load(f)
        if any(k not in d for k in keys):
            continue
        sel = {}
        if frac is not None and frac < 1:
            if "pair_clr" in keys:
                sel["pair"] = _keep_pairs(d["pair_clr"], rng, frac)
            if "arm_q" in keys:
                h = d["arm_hit"]
                af = frac if arm_frac is None else arm_frac
                sel["arm"] = rng.uniform(size=len(h)) < np.where(h > 0.5, min(1.0, 5 * af), af)
            if "opt_y" in keys:
                sel["opt"] = rng.uniform(size=len(d["opt_y"])) < np.where(np.abs(d["opt_y"][:, 0] - 0.22) < 0.15,
                                                                         min(1.0, 3 * frac), frac)
        for k in keys:
            x = d[k]
            g = k.split("_")[0]
            out[k].append(x[sel[g]] if g in sel else x)
    return {k: np.concatenate(v) if v else None for k, v in out.items()}, len(files)


def clr_weight(c):
    """Training weight of a clearance target: emphasis around the 0.20 / 0.25 m thresholds."""
    return 1.0 + 4.0 * np.exp(-((np.asarray(c) - 0.22) / 0.08) ** 2)


# ---------------------------------------------------------------------------------------------------- training
def _batches(n, B, rng):
    idx = rng.permutation(n)
    for s in range(0, n - B + 1, B):
        yield idx[s:s + B]


def train_pair(data, val, lo, hi, out, epochs=6, B=4096, lr=2e-3, hidden=512, layers=4, log=print, hours=None):
    net = PairNet(lo, hi, hidden, layers)
    opt = torch.optim.AdamW(net.parameters(), lr=lr, weight_decay=1e-5)
    q1, q2, c = (torch.as_tensor(data[k]) for k in ("pair_q1", "pair_q2", "pair_clr"))
    tgt = c.clamp(-0.3, 1.5)
    w = torch.as_tensor(clr_weight(data["pair_clr"]), dtype=torch.float32)
    n = len(c)
    steps = epochs * (n // B)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, lr, total_steps=max(1, steps), pct_start=0.05)
    rng = np.random.default_rng(0)
    t0, k = time.time(), 0
    for ep in range(epochs):
        net.train()
        for idx in _batches(n, B, rng):
            idx = torch.as_tensor(idx)
            p = net(q1[idx], q2[idx])
            loss = (w[idx] * F.smooth_l1_loss(p, tgt[idx], beta=0.02, reduction="none")).mean()
            opt.zero_grad(); loss.backward(); opt.step(); sched.step(); k += 1
            if k % 2000 == 0:
                log(f"[pair] ep {ep} step {k}/{steps} loss {float(loss.detach()):.5f} {time.time() - t0:.0f}s")
            if hours and time.time() - t0 > hours * 3600:
                break
        m = eval_pair(net, val)
        log(f"[pair] epoch {ep} val {json.dumps(m)}")
        torch.save(dict(net=net.state_dict(), cfg=net.cfg, lo=lo, hi=hi, val=m, epoch=ep), out)
    return net


@torch.no_grad()
def eval_pair(net, val, B=65536):
    net.eval()
    q1, q2, c = (torch.as_tensor(val[k]) for k in ("pair_q1", "pair_q2", "pair_clr"))
    p = torch.cat([net(q1[s:s + B], q2[s:s + B]) for s in range(0, len(c), B)]).numpy()
    c = c.numpy()
    err = p - c
    near = (c > 0.10) & (c < 0.40)
    return dict(n=int(len(c)), mae=float(np.abs(err[c < 1.5]).mean()), mae_near=float(np.abs(err[near]).mean()),
                p99_near=float(np.quantile(np.abs(err[near]), 0.99)),
                false_safe_020=float(((p > 0.20) & (c <= 0.20)).sum() / max(1, (c <= 0.20).sum())),
                false_unsafe_020=float(((p <= 0.20) & (c > 0.20)).sum() / max(1, (c > 0.20).sum())))


def train_arm(data, val, lo, hi, out, epochs=10, B=4096, lr=2e-3, hidden=256, layers=4, log=print):
    net = ArmNet(lo, hi, hidden, layers)
    opt = torch.optim.AdamW(net.parameters(), lr=lr, weight_decay=1e-5)
    r = torch.as_tensor(data["arm_r"]).long()
    q = torch.as_tensor(data["arm_q"])
    fl = torch.as_tensor(data["arm_floor"]).clamp(-0.5, 2.0)
    hit = torch.as_tensor(data["arm_hit"])
    dep = torch.as_tensor(data["arm_depth"])
    lab = torch.isfinite(hit)
    pos = float(hit[lab].mean())
    pw = torch.tensor(min(20.0, (1 - pos) / max(pos, 1e-4)))
    n = len(q)
    steps = epochs * (n // B)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, lr, total_steps=max(1, steps), pct_start=0.05)
    rng = np.random.default_rng(1)
    t0, k = time.time(), 0
    for ep in range(epochs):
        net.train()
        for idx in _batches(n, B, rng):
            idx = torch.as_tensor(idx)
            o = net(q[idx], r[idx])
            l_fl = F.smooth_l1_loss(o[:, 0], fl[idx], beta=0.02)
            m = lab[idx]
            l_hit = F.binary_cross_entropy_with_logits(o[m, 1], hit[idx][m], pos_weight=pw) if m.any() else o.sum() * 0
            mh = m & (hit[idx] > 0.5)
            mh_i = torch.zeros_like(m); mh_i[:] = mh
            l_dep = F.smooth_l1_loss(o[mh_i, 2], dep[idx][mh_i], beta=0.005) if mh_i.any() else o.sum() * 0
            loss = l_fl + l_hit + 10 * l_dep
            opt.zero_grad(); loss.backward(); opt.step(); sched.step(); k += 1
            if k % 2000 == 0:
                log(f"[arm] ep {ep} step {k}/{steps} fl {float(l_fl):.5f} hit {float(l_hit):.4f} {time.time() - t0:.0f}s")
        mm = eval_arm(net, val)
        log(f"[arm] epoch {ep} val {json.dumps(mm)}")
        torch.save(dict(net=net.state_dict(), cfg=net.cfg, lo=lo, hi=hi, val=mm, epoch=ep, pos_rate=pos), out)
    return net


@torch.no_grad()
def eval_arm(net, val, B=65536):
    net.eval()
    r = torch.as_tensor(val["arm_r"]).long()
    q = torch.as_tensor(val["arm_q"])
    o = torch.cat([net(q[s:s + B], r[s:s + B]) for s in range(0, len(q), B)]).numpy()
    fl = val["arm_floor"]
    hit = val["arm_hit"]
    lab = np.isfinite(hit)
    ph = o[lab, 1] > 0
    y = hit[lab] > 0.5
    return dict(n=int(len(q)), floor_mae=float(np.abs(o[:, 0] - fl)[fl < 1.0].mean()),
                floor_sign_err=float(((o[:, 0] < 0) != (fl < 0)).mean()),
                hit_n=int(lab.sum()), hit_pos=int(y.sum()), hit_recall=float((ph & y).sum() / max(1, y.sum())),
                hit_precision=float((ph & y).sum() / max(1, ph.sum())), hit_acc=float((ph == y).mean()))


OPT_KEYS = ("opt_i", "opt_kind", "opt_q0", "opt_cs", "opt_os", "opt_ts", "opt_n", "opt_y")


def _opt_t(d, idx=None):
    sl = slice(None) if idx is None else idx
    return [torch.as_tensor(d[k][sl]) for k in OPT_KEYS]


def opt_loss(o, y):
    w = torch.as_tensor(clr_weight(y[:, 0].numpy()), dtype=torch.float32)
    l = (w * F.smooth_l1_loss(o[:, 0], y[:, 0].clamp(-0.3, 1.5), beta=0.02, reduction="none")).mean()
    l = l + 0.2 * F.smooth_l1_loss(o[:, 1], y[:, 1], beta=0.05)
    l = l + F.smooth_l1_loss(o[:, 2], y[:, 2].clamp(-0.3, 1.5), beta=0.02)
    l = l + F.smooth_l1_loss(o[:, 3], y[:, 3].clamp(-0.5, 2.0), beta=0.02)
    l = l + F.binary_cross_entropy_with_logits(o[:, 4], y[:, 4], pos_weight=torch.tensor(5.0))
    m = y[:, 4] > 0.5
    if m.any():
        l = l + 10 * F.smooth_l1_loss(o[m, 5], y[m, 5], beta=0.005)
    return l


def train_opt(data, val, lo, hi, out, epochs=20, B=1024, lr=1e-3, hidden=256, layers=3, log=print):
    net = OptNet(lo, hi, n=data["opt_cs"].shape[1], nt=data["opt_ts"].shape[1], hidden=hidden, layers=layers)
    opt = torch.optim.AdamW(net.parameters(), lr=lr, weight_decay=1e-5)
    T = _opt_t(data)
    n = len(T[0])
    steps = epochs * (n // B)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, lr, total_steps=max(1, steps), pct_start=0.05)
    rng = np.random.default_rng(2)
    t0, k = time.time(), 0
    for ep in range(epochs):
        net.train()
        for idx in _batches(n, B, rng):
            idx = torch.as_tensor(idx)
            x = [t[idx] for t in T]
            o = net(*x[:-1])
            loss = opt_loss(o, x[-1])
            opt.zero_grad(); loss.backward(); opt.step(); sched.step(); k += 1
            if k % 1000 == 0:
                log(f"[opt] ep {ep} step {k}/{steps} loss {float(loss):.5f} {time.time() - t0:.0f}s")
        mm = eval_opt(net, val)
        log(f"[opt] epoch {ep} val {json.dumps(mm)}")
        torch.save(dict(net=net.state_dict(), cfg=net.cfg, lo=lo, hi=hi, val=mm, epoch=ep), out)
    return net


@torch.no_grad()
def predict_opt(net, d, B=8192):
    net.eval()
    T = _opt_t(d)
    return torch.cat([net(*[t[s:s + B] for t in T[:-1]]) for s in range(0, len(T[0]), B)]).numpy()


def option_metrics(cmin_p, cmin_t, hit_p, hit_t):
    """Decision-relevant agreement of predicted vs engineered option features."""
    cmin_p, cmin_t = np.asarray(cmin_p), np.asarray(cmin_t)
    near = (cmin_t > 0.10) & (cmin_t < 0.40)
    y, ph = np.asarray(hit_t) > 0.5, np.asarray(hit_p) > 0.5
    return dict(n=int(len(cmin_t)), cmin_mae=float(np.abs(cmin_p - cmin_t)[cmin_t < 1.5].mean()),
                cmin_mae_near=float(np.abs(cmin_p - cmin_t)[near].mean()) if near.any() else None,
                # predicted "clear of 0.25" but truly <= 0.20 (the heuristic's safe set admits a colliding option)
                false_safe_025_020=float(((cmin_p > 0.25) & (cmin_t <= 0.20)).sum() / max(1, (cmin_t <= 0.20).sum())),
                false_safe_020=float(((cmin_p > 0.20) & (cmin_t <= 0.20)).sum() / max(1, (cmin_t <= 0.20).sum())),
                false_unsafe_025=float(((cmin_p <= 0.25) & (cmin_t > 0.25)).sum() / max(1, (cmin_t > 0.25).sum())),
                flag020_acc=float(((cmin_p <= 0.20) == (cmin_t <= 0.20)).mean()),
                hit_recall=float((ph & y).sum() / max(1, y.sum())), hit_precision=float((ph & y).sum() / max(1, ph.sum())),
                hit_pos=int(y.sum()))


@torch.no_grad()
def eval_opt(net, val):
    o = predict_opt(net, val)
    y = val["opt_y"]
    m = option_metrics(o[:, 0], y[:, 0], 1 / (1 + np.exp(-o[:, 4])), y[:, 4])
    m["tmin_mae"] = float(np.abs(o[:, 1] - y[:, 1]).mean())
    m["fmin_sign_err"] = float(((o[:, 3] < 0) != (y[:, 3] < 0)).mean())
    return m


def load_net(path, kind):
    sd = torch.load(path, map_location="cpu", weights_only=False)
    cls = dict(pair=PairNet, arm=ArmNet, opt=OptNet)[kind]
    net = cls(sd["lo"], sd["hi"], **sd["cfg"])
    net.load_state_dict(sd["net"])
    net.eval()
    return net


def main():
    import argparse
    ap = argparse.ArgumentParser(description="train the learned lookahead predictors")
    ap.add_argument("--train", default="runs/hier_la_data/train/*.npz")
    ap.add_argument("--val", default="runs/hier_la_data/val/*.npz")
    ap.add_argument("--out", default="runs/hier_la_models")
    ap.add_argument("--which", default="pair,arm,opt")
    ap.add_argument("--threads", type=int, default=32)
    ap.add_argument("--max-files", type=int, default=None)
    ap.add_argument("--frac", type=float, default=0.15, help="train subsample (see load_shards)")
    ap.add_argument("--val-frac", type=float, default=0.1)
    ap.add_argument("--arm-frac", type=float, default=0.04)
    ap.add_argument("--opt-frac", type=float, default=0.3)
    ap.add_argument("--pair-epochs", type=int, default=6)
    ap.add_argument("--arm-epochs", type=int, default=10)
    ap.add_argument("--opt-epochs", type=int, default=20)
    ap.add_argument("--pair-hidden", type=int, default=512)
    ap.add_argument("--pair-layers", type=int, default=4)
    ap.add_argument("--pair-batch", type=int, default=8192)
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    torch.set_num_threads(a.threads)
    torch.manual_seed(0)
    out = ROOT / a.out
    out.mkdir(parents=True, exist_ok=True)
    logf = open(out / f"train{a.tag}.log", "a")

    def log(s):
        print(s, flush=True)
        logf.write(s + "\n"); logf.flush()
    lo, hi = joint_limits()
    which = a.which.split(",")
    if "pair" in which or "arm" in which:
        keys = (("pair_q1", "pair_q2", "pair_clr") if "pair" in which else ()) + \
            (("arm_r", "arm_q", "arm_floor", "arm_hit", "arm_depth") if "arm" in which else ())
        tr, nf = load_shards(str(ROOT / a.train), keys, a.max_files, frac=a.frac, arm_frac=a.arm_frac)
        va, nv = load_shards(str(ROOT / a.val), keys, frac=a.val_frac, seed=1, arm_frac=a.val_frac / 2)
        log(f"data: {nf} train shards, {nv} val shards; " + ", ".join(f"{k} {len(tr[k])} / {len(va[k])}" for k in keys))
        if "pair" in which:
            train_pair(tr, va, lo, hi, out / f"pair{a.tag}.pt", epochs=a.pair_epochs, hidden=a.pair_hidden,
                       layers=a.pair_layers, B=a.pair_batch, log=log)
        if "arm" in which:
            train_arm(tr, va, lo, hi, out / f"arm{a.tag}.pt", epochs=a.arm_epochs, log=log)
        del tr, va
    if "opt" in which:
        tr, nf = load_shards(str(ROOT / a.train), OPT_KEYS, a.max_files, frac=a.opt_frac)
        va, nv = load_shards(str(ROOT / a.val), OPT_KEYS)
        log(f"opt data: {len(tr['opt_y'])} / {len(va['opt_y'])}")
        train_opt(tr, va, lo, hi, out / f"opt{a.tag}.pt", epochs=a.opt_epochs, log=log)


if __name__ == "__main__":
    main()
