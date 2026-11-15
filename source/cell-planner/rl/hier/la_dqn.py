"""Variant D of the learned lookahead (phase 3, track G HIERLA, docs/p3_hierla.md): distil the engineered lookahead
into the scheduler GNN.

``HierQNetDistill`` is ``HierQNet`` that **zeroes the lookahead columns of its input itself** (``features.blind``:
cand 15..22, spec 5..10) before encoding, and has an auxiliary head per candidate edge / special option that predicts
those engineered columns from the edge's hidden representation. ``DistillLearner`` adds the auxiliary regression
(weight ``lam_aux``) to the DQN loss; the engineered values are only used as training targets (and by the
greedy-safe heuristic that collects demos, as in the ``hier_nl`` ablation). At test time the network never reads
them: feeding blinded observations gives bit-identical outputs (tests/test_hier_la.py).
"""
from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from .dqn import DQNLearner, q_forward, qc_forward
from .features import LOOK_CAND, LOOK_SPEC
from .model import HierQNet, masked_q, mlp, safe_q

N_LC, N_LS = LOOK_CAND.stop - LOOK_CAND.start, LOOK_SPEC.stop - LOOK_SPEC.start


class HierQNetDistill(HierQNet):
    def __init__(self, hidden=128, layers=3, heads=4, distill=True):
        super().__init__(hidden, layers, heads)
        self.cfg = dict(hidden=hidden, layers=layers, heads=heads, distill=True)
        H = hidden
        self.aux_c = mlp(H, H, N_LC, n=2)
        self.aux_s = mlp(4 * H, H, N_LS, n=2)
        self.last_aux = None

    def forward_both(self, robot, task, task_mask, cand, spec):
        tgt_c, tgt_s = cand[..., LOOK_CAND].clone(), spec[..., LOOK_SPEC].clone()
        cand = cand.clone(); spec = spec.clone()
        cand[..., LOOK_CAND] = 0.0
        spec[..., LOOK_SPEC] = 0.0
        B, T = task.shape[:2]
        K = cand.shape[2]
        xr = self.enc_r(robot) + self.type_emb[:2]
        xt = self.enc_t(task) + self.type_emb[2]
        x = torch.cat([xr, xt], 1)
        pad = torch.cat([torch.zeros(B, 2, dtype=torch.bool, device=task.device), ~task_mask], 1)
        for blk in self.blocks:
            x = blk(x, pad)
        x = self.ln(x)
        hd, ho, ht = x[:, 0], x[:, 1], x[:, 2:]
        valid = cand[..., 0] > 0.5
        bi, ti, ki = valid.nonzero(as_tuple=True)
        qc = cand.new_full((B, T, K), -1e9)
        cc = cand.new_full((B, T, K), 10.0)
        aux_c = None
        if len(bi):
            a = self.qc_r(hd) + self.qc_o(ho)
            bt = self.qc_t(ht)
            ec = self.qc_c(torch.relu(self.enc_c(cand[bi, ti, ki])))
            z = torch.relu(a[bi] + bt[bi, ti] + ec)
            o = self.q_c(z)
            qc = qc.index_put((bi, ti, ki), o[:, 0])
            cc = cc.index_put((bi, ti, ki), o[:, 1])
            aux_c = (self.aux_c(z), tgt_c[bi, ti, ki])
        qc, cc = qc.reshape(B, T * K), cc.reshape(B, T * K)
        tm = task_mask.float().unsqueeze(-1)
        pool = (ht * tm).sum(1) / tm.sum(1).clamp(min=1.0)
        es = self.enc_s(spec)
        S = spec.shape[1]
        zs = torch.cat([hd[:, None].expand(B, S, -1), ho[:, None].expand(B, S, -1), pool[:, None].expand(B, S, -1), es], -1)
        os_ = self.q_s(zs)
        self.last_aux = (aux_c, (self.aux_s(zs), tgt_s, spec[..., 0:4].sum(-1) > 0.5))
        return torch.cat([qc, os_[..., 0]], 1), torch.cat([cc, os_[..., 1]], 1)


def aux_loss(net, s):
    """Regression of the engineered lookahead columns (valid candidate edges + specials) from the last forward."""
    (ac, (asp, ts, sm)) = net.last_aux
    l = 0.0
    if ac is not None:
        l = l + F.smooth_l1_loss(ac[0], ac[1], beta=0.05)
    m = sm & s["mask"][:, -asp.shape[1]:]
    if m.any():
        l = l + F.smooth_l1_loss(asp[m], ts[m], beta=0.05)
    return l


class DistillLearner(DQNLearner):
    """DQNLearner with HierQNetDistill networks and the auxiliary distillation loss."""

    def __init__(self, hidden=128, layers=3, heads=4, lam_aux=1.0, **kw):
        import rl.hier.dqn as D
        orig = D.HierQNet
        D.HierQNet = HierQNetDistill
        try:
            super().__init__(hidden, layers, heads, **kw)
        finally:
            D.HierQNet = orig
        self.lam_aux = lam_aux

    def update(self, buf, B, beta, rng, demo_buf=None, demo_frac=0.0, lam_margin=None):
        lam = self.lam_margin if lam_margin is None else lam_margin
        parts = []
        nd = int(round(B * demo_frac)) if demo_buf is not None and demo_buf.size > 0 else 0
        if nd:
            parts.append((demo_buf, *demo_buf.sample(nd, beta, rng)))
        if buf.size > 0 and B - nd > 0:
            parts.append((buf, *buf.sample(B - nd, beta, rng)))
        stats = {}
        batches = [b.batch(i, self.device) for b, i, _ in parts]
        cat = lambda xs: torch.cat(xs, 0)  # noqa: E731
        s = {k: cat([b[0][k] for b in batches]) for k in batches[0][0]}
        ns = {k: cat([b[6][k] for b in batches]) for k in batches[0][6]}
        a, rn, gn, has_next, demo = (cat([b[j] for b in batches]) for j in range(1, 6))
        col = cat([b[7] for b in batches])
        w = torch.as_tensor(np.concatenate([p[2] for p in parts])).to(self.device)
        q_all, c_all = qc_forward(self.net, s)
        l_aux = aux_loss(self.net, s)
        q = q_all.gather(1, a[:, None]).squeeze(1)
        with torch.no_grad():
            qn, cn = qc_forward(self.net, ns)
            qn_online = masked_q(qn, ns["mask"]) if self.safe_thr is None else safe_q(qn, cn, ns["mask"], self.safe_thr)
            an = qn_online.argmax(1, keepdim=True)
            qn = q_forward(self.tgt, ns).gather(1, an).squeeze(1)
            y = rn + gn * qn * has_next.float()
        td = y - q
        loss_td = (w * F.smooth_l1_loss(q, y, reduction="none")).mean()
        loss = loss_td + self.lam_aux * l_aux
        stats["aux"] = float(l_aux.detach()) if torch.is_tensor(l_aux) else float(l_aux)
        if self.lam_safe > 0:
            logit = c_all.gather(1, a[:, None]).squeeze(1)
            l_safe = F.binary_cross_entropy_with_logits(logit, col)
            loss = loss + self.lam_safe * l_safe
            stats["safe_bce"] = float(l_safe.detach())
            stats["col_rate"] = float(col.mean())
        if self.cql > 0:
            lse = torch.logsumexp(masked_q(q_all, s["mask"]), 1)
            l_cql = (lse - q).mean()
            loss = loss + self.cql * l_cql
            stats["cql"] = float(l_cql.detach())
        if lam > 0 and demo.any():
            qm = masked_q(q_all, s["mask"]) + self.margin
            qm = qm.scatter(1, a[:, None], q[:, None])
            lm = (qm.max(1).values - q)[demo]
            loss = loss + lam * lm.mean()
            stats["margin"] = float(lm.mean().detach())
        self.opt.zero_grad()
        loss.backward()
        gnorm = torch.nn.utils.clip_grad_norm_(self.net.parameters(), self.grad_clip)
        self.opt.step()
        self.updates += 1
        with torch.no_grad():
            for pe, pn in zip(self.ema.parameters(), self.net.parameters()):
                pe.mul_(self.ema_decay).add_(pn.detach(), alpha=1 - self.ema_decay)
        if self.updates % self.target_every == 0:
            self.tgt.load_state_dict(self.net.state_dict())
        tdn = td.detach().cpu().numpy()
        o = 0
        for b, i, _ in parts:
            b.update(i, tdn[o:o + len(i)])
            o += len(i)
        stats.update(loss=float(loss_td.detach()), q=float(q.detach().mean()), td=float(np.abs(tdn).mean()),
                     gnorm=float(gnorm))
        return stats


def install_distill(lam_aux=1.0):
    """Patch the learner / network classes used by train_hier.py and evaluate_hier.py (fork-inherited)."""
    import rl.hier.dqn as D
    import rl.hier.model as M
    _Orig = M.HierQNet

    class AnyNet:
        """HierQNet(**cfg) factory: cfg with distill=True -> HierQNetDistill."""
        def __new__(cls, *a, distill=False, **kw):
            return HierQNetDistill(*a, **kw) if distill else _Orig(*a, **kw)
    M.HierQNet = AnyNet

    class L(DistillLearner):
        def __init__(self, *a, **kw):
            super().__init__(*a, lam_aux=lam_aux, **kw)
    D.DQNLearner = L
