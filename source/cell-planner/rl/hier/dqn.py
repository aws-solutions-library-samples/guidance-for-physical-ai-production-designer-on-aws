"""Double DQN with n-step semi-MDP returns, prioritized replay and a DQfD margin loss (Track H).

Semi-MDP targets: a transition from decision j carries R_j = sum_k gamma^(t_k - t_j) r_k (discounted
inside the primitive, ``hier_env``) and its duration tau_j (s). The n-step target is
  y_j = sum_{m<n} gamma^(T_{j+m} - T_j) R_{j+m} + gamma^(T_{j+n} - T_j) * Q_tgt(s_{j+n}, argmax_a Q(s_{j+n}, a))
(the bootstrap term is dropped when the episode ends within n decisions).
"""
from __future__ import annotations

import numpy as np
import torch
import torch.nn.functional as F

from env.weld_env import T_MAX

from .features import FC, FR, FS, FT
from .hier_env import N_ACT
from .instances import K_BRANCH
from .model import HierQNet, masked_q, safe_q

N_SPEC = 4


def nstep_episode(rewards, taus, gamma, n):
    """(Rn, gamma_n, next offset or -1) per decision of one finished episode (all ends terminal)."""
    L = len(rewards)
    T = np.concatenate([[0.0], np.cumsum(taus)])          # decision times relative to start
    Rn, Gn, nxt = np.zeros(L), np.zeros(L), np.full(L, -1)
    for j in range(L):
        s = 0.0
        for m in range(n):
            if j + m >= L:
                break
            s += gamma ** (T[j + m] - T[j]) * rewards[j + m]
        Rn[j] = s
        if j + n < L:
            Gn[j] = gamma ** (T[j + n] - T[j])
            nxt[j] = j + n
    return Rn, Gn, nxt


class Replay:
    def __init__(self, capacity, alpha=0.6, eps=1e-3):
        C = int(capacity)
        self.C = C
        f16 = np.float16
        self.robot = np.zeros((C, 2, FR), f16)
        self.task = np.zeros((C, T_MAX, FT), f16)
        self.task_mask = np.zeros((C, T_MAX), bool)
        self.cand = np.zeros((C, T_MAX, K_BRANCH, FC), f16)
        self.spec = np.zeros((C, N_SPEC, FS), f16)
        self.mask = np.zeros((C, N_ACT), bool)
        self.action = np.zeros(C, np.int64)
        self.rn = np.zeros(C, np.float32)
        self.gn = np.zeros(C, np.float32)
        self.nxt = np.full(C, -1, np.int64)
        self.demo = np.zeros(C, bool)
        self.col = np.zeros(C, bool)             # collision while this decision's primitive executed
        self.prio = np.zeros(C, np.float64)
        self.alpha, self.eps = alpha, eps
        self.ptr, self.size, self.max_prio = 0, 0, 1.0

    def add_episode(self, obs_list, actions, rewards, taus, gamma, n, demo=False, blame=()):
        L = len(actions)
        if L == 0:
            return
        if L > self.C:
            raise ValueError("episode longer than the buffer")
        Rn, Gn, nx = nstep_episode(np.asarray(rewards, float), np.asarray(taus, float), gamma, n)
        idx = (self.ptr + np.arange(L)) % self.C
        for k, o in zip(idx, obs_list):
            self.robot[k], self.task[k], self.task_mask[k] = o["robot"], o["task"], o["task_mask"]
            self.cand[k], self.spec[k], self.mask[k] = o["cand"], o["spec"], o["mask"]
        self.action[idx] = actions
        self.rn[idx], self.gn[idx] = Rn, Gn
        self.nxt[idx] = np.where(nx >= 0, idx[np.clip(nx, 0, L - 1)], -1)
        self.demo[idx] = demo
        self.col[idx] = False
        for j in blame:
            if 0 <= j < L:
                self.col[idx[j]] = True
        self.prio[idx] = self.max_prio
        self.ptr = int((self.ptr + L) % self.C)
        self.size = int(min(self.C, self.size + L))

    def sample(self, B, beta, rng):
        p = self.prio[:self.size] ** self.alpha
        p /= p.sum()
        idx = rng.choice(self.size, B, p=p)
        w = (self.size * p[idx]) ** (-beta)
        return idx, (w / w.max()).astype(np.float32)

    def update(self, idx, td):
        pr = np.abs(td) + self.eps
        self.prio[idx] = pr
        self.max_prio = max(self.max_prio, float(pr.max()))

    def batch(self, idx, device="cpu"):
        def T(x, dt=torch.float32):
            return torch.as_tensor(np.asarray(x)).to(device=device, dtype=dt)
        s = dict(robot=T(self.robot[idx]), task=T(self.task[idx]), task_mask=T(self.task_mask[idx], torch.bool),
                 cand=T(self.cand[idx]), spec=T(self.spec[idx]), mask=T(self.mask[idx], torch.bool))
        nx = self.nxt[idx]
        ni = np.where(nx >= 0, nx, idx)
        ns = dict(robot=T(self.robot[ni]), task=T(self.task[ni]), task_mask=T(self.task_mask[ni], torch.bool),
                  cand=T(self.cand[ni]), spec=T(self.spec[ni]), mask=T(self.mask[ni], torch.bool))
        return s, T(self.action[idx], torch.int64), T(self.rn[idx]), T(self.gn[idx]), T(nx >= 0), \
            T(self.demo[idx], torch.bool), ns, T(self.col[idx])

    def state_dict(self):
        keys = ("robot", "task", "task_mask", "cand", "spec", "mask", "action", "rn", "gn", "nxt", "demo", "col", "prio")
        d = {k: getattr(self, k)[:self.size] for k in keys}
        d.update(ptr=self.ptr, size=self.size, max_prio=self.max_prio)
        return d

    def load_state_dict(self, d):
        n = int(d["size"])
        for k in ("robot", "task", "task_mask", "cand", "spec", "mask", "action", "rn", "gn", "nxt", "demo", "col", "prio"):
            if k in d:
                getattr(self, k)[:n] = d[k][:n]
        self.ptr, self.size, self.max_prio = int(d["ptr"]), n, float(d["max_prio"])


def q_forward(net, s):
    return net(s["robot"], s["task"], s["task_mask"], s["cand"], s["spec"])


def qc_forward(net, s):
    return net.forward_both(s["robot"], s["task"], s["task_mask"], s["cand"], s["spec"])


class DQNLearner:
    def __init__(self, hidden=128, layers=3, heads=4, lr=2e-4, gamma=0.99, n_step=3, target_every=1500,
                 margin=0.8, lam_margin=0.5, grad_clip=10.0, device="cpu", lam_safe=1.0, cql=0.0, safe_thr=None):
        self.net = HierQNet(hidden, layers, heads).to(device)
        self.tgt = HierQNet(hidden, layers, heads).to(device)
        self.tgt.load_state_dict(self.net.state_dict())
        self.ema = HierQNet(hidden, layers, heads).to(device)     # Polyak average of the online net (evaluation)
        self.ema.load_state_dict(self.net.state_dict())
        self.ema_decay = 0.999
        self.opt = torch.optim.Adam(self.net.parameters(), lr=lr)
        self.gamma, self.n_step, self.target_every = gamma, n_step, target_every
        self.margin, self.lam_margin, self.grad_clip = margin, lam_margin, grad_clip
        self.device = device
        self.lam_safe, self.cql, self.safe_thr = lam_safe, cql, safe_thr
        self.updates = 0

    def update(self, buf: Replay, B, beta, rng, demo_buf: Replay | None = None, demo_frac=0.0, lam_margin=None):
        lam = self.lam_margin if lam_margin is None else lam_margin
        parts = []
        nd = int(round(B * demo_frac)) if demo_buf is not None and demo_buf.size > 0 else 0
        if nd:
            parts.append((demo_buf, *demo_buf.sample(nd, beta, rng)))
        if buf.size > 0 and B - nd > 0:
            parts.append((buf, *buf.sample(B - nd, beta, rng)))
        stats = {}
        batches = [b.batch(i, self.device) for b, i, _ in parts]
        cat = lambda xs: torch.cat(xs, 0)
        s = {k: cat([b[0][k] for b in batches]) for k in batches[0][0]}
        ns = {k: cat([b[6][k] for b in batches]) for k in batches[0][6]}
        a, rn, gn, has_next, demo = (cat([b[j] for b in batches]) for j in range(1, 6))
        col = cat([b[7] for b in batches])
        w = torch.as_tensor(np.concatenate([p[2] for p in parts])).to(self.device)
        q_all, c_all = qc_forward(self.net, s)
        q = q_all.gather(1, a[:, None]).squeeze(1)
        with torch.no_grad():
            qn, cn = qc_forward(self.net, ns)
            qn_online = masked_q(qn, ns["mask"]) if self.safe_thr is None else safe_q(qn, cn, ns["mask"], self.safe_thr)
            an = qn_online.argmax(1, keepdim=True)
            qn = q_forward(self.tgt, ns).gather(1, an).squeeze(1)
            y = rn + gn * qn * has_next.float()
        td = y - q
        loss_td = (w * F.smooth_l1_loss(q, y, reduction="none")).mean()
        loss = loss_td
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
            qm = qm.scatter(1, a[:, None], q[:, None])           # l(a_E, a_E) = 0
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
        stats.update(loss=float(loss_td.detach()), q=float(q.detach().mean()), td=float(np.abs(tdn).mean()), gnorm=float(gnorm))
        return stats

    def state_dict(self):
        return dict(net=self.net.state_dict(), tgt=self.tgt.state_dict(), opt=self.opt.state_dict(), updates=self.updates,
                    cfg=self.net.cfg, ema=self.ema.state_dict())

    def load_state_dict(self, d):
        self.net.load_state_dict(d["net"]); self.tgt.load_state_dict(d["tgt"])
        self.opt.load_state_dict(d["opt"]); self.updates = int(d["updates"])
        self.ema.load_state_dict(d.get("ema", d["net"]))
