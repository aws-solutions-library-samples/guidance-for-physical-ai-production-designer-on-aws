"""Greedy-safe scheduler in the hierarchical action space (Track H): demos + baseline.

Reads only the decision features (rl/hier/features.py). Among weld candidates whose lookahead
features say "predicted clearance > 0.25 m and no hull self/floor contact on the transit", it takes
the one with the shortest transit (bonus for welds only this arm can reach, penalty for welds the
other arm is closer to). Otherwise: wait 0.5 s, or go HOME when the other arm is idle/waiting and
HOME is predicted safe (deadlock breaker).
"""
from __future__ import annotations

import numpy as np

from env.weld_env import T_MAX

from .instances import K_BRANCH

# candidate feature columns (features.FeatureBuilder.build)
C_VALID, C_TRANSIT, C_DUR, C_CLR, C_WARN, C_WARN5, C_TMIN, C_CEND, C_HIT, C_DEPTH, C_FLOOR = 0, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22
S_DUR, S_CLR, S_WARN, S_WARN5, S_CEND, S_HIT = 4, 5, 6, 7, 8, 9
# task columns
T_EXCL_ME, T_EXCL_OTHER, T_DIST_ME, T_DIST_OTHER = 10, 11, 12, 13
NW = T_MAX * K_BRANCH


def safe_mask(obs, margin_col=C_WARN5):
    c = obs["cand"]
    ok = (c[..., C_VALID] > 0.5) & (c[..., margin_col] < 0.5) & (c[..., C_HIT] < 0.5) & (c[..., C_FLOOR] < 0.5)
    return ok.reshape(-1) & obs["mask"][:NW]


def heuristic_action(obs, rng=None, temp=0.0, excl_bonus=1.5, other_pen=0.7):
    c, t, m = obs["cand"], obs["task"], obs["mask"]
    ok = safe_mask(obs)
    if ok.any():
        tr = c[..., C_TRANSIT] * 5.0                                   # s
        score = tr - excl_bonus * t[:, None, T_EXCL_ME] \
            + other_pen * (t[:, None, T_DIST_OTHER] < t[:, None, T_DIST_ME]) * (t[:, None, T_EXCL_ME] < 0.5)
        s = np.where(ok, score.reshape(-1), np.inf)
        if rng is not None and temp > 0:
            s = s + rng.gumbel(size=s.shape) * temp
        return int(np.argmin(s))
    sp = obs["spec"]
    other_idle = obs["robot"][1, 27] > 0.5 or obs["robot"][1, 29] > 0.5      # kind idle / wait
    home_ok = m[NW + 3] and sp[3, S_WARN5] < 0.5 and sp[3, S_HIT] < 0.5
    if (other_idle or not m[NW]) and home_ok and (rng is None or rng.uniform() < 0.5 or not m[NW]):
        return NW + 3
    if m[NW]:
        return NW + 0
    # waiting not offered (the other arm waits) and HOME not predicted safe: the weld candidate with the
    # largest predicted clearance and no predicted self contact, else any valid action
    ok = (c[..., C_VALID] > 0.5).reshape(-1) & (c[..., C_HIT] < 0.5).reshape(-1) & m[:NW]
    if ok.any():
        s = np.where(ok, c[..., C_CLR].reshape(-1), -np.inf)
        return int(np.argmax(s))
    return int(np.flatnonzero(m)[0])
