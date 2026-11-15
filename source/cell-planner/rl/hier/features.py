"""Decision-point graph features for the hierarchical scheduler (Track H).

Graph per decision (padded, fixed shapes, world frame centred on the cell, metres / 2):
  robot  (2, FR)       slot 0 = deciding arm, slot 1 = the other arm
  task   (T_MAX, FT)   one node per weld of the episode (+ mask)
  cand   (T_MAX, K, FC) candidate edges deciding arm -> (task, IK branch) (+ mask = action mask)
  spec   (4, FS)        wait 0.5 / 1 / 2 s, HOME (+ mask)

Lookahead terms on each candidate (and on the specials), computed from the executor's nominal
primitive and the other arm's *committed* primitive (time-aligned at 20 Hz, stride ``stride``; after
its primitive the other arm is assumed to rest, which is exactly what it does until its next decision):
  * min umarell sphere clearance along the candidate (and at the final rest pair), time of the minimum;
  * MuJoCo hull self / floor contact along the direct transit (every waypoint and midpoint).
These are *observations*; nothing is masked or filtered. The policy has to learn which values are
safe (the env's verdict is the ground truth, the features are approximations: stride, future
decisions of the other arm are unknown).
"""
from __future__ import annotations

from collections import OrderedDict

import mujoco
import numpy as np

from env.cell import ROBOT_IDS
from env.kinematics import FLOOR_Z_MIN
from env.weld_env import T_MAX

from .executor import PHASE_NAMES, SPECIALS
from .instances import K_BRANCH

CENTER = np.array([-0.557, -1.138, 1.0])
SCALE = 0.5
FR, FT, FC, FS = 64, 16, 29, 11
WARN = 0.20


def _clr_feat(c):
    return np.clip((np.asarray(c, float) - WARN) / 0.5, -1.0, 2.0)


class FeatureBuilder:
    def __init__(self, henv, stride=2, cache_size=200_000):
        self.h = henv
        e = henv.env
        self.e = e
        self.sm = e.spheres
        self.stride = stride
        self.q_lo, self.q_hi = e.q_lo, e.q_hi
        self.q_mid, self.q_half = 0.5 * (e.q_lo + e.q_hi), 0.5 * (e.q_hi - e.q_lo)
        self.data = mujoco.MjData(e.model)
        self.cache = OrderedDict()
        self.cache_size = cache_size
        self.floor_idx = [np.asarray(self.sm.sphere_groups(r)["floor_idx"]) for r in ROBOT_IDS]
        if e.has_boxes:   # merged AABBs (env.weld_env.merge_boxes, each contains its member boxes) for a prefilter
            from env.weld_env import merge_boxes
            self.merged_c, self.merged_h = merge_boxes(*e.cell.boxes)

    # ------------------------------------------------------------------ geometry helpers
    def kp(self, i, q):
        return (self.sm.keypoints(ROBOT_IDS[i], np.asarray(q, float)) - CENTER) / SCALE

    def tcp(self, i, q):
        return self.sm.keypoints(ROBOT_IDS[i], np.asarray(q, float))[..., -1, :]

    def self_hit(self, i, q0, path):
        """(hit, depth) of hull self / floor contacts of arm i along q0 -> path (waypoints + midpoints); in a
        fixture cell ``hit`` also flags umarell's box rule along the transit."""
        if len(path) == 0:
            return 0.0, 0.0
        key = (i, np.round(q0, 4).tobytes(), np.round(path[-1], 4).tobytes(), len(path))
        v = self.cache.get(key)
        if v is not None:
            self.cache.move_to_end(key)
            return v
        e, d = self.e, self.data
        a = np.vstack([np.asarray(q0, float)[None], path])
        pts = np.concatenate([a[1:], 0.5 * (a[1:] + a[:-1])])
        o = 1 - i
        d.qpos[e.qadr[o]] = e.home[o]
        depth = 0.0
        own = e.geom_owner
        for x in pts:
            d.qpos[e.qadr[i]] = x
            mujoco.mj_kinematics(e.model, d)
            mujoco.mj_collision(e.model, d)
            n = d.ncon
            if n == 0:
                continue
            dist = d.contact.dist[:n]
            g = d.contact.geom[:n]
            oa, ob = own[g[:, 0]], own[g[:, 1]]
            bad = ((oa == i) & (ob == i)) | ((oa == i) & (ob == -2)) | ((oa == -2) & (ob == i))
            bad &= dist < -e.cfg.contact_tol
            if bad.any():
                depth = max(depth, float(-dist[bad].min()))
        box_hit = self.e.has_boxes and self.fixture_hit(i, pts)
        v = (1.0 if (depth > 0 or box_hit) else 0.0, depth)
        self.cache[key] = v
        if len(self.cache) > self.cache_size:
            self.cache.popitem(last=False)
        return v

    def fixture_hit(self, i, pts):
        """umarell's box rule (SphereModel.obstacle_terms) for arm i at the configurations pts (fixture cells).
        Exact; configurations clear of the merged AABBs (supersets of the boxes) skip the full 737-box test."""
        sm, rid = self.sm, ROBOT_IDS[i]
        arm = sm._arm[rid]
        S = sm.spheres(rid, np.asarray(pts, float))[:, arm.box_idx_u, :]           # (n, M, 3)
        r = arm.obst_radii[arm.box_idx_u] + float(np.max(sm.box_margin))
        C, H = self.merged_c, self.merged_h                                          # (K,3)
        dd = np.maximum(np.abs(S[:, :, None, :] - C[None, None]) - H[None, None], 0.0)
        dist = np.sqrt((dd ** 2).sum(-1)) - r[None, :, None]
        near = (dist <= 0).any(axis=(1, 2))
        if not near.any():
            return False
        return bool((sm.obstacle_terms(rid, np.asarray(pts, float)[near])["box"] < 0).any())

    def clearance_batch(self, i, paths, other_future):
        """Min umarell clearance of each candidate path (arm i) vs the other arm's future.

        paths: list of (N_c, 6) waypoint arrays (time +1, +2, ...); other_future (N_o, 6).
        Returns arrays (n_c,) of min clearance, normalised time of min, final rest clearance, floor margin."""
        No = len(other_future)
        qi, qo, seg = [], [], []
        for c, P in enumerate(paths):
            L = max(len(P), No)
            js = np.unique(np.concatenate([np.arange(0, L, self.stride), [L - 1]]))
            qi.append(P[np.minimum(js, len(P) - 1)])
            qo.append(other_future[np.minimum(js, No - 1)])
            seg.append(np.full(len(js), c))
        qi, qo, seg = np.concatenate(qi), np.concatenate(qo), np.concatenate(seg)
        Si = self.sm.spheres(ROBOT_IDS[i], qi)
        So = self.sm.spheres(ROBOT_IDS[1 - i], qo)
        clr = self.sm.pair_min_from_spheres(Si, So)[0] if i == 0 else self.sm.pair_min_from_spheres(So, Si)[0]
        fl = Si[:, self.floor_idx[i], 2].min(-1) - FLOOR_Z_MIN
        n = len(paths)
        cmin, tmin, cend, fmin = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
        bounds = np.flatnonzero(np.diff(seg)) + 1
        for c, (a, b) in enumerate(zip(np.r_[0, bounds], np.r_[bounds, len(seg)])):
            cc = clr[a:b]
            k = int(np.argmin(cc))
            cmin[c], tmin[c], cend[c] = cc[k], k / max(1, b - a - 1), cc[-1]
            fmin[c] = fl[a:b].min()
        return cmin, tmin, cend, fmin

    def lookahead(self, i, q_i, prs, fut_o):
        """Lookahead terms of every candidate primitive ``prs`` of arm i (at q_i) against the other arm's committed
        future ``fut_o``: arrays (cmin, tmin, cend, fmin, hit, depth). This is the engineered, exact model-based check;
        subclasses (rl/hier/la_features.py, phase 3 track G) may replace it with a learned predictor."""
        cmin, tmin, cend, fmin = self.clearance_batch(i, [p.wps for p in prs], fut_o)
        hits, depths = np.zeros(len(prs)), np.zeros(len(prs))
        for k, p in enumerate(prs):
            n_tr = p.info.get("n_transit", len(p.wps)) if p.kind != "wait" else 0
            hits[k], depths[k] = self.self_hit(i, q_i, p.wps[:n_tr]) if n_tr > 0 else (0.0, 0.0)
        return cmin, tmin, cend, fmin, hits, depths

    # ------------------------------------------------------------------ build
    def build(self):
        h, e = self.h, self.e
        i, o = h.deciding, 1 - h.deciding
        q = e.q
        prims = h.prims
        fut_o = prims[o].future(q[o]) if prims[o] is not None else q[o][None]
        n_open = int(((h.status[:h.n] == 0)).sum())
        t_el = h.t * h.dt

        robot = np.zeros((2, FR), np.float32)
        for slot, k in enumerate((i, o)):
            p = prims[k]
            f = [np.eye(2)[k], [1.0 if slot == 0 else 0.0], (q[k] - self.q_mid[k]) / self.q_half[k],
                 self.kp(k, q[k]).ravel()]
            kind = np.zeros(4)
            ph = np.zeros(6)
            rem = 0.0
            if p is None or p.done:
                kind[0] = 1
                end = q[k]
            else:
                kind[{"weld": 1, "wait": 2, "home": 3}[p.kind]] = 1
                ph[int(p.phase[min(p.i, len(p.phase) - 1)])] = 1
                rem = p.remaining * h.dt
                end = p.wps[-1]
            tgt, has = np.zeros(3), 0.0
            if p is not None and not p.done and p.kind == "weld":
                tgt, has = (h.task_pos[p.task] - CENTER) / SCALE, 1.0
            f += [kind, ph, [rem / 10.0], self.kp(k, end).ravel(), tgt, [has],
                  [float((h.done_by[:h.n] == k).sum()) / 15.0, float(h.at_home(k))], [(h.max_time - t_el) / 60.0 if h.cfg.time_feature == "left" else t_el / 60.0, n_open / 15.0]]
            robot[slot] = np.concatenate([np.ravel(x) for x in f])

        task = np.zeros((T_MAX, FT), np.float32)
        tmask = np.zeros(T_MAX, bool)
        tcp_i = self.tcp(i, q[i])
        end_o = fut_o[-1]
        tcp_o_end = self.tcp(o, end_o)
        ri, ro = ROBOT_IDS[i], ROBOT_IDS[o]
        for t in range(h.n):
            tmask[t] = True
            nb_i, nb_o = len(h.branches[t][ri]), len(h.branches[t][ro])
            app = e._task_app[t]
            task[t] = np.concatenate([(h.task_pos[t] - CENTER) / SCALE, app,
                                      [h.status[t] == 0, h.status[t] == 1, h.claim[t] == i, h.claim[t] == o,
                                       nb_i / K_BRANCH, nb_o / K_BRANCH, nb_i > 0 and nb_o == 0, nb_o > 0 and nb_i == 0,
                                       np.linalg.norm(h.task_pos[t] - tcp_i) / 2.0,
                                       np.linalg.norm(h.task_pos[t] - tcp_o_end) / 2.0]])

        mask = h.action_mask(i)
        cand = np.zeros((T_MAX, K_BRANCH, FC), np.float32)
        spec = np.zeros((len(SPECIALS), FS), np.float32)
        # primitives for every valid action
        acts, prs = [], []
        for a in np.flatnonzero(mask):
            p = h.make_primitive(i, a)
            acts.append(int(a)); prs.append(p)
        cmin, tmin, cend, fmin, hits, depths = self.lookahead(i, q[i], prs, fut_o)
        for a, p, c0, t0, c1, f0, hit, depth in zip(acts, prs, cmin, tmin, cend, fmin, hits, depths):
            n_tr = p.info.get("n_transit", len(p.wps)) if p.kind != "wait" else 0
            dur = len(p.wps) * h.dt
            lk = [_clr_feat(c0), float(c0 <= WARN), float(c0 <= WARN + 0.05), t0, _clr_feat(c1),
                  hit, min(depth / 0.05, 2.0), float(f0 < 0)]
            if p.kind == "weld":
                t, b = divmod(a, K_BRANCH)
                br = h.branch(i, t, b)
                f = np.concatenate([[1.0], (br.q_pre - self.q_mid[i]) / self.q_half[i], (br.q_pre - q[i]) / np.pi,
                                    [n_tr * h.dt / 5.0, dur / 10.0], lk,
                                    [np.clip(br.self_dist / 0.05, -1, 2), br.standoff / 0.1, br.cost_home / 10.0],
                                    (self.tcp(i, br.q_pre) - CENTER) / SCALE])
                cand[t, b] = f
            else:
                j = a - T_MAX * K_BRANCH
                spec[j] = np.concatenate([np.eye(len(SPECIALS))[j], [dur / 2.0], lk[:3], lk[4:7]])
        return dict(robot=robot, task=task, task_mask=tmask, cand=cand, spec=spec, mask=mask)


OBS_KEYS = ("robot", "task", "task_mask", "cand", "spec", "mask")

# lookahead columns (model-based forward check against the other arm's committed primitive + self/floor/fixture):
# cand 15..22 = [clr, clr<=0.20, clr<=0.25, t_min, end clr, hit, depth, floor]; spec 5..10 = [clr, <=0.20, <=0.25,
# end clr, hit, depth]
LOOK_CAND = slice(15, 23)
LOOK_SPEC = slice(5, 11)


def blind(obs):
    """Copy of obs with the lookahead columns zeroed (ablation: policy without lookahead features)."""
    o = dict(obs)
    o["cand"] = obs["cand"].copy(); o["spec"] = obs["spec"].copy()
    o["cand"][..., LOOK_CAND] = 0.0
    o["spec"][..., LOOK_SPEC] = 0.0
    return o
