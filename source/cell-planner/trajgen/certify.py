"""Continuous-time certification of the robot-robot hull buffer between sampled checks (round-4 finding: the 3 cm
buffer was checked at 10 ms samples only, and the true minimum between samples can be a few mm lower).

Both robots follow piecewise-linear joint paths (the executor interpolates its samples linearly; the planners'
dense grids do the same). A joint step dq moves any hull point by at most sum_j |dq_j| R_j (``SceneModel
.sweep_radii``), so for robot a at fractional path index x_a in [c_a - h_a, c_a + h_a] and robot b at x_b in
[c_b - h_b, c_b + h_b] the signed hull distance is >= d(c_a, c_b) - h_a E_a - h_b E_b, E = the largest per-interval
displacement over the intervals the range touches (distance functions are 1-Lipschitz in point displacements).
``certify_rects`` runs a branch and bound over such rectangles: a rectangle is certified when that bound is >= thr,
is a violation when the centre distance itself is < thr, and is split in four otherwise (down to ``min_h``, below
which it counts as not certified). A rectangle of half width 0.5 around a grid pair (i, j) covers every continuous
time AND every continuous relative shift within half a grid step of it, so certifying the rectangles of all grid
pairs of a check (nominal + every shift of the skew window, extended by one step) certifies the continuous check.
"""
from __future__ import annotations

import numpy as np


def interval_disp(model, rid, P, kind: str = "hull") -> np.ndarray:
    """(len(P) - 1,) displacement bound of each interval of the piecewise-linear path P."""
    P = np.asarray(P, float).reshape(-1, 6)
    if len(P) < 2:
        return np.zeros(1)
    return np.abs(np.diff(P, axis=0)) @ model.sweep_radii(rid, kind)


def _at(P, x):
    x = np.clip(x, 0.0, len(P) - 1.0)
    i = np.minimum(np.floor(x).astype(int), len(P) - 2) if len(P) > 1 else np.zeros_like(x, int)
    if len(P) == 1:
        return np.repeat(P[:1], len(np.atleast_1d(x)), 0)
    f = (x - i)[:, None]
    return P[i] + f * (P[i + 1] - P[i])


def _emax(D, lo, hi):
    """max interval displacement over the intervals touched by [lo, hi] (vectorised)."""
    n = len(D)
    a = np.clip(np.floor(lo).astype(int), 0, n - 1)
    b = np.clip(np.ceil(hi).astype(int) - 1, 0, n - 1)
    out = np.empty(len(lo))
    for k in range(len(lo)):
        out[k] = D[a[k]:max(a[k], b[k]) + 1].max()
    return out


def certify_rects(model, ra, rb, PA, PB, ca, cb, thr: float, *, h0: float = 0.5, min_h: float = 1.0 / 1024,
                  DA=None, DB=None, d0=None, tol: float = 2.5e-4, max_evals: int = 400000,
                  minimize: bool = False, prune_above: float = np.inf, band: float | None = None,
                  diag: bool = False):
    """Branch and bound over rectangles centred at the grid pairs (ca[k], cb[k]) with half width h0 (path index
    units of PA / PB). ``diag``: robot b's index moves with robot a's (cb - ca fixed: a time interval at a fixed
    shift, split in two); else the full rectangle (every time and every relative shift within it, split in four).
    ``band``: rectangles whose every point has |x_a - x_b| > band are outside the check (pruned).
    ``minimize=False``: a rectangle is done once its bound is >= thr (fast certificate; lower_bound is then only
    >= thr). ``minimize=True``: done once its bound is >= max(thr, min(prune_above, best distance found) - tol), so
    lower_bound is the continuous minimum to within tol (reporting).
    Returns dict(ok, lower_bound (min certified bound over the leaves), violation (min distance < thr found, or
    None), best (min distance evaluated), n_evals, uncertified (leaves at min_h with bound < thr))."""
    DA = interval_disp(model, ra, PA) if DA is None else DA
    DB = interval_disp(model, rb, PB) if DB is None else DB
    ca, cb = np.asarray(ca, float), np.asarray(cb, float)
    h = np.full(len(ca), float(h0))
    banded = band is not None and not diag

    def evalpt(ca, cb):
        """Evaluation point inside the band (a rectangle straddling the band edge is evaluated at its centre moved
        along x_b into the band; its bound then covers |x_b - point| <= h + the move)."""
        if not banded:
            return cb, np.zeros(len(cb))
        eb = np.clip(cb, ca - band, ca + band)
        return eb, np.abs(eb - cb)

    eb, ex = evalpt(ca, cb)
    if d0 is None or ex.any():
        d = model.mj_pair_min_distance(ra, rb, _at(PA, ca), _at(PB, eb))
        n_evals = len(ca)
    else:
        d, n_evals = np.asarray(d0, float), 0
    lb_all, viol, unc = np.inf, None, 0
    best = np.inf
    while len(ca):
        best = min(best, float(d.min()))
        lip = h * _emax(DA, ca - h, ca + h) + (h + ex) * _emax(DB, np.minimum(cb, eb) - h, np.maximum(cb, eb) + h)
        bnd = d - lip
        bad = d < thr - 1e-12
        bnd = np.where(bad, d, bnd)
        if bad.any():
            v = float(d[bad].min())
            viol = v if viol is None else min(viol, v)
        target = max(thr, min(prune_above, best) - tol) if minimize else thr
        done = bad | (bnd >= target) | (lip <= tol)
        if banded:
            done |= np.abs(ca - cb) - 2 * h > band + 1e-9            # entirely outside the band: not checked
            inside = np.abs(ca - cb) - 2 * h <= band + 1e-9
        else:
            inside = np.ones(len(ca), bool)
        last = (~done) & ((h / 2 < min_h) | (n_evals > max_evals))
        fin = (done | last) & inside
        if fin.any():
            lb_all = min(lb_all, float(bnd[fin].min()))
        unc += int((last & inside & (bnd < thr) & ~bad).sum())
        go = (~done) & (~last)
        if not go.any():
            break
        ca, cb, h = ca[go], cb[go], h[go] / 2
        if diag:
            ca = np.concatenate([ca - h, ca + h])
            cb = np.concatenate([cb - h, cb + h])
            h = np.concatenate([h] * 2)
        else:
            ca = np.concatenate([ca - h, ca - h, ca + h, ca + h])
            cb = np.concatenate([cb - h, cb + h, cb - h, cb + h])
            h = np.concatenate([h] * 4)
        eb, ex = evalpt(ca, cb)
        d = model.mj_pair_min_distance(ra, rb, _at(PA, ca), _at(PB, eb))
        n_evals += len(ca)
    ok = viol is None and unc == 0
    return {"ok": bool(ok), "lower_bound": lb_all, "violation": viol, "best": best, "n_evals": int(n_evals),
            "uncertified": unc}


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# Continuous certification of EVERY geometric rule of the verdict (phase 3, track TGCERT, docs/p3_tgcert.md).
#
# Model of the executed motion: umarell's executor (and ``NTimeline.sample``) interpolates each leg's joint samples
# linearly in time, so every robot's joint path is piecewise linear between its KNOTS (leg dispatch times and leg
# sample times); a robot holds its pose between legs. Between two consecutive knots of the robots involved in a
# check, every robot moves on a straight joint segment q(s) = q_a + s (q_b - q_a).
#
# Displacement bound (provable, no sampling): a point x rigid in joint frame k moves, for a joint velocity qdot, at
# speed <= sum_j |qdot_j| dist(x, axis_j) <= sum_j |qdot_j| R_j(x), where R_j(x) = sum of the rigid joint-to-joint
# offsets from joint j's axis to frame k + |x in frame k| (triangle inequality; 0 for joints that do not move frame
# k). Along a straight joint segment its path length is therefore <= L(x) = sum_j |q_b,j - q_a,j| R_j(x)
# (``chain_radii``: per sphere centre, the TCP, and per hull frame).
#
# Interval bound: every checked quantity g is 1-Lipschitz in the positions of the points it depends on (distances
# between sphere centres, sphere-to-box distance, z, hull signed distances, the rule-W distance lower bound phi
# below), so on a node [t_a, t_b] inside one knot interval, with exact values g_a, g_b at its ends and the node's
# displacement budget L (sum of the L(x) of the points of the two bodies involved),
#     min_{t in [t_a, t_b]} g(t) >= (g_a + g_b - L) / 2
# (the two one-sided bounds g_a - L f and g_b - L (1 - f) meet there). For a component-wise quantity (min over
# sphere pairs / spheres) the bound is taken per component with its own L and then minimised. A node whose bound
# reaches the threshold is certified; a node whose end value violates the threshold is a violation; otherwise it is
# bisected (the midpoint is evaluated exactly) down to ``min_dt`` (then: not certified). The bound tends to the
# exact minimum as nodes shrink, so only true minima within the bisection resolution of the threshold stay
# uncertified (they are reported, and the check fails closed).
#
# Rule W: the violation set of a weld is V = {x : |x - p| < radius and min((x - p).a, (x - p).t) > tol} (open).
# phi(x) = min over welds of max(|x - p| - radius, tol - min((x - p).a, (x - p).t)) is a 1-Lipschitz lower bound of
# the distance from x to the union of the V (both terms are 1-Lipschitz, a and t unit vectors), and x is outside
# every V iff phi(x) >= 0. Certified: phi(TCP(t)) >= 0 for all t.
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════

def chain_radii(rm, frames, V) -> np.ndarray:
    """(m, 6) R_j for m points rigid in joint frames ``frames`` (index into RobotModel.joint_poses: 0 = base_link,
    k = after joint k-1; -1 = fixed in the world) with local coordinates ``V`` (m, 3) in those frames. Joint j's axis
    passes through the origin of frame j + 1 and moves frames k >= j + 1; the origin of frame k is at most
    sum_{i=j+1}^{k-1} |T_origin(i)| from it."""
    frames = np.asarray(frames, int).reshape(-1)
    V = np.asarray(V, float).reshape(-1, 3)
    off = np.linalg.norm(rm._To[:, :3, 3], axis=1)               # |p_{i+1} - p_i|, rigid
    cum = np.concatenate([[0.0], np.cumsum(off)])                  # cum[k] = sum off[:k]
    r = np.linalg.norm(V, axis=1)
    R = np.zeros((len(frames), 6))
    for j in range(6):
        m = frames >= j + 1
        R[m, j] = cum[frames[m]] - cum[j + 1] + r[m]
    return R


def sphere_points(rm, umarell: bool = False):
    """(frames (n,), local coordinates (n, 3)) of every sphere centre of ``RobotModel.spheres`` (or
    ``umarell_spheres``), in the same order."""
    nf = rm.n_frame
    fr = [-1, -1, -1] + [int(k) for k in rm.frame_joint[3:]] + [int(rm.tip_joint)]
    V = [np.zeros(3)] * (nf - 1) + [rm.T_tcp[:3, 3]]
    for j, t in zip(rm.fill_seg, rm.fill_t):
        d = rm._To[j][:3, 3]
        fr.append(int(j))
        V.append(d / np.linalg.norm(d) * t)
    if umarell:
        fr += [int(rm.um_joint)] * len(rm.um_tool)
        V += list(rm.um_tool)
    else:
        fr += [int(rm.tool_joint)] * rm.n_tool
        V += list(rm.tool_c)
    return np.array(fr, int), np.array(V, float).reshape(-1, 3)


def _knots(tl, rids, T):
    """Sorted knot times in [0, T] of the robots ``rids`` (leg dispatch and sample times, plus 0 and T)."""
    ks = [np.array([0.0, T])]
    for l in tl.legs:
        if l.robot in rids:
            ks.append(np.concatenate([[l.t0], np.asarray(l.times, float), [getattr(l, "t1", l.t0)]]))
    K = np.unique(np.round(np.concatenate(ks), 9))
    return K[(K >= 0.0) & (K <= T)]


def _bisect(lb_fn, ta, tb, thr, *, strict: bool, minimize: bool, tol: float, cap: float = np.inf,
            min_dt: float = 1e-6, max_nodes: int = 2_000_000, chunk: int = 2048):
    """Bisection over nodes [ta, tb] (inside knot intervals). ``lb_fn(ta, tb) -> (bound, vmin, arg)``: per node the
    interval lower bound, the exact minimum at its two ends and a component label. ``minimize``: also refine until
    every node's bound is >= max(thr, min(cap, best end value) - tol), so the certified lower bound is within ~tol
    of the continuous minimum. Returns a dict (see ``certify_timeline``)."""
    ta, tb = np.asarray(ta, float), np.asarray(tb, float)
    lb_all, best, n_nodes, unc, viol = np.inf, np.inf, 0, 0, []
    t_lb = arg_lb = t_best = None
    while len(ta):
        parts = [lb_fn(ta[s:s + chunk], tb[s:s + chunk]) for s in range(0, len(ta), chunk)]
        bnd = np.concatenate([p[0] for p in parts])
        vmin = np.concatenate([p[1] for p in parts])
        arg = np.concatenate([p[2] for p in parts])
        n_nodes += len(ta)
        k = int(np.argmin(vmin))
        if vmin[k] < best:
            best, t_best = float(vmin[k]), float(0.5 * (ta[k] + tb[k]))
        bad = (vmin <= thr) if strict else (vmin < thr - 1e-12)
        if bad.any():
            for i in np.flatnonzero(bad)[:5]:
                viol.append({"t_s": round(float(ta[i]), 4), "value": round(float(vmin[i]), 6), "at": str(arg[i])})
        target = max(thr, min(cap, best) - tol) if minimize else thr
        okb = (bnd >= target) & ((bnd > thr) if strict else True)
        done = bad | okb
        last = (~done) & ((tb - ta) / 2 < min_dt)
        if n_nodes > max_nodes:
            last = ~done
        fin = (done & ~bad) | last
        if fin.any():
            i = int(np.flatnonzero(fin)[np.argmin(bnd[fin])])
            if bnd[i] < lb_all:
                lb_all, t_lb, arg_lb = float(bnd[i]), float(ta[i]), str(arg[i])
        unc += int((last & ((bnd <= thr) if strict else (bnd < thr))).sum())
        go = ~(done | last)
        if not go.any():
            break
        a, b = ta[go], tb[go]
        m = 0.5 * (a + b)
        ta, tb = np.concatenate([a, m]), np.concatenate([m, b])
    if viol:
        lb_all = min(lb_all, min(v["value"] for v in viol))
    ok = not viol and unc == 0
    return {"ok": bool(ok), "certified_lower_bound": None if not np.isfinite(lb_all) else float(np.floor(lb_all * 1e6) / 1e6),
            "t_lower_bound_s": None if t_lb is None else round(t_lb, 4), "at": arg_lb,
            "evaluated_min": None if not np.isfinite(best) else round(best, 6),
            "t_evaluated_min_s": None if t_best is None else round(t_best, 4),
            "nodes": int(n_nodes), "uncertified": int(unc), "violations": viol[:10]}


class ContinuousCertifier:
    """Continuous-time certificates of the verdict's geometric rules for an N-robot timeline (module docstring above
    ``chain_radii``). One instance per SceneModel (the per-robot radii are cached)."""

    def __init__(self, model):
        from trajgen.mjscene import MjWorld

        self.m = model
        sc = model.scene
        self.R_sph, self.R_um, self.R_tcp, self.R_hull, self.hull_frames = {}, {}, {}, {}, {}
        self.R_hull_frame = {}
        for rid, rm in model.rm.items():
            fr, V = sphere_points(rm)
            self.R_sph[rid] = chain_radii(rm, fr, V)
            if model.umarell is not None and rm.um_tool is not None:
                fr, V = sphere_points(rm, umarell=True)
                self.R_um[rid] = chain_radii(rm, fr, V)
            self.R_tcp[rid] = chain_radii(rm, [rm.tip_joint], rm.T_tcp[None, :3, 3])[0]
            ks = sorted(rm.hulls)
            self.hull_frames[rid] = [(k, rm.hulls[k]) for k in ks]
            rh = chain_radii(rm, ks, [np.array([np.linalg.norm(rm.hulls[k], axis=1).max(), 0, 0]) for k in ks])
            self.R_hull_frame[rid] = rh                                    # (n_frames, 6)
            self.R_hull[rid] = rh.max(0)                                   # any hull point
        # box-hull distances need a detection band (MjWorld's own band is 0.1 mm, too small for a Lipschitz bound)
        self.box_band = float(max(sc.box_hull.min_distance, 0.0)) + 0.05
        self.mjb = MjWorld(sc, detect=self.box_band) if model.box_hull_on else None
        # electrode off-weld rule (round 2, opt-in): every point of the electrode hull(s) lies within its bounding
        # sphere (SceneModel.electrode_geometry), the TCP is rigid in the tip frame too; their chain radii, max per
        # joint, bound the displacement of everything the rule's gate function depends on
        self.R_elec = {}
        if getattr(model, "electrode_on", False):
            for rid, rm in model.rm.items():
                K, C, Rr = model.electrode_geometry(rid)
                re = chain_radii(rm, K, [np.array([np.linalg.norm(c) + r, 0, 0]) for c, r in zip(C, Rr)]).max(0)
                self.R_elec[rid] = np.maximum(re, self.R_tcp[rid])

    # ------------------------------------------------------------------ component values
    def _disp(self, Ra, Qa, Qb):
        """(n, m) displacement budgets of m points (radii Ra (m, 6)) between configurations Qa -> Qb (n, 6)."""
        return np.abs(Qb - Qa) @ Ra.T

    def _phi(self, X):
        m = self.m
        on = m.W_on
        if not on.any():
            return np.full(len(X), np.inf)
        rel = X[:, None] - m.W_P[None, on]
        d = np.linalg.norm(rel, axis=-1) - m.W_rad[None, on]
        dep = np.minimum((rel * m.W_A[None, on]).sum(-1), (rel * m.W_T[None, on]).sum(-1))
        return np.maximum(d, m.W_tol[None, on] - dep).min(1)

    def _box_per_sphere(self, rid, S):
        """(n, M) box-rule margins per checked sphere (lower bounds; exact below 5 cm, as SceneModel.box_margin)."""
        m = self.m
        idx = m.box_idx[rid]
        n, M = len(S), len(idx)
        P = S[:, idx].reshape(-1, 3)
        r = np.tile(m.box_r[rid], n)
        ex = np.tile(m.wp_exempt[rid], n)
        F, lo, step, pad = m._box_field()
        ii = np.rint((P - lo) / step).astype(int)
        inside = np.all((ii >= 0) & (ii < np.array(F.shape)), axis=1)
        f = np.full(len(P), pad)
        jj = ii[inside]
        f[inside] = F[jj[:, 0], jj[:, 1], jj[:, 2]]
        g = f - 0.5 * step * np.sqrt(3.0) - r - m.box_m.max()
        near = g < 0.05
        if near.any():
            g[near] = m._box_exact(P[near], r[near], ex[near])
        return g.reshape(n, M)

    # ------------------------------------------------------------------ the checks
    def certify_timeline(self, tl, *, t_end=None, umarell_parity: bool = True, minimize: bool = True,
                         tol: float = 0.001, self_hull: bool = True, min_dt: float = 1e-6,
                         single: bool = True) -> dict:
        """Continuous certificates of: arm-arm corrected-sphere gate (and umarell's 35-sphere gate with parity),
        robot-robot hull buffer, hull self-distance, floor spheres, floor hulls, box spheres, box hulls, rule W, joint
        limits, and (when the scene enables it) the electrode off-weld rule at the scorer threshold min_distance -
        score_tol (gate max(distance - threshold, radius - TCP weld distance) >= 0). Returns {checks: {name: ok},
        rows: {name: details}, all_ok, ...}."""
        import time as _time

        from trajgen.ntimeline import as_ntimeline

        t_wall = _time.time()
        m, sc = self.m, self.m.scene
        tl = as_ntimeline(tl)
        rids = list(m.rids)
        col = {r: list(tl.robot_ids).index(r) for r in rids}
        T = float(tl.t[-1]) if t_end is None else float(t_end)

        def sampler(rs):
            cols = [col[r] for r in rs]
            return lambda ts: tl.sample(np.asarray(ts, float))[:, cols]

        rows, checks = {}, {}

        def nodes_of(rs):
            K = _knots(tl, set(rs), T)
            return K[:-1], K[1:]

        # ---------------- single-arm rules, per robot on its own knots
        fl = sc.floor
        agg = {k: [] for k in ("floor_spheres", "floor_hull", "boxes", "box_hull", "rule_W", "self_hull",
                               "electrode_off_weld")}
        for rid in (rids if single else []):
            rm = m.rm[rid]
            smp = sampler([rid])
            ta, tb = nodes_of([rid])
            Rs = self.R_sph[rid]

            def ends(ta_, tb_):
                Q = smp(np.concatenate([ta_, tb_]))[:, 0]
                n = len(ta_)
                return Q[:n], Q[n:]

            if fl.enabled and fl.sphere_rule:
                idx = np.arange(rm.base_frame + 1, rm.n_link)
                if m.geo.is_scene:
                    idx = np.concatenate([idx, rm.n_link + np.arange(rm.n_tool)])
                rr, Ri = rm.sphere_r[idx], Rs[idx]
                z0 = fl.z + fl.margin

                def lb_floor(ta_, tb_, idx=idx, rr=rr, Ri=Ri, z0=z0):
                    Qa, Qb = ends(ta_, tb_)
                    va = rm.spheres(Qa)[:, idx, 2] - rr - z0
                    vb = rm.spheres(Qb)[:, idx, 2] - rr - z0
                    b = 0.5 * (va + vb - self._disp(Ri, Qa, Qb))
                    k = b.argmin(1)
                    return b.min(1), np.minimum(va.min(1), vb.min(1)), np.array([f"{rid} sphere {i}" for i in idx[k]])
                agg["floor_spheres"].append((rid, _bisect(lb_floor, ta, tb, 0.0, strict=False, minimize=minimize,
                                                          tol=tol, min_dt=min_dt)))
            if fl.enabled and fl.hull_rule:
                HF = [(k, V) for k, V in self.hull_frames[rid] if k != 0]
                RH = np.stack([self.R_hull_frame[rid][i] for i, (k, _) in enumerate(self.hull_frames[rid]) if k != 0])

                def zmins(Q, HF=HF):
                    Rq, pq = rm.joint_poses(Q)
                    return np.stack([(np.einsum("nj,vj->nv", Rq[:, k, 2, :], V) + pq[:, k, 2][:, None]).min(1)
                                     for k, V in HF], 1) - fl.z

                def lb_fhull(ta_, tb_, RH=RH, HF=HF):
                    Qa, Qb = ends(ta_, tb_)
                    va, vb = zmins(Qa), zmins(Qb)
                    b = 0.5 * (va + vb - self._disp(RH, Qa, Qb))
                    k = b.argmin(1)
                    return b.min(1), np.minimum(va.min(1), vb.min(1)), np.array([f"{rid} hull frame {HF[i][0]}" for i in k])
                agg["floor_hull"].append((rid, _bisect(lb_fhull, ta, tb, 0.0, strict=False, minimize=minimize, tol=tol,
                                                       min_dt=min_dt)))
            if len(m.box_c):
                Ri = Rs[m.box_idx[rid]]

                def lb_box(ta_, tb_, Ri=Ri):
                    Qa, Qb = ends(ta_, tb_)
                    va = self._box_per_sphere(rid, rm.spheres(Qa))
                    vb = self._box_per_sphere(rid, rm.spheres(Qb))
                    b = 0.5 * (va + vb - self._disp(Ri, Qa, Qb))
                    k = b.argmin(1)
                    return b.min(1), np.minimum(va.min(1), vb.min(1)), np.array([f"{rid} sphere {m.box_idx[rid][i]}" for i in k])
                agg["boxes"].append((rid, _bisect(lb_box, ta, tb, 0.0, strict=False, minimize=minimize, tol=tol,
                                                  min_dt=min_dt)))
            if self.mjb is not None:
                bmin = m.box_hull_min
                RHmax = self.R_hull[rid][None]

                def lb_bhull(ta_, tb_, RHmax=RHmax):
                    Qa, Qb = ends(ta_, tb_)
                    va, vb = self.mjb.box_distance(rid, Qa), self.mjb.box_distance(rid, Qb)
                    b = 0.5 * (va + vb - self._disp(RHmax, Qa, Qb)[:, 0])
                    return b, np.minimum(va, vb), np.full(len(b), rid)
                agg["box_hull"].append((rid, _bisect(lb_bhull, ta, tb, bmin, strict=False, minimize=minimize, tol=tol,
                                                     cap=self.box_band - 0.01, min_dt=min_dt)))
            if m.wedge_on:
                Rt = self.R_tcp[rid][None]

                def lb_w(ta_, tb_, Rt=Rt):
                    Qa, Qb = ends(ta_, tb_)
                    va, vb = self._phi(m.tcp(rid, Qa)[0]), self._phi(m.tcp(rid, Qb)[0])
                    b = 0.5 * (va + vb - self._disp(Rt, Qa, Qb)[:, 0])
                    return b, np.minimum(va, vb), np.full(len(b), rid)
                agg["rule_W"].append((rid, _bisect(lb_w, ta, tb, 0.0, strict=False, minimize=minimize, tol=tol,
                                                   cap=0.02, min_dt=min_dt)))
            if rid in self.R_elec:
                er = m.electrode_rule
                thr_e = er.min_distance - er.score_tol
                Re = self.R_elec[rid][None]
                nd = bool(getattr(m, "nodip", False))

                def gate_e(Q, thr_e=thr_e, er=er, nd=nd):
                    """max(electrode - workpiece distance - thr, radius - TCP distance to the nearest weld): >= 0
                    iff the rule holds (scorer threshold); both terms 1-Lipschitz in rigid tip-frame points.
                    No-dip rule (trajgen.nodip): the exemption term is min(radius - TCP weld distance, distance +
                    contact_depth), the static form without the contact-point condition (that one, and the
                    contact windows, are checked by the dense nodip_check)."""
                    d, _ = m.electrode_distance(rid, Q)
                    ex = er.radius - m.weld_distance(m.tcp(rid, Q)[0])
                    if nd:
                        ex = np.minimum(ex, d + er.contact_depth)
                    return np.maximum(d - thr_e, ex)

                def lb_e(ta_, tb_, Re=Re):
                    Qa, Qb = ends(ta_, tb_)
                    va, vb = gate_e(Qa), gate_e(Qb)
                    b = 0.5 * (va + vb - self._disp(Re, Qa, Qb)[:, 0])
                    return b, np.minimum(va, vb), np.full(len(b), rid)
                agg["electrode_off_weld"].append((rid, _bisect(lb_e, ta, tb, 0.0, strict=False, minimize=minimize,
                                                               tol=tol, cap=0.02, min_dt=min_dt)))
            if self_hull:
                smin = float(sc.collision.self_min_distance)
                RHmax = self.R_hull[rid][None]

                def lb_self(ta_, tb_, RHmax=RHmax):
                    Qa, Qb = ends(ta_, tb_)
                    va, vb = m.self_distance(rid, Qa), m.self_distance(rid, Qb)
                    # relative motion of two links of one arm: in the frame of the proximal link only the joints in
                    # between move the distal one, by at most the whole-arm budget (factor 1, not 2)
                    b = 0.5 * (va + vb - self._disp(RHmax, Qa, Qb)[:, 0])
                    return b, np.minimum(va, vb), np.full(len(b), rid)
                agg["self_hull"].append((rid, _bisect(lb_self, ta, tb, smin, strict=False, minimize=minimize, tol=tol,
                                                      cap=0.05, min_dt=min_dt)))
        for k, lst in agg.items():
            if not lst:
                continue
            rows[k] = {"per_robot": {r: v for r, v in lst},
                       "certified_lower_bound": min((v["certified_lower_bound"] for _, v in lst
                                                     if v["certified_lower_bound"] is not None), default=None)}
            checks[k] = all(v["ok"] for _, v in lst)
        if single:
            rows["joint_limits"] = {"note": "certified by convexity: between two knots every joint moves linearly, so "
                                            "a path whose knots are within the limits stays within them (knots checked "
                                            "by the sampled verdict)"}
        # ---------------- arm vs arm, every pair on the union of the two robots' knots
        pairs = [(i, j) for i in range(len(rids)) for j in range(i + 1, len(rids))]
        pr = sc.collision.pair_radius
        gate = sc.collision.arm_arm_gate
        um = m.umarell if umarell_parity else None
        sph, ums, hul = {}, {}, {}
        for i, j in pairs:
            ra, rb = rids[i], rids[j]
            smp = sampler([ra, rb])
            ta, tb = nodes_of([ra, rb])

            def ends2(ta_, tb_, smp=smp):
                Q = smp(np.concatenate([ta_, tb_]))
                n = len(ta_)
                return Q[:n], Q[n:]

            def lb_spheres(ta_, tb_, ra=ra, rb=rb, umarell=False):
                Qa, Qb = ends2(ta_, tb_)
                if umarell:
                    ia, ib = m.rm[ra].um_pair_idx, m.rm[rb].um_pair_idx
                    A0, A1 = m.rm[ra].umarell_spheres(Qa[:, 0])[:, ia], m.rm[ra].umarell_spheres(Qb[:, 0])[:, ia]
                    B0, B1 = m.rm[rb].umarell_spheres(Qa[:, 1])[:, ib], m.rm[rb].umarell_spheres(Qb[:, 1])[:, ib]
                    Ra_, Rb_, rad = self.R_um[ra][ia], self.R_um[rb][ib], um.pair_radius
                else:
                    ia, ib = m.rm[ra].pair_idx, m.rm[rb].pair_idx
                    A0, A1 = m.rm[ra].spheres(Qa[:, 0])[:, ia], m.rm[ra].spheres(Qb[:, 0])[:, ia]
                    B0, B1 = m.rm[rb].spheres(Qa[:, 1])[:, ib], m.rm[rb].spheres(Qb[:, 1])[:, ib]
                    Ra_, Rb_, rad = self.R_sph[ra][ia], self.R_sph[rb][ib], pr
                Da = self._disp(Ra_, Qa[:, 0], Qb[:, 0])
                Db = self._disp(Rb_, Qa[:, 1], Qb[:, 1])
                d0 = np.linalg.norm(A0[:, :, None] - B0[:, None], axis=-1)
                d1 = np.linalg.norm(A1[:, :, None] - B1[:, None], axis=-1)
                b = 0.5 * (d0 + d1 - Da[:, :, None] - Db[:, None, :]) - 2 * rad
                bf = b.reshape(len(b), -1)
                k = bf.argmin(1)
                v = np.minimum(d0.reshape(len(b), -1).min(1), d1.reshape(len(b), -1).min(1)) - 2 * rad
                nb = len(ib)
                lab = np.array([f"{ra} sphere {ia[x // nb]} / {rb} sphere {ib[x % nb]}" for x in k])
                return bf.min(1), v, lab
            key = f"{ra}|{rb}"
            sph[key] = _bisect(lb_spheres, ta, tb, gate, strict=True, minimize=minimize, tol=tol, min_dt=min_dt,
                               chunk=256)
            if um is not None and ra in self.R_um and rb in self.R_um:
                ums[key] = _bisect(lambda a_, b_, ra=ra, rb=rb: lb_spheres(a_, b_, ra, rb, umarell=True), ta, tb,
                                   um.warning_distance, strict=True, minimize=minimize, tol=tol, min_dt=min_dt,
                                   chunk=256)
            hmin = m.hull_min
            Ra_h, Rb_h = self.R_hull[ra][None], self.R_hull[rb][None]

            def lb_hull(ta_, tb_, ra=ra, rb=rb, Ra_h=Ra_h, Rb_h=Rb_h):
                Qa, Qb = ends2(ta_, tb_)
                va = m.mj_pair_min_distance(ra, rb, Qa[:, 0], Qa[:, 1])
                vb = m.mj_pair_min_distance(ra, rb, Qb[:, 0], Qb[:, 1])
                L = self._disp(Ra_h, Qa[:, 0], Qb[:, 0])[:, 0] + self._disp(Rb_h, Qa[:, 1], Qb[:, 1])[:, 0]
                return 0.5 * (va + vb - L), np.minimum(va, vb), np.full(len(va), key)
            hul[key] = _bisect(lb_hull, ta, tb, hmin, strict=False, minimize=minimize, tol=tol,
                               cap=m.mjw.pair_band - 0.01, min_dt=min_dt)
        if pairs:
            def summary(d, thr, name):
                return {"threshold_m": thr, "pairs": d,
                        "certified_lower_bound": min((v["certified_lower_bound"] for v in d.values()
                                                      if v["certified_lower_bound"] is not None), default=None),
                        "note": name}
            rows["arm_arm_spheres"] = summary(sph, gate, "corrected spheres, clearance must stay > gate")
            checks["arm_arm_spheres"] = all(v["ok"] for v in sph.values())
            if ums:
                rows["arm_arm_umarell_spheres"] = summary(ums, um.warning_distance, "umarell 35-sphere parity gate")
                checks["arm_arm_umarell_spheres"] = all(v["ok"] for v in ums.values())
            rows["arm_arm_hull"] = summary(hul, m.hull_min, f"MuJoCo hull distance >= hull_min (capped at the "
                                                           f"{m.mjw.pair_band:.3f} m detection band)")
            checks["arm_arm_hull"] = all(v["ok"] for v in hul.values()) or not sc.collision.hull_backstop
        out = {"method": "piecewise-linear joint paths between the timeline's knots; provable chain displacement "
                         "bound per point; interval bound (g_a + g_b - L) / 2; bisection to min_dt "
                         "(trajgen.certify, docs/p3_tgcert.md)",
               "min_dt_s": min_dt, "report_tol_m": tol if minimize else None, "checks": checks, "rows": rows,
               "all_ok": bool(all(checks.values())), "wall_s": round(_time.time() - t_wall, 1)}
        return out


def knots_on_grid(tl, hz: float, t_end=None) -> bool:
    """True when every knot of every robot lies on the uniform grid 0, 1/hz, ... (then the verifier's sampled grid
    contains the whole piecewise-linear path, and the grid-based skew-window certificate is exact)."""
    from trajgen.ntimeline import as_ntimeline
    tl = as_ntimeline(tl)
    T = float(tl.t[-1]) if t_end is None else float(t_end)
    K = _knots(tl, set(tl.robot_ids), T)
    x = K * hz
    return bool(np.all(np.abs(x - np.round(x)) < 1e-6))
