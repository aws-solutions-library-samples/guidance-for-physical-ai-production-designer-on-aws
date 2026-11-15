"""Step 2: allocation of welds to arms and per-arm order (reachability-aware, balanced).

Cost model (seconds, the same caps as the time parameterisation): a weld costs its lift-off + its descent + the
dwell; going from weld i to weld j costs a straight joint move from i's retract configuration to j's pre-weld
configuration (``motion.line_time``), with the IK branch of every weld chosen by dynamic programming along the
order (Viterbi over the <= 6 options per weld). ``thin`` options (below the margin preference) cost a penalty.

* forced welds: reachable (comfortably, else at all) by one arm only;
* shared welds: a sweep split. Shared welds are ranked by how much more they cost robot1 than robot2 (HOME-to-weld
  cost difference); robot1 takes the k cheapest-for-it, robot2 the rest, for every k. Each split is ordered per arm
  (sequence groups in the task's order per arm, nearest neighbour + 2-opt per group) and scored by
  max(T_robot1, T_robot2) + 0.1 * (T_robot1 + T_robot2). The ``n_candidates`` best distinct splits are returned
  and the generator schedules each one (the true two-arm makespan decides).
"""
from __future__ import annotations

import itertools

import numpy as np

from trajgen.motion import line_time

THIN_PENALTY_S = 3.0


class CostModel:
    def __init__(self, model, planner, reach: dict):
        self.m, self.pl, self.reach = model, planner, reach
        self._seg = {}
        self._mv = {}
        self._oi = {}          # rid -> {id(option): index}; transit times option retract -> option pre-weld
        self._tm = {}          # rid -> (n, n) matrix (nan = not computed yet), same values as move()
        self._keep = []        # the indexed options (kept alive, so their ids stay unique)

    def _index(self, rid, ops):
        d = self._oi.setdefault(rid, {})
        out = []
        for o in ops:
            k = d.get(id(o))
            if k is None:
                k = d[id(o)] = len(d)
                self._keep.append(o)
            out.append(k)
        n = len(d)
        T = self._tm.get(rid)
        if T is None or len(T) < n:
            T2 = np.full((max(n, 2 * (0 if T is None else len(T))), ) * 2, np.nan)
            if T is not None:
                T2[:len(T), :len(T)] = T
            self._tm[rid] = T2
        return np.array(out, int)

    def move_matrix(self, rid, prev, cur):
        """(len(cur), len(prev)) move(rid, prev[a].q_ret, cur[b].q_pre) (cached matrix; the values of move())."""
        ia, ib = self._index(rid, prev), self._index(rid, cur)
        T = self._tm[rid]
        M = T[np.ix_(ib, ia)]
        miss = np.argwhere(np.isnan(M))
        for b, a in miss:
            M[b, a] = T[ib[b], ia[a]] = self.move(rid, prev[a].q_ret, cur[b].q_pre)
        return M

    def options(self, wid, rid):
        return self.reach.get((wid, rid), [])

    def _stroke(self, o):
        k = id(o)
        if k not in self._seg:
            T = self.pl.seg(o.rid, "retract", o.liftoff).T + self.pl.seg(o.rid, "approach", o.descent).T
            self._seg[k] = T + self.pl.dwell + (THIN_PENALTY_S if o.thin else 0.0)
        return self._seg[k]

    def move(self, rid, qa, qb, cap="transit"):
        key = (rid, cap, np.asarray(qa, float).tobytes(), np.asarray(qb, float).tobytes())
        v = self._mv.get(key)
        if v is None:
            s = self.pl.sp[rid]
            v = self._mv[key] = line_time(np.asarray(qb) - np.asarray(qa), s.v[cap], s.a[cap], s.j[cap])
        return v

    def route(self, rid, wids, start=None):
        """(time, [option per weld]) of HOME (or the retract pose of WeldOption ``start``) -> wids -> HOME with the
        DP-optimal branches."""
        home = self.m.robots[rid].home
        q0 = home if start is None else start.q_ret
        if not wids:
            return self.move(rid, q0, home, "home"), []
        layers = [self.options(w, rid) for w in wids]
        if any(not L for L in layers):
            return np.inf, None
        cost = [np.array([self.move(rid, q0, o.q_pre) + self._stroke(o) for o in layers[0]])]
        back = []
        for i in range(1, len(layers)):
            prev, cur = layers[i - 1], layers[i]
            M = cost[-1][None, :] + self.move_matrix(rid, prev, cur)
            back.append(M.argmin(1))
            cost.append(M.min(1) + np.array([self._stroke(o) for o in cur]))
        last = cost[-1] + np.array([self.move(rid, o.q_ret, home, "home") for o in layers[-1]])
        k = int(last.argmin())
        sel = [k]
        for b in back[::-1]:
            sel.append(int(b[sel[-1]]))
        sel = sel[::-1]
        return float(last[k]), [layers[i][j] for i, j in enumerate(sel)]

    def best_pair(self, rid, wa, wb):
        """Cheapest weld-to-weld move between any branches (for ordering heuristics)."""
        A, B = self.options(wa, rid), self.options(wb, rid)
        return min(self.move(rid, a.q_ret, b.q_pre) for a in A for b in B)


def order_group(cm: CostModel, rid, wids, start=None):
    """Nearest neighbour from ``start`` (None = HOME) + 2-opt on the open path (scored by CostModel.route)."""
    wids = list(wids)
    if len(wids) <= 1:
        return wids
    home = cm.m.robots[rid].home

    def c0(w):
        return min(cm.move(rid, home if start is None else start.q_ret, o.q_pre) for o in cm.options(w, rid))

    rest = set(wids)
    cur = min(rest, key=c0)
    seq = [cur]
    rest.remove(cur)
    while rest:
        nxt = min(rest, key=lambda w: cm.best_pair(rid, seq[-1], w))
        seq.append(nxt)
        rest.remove(nxt)
    return seq


def order_arm(cm: CostModel, rid, wids, groups_order, task):
    """Per-arm order: the task's sequence groups in order, each by nearest neighbour; then 2-opt inside groups."""
    gw = {}
    for w in wids:
        g = task.weld(w).group
        gw.setdefault(g, []).append(w)
    keys = [g for g in groups_order if g in gw] + [g for g in gw if g not in groups_order]
    seq = []
    for g in keys:
        seq += order_group(cm, rid, gw[g])
    bounds, k = [], 0
    for g in keys:
        bounds.append((k, k + len(gw[g])))
        k += len(gw[g])
    best, _ = cm.route(rid, seq)
    improved = True
    while improved:
        improved = False
        for a, b in bounds:
            for i, j in itertools.combinations(range(a, b), 2):
                cand = seq[:i] + seq[i:j + 1][::-1] + seq[j + 1:]
                c, _ = cm.route(rid, cand)
                if c < best - 1e-6:
                    seq, best, improved = cand, c, True
    return seq, best


def allocate(model, cm: CostModel, task, *, n_candidates: int = 3, group_order: str = "per_arm", log=print):
    """Candidate allocations: list of dicts {"queues": {rid: [wid, ...]}, "proxy": s, "T": {rid: s}, "k": int}."""
    r1, r2 = model.rids
    wids = [w.id for w in task.welds]
    groups = task.sequence_groups_order if group_order != "ignore" else []

    def reachable(w, r, comfy):
        ops = cm.options(w, r)
        return bool(ops) and (not comfy or any(not o.thin for o in ops))

    forced = {r1: [], r2: []}
    shared, unreachable = [], []
    for w in wids:
        c1, c2 = reachable(w, r1, True), reachable(w, r2, True)
        a1, a2 = reachable(w, r1, False), reachable(w, r2, False)
        if c1 and c2:
            shared.append(w)
        elif c1 or c2:
            if (a1 and a2):
                shared.append(w)          # one arm only thin: still shared, the penalty decides
            else:
                forced[r1 if c1 else r2].append(w)
        elif a1 and a2:
            shared.append(w)
        elif a1 or a2:
            forced[r1 if a1 else r2].append(w)
        else:
            unreachable.append(w)
    home = {r: model.robots[r].home for r in model.rids}

    def hcost(w, r):
        return min(cm.move(r, home[r], o.q_pre) + cm._stroke(o) for o in cm.options(w, r))

    shared.sort(key=lambda w: hcost(w, r1) - hcost(w, r2))
    cands = []
    for k in range(len(shared) + 1):
        q1 = forced[r1] + shared[:k]
        q2 = forced[r2] + shared[k:]
        s1, T1 = order_arm(cm, r1, q1, groups, task)
        s2, T2 = order_arm(cm, r2, q2, groups, task)
        proxy = max(T1, T2) + 0.1 * (T1 + T2)
        cands.append({"queues": {r1: s1, r2: s2}, "proxy": proxy, "T": {r1: T1, r2: T2}, "k": k})
    cands.sort(key=lambda c: c["proxy"])
    out, seen = [], set()
    for c in cands:
        key = tuple(sorted(c["queues"][r1]))
        if key in seen or not np.isfinite(c["proxy"]):
            continue
        seen.add(key)
        out.append(c)
        if len(out) >= n_candidates:
            break
    log(f"[alloc] forced {forced}, shared {shared}, unreachable {unreachable}")
    for c in out:
        log(f"[alloc] k={c['k']} proxy {c['proxy']:.1f}s T {c['T'][r1]:.1f}/{c['T'][r2]:.1f}  {c['queues']}")
    return out, {"forced": forced, "shared": shared, "unreachable": unreachable}


def queues_with_options(cm: CostModel, queues: dict) -> dict:
    """{rid: [[wid, [options: DP-chosen first, then the rest]], ...]}"""
    out = {}
    for rid, wids in queues.items():
        _, sel = cm.route(rid, wids)
        if sel is None:
            raise ValueError(f"{rid} cannot reach every weld of {wids}")
        out[rid] = [[w, [o] + [x for x in cm.options(w, rid) if x is not o]] for w, o in zip(wids, sel)]
    return out


# ------------------------------------------------------------------------------------------- N robots
def allocate_n(model, cm: CostModel, task, eligible: dict, *, n_candidates: int = 3, group_order: str = "per_arm",
               max_iter: int = 200, zone_weight: float = 2.0, leg_margin_s: float = 0.0,
               balance_spread_s: float = 5.0, log=print):
    """Reachability-aware min-makespan assignment for any number of robots.

    ``eligible``: {wid: [rid, ...]} robots that may do the weld. Each robot's welds are ordered by ``order_arm``
    (sequence groups in the task's order, nearest neighbour + 2-opt, DP over the IK branches); the score is
    max_r T_r + 0.1 * sum_r T_r + zone_weight * sum_w (|w - base_r(w)| - min_r' |w - base_r'|) with T_r the robot's
    route time (HOME -> its welds -> HOME, seconds); the last term (s per m, default 2) prefers the nearest eligible
    base, a proxy for the arm-arm interference the route times do not see (robots working in each other's zone).
    Greedy (most constrained weld first, cheapest increase of the score), then local search with single-weld moves
    and pairwise swaps until no move improves. Returns the ``n_candidates`` best distinct assignments
    [{"queues": {rid: [wid, ...]}, "proxy": s, "T": {rid: s}}] and an info dict."""
    rids = list(model.rids)
    groups = task.sequence_groups_order if group_order != "ignore" else []
    welds = [w.id for w in task.welds if eligible.get(w.id)]
    memo = {}

    def T(r, ws):
        key = (r, frozenset(ws))
        if key not in memo:
            memo[key] = order_arm(cm, r, sorted(ws), groups, task)
        return memo[key]

    base = {r: np.asarray(model.robots[r].base_xyz, float) for r in rids}
    pos = {w.id: w.position for w in task.welds}

    def zone(w, r):
        d = {x: float(np.linalg.norm(pos[w] - base[x])) for x in eligible.get(w, [r])}
        return d.get(r, float(np.linalg.norm(pos[w] - base[r]))) - min(d.values())

    def score(assign):
        Ts = {r: T(r, assign[r])[1] for r in rids}
        z = sum(zone(w, r) for r in rids for w in assign[r])
        return max(Ts.values()) + 0.1 * sum(Ts.values()) + zone_weight * z, Ts

    seen = {}

    def record(assign):
        key = tuple(tuple(sorted(assign[r])) for r in rids)
        if key not in seen:
            s, Ts = score(assign)
            seen[key] = (s, {r: list(T(r, assign[r])[0]) for r in rids}, Ts)
        return seen[key][0]

    home_cost = {(w, r): min(cm.move(r, model.robots[r].home, o.q_pre) + cm._stroke(o) for o in cm.options(w, r))
                 for w in welds for r in eligible[w] if cm.options(w, r)}
    for w in welds:
        eligible[w] = [r for r in eligible[w] if (w, r) in home_cost]
    welds = [w for w in welds if eligible[w]]

    def regret(w):
        c = sorted(home_cost[(w, r)] for r in eligible[w])
        return (c[1] - c[0]) if len(c) > 1 else np.inf

    assign = {r: [] for r in rids}
    for w in sorted(welds, key=lambda w: (len(eligible[w]), -regret(w), w)):
        best = None
        for r in eligible[w]:
            trial = {k: (v + [w] if k == r else v) for k, v in assign.items()}
            s = score(trial)[0]                       # partial assignment: scored, not recorded
            if best is None or s < best[0] - 1e-9:
                best = (s, r)
        assign[best[1]].append(w)
    cur = record(assign)
    it = 0
    improved = True
    while improved and it < max_iter:
        improved = False
        it += 1
        moves = []
        for r in rids:
            for w in assign[r]:
                for r2 in eligible[w]:
                    if r2 != r:
                        moves.append(("move", r, w, r2, None))
        for i, r in enumerate(rids):
            for r2 in rids[i + 1:]:
                for w in assign[r]:
                    for v in assign[r2]:
                        if r2 in eligible[w] and r in eligible[v]:
                            moves.append(("swap", r, w, r2, v))
        best = None
        for kind, r, w, r2, v in moves:
            trial = {k: list(x) for k, x in assign.items()}
            trial[r].remove(w)
            trial[r2].append(w)
            if kind == "swap":
                trial[r2].remove(v)
                trial[r].append(v)
            s = record(trial)
            if s < cur - 1e-6 and (best is None or s < best[0]):
                best = (s, trial)
        if best is not None:
            cur, assign = best[0], best[1]
            improved = True
    ranked = sorted(seen.items(), key=lambda kv: kv[1][0])
    out = [{"queues": q, "proxy": s, "T": Ts} for _, (s, q, Ts) in ranked[:max(1, n_candidates)]]
    # busy-balanced neighbour of the best assignment (round-4 finding: the proxy kept 56 / 47 s of busy time on two
    # robots that share welds): when the busy spread (route + one executor margin per leg) exceeds
    # balance_spread_s, the single move / swap off the busiest robot that lowers the largest busy time most is
    # added as an extra candidate (the coordinated makespan decides)
    bal_info = None
    best_assign = {r: list(ranked[0][1][1][r]) for r in rids} if ranked else None
    if best_assign is not None and len(rids) > 1:
        def busy(a):
            return {r: T(r, a[r])[1] + (len(a[r]) + 1) * leg_margin_s if a[r] else 0.0 for r in rids}
        b0 = busy(best_assign)
        spread = max(b0.values()) - min(b0.values())
        bal_info = {"busy_s": {r: round(v, 2) for r, v in b0.items()}, "spread_s": round(spread, 2)}
        if spread > balance_spread_s:
            rb = max(rids, key=lambda r: b0[r])
            bestb = None
            for w in best_assign[rb]:
                for r2 in eligible.get(w, []):
                    if r2 == rb:
                        continue
                    trials = [("move", None)] + [("swap", v) for v in best_assign[r2] if rb in eligible.get(v, [])]
                    for kind, v in trials:
                        a = {k: list(x) for k, x in best_assign.items()}
                        a[rb].remove(w)
                        a[r2].append(w)
                        if kind == "swap":
                            a[r2].remove(v)
                            a[rb].append(v)
                        bb = busy(a)
                        key = (round(max(bb.values()), 3), record(a))
                        if bestb is None or key < bestb[0]:
                            bestb = (key, a, f"{kind} {w} {rb}->{r2}" + (f" / {v} {r2}->{rb}" if v else ""))
            if bestb is not None and bestb[0][0] < max(b0.values()) - 0.5:
                a = bestb[1]
                k2 = tuple(tuple(sorted(a[r])) for r in rids)
                s2, q2, T2 = seen[k2]
                if all(tuple(tuple(sorted(c["queues"][r])) for r in rids) != k2 for c in out):
                    out.append({"queues": q2, "proxy": s2, "T": T2, "balanced": bestb[2]})
                bal_info.update(move=bestb[2], busy_after_max_s=bestb[0][0])
    info = {"eligible": {w: list(eligible.get(w, [])) for w in [x.id for x in task.welds]},
            "evaluated": len(seen), "local_search_iterations": it, "balance": bal_info}
    for c in out:
        log(f"[alloc] proxy {c['proxy']:.1f}s T " + " / ".join(f"{r} {c['T'][r]:.1f}" for r in rids) + f"  {c['queues']}")
    return out, info
