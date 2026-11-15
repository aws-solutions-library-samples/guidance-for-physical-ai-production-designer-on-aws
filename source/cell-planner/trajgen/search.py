"""Two-arm step search ("steps" coordination, umarell's sync-at-rest executor model): allocation, order and
coordination decided jointly (steps 2 and 5). Used for scenes with exactly two robots (any ids, any placement);
any robot count is handled by trajgen.coord (prioritized planning).

The program is a sequence of executor steps (trajgen.schedule docstring). A search state is (welds done, where each
arm is: HOME or resting on weld w with IK option k, clock). An action is one step:
  * parallel: arm 1 -> weld a, arm 2 -> weld b (or HOME when an arm has nothing left), one of them possibly
    started later (stationary prefix);
  * alone: one arm moves, the other stands;
  * yield: an arm returns HOME (only when a state has no other feasible action).
Candidate welds per arm: the ``k_cand`` remaining welds it reaches that are cheapest to go to from where it is
(sequence-group rule applied). Beam search with width ``beam`` on clock + an estimate of the remaining work.

Feasibility of a step is first screened on a *conflict map*: for the legs A, B (their 20 Hz samples) the matrix
M[i, j] = min(corrected-sphere clearance - gate, umarell-sphere clearance - umarell gate) - chord_extra;
a start delay d passes the screen iff the map stays > 0 along its diagonal (both arms stand still after their
last sample). All delays are screened at once. The 20 Hz samples are coarse in space (a transiting tool tip moves
up to ~0.3 m per sample), so the screen is only a ranking device: every state that makes it into the beam gets its
last step checked exactly (trajgen.schedule.PairChecker: 100 Hz, MuJoCo hulls, both sphere models); if that fails
the next screened delays are tried (``n_confirm``), else the state is dropped. Only exactly checked steps are
ever output.

With a given assignment (queues), each arm's candidates are restricted to the head of its own queue (``window``).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from trajgen.schedule import LegPlan


@dataclass
class Step:
    t0: float                    # motion-clock start
    legs: dict                   # rid -> LegPlan (wait prefix included)
    t_end: float                 # motion-clock end (legs + holds)
    min_clearance: float
    note: str = ""


@dataclass
class State:
    t: float                     # objective clock (motion + executor margins if objective == "executor")
    t_motion: float
    pos: tuple                   # per arm: None (HOME) or (wid, option index)
    done: frozenset
    grp: tuple                   # per arm: current sequence-group rank (per_arm rule)
    steps: tuple = ()
    yields: int = 0
    h: float = 0.0
    by: tuple = ()               # ((wid, rid), ...)

    w: float = 1.0               # heuristic weight (Search.h_weight)
    stay: float = 0.0            # time arms spent standing on a weld while the other moved (tie-break)

    @property
    def f(self):
        return self.t + self.w * self.h


class Search:
    def __init__(self, model, planner, checker, reach: dict, cost, task, *, beam: int = 8, k_cand: int = 4,
                 map_stride: int = 1, chord_factor: float = 0.0, chord_extra: float = 0.02, n_confirm: int = 12, objective: str = "executor",
                 executor_margin: float = 1.0, group_order: str = "per_arm", queues: dict | None = None,
                 window: int = 1, max_yields: int = 3, max_depth: int = 80, h_weight: float = 1.0, per_done_set: int = 0,
                 n_branch: int = 1, log=print):
        self.m, self.pl, self.ck, self.reach, self.cost, self.task = model, planner, checker, reach, cost, task
        self.rids = model.rids
        if len(self.rids) != 2:
            raise ValueError("trajgen.search (steps coordination) needs exactly two robots; use trajgen.coord")
        self.dt = planner.dt
        self.beam, self.k_cand, self.stride, self.chord_extra = beam, k_cand, max(1, map_stride), chord_extra
        self.chord_factor, self.n_confirm = chord_factor, n_confirm
        self.lam = 1.0 if objective == "executor" else 0.0
        self.margin = executor_margin
        self.group_order = group_order
        self.queues = queues
        self.window = max(1, window)
        self.max_yields, self.max_depth = max_yields, max_depth
        self.log = log
        self.h_weight = float(h_weight)
        self.per_done_set = int(per_done_set)
        self.n_branch = max(1, int(n_branch))
        self.hold_n = int(round(planner.dwell / self.dt))
        gorder = list(task.sequence_groups_order)
        self.rank = {w.id: (gorder.index(w.group) if w.group in gorder else len(gorder)) for w in task.welds}
        self.wids = [w.id for w in task.welds]
        self.reachable = {w: [r for r in self.rids if reach.get((w, r))] for w in self.wids}
        self.unreachable = [w for w in self.wids if not self.reachable[w]]
        self.target = frozenset(w for w in self.wids if self.reachable[w])
        self._legs, self._sph, self._maps = {}, {}, {}
        self.home = {r: model.robots[r].home for r in self.rids}
        self._est = self._estimates()
        self.stats = {"legs": 0, "maps": 0, "exact_checks": 0, "exact_fail": 0, "expanded": 0}

    # ------------------------------------------------------------------ geometry helpers
    def opt(self, key, rid):
        return None if key is None else self.reach[(key[0], rid)][key[1]]

    def q_of(self, rid, key):
        return self.home[rid] if key is None else self.opt(key, rid).q

    def leg(self, rid, frm, to):
        """LegPlan from pose key ``frm`` to ``to`` ((wid, k) or "home"), cached; None if no transit."""
        k = (rid, frm, to)
        if k not in self._legs:
            at = self.opt(frm, rid)
            tgt = "home" if to == "home" else self.opt(to, rid)
            self._legs[k] = self.pl.leg(rid, self.q_of(rid, frm), at, tgt)
            self.stats["legs"] += 1
        return self._legs[k]

    def stand(self, rid, key):
        k = (rid, key, "stand")
        if k not in self._legs:
            q = self.q_of(rid, key)
            self._legs[k] = LegPlan(rid, "stand", None, [], q[None].copy(), np.array(["wait"], dtype="<U16"), 0.0,
                                    dt=self.dt)
        return self._legs[k]

    def _spheres(self, L):
        k = id(L)
        if k not in self._sph:
            Q = L.Q[::self.stride]
            if (len(L.Q) - 1) % self.stride:
                Q = np.vstack([Q, L.Q[-1:]])
            S = self.m.pair_spheres(L.rid, Q).astype(np.float32)
            U = self.m.umarell_spheres(L.rid, Q).astype(np.float32) if self.ck.parity else None
            # allowance for the motion between map samples: half the largest sphere chord to either neighbour
            ch = np.zeros(len(Q))
            if len(Q) > 1:
                c = np.linalg.norm(np.diff(S, axis=0), axis=-1).max(1)
                if U is not None:
                    c = np.maximum(c, np.linalg.norm(np.diff(U, axis=0), axis=-1).max(1))
                ch[:-1] = np.maximum(ch[:-1], c)
                ch[1:] = np.maximum(ch[1:], c)
            self._sph[k] = (S, U, self.chord_factor * ch, L)
        return self._sph[k]

    @staticmethod
    def _clr(SA, SB, r):
        nA, a, _ = SA.shape
        nB, b, _ = SB.shape
        A2 = SA.reshape(-1, 3)
        B2 = SB.reshape(-1, 3)
        c = 0.5 * (A2.mean(0) + B2.mean(0))
        A2, B2 = A2 - c, B2 - c
        d2 = (A2 * A2).sum(1)[:, None] + (B2 * B2).sum(1)[None, :] - 2.0 * (A2 @ B2.T)
        d2 = d2.reshape(nA, a, nB, b).min(axis=(1, 3))
        return np.sqrt(np.maximum(d2, 0.0)) - 2.0 * r

    def cmap(self, LA, LB):
        k = (id(LA), id(LB))
        if k not in self._maps:
            SA, UA, cA, _ = self._spheres(LA)
            SB, UB, cB, _ = self._spheres(LB)
            M = self._clr(SA, SB, self.ck.pair_r) - self.ck.gate
            if self.ck.parity:
                M = np.minimum(M, self._clr(UA, UB, self.m.umarell.pair_radius) - self.ck.gate_u)
            M = M - cA[:, None] - cB[None, :] - self.chord_extra
            w = int(np.ceil(self.ck.skew_n / self.stride)) if getattr(self.ck, "skew_n", 0) else 0
            if w:                                          # dispatch skew window: B up to w map samples off
                from trajgen.coord import Coordinator
                M = Coordinator._winmin(M, w, axis=1)
            self._maps[k] = M
            self.stats["maps"] += 1
            if len(self._maps) > 20000:
                self._maps.clear()
        return self._maps[k]

    def delays(self, LA, LB, max_delay_s: float | None = None):
        """Feasible (E, dA, dB, min map value) for the pair (A runs LA, B runs LB), sorted by step end E.
        Delays are multiples of the map stride (in output samples)."""
        M = self.cmap(LA, LB)
        nA, nB = M.shape
        s = self.stride
        hA, hB = LA.hold_s, LB.hold_s
        TA, TB = (len(LA.Q) - 1) * self.dt, (len(LB.Q) - 1) * self.dt
        kmaxA = nB + int(np.ceil(hB / (self.dt * s))) + 1
        kmaxB = nA + int(np.ceil(hA / (self.dt * s))) + 1
        if max_delay_s is not None:
            kmaxA = min(kmaxA, int(max_delay_s / (self.dt * s)) + 1)
            kmaxB = min(kmaxB, int(max_delay_s / (self.dt * s)) + 1)
        out = []
        for who, kmax in (("B", kmaxB), ("A", kmaxA)):
            ks = np.arange(0 if who == "B" else 1, kmax + 1)
            if not len(ks):
                continue
            Tn = max(nA, nB) + int(ks.max()) + 1
            tau = np.arange(Tn)
            if who == "B":
                ia = np.broadcast_to(np.clip(tau, 0, nA - 1), (len(ks), Tn))
                ib = np.clip(tau[None] - ks[:, None], 0, nB - 1)
            else:
                ib = np.broadcast_to(np.clip(tau, 0, nB - 1), (len(ks), Tn))
                ia = np.clip(tau[None] - ks[:, None], 0, nA - 1)
            vals = M[ia, ib].min(1)
            for k, v in zip(ks, vals):
                if v > 0:
                    dA, dB = (0, int(k) * s) if who == "B" else (int(k) * s, 0)
                    E = max(dA * self.dt + TA + hA, dB * self.dt + TB + hB)
                    out.append((E, dA, dB, float(v)))
        out.sort(key=lambda x: x[0])
        return out

    # ------------------------------------------------------------------ estimates
    def _estimates(self):
        """Per (weld, arm): stroke + dwell + the cheapest incoming move (from HOME or any other weld's retract)."""
        est = {}
        for w in self.wids:
            for r in self.reachable[w]:
                best = np.inf
                for o in self.reach[(w, r)]:
                    stroke = self.cost._stroke(o)
                    srcs = [self.home[r]] + [x.q_ret for w2 in self.wids if w2 != w for x in self.reach.get((w2, r), [])[:1]]
                    mv = min(self.cost.move(r, qs, o.q_pre) for qs in srcs)
                    best = min(best, stroke + mv)
                est[(w, r)] = best
        return est

    def h(self, done, pos):
        R = [w for w in self.wids if w in self.target and w not in done]
        if not R:
            back = [self.cost.move(r, self.opt(pos[i], r).q_ret, self.home[r], "home") for i, r in enumerate(self.rids)
                    if pos[i] is not None]
            return max(back, default=0.0) + (self.lam * self.margin if back else 0.0)
        own = {r: 0.0 for r in self.rids}
        tot = 0.0
        for w in R:
            rs = self.reachable[w]
            c = min(self._est[(w, r)] for r in rs)
            tot += c
            if len(rs) == 1:
                own[rs[0]] += self._est[(w, rs[0])]
        work = max(max(own.values()), tot / 2.0)
        return work + self.lam * self.margin * int(np.ceil(len(R) / 2.0))

    # ------------------------------------------------------------------ candidates
    def candidates(self, st: State, i: int):
        r = self.rids[i]
        rem = [w for w in self.wids if w in self.target and w not in st.done and r in self.reachable[w]]
        if self.queues is not None:
            q = [w for w in self.queues[r] if w not in st.done]
            rem = [w for w in q[:self.window]]
        if self.group_order == "global":
            allr = [self.rank[w] for w in self.wids if w in self.target and w not in st.done]
            if allr:
                rem = [w for w in rem if self.rank[w] == min(allr)]
        elif self.group_order == "per_arm":
            # monotone per arm; an arm may move on to a later group only if no remaining earlier-group weld
            # depends on it (reachable by it alone, or the other arm has already moved past that group)
            left = [w for w in self.wids if w in self.target and w not in st.done]
            j = 1 - i

            def needs(w2):
                rs = self.reachable[w2]
                return r in rs and (len(rs) == 1 or st.grp[j] > self.rank[w2])
            rem = [w for w in rem if self.rank[w] >= st.grp[i]
                   and not any(self.rank[w2] < self.rank[w] and needs(w2) for w2 in left)]
        cur = self.opt(st.pos[i], r)
        q0 = self.home[r] if cur is None else cur.q_ret
        scored = []
        for w in rem:
            ops = self.reach[(w, r)]
            costs = np.array([self.cost.move(r, q0, o.q_pre) + self.cost._stroke(o) for o in ops])
            order = np.argsort(costs, kind="stable")
            scored.append((float(costs[order[0]]), w, [int(k) for k in order[:self.n_branch]]))
        scored.sort(key=lambda x: x[0])
        if self.queues is not None:
            scored.sort(key=lambda x: self.queues[r].index(x[1]))
        return [(w, k) for _, w, ks in scored[:self.k_cand] for k in ks]

    # ------------------------------------------------------------------ expansion
    def _has_work(self, st, i):
        r = self.rids[i]
        return any(r in self.reachable[w] and (self.queues is None or w in self.queues[r])
                   for w in self.target if w not in st.done)

    def expand(self, st: State):
        """Children of a state: parallel pairs, one arm alone, and HOME moves (a HOME move of an arm that still
        has work is a yield, at most ``max_yields`` per program; at a dead end the budget is ignored)."""
        self.stats["expanded"] += 1
        out = self._expand(st, self.max_yields)
        if not out:
            self.stats["dead_end_fallback"] = self.stats.get("dead_end_fallback", 0) + 1
            out = self._expand(st, 10 ** 6, homes_only=True)
        return out

    def _expand(self, st: State, max_yields: int, homes_only: bool = False):
        r0, r1 = self.rids
        acts = {}
        for i, r in enumerate(self.rids):
            work = self._has_work(st, i)
            c = self.candidates(st, i) if work else []
            if st.pos[i] is not None and (not work or st.yields < max_yields):
                c = c + ["home"]
            acts[r] = c
        out = []
        for a in acts[r0]:
            for b in acts[r1]:
                if a != "home" and b != "home" and a[0] == b[0]:
                    continue
                if homes_only and "home" not in (a, b):
                    continue
                LA, LB = self.leg(r0, st.pos[0], a), self.leg(r1, st.pos[1], b)
                if LA is None or LB is None:
                    continue
                ds = self.delays(LA, LB)
                if ds:
                    c = self._child(st, {r0: (a, LA), r1: (b, LB)}, ds, "parallel")
                    if c is not None:
                        out.append(c)
        for i, r in enumerate(self.rids):
            j = 1 - i
            for a in acts[r]:
                if homes_only and a != "home":
                    continue
                L = self.leg(r, st.pos[i], a)
                if L is None:
                    continue
                S = self.stand(self.rids[j], st.pos[j])
                ds = self.delays(L, S) if i == 0 else self.delays(S, L)
                ds = [d for d in ds if d[1] == 0 and d[2] == 0]
                if ds:
                    c = self._child(st, {r: (a, L)}, ds, "alone")
                    if c is not None:                 # None: the group prune fired (round-1 crash)
                        out.append(c)
        return out

    def _child(self, st, mv, ds, note):
        E, dA, dB, v = ds[0]
        pos = list(st.pos)
        grp = list(st.grp)
        done = set(st.done)
        by = list(st.by)
        for r, (k, L) in mv.items():
            i = self.rids.index(r)
            if k == "home":
                pos[i] = None
            else:
                pos[i] = k
                done.add(k[0])
                by.append((k[0], r))
                grp[i] = max(grp[i], self.rank[k[0]])
        n_y = sum(1 for r, (k, L) in mv.items() if k == "home" and self._has_work(st, self.rids.index(r)))
        if n_y:
            note = note + "+yield"
        rec = {"mv": mv, "ds": ds, "note": note}
        c = State(st.t + E + self.lam * self.margin, st.t_motion + E, tuple(pos), frozenset(done), tuple(grp),
                  st.steps + (rec,), st.yields + n_y, 0.0, tuple(by))
        c.stay = st.stay + sum(E for i, r in enumerate(self.rids) if r not in mv and st.pos[i] is not None)
        if self.group_order == "per_arm":             # prune: a remaining weld no arm may still do
            for w in self.target - c.done:
                if not any(c.grp[k] <= self.rank[w] for k, r in enumerate(self.rids) if r in self.reachable[w]):
                    return None
        c.h = self.h(c.done, c.pos)
        c.w = self.h_weight
        return c

    # ------------------------------------------------------------------ exact confirmation of a state's last step
    def confirm(self, parent: State, st: State) -> bool:
        rec = st.steps[-1]
        if rec.get("exact"):
            return True
        r0, r1 = self.rids
        mv = rec["mv"]
        LA = mv[r0][1] if r0 in mv else self.stand(r0, parent.pos[0])
        LB = mv[r1][1] if r1 in mv else self.stand(r1, parent.pos[1])
        for E, dA, dB, v in rec["ds"][:self.n_confirm]:
            self.stats["exact_checks"] += 1
            end_a, end_b = dA * self.dt + LA.T + LA.hold_s, dB * self.dt + LB.T + LB.hold_s
            tail = int(round((E - min(end_a, end_b)) / self.dt)) + self.hold_n
            ok, cmin, _ = self.ck.check(LA.Q, dA, LB.Q, dB, tail, want_min=False)
            if ok:
                rec["chk"] = (LA.Q, dA, LB.Q, dB, tail)       # min clearance computed for the output steps only
                if E > rec["ds"][0][0] + 1e-9:                       # a later delay: fix the clock
                    d = E - rec["ds"][0][0]
                    st.t += d
                    st.t_motion += d
                rec["chosen"] = (E, dA, dB)
                rec["exact"] = True
                return True
            self.stats["exact_fail"] += 1
        return False

    # ------------------------------------------------------------------ search
    def run(self):
        n = len(self.rids)
        root = State(0.0, 0.0, (None,) * n, frozenset(), (0,) * n)
        root.h = self.h(root.done, root.pos)
        root.w = self.h_weight
        beam = [(root, None)]
        best_final = None
        parents = {}
        for depth in range(self.max_depth):
            children = []
            for st, _ in beam:
                for c in self.expand(st):
                    children.append((c, st))
            if not children:
                for st, _ in beam[:3]:
                    self.log(f"[search] dead end: pos {st.pos} remaining {sorted(self.target - st.done)} "
                             f"yields {st.yields} t {st.t:.1f}")
                break
            # dedup: same (done, pos) keeps the smallest clock
            uniq = {}
            for c, p in children:
                key = (c.done, c.pos)
                u = uniq.get(key)
                if u is None or c.t < u[0].t - 1e-9 or (abs(c.t - u[0].t) <= 1e-9 and c.stay < u[0].stay):
                    uniq[key] = (c, p)
            ranked = sorted(uniq.values(), key=lambda cp: (round(cp[0].f, 6), cp[0].stay))   # ties: shorter stays
            beam = []
            per_set = {}
            for c, p in ranked:
                if best_final is not None and c.f >= best_final.t - 1e-9:
                    break
                if self.per_done_set and per_set.get(c.done, 0) >= self.per_done_set:
                    continue                                   # diversity: few states per set of done welds
                if not self.confirm(p, c):
                    continue
                if c.done >= self.target and all(x is None for x in c.pos):
                    if best_final is None or (round(c.t, 6), c.stay) < (round(best_final.t, 6), best_final.stay):
                        best_final = c
                    continue
                beam.append((c, p))
                per_set[c.done] = per_set.get(c.done, 0) + 1
                if len(beam) >= self.beam:
                    break
            self.log(f"[search] depth {depth + 1}: {len(children)} children, {len(uniq)} unique, beam {len(beam)}"
                     + (f", best f {beam[0][0].f:.1f}" if beam else "") + (f", best final {best_final.t:.2f}" if best_final else ""))
            if not beam:
                break
        return best_final

    def early_home(self, steps: list, max_cost_s: float = 1e-9) -> tuple[list, list]:
        """Post-pass, the weld-stay tie-break (round-4 finding: an arm with no work left stayed on its last weld
        while the other worked alone): move an arm's final HOME leg into an earlier step in which only the other arm
        moves, when the executor makespan does not grow by more than ``max_cost_s`` (the step may get longer if
        the final step gets shorter by as much) and every affected step passes the exact check again (the moved
        HOME leg against the other arm's leg; the other arm's later legs against this arm standing at HOME).
        Returns (steps, report rows: applied moves, and for each arm the cheapest rejected alternative)."""
        from dataclasses import replace
        r0, r1 = self.rids
        steps = list(steps)
        rows = []

        def chk(La, Lb):
            ok, c, _ = self.ck.check(La.Q, 0, Lb.Q, 0, self.hold_n)     # (want_min: the clearance is reported)
            return ok, c

        def dur(legs):
            return max(L.T + L.hold_s for L in legs.values())

        for ri in self.rids:
            rj = r1 if ri == r0 else r0
            idx = [k for k, st in enumerate(steps) if ri in st.legs]
            if len(idx) < 2:
                continue
            kh, kw = idx[-1], idx[-2]
            Lh = steps[kh].legs[ri]
            if Lh.target != "home" or steps[kw].legs[ri].target == "home" or kh - kw < 2:
                continue
            n0 = int(Lh.wait_n)
            Lh0 = replace(Lh, Q=Lh.Q[n0:].copy(), phase=Lh.phase[n0:].copy(), wait_n=0)
            home_stand = self.stand(ri, None)
            E_kh = steps[kh].t_end - steps[kh].t0
            rest = {k: v for k, v in steps[kh].legs.items() if k != ri}
            E_kh2 = dur(rest) if rest else 0.0
            cands = []
            for j in range(kw + 1, kh):
                sj = steps[j]
                if ri in sj.legs or rj not in sj.legs:
                    continue
                E_j = sj.t_end - sj.t0
                E_j2 = max(E_j, Lh0.T + Lh0.hold_s)
                cands.append((E_j2 - E_j + E_kh2 - E_kh, j, E_j2))
            applied = False
            for cost, j, E_j2 in sorted(cands):
                if cost > max_cost_s:
                    rows.append({"robot": ri, "applied": False, "cheapest_alternative_step": j,
                                 "makespan_cost_s": round(cost, 3), "stay_saved_s": round(steps[kh].t0 - steps[j].t0, 3)})
                    break
                Lo = steps[j].legs[rj]
                ok, cj = chk(Lh0, Lo) if ri == r0 else chk(Lo, Lh0)
                if not ok:
                    continue
                later_ok, cl = True, {}
                for m in range(j + 1, kh + 1):
                    if rj not in steps[m].legs:
                        continue
                    L2 = steps[m].legs[rj]
                    ok2, c2 = chk(home_stand, L2) if ri == r0 else chk(L2, home_stand)
                    if not ok2:
                        later_ok = False
                        break
                    cl[m] = c2
                if not later_ok:
                    continue
                new = []
                for m, st in enumerate(steps):
                    E = st.t_end - st.t0
                    if m == j:
                        st = replace(st, legs={**st.legs, ri: Lh0}, note=st.note + "+home_early",
                                     min_clearance=min(st.min_clearance, cj))
                        E = E_j2
                    elif m == kh:
                        if not rest:
                            continue
                        st = replace(st, legs=rest, min_clearance=cl.get(m, st.min_clearance))
                        E = E_kh2
                    new.append((st, E))
                t = 0.0
                for m, (st, E) in enumerate(new):
                    new[m] = replace(st, t0=t, t_end=t + E)
                    t += E
                rows.append({"robot": ri, "applied": True, "from_step": kh, "to_step": j,
                             "makespan_cost_s": round(cost, 3), "clearance_m": round(float(cj), 4)})
                steps = new
                applied = True
                break
            if not applied and cands and not any(r["robot"] == ri for r in rows):
                rows.append({"robot": ri, "applied": False, "note": "no collision-free earlier HOME within the cost"})
        return steps, rows

    def park_steps(self, steps: list) -> tuple[list, list]:
        """Post-pass, parking (round p3): an arm that rests on a weld while the other arm works alone lifts off at
        the start of the first such step (its own parking leaf, a lift-off-only ExecuteTrajectory) and waits at the
        retract pose, and its next leg starts from there (the leg without its lift-off); a leg that starts after a
        delay with the tool on the weld lifts off first and spends the delay at the retract pose (same leaf, same
        timing). Applied only where no step gets longer and every affected step passes the exact check again (the
        parking leaf against the other arm's leg, the other arm's legs against this arm at the retract pose, the
        shortened next leg). Returns (steps, report rows)."""
        from dataclasses import replace
        r0, r1 = self.rids
        steps = list(steps)
        rows = []
        dt = self.dt

        cmin_new, E_new = {}, {}

        def chk(legs, E):
            LA, LB = legs[r0], legs[r1]
            tail = int(round((E - min(LA.T + LA.hold_s, LB.T + LB.hold_s)) / dt)) + self.hold_n
            ok, c, _ = self.ck.check(LA.Q, 0, LB.Q, 0, tail)
            chk.c = c
            return ok

        def n_lift(L):
            seg = L.segments[0] if L.segments else None
            return int(seg.n) if (seg is not None and seg.phase == "retract") else 0

        def unprefixed(L):
            d = int(L.wait_n)
            return L.Q[d:], L.phase[d:], d

        # where every arm rests before each step (joint configuration), and the weld option it rests on
        for ri in self.rids:
            rj = r1 if ri == r0 else r0
            k = 0
            while k < len(steps):
                idx = [m for m, st in enumerate(steps) if ri in st.legs]
                nxt = [m for m in idx if m > k]
                if ri not in steps[k].legs or not nxt:
                    k += 1
                    continue
                j, m = k, nxt[0]
                Lj = steps[j].legs[ri]
                Lm = steps[m].legs[ri]
                k = m
                opt = Lj.opt
                if Lj.target in ("home", "park") or opt is None or n_lift(Lm) == 0:
                    continue
                Q0, ph0, d = unprefixed(Lm)
                nl = n_lift(Lm)
                q_ret = Q0[nl]
                applied, why = None, None
                for s0 in range(j + 1, m):           # (B) a parking leaf in the first idle step s0 where it fits,
                    P = self.pl.lift(ri, opt)          # then the arm waits at the retract pose until step m
                    sj = steps[s0]
                    E1 = sj.t_end - sj.t0
                    if P is None or P.T + P.hold_s > E1 + 1e-9 or rj not in sj.legs:
                        why = why or f"the lift-off leaf is longer than step {s0}"
                        continue
                    new = {}
                    ok = chk({ri: P, rj: sj.legs[rj]}, E1)
                    cs = {s0: chk.c}
                    if not ok:
                        why = why or f"lift-off leaf vs {rj}'s leg in step {s0}"
                        continue
                    new[s0] = replace(sj, legs={**sj.legs, ri: P}, note=sj.note + "+park")
                    stand = LegPlan(ri, "stand", None, [], q_ret[None].copy(), np.array(["wait"], dtype="<U16"),
                                    0.0, dt=dt)
                    for t in range(s0 + 1, m):
                        st = steps[t]
                        o_t = st.legs.get(rj) or self.stand_q(rj, self._rest_q(steps, rj, t))
                        if ri in st.legs or not chk({ri: stand, rj: o_t}, st.t_end - st.t0):
                            ok = False
                            why = why or f"retract pose vs {rj} in step {t}"
                            break
                        cs[t] = chk.c
                    if not ok:
                        continue
                    Q2 = np.vstack([np.repeat(Q0[nl:nl + 1], d, 0), Q0[nl:]]) if d else Q0[nl:].copy()
                    ph2 = np.concatenate([np.array(["wait"] * d, dtype="<U16"), ph0[nl:]]) if d else ph0[nl:].copy()
                    ph2[0] = "wait" if d else ph2[0]
                    L2 = LegPlan(ri, Lm.target, Lm.opt, Lm.segments[1:], Q2, ph2, Lm.hold_s, Lm.vias, d, dt)
                    sm = steps[m]
                    legs_m = {**sm.legs, ri: L2}
                    other = legs_m.get(rj)
                    if other is None:          # rj stands where it is at step m
                        other = self.stand_q(rj, self._rest_q(steps, rj, m))
                    Em = max(L2.T + L2.hold_s, other.T + other.hold_s)
                    if not chk({ri: L2, rj: other}, Em):
                        # the shortened leg moves earlier than the original: keep the original timing instead
                        # (stand at the retract pose for the lift-off's duration, then the same transit)
                        Q2 = np.vstack([np.repeat(Q0[nl:nl + 1], d + nl, 0), Q0[nl:]])
                        ph2 = np.concatenate([np.array(["wait"] * (d + nl), dtype="<U16"), ph0[nl:]])
                        L2 = LegPlan(ri, Lm.target, Lm.opt, Lm.segments[1:], Q2, ph2, Lm.hold_s, Lm.vias, d + nl, dt)
                        legs_m = {**sm.legs, ri: L2}
                        Em = sm.t_end - sm.t0
                        if not chk({ri: L2, rj: other}, Em):
                            why = why or f"the leg from the retract pose vs {rj} in step {m}"
                            break                       # (the same leg for every s0)
                    cs[m] = chk.c
                    new[m] = replace(sm, legs=legs_m, note=sm.note + "+from_park")
                    applied = ("park_leaf", s0, (sm.t_end - sm.t0) - Em)
                    E_new[m] = Em
                    for t, st in new.items():
                        steps[t] = st
                    for t, c in cs.items():
                        cmin_new[t] = min(cmin_new.get(t, np.inf), c)
                    break
                if applied is None and d > 0:        # (A) the delay after the lift-off, in the same leaf
                    Q2 = np.vstack([Q0[:nl + 1], np.repeat(Q0[nl:nl + 1], d, 0), Q0[nl + 1:]])
                    ph2 = np.concatenate([ph0[:nl + 1], np.array(["wait"] * d, dtype="<U16"), ph0[nl + 1:]])
                    L2 = LegPlan(ri, Lm.target, Lm.opt, Lm.segments, Q2, ph2, Lm.hold_s, Lm.vias, d, dt)
                    sm = steps[m]
                    other = sm.legs.get(rj) or self.stand_q(rj, self._rest_q(steps, rj, m))
                    if chk({ri: L2, rj: other}, sm.t_end - sm.t0):
                        cmin_new[m] = min(cmin_new.get(m, np.inf), chk.c)
                        steps[m] = replace(sm, legs={**sm.legs, ri: L2}, note=sm.note + "+park_in_leg")
                        applied = ("in_leg", m, 0.0)
                    else:
                        why = why or f"the delay at the retract pose vs {rj} in step {m}"
                if applied is not None:
                    rows.append({"robot": ri, "after_weld": Lj.target, "next": Lm.target, "how": applied[0],
                                 "step": applied[1], "step_shortened_s": round(float(applied[2]), 3)})
                elif m - j >= 2 or d > 0:
                    rows.append({"robot": ri, "after_weld": Lj.target, "next": Lm.target, "how": None,
                                 "not_parked": why})
        if not any(r_.get("how") for r_ in rows):
            return steps, rows
        t = 0.0
        out = []
        for k, st in enumerate(steps):
            E = E_new.get(k, st.t_end - st.t0)
            mc = st.min_clearance
            if k in cmin_new:          # the step changed: its clearance from the re-check (both sphere models)
                mc = cmin_new[k]
            out.append(replace(st, t0=t, t_end=t + E, min_clearance=mc))
            t += E
        return out, rows

    def stand_q(self, rid, q):
        """A standing 'leg' at joint configuration q (None = HOME)."""
        q = self.home[rid] if q is None else np.asarray(q, float)
        return LegPlan(rid, "stand", None, [], q[None].copy(), np.array(["wait"], dtype="<U16"), 0.0, dt=self.dt)

    def _rest_q(self, steps, rid, m):
        """Joint configuration where arm rid rests at the start of step m (end of its last leg before m; None =
        HOME)."""
        for t in range(m - 1, -1, -1):
            if rid in steps[t].legs:
                return steps[t].legs[rid].Q[-1]
        return None

    def steps_of(self, st: State) -> list:
        """Executable steps (motion clock) of a final state."""
        out = []
        t = 0.0
        r0, r1 = self.rids
        for rec in st.steps:
            E, dA, dB = rec["chosen"]
            legs = {}
            for r, (k, L) in rec["mv"].items():
                d = dA if r == r0 else dB
                legs[r] = L.with_wait(d)
            if "cmin" not in rec and "chk" in rec:
                rec["cmin"] = self.ck.check_min(*rec["chk"])
            out.append(Step(t, legs, t + E, rec.get("cmin", np.nan), rec["note"]))
            t += E
        return out
