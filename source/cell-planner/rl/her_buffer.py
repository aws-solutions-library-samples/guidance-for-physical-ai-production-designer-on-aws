"""Episode replay buffer with HER ("future", inserted-weld relabelling) and n-step returns.

Storage: each episode of L transitions is stored as L+1 consecutive observation rows
(row t = obs before transition t, row L = final next-obs) plus per-transition action, reward,
terminated and collision. Rows live in a ring buffer; an episode is only sampled while all its
rows are present.

HER relabelling (fraction ``her_ratio`` of a batch). For sample row a, pick a robot r that
has a goal, and a future transition f in [a, seg_end_r - 1], where seg_end_r is the last row
at which r's stored goal is unchanged. The pose r actually reached after f (TCP position,
rotation and entry direction) becomes a **new weld inserted at the front of r's queue**:
  * r's goal (robot_goal, robot_goal_approach, mask) is replaced by it, its dwell counter is
    recomputed from the stored history, and a task node for it is added in a free task slot
    (owner r, status open);
  * once the inserted weld completes, r's goal falls back to the stored one (the original
    queue head), so every stored later observation stays consistent;
  * while it is open, the n-step window is cut before r's stored goal would change.
Rewards, dwell and termination of every step in the window are recomputed with the env's own
``env.rewards.compute_reward`` (open-weld count + 1 for the inserted weld), so relabelled
data obeys exactly the env's reward rule. Non-relabelled samples are recomputed the same way;
a test checks they equal the rewards the env emitted.

n-step returns: b["reward"] = sum_j gamma^j r_j over up to n steps (stopping at a terminal or
episode end), b["next_obs"] = obs after the last step taken, b["disc"] = gamma^steps.
"""
from __future__ import annotations

import numpy as np

from env.rewards import RewardConfig, compute_reward, pose_match

# Keys kept from env observations (others, e.g. derived goal-error features, are dropped:
# the graph builder recomputes everything it needs from these).
STORE_KEYS = ["robot_q", "robot_qd", "robot_base", "robot_tcp_pos", "robot_tcp_rot6", "robot_kp",
              "robot_goal", "robot_goal_mask", "robot_dwell", "robot_goal_approach", "robot_entry",
              "task_pos", "task_rot6", "task_approach", "task_status", "task_owner", "task_rank", "task_mask",
              "obst_pos", "obst_size", "obst_mask", "min_clearance", "time_left",
              "robot_self_dist"]   # optional (EnvConfig.self_dist); kept only when the env provides it
OPTIONAL_ZERO = {"robot_goal_approach": 3, "robot_entry": 3}   # per robot
RANK_STEP = 1.0 / 16                                            # env task_rank = queue position / T_MAX


def clean_obs(obs: dict) -> dict:
    """Keep STORE_KEYS (float32); fill optional keys with zeros if the env does not provide them."""
    out = {k: np.asarray(obs[k], np.float32) for k in STORE_KEYS if k in obs}
    R = np.shape(obs["robot_q"])[-2]
    lead = np.shape(obs["robot_q"])[:-2]
    for k, d in OPTIONAL_ZERO.items():
        if k not in out:
            out[k] = np.zeros(lead + (R, d), np.float32)
    if "task_rank" not in out:   # derived: unknown order -> 0 for open queued tasks, 1 otherwise
        queued = (out["task_owner"].sum(-1) > 0.5) & (out["task_status"] < 0.5)
        out["task_rank"] = np.where(queued, 0.0, 1.0).astype(np.float32)
    if "time_left" not in out:
        out["time_left"] = np.ones(lead + (1,), np.float32)
    return out


def achieved_of(o: dict) -> np.ndarray:
    return np.concatenate([o["robot_tcp_pos"], o["robot_tcp_rot6"], o["robot_entry"]], -1)


def desired_of(o: dict) -> np.ndarray:
    return np.concatenate([o["robot_goal"], o["robot_goal_approach"]], -1)


def n_open(o: dict) -> np.ndarray:
    return ((o["task_mask"] > 0.5) & (o["task_status"] < 0.5)).sum(-1)


class HERReplayBuffer:
    def __init__(self, capacity: int, obs_example: dict, reward_cfg: RewardConfig, her_ratio: float = 0.5,
                 seed: int = 0, n_actions: int = 6, terminate_on_collision: bool = True):
        self.capacity, self.reward_cfg, self.her_ratio = int(capacity), reward_cfg, her_ratio
        self.terminate_on_collision = terminate_on_collision
        self.rng = np.random.default_rng(seed)
        ex = clean_obs(obs_example)
        self.keys = list(ex)
        self.R = ex["robot_q"].shape[0]
        self.D = int(reward_cfg.dwell_steps)
        cap = self.capacity
        self.obs = {k: np.zeros((cap,) + v.shape, np.float32) for k, v in ex.items()}
        self.action = np.zeros((cap, self.R, n_actions), np.float32)
        # BC target per row (Track G, DART-style demos: executed noisy action in `action`, the expert's clean
        # action here). Equals `action` unless add_episode gets bc_action.
        self.bc_action = np.zeros((cap, self.R, n_actions), np.float32)
        self.has_bc = False   # sample() returns "bc_action" only once an episode came with explicit targets
        self.reward = np.zeros(cap, np.float32)
        self.terminated = np.zeros(cap, bool)
        self.collision = np.zeros(cap, np.float32)
        self.row_abs = np.full(cap, -1, np.int64)   # absolute (unwrapped) row index
        self.row_t = np.zeros(cap, np.int64)        # step index in its episode
        self.row_len = np.zeros(cap, np.int64)      # episode length L (transitions)
        self.seg_end = np.zeros((cap, self.R), np.int64)  # abs row: last row with the same goal (per robot)
        self.ptr = 0
        self.n_episodes = 0
        self.n_transitions = 0

    # ------------------------------------------------------------------ storage
    def __len__(self):
        """Number of transitions currently sampleable (approximately)."""
        return int(min(self.n_transitions, self.capacity))

    @property
    def oldest(self):
        return max(0, self.ptr - self.capacity)

    def add_episode(self, obs_seq: dict, action, reward, terminated, collision, bc_action=None):
        """obs_seq: dict of (L+1, ...) arrays (obs before each step + final obs);
        action (L, R, 6) or (L, R*6); reward, terminated, collision (L,); bc_action (L, R, 6) optional
        BC target (default: action)."""
        o = clean_obs(obs_seq)
        L = len(reward)
        assert L >= 1 and L + 1 <= self.capacity, "episode longer than the buffer"
        assert o["robot_q"].shape[0] == L + 1, "obs_seq must have L+1 rows"
        idx = (self.ptr + np.arange(L + 1)) % self.capacity
        for k in self.keys:
            self.obs[k][idx] = o[k]
        a = np.asarray(action, np.float32).reshape(L, self.R, -1)
        self.action[idx[:L]] = a
        self.action[idx[L]] = 0
        self.bc_action[idx[:L]] = a if bc_action is None else np.asarray(bc_action, np.float32).reshape(a.shape)
        self.has_bc |= bc_action is not None
        self.bc_action[idx[L]] = 0
        self.reward[idx[:L]] = reward
        self.reward[idx[L]] = 0
        self.terminated[idx[:L]] = np.asarray(terminated, bool)
        self.terminated[idx[L]] = False
        self.collision[idx[:L]] = collision
        self.collision[idx[L]] = 0
        self.row_abs[idx] = self.ptr + np.arange(L + 1)
        self.row_t[idx] = np.arange(L + 1)
        self.row_len[idx] = L
        # per robot: last row (abs) whose goal equals this row's goal
        g = o["robot_goal"]
        gm = o["robot_goal_mask"]
        same = (np.abs(g[1:] - g[:-1]).max(-1) < 1e-6) & (np.abs(gm[1:] - gm[:-1]) < 0.5)  # (L, R)
        seg = np.empty((L + 1, self.R), np.int64)
        seg[L] = L
        for t in range(L - 1, -1, -1):
            seg[t] = np.where(same[t], seg[t + 1], t)
        self.seg_end[idx] = self.ptr + seg
        self.ptr += L + 1
        self.n_episodes += 1
        self.n_transitions += L
        self._sync_meta()

    def _sample_rows(self, n):
        """Uniform over valid transition rows: t < L and the whole episode still in the buffer."""
        hi = min(self.ptr, self.capacity)
        out = np.empty(n, np.int64)
        todo = np.arange(n)
        for _ in range(100):
            s = self.rng.integers(0, hi, len(todo))
            ok = (self.row_t[s] < self.row_len[s]) & (self.row_abs[s] - self.row_t[s] >= self.oldest) & \
                 (self.row_abs[s] >= 0)
            out[todo[ok]] = s[ok]
            todo = todo[~ok]
            if not len(todo):
                return out
        raise RuntimeError("could not sample valid rows (buffer nearly empty?)")

    def _slot(self, abs_idx):
        return np.asarray(abs_idx) % self.capacity

    def _get(self, slots) -> dict:
        return {k: v[slots] for k, v in self.obs.items()}

    # ------------------------------------------------------------------ sampling
    def sample(self, batch_size: int, n_step: int = 1, gamma: float = 0.98, her_ratio: float | None = None) -> dict:
        cfg, D, R = self.reward_cfg, self.D, self.R
        her_ratio = self.her_ratio if her_ratio is None else her_ratio
        B = batch_size
        s0 = self._sample_rows(B)
        a_abs = self.row_abs[s0]
        ep_last_row = a_abs - self.row_t[s0] + self.row_len[s0]   # abs index of row L
        obs = self._get(s0)

        # ---------------- HER: choose samples, robot, future step
        rel = self.rng.random(B) < her_ratio
        has_goal = obs["robot_goal_mask"] > 0.5                         # (B,R)
        seg = self.seg_end[s0]                                          # (B,R) abs rows
        eligible = has_goal & (seg >= a_abs[:, None] + 1)
        free = obs["task_mask"] < 0.5
        rel &= eligible.any(-1) & free.any(-1)
        r_sel = np.argmax(self.rng.random((B, R)) * eligible, -1)       # random eligible robot
        rows = np.flatnonzero(rel)
        rr = r_sel[rows]
        seg_r = np.full(B, np.iinfo(np.int64).max, np.int64)
        ins_slot = np.zeros(B, np.int64)
        g_des = np.zeros((B, 12), np.float32)
        c = np.zeros(B, np.int64)                                       # dwell of the inserted weld
        if len(rows):
            seg_r[rows] = seg[rows, rr]
            span = seg_r[rows] - a_abs[rows]                            # >= 1: f in [a, seg-1]
            f = a_abs[rows] + (self.rng.random(len(rows)) * span).astype(np.int64)
            fo = self._get(self._slot(f + 1))
            ach = achieved_of(fo)[np.arange(len(rows)), rr]             # (n,12)
            ent = ach[:, 9:12]
            orig_app = obs["robot_goal_approach"][rows, rr]
            app = np.where((np.linalg.norm(ent, axis=-1) > 0.5)[:, None], ent, orig_app)
            g_des[rows] = np.concatenate([ach[:, :9], app], -1)
            ins_slot[rows] = np.argmax(free[rows], -1)
            # dwell before row a under the inserted goal: consecutive matches of rows a, a-1, ...
            # (rows after the episode's reset row), capped at D-1 (it is still open at row a).
            run = np.zeros(len(rows), np.int64)
            alive = np.ones(len(rows), bool)
            ep_start = a_abs[rows] - self.row_t[s0[rows]]
            for k in range(D - 1):
                ab = a_abs[rows] - k
                alive &= ab >= ep_start + 1
                if not alive.any():
                    break
                ok = pose_match(cfg, achieved_of(self._get(self._slot(np.maximum(ab, 0))))[np.arange(len(rows)), rr],
                                g_des[rows])
                alive &= ok
                run += alive
            c[rows] = run
            self._apply_goal(obs, rows, rr, g_des[rows], c[rows], ins_slot[rows], done=np.zeros(len(rows), bool))

        # ---------------- n-step return under the (relabelled) goals
        ret = np.zeros(B, np.float64)
        term = np.zeros(B, bool)
        active = np.ones(B, bool)
        done_ins = np.zeros(B, bool)
        steps = np.zeros(B, np.int64)
        last_next = a_abs + 1
        for j in range(n_step):
            t_abs = a_abs + j
            if j > 0:
                active &= ~term & (t_abs < ep_last_row)
            # an open inserted weld must not see r's stored goal change
            active &= ~(rel & ~done_ins & (t_abs >= seg_r))
            if not active.any():
                break
            cur = self._get(self._slot(t_abs))
            nxt = self._get(self._slot(t_abs + 1))
            des = desired_of(cur)
            gmask = cur["robot_goal_mask"].copy()
            dwell = np.rint(cur["robot_dwell"] * D).astype(np.int64)
            nleft = n_open(cur)
            ins_open = rel & ~done_ins
            if ins_open.any():
                ii = np.flatnonzero(ins_open)
                des[ii, r_sel[ii]] = g_des[ii]
                gmask[ii, r_sel[ii]] = 1.0
                dwell[ii, r_sel[ii]] = c[ii]
                nleft = nleft + ins_open
            out = compute_reward(cfg, achieved_of(nxt), des, gmask, dwell, self.collision[self._slot(t_abs)],
                                 n_left=nleft, achieved_prev=achieved_of(cur), clearance=nxt["min_clearance"][:, 0],
                                 self_dist=nxt["robot_self_dist"][..., 0] if "robot_self_dist" in nxt else None)
            ret += np.where(active, gamma ** j * out.reward, 0.0)
            ar = np.arange(B)
            newly = ins_open & out.completed[ar, r_sel] & active
            c = np.where(ins_open & active, out.dwell[ar, r_sel], c)
            done_ins |= newly
            t_out = out.terminated if self.terminate_on_collision else \
                (out.terminated & ~(self.collision[self._slot(t_abs)] > 0.5))
            term |= active & t_out
            last_next = np.where(active, t_abs + 1, last_next)
            steps += active

        nobs = self._get(self._slot(last_next))
        if len(rows):
            self._apply_goal(nobs, rows, rr, g_des[rows], c[rows], ins_slot[rows], done=done_ins[rows])
        out_bc = {"bc_action": self.bc_action[s0].copy()} if self.has_bc else {}
        return {**out_bc, "obs": obs, "next_obs": nobs, "action": self.action[s0].copy(),
                "reward": ret.astype(np.float32), "terminated": term.astype(np.float32),
                "disc": (gamma ** steps).astype(np.float32), "relabelled": rel, "steps": steps,
                # diagnostics (tests): sampled abs row, relabelled robot and inserted goal
                "row_abs": a_abs, "her_robot": r_sel, "her_goal": g_des, "her_done": done_ins}

    def _apply_goal(self, o, rows, rr, g, dwell, slot, done):
        """Write the inserted weld into obs dict o (in place) for batch rows `rows`, robot rr."""
        D = self.D
        op = ~done
        ro, ra = rows[op], rr[op]
        o["robot_goal"][ro, ra] = g[op, :9]
        o["robot_goal_approach"][ro, ra] = g[op, 9:12]
        o["robot_goal_mask"][ro, ra] = 1.0
        o["robot_dwell"][ro, ra] = dwell[op] / D
        o["task_pos"][rows, slot] = g[:, :3]
        o["task_rot6"][rows, slot] = g[:, 3:9]
        o["task_approach"][rows, slot] = g[:, 9:12]
        o["task_status"][rows, slot] = done.astype(np.float32)
        o["task_owner"][rows, slot] = 0.0
        o["task_owner"][rows, slot, rr] = 1.0
        o["task_mask"][rows, slot] = 1.0
        # queue ranks: while open, the inserted weld is r's head and r's other open welds move back
        own_open = (o["task_owner"][ro, :, ra] > 0.5) & (o["task_rank"][ro] < 1.0 - 1e-6)
        o["task_rank"][ro] = np.where(own_open, o["task_rank"][ro] + RANK_STEP, o["task_rank"][ro])
        o["task_rank"][rows, slot] = np.where(done, 1.0, 0.0)

    # ------------------------------------------------------------------ persistence
    def state_dict(self) -> dict:
        n = min(self.ptr, self.capacity)
        d = {f"obs.{k}": v[:n] for k, v in self.obs.items()}
        d.update(action=self.action[:n], bc_action=self.bc_action[:n], has_bc=np.array(self.has_bc), reward=self.reward[:n], terminated=self.terminated[:n],
                 collision=self.collision[:n], row_abs=self.row_abs[:n], row_t=self.row_t[:n],
                 row_len=self.row_len[:n], seg_end=self.seg_end[:n],
                 meta=np.array([self.ptr, self.n_episodes, self.n_transitions, self.capacity], np.int64))
        return d

    def load_state_dict(self, d: dict):
        ptr, n_ep, n_tr, cap = (int(x) for x in d["meta"])
        assert cap == self.capacity, f"snapshot capacity {cap} != buffer capacity {self.capacity}"
        n = len(d["reward"])
        for k in self.keys:
            self.obs[k][:n] = d[f"obs.{k}"]
        for k in ("action", "reward", "terminated", "collision", "row_abs", "row_t", "row_len", "seg_end"):
            getattr(self, k)[:n] = d[k]
        self.bc_action[:n] = d["bc_action"] if "bc_action" in d else d["action"]
        self.has_bc = bool(d["has_bc"]) if "has_bc" in d else False
        self.ptr, self.n_episodes, self.n_transitions = ptr, n_ep, n_tr
        self._sync_meta()

    # ------------------------------------------------------------------ shared memory (Track G sampler processes)
    _ARRAYS = ("action", "bc_action", "reward", "terminated", "collision", "row_abs", "row_t", "row_len", "seg_end")

    def _sync_meta(self):
        m = getattr(self, "_meta", None)
        if m is not None:   # counters last: readers only sample rows below ptr (fully written)
            m[1], m[2], m[3] = self.n_episodes, self.n_transitions, int(self.has_bc)
            m[0] = self.ptr

    def share_memory(self) -> dict:
        """Move all arrays into POSIX shared memory (this process stays the only writer). Returns a picklable
        spec for `HERReplayBuffer.attach` in sampler processes. Idempotent."""
        if getattr(self, "_spec", None) is not None:
            return self._spec
        import atexit
        from dataclasses import asdict
        from multiprocessing import shared_memory
        self._shm = []
        spec = dict(capacity=self.capacity, reward_cfg=asdict(self.reward_cfg), her_ratio=self.her_ratio,
                    terminate_on_collision=self.terminate_on_collision, R=self.R, D=self.D, keys=self.keys,
                    arrays={})

        def mv(name, arr):
            shm = shared_memory.SharedMemory(create=True, size=max(8, arr.nbytes))
            new = np.ndarray(arr.shape, arr.dtype, buffer=shm.buf)
            new[...] = arr
            self._shm.append(shm)
            spec["arrays"][name] = (shm.name, arr.shape, arr.dtype.str)
            return new
        for k in list(self.obs):
            self.obs[k] = mv("obs." + k, self.obs[k])
        for k in self._ARRAYS:
            setattr(self, k, mv(k, getattr(self, k)))
        self._meta = mv("_meta", np.zeros(4, np.int64))
        self._sync_meta()
        self._spec = spec
        atexit.register(self.close_shared)
        return spec

    def close_shared(self):
        for shm in getattr(self, "_shm", []):
            try:
                shm.close()
                if not getattr(self, "_attached", False):
                    shm.unlink()
            except Exception:
                pass
        self._shm = []

    @classmethod
    def attach(cls, spec: dict, seed: int = 0) -> "HERReplayBuffer":
        """Read-only view of a shared buffer (in another process); call refresh() before sampling."""
        from multiprocessing import shared_memory
        self = cls.__new__(cls)
        self.capacity, self.her_ratio = spec["capacity"], spec["her_ratio"]
        self.reward_cfg = RewardConfig(**spec["reward_cfg"])
        self.terminate_on_collision, self.R, self.D, self.keys = (spec["terminate_on_collision"], spec["R"],
                                                                 spec["D"], list(spec["keys"]))
        self.rng = np.random.default_rng(seed)
        self._shm, self._attached, self.obs = [], True, {}
        for name, (shm_name, shape, dt) in spec["arrays"].items():
            shm = shared_memory.SharedMemory(name=shm_name)
            # (spawned children share the creator's resource tracker; the creator unlinks at exit)
            self._shm.append(shm)
            arr = np.ndarray(tuple(shape), np.dtype(dt), buffer=shm.buf)
            if name.startswith("obs."):
                self.obs[name[4:]] = arr
            else:
                setattr(self, name, arr)
        self.refresh()
        return self

    def refresh(self):
        m = self._meta
        self.ptr = int(m[0])
        self.n_episodes, self.n_transitions, self.has_bc = int(m[1]), int(m[2]), bool(m[3])

    def save(self, path):
        import os
        tmp = str(path) + ".tmp.npz"
        np.savez(tmp, **self.state_dict())
        os.replace(tmp, path)

    def load(self, path):
        with np.load(path) as z:
            self.load_state_dict({k: z[k] for k in z.files})


class EpisodeRecorder:
    """Accumulates one env's transitions; flush() returns add_episode kwargs."""

    def __init__(self):
        self.reset()

    def reset(self, obs=None):
        self.obs, self.act, self.rew, self.term, self.coll = [], [], [], [], []
        if obs is not None:
            self.obs.append(clean_obs(obs))

    def add(self, action, reward, terminated, collision, next_obs):
        self.act.append(np.asarray(action, np.float32))
        self.rew.append(float(reward))
        self.term.append(bool(terminated))
        self.coll.append(float(collision))
        self.obs.append(clean_obs(next_obs))

    def __len__(self):
        return len(self.rew)

    def flush(self, next_start_obs=None) -> dict:
        ep = dict(obs_seq={k: np.stack([o[k] for o in self.obs]) for k in self.obs[0]},
                  action=np.stack(self.act), reward=np.array(self.rew, np.float32),
                  terminated=np.array(self.term), collision=np.array(self.coll, np.float32))
        self.reset(next_start_obs)
        return ep
