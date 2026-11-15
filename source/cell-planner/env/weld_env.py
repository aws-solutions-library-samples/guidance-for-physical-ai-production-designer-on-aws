"""Gymnasium env: two KUKA KR240 stick-tool arms spot-welding the umarell car-door program.

Kinematic simulation at 20 Hz: ``q += clip(a, -1, 1) * v_max * velocity_scale * dt`` (clipped to
the joint limits). Each robot works through an ordered weld queue; the head of the queue is its
goal. A weld completes when the umarell pose check (``env.rewards.pose_match``) holds for
``dwell_steps`` consecutive steps.

Collision verdicts (any one terminates the episode):
  arm_arm  umarell sphere model clearance <= warning_distance 0.20 m (``env.kinematics.SphereModel``),
           or a MuJoCo hull contact between the two arms
  self     MuJoCo convex-hull contact within one arm (SRDF-adjacent pairs excluded), dist < -contact_tol
  floor    MuJoCo hull contact with the floor plane, or umarell's floor rule (sphere centres z < -0.05)
  fixture  umarell's box rule (sphere model) against the cell's obstacle boxes (welding_cell_2t)
           (+ MuJoCo hull contacts with the boxes as geoms when EnvConfig.fixture_hull=True; default off)
The motion between two control steps is sub-sampled (``n_substeps``) for both checks.

Observation: see ``KukaWeldEnv.observation_space`` / docs/DESIGN.md (Track E).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from .cell import N_JOINTS, ROBOT_IDS, ROOT, Cell, load_cell
from .rewards import RewardConfig, compute_reward, goal_features, mat_to_rot6, pose_errors, quat_xyzw_to_mat
from .scene import JOINTS, body_name, build_model, scene_option

R = len(ROBOT_IDS)
T_MAX = 16
O_MAX = 16
# keypoints: shoulder (link_2), mid upper arm, elbow (link_3), mid forearm, wrist (link_4/5/6 origin),
# then points on the tool in the link_6 frame: RAH frame, side bracket, electrode bar, TCP.
TOOL_KP_LOCAL = np.array([[-0.288, 0.004, 0.032], [-0.281, 0.009, 0.567], [-0.859, 0.269, -0.005]])
N_KP = 9

UMARELL_QUEUES = {  # plan bt_1790851970 (execution order per robot)
    "robot1": ["W01", "W02", "W03", "W06", "W05", "W04"],
    "robot2": ["W13", "W15", "W14", "W10", "W11", "W12", "W09", "W08", "W07"],
}
KIND_NAMES = ("arm_arm", "self", "floor", "fixture")
# hull self-distance pairs: all non-adjacent body pairs of one arm, except link_3/link_5 (structurally
# always ~ 2-10 cm apart, never in contact in 2000 random configurations). Groups of robot_self_dist:
#   0 overall min, 1 tool + link_6 vs base..link_3 (the link_3/RAH-frame contacts of the umarell plans),
#   2 link_4/link_5 vs base..link_2 (folding the forearm onto the upper arm)
SELF_SKIP = {("link_3", "link_5")}
SELF_TOOL_ARM = ({"link_6"}, {"base_link", "link_1", "link_2", "link_3"})
SELF_FORE_ARM = ({"link_4", "link_5"}, {"base_link", "link_1", "link_2"})


@dataclass
class EnvConfig:
    cell_id: str = "welding_cell_2_small"
    ctrl_dt: float = 0.05
    velocity_scale: float | None = None      # None = cell planning.max_velocity_scaling (0.7)
    episode_len: int = 2000
    assignment: str | dict = "umarell"        # "umarell" | "greedy" | {rid: [wid, ...]}
    # randomisation (ignored with reset(options={"canonical": True}))
    n_welds: tuple[int, int] = (1, 15)        # inclusive range of the weld subset size
    pos_noise: float = 0.03                   # uniform +- per axis (m)
    rot_noise_deg: float = 0.0                # random tilt of weld_orientation (deg)
    start_noise: float = 0.1                  # N(0, sigma) rad around HOME
    shuffle_prob: float = 0.5                 # prob. of shuffling each robot's queue
    active_robots: tuple[str, ...] = ROBOT_IDS  # robots that get welds (curriculum: one arm)
    # collision
    n_substeps: int = 2                       # sub-samples of each control step for collision checks
    contact_tol: float = 0.0                  # MuJoCo contact counts if dist < -contact_tol
    check_self: bool = True
    check_floor: bool = True
    approach_span: float = 0.04               # umarell validate_approach approach_span_m
    history_len: int = 400                    # TCP history kept per robot for the entry direction
    # strict entry check (off by default, docs/strict_checks.md): the entry walk-back sees the FULL TCP path since the
    # arm's previous weld completion (or the episode start) instead of the last history_len states (umarell
    # validate_approach walk-back on an un-truncated leg; docs/verification_ppo_v3.md V3-2)
    entry_full_history: bool = False
    # hull-vs-fixture check (off by default, docs/p3_fixhull.md): the simulation model also gets the cell's obstacle
    # boxes (welding_cell_2t: 737) as MuJoCo box geoms, so a convex arm/tool hull penetrating a box (dist <
    # -contact_tol) is a "fixture" collision, in addition to umarell's sphere box rule (which skips the tool spheres;
    # docs/verification_ppo_2t.md P2T-3). No effect on cells without boxes; obs unchanged.
    fixture_hull: bool = False
    terminate_on_collision: bool = True
    # Track G: MuJoCo hull self-distances per arm (obs robot_self_dist (2, 3), used by RewardConfig.self_coef).
    # Off by default (obs dict unchanged). Computed automatically when reward.self_coef > 0.
    self_dist: bool = False
    self_dist_max: float = 0.3                # mj_geomDistance cut-off (m); larger distances read as this
    # Track PPO v2 (off by default): door-panel workpiece (env/workpiece.py) -> collision kind "workpiece" and
    # obs robot_wp_dist (2, 2) [tool, rest of arm] min hull distance to the panel (capped at dist_max)
    workpiece: bool = False
    workpiece_cfg: dict = field(default_factory=dict)   # env.workpiece.WorkpieceCfg overrides
    workpiece_soft: bool = False   # training aid: penetrations are not collisions; info["workpiece_depth"] reports them
    wedge: bool = False                       # certain-material wedge rule (docs/workpiece_decision.md, env/wedge.py)
    wedge_cfg: dict = field(default_factory=dict)   # WedgeCfg overrides
    wedge_obs: bool = False                   # add obs robot_wedge_margin (2,)
    # Track PPO v3 (off by default): a weld's dwell only counts while the TCP stays within this distance (m) of the
    # pose where the dwell started (a still hold); leaving it restarts the dwell
    dwell_still_tol: float | None = None
    # Track PPO v2 (off by default): after the last weld both arms must return to HOME (max |q - home| <= home_tol)
    # before the episode succeeds/terminates; +home_bonus on arrival
    require_home: bool = False
    home_tol: float = 0.01
    home_bonus: float = 1.0
    reward: RewardConfig = field(default_factory=RewardConfig)
    render_size: tuple[int, int] = (544, 960)  # (height, width)
    render_camera: str = "overview"
    render_fixture: str = "auto"              # "auto" (mesh if the cell has a fixture) | "none" | "mesh" | "boxes"


def _box_space(*shape, lo=-np.inf, hi=np.inf):
    return spaces.Box(lo, hi, shape=shape, dtype=np.float32)


def merge_boxes(centers: np.ndarray, halfs: np.ndarray, k: int = O_MAX, iters: int = 30):
    """Cluster boxes into <= k AABBs (graph nodes only). Deterministic k-means on centres."""
    n = len(centers)
    if n == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))
    if n <= k:
        return centers.copy(), halfs.copy()
    rng = np.random.default_rng(0)
    cen = centers[rng.choice(n, k, replace=False)].copy()
    for _ in range(iters):
        lab = np.argmin(((centers[:, None] - cen[None]) ** 2).sum(-1), 1)
        for j in range(k):
            if np.any(lab == j):
                cen[j] = centers[lab == j].mean(0)
    lo, hi = centers - halfs, centers + halfs
    oc, oh = [], []
    for j in range(k):
        m = lab == j
        if m.any():
            a, b = lo[m].min(0), hi[m].max(0)
            oc.append((a + b) / 2), oh.append((b - a) / 2)
    return np.array(oc), np.array(oh)


def load_reach(cell_id: str) -> dict | None:
    """{rid: [wid]} from data/weld_ik*.json (Track X), or None."""
    p = ROOT / "data" / ("weld_ik.json" if cell_id == "welding_cell_2_small" else f"weld_ik_{cell_id}.json")
    if not p.exists():
        return None
    try:
        d = json.load(open(p))["reach"]
    except Exception:
        return None
    from robotmodel.stamp import check_table           # warns if the table was solved for another robot model
    check_table(p, cell_id, "reachability (IK) table")
    return d


class KukaWeldEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 20}

    def __init__(self, config: EnvConfig | None = None, render_mode: str | None = None, **overrides):
        cfg = config or EnvConfig()
        for k, v in overrides.items():
            if not hasattr(cfg, k):
                raise TypeError(f"unknown EnvConfig field {k}")
            setattr(cfg, k, v)
        self.cfg = cfg
        self.render_mode = render_mode
        self.cell: Cell = load_cell(cfg.cell_id)
        self.vscale = cfg.velocity_scale if cfg.velocity_scale is not None else self.cell.velocity_scale
        self.dt = cfg.ctrl_dt

        # Simulation model: hulls only (fast). Rendering uses its own visual model (lazy).
        fx_hull = bool(cfg.fixture_hull) and len(self.cell.boxes[0]) > 0
        fx_kw = dict(fixture="boxes", fixture_contacts=True) if fx_hull else dict(fixture="none")
        if cfg.workpiece:
            from .scene import build_spec
            from .workpiece import add_workpiece_geoms
            spec = build_spec(self.cell, visual=False, weld_sites=False, **fx_kw)
            add_workpiece_geoms(spec)
            self.model = spec.compile()
        else:
            self.model = build_model(self.cell, visual=False, weld_sites=False, **fx_kw)
        self.data = mujoco.MjData(self.model)
        self._scratch = mujoco.MjData(self.model)
        self._index_model()
        self.wp = None
        self._wp_d = None
        from .wedge import Wedge
        self.wedge = Wedge(cfg.wedge_cfg, ROBOT_IDS) if cfg.wedge else None
        self._dwell_anchor = np.zeros((R, 3))
        if cfg.workpiece:
            from .workpiece import Workpiece
            self.wp = Workpiece(self.model, self.geom_owner, self.tcp_site, cfg.workpiece_cfg)

        from .kinematics import SphereModel  # Track K: umarell-identical sphere model
        self.spheres = SphereModel(self.cell)
        self.has_boxes = len(self.spheres.box_center) > 0
        self._fast_spheres = hasattr(self.spheres, "pair_min_from_spheres")
        if self._fast_spheres:
            self._floor_idx = [np.asarray(self.spheres.sphere_groups(r)["floor_idx"]) for r in ROBOT_IDS]

        rc = [self.cell.robots[r] for r in ROBOT_IDS]
        self.home = np.stack([r.home for r in rc])
        self.q_lo = np.stack([r.q_lo for r in rc])
        self.q_hi = np.stack([r.q_hi for r in rc])
        self.v_max = np.stack([r.v_max for r in rc]) * self.vscale
        self.robot_base = np.array([[*r.base, r.yaw] for r in rc], np.float32)
        self.wids = [w.wid for w in self.cell.welds]
        self.weld_by_id = {w.wid: w for w in self.cell.welds}
        self.reach = load_reach(cfg.cell_id)
        oc, oh = merge_boxes(*self.cell.boxes)
        self._obst = np.zeros((O_MAX, 3), np.float32)
        self._obst_size = np.zeros((O_MAX, 3), np.float32)
        self._obst_mask = np.zeros(O_MAX, np.float32)
        self._obst[:len(oc)], self._obst_size[:len(oc)], self._obst_mask[:len(oc)] = oc, oh, 1

        T = T_MAX
        self.observation_space = spaces.Dict(
            robot_q=_box_space(R, N_JOINTS), robot_qd=_box_space(R, N_JOINTS), robot_base=_box_space(R, 4),
            robot_tcp_pos=_box_space(R, 3), robot_tcp_rot6=_box_space(R, 6, lo=-1, hi=1),
            robot_kp=_box_space(R, N_KP, 3),
            robot_goal=_box_space(R, 9), robot_goal_mask=_box_space(R, lo=0, hi=1),
            robot_goal_approach=_box_space(R, 3, lo=-1, hi=1),
            robot_goal_err=_box_space(R, 6), robot_goal_errmag=_box_space(R, 3, lo=0),
            robot_entry=_box_space(R, 3, lo=-1, hi=1),
            robot_dwell=_box_space(R, lo=0, hi=1),
            task_pos=_box_space(T, 3), task_rot6=_box_space(T, 6, lo=-1, hi=1), task_approach=_box_space(T, 3, lo=-1, hi=1),
            task_status=_box_space(T, lo=0, hi=1), task_owner=_box_space(T, R, lo=0, hi=1),
            task_rank=_box_space(T, lo=0, hi=1), task_mask=_box_space(T, lo=0, hi=1),
            obst_pos=_box_space(O_MAX, 3), obst_size=_box_space(O_MAX, 3, lo=0), obst_mask=_box_space(O_MAX, lo=0, hi=1),
            min_clearance=_box_space(1), time_left=_box_space(1, lo=0, hi=1),
        )
        if cfg.self_dist:
            self.observation_space.spaces["robot_self_dist"] = _box_space(R, 3, lo=-1, hi=cfg.self_dist_max)
        if cfg.workpiece:
            self.observation_space.spaces["robot_wp_dist"] = _box_space(R, 2, lo=-1, hi=1)
        if cfg.wedge and cfg.wedge_obs:
            self.observation_space.spaces["robot_wedge_margin"] = _box_space(R, lo=-1, hi=1)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(R * N_JOINTS,), dtype=np.float32)
        self._renderer = None
        self._rmodel = self._rdata = None
        self.reset(options={"canonical": True})

    # ------------------------------------------------------------------ model
    def _index_model(self):
        m = self.model
        self.qadr = np.array([[m.jnt_qposadr[m.joint(f"{rid}/{j}").id] for j in JOINTS] for rid in ROBOT_IDS])
        self.tcp_site = np.array([m.site(f"{rid}/tcp").id for rid in ROBOT_IDS])
        self.kp_body = np.array([[m.body(body_name(rid, b)).id for b in ("link_2", "link_3", "link_4", "link_6")]
                                 for rid in ROBOT_IDS])
        owner = np.full(m.ngeom, -1, np.int64)  # robot index, -2 floor, -3 fixture, -1 other
        for g in range(m.ngeom):
            bname = m.body(m.geom_bodyid[g]).name
            for i, rid in enumerate(ROBOT_IDS):
                if bname.startswith(rid + "/"):
                    owner[g] = i
            if m.geom(g).name == "floor":
                owner[g] = -2
            elif m.geom(g).name.startswith("fixbox"):
                owner[g] = -3
        self.geom_owner = owner
        # self-distance geom pairs per arm (see SELF_SKIP / SELF_* groups)
        ex = set()
        for k in range(m.nexclude):
            sig = int(m.exclude_signature[k])
            ex |= {(sig >> 16, sig & 0xFFFF), (sig & 0xFFFF, sig >> 16)}

        def short(g):  # 'robot1/like_kr240r2900_link_3' -> 'link_3'
            n = m.body(m.geom_bodyid[g]).name.split("/")[-1]
            return n[n.index("link"):] if "link" in n else n

        self._self_pairs = []
        for i in range(R):
            gs = [g for g in range(m.ngeom) if owner[g] == i and m.geom_contype[g]]
            pairs, grp = [], []
            for x in range(len(gs)):
                for y in range(x + 1, len(gs)):
                    a, b = gs[x], gs[y]
                    ba, bb = m.geom_bodyid[a], m.geom_bodyid[b]
                    na, nb = short(a), short(b)
                    if ba == bb or (ba, bb) in ex or (na, nb) in SELF_SKIP or (nb, na) in SELF_SKIP:
                        continue
                    g1 = (na in SELF_TOOL_ARM[0] and nb in SELF_TOOL_ARM[1]) or (nb in SELF_TOOL_ARM[0] and na in SELF_TOOL_ARM[1])
                    g2 = (na in SELF_FORE_ARM[0] and nb in SELF_FORE_ARM[1]) or (nb in SELF_FORE_ARM[0] and na in SELF_FORE_ARM[1])
                    pairs.append((a, b))
                    grp.append((g1, g2))
            self._self_pairs.append((pairs, np.array(grp, bool).reshape(-1, 2)))
        self._fromto = np.zeros(6)

    # ------------------------------------------------------------------ collisions
    def _mj_contacts(self, d) -> dict:
        """Classify penetrating MuJoCo contacts (dist < -contact_tol) of the current d.contact."""
        out = {"self": [], "floor": [], "arm_arm": [], "fixture": []}
        n = d.ncon
        if n == 0:
            return out
        con = d.contact
        dist = con.dist[:n]
        keep = dist < -self.cfg.contact_tol
        if not keep.any():
            return out
        geoms = con.geom[:n][keep]
        o = self.geom_owner[geoms]
        for (g1, g2), (a, b), dd in zip(geoms, o, dist[keep]):
            if a >= 0 and b >= 0:
                k = "self" if a == b else "arm_arm"
            elif (a == -2 and b >= 0) or (b == -2 and a >= 0):
                k = "floor"
            elif (a == -3 and b >= 0) or (b == -3 and a >= 0):
                k = "fixture"
            else:
                continue
            out[k].append((self.model.body(self.model.geom_bodyid[g1]).name,
                           self.model.body(self.model.geom_bodyid[g2]).name, float(dd)))
        return out

    def _sphere_terms(self, q):
        """q (..., 2, 6) -> clearance (...), floor margins (..., 2) or None, box margins (..., 2) or None.

        Fast path: sphere centres computed once per arm (``SphereModel.spheres``), then
        ``pair_min_from_spheres`` and umarell's floor rule (centres of ``sphere_groups()["floor_idx"]``
        at z >= FLOOR_Z_MIN). ``tests/test_env.py`` checks it equals ``pair_clearance`` /
        ``obstacle_terms``. Boxes always go through the public ``obstacle_terms``.
        """
        sm = self.spheres
        floor = box = None
        if self._fast_spheres:
            from .kinematics import FLOOR_Z_MIN
            S1, S2 = sm.spheres(ROBOT_IDS[0], q[..., 0, :]), sm.spheres(ROBOT_IDS[1], q[..., 1, :])
            clr = sm.pair_min_from_spheres(S1, S2)[0]
            if self.cfg.check_floor:
                floor = np.stack([S1[..., self._floor_idx[0], 2].min(-1), S2[..., self._floor_idx[1], 2].min(-1)],
                                 -1) - FLOOR_Z_MIN
        else:
            clr = sm.pair_clearance(q[..., 0, :], q[..., 1, :])
            if self.cfg.check_floor:
                floor = np.stack([sm.obstacle_terms(rid, q[..., i, :])["floor"] for i, rid in enumerate(ROBOT_IDS)], -1)
        if self.has_boxes:
            box = np.stack([sm.obstacle_terms(rid, q[..., i, :])["box"] for i, rid in enumerate(ROBOT_IDS)], -1)
        return clr, floor, box

    def _self_dist_from(self, d) -> np.ndarray:
        """(R, 3) hull self-distances [min, tool/link_6 vs base..link_3, link_4/5 vs base..link_2] (m),
        capped at cfg.self_dist_max; negative = penetration depth (d after mj_kinematics)."""
        dm = self.cfg.self_dist_max
        out = np.full((R, 3), dm)
        for i, (pairs, grp) in enumerate(self._self_pairs):
            if not pairs:
                continue
            dist = np.array([mujoco.mj_geomDistance(self.model, d, a, b, dm, self._fromto) for a, b in pairs])
            dist = np.minimum(dist, dm)
            out[i, 0] = dist.min()
            if grp[:, 0].any():
                out[i, 1] = dist[grp[:, 0]].min()
            if grp[:, 1].any():
                out[i, 2] = dist[grp[:, 1]].min()
        return out

    def self_distance(self, q12) -> np.ndarray:
        """(R, 3) hull self-distances (see _self_dist_from) of a configuration; env state untouched."""
        d = self._scratch
        d.qpos[self.qadr] = np.asarray(q12, float).reshape(R, N_JOINTS)
        mujoco.mj_kinematics(self.model, d)
        return self._self_dist_from(d)

    @property
    def _want_self_dist(self) -> bool:
        return bool(self.cfg.self_dist or getattr(self.cfg.reward, "self_coef", 0.0))

    def _verdict(self, clr, floor, box, mj) -> list[str]:
        """Collision kinds present (priority order arm_arm, self, floor, fixture)."""
        kinds = []
        if clr <= self.spheres.warning_distance or mj["arm_arm"]:
            kinds.append("arm_arm")
        if mj["self"]:
            kinds.append("self")
        if mj["floor"] or (floor is not None and np.any(floor < 0)):
            kinds.append("floor")
        if mj.get("workpiece"):
            kinds.append("workpiece")
        if mj["fixture"] or (box is not None and np.any(box < 0)):
            kinds.append("fixture")
        return kinds

    def check_collisions(self, q12) -> dict:
        """Full collision report for one configuration (no stepping, env state untouched)."""
        q = np.asarray(q12, float).reshape(R, N_JOINTS)
        d = self._scratch
        d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.model, d)
        mujoco.mj_collision(self.model, d)
        mj = (self._mj_contacts(d) if (self.cfg.check_self or self.cfg.check_floor or self.cfg.fixture_hull)
              else {k: [] for k in KIND_NAMES})
        if not self.cfg.check_self:
            mj["self"] = []
        if self.wp is not None:
            mj["workpiece"] = self.wp.distances(d)[1]
        if self.wedge is not None:
            x = d.site_xpos[self.tcp_site].copy()
            if self.wedge.electrode:
                Rx = d.site_xmat[self.tcp_site].reshape(R, 3, 3).copy()
                mj["workpiece"] = list(mj.get("workpiece", [])) + self.wedge.check(x, x, Rx, Rx)[1]
            else:
                mj["workpiece"] = list(mj.get("workpiece", [])) + self.wedge.check(x, x)[1]
        clr, floor, box = self._sphere_terms(q)
        kinds = self._verdict(float(clr), floor, box, mj)
        return dict(collision=bool(kinds), kind=kinds[0] if kinds else None, kinds=kinds, min_clearance=float(clr),
                    floor_margin=None if floor is None else floor.astype(float),
                    box_margin=None if box is None else box.astype(float), contacts=mj)

    # ------------------------------------------------------------------ kinematics
    def _fk(self, d):
        """TCP pos (2,3), rot (2,3,3), keypoints (2,K,3) from d (after mj_kinematics)."""
        p = d.site_xpos[self.tcp_site].copy()
        Rm = d.site_xmat[self.tcp_site].reshape(R, 3, 3).copy()
        b = d.xpos[self.kp_body]  # (2,4,3) link_2, link_3, link_4 (wrist), link_6
        R6 = d.xmat[self.kp_body[:, 3]].reshape(R, 3, 3)
        tool = b[:, 3, None, :] + np.einsum("rij,kj->rki", R6, TOOL_KP_LOCAL)
        kp = np.concatenate([b[:, 0:1], 0.5 * (b[:, 0:1] + b[:, 1:2]), b[:, 1:2], 0.5 * (b[:, 1:2] + b[:, 2:3]),
                             b[:, 2:3], tool, p[:, None]], 1)
        return p, Rm, kp

    def set_q(self, q12):
        """Teleport both arms (no collision check, no task progress). Updates kinematics/obs state."""
        q = np.clip(np.asarray(q12, float).reshape(R, N_JOINTS), self.q_lo, self.q_hi)
        self.data.qpos[self.qadr] = q
        self.data.qvel[:] = 0
        mujoco.mj_kinematics(self.model, self.data)
        self._q = q
        self._qd = np.zeros_like(q)
        self._tcp, self._tcp_R, self._kp = self._fk(self.data)
        self._self_d = self._self_dist_from(self.data) if self._want_self_dist else None
        self._wp_d = None
        self._clear_history()
        self._update_achieved()
        self._min_clr = float(self.spheres.pair_clearance(q[0], q[1]))
        self._min_clr_sub = self._min_clr
        return self._obs()

    # ------------------------------------------------------------------ tasks
    def _base_assignment(self, wids: list[str]) -> dict:
        a = self.cfg.assignment
        if isinstance(a, dict):
            return {rid: [w for w in a.get(rid, []) if w in wids] for rid in ROBOT_IDS}
        if a == "umarell":
            return {rid: [w for w in UMARELL_QUEUES[rid] if w in wids] for rid in ROBOT_IDS}
        if a == "greedy":
            return self._greedy(wids)
        raise ValueError(f"unknown assignment {a!r}")

    def _shoulder_heights(self):
        """Height of joint 2's origin above the base, per robot, from the URDF (P3U round 2). The greedy split
        historically used 0.86 m (the KR240 R2900's 0.8606 m, rounded); it is kept where the URDF is within 1 cm of
        it, so the shipped cells' splits are bit-identical, and the URDF's value is used for any other robot."""
        if getattr(self, "_shoulder_h", None) is None:
            from .kinematics import ArmKinematics
            out = []
            for r in ROBOT_IDS:
                try:
                    _, ps = ArmKinematics(self.cell.robots[r]).joint_poses_local(np.zeros(N_JOINTS))
                    z = float(ps[min(2, len(ps) - 1)][2])
                except Exception:  # noqa: BLE001
                    z = 0.86
                out.append(0.86 if abs(z - 0.86) <= 0.01 else z)
            self._shoulder_h = out
        return self._shoulder_h

    def _greedy(self, wids):
        """Balanced, reachability-aware split + nearest-neighbour order from the HOME TCP.

        Reachability from data/weld_ik.json (Track X) when it exists, else the umarell split.
        """
        reach = self.reach or {rid: UMARELL_QUEUES[rid] for rid in ROBOT_IDS}
        shoulder = self.robot_base[:, :3] + np.array([[0, 0, z] for z in self._shoulder_heights()])
        cnt = {rid: 0 for rid in ROBOT_IDS}
        own = {}
        cap = int(np.ceil(len(wids) / R)) + 1
        # most constrained welds first
        order = sorted(wids, key=lambda w: sum(w in reach.get(r, []) for r in ROBOT_IDS))
        for w in order:
            pos = self.weld_by_id[w].pos
            cands = [i for i, r in enumerate(ROBOT_IDS) if w in reach.get(r, [])] or list(range(R))
            cands.sort(key=lambda i: (cnt[ROBOT_IDS[i]] >= cap, np.linalg.norm(pos - shoulder[i])))
            rid = ROBOT_IDS[cands[0]]
            own[w] = rid
            cnt[rid] += 1
        out = {}
        for i, rid in enumerate(ROBOT_IDS):
            todo = [w for w in wids if own[w] == rid]
            cur = self.home_tcp[i]
            seq = []
            while todo:
                j = int(np.argmin([np.linalg.norm(self.weld_by_id[w].pos - cur) for w in todo]))
                seq.append(todo.pop(j))
                cur = self.weld_by_id[seq[-1]].pos
            out[rid] = seq
        return out

    @property
    def home_tcp(self):
        if not hasattr(self, "_home_tcp"):
            d = self._scratch
            d.qpos[self.qadr] = self.home
            mujoco.mj_kinematics(self.model, d)
            self._home_tcp = d.site_xpos[self.tcp_site].copy()
        return self._home_tcp

    def _setup_tasks(self, wids, queues, pos, Rw):
        """wids: episode welds (task index order); queues {rid: [wid]}; pos (n,3); Rw (n,3,3)."""
        n = len(wids)
        assert n <= T_MAX
        self.task_ids = list(wids)
        self._task_pos = np.zeros((T_MAX, 3)); self._task_R = np.tile(np.eye(3), (T_MAX, 1, 1))
        self._task_app = np.zeros((T_MAX, 3)); self._task_mask = np.zeros(T_MAX)
        self._task_status = np.zeros(T_MAX); self._task_owner = np.zeros((T_MAX, R))
        self._task_pos[:n], self._task_R[:n] = pos, Rw
        self._task_app[:n] = [self.weld_by_id[w].approach for w in wids]
        self._task_mask[:n] = 1
        idx = {w: k for k, w in enumerate(wids)}
        self.queues = [[idx[w] for w in queues.get(rid, []) if w in idx] for rid in ROBOT_IDS]
        for i, qu in enumerate(self.queues):
            self._task_owner[qu, i] = 1
        # welds not in any queue cannot be completed: drop them from the episode
        unassigned = [k for k in range(n) if not self._task_owner[k].any()]
        self._task_mask[unassigned] = 0
        self.queue_pos = [0] * R
        self._dwell = np.zeros(R, np.int64)
        self.weld_events = []  # (robot, wid, t_start, t_end) in steps

    def _goal_idx(self, i):
        qu = self.queues[i]
        return qu[self.queue_pos[i]] if self.queue_pos[i] < len(qu) else -1

    def _desired(self):
        """(R, 12) desired goal vectors + mask (R,)."""
        des = np.zeros((R, 12)); mask = np.zeros(R)
        for i in range(R):
            k = self._goal_idx(i)
            if k >= 0:
                des[i, :3] = self._task_pos[k]
                des[i, 3:9] = mat_to_rot6(self._task_R[k])
                des[i, 9:] = self._task_app[k]
                mask[i] = 1
        return des, mask

    # ------------------------------------------------------------------ entry direction
    def _clear_history(self):
        H = self.cfg.history_len
        self._hist = np.zeros((R, H, 3))
        self._hist_n = np.zeros(R, np.int64)
        self._push_history(self._tcp)

    def _push_history(self, p):
        H = self.cfg.history_len
        for i in range(R):
            if self._hist_n[i] < H:
                self._hist[i, self._hist_n[i]] = p[i]
                self._hist_n[i] += 1
            elif self.cfg.entry_full_history:   # grow (all arms), never forget within a leg
                n = self._hist.shape[1]
                if self._hist_n[i] >= n:
                    self._hist = np.concatenate([self._hist, np.zeros_like(self._hist)], 1)
                self._hist[i, self._hist_n[i]] = p[i]
                self._hist_n[i] += 1
            else:
                self._hist[i, :-1] = self._hist[i, 1:]
                self._hist[i, -1] = p[i]

    def _reset_history(self, i):
        self._hist[i, 0] = self._tcp[i]
        self._hist_n[i] = 1

    def _entry(self):
        """(R,3) entry direction: umarell validate_approach walk-back over approach_span."""
        out = np.zeros((R, 3))
        for i in range(R):
            n = self._hist_n[i]
            if n < 2:
                continue
            h = self._hist[i, :n]
            p_end = h[-1]
            dist = np.linalg.norm(h[:-1] - p_end, axis=-1)
            far = np.nonzero(dist >= self.cfg.approach_span)[0]
            p_start = h[far[-1]] if len(far) else h[0]
            mv = p_end - p_start
            nm = np.linalg.norm(mv)
            if nm > 1e-6:
                out[i] = mv / nm
        return out

    # ------------------------------------------------------------------ gym API
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        cfg, rng = self.cfg, self.np_random
        options = dict(options or {})
        canonical = options.get("canonical", False)
        if canonical:
            wids = list(options.get("welds", self.wids))
            queues = options.get("assignment") or self._base_assignment(wids)
            pos = np.array([self.weld_by_id[w].pos for w in wids])
            Rw = quat_xyzw_to_mat(np.array([self.weld_by_id[w].quat_xyzw for w in wids]))
            q0 = np.asarray(options.get("q0", self.home), float).reshape(R, N_JOINTS)
        else:
            if "welds" in options:
                wids = list(options["welds"])
            else:
                lo, hi = options.get("n_welds", cfg.n_welds)
                pool = [w for w in self.wids if any(w in qu for rid, qu in self._base_assignment(self.wids).items()
                                                    if rid in cfg.active_robots)]
                n = int(rng.integers(min(lo, len(pool)), min(hi, len(pool)) + 1))
                wids = sorted(rng.choice(pool, n, replace=False).tolist())
            base = options.get("assignment") or self._base_assignment(wids)
            queues = {}
            for rid in ROBOT_IDS:
                qu = list(base.get(rid, [])) if rid in cfg.active_robots else []
                if len(qu) > 1 and rng.uniform() < cfg.shuffle_prob:
                    qu = [qu[j] for j in rng.permutation(len(qu))]
                queues[rid] = qu
            pn = options.get("pos_noise", cfg.pos_noise)
            pos = np.array([self.weld_by_id[w].pos for w in wids]) + rng.uniform(-pn, pn, (len(wids), 3))
            Rw = quat_xyzw_to_mat(np.array([self.weld_by_id[w].quat_xyzw for w in wids]))
            rn = np.deg2rad(options.get("rot_noise_deg", cfg.rot_noise_deg))
            if rn > 0:
                Rw = np.einsum("nij,njk->nik", _small_rotations(rng, len(wids), rn), Rw)
            q0 = options.get("q0")
            if q0 is None:
                q0 = self._sample_start(options.get("start_noise", cfg.start_noise))
            q0 = np.asarray(q0, float).reshape(R, N_JOINTS)
        self._setup_tasks(wids, queues, pos, Rw)
        if self.wp is not None:
            self.wp.layout(pos, -Rw[:, :, 0])
        if self.wedge is not None:
            ks = np.flatnonzero(self._task_mask > 0.5)
            self.wedge.layout(self._task_pos[ks], self._task_app[ks], -self._task_R[ks][:, :, 0],
                              [self.task_ids[k] for k in ks])
        self._welds_complete = False
        self._home_done = False
        self._t = 0
        self.set_q(q0)
        self._ep = dict(ret=0.0, collision=False)
        info = self._info(dict(collision=False, kind=None))
        return self._obs(), info

    def _sample_start(self, sigma):
        if sigma <= 0:
            return self.home.copy()
        for _ in range(50):
            q = np.clip(self.home + self.np_random.normal(0, sigma, self.home.shape), self.q_lo, self.q_hi)
            if not self.check_collisions(q)["collision"]:
                return q
        return self.home.copy()

    def step(self, action):
        cfg, m, d = self.cfg, self.model, self.data
        a = np.clip(np.asarray(action, np.float64).reshape(R, N_JOINTS), -1.0, 1.0)
        q0 = self._q
        q1 = np.clip(q0 + a * self.v_max * self.dt, self.q_lo, self.q_hi)
        ach_prev = self._achieved()

        # collision sub-samples: sphere model batched, MuJoCo hulls per sub-step
        ns = max(1, cfg.n_substeps)
        self._wp_soft_depth = 0.0
        self._wedge_depth = np.full(R, -np.inf)
        x_sub_prev = self._tcp.copy()
        el = self.wedge is not None and self.wedge.electrode   # rule W on electrode points too (off by default)
        R_sub_prev = self._tcp_R.copy() if el else None
        fr = np.arange(1, ns + 1)[:, None, None] / ns
        qs = q0[None] + (q1 - q0)[None] * fr                         # (ns, 2, 6)
        clr, floor, box = self._sphere_terms(qs)
        col, kind, q_end, s_hit = False, None, q1, ns - 1
        mj = {k: [] for k in KIND_NAMES}
        for s in range(ns):
            d.qpos[self.qadr] = qs[s]
            mujoco.mj_kinematics(m, d)
            if cfg.check_self or cfg.check_floor or self.has_boxes:
                mujoco.mj_collision(m, d)
                mj = self._mj_contacts(d)
                if not cfg.check_self:
                    mj["self"] = []
                if not cfg.check_floor:
                    mj["floor"] = []
            if self.wp is not None:
                self._wp_d, mj["workpiece"] = self.wp.distances(d)
            if self.wedge is not None:
                x_sub = d.site_xpos[self.tcp_site].copy()
                if el:
                    R_sub = d.site_xmat[self.tcp_site].reshape(R, 3, 3).copy()
                    wdep, wcon = self.wedge.check(x_sub_prev, x_sub, R_sub_prev, R_sub)
                    R_sub_prev = R_sub
                else:
                    wdep, wcon = self.wedge.check(x_sub_prev, x_sub)
                self._wedge_depth = np.maximum(self._wedge_depth, wdep)
                x_sub_prev = x_sub
                mj["workpiece"] = list(mj.get("workpiece", [])) + wcon
            if (self.wp is not None or self.wedge is not None) and cfg.workpiece_soft and mj.get("workpiece"):
                self._wp_soft_depth = max(self._wp_soft_depth, -min(x[2] for x in mj["workpiece"]))
                mj["workpiece"] = []
            kinds = self._verdict(float(clr[s]), None if floor is None else floor[s],
                                  None if box is None else box[s], mj)
            col, kind = bool(kinds), (kinds[0] if kinds else None)
            if col:
                q_end, s_hit = qs[s], s
                break
        self._qd = (q_end - q0) / self.dt
        self._q = q_end.copy()
        self._tcp, self._tcp_R, self._kp = self._fk(d)
        self._self_d = self._self_dist_from(d) if self._want_self_dist else None
        self._min_clr = float(clr[s_hit])
        self._min_clr_sub = float(np.min(clr[:s_hit + 1]))   # min over all checked sub-steps
        self._push_history(self._tcp)
        self._update_achieved()
        ach = self._ach

        des, gmask = self._desired()
        n_left = int(self._task_mask.sum() - (self._task_status * self._task_mask).sum())
        if cfg.dwell_still_tol is not None:   # still hold: moving away from the dwell start pose restarts the dwell
            moved = np.linalg.norm(self._tcp - self._dwell_anchor, axis=-1) > cfg.dwell_still_tol
            self._dwell = np.where((self._dwell > 0) & moved, 0, self._dwell)
        out = compute_reward(cfg.reward, ach, des, gmask, self._dwell, float(col), n_left=n_left,
                             achieved_prev=ach_prev, clearance=self._min_clr,
                             self_dist=None if self._self_d is None else self._self_d[:, 0])
        if cfg.dwell_still_tol is not None:
            start = out.dwell == 1
            self._dwell_anchor[start] = self._tcp[start]
        self._dwell = out.dwell
        newly = []
        for i in np.flatnonzero(out.completed):
            k = self._goal_idx(i)
            self._task_status[k] = 1
            self.weld_events.append((ROBOT_IDS[i], self.task_ids[k], self._t + 1 - cfg.reward.dwell_steps, self._t + 1))
            newly.append(self.task_ids[k])
            self.queue_pos[i] += 1
            self._reset_history(i)
        if newly:
            self._update_achieved()
        self._t += 1
        terminated = bool(out.terminated) if cfg.terminate_on_collision else bool(out.terminated and not col)
        r = float(out.reward)
        if cfg.require_home:   # Track PPO v2: the episode ends at HOME, not at the last weld
            if n_left > 0 and n_left - int(out.completed.sum()) <= 0:
                self._welds_complete = True
            home = self._welds_complete and not col and float(np.abs(self._q - self.home).max()) <= cfg.home_tol
            if home and not self._home_done:
                r += cfg.home_bonus
            self._home_done = self._home_done or home
            terminated = (col and cfg.terminate_on_collision) or self._home_done
        truncated = self._t >= cfg.episode_len and not terminated
        self._ep["ret"] += r
        self._ep["collision"] |= col
        info = self._info(dict(collision=col, kind=kind, contacts=mj), match=out.match, err=out.err, gmask=gmask)
        info["newly_completed"] = newly
        if (self.wp is not None or self.wedge is not None) and cfg.workpiece_soft:
            info["workpiece_depth"] = self._wp_soft_depth
        if self.wedge is not None:
            info["wedge_depth"] = np.clip(self._wedge_depth, 0.0, None)
        return self._obs(), r, terminated, truncated, info

    # ------------------------------------------------------------------ obs / info
    def _update_achieved(self):
        self._entry_v = self._entry()
        self._ach = np.concatenate([self._tcp, mat_to_rot6(self._tcp_R), self._entry_v], -1)

    def _achieved(self):
        """(R, 12) achieved goal vectors [tcp pos, tcp rot6, entry dir] (see env.rewards)."""
        return self._ach.copy()

    def pose_errors(self, err: dict | None = None, mask=None):
        """Per-robot errors to the current goal: dict of (R,) arrays (pos m, axis/rot/entry deg, NaN = no goal)."""
        if err is None:
            des, mask = self._desired()
            err = pose_errors(self._ach, des)
        return {k: np.where(mask > 0, err[k] if k == "pos" else np.degrees(err[k]), np.nan)
                for k in ("pos", "axis", "rot", "entry")}

    def _obs(self):
        des, gmask = self._desired()
        rot6 = mat_to_rot6(self._tcp_R)
        gerr, gmag = goal_features(self._tcp, rot6, des[:, :9], gmask)
        rank = np.ones(T_MAX)
        for i, qu in enumerate(self.queues):
            rem = qu[self.queue_pos[i]:]
            for j, k in enumerate(rem):
                rank[k] = j / T_MAX
        f = np.float32
        return dict(
            robot_q=self._q.astype(f), robot_qd=self._qd.astype(f), robot_base=self.robot_base.copy(),
            robot_tcp_pos=self._tcp.astype(f), robot_tcp_rot6=rot6.astype(f), robot_kp=self._kp.astype(f),
            robot_goal=des[:, :9].astype(f), robot_goal_mask=gmask.astype(f), robot_goal_approach=des[:, 9:].astype(f),
            robot_goal_err=gerr, robot_goal_errmag=gmag, robot_entry=self._entry_v.astype(f),
            robot_dwell=(self._dwell / cfg_dwell(self.cfg)).astype(f),
            task_pos=self._task_pos.astype(f), task_rot6=mat_to_rot6(self._task_R).astype(f) * self._task_mask[:, None].astype(f),
            task_approach=self._task_app.astype(f), task_status=(self._task_status * self._task_mask).astype(f),
            task_owner=self._task_owner.astype(f), task_rank=(rank * self._task_mask + (1 - self._task_mask)).astype(f),
            task_mask=self._task_mask.astype(f),
            obst_pos=self._obst.copy(), obst_size=self._obst_size.copy(), obst_mask=self._obst_mask.copy(),
            min_clearance=np.array([min(self._min_clr, 10.0)], f),
            time_left=np.array([max(0.0, 1.0 - self._t / self.cfg.episode_len)], f),
        ) | ({"robot_self_dist": self._self_d.astype(f)} if self.cfg.self_dist and self._self_d is not None else {}) \
          | ({"robot_wp_dist": self._wp_dist_now().astype(f)} if self.wp is not None else {}) \
          | ({"robot_wedge_margin": self.wedge.margin(self._tcp).astype(f)}
             if self.wedge is not None and self.cfg.wedge_obs else {})

    def _info(self, col: dict, match=None, err=None, gmask=None):
        """pose_err: errors w.r.t. the goal evaluated in this step (before a completed weld advances the queue)."""
        n = int(self._task_mask.sum())
        done = int((self._task_status * self._task_mask).sum())
        pe = self.pose_errors(err, gmask)
        succ = bool(n > 0 and done >= n) and (not self.cfg.require_home or getattr(self, "_home_done", False))
        return dict(welds_done=done, n_welds=n, success=succ, at_home=getattr(self, "_home_done", None),
                    min_clearance=self._min_clr, min_clearance_substeps=getattr(self, "_min_clr_sub", self._min_clr),
                    collision=bool(col["collision"]), collision_kind=col["kind"],
                    self_dist=None if getattr(self, "_self_d", None) is None else self._self_d[:, 0].copy(),
                    contacts=col.get("contacts"),
                    pose_err=np.stack([pe["pos"], pe["axis"], pe["entry"]], -1),  # (R,3): m, deg, deg
                    pose_rot_deg=pe["rot"], pose_match=None if match is None else np.asarray(match),
                    dwell=self._dwell.copy(), goal=[self.task_ids[k] if k >= 0 else None for k in map(self._goal_idx, range(R))],
                    t=self._t, time_s=self._t * self.dt)

    def _wp_dist_now(self):
        """(R, 2) workpiece distances [tool, arm] at the current state (from the last step, or computed)."""
        if self._wp_d is None:
            d = self._scratch
            d.qpos[self.qadr] = self._q
            mujoco.mj_kinematics(self.model, d)
            self._wp_d = self.wp.distances(d, want_contacts=False)[0]
        return self._wp_d

    @property
    def q(self):
        return self._q.copy()

    # ------------------------------------------------------------------ render
    def _render_setup(self):
        fx = self.cfg.render_fixture
        if fx == "auto":
            fx = "mesh" if self.cell.has_fixture else "none"
        self._rmodel = build_model(self.cell, visual=True, fixture=fx, weld_sites=True)
        self._rdata = mujoco.MjData(self._rmodel)
        h, w = self.cfg.render_size
        self._renderer = mujoco.Renderer(self._rmodel, h, w)
        self._ropt = scene_option()
        self._rqadr = np.array([[self._rmodel.jnt_qposadr[self._rmodel.joint(f"{rid}/{j}").id] for j in JOINTS]
                                for rid in ROBOT_IDS])
        self._rweld = {wid: self._rmodel.site(f"weld/{wid}").id for wid in self.wids}

    def render(self, camera: str | None = None):
        if self._renderer is None:
            self._render_setup()
        rm, rd = self._rmodel, self._rdata
        rd.qpos[self._rqadr] = self._q
        mujoco.mj_kinematics(rm, rd)
        mujoco.mj_comPos(rm, rd)
        mujoco.mj_camlight(rm, rd)  # camera/light poses (not part of mj_kinematics)
        for wid, sid in self._rweld.items():  # weld markers: red open, green done, hidden if not in episode
            if wid in self.task_ids:
                k = self.task_ids.index(wid)
                rd.site_xpos[sid] = self._task_pos[k]
                rm.site_rgba[sid] = [0.1, 0.85, 0.2, 0.9] if self._task_status[k] else [0.95, 0.1, 0.1, 0.9]
                if not self._task_mask[k]:
                    rm.site_rgba[sid, 3] = 0
            else:
                rm.site_rgba[sid, 3] = 0.0
        self._renderer.update_scene(rd, camera=camera or self.cfg.render_camera, scene_option=self._ropt)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def cfg_dwell(cfg: EnvConfig) -> float:
    return float(max(1, cfg.reward.dwell_steps))


def _small_rotations(rng, n, max_angle):
    ax = rng.normal(size=(n, 3))
    ax /= np.linalg.norm(ax, axis=-1, keepdims=True)
    ang = rng.uniform(0, max_angle, n)
    K = np.zeros((n, 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -ax[:, 2], ax[:, 1], -ax[:, 0]
    K = K - K.transpose(0, 2, 1)
    s, c = np.sin(ang)[:, None, None], np.cos(ang)[:, None, None]
    return np.eye(3) + s * K + (1 - c) * K @ K
