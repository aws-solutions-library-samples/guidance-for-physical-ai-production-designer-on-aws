"""Exact speed-ups of umarell 76ea13a's OWN collision predicate (kernels ``x76``; nothing re-implemented).

The eccb1bc drop-ins (kernels_np, u1/kernels, u2/kernels_fix) re-create eccb1bc's ``_config_is_free`` and would drop
76ea13a's arm-vs-itself check, link envelope, tool hulls and workpiece rule (they refuse to install there). This
module keeps umarell's predicate and removes only the two costs the profile of a 76ea13a plan shows
(docs: PROGRESS.md 2026-10-23 U3 port; data/umarell_fast/u3port/profile_*):

1. ``robot_kinematics._config_is_free``: its first check is a pure-Python double loop of ``np.linalg.norm`` over
   (arm + tool spheres) x moving-obstacle spheres. With another arm's swept volume as the obstacles (coupled steps,
   the assembler's home step) that is ~30 000 norms per call: 60 ms of a 61 ms check. :func:`config_is_free` runs
   the same test vectorised and then calls umarell's ORIGINAL with ``obstacle_spheres=[]``. Since the loop only
   ever returns False, ``orig(q, obs) == (no moving hit) and orig(q, [])`` for any body after it (the v6 patches
   change that body: 22v2 adds the tool hulls). The vectorised distance can differ from ``float(np.linalg.norm())``
   in the last ulp, so every pair within ``BAND`` (1e-9 m) of the threshold is decided by the original scalar
   expression: the boolean is identical, not approximately so. :func:`install` checks the original's source still
   has exactly that loop (and uses ``obstacle_spheres`` nowhere else) and refuses otherwise.
2. ``utils.cell_layout.read_manifest``: re-parsed the cell manifest (cell.yaml) with PyYAML on every call -- every
   self-collision check resolves the robot's URDF through it (``leg_verdict.cell_stamp``, ``self_collision.checker``):
   ~1.6 ms per collision check. Parsed once per (path, mtime, size), a deep copy per call (as U3_01 does for
   robots.yaml).

Equality on real planner inputs: ``python -m umarell_fast.kernels_76ea13a --repo <76ea13a copy> --cell <cell>``
(recorded calls of a real plan + perturbations; booleans exact; also which check decided each state). Install into
a scratch working copy only (never third_party/).
"""
from __future__ import annotations

import copy
import inspect
import operator
import sys
import threading

import numpy as np

BAND = 1e-9            # |distance - threshold| below this: decided by umarell's own scalar expression
CHUNK = 4096           # obstacle spheres per broadcast block (bounded memory, early exit)
_ORIG: dict = {}
_LOCK = threading.Lock()
_OBS: dict = {}        # id(list) -> (list, items tuple, O, R)
_MANIFEST: dict = {}
STATS = {"moving_calls": 0, "moving_hits": 0, "band_checks": 0, "manifest_hits": 0, "manifest_misses": 0,
         "policy_hits": 0, "policy_misses": 0, "pair_blocks": 0, "pair_blocks_skipped": 0,
         "pair_verified": 0, "pair_mismatches": 0}

_LOOP = """    for obs_pos, obs_r in obstacle_spheres:
        for s_pos in spheres:
            if float(np.linalg.norm(s_pos - obs_pos)) < obs_r + sphere_radius:
                return False
        for s_pos in tool:
            if float(np.linalg.norm(s_pos - obs_pos)) < obs_r + sphere_radius:
                return False
"""
_PARAMS = ("model", "col_data", "q", "base", "yaw", "obstacle_spheres", "static_obstacles", "sphere_radius",
           "tool_cfg")


def _obs_arrays(obstacle_spheres: list):
    """(O (M, 3), R (M,)) of a moving-obstacle list, cached while the list holds the same items."""
    key = id(obstacle_spheres)
    with _LOCK:
        hit = _OBS.get(key)
    if hit is not None and hit[0] is obstacle_spheres and len(hit[1]) == len(obstacle_spheres) \
            and all(map(operator.is_, hit[1], obstacle_spheres)):
        return hit[2], hit[3]
    items = tuple(obstacle_spheres)
    O = np.asarray([np.asarray(p, float).reshape(3) for p, _r in items], float).reshape(-1, 3)
    R = np.asarray([float(r) for _p, r in items], float)
    with _LOCK:
        _OBS[key] = (obstacle_spheres, items, O, R)
        while len(_OBS) > 64:
            _OBS.pop(next(iter(_OBS)))
    return O, R


def moving_hit(points: list, obstacle_spheres: list, sphere_radius: float) -> bool:
    """True iff ``float(np.linalg.norm(s - o)) < r + sphere_radius`` for some point s and obstacle (o, r) -- the
    original loop's condition, exactly."""
    if not obstacle_spheres or not len(points):
        return False
    S = np.asarray(points, float).reshape(-1, 3)
    O, R = _obs_arrays(obstacle_spheres)
    for a in range(0, len(O), CHUNK):
        Oc = O[a:a + CHUNK]
        thr = R[a:a + CHUNK] + sphere_radius
        diff = S[:, None, :] - Oc[None, :, :]
        m = np.sqrt((diff * diff).sum(-1)) - thr[None, :]
        if (m < -BAND).any():
            return True
        for i, j in zip(*np.nonzero(np.abs(m) <= BAND)):
            STATS["band_checks"] += 1
            obs_pos, obs_r = obstacle_spheres[a + int(j)]
            if float(np.linalg.norm(S[int(i)] - obs_pos)) < obs_r + sphere_radius:
                return True
    return False


def config_is_free(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles, *args, **kw):
    """umarell's ``_config_is_free`` with its moving-obstacle loop vectorised (module doc)."""
    orig = _ORIG["_config_is_free"]
    if obstacle_spheres:
        import pinocchio as pin
        from agents.trajectory.tools import robot_kinematics as rk
        sphere_radius = args[0] if len(args) > 0 else kw.get("sphere_radius", 0.12)
        tool_cfg = args[1] if len(args) > 1 else kw.get("tool_cfg")
        pin.forwardKinematics(model, col_data, q)
        pin.updateFramePlacements(model, col_data)
        spheres = rk._sphere_positions(model, col_data, base, yaw)
        tool = rk._tool_spheres(model, col_data, base, yaw, tool_cfg)
        STATS["moving_calls"] += 1
        if moving_hit(list(spheres) + list(tool), obstacle_spheres, float(sphere_radius)):
            STATS["moving_hits"] += 1
            return False
    return orig(model, col_data, q, base, yaw, [], static_obstacles, *args, **kw)


def _check_original(fn) -> None:
    """Refuse to install unless the original is the predicate this module was proven against."""
    sig = list(inspect.signature(fn).parameters)
    if tuple(sig[:len(_PARAMS)]) != _PARAMS:
        raise RuntimeError(f"kernels_76ea13a: unexpected _config_is_free signature {sig}")
    src = inspect.getsource(fn)
    body = src.split('"""', 2)[-1] if src.count('"""') >= 2 else src
    if _LOOP not in body:
        raise RuntimeError("kernels_76ea13a: _config_is_free's moving-obstacle loop is not the one this kernel mirrors")
    code = [ln.split("#", 1)[0] for ln in body.splitlines()]
    uses = sum(ln.count("obstacle_spheres") for ln in code)
    if uses != 1:
        raise RuntimeError(f"kernels_76ea13a: _config_is_free uses obstacle_spheres {uses} times (expected only the "
                           "loop)")
    pre = body.split(_LOOP)[0]
    for need in ("pin.forwardKinematics(model, col_data, q)", "spheres = _sphere_positions(model, col_data, base, yaw)",
                 "tool = _tool_spheres(model, col_data, base, yaw, tool_cfg)"):
        if need not in pre:
            raise RuntimeError(f"kernels_76ea13a: _config_is_free no longer computes {need!r} before the loop")
    if "return" in "\n".join(ln for ln in pre.splitlines() if not ln.strip().startswith("#")):
        raise RuntimeError("kernels_76ea13a: _config_is_free returns before its moving-obstacle loop")


def make_read_manifest(orig):
    def read_manifest(cell_id, cell_base=None):
        from utils import cell_layout
        try:
            p = cell_layout.cell_yaml(cell_id, cell_base)
            st = p.stat()
            key = (str(p), st.st_mtime_ns, st.st_size)
        except OSError:
            return orig(cell_id, cell_base)
        with _LOCK:
            hit = _MANIFEST.get(key)
        if hit is None:
            STATS["manifest_misses"] += 1
            hit = copy.deepcopy(orig(cell_id, cell_base))
            with _LOCK:
                _MANIFEST[key] = hit
        else:
            STATS["manifest_hits"] += 1
        return copy.deepcopy(hit)
    read_manifest._uf_x76 = True
    return read_manifest


_POLICY: dict = {}


def make_clearance_policy(orig):
    """``robot_kinematics._load_clearance_policy`` once per obstacles.yaml (path, mtime, size). The original reads the
    whole file through ``cell_store.read_yaml`` -- which deep-copies every obstacle (body_weld_3: thousands of voxel
    boxes) -- for seven scalars, on every IK solve, sphere-radius lookup, speed margin and leg check: ~23 % of a
    body_weld_3 U3 plan's worker time (py-spy, 2026-11-04). The policy is a function of that file alone
    (:func:`_check_policy` checks the original still reads nothing else); each call gets its own copy of the dict."""
    def _load_clearance_policy(cell_id):
        from utils import cell_layout
        try:
            p = cell_layout.obstacles_yaml(cell_id)
            st = p.stat()
            key = (str(cell_id), str(p), st.st_mtime_ns, st.st_size)
        except OSError:
            return orig(cell_id)
        with _LOCK:
            hit = _POLICY.get(key)
        if hit is None:
            STATS["policy_misses"] += 1
            hit = dict(orig(cell_id))
            with _LOCK:
                _POLICY[key] = hit
        else:
            STATS["policy_hits"] += 1
        return dict(hit)
    _load_clearance_policy._uf_x76 = True
    return _load_clearance_policy


def _check_policy(fn) -> None:
    """Refuse to cache the clearance policy unless the original reads obstacles.yaml and nothing else."""
    src = inspect.getsource(fn)
    reads = [ln for ln in src.splitlines() if "_read_" in ln or "_load_" in ln.replace("def _load_clearance_policy", "")
             or "read_yaml" in ln or "open(" in ln or "environ" in ln]
    if reads != ["    cfg = _read_obstacles_yaml(cell_id)"]:
        raise RuntimeError(f"kernels_76ea13a: _load_clearance_policy reads more than obstacles.yaml: {reads}")


# ─── arm-vs-arm sphere distances (the inter-arm gate) ─────────────────────────
# robot_kinematics._check_robot_pair_collision as proven against (sha256 of its source, umarell dc60669 + fix set v9)
PAIR_SRC_SHA = "786b7173e9cf6af23817402e4c28a43f1c5a6c9bbe650902beba21307553cae6"
PAIR_TOL = 1e-5        # m: candidates within this of a block's fast minimum are measured by the original expression


def check_robot_pair_collision(s1, s2, radius: float = 0.12, stop1=None, stop2=None, groups1=None, groups2=None,
                               radii1=None, radii2=None):
    """The fast pair check; with ``UF_PAIR_VERIFY=1`` every call also runs the original (a mismatch is counted in
    ``STATS["pair_mismatches"]``, logged once, and the original's answer returned)."""
    res = _pair_fast(s1, s2, radius, stop1, stop2, groups1, groups2, radii1, radii2)
    if _PAIR_VERIFY:
        ref = _ORIG["_check_robot_pair_collision"](s1, s2, radius, stop1, stop2, groups1, groups2, radii1, radii2)
        STATS["pair_verified"] += 1
        if (bool(ref[0]), float(ref[1])) != (bool(res[0]), float(res[1])):
            STATS["pair_mismatches"] += 1
            if STATS["pair_mismatches"] == 1:
                print(f"[uf] x76 pair kernel MISMATCH: fast {res} original {ref}", flush=True)
            return ref
    return res


_PAIR_VERIFY = __import__("os").environ.get("UF_PAIR_VERIFY", "0") == "1"


def _pair_fast(s1, s2, radius: float = 0.12, stop1=None, stop2=None, groups1=None, groups2=None,
               radii1=None, radii2=None):
    """umarell's ``_check_robot_pair_collision``, the same branch and bound (bounds, order, break), with each group
    pair's distance block measured fast: ``|a|^2 + |b|^2 - 2 a.b`` (one matrix product, no (n, m, 3) temporary).
    Only the block entries within :data:`PAIR_TOL` of the block's fast minimum -- which always include the exact
    argmin, the fast distance's error being < 1e-6 m even at contact for coordinates within 100 m -- are measured by
    umarell's original expression, so every block minimum, the running minimum and the returned ``(collision,
    min_clearance)`` are the original's bit for bit. A block whose fast minimum cannot lower the running minimum
    (even with the margin) is skipped, as the original's ``min`` would leave it unchanged. ~2.5x faster on 700-sphere
    arms (29 % of a body_weld_3 U3 plan's worker time in the original, py-spy 2026-11-04)."""
    A = np.asarray(s1, float).reshape(-1, 3)
    B = np.asarray(s2, float).reshape(-1, 3)
    if not len(A) or not len(B):
        return False, float("inf")
    ra_ = np.full(len(A), float(radius)) if radii1 is None else np.asarray(radii1, float)
    rb_ = np.full(len(B), float(radius)) if radii2 is None else np.asarray(radii2, float)
    sa = (np.zeros(len(A)) if stop1 is None else np.asarray(stop1, float)) + ra_ - radius
    sb = (np.zeros(len(B)) if stop2 is None else np.asarray(stop2, float)) + rb_ - radius
    ga = groups1 or [slice(0, len(A))]
    gb = groups2 or [slice(0, len(B))]

    def bound(P, S, groups):
        c = np.asarray([P[g].mean(0) for g in groups])
        r = np.asarray([np.sqrt(((P[g] - c[k]) ** 2).sum(-1)).max() for k, g in enumerate(groups)])
        return c, r, np.asarray([S[g].max() for g in groups])

    ca, ra, ma = bound(A, sa, ga)
    cb, rb, mb = bound(B, sb, gb)
    lb = (np.sqrt(((ca[:, None, :] - cb[None, :, :]) ** 2).sum(-1))
          - ra[:, None] - rb[None, :] - 2 * radius - ma[:, None] - mb[None, :])
    A2 = (A * A).sum(1)
    B2 = (B * B).sum(1)
    scale = float(max(A2.max(), B2.max()))
    if not np.isfinite(scale) or scale > 1e4:          # beyond the proven error bound: the original
        return _ORIG["_check_robot_pair_collision"](s1, s2, radius, stop1, stop2, groups1, groups2, radii1, radii2)
    min_d = float("inf")
    for flat in np.argsort(lb, axis=None):
        u, v = divmod(int(flat), len(gb))
        if lb[u, v] >= min_d:
            break
        gu, gv = ga[u], gb[v]
        a, b = A[gu], B[gv]
        sq = A2[gu][:, None] + B2[gv][None, :] - 2.0 * (a @ b.T)
        Df = np.sqrt(np.maximum(sq, 0.0)) - sa[gu][:, None] - sb[gv][None, :]
        m = float(Df.min())
        if m - 2 * radius - PAIR_TOL >= min_d:
            STATS["pair_blocks_skipped"] += 1
            continue
        I, J = np.nonzero(Df <= m + PAIR_TOL)
        D = np.sqrt(((a[I] - b[J]) ** 2).sum(-1)) - 2 * radius - sa[gu][I] - sb[gv][J]
        min_d = min(min_d, float(D.min()))
        STATS["pair_blocks"] += 1
    return min_d < 0, min_d


def _check_pair_original(fn) -> None:
    import hashlib
    got = hashlib.sha256(inspect.getsource(fn).strip().encode()).hexdigest()
    if got != PAIR_SRC_SHA:
        raise RuntimeError(f"kernels_76ea13a: _check_robot_pair_collision is not the proven original ({got[:12]})")


def _rebind(old, new) -> None:
    """Replace ``old`` by ``new`` in every imported agents.* / utils.* module that bound it by name."""
    for mname, mod in list(sys.modules.items()):
        if mod is None or not (mname.startswith("agents.") or mname.startswith("utils.")):
            continue
        for attr, val in list(vars(mod).items()):
            if val is old:
                setattr(mod, attr, new)


def install() -> dict:
    """Install both into this process's umarell (76ea13a only). Returns the originals (``kernel_check.selfcheck``
    / :func:`uninstall`). Idempotent."""
    from agents.trajectory.tools import robot_kinematics as rk
    from utils import cell_layout
    if getattr(rk, "_uf_x76", False):
        return {"_config_is_free": _ORIG["_config_is_free"]}
    if not (hasattr(rk, "_self_free") and hasattr(rk, "gate_groups")):
        raise RuntimeError("kernels_76ea13a: this umarell is not 76ea13a (no _self_free / gate_groups)")
    _check_original(rk._config_is_free)
    _ORIG["_config_is_free"] = rk._config_is_free
    _ORIG["read_manifest"] = cell_layout.read_manifest
    _rebind(_ORIG["_config_is_free"], config_is_free)
    _rebind(_ORIG["read_manifest"], make_read_manifest(_ORIG["read_manifest"]))
    _check_policy(rk._load_clearance_policy)
    _ORIG["_load_clearance_policy"] = rk._load_clearance_policy
    _rebind(_ORIG["_load_clearance_policy"], make_clearance_policy(_ORIG["_load_clearance_policy"]))
    try:                                   # the pair kernel only over the exact original it was proven against
        _check_pair_original(rk._check_robot_pair_collision)
        _ORIG["_check_robot_pair_collision"] = rk._check_robot_pair_collision
        _rebind(_ORIG["_check_robot_pair_collision"], check_robot_pair_collision)
        print(f"[uf] x76: fast pair kernel installed (verify {'on' if _PAIR_VERIFY else 'off'})", flush=True)
    except (RuntimeError, OSError, TypeError) as exc:
        print(f"[uf] x76: fast pair kernel NOT installed ({exc})", flush=True)
    rk._uf_x76 = True
    return {"_config_is_free": _ORIG["_config_is_free"]}


def uninstall() -> None:
    from agents.trajectory.tools import robot_kinematics as rk
    from utils import cell_layout
    if not getattr(rk, "_uf_x76", False):
        return
    _rebind(rk._config_is_free, _ORIG["_config_is_free"])
    _rebind(cell_layout.read_manifest, _ORIG["read_manifest"])
    _rebind(rk._load_clearance_policy, _ORIG["_load_clearance_policy"])
    if "_check_robot_pair_collision" in _ORIG:
        _rebind(rk._check_robot_pair_collision, _ORIG["_check_robot_pair_collision"])
    rk._uf_x76 = False


# ─── equality on real planner inputs ─────────────────────────────────────────

def record_calls(limit: int = 4000):
    """Wrap umarell's ``_config_is_free`` to record (a sample of) its real calls; returns (list, restore)."""
    from agents.trajectory.tools import robot_kinematics as rk
    orig = rk._config_is_free
    calls: list = []
    seen = [0]

    def rec(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles, *args, **kw):
        seen[0] += 1
        if len(calls) < limit and (obstacle_spheres or seen[0] % 7 == 0):
            calls.append((model, np.array(q, float), np.array(base, float), float(yaw), list(obstacle_spheres),
                          static_obstacles, args, dict(kw)))
        return orig(model, col_data, q, base, yaw, obstacle_spheres, static_obstacles, *args, **kw)
    _rebind(orig, rec)
    return calls, (lambda: _rebind(rec, orig)), seen


def compare(calls: list, perturb: int = 3, sigma: float = 0.05, seed: int = 0) -> dict:
    """``_config_is_free`` original vs :func:`config_is_free` on recorded calls and Gaussian perturbations of
    their configurations (states near the decision boundary). Also classifies each state by the check that
    decides it (umarell's ``_config_collision_info``: floor / robot / box / sphere / tool_penetration / self /
    free) so the evidence shows every sub-check was exercised."""
    import time
    from agents.trajectory.tools import robot_kinematics as rk
    orig = _ORIG.get("_config_is_free") or rk._config_is_free
    rng = np.random.default_rng(seed)
    n = mism = 0
    kinds: dict = {}
    t_orig = t_new = 0.0
    ex: list = []
    for model, q, base, yaw, obs, static, args, kw in calls:
        qs = [q] + [q + rng.normal(0.0, sigma, q.shape) for _ in range(perturb)]
        for qq in qs:
            d1, d2 = model.createData(), model.createData()
            t = time.perf_counter()
            a = bool(orig(model, d1, qq, base, yaw, obs, static, *args, **kw))
            t_orig += time.perf_counter() - t
            t = time.perf_counter()
            b = bool(config_is_free(model, d2, qq, base, yaw, obs, static, *args, **kw))
            t_new += time.perf_counter() - t
            n += 1
            if a != b:
                mism += 1
                if len(ex) < 5:
                    ex.append({"q": qq.round(5).tolist(), "orig": a, "new": b, "n_obs": len(obs)})
            if not a:
                sr = args[0] if args else kw.get("sphere_radius", 0.12)
                ikw = {k: kw[k] for k in ("sphere_radii", "extra_margin") if k in kw}
                if len(args) > 2:
                    ikw.setdefault("sphere_radii", args[2])
                if len(args) > 3:
                    ikw.setdefault("extra_margin", args[3])
                try:
                    info = rk._config_collision_info(model, model.createData(), qq, base, yaw, obs, static,
                                                     sphere_radius=sr, **ikw)
                    k = (info or {}).get("type") or "tool_hull_or_other"
                except Exception as exc:  # noqa: BLE001
                    k = f"diag_error:{type(exc).__name__}"
            else:
                k = "free"
            k = f"{k}{'+moving' if obs else ''}"
            kinds[k] = kinds.get(k, 0) + 1
    return {"states": n, "mismatches": mism, "examples": ex, "decided_by": dict(sorted(kinds.items())),
            "mean_ms_orig": round(1e3 * t_orig / max(n, 1), 3), "mean_ms_x76": round(1e3 * t_new / max(n, 1), 3),
            "band_checks": STATS["band_checks"]}


def _main(argv=None) -> int:
    """Record real ``_config_is_free`` calls while umarell plans a few legs of each arm, a coupled step and a home
    step on one cell (76ea13a copy), then compare. Prints JSON."""
    import argparse
    import json
    from pathlib import Path
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    ap.add_argument("--cell", default="welding_cell_2_small")
    ap.add_argument("--legs", type=int, default=3)
    ap.add_argument("--limit", type=int, default=1500)
    ap.add_argument("--perturb", type=int, default=3)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    out_path = Path(a.out).resolve() if a.out else None
    from umarell_fast.u2 import session as S
    from umarell_fast import workers as W
    from umarell_fast.u3 import RulePolicy
    repo = Path(a.repo).resolve()
    run = root / "data" / "umarell_fast" / "u3port" / f"x76eq_{a.cell}"
    base = S.make_cell_base(repo, run, a.cell)
    env = S.env_for(repo, a.cell, base, run / "scratch",
                    {"OMPL_PARALLEL_CANDIDATES": "0", "PLAN_STEP_GATE": "0", "OMPL_SEED": "11"})
    W.init(str(repo), a.cell, env, "none", seed=0)
    calls, restore, seen = record_calls(a.limit)
    task = S.load_task(base, a.cell)
    ops = {o["id"]: o for o in task["operations"]}
    mrta = S.call("run_mrta_pipeline", cell_id=a.cell, operations=task["operations"])
    pol = RulePolicy()
    from agents.home_pose import home_pose
    seqs: dict = {}
    for st in mrta["steps"]:
        for ns, op in st.items():
            if isinstance(op, dict) and "id" in op:
                seqs.setdefault(ns, []).append(op["id"])
    ends, planned, trajs = {}, [], {}
    for ns, ids in sorted(seqs.items()):
        start, prev = home_pose(a.cell, ns), None
        for oid in ids[:a.legs + 2]:
            if len(trajs.get(ns, [])) >= a.legs:
                break
            r = W.plan_chain(a.cell, [{"ns": ns, "op_id": oid, "req": pol.weld_request(ns, ops[oid]),
                                       "start": start, "prev_pose": prev}], pol.call_kwargs())["legs"][0]
            planned.append((ns, oid, r["ok"]))
            if not r["ok"]:
                continue                     # skip it (the next weld from the same start)
            start, prev = r["end"], r.get("last_pose")
            ends[ns] = (start, prev, r)
            trajs.setdefault(ns, []).append(r["traj"])
    # a coupled step (each arm's next weld in one call: the later arm plans around the earlier one's swept volume)
    nxt = [{"ns": ns, "op_id": ids[a.legs], "req": pol.weld_request(ns, ops[ids[a.legs]]), "start": ends[ns][0],
            "prev_pose": ends[ns][1]} for ns, ids in sorted(seqs.items()) if ns in ends and len(ids) > a.legs]
    if len(nxt) > 1:
        r = W.plan_coupled(a.cell, nxt, pol.call_kwargs())
        planned += [(l["ns"], l["op_id"], l["ok"]) for l in r["legs"]]
    # the home step (every arm's return home as one coordinated set)
    from agents.trajectory.tools.planner_backend import plan_home_trajectories
    W.reset_session(a.cell)
    for ns, (q, pose, r) in ends.items():
        W.inject(a.cell, ns, q, pose, {r["op_id"]: r.get("weld_pose")} if r.get("weld_pose") else None)
    home = plan_home_trajectories(a.cell, {ns: v[0] for ns, v in ends.items()}) or {}
    restore()
    # moving obstacles as the planner builds them for a coupled step: the other arm's swept volume
    # (backends/ompl/planner.py:_swept_spheres -- SWEPT_SAMPLES configurations, deduplicated at 1 cm, SWEPT_R),
    # against this arm's own planned configurations (they work at the same part: near the boundary)
    from agents.trajectory.tools import robot_kinematics as rk
    from agents.trajectory.backends.ompl import planner as op
    sw_n, sw_r = int(getattr(op, "SWEPT_SAMPLES", 12)), float(getattr(op, "SWEPT_R", 0.14))
    tmpl = {}
    for c in calls:
        tmpl.setdefault(id(c[0]), c)
    models = {}
    for ns in trajs:
        m, _ = rk._load_pinocchio_model(a.cell, ns)
        cfg = rk._load_robot_config(a.cell, ns)
        models[ns] = (m, np.array(cfg["base_position"], float), float(cfg.get("base_orientation_yaw", 0.0)),
                      cfg.get("tool_collision"))
    synth = 0
    for ns_a, ta in trajs.items():
        m, b, y, tool = models[ns_a]
        t = tmpl.get(id(m))
        if t is None:
            continue
        for ns_b, tb in trajs.items():
            if ns_b == ns_a:
                continue
            mb, bb, yb, toolb = models[ns_b]
            for trb in tb + list(filter(None, [home.get(ns_b)])):
                n = len(trb)
                idx = sorted({int(round(i * (n - 1) / (sw_n - 1))) for i in range(sw_n)}) if n > 1 else [0]
                obs, seen_k = [], set()
                for i in idx:
                    for p in rk._config_spheres(mb, mb.createData(), np.array(trb[i]["positions"][:mb.nq]), bb, yb,
                                                toolb):
                        k = tuple(round(float(v), 2) for v in p)
                        if k not in seen_k:
                            seen_k.add(k)
                            obs.append((p, sw_r))
                for tra in ta:
                    for i in range(0, len(tra), max(1, len(tra) // 8)):
                        calls.append((m, np.array(tra[i]["positions"][:m.nq], float), b, y, obs, t[5], t[6], t[7]))
                        synth += 1
    # random configurations in each arm's joint box (floor / fixtures / self decide most of them), with and
    # without the other arm's swept volume
    from agents.trajectory.tools.joint_limits import position_limits
    rng = np.random.default_rng(5)
    rnd = 0
    for ns_a in trajs:
        m, b, y, tool = models[ns_a]
        t = tmpl.get(id(m))
        if t is None:
            continue
        try:
            lo, hi = (np.asarray(v, float)[:m.nq] for v in position_limits(rk._load_robot_config(a.cell, ns_a), m))
        except Exception:  # noqa: BLE001
            lo, hi = m.lowerPositionLimit[:m.nq], m.upperPositionLimit[:m.nq]
        lo, hi = np.maximum(lo, -np.pi), np.minimum(hi, np.pi)
        moving = next((c[4] for c in calls if c[0] is not m and c[4]), [])
        for k in range(300):
            q = lo + (hi - lo) * rng.random(m.nq)
            calls.append((m, q, b, y, moving if k % 2 else [], t[5], t[6], t[7]))
            rnd += 1
    install()
    res = compare(calls, perturb=a.perturb)
    res["synthetic_moving_calls"] = synth
    res["random_calls"] = rnd
    res.update(cell=a.cell, repo=str(repo), recorded=len(calls), predicate_calls_seen=seen[0], planned=planned,
               home_legs=sorted(home), with_moving=sum(1 for c in calls if c[4]))
    print(json.dumps(res, indent=1))
    if out_path:
        out_path.write_text(json.dumps(res, indent=1))
    return 0 if res["mismatches"] == 0 and res["states"] else 1


if __name__ == "__main__":
    sys.exit(_main())
