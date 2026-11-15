"""Planning session over a umarell working copy (no LLM in the loop).

Sets up exactly what ``agents/trajectory/agent.py:_plan`` sets up before its ReAct loop -- backend binding ``ompl``,
classic planning mode, a cleared trajectory cache and chaining configs, the skill registry bound to the cell, a
registered cancellation id, the bound model -- so umarell's own tools can be driven through their ``.func`` with
their unchanged semantics (same as scripts/uf_replay.py). Also used as the initializer of every worker process.

The working copy is created by ``umarell_fast/apply.py`` (data/umarell_fast/repo_U2[_fix]/, never third_party/).
Each run gets its own CELL_BASE_PATH (the target cell copied, all other entries symlinked), so concurrent runs cannot
share umarell's per-cell knowledge base, plans or logs.
"""
from __future__ import annotations

import contextlib
import json
import os
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

_STATE: dict = {}


def make_cell_base(repo: Path, run_dir: Path, cell: str) -> Path:
    """Per-run CELL_BASE_PATH under ``run_dir``: the target cell copied (without old plans/logs/recordings/
    checkpoints), every other entry a symlink into the working copy."""
    base = Path(run_dir) / "cells"
    if base.exists():
        shutil.rmtree(base)
    base.mkdir(parents=True)
    for p in (Path(repo) / "cells").iterdir():
        if p.name == cell:
            shutil.copytree(p, base / p.name, symlinks=True,
                            ignore=shutil.ignore_patterns("plans", "logs", "recordings", "checkpoints"))
        else:
            (base / p.name).symlink_to(p.resolve())
    return base


def env_for(repo: Path, cell: str, cell_base: Path, scratch: Path, extra: dict | None = None) -> dict:
    env = {
        "CELL_BASE_PATH": str(cell_base), "CELL_ID": cell, "TRAJECTORY_BACKEND": "ompl",
        "PLANNER_SCRATCH_PATH": str(scratch),
        # umarell's HOME staging proposer would call the LLM from inside OMPL; U2 keeps every LLM call in the
        # decision router (logged + budgeted), so the nested proposer is off (umarell's deterministic fallback runs).
        "HOME_STRATEGY_LLM": "0",
        "LLM_PROVIDER": "bedrock", "LLM_MODEL": "us.anthropic.claude-opus-5",
        "BEDROCK_MODEL_ID": "us.anthropic.claude-opus-5", "BEDROCK_REGION": "us-east-1", "AWS_REGION": "us-east-1",
        "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
    }
    env.update(extra or {})
    return env


def setup(repo: str | Path, cell: str, env: dict, kernels: str = "npplus", seed: int | None = None) -> dict:
    """Import umarell from ``repo`` into this process and open a planning session for ``cell``.

    ``kernels``: ``none`` (umarell's own Python collision checks), ``np`` / ``npplus`` (umarell_fast.kernels_np,
    docs/uf_profile.md §5; ``npplus`` adds the collision-diagnosis fast path and the parse-once YAML cache), or
    ``fix`` (npplus semantics adapted to the upstream fixes 02/03: tool spheres vs floor and static obstacles; see
    umarell_fast.u2.kernels_fix). Idempotent per process.
    """
    if _STATE.get("ready"):
        return _STATE
    repo = Path(repo).resolve()
    os.environ.update(env)
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))
    if str(ROOT) not in sys.path:
        sys.path.insert(1, str(ROOT))
    os.chdir(repo)

    import pinocchio as pin
    if not hasattr(pin.Frame, "parentJoint"):          # pin 2.7 shim (docs/scouting/planner.md §5, upstream 11)
        pin.Frame.parentJoint = property(lambda s: s.parent)

    try:
        import agents.trajectory.agent as tagent
    except ModuleNotFoundError as exc:
        # agent.py imports langgraph (the ReAct loop) at module level; nothing here runs that loop, so a venv without
        # langgraph (e.g. .venv-umarell-mac) gets the same @tool objects from the modules that define them
        if not str(exc.name or "").startswith("langgraph"):
            raise
        tagent = None
    from agents.llm_model import bind_model
    from utils import skill_registry
    from agents.trajectory.tools import plan_cancellation
    from agents.trajectory.tools import coordination as _coord
    from agents.trajectory.tools.planner_backend import set_planning_backend
    from agents.trajectory.tools.trajectory_cache import clear_trajectory_cache, reset_robot_configs

    # Guard: nothing inside umarell may reach an LLM on its own (every LLM call goes through the router).
    def _no_llm(*_a, **_k):
        raise RuntimeError("umarell_fast.u2: umarell attempted its own LLM call (HOME_STRATEGY_LLM must be 0)")
    if tagent is not None:
        tagent.build_llm = _no_llm
    import agents.llm_model as _lm
    _lm.build_llm = _no_llm

    if kernels in ("np", "npplus"):
        from umarell_fast import kernels_np
        kernels_np.install(plus=(kernels == "npplus"))
    elif kernels == "fix":
        from umarell_fast.u2 import kernels_fix
        kernels_fix.install()
    elif kernels != "none":
        raise ValueError(f"unknown kernels {kernels!r}")

    if seed is not None:
        seed_rngs(seed, ompl_too=True)

    cancel_id = f"uf_u2_{cell}_{os.getpid()}"
    plan_cancellation.begin(cancel_id)
    stack = contextlib.ExitStack()
    stack.enter_context(bind_model(None))
    stack.enter_context(skill_registry.bind_cell(cell))
    stack.enter_context(_coord.bind_planning_mode(None))
    _coord._MODE.set("classic")
    set_planning_backend("ompl")
    clear_trajectory_cache(cell)
    reset_robot_configs(cell)
    tools = {t.name: t for t in tagent._TOOLS} if tagent is not None else _module_tools()
    _STATE.update(ready=True, repo=str(repo), cell=cell, tools=tools, stack=stack, kernels=kernels,
                  base=umarell_base(repo),
                  cancel_id=cancel_id, pid=os.getpid())
    return _STATE


_TOOL_MODULES = ("agents.trajectory.tools.planning_tools", "agents.trajectory.tools.pinocchio",
                 "agents.trajectory.tools.behaviour_tree", "agents.trajectory.tools.knowledge_base",
                 "agents.trajectory.tools.coordinated_tree", "agents.trajectory.tools.program_eval")


def _module_tools() -> dict:
    """umarell's planning tools (the ``@tool`` objects agent.py lists in ``_TOOLS``) from their own modules -- for a
    venv without langgraph. ``load_skill`` (defined in agent.py, for the LLM) is the only one missing."""
    import importlib
    out = {}
    for mname in _TOOL_MODULES:
        try:
            mod = importlib.import_module(mname)
        except ImportError:
            continue                        # e.g. program_eval / coordinated_tree on an old base
        for v in vars(mod).values():
            if callable(getattr(v, "func", None)) and isinstance(getattr(v, "name", None), str):
                out.setdefault(v.name, v)
    return out


def umarell_base(repo) -> str:
    """The umarell base of a working copy: ``UF_APPLIED.json`` ``base`` (apply.py), else ``eccb1bc``."""
    try:
        return json.loads((Path(repo) / "UF_APPLIED.json").read_text()).get("base") or "eccb1bc"
    except (OSError, ValueError):
        return "eccb1bc"


def seed_rngs(seed: int, ompl_too: bool = False) -> None:
    """Seed numpy's / random's RNGs, and OMPL's when ``ompl_too`` (OMPL accepts a seed only before its first
    sample in the process, so the workers seed it once at start; umarell never seeds it)."""
    import random
    import numpy as np
    random.seed(seed)
    np.random.seed(seed % (2 ** 32))
    if not ompl_too:
        return
    try:
        from ompl import util as ou
        ou.RNG.setSeed(int(seed) % (2 ** 31 - 1) or 1)
    except Exception:  # noqa: BLE001 -- best effort; the planner still runs unseeded
        pass


def tool(name: str):
    """The umarell tool's plain function (``@tool`` object's ``.func``)."""
    return _STATE["tools"][name].func


def call(name: str, **kwargs):
    res = tool(name)(**kwargs)
    if isinstance(res, str):
        try:
            res = json.loads(res)
        except Exception:  # noqa: BLE001
            res = {"raw": res}
    return res


def load_task(cell_base: Path, cell: str, task: str = "car_door_spot_welding.json") -> dict:
    return json.loads((Path(cell_base) / cell / "agentic_planner" / "tasks" / task).read_text())
