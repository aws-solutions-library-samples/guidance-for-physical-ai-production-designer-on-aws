"""The numerical library stack a cache was prepared (or a plan made) under.

Component keys hash module sources, cell files and the planner copy, not the numerical libraries: a cache prepared
under another stack (e.g. pin 3.7 / coal / numpy 2 / no toppra) is indistinguishable by its keys from one of the
session's (pin 2.7.0, hpp-fcl 2.4.4, numpy 1.26.4, scipy 1.13.1, ompl 2.0.1, toppra 0.6.3, aarch64 Linux,
CPython 3.12). :func:`fingerprint` records the stack; bundles (MANIFEST.json ``stack``), prepare's run log and
state and plan-cell's result.json carry it, ``import`` compares it (:func:`differences`) and ``prepare`` refuses to
write components without toppra (:func:`require_toppra`); ``plan`` / ``plan-cell`` clamp their repair and prepare
budgets to 0 without toppra (cache-only, :func:`repair_guard`).

Versions come from ``importlib.metadata`` (no numerical import); only a module that is importable but has no
distribution metadata (e.g. a source build on ``PYTHONPATH``) is imported for its ``__version__``. Standard library
only. This module is not key-relevant: no component key hashes it.
"""
from __future__ import annotations

import importlib.metadata as metadata
import importlib.util
import os
import platform
import sys

SCHEMA = "umarell-jam-stack-v1"
# fingerprint key -> (distribution names, in order of preference; the module name)
PACKAGES = {
    "numpy": (("numpy",), "numpy"),
    "scipy": (("scipy",), "scipy"),
    "pinocchio": (("pin", "pinocchio", "cmeel-pinocchio"), "pinocchio"),
    "hppfcl": (("hpp-fcl", "hppfcl", "cmeel-hpp-fcl"), "hppfcl"),
    "coal": (("coal", "cmeel-coal"), "coal"),
    "ompl": (("ompl",), "ompl"),
    "toppra": (("toppra",), "toppra"),
}
# what makes two stacks numerically different (import warns / --strict-stack refuses on these)
COMPARED = ("numpy", "scipy", "pinocchio", "hppfcl", "hppfcl_is_coal_shim", "ompl", "toppra", "machine")
STRICT_ENV = "UMARELL_JAM_STRICT_STACK"


REPAIR_DISABLED = "repair disabled: toppra missing"


class StackMismatch(RuntimeError):
    pass


def _find_spec(module: str):
    try:
        return importlib.util.find_spec(module)
    except (ImportError, ValueError):
        return None


def _version(names, module: str) -> str | None:
    for name in names:
        try:
            return metadata.version(name)
        except metadata.PackageNotFoundError:
            continue
    if _find_spec(module) is None:
        return None
    try:                                     # importable without metadata: its own __version__
        mod = importlib.import_module(module)
    except Exception:  # noqa: BLE001 - a broken install counts as absent
        return None
    v = getattr(mod, "__version__", None)
    return str(v) if v is not None else "unknown"


def _coal_shim(coal: str | None) -> bool:
    """hppfcl is coal's compatibility shim (coal >= 3 ships ``hppfcl`` re-exporting ``coal``)."""
    if coal is None:
        return False
    spec = _find_spec("hppfcl")
    if spec is None or not spec.origin or not os.path.isfile(spec.origin):
        return False
    try:
        with open(spec.origin, encoding="utf-8", errors="replace") as f:
            return "coal" in f.read()
    except OSError:
        return False


def fingerprint() -> dict:
    """{schema, numpy, scipy, pinocchio, hppfcl, hppfcl_is_coal_shim, coal, ompl, toppra, machine, platform,
    python}: versions (None if absent), platform.machine(), sys.platform and the Python major.minor."""
    out = {"schema": SCHEMA}
    for key, (names, module) in PACKAGES.items():
        out[key] = _version(names, module)
    out["hppfcl_is_coal_shim"] = _coal_shim(out["coal"])
    if out["hppfcl_is_coal_shim"] and out["hppfcl"] is None:
        out["hppfcl"] = out["coal"]          # the shim has no distribution of its own: it is coal
    out.update(machine=platform.machine(), platform=sys.platform,
               python=f"{sys.version_info.major}.{sys.version_info.minor}")
    return out


def differences(recorded, local: dict | None = None) -> dict:
    """{key: {"recorded", "local"}} of the :data:`COMPARED` keys that differ."""
    local = fingerprint() if local is None else local
    recorded = recorded or {}
    return {k: {"recorded": recorded.get(k), "local": local.get(k)} for k in COMPARED
            if recorded.get(k) != local.get(k)}


def describe(diff: dict) -> str:
    return ", ".join(f"{k} {v['recorded']!r} (bundle) != {v['local']!r} (here)" for k, v in sorted(diff.items()))


def strict_default() -> bool:
    return os.environ.get(STRICT_ENV, "").strip().lower() in {"1", "true", "yes", "on"}


def require_toppra(allow: bool = False, local: dict | None = None) -> dict:
    """The local fingerprint; raises :class:`StackMismatch` when toppra is missing and ``allow`` is false: without
    it umarell's time parameterisation falls back to Hermite retiming, whose certificates would be written,
    differently timed, under the same component keys."""
    local = fingerprint() if local is None else local
    if local.get("toppra") is None and not allow:
        raise StackMismatch(
            "toppra is not installed: umarell's time parameterisation would fall back to Hermite retiming and "
            "write differently timed certificates under the same component keys. Install toppra (the session "
            "stack: toppra 0.6.3), or pass --allow-stack-mismatch to write components anyway. Cache-only "
            "planning does not need toppra.")
    return local


def repair_guard(budgets: dict, allow: bool = False, local: dict | None = None, warn=None) -> str | None:
    """:data:`REPAIR_DISABLED` when toppra is missing, ``allow`` is false and any of ``budgets`` ({option: seconds},
    the repair / prepare budgets of a planning call) is > 0: the caller then plans cache-only (budgets 0), so a
    cache miss is the ordinary missing_certified_primitives. A note goes to ``warn`` (default stderr). None
    otherwise."""
    if allow or not any(v > 0 for v in budgets.values()):
        return None
    local = fingerprint() if local is None else local
    if local.get("toppra") is not None:
        return None
    if warn is None:
        def warn(message):
            print(message, file=sys.stderr, flush=True)
    asked = ", ".join(f"{k} {v:g}" for k, v in budgets.items() if v > 0)
    warn(f"[umarell_jam] {REPAIR_DISABLED}: {asked} clamped to 0, planning cache-only (umarell's Hermite fallback "
         "would write differently timed certificates under the same keys; --allow-stack-mismatch overrides)")
    return REPAIR_DISABLED
