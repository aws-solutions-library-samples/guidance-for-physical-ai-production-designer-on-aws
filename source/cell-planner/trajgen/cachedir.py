"""Where trajgen keeps its caches (reach tables, box distance fields, MuJoCo URDF copies).

``$TRAJGEN_CACHE_DIR`` when set; else ``data/trajgen/cache`` in the checkout, when it can be written; else a
per-user folder in the temp dir. A deployed cell planner (FactoryOS: /opt/factoryos-cell-planner, owned by
root, run as the service user) cannot write into its own install, and every cache write used to crash there.
"""
import os
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHECKOUT_CACHE = ROOT / "data" / "trajgen" / "cache"


def _writable(path: Path) -> bool:
    """Whether ``path`` exists and can be written, or can be created (its nearest existing parent can be)."""
    p = path
    while not p.exists():
        if p.parent == p:
            return False
        p = p.parent
    return p.is_dir() and os.access(p, os.W_OK | os.X_OK)


def cache_dir() -> Path:
    env = os.environ.get("TRAJGEN_CACHE_DIR", "").strip()
    if env:
        return Path(env)
    if _writable(CHECKOUT_CACHE):
        return CHECKOUT_CACHE
    return Path(tempfile.gettempdir()) / f"trajgen-cache-{os.getuid() if hasattr(os, 'getuid') else 'user'}"


CACHE_DIR = cache_dir()
