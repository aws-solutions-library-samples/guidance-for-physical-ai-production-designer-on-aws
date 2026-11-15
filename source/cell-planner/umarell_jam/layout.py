"""Where umarell_jam reads and writes: one writable ``--root`` and one FactoryOS ``--workspace``.

``--root R`` (default ``data/umarell_jam`` in this repository, gitignored):

* ``R/components/``   the content-addressed component store (``Store``; ``--cache`` overrides it)
* ``R/umarell/``      the patched Umarell copy (apply.py U3 + upstream fixes + the integration hooks); ``--umarell``
  naming an already patched copy (UF_APPLIED.json + WELD_INTEGRATION.json) uses that one instead, ``--umarell``
  naming a plain checkout is the source R/umarell is built from
* ``R/plans/bundles/<version>/``  published plan bundles (bt.json, blackboard.json, bt.xml, manifest.json);
  ``R/plans/repair-logs/`` and ``R/plans/execution-failures/`` beside them (``--output`` overrides the bundle dir;
  the logs then go to its parent, as before)
* ``R/runs/<job>/``   one ``plan-cell`` job (progress.json, request.json, result.json, scratch);
  ``R/runs/prepare/`` the preparation steps' logs, diagnostics and worker scratch
* ``R/logs/``         setup log, prepare run logs, the cache manifest
* ``R/workspace/``    only after ``stage``: a private copy of a FactoryOS data root
* ``R/.setup.lock``   held while R/umarell is created or replaced

``--workspace W`` is a FactoryOS data root with ``cells/``, ``catalogue/`` and ``products/`` (``/var/lib/factoryos``
on a workspace). Default: ``R/workspace`` when it exists (staged), else ``/var/lib/factoryos``.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = REPO / "data" / "umarell_jam"
FACTORYOS_ROOT = Path("/var/lib/factoryos")


CELL_MANIFEST = Path("agentic_planner") / "cell.yaml"


def cell_ids(workspace) -> list[str]:
    """The cells of a FactoryOS workspace: directories of ``W/cells`` that have ``agentic_planner/cell.yaml``.

    A live workspace's ``cells/`` also holds directories that are no cell (yet): the hidden ``.backup`` of the
    workspace backup, and cells that were never run through Prepare Cell (no ``agentic_planner/``). Loading those as
    cells raises ``CellLoadError`` and used to stop every ``prepare`` run (the manifest export after ``augment``)."""
    root = Path(workspace) / "cells"
    if not root.is_dir():
        return []
    return [p.name for p in sorted(root.iterdir())
            if p.is_dir() and not p.name.startswith(".") and (p / CELL_MANIFEST).is_file()]


def is_patched_copy(path) -> bool:
    """A patched, hook-installed Umarell copy (apply.py's UF_APPLIED.json and install's WELD_INTEGRATION.json), as
    opposed to a plain checkout."""
    return path is not None and (Path(path) / "UF_APPLIED.json").is_file() \
        and (Path(path) / "WELD_INTEGRATION.json").is_file()


@dataclass(frozen=True)
class Layout:
    root: Path
    workspace: Path
    components_override: Path | None = None     # --cache / --components
    umarell_override: Path | None = None        # --umarell naming an already patched copy
    plans_override: Path | None = None          # --output

    @classmethod
    def make(cls, root=None, workspace=None, components=None, umarell=None, output=None) -> "Layout":
        root = Path(root or DEFAULT_ROOT).resolve()
        if workspace is None:
            workspace = root / "workspace" if (root / "workspace" / "cells").is_dir() else FACTORYOS_ROOT
        res = lambda p: Path(p).resolve() if p else None  # noqa: E731
        return cls(root, Path(workspace).resolve(), res(components),
                   res(umarell) if is_patched_copy(umarell) else None, res(output))

    @property
    def components(self) -> Path:
        return self.components_override or self.root / "components"

    @property
    def umarell(self) -> Path:
        return self.umarell_override or self.root / "umarell"

    @property
    def plans(self) -> Path:
        return self.plans_override or self.root / "plans" / "bundles"

    @property
    def runs(self) -> Path:
        return self.root / "runs"

    @property
    def prepare_runs(self) -> Path:
        """The session's ``runs/`` for the preparation steps."""
        return self.root / "runs" / "prepare"

    @property
    def logs(self) -> Path:
        return self.root / "logs"

    @property
    def manifest(self) -> Path:
        return self.logs / "cache-manifest.json"

    @property
    def setup_lock(self) -> Path:
        return self.root / ".setup.lock"

    def activate(self) -> "Layout":
        """Make the patched Umarell copy and this repository importable (worker processes call this first)."""
        for p in (str(REPO), str(self.umarell)):
            if p not in sys.path:
                sys.path.insert(0, p)
        return self

    def cells(self, selected=None) -> list[str]:
        """Cell ids under the workspace (``selected`` keeps the given order and must exist)."""
        if selected:
            missing = [c for c in selected if not (self.workspace / "cells" / c).is_dir()]
            if missing:
                raise ValueError(f"no such cell(s) in {self.workspace / 'cells'}: {missing}")
            return list(selected)
        return cell_ids(self.workspace)
