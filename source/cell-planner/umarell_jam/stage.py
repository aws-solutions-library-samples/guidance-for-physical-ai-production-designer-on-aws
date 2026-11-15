"""Optional private staging of a FactoryOS data root. Never follow writes through supplied asset symlinks.

Only needed for a private copy (the weld_cache session planned on one, tools/stage.py): ``catalogue/``, ``products/`` and
``cells/`` are copied (without plans / recordings / logs / plan2cell / .git), symlinks are resolved through the
source root, and every ``/var/lib/factoryos/`` reference in .urdf / .yaml / .json / .srdf files is rewritten to the
private copy. The rewritten paths are part of the cells' content, so component keys of a staged copy depend on its
absolute location (docs/umarell_jam.md, "Cache compatibility"). Planning directly on ``/var/lib/factoryos`` needs no
staging: ``Cell`` maps ``/var/lib/factoryos/`` references onto ``--workspace`` itself.
"""
from pathlib import Path
import json
import os
import shutil

PREFIX = "/var/lib/factoryos/"
SKIP = {"plans", "recordings", "logs", "plan2cell", ".git"}


def stage(source: Path, dest: Path, cells=None) -> dict:
    source, dest = Path(source).resolve(), Path(dest).resolve()

    def mapped(path):
        s = str(path)
        if s.startswith(PREFIX):
            return source / s.removeprefix(PREFIX)
        return path

    def copy(src, dst):
        if src.is_symlink():
            target = Path(os.readlink(src))
            if not target.is_absolute():
                target = src.parent / target
            src = mapped(target)
        if src.is_dir():
            dst.mkdir(parents=True, exist_ok=True)
            for p in src.iterdir():
                if p.name in SKIP:
                    continue
                copy(p, dst / p.name)
        elif src.is_file():
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
        else:
            raise FileNotFoundError(src)

    if dest.exists():
        raise SystemExit("staging exists; retain immutable snapshot or remove explicitly")
    for name in ["catalogue", "products"]:
        copy(source / name, dest / name)
    if cells:
        for cid in cells:
            copy(source / "cells" / cid, dest / "cells" / cid)
    else:
        copy(source / "cells", dest / "cells")
    changes = []
    for p in dest.rglob("*"):
        if p.is_file() and p.suffix in {".urdf", ".yaml", ".json", ".srdf"}:
            b = p.read_bytes()
            out = b.replace(PREFIX.encode(), (str(dest) + "/").encode())
            if out != b:
                p.write_bytes(out)
                changes.append(str(p.relative_to(dest)))
    record = {"source": str(source), "private_root": str(dest), "path_mapping": {PREFIX: str(dest) + "/"},
              "cells": list(cells) if cells else None, "rewritten_files": changes}
    (dest.parent / "staging.json").write_text(json.dumps(record, indent=2))
    return record
