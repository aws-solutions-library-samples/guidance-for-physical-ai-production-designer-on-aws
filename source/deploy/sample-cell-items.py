#!/usr/bin/env python3
"""The catalogue items a release's sample cells need, one `<collection>/<item>` per line (factoryos-deploy.sh):

    python3 sample-cell-items.py <release>/cells <release>/assets/catalogue

A reference cell (`<cell>/factoryos-cell.json`, format 2, e.g. welding_cell_2_small) references its Robo Fitter scene
`scenes/<fitted_scene>/scene.yaml` in the catalogue, and that scene its dressed robots, fixtures and floor plan
(`../../<collection>/<item>/...`, `../../floor-plans/<file>`). Printed: the scene (`scenes/<name>`) and each of those
items that the catalogue has, in order, each once. A legacy cell (no manifest, or its own scene.yaml) needs nothing.
Paths outside the catalogue (absolute ones, the products folder) are not catalogue items: skipped. Standard library
only (the host's python3; no PyYAML): the scene's `dressed_urdf:`, `urdf:` and `geometry_file:` lines are read as
text, which is how the Robo Fitter writes them.
"""
from __future__ import annotations

import json
import posixpath
import re
import sys
from pathlib import Path

LINE = re.compile(r"""^\s*(?:-\s+)?(dressed_urdf|urdf|geometry_file):\s*['"]?([^'"#]+?)['"]?\s*(?:#.*)?$""")
NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.@-]*$")


def scene_items(scene: str, text: str, catalogue: Path) -> list[str]:
    """The catalogue items scene `scene` (its text) names, from its folder scenes/<scene>/."""
    out = []
    for line in text.splitlines():
        m = LINE.match(line)
        if not m or m.group(2).startswith("/"):
            continue
        path = posixpath.normpath(posixpath.join("scenes", scene, m.group(2).strip()))
        parts = path.split("/")
        if len(parts) < 2 or parts[0] in ("..", "scenes") or not all(NAME.match(p) for p in parts[:2]):
            continue
        item = "/".join(parts[:2])
        if (catalogue / item).exists():
            out.append(item)
    return out


def items(cells: Path, catalogue: Path) -> list[str]:
    out: list[str] = []
    for cell in sorted(p for p in cells.iterdir() if p.is_dir()) if cells.is_dir() else []:
        if (cell / "scene.yaml").is_file() or (cell / "scene.factoryos.yaml").is_file():
            continue   # a legacy cell: its own scene wins (cell_resolve.rs)
        try:
            manifest = json.loads((cell / "factoryos-cell.json").read_text())
        except (OSError, ValueError):
            continue
        scene = manifest.get("fitted_scene") if isinstance(manifest, dict) else None
        if not (isinstance(manifest, dict) and int(manifest.get("format", 1)) >= 2 and manifest.get("kind") != "copy"
                and isinstance(scene, str) and NAME.match(scene)):
            continue
        scene_file = catalogue / "scenes" / scene / "scene.yaml"
        if not scene_file.is_file():
            continue
        for item in [f"scenes/{scene}", *scene_items(scene, scene_file.read_text(), catalogue)]:
            if item not in out:
                out.append(item)
    return out


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: sample-cell-items.py <cells dir> <catalogue dir>", file=sys.stderr)
        return 2
    for item in items(Path(argv[0]), Path(argv[1])):
        print(item)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
