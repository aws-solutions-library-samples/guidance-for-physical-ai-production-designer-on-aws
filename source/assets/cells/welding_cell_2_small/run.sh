#!/usr/bin/env bash
# Start the trajectory player for this cell on the cell-runner port (9902).
# Run this BEFORE opening the extension's 3D Cell Viewer.
set -euo pipefail
cd "$(dirname "$0")"
exec python3 scripts/trajectory_player.py "$@"
