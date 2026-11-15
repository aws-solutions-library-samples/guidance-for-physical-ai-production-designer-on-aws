#!/usr/bin/env bash
# Fails if factoryos-backend depends on Rerun (any `re_*` or `rerun*` crate), directly or not.
# The backend only publishes renderer-neutral scenes (factoryos-protocol's `scene`); everything
# Rerun is the viewer's (crates/factoryos-viewer/src/scene3d/rerun).
#
#   scripts/check-backend-no-rerun.sh
#
# Run by ci/build-in-container.sh. Needs the workspace resolvable (third_party/rerun fetched, for
# the viewer's path dependencies), not built.
set -euo pipefail

cd "$(dirname "$0")/.."
tree=$(cargo tree --locked -p factoryos-backend -e normal,build --prefix none --format '{p}')
found=$(grep -E '^(re_[A-Za-z0-9_]+|rerun[A-Za-z0-9_-]*) v' <<<"$tree" | sort -u || true)
if [ -n "$found" ]; then
    echo "factoryos-backend must not depend on Rerun, but its dependency tree has:" >&2
    echo "$found" >&2
    echo "(cargo tree -p factoryos-backend -e normal,build -i <crate> shows why)" >&2
    exit 1
fi
echo "factoryos-backend: no Rerun crate among its $(sort -u <<<"$tree" | wc -l | tr -d ' ') dependencies"
