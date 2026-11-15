#!/usr/bin/env bash
#
# scripts/package-bundle.sh <out.tar.gz>
#
# Packages what scripts/build-release.sh built into one factoryos-viewer-linux-<arch> bundle, the FACTORY_OS_HOME
# layout of ci/release-bundle.sh, which deploy/factoryos-deploy.sh installs (deployment/CONTRACT.md: the public
# deployment packages on its instance and deploys the local file):
#   bin/factoryos-backend, bin/cell-rerun-server, bin/plan2cell, bin/factoryos-terminal-agent, bin/factoryos-mcp
#   web/                       the wasm viewer (web/dist)
#   cells/                     assets/cells (the sample cells)
#   assets/catalogue/          the sample catalogue; assets/sample-data.txt (its large-data manifest)
#   deploy/                    the server's setup (factoryos-deploy runs deploy/provision.sh from here)
#   cell-planner/              the export's factoryos-cell-planner subset: public provisioning installs the planner
#                              router's cell planner from the release (deploy/provision-cell-planner.sh), not from an
#                              archive
#   BUILD-INFO.txt, VERSION.json (when the tree has one)
# Modes, whatever the caller's umask (the release unit's, a CI job's): every directory 0755, every file 0644 except
# what was executable (bin/*, deploy/*.sh, a cell planner script: 0755), the archive's top-level entry included (tar
# records the staging directory's mode; mktemp's 0700 would make the unpacked release root-only). The backend runs
# bin/factoryos-backend as the factoryos user; a bin/* it cannot execute fails the packaging here.
set -euo pipefail
umask 022

out="${1:?usage: package-bundle.sh <out.tar.gz>}"
case "$out" in /*) ;; *) out="$PWD/$out" ;; esac
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"

bins=(target/release/factoryos-backend cell/target/release/cell-rerun-server plan2cell/target/release/plan2cell
      target/release/factoryos-terminal-agent target/release/factoryos-mcp)
for f in "${bins[@]}"; do
  [ -x "$f" ] || { echo "package-bundle: $f is missing: run scripts/build-release.sh first" >&2; exit 1; }
done
[ -f web/dist/index.html ] || { echo "package-bundle: web/dist is missing: run scripts/build-release.sh first" >&2; exit 1; }
[ -d cell-planner ] || { echo "package-bundle: no cell-planner/ in $root (the public export has it at its root)" >&2; exit 1; }

stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
chmod 0755 "$stage"
mkdir -p "$stage/bin" "$stage/web" "$stage/assets"
for f in "${bins[@]}"; do install -m 0755 "$f" "$stage/bin/$(basename "$f")"; done
cp -R web/dist/. "$stage/web/"
cp -R assets/cells "$stage/cells"
cp -R assets/catalogue "$stage/assets/catalogue"
cp assets/sample-data.txt "$stage/assets/"
cp -R deploy "$stage/deploy" && chmod 0755 "$stage"/deploy/*.sh
cp -R cell-planner "$stage/cell-planner"
find "$stage/cell-planner" -name __pycache__ -type d -prune -exec rm -rf {} +
[ -f VERSION.json ] && cp VERSION.json "$stage/"
{
  cat out/build-release.info 2>/dev/null || echo "build:                  (no out/build-release.info)"
  echo "bundle:                 $(basename "$out") (scripts/package-bundle.sh, $(date -u +%FT%TZ))"
} > "$stage/BUILD-INFO.txt"
chmod -R u=rwX,go=rX "$stage"
# The check: no directory without r-x for others, no bin/* without r-x for others (find -perm, GNU and BSD alike).
bad=$( { find "$stage" -type d ! -perm -0555 -print; find "$stage/bin" -type f ! -perm -0555 -print; } \
       | sed "s|^$stage|.|" | tr '\n' ' ')
[ -z "$bad" ] || { echo "package-bundle: error: not readable and executable by others (the service user): $bad" >&2; exit 1; }

mkdir -p "$(dirname "$out")"
tar -czf "$out.partial" -C "$stage" .
mv -f "$out.partial" "$out"
echo "package-bundle: $out ($(du -h "$out" | cut -f1))"
