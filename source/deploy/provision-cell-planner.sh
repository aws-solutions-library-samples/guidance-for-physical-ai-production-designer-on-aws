#!/bin/bash
# provision-cell-planner.sh <release dir>
#
# The planner router's factoryos-cell-planner (deploy/cell-planner.txt) and its venv, idempotent; run
# by provision.sh as root. A failure here does not stop the deploy (provision.sh logs it): the router
# then offers the umarell planner only (its planners report "not installed on this host").
#
#   /opt/factoryos-cell-planner/router/src    the trimmed git archive (root-owned, read-only for the router)
#   /opt/factoryos-cell-planner/router/venv   requirements-cell-planner.txt (prebuilt wheels only)
#   /opt/factoryos-cell-planner/router/jam-venv  umarell_jam's: the cell planner's requirements-umarell.txt
#                                             (+ toppra: deploy/toppra-wheel.txt's wheel, else deploy/toppra.pin's
#                                             sdist, built here), if the cell planner has umarell_jam
#   /var/lib/factoryos/cell-planner/          the router's data (factoryos): work/ (jobs, cell views) and
#                                             generated/ (env/assets/generated of the source tree links here)
#
# Anything else in /opt/factoryos-cell-planner (e.g. a hand-made install in src/, venv/) is left alone.
#
# Public mode (FACTORYOS_PUBLIC=1, as provision.sh): the tree is the release's cell-planner/ (the public export's
# subset, which scripts/package-bundle.sh puts into the bundle), not an archive; its stamp is the sha256 of the tree
# (every file's path and content), so an update that changes it installs it again. Without it this fails.
set -euo pipefail
release="${1:?usage: provision-cell-planner.sh <release dir>}"
here="$release/deploy"
# The release archive (deploy/*.txt and assets/sample-data.txt keys): /etc/factoryos/artifacts.env in prod, else the
# internal build's default (deploy/internal.env; the environment comes first). The same in provision.sh.
[ -r "$here/internal.env" ] && . "$here/internal.env"
[ -r /etc/factoryos/artifacts.env ] && . /etc/factoryos/artifacts.env
ARCHIVE_BUCKET=${FACTORYOS_BUILDS_BUCKET:-${FACTORYOS_DEFAULT_BUILDS_BUCKET:-}}
ARCHIVE_REGION=${FACTORYOS_BUILDS_REGION:-${FACTORYOS_DEFAULT_BUILDS_REGION:-us-east-1}}
# Public mode (see the header); the same in provision.sh.
public=$( [ -r /etc/factoryos/stack.env ] && . /etc/factoryos/stack.env; echo "${FACTORYOS_PUBLIC:-0}")
# archive_get <key> <file>: <key> of the release archive into <file>. Without one (no deploy/internal.env, no
# FACTORYOS_BUILDS_BUCKET from /etc/factoryos/artifacts.env or the environment: the public export's scripts) it stops,
# saying so, instead of aws failing on "s3:///<key>".
archive_get() {
  [ -n "$ARCHIVE_BUCKET" ] || { echo "no release archive to download $1 from: deploy/internal.env is missing and" \
    "neither /etc/factoryos/artifacts.env nor the environment sets FACTORYOS_BUILDS_BUCKET" >&2; exit 1; }
  aws s3 cp "s3://$ARCHIVE_BUCKET/$1" "$2" --region "$ARCHIVE_REGION" --only-show-errors
}

root=/opt/factoryos-cell-planner/router
src="$root/src"
venv="$root/venv"
state=/var/lib/factoryos/cell-planner
umarell=/var/lib/factoryos/umarell

log() { echo "[provision] $*"; }

if [ "$public" = 1 ]; then
  tree="$release/cell-planner"
  [ -d "$tree" ] || { echo "$tree is missing (public mode installs the cell planner from the release)" >&2; exit 1; }
  # The tree's sha256: every regular file's path and content, in a fixed order.
  sha=$( (cd "$tree" && find . -type f ! -path '*/__pycache__/*' -print | LC_ALL=C sort | while IFS= read -r f; do
          printf '%s  %s\n' "$(sha256sum < "$f" | cut -d' ' -f1)" "$f"; done) | sha256sum | cut -d' ' -f1)
  commit="tree-${sha:0:12}"
  pin_line="$commit  $sha  $(basename "$release")"
else
  read -r commit sha key < <(grep -vE '^\s*(#|$)' "$here/cell-planner.txt" | head -1)
  [ -n "${key:-}" ] || { echo "deploy/cell-planner.txt pins no archive" >&2; exit 1; }
fi

install -d -o factoryos -g factoryos "$state" "$state/work" "$state/generated"
install -d -m 0755 "$root"

if [ "$(cat "$src/.cell-planner.sha256" 2>/dev/null)" != "$sha" ]; then
  log "cell planner $commit"
  tmp=$(mktemp -d)
  trap 'rm -rf "$tmp"' EXIT
  if [ "$public" = 1 ]; then
    cp -R "$tree" "$tmp/cell-planner"
    find "$tmp/cell-planner" -name __pycache__ -type d -prune -exec rm -rf {} +
  else
    archive_get "$key" "$tmp/cell-planner.tar.gz"
    echo "$sha  $tmp/cell-planner.tar.gz" | sha256sum -c - >/dev/null
    tar -xzf "$tmp/cell-planner.tar.gz" -C "$tmp"
  fi
  # env/scene.py writes the patched robot URDFs into env/assets/generated: the router's data instead.
  rm -rf "$tmp/cell-planner/env/assets/generated"
  ln -s "$state/generated" "$tmp/cell-planner/env/assets/generated"
  # The router reports the first word as the cell planner's commit (public mode: tree-<12 hex of the tree's sha256>).
  if [ "$public" = 1 ]; then
    echo "$pin_line" > "$tmp/cell-planner/.cell-planner.txt"
  else
    grep -vE '^\s*(#|$)' "$here/cell-planner.txt" | head -1 > "$tmp/cell-planner/.cell-planner.txt"
  fi
  echo "$sha" > "$tmp/cell-planner/.cell-planner.sha256"
  chown -R root:root "$tmp/cell-planner"
  chmod -R u=rwX,go=rX "$tmp/cell-planner"
  rm -rf "$src.old"
  [ -d "$src" ] && mv "$src" "$src.old"
  mv "$tmp/cell-planner" "$src"
  rm -rf "$src.old"
  # The patched URDFs name the old paths' meshes; they are regenerated on demand.
  find "$state/generated" -mindepth 1 -delete
fi

want=$(sha256sum "$here/requirements-cell-planner.txt" | cut -d' ' -f1)
if [ "$(cat "$venv/.requirements.sha256" 2>/dev/null)" != "$want" ]; then
  log "cell planner venv"
  rm -rf "$venv"
  python3 -m venv "$venv"
  "$venv/bin/pip" install --quiet --upgrade pip
  # Prebuilt wheels only: a missing one should fail here, not compile for an hour on the server.
  nice -n 10 "$venv/bin/pip" install --quiet --no-cache-dir --only-binary=:all: -r "$here/requirements-cell-planner.txt"
  chmod -R u=rwX,go=rX "$venv"
  echo "$want" > "$venv/.requirements.sha256"
fi

# --- umarell_jam's own venv (planner umarell_jam), only when the cell planner has umarell_jam: the session's numerical
# stack as the cell planner pins it in requirements-umarell.txt (pin 2.7.0, hpp-fcl 2.4.4, numpy 1.26.4, scipy 1.13.1,
# ompl 2.0.1) and toppra 0.6.3. Its component cache keys depend on that stack, so it cannot share umarell's venv
# (requirements-planner.txt: pin 3.7, numpy 2, no toppra). Prebuilt wheels only, without the freeze's test tools, but
# for toppra: it has no aarch64 wheel on PyPI, so it is the one pinned in deploy/toppra-wheel.txt (built once, in the
# release archive: internal servers), else built here from the sdist deploy/toppra.pin pins (toppra_from_sdist), else
# none (the router's gate warns, umarell_jam cannot repair or prepare and its umarell_u3 fallback plans).
# A failure here never fails the cell planner's setup. Public mode: a venv that cannot be built, has no toppra or does
# not import marks umarell_jam provision-incomplete (factoryos-deploy then leaves the provision unstamped, so the next
# deploy retries it; the release goes on). FACTORY_OS_UMARELL_JAM=0 in workspace.env skips it.
jam_venv="$root/jam-venv"
# toppra_version: the toppra version deploy/toppra.pin pins (none without one).
toppra_version() { [ -f "$here/toppra.pin" ] || return 0; sed -n 's/^toppra==\([^ ]*\).*/\1/p' "$here/toppra.pin" | head -1; }
# toppra_from_sdist <venv>: toppra as deploy/toppra.pin pins it, built in <venv> from its sdist: the pin's build
# requirements (hash-checked wheels), the sdist (hash-checked by pip and here), a wheel built against <venv>'s numpy
# (--no-build-isolation: nothing else is downloaded), installed without the index.
toppra_from_sdist() {
  local venv=$1 pin="$here/toppra.pin" t rc=0
  t=$(mktemp -d "${TMPDIR:-/tmp}/toppra-build.XXXXXX") || return 1
  _toppra_build "$venv" "$pin" "$t" || rc=1
  rm -rf "$t"
  return "$rc"
}
_toppra_build() {
  local venv=$1 pin=$2 t=$3 tver tsha sdist wheels
  tver=$(toppra_version)
  tsha=$(sed -n 's/^toppra==[^ ]* .*--hash=sha256:\([0-9a-f]\{64\}\).*/\1/p' "$pin" | head -1)
  [ -n "$tver" ] && [ -n "$tsha" ] || { log "deploy/toppra.pin pins no toppra==<version> --hash=sha256:<64 hex>"; return 1; }
  grep -vE '^\s*(#|$)' "$pin" | grep -vE '^toppra==' > "$t/build.txt" || { log "deploy/toppra.pin lists no build requirements"; return 1; }
  grep -E '^toppra==' "$pin" | head -1 > "$t/toppra.txt"
  log "umarell_jam venv: toppra $tver from its sdist"
  nice -n 10 "$venv/bin/pip" install --quiet --no-cache-dir --require-hashes --only-binary=:all: -r "$t/build.txt" \
    || return 1
  "$venv/bin/pip" download --quiet --no-cache-dir --no-deps --no-binary=:all: --no-build-isolation --require-hashes \
    -d "$t/sdist" -r "$t/toppra.txt" || return 1
  sdist="$t/sdist/toppra-$tver.tar.gz"
  [ -f "$sdist" ] || { log "pip download gave no toppra-$tver.tar.gz"; return 1; }
  echo "$tsha  $sdist" | sha256sum -c - >/dev/null 2>&1 || { log "toppra-$tver.tar.gz: sha256 mismatch (want $tsha)"; return 1; }
  nice -n 10 "$venv/bin/pip" wheel --quiet --no-cache-dir --no-build-isolation --no-deps --no-index -w "$t/wheel" "$sdist" \
    || return 1
  wheels=("$t/wheel/toppra-$tver-"*.whl)
  [ "${#wheels[@]}" = 1 ] && [ -f "${wheels[0]}" ] || { log "pip wheel gave no single toppra-$tver wheel"; return 1; }
  "$venv/bin/pip" install --quiet --no-deps --no-index "${wheels[0]}"
}
jam_venv_setup() {
  local req="$src/requirements-umarell.txt" want tver tsha tkey wheel tmpw
  want=$( (cat "$req"; cat "$here/toppra-wheel.txt" 2>/dev/null || true; cat "$here/toppra.pin" 2>/dev/null || true) |
          sha256sum | cut -d' ' -f1)
  [ "$(cat "$jam_venv/.requirements.sha256" 2>/dev/null)" = "$want" ] && return 0
  log "umarell_jam venv"
  rm -rf "$jam_venv"
  python3 -m venv "$jam_venv" || return 1
  "$jam_venv/bin/pip" install --quiet --upgrade pip || return 1
  grep -vE '^(toppra|pytest|hypothesis|pluggy|iniconfig)==' "$req" > "$jam_venv/requirements.txt" || return 1
  nice -n 10 "$jam_venv/bin/pip" install --quiet --no-cache-dir --only-binary=:all: -r "$jam_venv/requirements.txt" \
    || return 1
  if read -r tver tsha tkey < <(grep -vE '^\s*(#|$)' "$here/toppra-wheel.txt" 2>/dev/null | head -1) && [ -n "${tkey:-}" ]; then
    tmpw=$(mktemp -d)
    wheel="$tmpw/$(basename "$tkey")"
    # archive_get in a subshell: without a release archive (public mode) it exits, which must fail this venv (the
    # caller's WARNING, umarell_jam falls back to umarell_u3), not end the whole script before the cell planner's check.
    ( archive_get "$tkey" "$wheel" ) && echo "$tsha  $wheel" | sha256sum -c - >/dev/null \
      && "$jam_venv/bin/pip" install --quiet --no-deps --no-index "$wheel" || { rm -rf "$tmpw"; return 1; }
    rm -rf "$tmpw"
  elif [ -f "$here/toppra.pin" ]; then
    toppra_from_sdist "$jam_venv" || { log "WARNING: toppra $(toppra_version) could not be built from its sdist"; return 1; }
  elif [ "$public" = 1 ]; then
    log "WARNING: no toppra pinned (deploy/toppra.pin): umarell_jam's venv would have no toppra"
    return 1
  else
    log "WARNING: no toppra pinned (deploy/toppra-wheel.txt, deploy/toppra.pin): umarell_jam's venv has no toppra"
  fi
  chmod -R u=rwX,go=rX "$jam_venv"
  echo "$want" > "$jam_venv/.requirements.sha256"
}
# mark_incomplete <step>: <step> into /etc/factoryos/provision-incomplete (once), which factoryos-deploy reports and
# which keeps the provision unstamped (provision.sh's own).
mark_incomplete() {
  mkdir -p /etc/factoryos
  grep -qxF "$1" /etc/factoryos/provision-incomplete 2>/dev/null || echo "$1" >> /etc/factoryos/provision-incomplete
}
umarell_jam=$(. /etc/factoryos/workspace.env 2>/dev/null; echo "${FACTORY_OS_UMARELL_JAM:-1}")
if [ -d "$src/umarell_jam" ] && [ -f "$src/requirements-umarell.txt" ] && [ "$umarell_jam" != 0 ]; then
  jam_ok=yes
  # Public mode: toppra is required (the pinned version); elsewhere a venv without it is a warning, as before.
  jam_toppra=""; [ "$public" = 1 ] && jam_toppra=$(toppra_version)
  if jam_venv_setup; then
    sudo -u factoryos -H env PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 JAM_TOPPRA="$jam_toppra" "$jam_venv/bin/python" -W ignore -c '
import os, sys
import numpy, pinocchio, hppfcl, scipy, ompl  # noqa: F401 -- the numerical runtime umarell_jam keys its cache by
try:
    import toppra  # noqa: F401 -- toppra 0.6.3 has no __version__: read it from the distribution
    from importlib.metadata import version
    t = version("toppra")
except ImportError:
    t = "MISSING"
print("[provision] umarell_jam venv: pin", pinocchio.__version__, "hpp-fcl", hppfcl.__version__, "numpy",
      numpy.__version__, "toppra", t)
need = os.environ.get("JAM_TOPPRA", "")
if need and t != need:
    sys.exit(f"umarell_jam venv: toppra {t}, not {need}")
' || { log "WARNING: umarell_jam's venv does not import its numerical runtime"; jam_ok=no; }
  else
    log "WARNING: umarell_jam's venv could not be built; the router's umarell_jam falls back to umarell_u3"
    rm -rf "$jam_venv"
    jam_ok=no
  fi
  if [ "$jam_ok" = no ] && [ "$public" = 1 ]; then
    # Rebuilt on the retry, not kept stamped.
    rm -rf "$jam_venv"
    log "WARNING: umarell_jam is not set up (provision-incomplete); its jobs fall back to umarell_u3, the next deploy retries it"
    mark_incomplete umarell_jam
  fi
else
  rm -rf "$jam_venv"
fi

# As the router runs it: the router's user, the read-only tree, the umarell checkout's cells.
(cd "$src" && sudo -u factoryos -H env PYTHONPATH=. PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 MUJOCO_GL=disable \
  UMARELL_REPO="$umarell" "$venv/bin/python" -W ignore -c '
import torch, mujoco, pinocchio, torch_geometric  # noqa: F401
import tools.btexport, env.cell  # noqa: F401
print("[provision] cell planner ok: torch", torch.__version__, "mujoco", mujoco.__version__, "pin", pinocchio.__version__)
')
