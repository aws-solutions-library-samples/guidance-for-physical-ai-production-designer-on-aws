#!/bin/bash
# fetch-umarell.sh <release dir> <data dir>
#
# The umarell planner of a public deployment (deploy/provision.sh's public mode, FACTORYOS_PUBLIC=1), idempotent.
# Internal servers install it from pinned archives instead (deploy/umarell.txt, umarell-base.txt). Run as root.
#
#   <data>/umarell-base   the plain commit of deploy/umarell.pin (root-owned, read-only): the source the planner
#                         router's umarell_u3 patches itself (deploy/planner-router/umarell_u3.py u3_source)
#   <data>/umarell        the chat agent's checkout: that commit with the cell planner's upstream fixes for it
#                         applied (<release>/cell-planner/docs/upstream/patches_<first 7 hex>/series.txt, in order,
#                         `patch -p1 --forward --batch`, no rejects or backups: as scripts/publish-umarell.sh --fixes
#                         makes deploy/umarell.txt's archive), then the public deployment's own patches
#                         (<release>/deploy/umarell-public/*.patch, in name order, the same way; e.g.
#                         0001-trajectory-mcp-host.patch: the planner MCP listens on 127.0.0.1, not 0.0.0.0). Its
#                         .venv-planner survives an update (provision.sh builds it).
# Each carries a .umarell.sha256 stamp: the pin's commit (base), <commit>+<sha256 of the series and its patches> (the
# chat tree), +<sha256 of the public patches' names and contents> when there are any. A tree whose stamp matches is
# left alone, so a re-run is a no-op; a changed series or public patch rebuilds the chat tree.
#
# Where the commit comes from:
#   FACTORYOS_UMARELL_TARBALL (+ FACTORYOS_UMARELL_SHA256, required with it): a local `git archive --prefix=umarell/`
#     of the commit (the installer's optional vendor/umarell-<sha12>.tar.gz, e.g. for an instance without GitHub);
#     checked by sha256 and, when the archive records one (git archive does), by its commit id.
#   otherwise: `git fetch --depth 1 <url> <commit>` of deploy/umarell.pin's line, archived the same way (the same
#     tree as the tarball: committed files only, export-ignore honoured, no .git).
#
# fetch-umarell.sh <release dir> <data dir> --jam
#
# The plain umarell commit the planner router's umarell_jam is built on (deploy/umarell-jam.pin), and nothing
# else: <data>/umarell-<first 7 hex> (root-owned, read-only; found by deploy/planner-router/umarell_jam.py jam_source),
# unpatched (umarell_jam patches its own copy), stamped .umarell.sha256 = the pin's commit; a matching stamp is a no-op.
# From FACTORYOS_UMARELL_JAM_TARBALL (+ FACTORYOS_UMARELL_JAM_SHA256, required with it; the installer's
# --umarell-jam-tarball) checked as above, otherwise `git fetch --depth 1` of the pin's line. Its own run and exit
# code: provision.sh treats a failure as umarell_jam not set up (provision-incomplete), not as a failed provision.
set -euo pipefail
usage="usage: fetch-umarell.sh <release dir> <data dir> [--jam]"
release="${1:?$usage}"
data="${2:?$usage}"
mode="${3:-}"
case "$mode" in ''|--jam) ;; *) echo "fetch-umarell: unknown option '$mode' ($usage)" >&2; exit 1 ;; esac

log() { echo "[umarell] $*"; }
die() { echo "fetch-umarell: $*" >&2; exit 1; }
sha256() { sha256sum "$1" | cut -d' ' -f1; }

# get_tree <commit> <url> <pin name> <env prefix> <dir>: the commit's tree as <dir>/umarell, from $<prefix>_TARBALL
# (checked against $<prefix>_SHA256 and, when the archive records one, its commit id) or from git.
get_tree() {
  local commit=$1 url=$2 pin_name=$3 tar_var="$4_TARBALL" sha_var="$4_SHA256" dir=$5 tarball want recorded
  tarball=${!tar_var:-}
  want=${!sha_var:-}
  mkdir -p "$dir"
  if [ -n "$tarball" ]; then
    [ -n "$want" ] || die "$tar_var needs $sha_var"
    [ -f "$tarball" ] || die "$tarball is missing"
    log "umarell ${commit:0:7} from $(basename "$tarball")"
    [ "$(sha256 "$tarball")" = "$want" ] || die "$tarball: sha256 mismatch (want $want)"
    if command -v git >/dev/null; then
      recorded=$(gzip -dc "$tarball" | git get-tar-commit-id 2>/dev/null || true)
      [ -z "$recorded" ] || [ "$recorded" = "$commit" ] \
        || die "$tarball is an archive of $recorded, deploy/$pin_name pins $commit"
    fi
    tar -xzf "$tarball" -C "$dir"
    [ -d "$dir/umarell" ] && [ "$(find "$dir" -mindepth 1 -maxdepth 1 | wc -l | tr -d ' ')" = 1 ] \
      || die "$tarball is not a git archive --prefix=umarell/ (one top directory umarell/)"
  else
    log "umarell ${commit:0:7} from $url"
    git init -q "$dir.git"
    # No credential prompt: a repository that is not public fails at once (provision.sh says so), never waits on a tty.
    GIT_TERMINAL_PROMPT=0 git -C "$dir.git" fetch -q --depth 1 "$url" "$commit" || die "cannot fetch $commit from $url"
    git -C "$dir.git" checkout -q FETCH_HEAD
    [ "$(git -C "$dir.git" rev-parse HEAD)" = "$commit" ] || die "$url gave $(git -C "$dir.git" rev-parse HEAD), not $commit"
    git -C "$dir.git" archive --format=tar --prefix=umarell/ HEAD | tar -xf - -C "$dir"
    rm -rf "$dir.git"
  fi
}

# read_pin <pin>: its first '<commit> <url>' line into commit, url and short.
read_pin() {
  [ -f "$1" ] || die "$1 is missing"
  read -r commit url _ < <(grep -vE '^\s*(#|$)' "$1" | head -1) || die "$1 has no '<commit> <url>' line"
  [[ "$commit" =~ ^[0-9a-f]{40}$ ]] || die "$1: '$commit' is not a 40-hex commit"
  [ -n "${url:-}" ] || die "$1: no URL after the commit"
  short=${commit:0:7}
}

as_root=no; [ "$(id -u)" = 0 ] && as_root=yes

# --- --jam: umarell_jam's plain commit only
if [ "$mode" = --jam ]; then
  read_pin "$release/deploy/umarell-jam.pin"
  jam="$data/umarell-$short"
  if [ "$(cat "$jam/.umarell.sha256" 2>/dev/null)" = "$commit" ]; then
    log "umarell $short for umarell_jam: up to date"
    exit 0
  fi
  mkdir -p "$data"
  tmp=$(mktemp -d "$data/.umarell-fetch.XXXXXX")
  trap 'rm -rf "$tmp"' EXIT
  # An update keeps the installer's recorded --umarell-jam-tarball: one of another commit than this release's pin (the
  # pin moved, e.g. 76ea13a -> the public 8a02db9) is left out with a WARNING and the pin's commit fetched from git.
  if [ -n "${FACTORYOS_UMARELL_JAM_TARBALL:-}" ] && [ -f "$FACTORYOS_UMARELL_JAM_TARBALL" ] && command -v git >/dev/null; then
    recorded=$(gzip -dc "$FACTORYOS_UMARELL_JAM_TARBALL" | git get-tar-commit-id 2>/dev/null || true)
    if [ -n "$recorded" ] && [ "$recorded" != "$commit" ]; then
      log "WARNING: $(basename "$FACTORYOS_UMARELL_JAM_TARBALL") is an archive of ${recorded:0:7}, deploy/umarell-jam.pin" \
          "pins $short: left out, $short from $url instead"
      unset FACTORYOS_UMARELL_JAM_TARBALL FACTORYOS_UMARELL_JAM_SHA256
    fi
  fi
  get_tree "$commit" "$url" umarell-jam.pin FACTORYOS_UMARELL_JAM "$tmp/src"
  [ -d "$tmp/src/umarell/agents" ] || die "the umarell tree of $short has no agents/ (not umarell?)"
  echo "$commit" > "$tmp/src/umarell/.umarell.sha256"
  [ "$as_root" = yes ] && chown -R root:root "$tmp/src/umarell"
  chmod -R u=rwX,go=rX "$tmp/src/umarell"
  rm -rf "$jam" && mv "$tmp/src/umarell" "$jam"
  log "umarell $short for umarell_jam: done"
  exit 0
fi

read_pin "$release/deploy/umarell.pin"

patches="$release/cell-planner/docs/upstream/patches_$short"
[ -f "$patches/series.txt" ] || die "$patches/series.txt is missing (the release has no fixes for umarell $short)"
series=()
while read -r p; do series+=("$p"); done < <(grep -vE '^\s*(#|$)' "$patches/series.txt")
[ "${#series[@]}" -gt 0 ] || die "$patches/series.txt lists no patch"
series_sha=$( (cat "$patches/series.txt"; for p in "${series[@]}"; do
                [ -f "$patches/$p" ] || { echo "missing $p" >&2; exit 1; }; cat "$patches/$p"; done) | sha256sum | cut -d' ' -f1) \
  || die "$patches/series.txt names a patch that is not there"

# The public deployment's own patches: the chat tree only, after the series, in name order (C collation).
public_dir="$release/deploy/umarell-public"
public_patches=()
if [ -d "$public_dir" ]; then
  while IFS= read -r p; do public_patches+=("$p"); done \
    < <(find "$public_dir" -mindepth 1 -maxdepth 1 -type f -name '*.patch' -exec basename {} \; | LC_ALL=C sort)
fi
public_sha=""
if [ "${#public_patches[@]}" -gt 0 ]; then
  public_sha=$(for p in "${public_patches[@]}"; do printf '%s\n' "$p"; cat "$public_dir/$p"; done | sha256sum | cut -d' ' -f1)
fi

base="$data/umarell-base"
chat="$data/umarell"
want_base="$commit"
want_chat="$commit+$series_sha${public_sha:+"+$public_sha"}"
if [ "$(cat "$base/.umarell.sha256" 2>/dev/null)" = "$want_base" ] && [ "$(cat "$chat/.umarell.sha256" 2>/dev/null)" = "$want_chat" ]; then
  log "umarell $short: up to date"
  exit 0
fi

mkdir -p "$data"
# On the data dir's filesystem, so the trees (and the kept venv) move into place with a rename.
tmp=$(mktemp -d "$data/.umarell-fetch.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

# --- the commit's tree, as <tmp>/src/umarell
get_tree "$commit" "$url" umarell.pin FACTORYOS_UMARELL "$tmp/src"
src="$tmp/src/umarell"
[ -d "$src/agents" ] || die "the umarell tree of $short has no agents/ (not umarell?)"
leftovers() { find "$1" \( -name '*.rej' -o -name '*.orig' \) -print | head -5; }
[ -z "$(leftovers "$src")" ] || die "the umarell tree of $short already has *.rej / *.orig files: $(leftovers "$src" | tr '\n' ' ')"

# --- the plain commit: read-only for the router
if [ "$(cat "$base/.umarell.sha256" 2>/dev/null)" != "$want_base" ]; then
  log "umarell base $short"
  cp -R "$src" "$tmp/base"
  echo "$want_base" > "$tmp/base/.umarell.sha256"
  [ "$as_root" = yes ] && chown -R root:root "$tmp/base"
  chmod -R u=rwX,go=rX "$tmp/base"
  rm -rf "$base" && mv "$tmp/base" "$base"
fi

# --- the chat agent's checkout: the commit with the fixes
if [ "$(cat "$chat/.umarell.sha256" 2>/dev/null)" != "$want_chat" ]; then
  log "umarell $short + fixes (${#series[@]} patches of $(basename "$patches"), ${#public_patches[@]} of deploy/umarell-public)"
  cp -R "$src" "$tmp/chat"
  for p in "${series[@]}"; do
    out=$(patch -p1 --forward --batch -d "$tmp/chat" -i "$patches/$p" 2>&1) \
      || { echo "$out" >&2; die "$p does not apply to $short"; }
    [ -z "$(leftovers "$tmp/chat")" ] || { echo "$out" >&2; die "$p left rejects / backups: $(leftovers "$tmp/chat" | tr '\n' ' ')"; }
    log "applied $p"
  done
  for p in ${public_patches[@]+"${public_patches[@]}"}; do
    out=$(patch -p1 --forward --batch -d "$tmp/chat" -i "$public_dir/$p" 2>&1) \
      || { echo "$out" >&2; die "deploy/umarell-public/$p does not apply to $short + $(basename "$patches")"; }
    [ -z "$(leftovers "$tmp/chat")" ] \
      || { echo "$out" >&2; die "deploy/umarell-public/$p left rejects / backups: $(leftovers "$tmp/chat" | tr '\n' ' ')"; }
    log "applied deploy/umarell-public/$p"
  done
  [ -d "$chat/.venv-planner" ] && mv "$chat/.venv-planner" "$tmp/chat/.venv-planner"
  echo "$want_chat" > "$tmp/chat/.umarell.sha256"
  if [ "$as_root" = yes ] && id factoryos >/dev/null 2>&1; then chown -R factoryos:factoryos "$tmp/chat"; fi
  rm -rf "$chat" && mv "$tmp/chat" "$chat"
fi
log "umarell $short: done"
