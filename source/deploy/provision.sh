#!/bin/bash
# provision.sh <release dir>
#
# Everything the Factory OS server needs besides the release itself, idempotent. factoryos-deploy
# runs it from each release's deploy/ whenever this file or requirements-planner.txt changed since
# the last run, so the server's setup is versioned with the code (no hand steps, no user-data change
# per tweak). Ubuntu 24.04, run as root.
#
#   /opt/factoryos/current            the release (FACTORY_OS_HOME), replaced by every deploy
#   /var/lib/factoryos/               the backend's own data, never touched by a deploy:
#     factoryos.config.json           created with defaults if missing, otherwise left alone
#     cells/                          sample cells, seeded from the release (the planner writes
#                                     agentic_planner/ into them, so existing cells are kept), and
#                                     the starter's (reference) cells, seeded by factoryos-deploy
#     products/                       Product & Task's products, the starter's seeded by factoryos-deploy
#     sample-data/                    the large sample data (assets/sample-data.txt) from S3,
#                                     linked into each release's cells/ by factoryos-deploy
#     umarell/                        the agentic planner (deploy/umarell.txt), + .venv-planner, and
#                                     .kiro/agents/factoryos-planner.json (deploy/kiro: the chat agent),
#                                     .kiro/steering/factoryos-*.md (its steering, deploy/kiro/steering)
#     umarell-base/                   the plain umarell commit (deploy/umarell-base.txt, if pinned): umarell_u3's
#                                     source when umarell/ is that commit with the cell planner's fixes applied
#     umarell-<commit>/               the plain umarell commit umarell_jam is built on (deploy/umarell-jam-base.txt,
#                                     if pinned, public mode deploy/umarell-jam.pin: 76ea13a): the planner router's
#                                     umarell_jam's source
#     cell-planner/                   the planner router's jobs and cell views
#   /opt/factoryos-cell-planner/router/  factoryos-cell-planner (deploy/cell-planner.txt) and its venv, for
#                                     the planner router (factoryos-planner-router.service, 127.0.0.1:8893)
#     catalogue/                      the Robo Dresser's / Fitter's asset root (they write dressed
#                                     robots and scenes into it), seeded by factoryos-deploy from
#     starter/                        the starter assets (deploy/starter.txt) from S3, if pinned,
#                                     else the release's assets/catalogue (assets/, and products/
#                                     and cells/ if the starter has them)
#
# On a workspace host (one per user, see deploy/factoryos-workspace.sh) the home above is the
# workspace: backed up to S3 every 30 min and on stop, restored on a new instance, and the instance
# stops itself when idle. /etc/factoryos/workspace.env (from the instance's tags) says which kind
# of host this is; the backup and idle units are only enabled for a personal workspace.
#
# Public mode (FACTORYOS_PUBLIC=1, from /etc/factoryos/stack.env, which the public deployment's user data writes, or
# the environment; deployment/CONTRACT.md): the release is a public export's, built on the host, and nothing comes
# from a release archive:
#   - umarell from deploy/umarell.pin (deploy/fetch-umarell.sh: a public git commit, or FACTORYOS_UMARELL_TARBALL),
#     its base and its patched chat tree;
#   - umarell_jam's plain umarell from deploy/umarell-jam.pin (fetch-umarell.sh --jam: git, or
#     FACTORYOS_UMARELL_JAM_TARBALL), when the release's cell planner has umarell_jam; not fatal: a failure marks
#     umarell_jam provision-incomplete (as its venv's, provision-cell-planner.sh) and provisioning goes on;
#   - the cell planner from <release>/cell-planner (required);
#   - no starter assets (the catalogue is seeded from the release's) and no sample data archive;
#   - no workspace lifecycle (factoryos-workspace, the cell wiring path unit) unless the release ships it;
#   - /etc/factoryos/workspace.env (if missing): the single workspace "default" whose user administers the
#     catalogue, no remote planning, no shared plan cache; the default config's agenticRegion is the stack's region;
#   - systemd drop-ins: the backend reads /etc/factoryos/kiro.env (the optional Kiro key, written by
#     factoryos-release) and /etc/factoryos/origin.env (the origin secret, FACTORY_OS_ORIGIN_SECRET, written by the
#     release handoff), the planner router workspace.env;
#   - it fails when docker is on PATH (agentic mode would run the planner in Docker instead of the host's venv).
set -euo pipefail
# What it installs for the factoryos user (units, drop-ins, workspace.env, the venvs and trees provision-cell-planner.sh
# builds) must not depend on the caller's umask (the public deployment's SSM document runs under umask 077).
umask 022
release="${1:?usage: provision.sh <release dir>}"
here="$release/deploy"
# The release archive (deploy/*.txt and assets/sample-data.txt keys): /etc/factoryos/artifacts.env in prod, else the
# internal build's default (deploy/internal.env; the environment comes first). The same in provision-cell-planner.sh.
[ -r "$here/internal.env" ] && . "$here/internal.env"
[ -r /etc/factoryos/artifacts.env ] && . /etc/factoryos/artifacts.env
ARCHIVE_BUCKET=${FACTORYOS_BUILDS_BUCKET:-${FACTORYOS_DEFAULT_BUILDS_BUCKET:-}}
ARCHIVE_REGION=${FACTORYOS_BUILDS_REGION:-${FACTORYOS_DEFAULT_BUILDS_REGION:-us-east-1}}
# Public mode and the stack's region (see the header); the same in provision-cell-planner.sh and factoryos-deploy.sh.
public=$( [ -r /etc/factoryos/stack.env ] && . /etc/factoryos/stack.env; echo "${FACTORYOS_PUBLIC:-0}")
stack_region=$( [ -r /etc/factoryos/stack.env ] && . /etc/factoryos/stack.env; echo "${AWS_REGION:-}")
# archive_get <key> <file>: <key> of the release archive into <file>. Without one (no deploy/internal.env, no
# FACTORYOS_BUILDS_BUCKET from /etc/factoryos/artifacts.env or the environment: the public export's scripts) it stops,
# saying so, instead of aws failing on "s3:///<key>".
archive_get() {
  [ -n "$ARCHIVE_BUCKET" ] || { echo "no release archive to download $1 from: deploy/internal.env is missing and" \
    "neither /etc/factoryos/artifacts.env nor the environment sets FACTORYOS_BUILDS_BUCKET" >&2; exit 1; }
  aws s3 cp "s3://$ARCHIVE_BUCKET/$1" "$2" --region "$ARCHIVE_REGION" --only-show-errors
}

data=/var/lib/factoryos

KIRO_VERSION=2.26.1
KIRO_SHA256_AARCH64=a038d2cf09124dbe62a3cfce1398d20c724b9fed758295512cfdaccee5215182

log() { echo "[provision] $*"; }
# mark_incomplete <step>: an optional step that failed, into /etc/factoryos/provision-incomplete (once): factoryos-deploy
# reports it and leaves the provision unstamped, so the next deploy provisions (and retries it) again.
mark_incomplete() {
  mkdir -p /etc/factoryos
  grep -qxF "$1" /etc/factoryos/provision-incomplete 2>/dev/null || echo "$1" >> /etc/factoryos/provision-incomplete
}
as_factoryos() { sudo -u factoryos -H "$@"; }

export DEBIAN_FRONTEND=noninteractive
# apt-get <args>, waiting for the apt/dpkg locks: at boot, SSM Patch Manager (AWS-RunPatchBaseline's
# `python3 main_entrance.py --file snapshot.json`), unattended-upgrades or cloud-init can hold them for minutes,
# and apt-get fails at once on them ("Unable to lock directory /var/lib/apt/lists/", "Could not get lock").
# Waits up to FACTORYOS_APT_LOCK_WAIT seconds (default 900); any other failure fails as before.
apt_get() {
  local out rc deadline=$(( $(date +%s) + ${FACTORYOS_APT_LOCK_WAIT:-900} ))
  while :; do
    rc=0
    out=$(apt-get -o DPkg::Lock::Timeout=60 "$@" 2>&1) || rc=$?
    if [ "$rc" = 0 ]; then
      [ -z "$out" ] || printf '%s\n' "$out"
      return 0
    fi
    if grep -qE 'Could not get lock|Unable to lock|Unable to acquire the dpkg frontend lock' <<<"$out" \
       && [ "$(date +%s)" -lt "$deadline" ]; then
      log "apt is locked ($(pgrep -af 'apt|dpkg|unattended-upgr|main_entrance' 2>/dev/null | head -3 | tr '\n' ';' || true)); waiting"
      sleep 10
      continue
    fi
    printf '%s\n' "$out" >&2
    return "$rc"
  done
}
if [ "$public" = 1 ] && command -v docker >/dev/null 2>&1; then
  echo "provision.sh: docker is on PATH ($(command -v docker)): the backend's agentic mode would run the planner in" \
       "Docker instead of this host's venv; the public deployment's host must not have it" >&2
  exit 1
fi

log "packages"
apt_get update -y -qq
# patch, rsync: the planner router's umarell_u3 (factoryos-cell-planner umarell_fast/apply.py); public mode: git for
# deploy/fetch-umarell.sh, a C compiler and Python.h for umarell_jam's toppra (built from its sdist,
# provision-cell-planner.sh: deploy/toppra.pin).
public_packages=""; [ "$public" = 1 ] && public_packages="git gcc libc6-dev python3.12-dev"
# shellcheck disable=SC2086 # public_packages: none, or more words
apt_get install -y -qq curl unzip make xz-utils zstd python3.12-venv python3-pip patch rsync $public_packages >/dev/null

id factoryos >/dev/null 2>&1 \
  || useradd --system --home-dir "$data" --create-home --shell /usr/sbin/nologin factoryos
# products/ and catalogue/ too: a public install has no starter to seed products/ (the first upload failed without it).
install -d -o factoryos -g factoryos "$data" "$data/cells" "$data/products" "$data/catalogue"

# --- public mode: the single workspace's settings (read by the backend's, the planner router's and provision's own
# steps below); an existing file is left alone.
if [ "$public" = 1 ] && [ ! -f /etc/factoryos/workspace.env ]; then
  log "workspace.env (public: workspace default)"
  mkdir -p /etc/factoryos
  cat > /etc/factoryos/workspace.env <<'ENV'
FACTORY_OS_WORKSPACE=default
FACTORY_OS_CATALOGUE_ADMINS=default
FACTORYOS_BURST=off
FACTORY_OS_U3_CACHE_S3=0
FACTORY_OS_JAM_CACHE_S3=0
ENV
fi

# --- kiro-cli: the agent behind Agentic Planning's chat (all of its binaries: kiro-cli hands chat
# off to kiro-cli-chat). Signing in stays manual: kiro-cli login --use-device-flow, as factoryos.
if [ "$(kiro-cli --version 2>/dev/null | awk '{print $2}')" != "$KIRO_VERSION" ]; then
  case "$(uname -m)" in
    aarch64) sha="$KIRO_SHA256_AARCH64"; arch=aarch64 ;;
    *) echo "kiro-cli: no pinned build for $(uname -m)" >&2; exit 1 ;;
  esac
  log "kiro-cli $KIRO_VERSION"
  tmp=$(mktemp -d)
  curl -fsSL -o "$tmp/kiro.tar.gz" \
    "https://desktop-release.q.us-east-1.amazonaws.com/$KIRO_VERSION/kirocli-$arch-linux.tar.gz"
  echo "$sha  $tmp/kiro.tar.gz" | sha256sum -c - >/dev/null
  tar -xzf "$tmp/kiro.tar.gz" -C "$tmp"
  for bin in "$tmp"/kirocli/bin/*; do install -m 0755 "$bin" /usr/local/bin/; done
  rm -rf "$tmp"
fi

# --- umarell at the commit in deploy/umarell.txt, from the CI archive (the GitHub repo is private).
# Its upstream cells are not used: cellsDir is $data/cells. The venv survives an update.
umarell="$data/umarell"
# umarell_jam's plain umarell commit (<umarell>-<7 hex>), kept by the cleanup below; set by the step that installs it.
jbase=""
if [ "$public" = 1 ]; then
  # Public mode: deploy/umarell.pin's commit (its base and the chat tree with the cell planner's fixes and
  # deploy/umarell-public's patches) from git, or FACTORYOS_UMARELL_TARBALL (+ _SHA256) from the environment
  # (the installer's --umarell-tarball). Fatal: without umarell the agentic planner has nothing to run. While
  # sample-umarell is not public its git fetch fails for everyone; that failure gets its own message.
  umarell_err=$(mktemp)
  if ! "$here/fetch-umarell.sh" "$release" "$data" 2> "$umarell_err"; then
    cat "$umarell_err" >&2
    if [ -z "${FACTORYOS_UMARELL_TARBALL:-}" ] && grep -q '^fetch-umarell: cannot fetch ' "$umarell_err"; then
      read -r ucommit uurl _ < <(grep -vE '^\s*(#|$)' "$here/umarell.pin" | head -1) || true
      echo "provision.sh: umarell cannot be installed: the pinned sample-umarell commit ${ucommit:-?} is not" \
           "publicly fetchable from ${uurl:-?} (the repository is not public yet). Install with a tarball of it" \
           "instead: deployment/install.sh install ... --umarell-tarball s3://<bucket>/<key>.tar.gz (a" \
           "git archive --prefix=umarell/ of that commit)." >&2
    else
      echo "provision.sh: umarell could not be installed (deploy/fetch-umarell.sh failed, see above)" >&2
    fi
    rm -f "$umarell_err"
    exit 1
  fi
  cat "$umarell_err" >&2
  rm -f "$umarell_err"

  # umarell_jam's plain umarell (deploy/umarell-jam.pin, 76ea13a), when the release's cell planner has umarell_jam and
  # the host runs it (FACTORY_OS_CELL_PLANNER, FACTORY_OS_UMARELL_JAM in workspace.env): deploy/fetch-umarell.sh --jam,
  # from git or FACTORYOS_UMARELL_JAM_TARBALL (+ _SHA256; the installer's --umarell-jam-tarball). Not fatal: without it
  # umarell_jam does not set up and its jobs fall back to umarell_u3; provision-incomplete says so, the next deploy
  # retries. Its directory is kept by the cleanup even then (a tree installed before stays usable).
  jam_pin="$here/umarell-jam.pin"
  jam_on=$(. /etc/factoryos/workspace.env 2>/dev/null
           [ "${FACTORY_OS_CELL_PLANNER:-1}" != 0 ] && [ "${FACTORY_OS_UMARELL_JAM:-1}" != 0 ] && echo yes || echo no)
  if [ -f "$jam_pin" ] && [ -d "$release/cell-planner/umarell_jam" ] && [ "$jam_on" = yes ]; then
    read -r jcommit jurl _ < <(grep -vE '^\s*(#|$)' "$jam_pin" | head -1) || true
    [[ "${jcommit:-}" =~ ^[0-9a-f]{40}$ ]] && jbase="$data/umarell-${jcommit:0:7}"
    jam_err=$(mktemp)
    if "$here/fetch-umarell.sh" "$release" "$data" --jam 2> "$jam_err"; then
      cat "$jam_err" >&2
    else
      cat "$jam_err" >&2
      if [ -z "${FACTORYOS_UMARELL_JAM_TARBALL:-}" ] && grep -q '^fetch-umarell: cannot fetch ' "$jam_err"; then
        log "WARNING: umarell_jam's umarell cannot be installed: the pinned sample-umarell commit ${jcommit:-?} is not" \
            "publicly fetchable from ${jurl:-?} (the repository is not public yet). Install with a tarball of it:" \
            "deployment/install.sh install ... --umarell-jam-tarball s3://<bucket>/<key>.tar.gz (a" \
            "git archive --prefix=umarell/ of that commit)."
      else
        log "WARNING: umarell_jam's umarell could not be installed (deploy/fetch-umarell.sh --jam failed, see above)"
      fi
      log "WARNING: umarell_jam is not set up (provision-incomplete); its jobs fall back to umarell_u3, the next deploy retries it"
      mark_incomplete umarell_jam
    fi
    rm -f "$jam_err"
  fi
else
  read -r commit sha key < <(grep -vE '^\s*(#|$)' "$here/umarell.txt" | head -1)
  if [ "$(cat "$umarell/.umarell.sha256" 2>/dev/null)" != "$sha" ]; then
    log "umarell $commit"
    tmp=$(mktemp -d)
    archive_get "$key" "$tmp/umarell.tar.gz"
    echo "$sha  $tmp/umarell.tar.gz" | sha256sum -c - >/dev/null
    tar -xzf "$tmp/umarell.tar.gz" -C "$tmp"
    [ -d "$umarell/.venv-planner" ] && mv "$umarell/.venv-planner" "$tmp/umarell/.venv-planner"
    rm -rf "$umarell" && mv "$tmp/umarell" "$umarell"
    echo "$sha" > "$umarell/.umarell.sha256"
    chown -R factoryos:factoryos "$umarell"
    rm -rf "$tmp"
  fi

  # --- the plain umarell commit (deploy/umarell-base.txt), if pinned: deploy/umarell.txt is then that commit
  # with the cell planner's upstream fixes applied, and the planner router's umarell_u3 (which applies the fixes
  # itself) takes its source from here (<umarell>-base, found by umarell_u3.py). Read-only for the router.
  base="$data/umarell-base"
  if [ -f "$here/umarell-base.txt" ] && read -r bcommit bsha bkey < <(grep -vE '^\s*(#|$)' "$here/umarell-base.txt" | head -1) \
     && [ -n "${bkey:-}" ]; then
    if [ "$(cat "$base/.umarell.sha256" 2>/dev/null)" != "$bsha" ]; then
      log "umarell base $bcommit"
      tmp=$(mktemp -d)
      archive_get "$bkey" "$tmp/umarell.tar.gz"
      echo "$bsha  $tmp/umarell.tar.gz" | sha256sum -c - >/dev/null
      tar -xzf "$tmp/umarell.tar.gz" -C "$tmp"
      echo "$bsha" > "$tmp/umarell/.umarell.sha256"
      chown -R root:root "$tmp/umarell"
      chmod -R u=rwX,go=rX "$tmp/umarell"
      rm -rf "$base" && mv "$tmp/umarell" "$base"
      rm -rf "$tmp"
    fi
  else
    rm -rf "$base"
  fi
fi

# --- the plain umarell 76ea13a (deploy/umarell-jam-base.txt), if pinned: the planner router's umarell_jam (umarell
# 76ea13a + the cell planner's fix set v8, which it applies to its own copy) takes its source from here
# (<umarell>-<commit>, found by umarell_jam.py). Read-only for the router. Not in public mode (its archive is
# internal): the public branch above installs it from deploy/umarell-jam.pin.
if [ "$public" != 1 ] && [ -f "$here/umarell-jam-base.txt" ] && read -r jcommit jsha jkey < <(grep -vE '^\s*(#|$)' "$here/umarell-jam-base.txt" | head -1) \
   && [ -n "${jkey:-}" ]; then
  [[ "$jcommit" =~ ^[0-9a-f]{7}$ ]] || { echo "deploy/umarell-jam-base.txt: '$jcommit' is not a 7-digit commit" >&2; exit 1; }
  jbase="$data/umarell-$jcommit"
  if [ "$(cat "$jbase/.umarell.sha256" 2>/dev/null)" != "$jsha" ]; then
    log "umarell $jcommit for umarell_jam"
    tmp=$(mktemp -d)
    archive_get "$jkey" "$tmp/umarell.tar.gz"
    echo "$jsha  $tmp/umarell.tar.gz" | sha256sum -c - >/dev/null
    tar -xzf "$tmp/umarell.tar.gz" -C "$tmp"
    echo "$jsha" > "$tmp/umarell/.umarell.sha256"
    chown -R root:root "$tmp/umarell"
    chmod -R u=rwX,go=rX "$tmp/umarell"
    rm -rf "$jbase" && mv "$tmp/umarell" "$jbase"
    rm -rf "$tmp"
  fi
fi
# an umarell_jam base no longer pinned (another commit, or none): removed
for d in "$data"/umarell-[0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f]; do
  if [ -d "$d" ] && [ "$d" != "$jbase" ]; then rm -rf "$d"; fi
done

# --- the planner's venv, rebuilt when its requirements change. Prebuilt wheels only: a missing
# one should fail here, not compile for an hour on the server.
venv="$umarell/.venv-planner"
want=$(sha256sum "$here/requirements-planner.txt" | cut -d' ' -f1)
if [ "$(cat "$venv/.requirements.sha256" 2>/dev/null)" != "$want" ]; then
  log "planner venv"
  rm -rf "$venv"
  as_factoryos python3 -m venv "$venv"
  as_factoryos "$venv/bin/pip" install --quiet --upgrade pip
  install -m 0644 "$here/requirements-planner.txt" /tmp/requirements-planner.txt
  as_factoryos "$venv/bin/pip" install --quiet --only-binary=:all: -r /tmp/requirements-planner.txt
  rm -f /tmp/requirements-planner.txt
  as_factoryos env PYTHONPATH="$umarell" "$venv/bin/python" -c '
import pinocchio as pin, ompl, langgraph, mcp  # noqa: F401
assert hasattr(pin.Frame(), "parentJoint"), "Pinocchio 3 needed"
import agents.trajectory.tools.pinocchio, agents.trajectory.tools.ompl_planner  # noqa: F401
'
  as_factoryos sh -c "echo $want > '$venv/.requirements.sha256'"
fi

# --- the chat agent: the factoryos-planner agent (plans through the planner router), into the umarell
# checkout, where kiro-cli runs (services/chat.rs passes --agent factoryos-planner when it is there).
# On every run: extracting a new umarell replaces the checkout.
install -d -o factoryos -g factoryos "$umarell/.kiro" "$umarell/.kiro/agents" "$umarell/.kiro/steering"
install -m 0644 -o factoryos -g factoryos "$here/kiro/factoryos-planner.json" "$umarell/.kiro/agents/factoryos-planner.json"
# Its steering (the agent's resources: one file per workflow step); stale factoryos-*.md of an older release go.
find "$umarell/.kiro/steering" -maxdepth 1 -name 'factoryos-*.md' -delete
for f in "$here"/kiro/steering/factoryos-*.md; do
  [ -f "$f" ] && install -m 0644 -o factoryos -g factoryos "$f" "$umarell/.kiro/steering/"
done
# Its skills (deploy/kiro/skills/<name>/SKILL.md, loaded on demand): for the chat agent (the umarell checkout) and for
# kiro-cli started by the user in the workspace IDE's terminals (factoryos's ~/.kiro/skills).
fos_home=$(getent passwd factoryos | cut -d: -f6)
for dest in "$umarell/.kiro/skills" ${fos_home:+"$fos_home/.kiro/skills"}; do
  install -d -o factoryos -g factoryos "$dest"
  for skill in "$here"/kiro/skills/*/; do
    [ -f "$skill/SKILL.md" ] || continue
    name=$(basename "$skill")
    install -d -o factoryos -g factoryos "$dest/$name"
    install -m 0644 -o factoryos -g factoryos "$skill/SKILL.md" "$dest/$name/SKILL.md"
  done
done
# kiro-cli validates only when it is logged in: without a login (a public install without a Kiro key, which the
# chat's KIRO_API_KEY replaces) that is expected, and said as info; a real validation error is one line, as a warning.
agent_check=$(as_factoryos timeout 30 kiro-cli agent validate --path "$umarell/.kiro/agents/factoryos-planner.json" 2>&1 || true)
case "$agent_check" in
  *"not logged in"*) log "kiro agent file: not validated (kiro-cli has no login here; expected without one)" ;;
  *rror*) log "kiro agent file: WARNING: does not validate: $(printf '%s' "$agent_check" | tr '\n' ' ' | cut -c1-300)" ;;
esac

# --- factoryos-cell-planner for the planner router (its own script: a failure there leaves the router
# with the umarell planner only, and does not stop the deploy). FACTORY_OS_CELL_PLANNER=0 in
# /etc/factoryos/workspace.env skips it on a host (~2.5 GB with its venv).
cell_planner=$(. /etc/factoryos/workspace.env 2>/dev/null; echo "${FACTORY_OS_CELL_PLANNER:-1}")
# Public mode: from the release's cell-planner/ (scripts/package-bundle.sh puts the export's there); a release without
# one cannot provision.
if [ "$public" = 1 ] && [ "$cell_planner" != 0 ] && [ ! -d "$release/cell-planner" ]; then
  echo "provision.sh: $release/cell-planner is missing (public mode installs the cell planner from the release)" >&2
  exit 1
fi
if { [ -f "$here/cell-planner.txt" ] || [ "$public" = 1 ]; } && [ "$cell_planner" != 0 ]; then
  # On failure: the marker tells factoryos-deploy not to record the provision stamp, so the next deploy
  # provisions (and retries this) again even with the same pins.
  "$here/provision-cell-planner.sh" "$release" || {
    log "WARNING: cell planner setup failed (see above); the planner router offers the umarell planner only;" \
        "the next deploy retries it"
    mark_incomplete cell-planner
  }
fi

# --- starter assets (sample-factory-os-starter), content-addressed in the CI archive, if pinned in
# deploy/starter.txt: factoryos-deploy seeds the catalogue from them instead of the release's copy.
starter="$data/starter"
# (Public mode: none, the export has no deploy/starter.txt.)
if [ "$public" != 1 ] && read -r id sha key < <(grep -vE '^\s*(#|$)' "$here/starter.txt" 2>/dev/null | head -1) \
   && [ -n "${key:-}" ]; then
  if [ "$(cat "$starter/.starter.sha256" 2>/dev/null)" != "$sha" ]; then
    log "starter assets $id"
    tmp=$(mktemp -d)
    archive_get "$key" "$tmp/starter.tar.gz"
    echo "$sha  $tmp/starter.tar.gz" | sha256sum -c - >/dev/null
    tar -xzf "$tmp/starter.tar.gz" -C "$tmp"
    rm -rf "$starter" && mv "$tmp/starter" "$starter"
    echo "$sha" > "$starter/.starter.sha256"
    chown -R factoryos:factoryos "$starter"
    rm -rf "$tmp"
  fi
fi

# --- large sample data (assets/sample-data.txt), content-addressed in the CI archive; a copy at
# the listed sha256 is kept, so this downloads only what is new.
sample="$data/sample-data"
install -d -o factoryos -g factoryos "$sample"
# Public mode: the export's manifest lists none (comment lines only); without one there is nothing to fetch either.
sample_list="$release/assets/sample-data.txt"
[ "$public" = 1 ] && [ ! -f "$sample_list" ] && sample_list=/dev/null
while read -r dir sha key; do
  case "$dir" in ''|'#'*) continue ;; esac
  [ "$(cat "$sample/$dir/.sample-data.sha256" 2>/dev/null)" = "$sha" ] && continue
  log "sample data $dir"
  tmp=$(mktemp -d)
  archive_get "$key" "$tmp/data.tar.gz"
  echo "$sha  $tmp/data.tar.gz" | sha256sum -c - >/dev/null
  rm -rf "${sample:?}/${dir:?}"
  tar -xzf "$tmp/data.tar.gz" -C "$sample"
  echo "$sha" > "$sample/$dir/.sample-data.sha256"
  chown -R factoryos:factoryos "$sample/$dir"
  chmod -R u=rwX,go=rX "$sample/$dir"
  rm -rf "$tmp"
done < "$sample_list"

# --- sample cells: new ones from the release; existing ones are the planner's working copies.
for cell in "$release"/cells/*/; do
  [ -d "$cell" ] || continue
  name=$(basename "$cell")
  if [ ! -e "$data/cells/$name" ]; then
    log "cell $name"
    cp -R "$cell" "$data/cells/$name"
    chown -R factoryos:factoryos "$data/cells/$name"
  fi
done

# --- config: defaults for a new server only; an existing file is the user's.
config="$data/factoryos.config.json"
# The agentic planner's Bedrock region: public mode the stack's (/etc/factoryos/stack.env).
agentic_region=us-east-1
[ "$public" = 1 ] && [ -n "$stack_region" ] && agentic_region=$stack_region
if [ ! -f "$config" ]; then
  log "default config"
  cat > "$config" <<JSON
{
  "catalogue": "$data/catalogue",
  "assetRoot": "$data/catalogue",
  "cellsDir": "$data/cells",
  "cellDir": "$data/cells/welding_cell_2_small",
  "envDir": "$data/cells/welding_cell_2_small",
  "scene": "scene.factoryos.yaml",
  "agenticRepoDir": "$umarell",
  "agenticRegion": "$agentic_region",
  "agenticRvizEnabled": false
}
JSON
  chown factoryos:factoryos "$config"
fi

# --- the service, and a workspace host's lifecycle units.
# (Public mode: only if the release ships it; the public export does not.)
if [ "$public" != 1 ] || [ -f "$here/factoryos-workspace.sh" ]; then
  install -m 0755 "$here/factoryos-workspace.sh" /usr/local/bin/factoryos-workspace
fi
# fos-migrate-cells: the backend's cell migration (legacy cells to reference cells), from the release the
# server runs (the backend dispatches on its name).
ln -sfn /opt/factoryos/current/bin/factoryos-backend /usr/local/bin/fos-migrate-cells
# factoryos-deploy itself (user data installed the first one): its provision trigger follows the release.
# install(1) writes a new file, so the running deploy keeps reading its own copy.
cmp -s "$here/factoryos-deploy.sh" /usr/local/bin/factoryos-deploy \
  || install -m 0755 "$here/factoryos-deploy.sh" /usr/local/bin/factoryos-deploy
# nullglob: the public export ships services only (no timers, no path units).
shopt -s nullglob
for file in "$here"/systemd/*.service "$here"/systemd/*.timer "$here"/systemd/*.path; do
  unit=/etc/systemd/system/$(basename "$file")
  if ! cmp -s "$file" "$unit"; then
    log "systemd unit $(basename "$file")"
    install -m 0644 "$file" "$unit"
  fi
done
shopt -u nullglob
# cell-rerun-server ships in the release (bin/). The drop-in of the retired Kiro-bundle fallback set
# FACTORY_OS_KIRO_BUNDLE, which the backend no longer reads; removed where an older release left it.
rm -f /etc/systemd/system/factoryos-backend.service.d/kiro-bundle.conf
rmdir /etc/systemd/system/factoryos-backend.service.d 2>/dev/null || true
# Public mode: the backend reads the optional Kiro key (factoryos-release writes /etc/factoryos/kiro.env, 0600, when
# the stack names a secret), the planner router this host's workspace.env (FACTORYOS_BURST=off,
# FACTORY_OS_U3_CACHE_S3=0). The terminals leave KIRO_API_KEY out of a shell's environment, but that hides the key
# from no user: every signed-in user may open a terminal (a shell as factoryos, with the instance role through IMDS,
# which may read the secret). deployment/CONTRACT.md (kiroSecretName) and the Guidance README's security section.
if [ "$public" = 1 ]; then
  install -d -m 0755 /etc/systemd/system/factoryos-backend.service.d /etc/systemd/system/factoryos-planner-router.service.d
  # origin.env: the origin secret the CloudFront gate sends as x-origin-verify (0600 root, written by
  # deployment/bootstrap/release-handoff.sh); with it the backend refuses peers other than loopback without that header.
  printf '[Service]\nEnvironmentFile=-/etc/factoryos/kiro.env\nEnvironmentFile=-/etc/factoryos/origin.env\n' \
    > /etc/systemd/system/factoryos-backend.service.d/public.conf
  printf '[Service]\nEnvironmentFile=-/etc/factoryos/workspace.env\n' \
    > /etc/systemd/system/factoryos-planner-router.service.d/public.conf
fi
systemctl daemon-reload
systemctl enable --quiet factoryos-backend.service
# A workspace's own virtual cell: each launch's wiring restarts the live bridge on it (idle without one).
# (Public mode: only if the release ships the unit.)
if [ "$public" != 1 ] || [ -f "$here/systemd/factoryos-cell-wire.path" ]; then
  systemctl enable --now --quiet factoryos-cell-wire.path
fi
# The planner router (deploy/planner-router; PartOf the backend, so every deploy's backend restart
# restarts it on the new release too).
install -d -o factoryos -g factoryos "$data/cell-planner" "$data/cell-planner/work"
systemctl enable --quiet factoryos-planner-router.service
systemctl restart factoryos-planner-router.service
kind=$(. /etc/factoryos/workspace.env 2>/dev/null; echo "${FACTORY_OS_WORKSPACE_KIND:-}")
workspace_units=(factoryos-update.service factoryos-backup.timer factoryos-backup-on-stop.service factoryos-idle.timer)
if [ "$kind" = personal ]; then
  log "personal workspace: backups, idle stop, update at boot"
  systemctl enable --quiet "${workspace_units[@]}"
  systemctl start --quiet factoryos-backup.timer factoryos-backup-on-stop.service factoryos-idle.timer
else
  # The CI / shared server: deployed by the pipeline, never stopped or backed up by itself.
  systemctl disable --now --quiet "${workspace_units[@]}" 2>/dev/null || true
fi
log "done"
