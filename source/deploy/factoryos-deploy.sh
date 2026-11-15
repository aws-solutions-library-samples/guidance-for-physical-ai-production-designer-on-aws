#!/bin/bash
# factoryos-deploy <s3://…/factoryos-viewer-linux-arm64-<id>.tar.gz>
# factoryos-deploy </…/factoryos-viewer-linux-arm64-<id>.tar.gz> <sha256>
#
# Installs a Factory OS release from the CI archive on this server and restarts the backend, which
# serves the wasm viewer, its API and Zenoh (/zenoh) on port 80. Releases that carry deploy/ also
# bring the server's setup: deploy/provision.sh runs first whenever it (or what it installs: the
# planners' pinned requirements and archives, the systemd units, the chat agent file, the archive
# defaults of deploy/internal.env) changed since the last run. The planner router (deploy/planner-router)
# needs no provisioning: it restarts with the backend.
# Run by the pipeline over SSM (deploy:server), and by user data on a new server.
# A local bundle (an absolute path; the public deployment's factoryos-release builds and packages it on the host,
# deployment/CONTRACT.md) comes with its sha256, checked before anything is changed. Public mode (FACTORYOS_PUBLIC=1,
# from /etc/factoryos/stack.env or the environment) adds what public provisioning installs from the release to the
# provision trigger: deploy/fetch-umarell.sh, deploy/umarell.pin, the cell-planner/ tree, the umarell tarball's sha256.
# The release must be runnable by the backend's user (factoryos) whatever umask this runs under: /opt/factoryos and
# releases/ 0755, the release's directories 0755 and bin/* 0755, then checked (release_problems) before the switch.
set -euo pipefail
umask 022

url="${1:?usage: factoryos-deploy <s3 url of a factoryos-viewer-linux-arm64 bundle> | <local bundle path> <sha256>}"
case "$url" in
  /*)
    want_sha="${2:?usage: factoryos-deploy <local bundle path> <sha256>}"
    [ -f "$url" ] || { echo "factoryos-deploy: $url: no such file" >&2; exit 1; }
    got_sha=$(sha256sum "$url" | cut -d' ' -f1)
    [ "$got_sha" = "$want_sha" ] || { echo "factoryos-deploy: $url: sha256 $got_sha, expected $want_sha" >&2; exit 1; }
    ;;
esac
name="$(basename "$url" .tar.gz)"
releases=/opt/factoryos/releases
dir="$releases/$name"

tmp="$(mktemp -d)"
# Once /opt/factoryos/current points at the new release, the backend (and the planner router, PartOf it)
# must be restarted on it whatever happens next: an exit before the restart below (set -e) restarts it here.
switched=no restarted=no
on_exit() {
  rm -rf "$tmp"
  if [ "$switched" = yes ] && [ "$restarted" = no ]; then
    echo "factoryos-deploy: stopped before the restart; restarting factoryos-backend on $name" >&2
    systemctl restart factoryos-backend || true
  fi
}
trap on_exit EXIT
# The archive: /etc/factoryos/artifacts.env (prod), else the dev archive in us-east-1.
builds_region=$( [ -r /etc/factoryos/artifacts.env ] && . /etc/factoryos/artifacts.env; echo "${FACTORYOS_BUILDS_REGION:-us-east-1}")
bundle="$url"
case "$url" in
  /*) ;;
  *) aws s3 cp "$url" "$tmp/bundle.tar.gz" --region "$builds_region" --only-show-errors; bundle="$tmp/bundle.tar.gz" ;;
esac
# release_problems <release dir>: what keeps the backend (User=factoryos) from running the release, one line each;
# nothing when it can. By the modes: every directory from / to the release searchable by others, the release and its
# bin/ also readable, every bin/* a file others can read and execute. As root with the factoryos user present, also
# by access(2) as that user (which sees ACLs and a noexec mount too). A 0700 /opt/factoryos (the public handoff's umask
# 077) made the backend fail with 203/EXEC "Permission denied" and the deploy with "did not become healthy" only.
release_problems() {
  local rel=$1 d f
  d=$rel
  while :; do
    [ -n "$(find -L "$d" -maxdepth 0 -perm -0001 2>/dev/null)" ] \
      || echo "$d ($(ls -ld "$d" 2>&1 | awk '{print $1, $3 ":" $4}')): others cannot enter it"
    [ "$d" = / ] && break
    d=$(dirname "$d")
  done
  for d in "$rel" "$rel/bin"; do
    [ -e "$d" ] || continue
    [ -n "$(find -L "$d" -maxdepth 0 -perm -0005 2>/dev/null)" ] \
      || echo "$d ($(ls -ld "$d" 2>&1 | awk '{print $1, $3 ":" $4}')): others cannot list it"
  done
  # bin/* and the flat layout's factoryos-backend (the bin/ layout's is a link to bin/factoryos-backend).
  for f in "$rel"/bin/* "$rel/factoryos-backend"; do
    if [ ! -e "$f" ] && [ ! -L "$f" ]; then continue; fi
    if [ ! -f "$f" ] || [ -z "$(find -L "$f" -maxdepth 0 -perm -0005 2>/dev/null)" ]; then
      echo "$f ($(ls -lLd "$f" 2>&1 | awk '{print $1, $3 ":" $4}')): others cannot execute it"
    elif [ "$(id -u)" = 0 ] && id factoryos >/dev/null 2>&1 && command -v runuser >/dev/null 2>&1 \
         && ! runuser -u factoryos -- test -x "$f"; then
      echo "$f: the factoryos user cannot execute it (access(2): an ACL, a noexec mount)"
    fi
  done
}
# /opt/factoryos and releases/: 0755 whatever created them (repairs a 0700 one).
mkdir -p "$releases" && chmod 0755 "$(dirname "$releases")" "$releases"
rm -rf "$dir.new" && mkdir -p "$dir.new"
tar -xzf "$bundle" -C "$dir.new"
# The backend runs unprivileged, so everything it needs must be traversable/readable by it. The
# bundle's own modes cannot be relied on: the tarball's top-level entry carries the mode of the
# mktemp -d the CI staged it in (0700), which would make the whole release root-only.
chmod -R u=rwX,go=rX "$dir.new"
# bin/ holds executables only: 0755 even when the bundle lost their modes (a tar or zip without them).
for f in "$dir.new"/bin/*; do
  if [ -f "$f" ] && [ ! -L "$f" ]; then chmod 0755 "$f"; fi
done
# Two bundle layouts: bin/factoryos-backend (+ bin/cell-rerun-server, cells/, assets/; the
# FACTORY_OS_HOME layout), and the earlier flat one with factoryos-backend next to web/.
if [ -x "$dir.new/bin/factoryos-backend" ]; then
  # Keep the flat path working too: the systemd unit of servers set up before the bin/ layout
  # runs <release>/factoryos-backend (the backend finds its home from its real path, bin/..).
  ln -sfn bin/factoryos-backend "$dir.new/factoryos-backend"
fi
[ -x "$dir.new/factoryos-backend" ] && [ -f "$dir.new/web/index.html" ] \
  || { echo "$url is not a factoryos-viewer bundle (factoryos-backend + web/)" >&2; exit 1; }
# Before the switch (and before a same-named release is replaced): the release before keeps serving.
problems=$(release_problems "$dir.new")
if [ -n "$problems" ]; then
  # One line (factoryos-release's state error is the last error line a script wrote).
  echo "factoryos-deploy: error: the factoryos user cannot run $name: $(printf '%s\n' "$problems" | sed 's/$/;/' \
    | tr '\n' ' ' | sed 's/; *$//')" >&2
  rm -rf "$dir.new"
  exit 1
fi
rm -rf "$dir" && mv "$dir.new" "$dir"
# Until this deploy finished (backend up on it, provisioned): factoryos-workspace update deploys again
# when it finds it, even though /opt/factoryos/current is the release already.
mkdir -p /etc/factoryos
pending=/etc/factoryos/deploy-pending
echo "$name" > "$pending"
ln -sfn "$dir" /opt/factoryos/current
switched=yes
# The release the backend reports (the CI build id: p<pipeline> or the tag), and which workspace
# this host serves (from its tags, for a workspace host or the shared CI server).
echo "FACTORY_OS_RELEASE=${name#factoryos-viewer-linux-arm64-}" > /etc/factoryos/release.env
# fos-migrate-cells is the backend under another name (provision.sh links it too).
[ -x "$dir/bin/factoryos-backend" ] && ln -sfn /opt/factoryos/current/bin/factoryos-backend /usr/local/bin/fos-migrate-cells
if [ -f "$dir/deploy/factoryos-workspace.sh" ]; then
  workspace_changed=no
  cmp -s "$dir/deploy/factoryos-workspace.sh" /usr/local/bin/factoryos-workspace 2>/dev/null || workspace_changed=yes
  install -m 0755 "$dir/deploy/factoryos-workspace.sh" /usr/local/bin/factoryos-workspace
  /usr/local/bin/factoryos-workspace configure || true
  # A `factoryos-workspace update` of an older release runs its own connect-cell after this deploy (the function it
  # loaded): once it is done, the new one's, so a workspace connected to a cell ends on this release's live bridge.
  # Idempotent; only a workspace host (workspace.env), only when the script changed.
  if [ "$workspace_changed" = yes ] && [ -f /etc/factoryos/workspace.env ] && command -v systemd-run >/dev/null 2>&1; then
    systemd-run --quiet --collect --on-active=180 --unit="factoryos-connect-cell-after-deploy-$$" \
      /usr/local/bin/factoryos-workspace connect-cell 2>/dev/null || true
  fi
fi

provision_failed=no
if [ -x "$dir/deploy/provision.sh" ]; then
  # Per machine, not in the backed-up home: a new instance restoring a workspace must still provision.
  stamp=/etc/factoryos/provisioned.sha256
  rm -f /var/lib/factoryos/.provisioned.sha256
  # Some inputs are optional (a CI server has no workspace.env): `|| true`, or pipefail ends the deploy.
  # Public mode only (internal hosts keep their stamps): what public provisioning installs from the release (umarell,
  # umarell_jam's plain umarell, either from a tarball, and the cell planner's tree).
  public=$( [ -r /etc/factoryos/stack.env ] && . /etc/factoryos/stack.env; echo "${FACTORYOS_PUBLIC:-0}")
  public_inputs=""
  if [ "$public" = 1 ]; then
    public_inputs=$( (cat "$dir/deploy/fetch-umarell.sh" "$dir/deploy/umarell.pin" "$dir/deploy/umarell-jam.pin" 2>/dev/null || true
                      echo "umarell tarball ${FACTORYOS_UMARELL_SHA256:-none}"
                      echo "umarell_jam tarball ${FACTORYOS_UMARELL_JAM_SHA256:-none}"
                      if [ -d "$dir/cell-planner" ]; then
                        (cd "$dir/cell-planner" && find . -type f -print | LC_ALL=C sort |
                          while IFS= read -r f; do printf '%s  %s\n' "$(sha256sum < "$f" | cut -d' ' -f1)" "$f"; done)
                      fi) | sha256sum | cut -d' ' -f1)
  fi
  want=$( (cat "$dir/deploy/provision.sh" "$dir/deploy/requirements-planner.txt" \
           "$dir/deploy/systemd/"* "$dir/deploy/umarell.txt" "$dir/deploy/umarell-base.txt" "$dir/deploy/umarell-jam-base.txt" "$dir/deploy/starter.txt" \
           "$dir/deploy/provision-cell-planner.sh" "$dir/deploy/cell-planner.txt" \
           "$dir/deploy/requirements-cell-planner.txt" "$dir/deploy/toppra-wheel.txt" "$dir/deploy/toppra.pin" "$dir/deploy/kiro/"* "$dir/deploy/internal.env" \
           "$dir/deploy/factoryos-workspace.sh" /etc/factoryos/workspace.env \
           "$dir/assets/sample-data.txt" 2>/dev/null || true; printf '%s' "$public_inputs") |
           sha256sum | cut -d' ' -f1)
  if [ "$(cat "$stamp" 2>/dev/null)" != "$want" ] || [ ! -f /etc/systemd/system/factoryos-backend.service ]; then
    # provision.sh lists optional steps that failed (and should be retried) in provision-incomplete.
    rm -f /etc/factoryos/provision-incomplete
    # A failure (exit status) still restarts the backend on the new release below, then fails the deploy;
    # unstamped and deploy-pending, the next deploy (or factoryos-workspace update) provisions again.
    if ! "$dir/deploy/provision.sh" "$dir"; then
      provision_failed=yes
      echo "factoryos-deploy: provisioning failed; not stamped, the next deploy provisions again" >&2
      rm -f "$stamp"
    elif [ -s /etc/factoryos/provision-incomplete ]; then
      echo "factoryos-deploy: provisioning incomplete ($(tr '\n' ' ' < /etc/factoryos/provision-incomplete));" \
           "not stamped, the next deploy provisions again" >&2
      rm -f "$stamp"
    else
      mkdir -p "$(dirname "$stamp")" && echo "$want" > "$stamp"
    fi
  fi
fi
# The large sample data lives outside the releases (provision.sh fetches it once); link it into this
# one's cells/, where the backend looks for sample cells.
for data in /var/lib/factoryos/sample-data/*/; do
  [ -d "$data" ] && [ -d "$dir/cells" ] || continue
  data_name=$(basename "$data")
  [ -e "$dir/cells/$data_name" ] || ln -s "${data%/}" "$dir/cells/$data_name"
done
# The Robo Dresser and Robo Fitter write into their asset root (dressed robots, fitted scenes), so
# on the server it is a copy outside the releases, owned by the backend. Items (<collection>/<item>)
# new in this release's catalogue are added once: one the user deleted stays deleted, and existing
# ones are the user's.
catalogue=/var/lib/factoryos/catalogue
# From the starter assets if provision.sh installed them (deploy/starter.txt), else the release's.
source_catalogue="$dir/assets/catalogue"
[ -d /var/lib/factoryos/starter/assets ] && source_catalogue=/var/lib/factoryos/starter/assets
if [ -d "$source_catalogue" ]; then
  install -d -o factoryos -g factoryos "$catalogue"
  seeded="$catalogue/.seeded"
  touch "$seeded"
  # Scenes are in the catalogue's scenes/, a folder each (scenes/<name>/scene.yaml). A starter's
  # fitted-robots/<name>.yaml goes there as the flat scenes/<name>.yaml, which the backend makes a
  # folder at its start (restarted below), as it migrates the workspace's own fitted-robots/.
  # Seeded under any of those names (and present as either layout): seeded once.
  # seed_item <source catalogue> <collection>/<item>
  seed_item() {
    local from=$1 item=$2 dest present scene seen there
    dest=$item
    case "$item" in fitted-robots/*) dest="scenes/${item#fitted-robots/}" ;; esac
    grep -qxF "$item" "$seeded" && return 0
    grep -qxF "$dest" "$seeded" && return 0
    grep -qxF "fitted-robots/${dest#scenes/}" "$seeded" && return 0
    [ -e "$from/$item" ] || return 0
    present=no
    case "$dest" in
      scenes/*)
        scene=${dest#scenes/}
        scene=${scene%.yaml}
        scene=${scene%.yml}
        for seen in "scenes/$scene" "scenes/$scene.yaml" "fitted-robots/$scene.yaml"; do
          grep -qxF "$seen" "$seeded" && present=yes
        done
        for there in "scenes/$scene/scene.yaml" "scenes/$scene.yaml" "scenes/$scene.yml" "fitted-robots/$scene.yaml"; do
          [ -e "$catalogue/$there" ] && present=yes
        done
        ;;
    esac
    [ "$present" = yes ] && { echo "$dest" >> "$seeded"; return 0; }
    if [ ! -e "$catalogue/$dest" ] && [ ! -e "$catalogue/fitted-robots/${dest#scenes/}" ]; then
      mkdir -p "$catalogue/$(dirname "$dest")"
      cp -R "$from/$item" "$catalogue/$dest"
    fi
    echo "$dest" >> "$seeded"
  }
  while read -r item; do
    seed_item "$source_catalogue" "$item"
  done < <(cd "$source_catalogue" && find . -mindepth 2 -maxdepth 2 ! -name .gitkeep | sed 's|^\./||' | sort)
  # Seeded from the starter: the release's sample cells (cells/, provision.sh copies them) still need what they
  # reference. A reference cell (factoryos-cell.json, e.g. welding_cell_2_small) names its Robo Fitter scene, whose
  # robots, fixtures and floor plan are catalogue items: those of the release's catalogue, added the same way (once,
  # unless there).
  if [ "$source_catalogue" != "$dir/assets/catalogue" ] && [ -d "$dir/assets/catalogue" ] && [ -d "$dir/cells" ]; then
    while read -r item; do
      seed_item "$dir/assets/catalogue" "$item"
    done < <(python3 "$dir/deploy/sample-cell-items.py" "$dir/cells" "$dir/assets/catalogue" || true)
  fi
  (cd "$source_catalogue" && find . -mindepth 1 -maxdepth 1 -type d ! -name fitted-robots) | while read -r collection; do
    mkdir -p "$catalogue/$collection"
  done
  mkdir -p "$catalogue/scenes"
  chown -R factoryos:factoryos "$catalogue"
  # Configs from before point the catalogue and asset root at the (read-only) release.
  config=/var/lib/factoryos/factoryos.config.json
  [ -f "$config" ] && python3 - "$config" "$catalogue" <<'PY'
import json, sys
path, catalogue = sys.argv[1:]
config = json.load(open(path))
moved = [key for key in ("catalogue", "assetRoot") if config.get(key) == "/opt/factoryos/current/assets/catalogue"]
for key in moved:
    config[key] = catalogue
if moved:
    with open(path, "w") as file:
        json.dump(config, file, indent=2)
    print("factoryos-deploy: config " + ", ".join(moved) + " -> " + catalogue)
PY
fi
# The starter's products and cells (<name>/ each, e.g. products/body_weld_3 and the reference cell on
# its scene, catalogue/scenes/body_weld_3/scene.yaml) are added the same way: once, an
# existing one is the user's and one the user deleted stays deleted.
for part in products cells; do
  [ -d "/var/lib/factoryos/starter/$part" ] || continue
  target="/var/lib/factoryos/$part"
  install -d -o factoryos -g factoryos "$target"
  seeded="$target/.seeded"
  touch "$seeded"
  for item in "/var/lib/factoryos/starter/$part"/*/; do
    [ -d "$item" ] || continue
    item=$(basename "$item")
    grep -qxF "$item" "$seeded" && continue
    if [ ! -e "$target/$item" ]; then
      cp -R "/var/lib/factoryos/starter/$part/$item" "$target/$item"
      chown -R factoryos:factoryos "$target/$item"
    fi
    echo "$item" >> "$seeded"
  done
  chown factoryos:factoryos "$seeded"
done
systemctl restart factoryos-backend
restarted=yes

# Keep the five newest releases.
ls -1dt "$releases"/*/ | tail -n +6 | xargs -r rm -rf

workspace=$(. /etc/factoryos/workspace.env 2>/dev/null; echo "${FACTORY_OS_WORKSPACE:-default}")
for _ in $(seq 1 30); do
  # Also under the workspace's prefix, as viewers behind the load balancer reach it.
  if curl -fsS http://127.0.0.1/api/v1/health && curl -fsS -o /dev/null "http://127.0.0.1/workspace/$workspace/api/v1/health"; then
    echo; echo "factoryos-deploy: $name is up"; cat "$dir/BUILD-INFO.txt" 2>/dev/null || true
    # Tag the instance with its release: the web viewer loads the same one (the provisioner says which).
    [ -x /usr/local/bin/factoryos-workspace ] && /usr/local/bin/factoryos-workspace ready >/dev/null 2>&1 || true
    if [ "$provision_failed" = yes ]; then
      echo "factoryos-deploy: $name runs, but its provisioning failed (see above)" >&2
      exit 1
    fi
    rm -f "$pending"
    exit 0
  fi
  sleep 1
done
echo "factoryos-deploy: backend did not become healthy" >&2
journalctl -u factoryos-backend -n 80 --no-pager >&2 || true
exit 1
