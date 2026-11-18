#!/bin/bash
#
# factoryos-release: builds and deploys the release the handoff unpacked (deployment/CONTRACT.md), on the instance,
# as root, from factoryos-release.service (deployment/bootstrap/release-handoff.sh installs both from the export it
# releases, and starts the unit). It continues the release recorded in /var/lib/factoryos-release/state.json (the
# handoff wrote it `downloading`) through the phases
#   deps        deployment/bootstrap/install-build-deps.sh (skipped while its stamp matches)
#   building    <tree>/scripts/build-release.sh, <tree> the build root in /opt/factoryos/src/<version> (below)
#   packaging   <tree>/scripts/package-bundle.sh -> /opt/factoryos/bundles/factoryos-viewer-linux-arm64-<version>.tar.gz
#               (+ .sha256); then the export's cargo target dirs are deleted
#   deploying   <tree>/deploy/factoryos-deploy.sh <bundle> <sha256> with FACTORYOS_PUBLIC=1 (switches
#               /opt/factoryos/current, provisions, restarts the backend)
#   checking    GET http://127.0.0.1/api/v1/health and factoryos-planner-router active, for ~3 min; then the planners'
#               venvs (provision.sh's) import their runtime, as the factoryos user; umarell_jam's setup is checked
#               too, but only reported: one that is not set up is marked incomplete, never a rollback
# A request with a `bundle` (the release build in AWS CodeBuild made it of this export; the handoff downloaded it to
# the packaging phase's path) goes from `downloading` straight to `deploying`: no deps, build or packaging here.
# The build root <tree>: the export's source/ in the Guidance layout (deployment/ at the export's root, everything else
# under source/: source/scripts/build-release.sh exists), else the export's root (the flat layout of the internal
# repo). deployment/bootstrap/ is at the export's root in both.
# and ends `ready`, `failed` (current untouched: the release before keeps serving) or `rolled-back` (the new release
# was switched to but is not healthy, or its deploy failed: current points back at the previous release, whose
# backend is restarted, and the next deploy provisions again). A `ready` state lists in `incomplete` what
# /etc/factoryos/provision-incomplete names (optional parts provisioned without, e.g. umarell_jam), if anything.
# Each phase goes to the state parameter /factoryos/<stack>/release/state; the error of a failure is a one-line
# summary of the failing step's output.
#
# Resumable: the unit is also started at boot (WantedBy=multi-user.target). A release whose phase is not terminal (a
# reboot, e.g. a patch association's, stopped it) continues from the start of that phase; with nothing to continue
# it exits 0. A SIGTERM during a shutdown leaves the phase as it is (the boot resumes it); one by hand fails it
# (`systemctl kill`: the unit refuses `systemctl stop` and `restart`, RefuseManualStop=yes).
# One run at a time: a lock (flock) in /run.
#
# Also: the optional Kiro key (FACTORYOS_KIRO_SECRET in stack.env, which the handoff keeps current, names a Secrets
# Manager secret) into
# /etc/factoryos/kiro.env (0600 root, KIRO_API_KEY=…; never echoed), apt's timers held during the run (and restored),
# NEEDRESTART_SUSPEND=1 and /etc/needrestart/conf.d/90-factoryos.conf (no FactoryOS unit restarted by needrestart), a
# refusal when docker is on PATH. It keeps the release before this one and this one (their
# release dirs, exports and bundles) and deletes the rest. It never touches /var/lib/factoryos (the user's data).
# Every step's output is appended to /var/log/factoryos/release.log (CloudWatch: <instance id>/release.log).
#
# Paths honour FACTORYOS_ROOT (a prefix for /opt, /etc, /var, /run: deployment/bootstrap/tests/test-release.sh), and
# FACTORYOS_RELEASE_HEALTH_TRIES / _INTERVAL (36 x 5 s) the health check.
set -euo pipefail
umask 022

R=${FACTORYOS_ROOT:-}
[ -r "$R/etc/factoryos/stack.env" ] || { echo "factoryos-release: $R/etc/factoryos/stack.env is missing" >&2; exit 1; }
# The stack's settings are stack.env's only (the handoff writes FACTORYOS_KIRO_SECRET and FACTORYOS_BEDROCK into it).
unset FACTORYOS_KIRO_SECRET FACTORYOS_BEDROCK
# shellcheck disable=SC1091
. "$R/etc/factoryos/stack.env"
: "${FACTORYOS_STACK:?}" "${AWS_REGION:?}"
export AWS_DEFAULT_REGION="$AWS_REGION" NEEDRESTART_SUSPEND=1 DEBIAN_FRONTEND=noninteractive

opt="$R/opt/factoryos"
srcs="$opt/src"
bundles="$opt/bundles"
releases="$opt/releases"
lib="$R/var/lib/factoryos-release"
log="$R/var/log/factoryos/release.log"
lock="$R/run/factoryos-release.lock"
prefix=factoryos-viewer-linux-arm64-
param="/factoryos/$FACTORYOS_STACK/release/state"
mkdir -p "$srcs" "$bundles" "$releases" "$lib" "$(dirname "$log")" "$(dirname "$lock")"
# The backend and the planner router run the release as the factoryos user: /opt/factoryos and its releases/ must be
# traversable by it whatever umask created them (an earlier handoff's 0700 /opt/factoryos is repaired here too).
chmod 0755 "$opt" "$releases"
# needrestart (Ubuntu 24.04 runs it after every apt/dpkg run) never restarts a FactoryOS unit. Our own apt calls set
# NEEDRESTART_SUSPEND=1, an account patch association's do not: its upgrade had needrestart restart the test-install
# runner's unit mid-phase (2611122014-dc89), and here it would restart this release mid-build or the backend
# mid-check. The rest of the system keeps needrestart's default; factoryos-release.service also refuses a manual
# stop or restart (RefuseManualStop=yes). Rewritten only when it differs.
nr_conf="$R/etc/needrestart/conf.d/90-factoryos.conf"
nr_want="# FactoryOS (deployment/bootstrap/factoryos-release.sh): needrestart never restarts a FactoryOS unit.
\$nrconf{override_rc}{qr/^(factoryos|fos)-/} = 0;"
if [ "$(cat "$nr_conf" 2>/dev/null || true)" != "$nr_want" ]; then
  { mkdir -p "$(dirname "$nr_conf")" && printf '%s\n' "$nr_want" > "$nr_conf.new" \
      && mv -f "$nr_conf.new" "$nr_conf"; } || echo "factoryos-release: WARNING: could not write $nr_conf" >&2
fi

now() { date -u +%Y-%m-%dT%H:%M:%SZ; }
say() { echo "factoryos-release: $*"; echo "[$(now) release] $*" >> "$log"; }
sha256() { sha256sum "$1" | cut -d' ' -f1; }

exec 9>"$lock"
flock -n 9 || { say "another release run holds $lock; not starting a second one"; exit 1; }

# --- the release to continue: the handoff's state.json and request.json
[ -f "$lib/state.json" ] || { say "nothing to do: no release was requested on this instance"; exit 0; }
{ read -r phase; read -r version; read -r previous; read -r started; } < <(python3 -I -c '
import json, sys
s = json.load(open(sys.argv[1]))
for k in ("phase", "version", "previous", "startedAt"):
    print(s.get(k) or "")' "$lib/state.json")
stream="$(cat "$R/var/lib/cloud/data/instance-id" 2>/dev/null || echo unknown)/release.log"

terminal=no   # a terminal state was written (the exit trap writes `failed` otherwise)
interrupted=no
# set_state <phase> [<error>]: the state into state.json and the parameter (retried; never fatal). `ready` with
# `incomplete`: the names in /etc/factoryos/provision-incomplete (once each, in order), when there are any.
set_state() {
  local json i finished="" incomplete=""
  case "$1" in ready|failed|rolled-back) finished=$(now); terminal=yes ;; esac
  [ "$1" != ready ] || incomplete=$(cat "$R/etc/factoryos/provision-incomplete" 2>/dev/null || true)
  phase=$1
  json=$(python3 -I -c '
import json, sys
phase, version, previous, started, finished, error, stream, now, incomplete = sys.argv[1:]
s = {"phase": phase, "version": version, "previous": previous or None, "startedAt": started, "updatedAt": now,
     "finishedAt": finished or None, "error": error or None, "logStream": stream}
inc = list(dict.fromkeys(x.strip() for x in incomplete.splitlines() if x.strip()))
if inc:
    s["incomplete"] = inc
print(json.dumps(s, separators=(",", ":")))' "$1" "$version" "$previous" "$started" "$finished" "${2:-}" "$stream" "$(now)" \
    "$incomplete")
  printf '%s\n' "$json" > "$lib/state.json.new" && mv -f "$lib/state.json.new" "$lib/state.json"
  for i in 1 2 3 4 5; do
    aws ssm put-parameter --region "$AWS_REGION" --name "$param" --type String --overwrite --value "$json" \
      >/dev/null 2>>"$log" && break
    [ "$i" = 5 ] && { say "WARNING: could not write $param ($1)"; break; }
    sleep $(( i * 2 ))
  done
  say "state: $1${2:+ ($2)}"
}
fail() { set_state failed "$1"; exit 1; }

# summary <log offset>: one line (at most 300 characters) for the state's error, from what the step wrote to the log
# after <offset>:
#   - the reason: the last error a step's own script wrote ("factoryos-deploy: backend did not become healthy",
#     "provision.sh: … could not be installed", "package-bundle: … is missing": `<script>[.sh]: ` and an error word),
#     else the last error-looking line (a compiler's `error: could not compile …`, ERROR, FATAL, FAILED, a WARNING
#     that something failed, No space left), else the last line
#   - then, when a systemd unit failed (the journal lines factoryos-deploy prints after its health check), the unit's
#     last journal error ("factoryos-backend.service: Failed to execute …: Permission denied"); without one, from the
#     reason's own script an earlier error of another script (factoryos-deploy's "provisioning failed" names
#     provision.sh's)
# Never the reason: provision.sh's informational `[provision] kiro agent file: …` (kiro-cli validates the chat agent
# only when it is logged in; "not logged in" is the usual answer on a public install), nor this script's step headers.
summary() {
  tail -c +"$(( $1 + 1 ))" "$log" | python3 -I -c '
import re, sys
own = re.compile(r"^(factoryos-[a-z-]+|provision[a-z-]*|build-release|package-bundle|install-build-deps|fetch-umarell"
                 r"|release-handoff)(\.sh)?: ")
own_err = re.compile(r"(?i)error|fail|did not|not become|missing|no such|cannot|can.t |could not|not found|mismatch"
                     r"|expected|denied|refus|not a |is not |timed? ?out")
generic = re.compile(r"^(error|Error|ERROR|fatal|FATAL|E:)|[^a-z]error(\[|:)|FAILED|No space left"
                     r"|^(\[[a-z-]+\] )?WARNING: .*(fail|not )")
noise = re.compile(r"^\[provision\] kiro agent file\b|^---- ")
# journalctl -o short: "Nov 12 21:07:31 host ident[pid]: message".
journal = re.compile(r"^[A-Z][a-z]{2} [ 0-9]?[0-9] [0-9]{2}:[0-9]{2}:[0-9]{2} \S+ (\S+?)(\[[0-9]+\])?: (.*)$")
j_err = re.compile(r"Failed to |[Pp]ermission denied|panicked|[Ee]rror|ERROR|No such file|[Kk]illed|core-dump"
                   r"|Out of memory|address already in use")
j_bookkeeping = re.compile(r"Failed with result|Main process exited|Scheduled restart job|Start request repeated"
                           r"|Consumed |Deactivated successfully|^Started |^Stopped |^Starting |^Stopping ")
clean = lambda s: "".join(c if c.isprintable() else " " for c in s).strip()
lines, jerr, jexit = [], None, None
for raw in sys.stdin.buffer.read().decode("utf-8", "replace").replace("\r", "").split("\n"):
    s = clean(raw)
    if not s or noise.search(s):
        continue
    m = journal.match(s)
    if m:
        ident, msg = m.group(1).strip("()"), m.group(3)
        if not re.match(r"\S+\.(service|socket|timer): ", msg):
            msg = ident + ": " + msg
        if j_err.search(msg) and not j_bookkeeping.search(msg):
            jerr = msg
        elif "Main process exited" in msg:
            jexit = msg
        continue
    lines.append(s)
own_lines = [i for i, s in enumerate(lines) if own.match(s) and own_err.search(s)]
if own_lines:
    i = own_lines[-1]
else:
    gen = [i for i, s in enumerate(lines) if generic.search(s)]
    i = gen[-1] if gen else (len(lines) - 1 if lines else None)
reason = lines[i] if i is not None else ""
detail = jerr or jexit
if detail is None and i is not None and own_lines:
    script = own.match(lines[i]).group(1)
    earlier = [s for s in lines[:i] if (own.match(s) and own_err.search(s) and own.match(s).group(1) != script)
               or re.match(r"\[[a-z-]+\] (WARNING|ERROR)\b", s)]
    detail = earlier[-1] if earlier else None
out = "; ".join(x for x in ((reason[:200] if detail else reason), detail) if x)
print(out[:300], end="")' || true
}
# run_step <label> <command…>: the command with its output appended to the log; fails the release with a summary.
run_step() {
  local label=$1 offset rc=0 a what=""
  shift
  for a in "$@"; do case "$a" in *.sh) what=$(basename "$a"); break ;; esac; done
  offset=$(wc -c < "$log" | tr -d ' ')
  echo "---- [$(now)] $label: $*" >> "$log"
  "$@" >> "$log" 2>&1 < /dev/null || rc=$?
  if [ "$rc" = 0 ]; then return 0; fi
  step_error="$label: ${what:-$1} exited $rc: $(summary "$offset")"
  return "$rc"
}

case "$phase" in
  downloading)
    # The handoff starts the unit once the export is unpacked; at boot an unfinished download is a failed one.
    if [ -n "$version" ] && [ -f "$lib/request.json" ] \
       && [ "$(cat "$srcs/$version/.factoryos-export" 2>/dev/null)" = "$(python3 -I -c 'import json, sys
print(json.load(open(sys.argv[1])).get("sha256") or "-")' "$lib/request.json")" ]; then
      # A bundle the release build in AWS CodeBuild made (the request's `bundle`): no deps, build or packaging here.
      if [ -n "$(python3 -I -c 'import json, sys
print((json.load(open(sys.argv[1])).get("bundle") or {}).get("sha256") or "")' "$lib/request.json")" ]; then
        phase=deploying
      else
        phase=deps
      fi
    else
      fail "downloading: interrupted before the export was unpacked; run the release step again"
    fi ;;
  deps|building|packaging|deploying|checking) ;;
  *) say "nothing to do: ${version:-no release} is ${phase:-unknown}"; exit 0 ;;
esac

# The request (the handoff's request.json): the umarell and umarell_jam tarballs, if any.
{ read -r req_version; read -r utar; read -r usha; read -r jtar; read -r jsha; read -r bsha; } < <(python3 -I -c '
import json, sys
r = json.load(open(sys.argv[1]))
u = r.get("umarell") or {}
j = r.get("umarellJam") or {}
b = r.get("bundle") or {}
print(r.get("version") or ""); print(u.get("path") or ""); print(u.get("sha256") or "")
print(j.get("path") or ""); print(j.get("sha256") or ""); print(b.get("sha256") or "")' "$lib/request.json")
src="$srcs/$version"
bundle="$bundles/$prefix$version.tar.gz"
new_dir="$releases/$prefix$version"
keep_dir="$releases/.rollback-$version"
echo "---- $(now) factoryos-release: $version from $phase (previous ${previous:-none}) ----" >> "$log"

# --- exit handling: apt's timers back; an unexpected exit fails the release, a shutdown leaves it to the boot.
held=""
on_exit() {
  local rc=$?
  # shellcheck disable=SC2086 # $held: a list of unit names
  [ -z "$held" ] || systemctl start $held >/dev/null 2>&1 || true
  if [ "$rc" != 0 ] && [ "$terminal" = no ] && [ "$interrupted" = no ]; then
    set_state failed "$phase: factoryos-release stopped unexpectedly (exit $rc), see $stream"
  fi
}
on_term() {
  if [ "$(systemctl is-system-running 2>/dev/null || true)" = stopping ]; then
    interrupted=yes
    say "stopped by the shutdown during $phase; the next boot continues $version from there"
    exit 143
  fi
  fail "$phase: stopped (SIGTERM, not a shutdown)"
}
trap on_exit EXIT
trap on_term TERM INT

[ "$req_version" = "$version" ] || fail "$phase: $lib/request.json is for '${req_version}', the state for '$version'"
[ -d "$src" ] || fail "$phase: $src is missing"
# The build root (see the header): the Guidance layout's source/, else the flat layout's root.
if [ -f "$src/source/scripts/build-release.sh" ]; then tree="$src/source"; else tree="$src"; fi

# The host must not have docker: the backend's agentic mode would run the planner in Docker instead of its venv.
if command -v docker >/dev/null 2>&1; then
  fail "$phase: docker is on PATH ($(command -v docker)); the public deployment runs the planner natively: remove it"
fi

# apt's timers would compete for the dpkg lock and restart services mid-build.
for u in apt-daily.timer apt-daily-upgrade.timer unattended-upgrades.service; do
  if systemctl is-active --quiet "$u" 2>/dev/null && systemctl stop "$u" 2>/dev/null; then held="$held $u"; fi
done
[ -z "$held" ] || say "holding apt:$held (until the end of this run)"

# --- the optional Kiro key: root reads the secret into a 0600 EnvironmentFile (the backend's drop-in reads it).
kiro_env() {
  local f="$R/etc/factoryos/kiro.env"
  mkdir -p "$R/etc/factoryos"
  if [ -z "${FACTORYOS_KIRO_SECRET:-}" ]; then
    if [ -e "$f" ]; then rm -f "$f"; say "kiro: no secret configured; $f removed"; fi
    return 0
  fi
  rm -f "$f.new"
  # The value only ever goes from the CLI's stdout through python into the file: no argv, no log.
  if ! (umask 077
        aws secretsmanager get-secret-value --region "$AWS_REGION" --secret-id "$FACTORYOS_KIRO_SECRET" \
            --query SecretString --output json 2>"$lib/kiro.err" | python3 -I -c '
import json, re, sys
raw = json.load(sys.stdin)
if not isinstance(raw, str):
    sys.exit("the secret has no SecretString")
value = raw.strip()
try:
    obj = json.loads(value)
except ValueError:
    obj = None
if isinstance(obj, dict):
    value = next((obj[k] for k in ("KIRO_API_KEY", "apiKey", "api_key") if isinstance(obj.get(k), str)), "").strip()
if not re.fullmatch(r"[A-Za-z0-9._~+/=:-]{8,4096}", value):
    sys.exit("the secret is not a key (one word of letters, digits and ._~+/=:-), or a JSON object with KIRO_API_KEY")
print("KIRO_API_KEY=" + value)' > "$f.new" 2>>"$lib/kiro.err"); then
    rm -f "$f.new"
    fail "$phase: cannot read the Kiro key from secret $FACTORYOS_KIRO_SECRET: $(tr '\n' ' ' < "$lib/kiro.err" | cut -c1-200)"
  fi
  chmod 0600 "$f.new"
  if [ "$(id -u)" = 0 ]; then chown root:root "$f.new"; fi
  mv -f "$f.new" "$f"
  rm -f "$lib/kiro.err"
  say "kiro: $f written from secret $FACTORYOS_KIRO_SECRET"
}

health_ok() {
  curl -fsS -m 10 -o /dev/null http://127.0.0.1/api/v1/health 2>>"$log" || { health_error="GET /api/v1/health failed"; return 1; }
  systemctl is-active --quiet factoryos-planner-router || { health_error="factoryos-planner-router is not active"; return 1; }
}
wait_healthy() { # <tries>
  local i
  for i in $(seq 1 "$1"); do
    health_ok && return 0
    [ "$i" = "$1" ] || sleep "${FACTORYOS_RELEASE_HEALTH_INTERVAL:-5}"
  done
  return 1
}

# venv_ok <label> <dir> <python> <code> [<env>…]: <python> runs <code> in <dir> with the environment <env>, as the
# factoryos user (the venvs are its: root never runs code it could have written), no bytecode written, at most 5 min.
venv_ok() {
  local label=$1 dir=$2 py=$3 code=$4 offset rc=0
  shift 4
  [ -x "$py" ] && [ -d "$dir" ] || { health_error="the $label venv is missing ($py, $dir)"; return 1; }
  local as=() limit=()
  if [ "$(id -u)" = 0 ]; then as=(sudo -u factoryos -H); fi
  if command -v timeout >/dev/null 2>&1; then limit=(timeout 300); fi
  offset=$(wc -c < "$log" | tr -d ' ')
  echo "---- [$(now)] checking: the $label venv imports its runtime ($py)" >> "$log"
  (cd "$dir" && ${as[@]+"${as[@]}"} ${limit[@]+"${limit[@]}"} env PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 "$@" \
     "$py" -W ignore -c "$code") >> "$log" 2>&1 < /dev/null || rc=$?
  [ "$rc" = 0 ] && return 0
  health_error="the $label venv does not import (exit $rc): $(summary "$offset")"
  return 1
}
# mark_incomplete <name>: <name> into /etc/factoryos/provision-incomplete (once), and the provision stamp dropped: the
# state's `incomplete` reports it, the next deploy provisions again.
mark_incomplete() {
  local f="$R/etc/factoryos/provision-incomplete"
  mkdir -p "$R/etc/factoryos"
  grep -qxF "$1" "$f" 2>/dev/null || echo "$1" >> "$f"
  rm -f "$R/etc/factoryos/provisioned.sha256"
}
# jam_check: umarell_jam (the planner router's umarell_jam) is set up, when the release ships it (its router script and
# the cell planner's umarell_jam/) and workspace.env does not disable it (FACTORY_OS_UMARELL_JAM=0): its venv
# (provision-cell-planner.sh's jam-venv) imports the session's numerical stack at the versions its cache keys depend on
# (pinocchio as the cell planner's requirements-umarell.txt pins it, toppra as deploy/toppra.pin), as the factoryos
# user; and the plain umarell commit of deploy/umarell-jam.pin is at /var/lib/factoryos/umarell-<7 hex>
# (deploy/fetch-umarell.sh). umarell_jam is optional (its jobs fall back to umarell_u3): a failure is never a health
# failure, only a WARNING in the log and umarell_jam marked incomplete (mark_incomplete). Always returns 0.
JAM_IMPORTS='
import os, numpy, pinocchio, hppfcl, scipy, ompl, toppra  # noqa: F401 -- what umarell_jam keys its cache by
from importlib.metadata import version
want, have = os.environ.get("JAM_PIN", ""), pinocchio.__version__
assert not want or have == want, "pinocchio %s, the cell planner pins %s" % (have, want)
want, have = os.environ.get("JAM_TOPPRA", ""), version("toppra")
assert not want or have == want, "toppra %s, deploy/toppra.pin pins %s" % (have, want)
'
jam_check() {
  local cp="$R/opt/factoryos-cell-planner/router" pin commit base stamp pinv tv err="" saved="${health_error:-}"
  [ -f "$new_dir/deploy/planner-router/umarell_jam.py" ] && [ -f "$new_dir/cell-planner/umarell_jam/__main__.py" ] \
    || return 0
  if [ "$( . "$R/etc/factoryos/workspace.env" 2>/dev/null; echo "${FACTORY_OS_UMARELL_JAM:-1}")" = 0 ]; then
    say "checking: umarell_jam is off on this host (FACTORY_OS_UMARELL_JAM=0)"
    return 0
  fi
  pin="$new_dir/deploy/umarell-jam.pin"
  commit=$(awk '!/^[[:space:]]*(#|$)/ {print $1; exit}' "$pin" 2>/dev/null || true)
  if ! [[ "$commit" =~ ^[0-9a-f]{40}$ ]]; then
    err="the release has no deploy/umarell-jam.pin (<40-hex commit> <url>)"
  else
    base="$R/var/lib/factoryos/umarell-${commit:0:7}"
    stamp=$(cat "$base/.umarell.sha256" 2>/dev/null || true)
    if [ ! -d "$base/agents" ]; then
      err="no plain umarell ${commit:0:7} in $base (deploy/fetch-umarell.sh: from GitHub, or --umarell-jam-tarball)"
    elif [ -n "$stamp" ] && [[ "$stamp" != "$commit"* ]]; then
      err="$base is not umarell $commit (its stamp: ${stamp:0:60})"
    fi
  fi
  if [ -z "$err" ]; then
    pinv=$(sed -n 's/^pin==\([^[:space:];]*\).*/\1/p' "$cp/src/requirements-umarell.txt" 2>/dev/null | head -1)
    # deploy/toppra.pin: its toppra==<version> line (a hash-checking requirements file); else the requirements' pin.
    tv=$(sed -n 's/^[[:space:]]*toppra[[:space:]=]*\([0-9][^[:space:];]*\).*/\1/p' "$new_dir/deploy/toppra.pin" 2>/dev/null | head -1)
    [ -n "$tv" ] || tv=$(sed -n 's/^toppra==\([^[:space:];]*\).*/\1/p' "$cp/src/requirements-umarell.txt" 2>/dev/null | head -1)
    venv_ok umarell_jam "$cp/src" "$cp/jam-venv/bin/python" "$JAM_IMPORTS" JAM_PIN="$pinv" JAM_TOPPRA="$tv" \
      || err="$health_error"
  fi
  health_error="$saved"
  if [ -z "$err" ]; then
    say "checking: umarell_jam is set up (its venv, umarell ${commit:0:7})"
    return 0
  fi
  say "WARNING: umarell_jam is not set up: $err. Marked incomplete in /etc/factoryos/provision-incomplete: the release serves, umarell_jam's jobs fall back to umarell_u3, the next deploy provisions it again"
  mark_incomplete umarell_jam
  return 0
}
# planner_venvs_ok: the planners' venvs that deploy/provision.sh builds import what the planners need (the imports of
# provision.sh's and provision-cell-planner.sh's own checks): umarell's planner venv always; the planner router's cell
# planner venv unless /etc/factoryos/workspace.env disables the cell planner (FACTORY_OS_CELL_PLANNER=0), then
# umarell_jam's setup (jam_check: reported, never a failure).
planner_venvs_ok() {
  local umarell="$R/var/lib/factoryos/umarell" cp="$R/opt/factoryos-cell-planner/router" cell_planner
  venv_ok planner "$umarell" "$umarell/.venv-planner/bin/python" '
import pinocchio as pin, ompl, langgraph, mcp  # noqa: F401
assert hasattr(pin.Frame(), "parentJoint"), "Pinocchio 3 needed"
import agents.trajectory.tools.pinocchio, agents.trajectory.tools.ompl_planner  # noqa: F401
' PYTHONPATH="$umarell" || return 1
  cell_planner=$( . "$R/etc/factoryos/workspace.env" 2>/dev/null; echo "${FACTORY_OS_CELL_PLANNER:-1}")
  [ "$cell_planner" = 0 ] && return 0
  venv_ok "cell planner" "$cp/src" "$cp/venv/bin/python" '
import torch, mujoco, pinocchio, torch_geometric  # noqa: F401
import tools.btexport, env.cell  # noqa: F401
' PYTHONPATH=. MUJOCO_GL=disable UMARELL_REPO="$umarell" || return 1
  jam_check
}

# prune: the release before this one and this one stay (release dirs, exports, umarell tarballs, bundles); the
# release current points at always stays.
prune() {
  local d name v cur
  cur=$(readlink "$opt/current" 2>/dev/null || true)
  keep() { [ "$1" = "$version" ] || { [ -n "$previous" ] && [ "$1" = "$previous" ]; }; }
  for d in "$releases/$prefix"* "$releases"/.rollback-*; do
    [ -e "$d" ] || continue
    [ "$d" = "$cur" ] && continue
    name=${d##*/}
    case "$name" in .rollback-*) rm -rf "$d"; continue ;; esac
    keep "${name#"$prefix"}" || { rm -rf "$d"; say "pruned release $name"; }
  done
  for d in "$srcs"/* "$srcs"/.incoming-*; do
    [ -e "$d" ] || continue
    name=${d##*/}
    case "$name" in
      .incoming-*) rm -rf "$d"; continue ;;
      *.umarell.tar.gz) v=${name%.umarell.tar.gz} ;;
      *.umarell-jam.tar.gz) v=${name%.umarell-jam.tar.gz} ;;
      *) v=$name ;;
    esac
    keep "$v" || { rm -rf "$d"; say "pruned export $name"; }
  done
  for d in "$bundles"/*; do
    [ -e "$d" ] || continue
    name=${d##*/}
    v=${name#"$prefix"}; v=${v%.partial}; v=${v%.sha256}; v=${v%.tar.gz}
    keep "$v" || rm -f "$d"
  done
}

# rollback <reason>: current back to the previous release (restored from its copy when it is the same version),
# its backend restarted, `rolled-back`; without a previous release, `failed`.
rollback() {
  local reason=$1 prev_dir="$releases/$prefix$previous" note=""
  [ -n "$previous" ] || fail "$reason; no previous release to roll back to"
  if [ "$previous" = "$version" ]; then
    [ -d "$keep_dir" ] || fail "$reason; the earlier build of $version was not kept, no rollback"
    rm -rf "$prev_dir" && mv "$keep_dir" "$prev_dir"
  fi
  [ -x "$prev_dir/bin/factoryos-backend" ] || fail "$reason; the previous release $previous is gone, no rollback"
  say "rolling back to $previous: $reason"
  ln -sfn "$prev_dir" "$opt/current"
  mkdir -p "$R/etc/factoryos"
  echo "FACTORY_OS_RELEASE=$previous" > "$R/etc/factoryos/release.env"
  # The new release's provisioning ran: the next deploy provisions again.
  rm -f "$R/etc/factoryos/provisioned.sha256" "$R/etc/factoryos/deploy-pending"
  systemctl restart factoryos-backend >> "$log" 2>&1 || note="; its restart failed"
  if [ -z "$note" ] && ! wait_healthy "$(( (${FACTORYOS_RELEASE_HEALTH_TRIES:-36} + 2) / 3 ))"; then
    note="; $previous is not healthy either ($health_error)"
  fi
  set_state rolled-back "$reason; rolled back to $previous$note"
  prune
  exit 1
}

kiro_env

# --- the phases, from the one to continue
order="deps building packaging deploying checking"
from=$phase
reached=no
built=no
for p in $order; do
  [ "$p" = "$from" ] && reached=yes
  [ "$reached" = yes ] || continue
  case "$p" in
    deps)
      deps_script="$src/deployment/bootstrap/install-build-deps.sh"
      [ -f "$deps_script" ] || fail "deps: $deps_script is missing"
      if [ "$(bash "$deps_script" --stamp)" = "$(cat "$lib/deps.sha256" 2>/dev/null)" ]; then
        say "deps: up to date ($lib/deps.sha256)"
      else
        set_state deps
        run_step deps bash "$deps_script" || fail "$step_error"
      fi ;;
    building)
      set_state building
      # A new build is packaged anew: an earlier bundle of this version (a forced rebuild) goes.
      rm -f "$bundle" "$bundle.sha256" "$bundle.partial"
      # The toolchain's environment (install-build-deps.sh writes it).
      # shellcheck disable=SC1091
      if [ -r "$R/etc/profile.d/factoryos-build.sh" ]; then . "$R/etc/profile.d/factoryos-build.sh"; fi
      run_step building env FACTORYOS_BUILD_ID="$version" bash "$tree/scripts/build-release.sh" || fail "$step_error"
      built=yes ;;
    packaging)
      set_state packaging
      if [ "$built" = no ] && [ -f "$bundle" ] && [ "$(cat "$bundle.sha256" 2>/dev/null)" = "$(sha256 "$bundle")" ]; then
        say "packaging: $bundle is complete (packaged before the interruption)"
      else
        # package-bundle.sh writes it atomically; the .sha256 last marks it complete.
        rm -f "$bundle" "$bundle.sha256"
        run_step packaging bash "$tree/scripts/package-bundle.sh" "$bundle" || fail "$step_error"
        sha256 "$bundle" > "$bundle.sha256.new" && mv -f "$bundle.sha256.new" "$bundle.sha256"
      fi
      # The build's cargo target dirs (tens of GB; marked by cargo's CACHEDIR.TAG): target, cell/target and
      # plan2cell/target of the build root (in the Guidance layout source/target, source/cell/target, ...).
      while IFS= read -r d; do
        if [ -f "$d/CACHEDIR.TAG" ]; then rm -rf "$d"; say "packaging: deleted $d"; fi
      done < <(find "$src" -maxdepth 4 -type d -name target -prune -print 2>/dev/null)
      ;;
    deploying)
      [ -f "$bundle" ] && [ -f "$bundle.sha256" ] || fail "deploying: $bundle is missing"
      # The CodeBuild release build's bundle: the one the request names, byte for byte.
      if [ -n "$bsha" ] && [ "$(sha256 "$bundle")" != "$bsha" ]; then
        fail "deploying: $bundle is not the bundle the release build made (sha256 $bsha)"
      fi
      [ -z "$bsha" ] || [ "$built" = yes ] || say "deploying: the bundle the release build in AWS CodeBuild made"
      set_state deploying
      # The same version again (force): its running build is kept for a rollback (factoryos-deploy replaces it).
      if [ "$previous" = "$version" ] && [ ! -d "$keep_dir" ] && [ -d "$new_dir" ]; then
        cp -a "$new_dir" "$keep_dir"
      fi
      umarell_env=()
      if [ -n "$utar" ]; then
        [ -f "$utar" ] || fail "deploying: the umarell tarball $utar is missing"
        umarell_env=(FACTORYOS_UMARELL_TARBALL="$utar" FACTORYOS_UMARELL_SHA256="$usha")
      fi
      # umarell_jam's (optional): a missing one only leaves umarell_jam to GitHub (or incomplete), never a failure.
      if [ -n "$jtar" ] && [ -f "$jtar" ]; then
        umarell_env+=(FACTORYOS_UMARELL_JAM_TARBALL="$jtar" FACTORYOS_UMARELL_JAM_SHA256="$jsha")
      elif [ -n "$jtar" ]; then
        say "WARNING: the umarell_jam tarball $jtar is missing: provisioning goes on without it"
      fi
      if ! run_step deploying env FACTORYOS_PUBLIC=1 ${umarell_env[@]+"${umarell_env[@]}"} \
           bash "$tree/deploy/factoryos-deploy.sh" "$bundle" "$(cat "$bundle.sha256")"; then
        # Switched (or the same version, whose directory the deploy replaces): back to the previous release.
        if [ "$previous" = "$version" ] || [ "$(readlink "$opt/current" 2>/dev/null || true)" = "$new_dir" ]; then
          rollback "$step_error"
        fi
        fail "$step_error"
      fi ;;
    checking)
      set_state checking
      [ "$(readlink "$opt/current" 2>/dev/null || true)" = "$new_dir" ] \
        || fail "checking: $opt/current does not point at $new_dir after the deploy"
      health_error=""
      wait_healthy "${FACTORYOS_RELEASE_HEALTH_TRIES:-36}" || rollback "checking: $health_error"
      # Once, after the backend and the router are up: an import check takes seconds (torch) and does not get better
      # by waiting. A failure is a health failure (a rollback; on a first install, failed).
      planner_venvs_ok || rollback "checking: $health_error"
      ;;
  esac
done
[ "$reached" = yes ] || fail "unknown phase '$from'"

rm -rf "$keep_dir"
set_state ready
prune
say "$version is ready ($(grep -m1 '^build:' "$new_dir/BUILD-INFO.txt" 2>/dev/null || echo "no BUILD-INFO.txt"))"
if [ -s "$R/etc/factoryos/provision-incomplete" ]; then
  say "WARNING: provisioning incomplete: $(tr '\n' ' ' < "$R/etc/factoryos/provision-incomplete" | sed 's/ $//') (the state's incomplete; the next deploy provisions it again)"
fi
