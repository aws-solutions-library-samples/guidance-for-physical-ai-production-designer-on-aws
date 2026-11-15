#!/usr/bin/env bash
# The release step on the instance (deployment/bootstrap/release-handoff.sh and factoryos-release.sh), in a temporary
# root (FACTORYOS_ROOT) with aws, systemctl and curl stubbed and an export whose build, package and deploy scripts are
# stand-ins:
#   - the export's layout: the Guidance layout (deployment/ at the root, the rest under source/: the build, package and
#     deploy scripts run from src/<version>/source/, the target dirs under it are deleted) for every case, and the
#     flat layout (all at the root) for a release to ready
#   - the handoff: refuses while a release runs (unit active, lock held); checks the sha256; unpacks a tar.gz or zip
#     with its top directory stripped; installs and starts the export's unit; "already installed" unless forced
#   - a release: every phase into the state parameter, state JSON shapes, ready / previous, target dirs deleted, the
#     bundle and its sha256 handed to factoryos-deploy with FACTORYOS_PUBLIC=1 (and the umarell tarball), pruning to
#     two releases
#   - a build failure leaves current alone (failed); a health failure after the switch rolls back (rolled-back), also
#     for a forced rebuild of the running version; a deploy failure before the switch fails
#   - the planners' venvs: imported once after /health and the router (in their dirs, no bytecode); one that does not
#     import, or is missing, rolls back (on a first install: failed); the cell planner's is skipped when it is disabled
#   - umarell_jam (a release that ships it): its tarball carried from wanted to the deploy (one that cannot be
#     downloaded is left out, never a failure); its venv (the pinned pinocchio and toppra) and the plain umarell of
#     deploy/umarell-jam.pin checked after the cell planner's; not set up: ready all the same, umarell_jam in
#     provision-incomplete and the state's `incomplete`, the provision stamp dropped, no rollback; off with
#     FACTORY_OS_UMARELL_JAM=0; what provision.sh marked incomplete is in the ready state too
#   - a failure's reason (the state's error): the step's script's own last error, then a failed unit's journal error
#     or an earlier script's error (the first real test install's deploy output); never provision.sh's kiro line
#   - the lock; resuming after a reboot (a shutdown's SIGTERM keeps the phase; one by hand fails); nothing to do on
#     a terminal state; an interrupted download fails at boot
#   - the Kiro key: kiro.env 0600, the value in no log, state or argv; a malformed one fails without showing it
#   - the origin secret: origin.env 0600 from the SecureString on every handoff, the backend restarted only when it
#     changed; a rotation's secretsOnly request touches nothing else; missing, not a SecureString or malformed:
#     refused before anything changes, without showing it; the value in no log, state or argv
#   - docker on PATH fails; apt's timers are held and restored; needrestart never restarts a FactoryOS unit, and the
#     unit refuses a manual stop
#   - /var/lib/factoryos (the user's data) is never touched
# Run by ci/test-deploy.sh and verify-build (the export's tree); needs bash (3.2+), python3, tar, gzip and coreutils;
# unzip for the zip case (skipped without it). flock and find are the host's when it has them (Ubuntu, as on the
# instance), else stand-ins below (macOS has no flock, amazonlinux:2023 no findutils).
set -uo pipefail

here=$(cd "$(dirname "$0")" && pwd)
boot=$(dirname "$here")
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
fail=0
check() { # check <name> <want> <got>
  if [ "$2" != "$3" ]; then printf 'FAIL %s\n  want: %s\n  got:  %s\n' "$1" "$2" "$3"; fail=1; else echo "ok   $1"; fi
}
# same_file <a> <b>: the same bytes (python3, not cmp: amazonlinux:2023 has no diffutils).
same_file() { python3 -I -c 'import sys; sys.exit(open(sys.argv[1], "rb").read() != open(sys.argv[2], "rb").read())' "$1" "$2" 2>/dev/null; }
contains() { # contains <name> <needle> <haystack>
  case "$3" in *"$2"*) echo "ok   $1" ;; *) printf 'FAIL %s\n  no "%s" in: %s\n' "$1" "$2" "$3"; fail=1 ;; esac
}

# --- a PATH with the stubs and only the tools the scripts need (no docker, whatever the host has)
sandbox="$work/path"
mkdir -p "$sandbox"
for c in bash sh env cat cp mv rm mkdir rmdir ln readlink basename dirname date sed grep tr cut head tail wc find sort \
         python3 sha256sum tar gzip unzip id chmod chown install mktemp seq sleep touch ls awk du uname hostname kill \
         true false printf xargs cmp stat paste timeout; do
  p=$(command -v "$c" 2>/dev/null) && [ -n "$p" ] && [ "${p#/}" != "$p" ] && ln -sf "$p" "$sandbox/$c"
done
stubs="$work/stubs"
mkdir -p "$stubs"
if ! command -v flock >/dev/null 2>&1; then
  # flock -n <fd> (macOS has none): the same flock(2) on the inherited descriptor.
  cat > "$stubs/flock" <<'EOF'
#!/bin/bash
[ "$1" = -n ] && shift
exec python3 -c 'import fcntl, sys
try:
    fcntl.flock(int(sys.argv[1]), fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError:
    sys.exit(1)' "$1"
EOF
else
  ln -sf "$(command -v flock)" "$stubs/flock"
fi
if ! command -v find >/dev/null 2>&1; then
  # find (amazonlinux:2023, verify-build's image, has no findutils): GNU find's subset the scripts and this test use
  # (one start path; -mindepth, -maxdepth, -type d|f, -name, -prune, -print, ANDed; symlinks not followed; the
  # implicit -print). Anything else fails (exit 1), so a script that needs more fails here, not passes by chance.
  cat > "$stubs/find" <<'EOF'
#!/bin/bash
exec python3 -I -c '
import fnmatch, os, stat, sys
a = sys.argv[1:]
if not a or a[0].startswith("-"):
    sys.exit("find stand-in: one start path first")
start, a, tests, lo, hi, act = a[0], a[1:], [], 0, None, False
while a:
    o = a.pop(0)
    if o in ("-mindepth", "-maxdepth") and a and a[0].isdigit():
        lo, hi = (int(a.pop(0)), hi) if o == "-mindepth" else (lo, int(a.pop(0)))
    elif o == "-type" and a and a[0] in ("d", "f"):
        tests.append(("type", a.pop(0)))
    elif o == "-name" and a:
        tests.append(("name", a.pop(0)))
    elif o in ("-prune", "-print"):
        tests.append((o[1:], None)); act = True
    else:
        sys.exit(f"find stand-in: {o} is not supported")
if not act:
    tests.append(("print", None))
def visit(p, depth):
    try:
        st = os.lstat(p)
    except OSError as e:
        print(f"find: {p}: {e.strerror}", file=sys.stderr); return 1
    descend, rc = stat.S_ISDIR(st.st_mode), 0
    if depth >= lo:
        for t, v in tests:
            if t == "type" and not (stat.S_ISDIR(st.st_mode) if v == "d" else stat.S_ISREG(st.st_mode)): break
            if t == "name" and not fnmatch.fnmatchcase(os.path.basename(p.rstrip("/")) or p, v): break
            if t == "prune": descend = False
            if t == "print": print(p)
    if descend and (hi is None or depth < hi):
        for n in sorted(os.listdir(p)):
            rc |= visit(p.rstrip("/") + "/" + n if p != "/" else "/" + n, depth + 1)
    return rc
sys.exit(visit(start, 0))
' "$@"
EOF
fi
# sudo: as root (CI's containers run the tests as root) factoryos-release.sh checks the venvs as the factoryos user,
# `sudo -u factoryos -H <command>`: logged, then <command> as this user (a test cannot switch to a user the host
# lacks). Any other use fails, so a change of how the release drops root is seen here.
cat > "$stubs/sudo" <<'EOF'
#!/bin/bash
f="$FACTORYOS_ROOT/.fake"
{ [ "${1:-}" = -u ] && [ "${2:-}" = factoryos ] && [ "${3:-}" = -H ] && [ $# -gt 3 ]; } \
  || { echo "sudo stub: unexpected sudo $*" >&2; exit 1; }
echo "sudo $*" >> "$f/calls"
shift 3
exec "$@"
EOF
# aws: ssm get/put-parameter on files, s3 cp from a directory, secretsmanager from files; every call logged (argv).
cat > "$stubs/aws" <<'EOF'
#!/bin/bash
f="$FACTORYOS_ROOT/.fake"
echo "aws $*" >> "$f/calls"
arg() { local want=$1; shift; while [ $# -gt 0 ]; do [ "$1" = "$want" ] && { echo "$2"; return; }; shift; done; }
case "$1 $2" in
  "ssm get-parameter")
    n=$(arg --name "$@" | tr / _)
    [ -f "$f/ssm/$n" ] || { echo "An error occurred (ParameterNotFound) when calling the GetParameter operation" >&2; exit 254; }
    cat "$f/ssm/$n" ;;
  "ssm put-parameter")
    n=$(arg --name "$@" | tr / _)
    mkdir -p "$f/ssm"
    arg --value "$@" > "$f/ssm/$n"
    arg --value "$@" >> "$f/ssm/$n.history"
    arg --value "$@" >> "$f/all-states" ;;
  "s3 cp")
    src=${3#s3://}
    [ -f "$f/s3/$src" ] || { echo "fatal error: An error occurred (404) when calling the HeadObject operation" >&2; exit 1; }
    cp "$f/s3/$src" "$4" ;;
  "secretsmanager get-secret-value")
    id=$(arg --secret-id "$@")
    [ -f "$f/secrets/$id" ] || { echo "An error occurred (AccessDeniedException)" >&2; exit 254; }
    python3 -c 'import json, sys; print(json.dumps(open(sys.argv[1]).read()))' "$f/secrets/$id" ;;
  *) echo "aws stub: unexpected $*" >&2; exit 2 ;;
esac
EOF
# systemctl: unit states from files (.fake/units/<unit>), everything else logged.
cat > "$stubs/systemctl" <<'EOF'
#!/bin/bash
f="$FACTORYOS_ROOT/.fake"
echo "systemctl $*" >> "$f/calls"
case "$1" in
  is-active)
    shift; quiet=no; [ "$1" = --quiet ] && { quiet=yes; shift; }
    s=$(cat "$f/units/$1" 2>/dev/null || echo inactive)
    [ "$quiet" = yes ] || echo "$s"
    [ "$s" = active ] ;;
  is-system-running) cat "$f/system-state" 2>/dev/null || echo running ;;
  # A unit that is not installed (no .fake/units/<unit> and no .fake/installed/<unit>): restart and try-restart fail
  # as systemd's do ("Unit ... not found", exit 5) when .fake/strict-units exists; the first-install case sets it.
  restart|try-restart)
    u="${2%.service}.service"
    if [ -e "$f/strict-units" ] && [ ! -e "$f/units/$u" ] && [ ! -e "$f/units/${u%.service}" ]; then
      echo "Failed to $1 $u: Unit $u not found." >&2; exit 5
    fi ;;
  *) exit 0 ;;
esac
EOF
# curl: /api/v1/health is healthy unless current points at a release listed in .fake/unhealthy.
cat > "$stubs/curl" <<'EOF'
#!/bin/bash
f="$FACTORYOS_ROOT/.fake"
echo "curl $*" >> "$f/calls"
cur=$(basename "$(readlink "$FACTORYOS_ROOT/opt/factoryos/current" 2>/dev/null)")
grep -qxF "$cur" "$f/unhealthy" 2>/dev/null && { echo "curl: (22) The requested URL returned error: 503" >&2; exit 22; }
exit 0
EOF
chmod 0755 "$stubs"/*
export PATH="$stubs:$sandbox"
# A belt to the stubs' braces: no real AWS call can authenticate from here.
export AWS_CONFIG_FILE=/nonexistent AWS_SHARED_CREDENTIALS_FILE=/nonexistent AWS_EC2_METADATA_DISABLED=true
unset AWS_PROFILE AWS_ACCESS_KEY_ID AWS_SECRET_ACCESS_KEY AWS_SESSION_TOKEN
[ "$(command -v aws)" = "$stubs/aws" ] || { echo "FAIL aws is not the stub: $(command -v aws)"; exit 1; }

# --- the export (the real handoff-side files, stand-ins for the build, package and deploy)
# make_export <dir> <version> <layout>: flat (everything at the root, the internal repo's layout) or guidance
# (deployment/ at the root, everything else under source/, VERSION.json in source/: the public export's layout).
make_export() {
  local d=$1 t
  case "$3" in
    flat) t=$d ;;
    guidance) t=$d/source ;;
    *) echo "make_export: layout '$3' is not flat|guidance" >&2; exit 2 ;;
  esac
  mkdir -p "$d/deployment/bootstrap" "$t/scripts" "$t/deploy" "$t/cell-planner"
  cp "$boot/factoryos-release.sh" "$boot/factoryos-release.service" "$d/deployment/bootstrap/"
  printf '{"format":1,"release":"%s","tree":"","umarell":""}\n' "$2" > "$t/VERSION.json"
  cat > "$d/deployment/bootstrap/install-build-deps.sh" <<'EOF'
#!/bin/bash
[ "${1:-}" = --stamp ] && { echo deps-stamp-1; exit 0; }
echo "deps" >> "$FACTORYOS_ROOT/.fake/calls"
mkdir -p "$FACTORYOS_ROOT/var/lib/factoryos-release"
echo deps-stamp-1 > "$FACTORYOS_ROOT/var/lib/factoryos-release/deps.sha256"
EOF
  cat > "$t/scripts/build-release.sh" <<'EOF'
#!/bin/bash
f="$FACTORYOS_ROOT/.fake"
echo "build $FACTORYOS_BUILD_ID" >> "$f/calls"
root=$(cd "$(dirname "$0")/.." && pwd)
echo "ran build-release.sh in $root" >> "$f/calls"
mkdir -p "$root/target/release" "$root/cell/target" "$root/plan2cell/target/release" "$root/cell-planner/target"
touch "$root/target/CACHEDIR.TAG" "$root/cell/target/CACHEDIR.TAG" "$root/plan2cell/target/CACHEDIR.TAG" \
  "$root/target/release/factoryos-backend"
if [ -f "$f/build-term" ]; then rm -f "$f/build-term"; kill -TERM "$PPID"; exit 0; fi
if [ -f "$f/build-fail" ]; then
  echo "   Compiling factoryos-backend v0.1.0"
  echo "error[E0308]: mismatched types"
  echo "error: could not compile \`factoryos-backend\` (bin \"factoryos-backend\") due to 1 previous error"
  exit 101
fi
echo "build ok"
EOF
  cat > "$t/scripts/package-bundle.sh" <<'EOF'
#!/bin/bash
root=$(cd "$(dirname "$0")/.." && pwd)
echo "package $1" >> "$FACTORYOS_ROOT/.fake/calls"
echo "ran package-bundle.sh in $root" >> "$FACTORYOS_ROOT/.fake/calls"
s=$(mktemp -d); mkdir -p "$s/bin" "$s/web"
printf '#!/bin/sh\n' > "$s/bin/factoryos-backend"; chmod 0755 "$s/bin/factoryos-backend"
# .fake/ship-jam: a release with umarell_jam (the router's script, the cell planner's package, the pins);
# .fake/jam-pin: another deploy/umarell-jam.pin.
if [ -f "$FACTORYOS_ROOT/.fake/ship-jam" ]; then
  mkdir -p "$s/deploy/planner-router" "$s/cell-planner/umarell_jam"
  : > "$s/deploy/planner-router/umarell_jam.py"; : > "$s/cell-planner/umarell_jam/__main__.py"
  if [ -f "$FACTORYOS_ROOT/.fake/jam-pin" ]; then cp "$FACTORYOS_ROOT/.fake/jam-pin" "$s/deploy/umarell-jam.pin"
  else printf '# umarell_jam base\n76ea13aafdeede10be11936cf0830613e8717907 https://github.com/aws-samples/sample-umarell.git\n' > "$s/deploy/umarell-jam.pin"; fi
  # deploy/toppra.pin's shape: a hash-checking pip requirements file (build requirements, then the sdist).
  printf '# toppra for umarell_jam\nsetuptools==80.9.0 --hash=sha256:%s\ntoppra==0.6.3 --hash=sha256:%s\n' \
    "$(printf '1%.0s' $(seq 1 64))" "$(printf '2%.0s' $(seq 1 64))" > "$s/deploy/toppra.pin"
fi
echo '<html>' > "$s/web/index.html"
echo "build:                  $(python3 -c 'import json; print(json.load(open("'"$root"'/VERSION.json"))["release"])')" > "$s/BUILD-INFO.txt"
tar -czf "$1" -C "$s" . && rm -rf "$s"
EOF
  cat > "$t/deploy/factoryos-deploy.sh" <<'EOF'
#!/bin/bash
f="$FACTORYOS_ROOT/.fake"
echo "deploy $1 $2 public=${FACTORYOS_PUBLIC:-} umarell=${FACTORYOS_UMARELL_TARBALL:-}:${FACTORYOS_UMARELL_SHA256:-} jam=${FACTORYOS_UMARELL_JAM_TARBALL:-}:${FACTORYOS_UMARELL_JAM_SHA256:-}" >> "$f/calls"
echo "ran factoryos-deploy.sh in $(cd "$(dirname "$0")/.." && pwd)" >> "$f/calls"
[ "$(sha256sum "$1" | cut -d' ' -f1)" = "$2" ] || { echo "factoryos-deploy: sha256 mismatch" >&2; exit 1; }
[ -f "$f/deploy-fail-before" ] && { echo "factoryos-deploy: provisioning failed (before the switch)" >&2; exit 1; }
# Like the real factoryos-deploy.sh: umask 022 and the modes normalised after the unpack (GNU tar as non-root
# applies the inherited umask 077 of the test otherwise; test-deploy-modes.sh tests the real script).
umask 022
name=$(basename "$1" .tar.gz)
dir="$FACTORYOS_ROOT/opt/factoryos/releases/$name"
rm -rf "$dir.new" && mkdir -p "$dir.new" && tar -xzf "$1" -C "$dir.new" && chmod -R u=rwX,go=rX "$dir.new" \
  && rm -rf "$dir" && mv "$dir.new" "$dir"
ln -sfn "$dir" "$FACTORYOS_ROOT/opt/factoryos/current"
echo "FACTORY_OS_RELEASE=${name#factoryos-viewer-linux-arm64-}" > "$FACTORYOS_ROOT/etc/factoryos/release.env"
# Provisioning starts the list of what it could not set up anew; .fake/provision-incomplete: what it lists.
rm -f "$FACTORYOS_ROOT/etc/factoryos/provision-incomplete"
if [ -f "$f/provision-incomplete" ]; then cp "$f/provision-incomplete" "$FACTORYOS_ROOT/etc/factoryos/provision-incomplete"; fi
echo provisioned > "$FACTORYOS_ROOT/etc/factoryos/provisioned.sha256"
systemctl restart factoryos-backend
[ -f "$f/deploy-fail-after" ] && { echo "factoryos-deploy: backend did not become healthy" >&2; exit 1; }
# .fake/deploy-output: what a real deploy wrote before it failed (after the switch).
[ -f "$f/deploy-output" ] && { cat "$f/deploy-output" >&2; exit 1; }
echo "factoryos-deploy: $name is up"
EOF
  chmod 0755 "$d"/deployment/bootstrap/*.sh "$t"/scripts/*.sh "$t"/deploy/*.sh
}
# use_layout <flat|guidance>: the layout publish exports; $S is the build root below src/<version> ("" or /source).
use_layout() {
  layout=$1
  case "$1" in flat) S="" ;; guidance) S=/source ;; esac
}
use_layout guidance
# ran <version>: where the build, package and deploy scripts ran (the stand-ins log their build root).
ran() {
  local d="$R/opt/factoryos/src/$1$S"
  printf 'ran build-release.sh in %s ran package-bundle.sh in %s ran factoryos-deploy.sh in %s' "$d" "$d" "$d"
}

# publish <version> [zip] [force] [umarell] [jam|jam-bad]: the export (in $layout) into the fake source bucket and
# `wanted` pointing at it (jam: with a umarell_jam tarball; jam-bad: one whose sha256 in wanted is wrong).
publish() {
  local v=$1 fmt=${2:-tar} force=${3:-false} um=${4:-} jam=${5:-} t="$work/exports/$1" key ext uj=null jj=null
  rm -rf "$t" && mkdir -p "$t"
  make_export "$t/factoryos-$v" "$v" "$layout"
  if [ "$fmt" = zip ]; then
    ext=zip; (cd "$t" && python3 -m zipfile -c "export.zip" "factoryos-$v/")
  else
    ext=tar.gz; tar -czf "$t/export.tar.gz" -C "$t" "factoryos-$v"
  fi
  key="releases/$v.$ext"
  mkdir -p "$F/s3/bucket/releases" "$F/s3/bucket/vendor" "$F/ssm"
  cp "$t/export.$ext" "$F/s3/bucket/$key"
  if [ -n "$um" ]; then
    echo "umarell $v" > "$F/s3/bucket/vendor/umarell-$v.tar.gz"
    uj="{\"key\":\"vendor/umarell-$v.tar.gz\",\"sha256\":\"$(sha256sum "$F/s3/bucket/vendor/umarell-$v.tar.gz" | cut -d' ' -f1)\"}"
  fi
  if [ -n "$jam" ]; then
    echo "umarell-jam $v" > "$F/s3/bucket/vendor/umarell-jam-$v.tar.gz"
    jj="{\"key\":\"vendor/umarell-jam-$v.tar.gz\",\"sha256\":\"$(if [ "$jam" = jam-bad ]; then printf 'f%.0s' $(seq 1 64); else sha256sum "$F/s3/bucket/vendor/umarell-jam-$v.tar.gz" | cut -d' ' -f1; fi)\"}"
  fi
  printf '{"version":"%s","key":"%s","sha256":"%s","umarell":%s,"umarellJam":%s,"requestedAt":"2026-11-10T00:00:00Z","force":%s}' \
    "$v" "$key" "$(sha256sum "$t/export.$ext" | cut -d' ' -f1)" "$uj" "$jj" "$force" > "$F/ssm/_factoryos_t_release_wanted"
}
handoff() { bash "$boot/release-handoff.sh" > "$work/out" 2>&1; echo $?; }
release() { bash "$R/usr/local/sbin/factoryos-release" > "$work/out" 2>&1; echo $?; }
state() { python3 -c 'import json, sys; s = json.load(open(sys.argv[1])); print(s.get(sys.argv[2]))' "$F/ssm/_factoryos_t_release_state" "$1"; }
phases() { python3 -c 'import json, sys
print(" ".join(json.loads(l)["phase"] for l in open(sys.argv[1]) if l.strip()))' "$F/ssm/_factoryos_t_release_state.history"; }
current() { basename "$(readlink "$R/opt/factoryos/current")"; }
calls() { cat "$F/calls" 2>/dev/null; }
reset_calls() { rm -f "$F/calls" "$F/ssm/_factoryos_t_release_state.history"; }
data_snapshot() { (cd "$R/var/lib/factoryos" && find . -print | LC_ALL=C sort | while IFS= read -r p; do
  if [ -f "$p" ]; then echo "$p $(sha256sum < "$p" | cut -d' ' -f1)"; else echo "$p"; fi; done); }

# --- the host: stack.env, an instance id, release v1 running, the user's data
R="$work/root"
F="$R/.fake"
export FACTORYOS_ROOT="$R" FACTORYOS_RELEASE_HEALTH_TRIES=3 FACTORYOS_RELEASE_HEALTH_INTERVAL=0
mkdir -p "$R/etc/factoryos" "$R/var/lib/cloud/data" "$F/units" "$R/run" \
         "$R/opt/factoryos/releases/factoryos-viewer-linux-arm64-v1/bin" "$R/var/lib/factoryos/cells/mine" \
         "$R/var/lib/factoryos/umarell/.venv-planner"
printf '%s\n' FACTORYOS_STACK=t AWS_REGION=eu-test-1 AWS_DEFAULT_REGION=eu-test-1 FACTORYOS_SOURCE_BUCKET=bucket FACTORYOS_LOG_GROUP=lg FACTORYOS_PUBLIC=1 \
  > "$R/etc/factoryos/stack.env"
echo i-0123456789abcdef0 > "$R/var/lib/cloud/data/instance-id"
printf '#!/bin/sh\n' > "$R/opt/factoryos/releases/factoryos-viewer-linux-arm64-v1/bin/factoryos-backend"
chmod 0755 "$R/opt/factoryos/releases/factoryos-viewer-linux-arm64-v1/bin/factoryos-backend"
ln -s "$R/opt/factoryos/releases/factoryos-viewer-linux-arm64-v1" "$R/opt/factoryos/current"
echo '{"cellsDir": "/var/lib/factoryos/cells"}' > "$R/var/lib/factoryos/factoryos.config.json"
echo "my plan" > "$R/var/lib/factoryos/cells/mine/plan.json"
echo active > "$F/units/factoryos-planner-router"
echo active > "$F/units/apt-daily.timer"
# The origin secret (the installer's SecureString; the stub prints what `--query Parameter --output json` would), the
# test's own sentinels (grepped for below).
origin_value=ORIGIN-sentinel_0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJK # pragma: allowlist secret (fake)
origin_new=ORIGIN-rotated_0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKL # pragma: allowlist secret (fake)
origin_param() { # origin_param <value> [<type>]
  mkdir -p "$F/ssm"
  printf '{"Name":"/factoryos/t/origin-secret","Type":"%s","Value":"%s","Version":1}\n' "${2:-SecureString}" "$1" \
    > "$F/ssm/_factoryos_t_origin-secret"
}
origin_param "$origin_value"
# The planners' venvs (provision.sh's): a python that logs where and how it ran, and fails for the venvs named in
# .fake/venv-fail (as an import error does).
fake_python() { # fake_python <file> <name>
  mkdir -p "$(dirname "$1")"
  cat > "$1" <<EOF
#!/bin/bash
f="\$FACTORYOS_ROOT/.fake"
echo "venv-check $2 in \$(pwd) PYTHONPATH=\${PYTHONPATH:-} bytecode=\${PYTHONDONTWRITEBYTECODE:-}" >> "\$f/calls"
if grep -qxF "$2" "\$f/venv-fail" 2>/dev/null; then
  echo 'Traceback (most recent call last):' >&2
  echo '  File "<string>", line 2, in <module>' >&2
  echo "ModuleNotFoundError: No module named 'ompl'" >&2
  exit 1
fi
EOF
  chmod 0755 "$1"
}
fake_python "$R/var/lib/factoryos/umarell/.venv-planner/bin/python" planner
fake_python "$R/opt/factoryos-cell-planner/router/venv/bin/python" "cell planner"
mkdir -p "$R/opt/factoryos-cell-planner/router/src"
# umarell_jam's: its venv (which logs the versions it is asked for), the cell planner's pins, the plain 76ea13a.
mkdir -p "$R/opt/factoryos-cell-planner/router/jam-venv/bin"
cat > "$R/opt/factoryos-cell-planner/router/jam-venv/bin/python" <<'EOF'
#!/bin/bash
f="$FACTORYOS_ROOT/.fake"
echo "venv-check umarell_jam in $(pwd) JAM_PIN=${JAM_PIN:-} JAM_TOPPRA=${JAM_TOPPRA:-} bytecode=${PYTHONDONTWRITEBYTECODE:-}" >> "$f/calls"
if grep -qxF umarell_jam "$f/venv-fail" 2>/dev/null; then
  echo 'Traceback (most recent call last):' >&2
  echo "ModuleNotFoundError: No module named 'toppra'" >&2
  exit 1
fi
EOF
chmod 0755 "$R/opt/factoryos-cell-planner/router/jam-venv/bin/python"
printf 'numpy==1.26.4\npin==2.7.0\nscipy==1.13.1\n' > "$R/opt/factoryos-cell-planner/router/src/requirements-umarell.txt"
mkdir -p "$R/var/lib/factoryos/umarell-76ea13a/agents"
echo 76ea13aafdeede10be11936cf0830613e8717907 > "$R/var/lib/factoryos/umarell-76ea13a/.umarell.sha256"
data_before=$(data_snapshot)

echo "== no release requested: the unit at boot does nothing"
check "exit 0" 0 "$(bash "$boot/factoryos-release.sh" > "$work/out" 2>&1; echo $?)"
contains "says so" "nothing to do" "$(cat "$work/out")"

echo "== handoff refuses while a release runs"
publish v2
echo activating > "$F/units/factoryos-release.service"
check "unit activating: exit 1" 1 "$(handoff)"
contains "says why" "a release is running" "$(cat "$work/out")"
check "nothing downloaded" "" "$(calls | grep 's3 cp' || true)"
rm -f "$F/units/factoryos-release.service"
exec 8>"$R/run/factoryos-release.lock"
flock -n 8
check "lock held: exit 1" 1 "$(handoff)"
contains "says why" "lock" "$(cat "$work/out")"
exec 8>&-
reset_calls

echo "== v1 -> v2 (tar.gz): ready"
check "handoff exit 0" 0 "$(handoff)"
check "the repo root directly in src/v2" "yes" "$([ -f "$R/opt/factoryos/src/v2/deployment/bootstrap/factoryos-release.sh" ] && echo yes)"
check "unit installed" "yes" "$(same_file "$boot/factoryos-release.service" "$R/etc/systemd/system/factoryos-release.service" && echo yes)"
check "release script installed" "yes" "$(same_file "$boot/factoryos-release.sh" "$R/usr/local/sbin/factoryos-release" && echo yes)"
contains "unit started without blocking" "systemctl start --no-block factoryos-release.service" "$(calls)"
check "state downloading" downloading "$(state phase)"
check "previous v1" v1 "$(state previous)"
check "release exit 0" 0 "$(release)"
check "phases" "downloading deps building packaging deploying checking ready" "$(phases)"
check "ready v2" "ready v2 v1 None" "$(state phase) $(state version) $(state previous) $(state error)"
check "current -> v2" factoryos-viewer-linux-arm64-v2 "$(current)"
check "Guidance layout: the build root src/v2/source" yes "$([ -x "$R/opt/factoryos/src/v2/source/scripts/build-release.sh" ] && [ ! -e "$R/opt/factoryos/src/v2/scripts" ] && echo yes)"
check "build, package and deploy ran from src/v2/source" "$(ran v2)" "$(calls | grep '^ran ' | tr '\n' ' ' | sed 's/ $//')"
check "cargo's target dirs deleted (source/target, source/cell/target, source/plan2cell/target)" \
  "./source/cell-planner/target" "$(cd "$R/opt/factoryos/src/v2" && find . -name target -type d)"
check "a target-named dir without CACHEDIR.TAG kept" yes "$([ -d "$R/opt/factoryos/src/v2/source/cell-planner/target" ] && echo yes)"
contains "the planner venv checked in umarell's dir, umarell on the path, no bytecode" \
  "venv-check planner in $R/var/lib/factoryos/umarell PYTHONPATH=$R/var/lib/factoryos/umarell bytecode=1" "$(calls)"
contains "the cell planner venv checked in its tree" \
  "venv-check cell planner in $R/opt/factoryos-cell-planner/router/src PYTHONPATH=. bytecode=1" "$(calls)"
check "venvs checked after the health check" "curl venv-check venv-check" \
  "$(calls | grep -E '^(curl|venv-check)' | cut -d' ' -f1 | tail -3 | tr '\n' ' ' | sed 's/ $//')"
if [ "$(id -u)" = 0 ]; then
  check "as root: both venvs checked as the factoryos user" 2 "$(calls | grep -c '^sudo -u factoryos -H ')"
else
  check "not root: the venvs checked as this user (no sudo)" 0 "$(calls | grep -c '^sudo ')"
fi
b="$R/opt/factoryos/bundles/factoryos-viewer-linux-arm64-v2.tar.gz"
check "bundle sha256 recorded" "$(sha256sum "$b" | cut -d' ' -f1)" "$(cat "$b.sha256")"
contains "deploy: local bundle, its sha256, public mode" "deploy $b $(cat "$b.sha256") public=1 umarell=:" "$(calls)"
contains "apt timer held" "systemctl stop apt-daily.timer" "$(calls)"
contains "apt timer restored" "systemctl start apt-daily.timer" "$(calls)"
check "needrestart never restarts a FactoryOS unit (conf.d)" "\$nrconf{override_rc}{qr/^(factoryos|fos)-/} = 0;" \
  "$(grep -v '^#' "$R/etc/needrestart/conf.d/90-factoryos.conf" 2>/dev/null)"
check "the unit refuses a manual stop or restart" RefuseManualStop=yes \
  "$(grep -x 'RefuseManualStop=yes' "$R/etc/systemd/system/factoryos-release.service")"
check "deps ran once" 1 "$(calls | grep -cx deps)"
check "log stream" "i-0123456789abcdef0/release.log" "$(state logStream)"
contains "release.log has a header" "factoryos release v2 (previous v1" "$(cat "$R/var/log/factoryos/release.log")"
reset_calls

# The first real test install (2611122014-dc89): the stack's SSM document runs the handoff under umask 077, its
# `mkdir -p /opt/factoryos/src` made /opt/factoryos 0700, and the backend (User=factoryos) could not execute
# /opt/factoryos/current/bin/factoryos-backend (203/EXEC "Permission denied"). Both scripts under umask 077, on the
# 0700 /opt/factoryos and releases/ that instance was left with: repaired to 0755, the release reachable by others.
echo "== under the SSM document's umask 077, on a 0700 /opt/factoryos: 0755 again, the release reachable"
mode() { python3 -I -c 'import os, sys; print(" ".join(oct(os.stat(p).st_mode & 0o777)[2:] for p in sys.argv[1:]))' "$@"; }
chmod 0700 "$R/opt/factoryos" "$R/opt/factoryos/releases"
publish v2 tar true
check "handoff exit 0" 0 "$(umask 077; handoff)"
check "after the handoff: /opt/factoryos 0755" 755 "$(mode "$R/opt/factoryos")"
check "release exit 0" 0 "$(umask 077; release)"
check "ready v2 (forced)" "ready v2" "$(state phase) $(state version)"
check "/opt/factoryos, releases/ 0755" "755 755" "$(mode "$R/opt/factoryos" "$R/opt/factoryos/releases")"
check "every directory from /opt to current's bin/ searchable by others" "" \
  "$(d="$R/opt/factoryos/current/bin"; while [ "$d" != "$R" ]; do
       [ $(( 0$(mode "$d") & 1 )) = 1 ] || echo "$d $(mode "$d")"; d=$(dirname "$d"); done)"
reset_calls

echo "== same version, not forced: already installed"
publish v2
check "handoff exit 0" 0 "$(handoff)"
contains "says so" "already installed" "$(cat "$work/out")"
check "nothing downloaded" "" "$(calls | grep 's3 cp' || true)"
check "state ready v2" "ready v2" "$(state phase) $(state version)"
reset_calls

echo "== v2 -> v3 (zip, umarell tarball): ready, pruned to two releases"
if command -v unzip >/dev/null 2>&1; then
  publish v3 zip false umarell
  check "handoff exit 0" 0 "$(handoff)"
  check "the repo root directly in src/v3" "yes" "$([ -x "$R/opt/factoryos/src/v3$S/scripts/build-release.sh" ] && echo yes)"
  check "umarell tarball" "umarell v3" "$(cat "$R/opt/factoryos/src/v3.umarell.tar.gz")"
  check "release exit 0" 0 "$(release)"
  check "ready v3" "ready v3 v2" "$(state phase) $(state version) $(state previous)"
  check "ran from src/v3/source" "$(ran v3)" "$(calls | grep '^ran ' | tr '\n' ' ' | sed 's/ $//')"
  contains "deploy gets the umarell tarball" "umarell=$R/opt/factoryos/src/v3.umarell.tar.gz:$(sha256sum "$F/s3/bucket/vendor/umarell-v3.tar.gz" | cut -d' ' -f1)" "$(calls)"
  check "deps skipped (stamp)" 0 "$(calls | grep -cx deps)"
  check "releases kept: v2 v3" "factoryos-viewer-linux-arm64-v2 factoryos-viewer-linux-arm64-v3" "$(ls "$R/opt/factoryos/releases" | tr '\n' ' ' | sed 's/ $//')"
  check "exports kept: v2 v3" "v2 v3 v3.umarell.tar.gz" "$(ls "$R/opt/factoryos/src" | tr '\n' ' ' | sed 's/ $//')"
else
  echo "skip: no unzip (the zip case); v3 as tar.gz"
  publish v3 tar false umarell
  handoff >/dev/null; release >/dev/null
  check "ready v3" "ready v3 v2" "$(state phase) $(state version) $(state previous)"
fi
reset_calls

echo "== sha256 mismatch: failed, current untouched"
publish v4
python3 - "$F/ssm/_factoryos_t_release_wanted" <<'PY'
import json, sys
w = json.load(open(sys.argv[1])); w["sha256"] = "0" * 64; json.dump(w, open(sys.argv[1], "w"))
PY
check "handoff exit 1" 1 "$(handoff)"
check "failed v4" "failed v4" "$(state phase) $(state version)"
contains "error names the sha256" "downloading: releases/v4.tar.gz: sha256" "$(state error)"
check "current still v3" factoryos-viewer-linux-arm64-v3 "$(current)"
check "unit not started" "" "$(calls | grep 'start --no-block' || true)"
reset_calls

echo "== build failure: failed, current untouched"
publish v4
touch "$F/build-fail"
handoff >/dev/null
check "release exit 1" 1 "$(release)"
rm -f "$F/build-fail"
check "failed v4" "failed v4 v3" "$(state phase) $(state version) $(state previous)"
contains "error: the compiler's last error" "building: build-release.sh exited 101: error: could not compile \`factoryos-backend\`" "$(state error)"
check "current still v3" factoryos-viewer-linux-arm64-v3 "$(current)"
check "no deploy" "" "$(calls | grep '^deploy' || true)"
check "targets kept (a retry builds incrementally)" yes "$([ -f "$R/opt/factoryos/src/v4$S/target/CACHEDIR.TAG" ] && echo yes)"
check "the build ran from src/v4/source" "ran build-release.sh in $R/opt/factoryos/src/v4$S" "$(calls | grep '^ran ')"
check "finishedAt set" yes "$([ "$(state finishedAt)" != None ] && echo yes)"
reset_calls

echo "== health failure after the switch: rolled back"
publish v5
echo factoryos-viewer-linux-arm64-v5 > "$F/unhealthy"
handoff >/dev/null
check "release exit 1" 1 "$(release)"
check "rolled-back v5, previous v3" "rolled-back v5 v3" "$(state phase) $(state version) $(state previous)"
contains "error" "checking: GET /api/v1/health failed; rolled back to v3" "$(state error)"
check "current back at v3" factoryos-viewer-linux-arm64-v3 "$(current)"
check "release.env back" FACTORY_OS_RELEASE=v3 "$(cat "$R/etc/factoryos/release.env")"
check "provision stamp dropped" no "$([ -f "$R/etc/factoryos/provisioned.sha256" ] && echo yes || echo no)"
check "backend restarted twice (deploy, rollback)" 2 "$(calls | grep -c 'systemctl restart factoryos-backend')"
rm -f "$F/unhealthy"
reset_calls

echo "== router down after the switch: rolled back"
publish v5
echo inactive > "$F/units/factoryos-planner-router"
handoff >/dev/null
check "release exit 1" 1 "$(release)"
contains "error" "checking: factoryos-planner-router is not active" "$(state error)"
check "current back at v3" factoryos-viewer-linux-arm64-v3 "$(current)"
echo active > "$F/units/factoryos-planner-router"
reset_calls

echo "== the planner venv does not import after the switch: rolled back"
publish v5
echo planner > "$F/venv-fail"
handoff >/dev/null
check "release exit 1" 1 "$(release)"
rm -f "$F/venv-fail"
check "rolled-back v5, previous v3" "rolled-back v5 v3" "$(state phase) $(state version) $(state previous)"
check "error: the import error, rolled back" \
  "checking: the planner venv does not import (exit 1): ModuleNotFoundError: No module named 'ompl'; rolled back to v3" \
  "$(state error)"
check "current back at v3" factoryos-viewer-linux-arm64-v3 "$(current)"
check "the cell planner's not checked after the planner's failed" "" "$(calls | grep 'venv-check cell planner' || true)"
check "the import check ran once" 1 "$(calls | grep -c '^venv-check planner')"
reset_calls

echo "== the cell planner venv is missing after the switch: rolled back"
publish v5
mv "$R/opt/factoryos-cell-planner/router/venv" "$work/cell-planner-venv"
handoff >/dev/null
check "release exit 1" 1 "$(release)"
mv "$work/cell-planner-venv" "$R/opt/factoryos-cell-planner/router/venv"
contains "error" "checking: the cell planner venv is missing ($R/opt/factoryos-cell-planner/router/venv/bin/python" "$(state error)"
check "rolled-back v5" "rolled-back v5" "$(state phase) $(state version)"
check "current back at v3" factoryos-viewer-linux-arm64-v3 "$(current)"
reset_calls

echo "== deploy failure before the switch: failed"
publish v6
touch "$F/deploy-fail-before"
handoff >/dev/null
check "release exit 1" 1 "$(release)"
rm -f "$F/deploy-fail-before"
check "failed v6" "failed v6" "$(state phase) $(state version)"
contains "error" "deploying: factoryos-deploy.sh exited 1: factoryos-deploy: provisioning failed" "$(state error)"
check "current still v3" factoryos-viewer-linux-arm64-v3 "$(current)"
reset_calls

echo "== deploy failure after the switch: rolled back"
publish v6
touch "$F/deploy-fail-after"
handoff >/dev/null
check "release exit 1" 1 "$(release)"
rm -f "$F/deploy-fail-after"
check "rolled-back v6" "rolled-back v6" "$(state phase) $(state version)"
check "current back at v3" factoryos-viewer-linux-arm64-v3 "$(current)"
reset_calls

echo "== the lock: a second run does not start"
publish v7
handoff >/dev/null
exec 8>"$R/run/factoryos-release.lock"
flock -n 8
check "release exit 1" 1 "$(release)"
contains "says why" "holds" "$(cat "$work/out")"
check "state untouched" downloading "$(state phase)"
exec 8>&-

echo "== a shutdown during the build keeps the phase; the boot resumes it"
touch "$F/build-term"
echo stopping > "$F/system-state"
check "release exit 143" 143 "$(release)"
check "still building v7" "building v7" "$(state phase) $(state version)"
check "current still v3" factoryos-viewer-linux-arm64-v3 "$(current)"
echo running > "$F/system-state"
reset_calls
check "boot: release exit 0" 0 "$(release)"
check "resumed from building" "building packaging deploying checking ready" "$(phases)"
check "ready v7, previous v3" "ready v7 v3" "$(state phase) $(state version) $(state previous)"
reset_calls

echo "== a terminal state at boot: nothing to do"
check "release exit 0" 0 "$(release)"
check "no state written" "" "$(calls | grep put-parameter || true)"

echo "== SIGTERM by hand: failed"
publish v8
touch "$F/build-term"
handoff >/dev/null
check "release exit 1" 1 "$(release)"
check "failed v8" "failed v8" "$(state phase) $(state version)"
contains "error" "building: stopped (SIGTERM, not a shutdown)" "$(state error)"
reset_calls

echo "== an interrupted download at boot: failed"
publish v8
handoff >/dev/null
rm -f "$R/opt/factoryos/src/v8/.factoryos-export"
check "release exit 1" 1 "$(release)"
contains "error" "downloading: interrupted before the export was unpacked" "$(state error)"
check "current still v7" factoryos-viewer-linux-arm64-v7 "$(current)"
reset_calls

echo "== forced rebuild of the running version, unhealthy: the running build is restored"
echo "the v7 that ran" > "$R/opt/factoryos/releases/factoryos-viewer-linux-arm64-v7/OLD"
publish v7 tar true
echo factoryos-viewer-linux-arm64-v7 > "$F/unhealthy"
check "handoff exit 0" 0 "$(handoff)"
contains "downloaded again" "s3 cp s3://bucket/releases/v7.tar.gz" "$(calls)"
rm -f "$F/unhealthy"   # the stand-in health check cannot tell the two v7 builds apart: the rollback's check passes
cp "$R/opt/factoryos/src/v7$S/deploy/factoryos-deploy.sh" "$work/deploy.sh"
cat > "$R/opt/factoryos/src/v7$S/deploy/factoryos-deploy.sh" <<EOF
#!/bin/bash
bash "$work/deploy.sh" "\$@" || exit 1
echo "factoryos-viewer-linux-arm64-v7" > "\$FACTORYOS_ROOT/.fake/unhealthy-next"
EOF
# The new v7 is unhealthy only until the rollback has restored the old one.
cat > "$stubs/curl" <<'EOF'
#!/bin/bash
f="$FACTORYOS_ROOT/.fake"
echo "curl $*" >> "$f/calls"
[ -f "$FACTORYOS_ROOT/opt/factoryos/current/OLD" ] && exit 0
[ -f "$f/unhealthy-next" ] && exit 22
exit 0
EOF
check "release exit 1" 1 "$(release)"
check "rolled-back v7, previous v7" "rolled-back v7 v7" "$(state phase) $(state version) $(state previous)"
check "the running build is back" "the v7 that ran" "$(cat "$R/opt/factoryos/current/OLD" 2>/dev/null)"
check "no rollback copy left" "" "$(find "$R/opt/factoryos/releases" -maxdepth 1 -name '.rollback-*')"
rm -f "$F/unhealthy-next"
reset_calls

echo "== the Kiro key: kiro.env 0600, the value nowhere else"
mkdir -p "$F/secrets"
# Fake Kiro keys, the test's own sentinels (grepped for below), no key of anyone's.
kiro_value=kiro-SECRET-value-1234567 # pragma: allowlist secret (a fake key, see above)
kiro_json=kiro-JSON-value-7654321 # pragma: allowlist secret (a fake key, see above)
printf '%s' "$kiro_value" > "$F/secrets/kiro-key"
publish v9
# The release document exports the stack's settings; the handoff keeps them in stack.env.
check "handoff exit 0" 0 "$(FACTORYOS_KIRO_SECRET=kiro-key FACTORYOS_BEDROCK=1 handoff)"
check "stack.env: the settings added, the other keys kept" \
  "FACTORYOS_STACK=t AWS_REGION=eu-test-1 AWS_DEFAULT_REGION=eu-test-1 FACTORYOS_SOURCE_BUCKET=bucket FACTORYOS_LOG_GROUP=lg FACTORYOS_PUBLIC=1 FACTORYOS_KIRO_SECRET=kiro-key FACTORYOS_BEDROCK=1" \
  "$(tr '\n' ' ' < "$R/etc/factoryos/stack.env" | sed 's/ $//')"
check "release exit 0" 0 "$(release)"
check "kiro.env" "KIRO_API_KEY=$kiro_value" "$(cat "$R/etc/factoryos/kiro.env")"
check "kiro.env mode 0600" 600 "$(python3 -c 'import os, sys; print(oct(os.stat(sys.argv[1]).st_mode & 0o777)[2:])' "$R/etc/factoryos/kiro.env")"
check "the value in no log, call or state" "" "$(grep -rl 'kiro-SECRET-value' "$R/var/log" "$F/calls" "$F/ssm" "$work/out" 2>/dev/null || true)"
printf '{"KIRO_API_KEY": "%s"}' "$kiro_json" > "$F/secrets/kiro-key"
publish v9 tar true
handoff >/dev/null; release >/dev/null
check "a JSON secret" "KIRO_API_KEY=$kiro_json" "$(cat "$R/etc/factoryos/kiro.env")"
printf 'not a key with spaces' > "$F/secrets/kiro-key"
publish v10
handoff >/dev/null
check "a malformed secret: exit 1" 1 "$(release)"
contains "error" "cannot read the Kiro key from secret kiro-key: the secret is not a key" "$(state error)"
check "...without its value" "" "$(grep -rl 'not a key with spaces' "$R/var/log" "$F/calls" "$F/ssm" "$work/out" 2>/dev/null || true)"
check "current still v9" factoryos-viewer-linux-arm64-v9 "$(current)"
publish v10
check "a settings-less handoff keeps them" 0 "$(handoff)"
contains "...in stack.env" "FACTORYOS_KIRO_SECRET=kiro-key" "$(cat "$R/etc/factoryos/stack.env")"
check "an empty secret name removes it" 0 "$(FACTORYOS_KIRO_SECRET='' FACTORYOS_BEDROCK=0 handoff)"
check "...from stack.env" "FACTORYOS_PUBLIC=1 FACTORYOS_BEDROCK=0" "$(tail -2 "$R/etc/factoryos/stack.env" | tr '\n' ' ' | sed 's/ $//')"
check "a bad setting: exit 1" 1 "$(FACTORYOS_BEDROCK=yes handoff)"
check "release exit 0 (no key)" 0 "$(release)"
check "kiro.env removed" no "$([ -e "$R/etc/factoryos/kiro.env" ] && echo yes || echo no)"
reset_calls

echo "== the origin secret: origin.env 0600 from the SecureString, nowhere else"
o="$R/etc/factoryos/origin.env"
mode1() { python3 -c 'import os, sys; print(oct(os.stat(sys.argv[1]).st_mode & 0o777)[2:])' "$1"; }
check "origin.env (written by the handoffs above)" "FACTORY_OS_ORIGIN_SECRET=$origin_value" "$(cat "$o")"
check "origin.env mode 0600" 600 "$(mode1 "$o")"
publish v10 tar true
check "handoff exit 0" 0 "$(handoff)"
contains "read with decryption" "ssm get-parameter --region eu-test-1 --name /factoryos/t/origin-secret --with-decryption --query Parameter --output json" "$(calls)"
contains "unchanged: says so" "origin secret: $o is current" "$(cat "$work/out")"
check "unchanged: the backend not restarted" "" "$(calls | grep -E '(try-)?restart factoryos-backend' || true)"
check "release exit 0" 0 "$(release)"
reset_calls
# The installer's rotation, step 1: both values, a secretsOnly request (the wanted fields kept, as the installer does).
origin_param "$origin_new,$origin_value"
cp "$F/ssm/_factoryos_t_release_wanted" "$work/wanted.keep"
python3 -c 'import json, sys; w = json.load(open(sys.argv[1])); w["secretsOnly"] = True; print(json.dumps(w))' \
  "$work/wanted.keep" > "$F/ssm/_factoryos_t_release_wanted"
cp "$F/ssm/_factoryos_t_release_state" "$work/state.keep"
check "secretsOnly: handoff exit 0" 0 "$(handoff)"
check "secretsOnly: both values" "FACTORY_OS_ORIGIN_SECRET=$origin_new,$origin_value" "$(cat "$o")"
check "secretsOnly: still 0600" 600 "$(mode1 "$o")"
check "secretsOnly: the backend not running: not restarted" "" "$(calls | grep -E 'restart factoryos-backend' || true)"
contains "secretsOnly: ...and says so" "the backend is not running" "$(cat "$work/out")"
contains "secretsOnly: says so" "secrets only" "$(cat "$work/out")"
check "secretsOnly: the state untouched" yes "$(same_file "$work/state.keep" "$F/ssm/_factoryos_t_release_state" && echo yes)"
check "secretsOnly: nothing downloaded, no unit started" "" "$(calls | grep -E 's3 cp|start --no-block|put-parameter' || true)"
check "secretsOnly: current still v10" factoryos-viewer-linux-arm64-v10 "$(current)"
reset_calls
# Step 3: the new value alone.
origin_param "$origin_new"
check "the new value alone: exit 0" 0 "$(handoff)"
check "...in origin.env" "FACTORY_OS_ORIGIN_SECRET=$origin_new" "$(cat "$o")"
# A first install: the backend unit is not installed yet, and systemd fails try-restart/restart of a missing unit
# (test install rf-new2: "Failed to try-restart factoryos-backend.service: Unit factoryos-backend.service not found").
touch "$F/strict-units"; rm -f "$F/units/factoryos-backend.service" "$F/units/factoryos-backend"
origin_param "$origin_value"
check "first install (no backend unit), a new value: exit 0" 0 "$(handoff)"
check "...not restarted" "" "$(calls | grep -E 'restart factoryos-backend' || true)"
contains "...says the deploy starts it" "the backend is not running (first install)" "$(cat "$work/out")"
reset_calls
# A running backend: restarted with the new value.
echo active > "$F/units/factoryos-backend.service"
origin_param "$origin_new"
check "running backend, a new value: exit 0" 0 "$(handoff)"
contains "...restarted" "systemctl restart factoryos-backend.service" "$(calls)"
rm -f "$F/strict-units" "$F/units/factoryos-backend.service"
cp "$work/wanted.keep" "$F/ssm/_factoryos_t_release_wanted"
reset_calls
# Refusals: before anything else (no download, no state), origin.env as it was, the value never shown.
before=$(cat "$o")
rm -f "$F/ssm/_factoryos_t_origin-secret"
check "missing: exit 1" 1 "$(handoff)"
contains "missing: names the parameter" "cannot read the origin secret /factoryos/t/origin-secret" "$(cat "$work/out")"
origin_param "$origin_value" String
check "a String parameter: exit 1" 1 "$(handoff)"
contains "...says so" "not a SecureString" "$(cat "$work/out")"
origin_param "ORIGIN-malformed with spaces 0123456789abcdefghijklmnop"
check "malformed: exit 1" 1 "$(handoff)"
contains "...says so" "not one or two comma-separated base64url values" "$(cat "$work/out")"
origin_param "$origin_new,$origin_value,$origin_new"
check "three values: exit 1" 1 "$(handoff)"
check "refused: origin.env as it was" "$before" "$(cat "$o")"
check "refused: no origin.env.new left" no "$([ -e "$o.new" ] && echo yes || echo no)"
check "refused: nothing downloaded, no state written" "" "$(calls | grep -E 's3 cp|put-parameter' || true)"
check "refused: the malformed value not shown" "" "$(grep -rl 'ORIGIN-malformed' "$R/var/log" "$F/calls" "$work/out" 2>/dev/null || true)"
origin_param "$origin_value"
check "the document's FACTORYOS_ORIGIN_SECRET_PARAMETER (the stack's name): exit 0" 0 "$(FACTORYOS_ORIGIN_SECRET_PARAMETER=/factoryos/t/origin-secret handoff)"
check "another parameter: refused before it is read" 1 "$(FACTORYOS_ORIGIN_SECRET_PARAMETER=/factoryos/other/origin-secret handoff)"
contains "...says so" "FACTORYOS_ORIGIN_SECRET_PARAMETER is not /factoryos/t/origin-secret" "$(cat "$work/out")"
check "valid again: exit 0" 0 "$(handoff)"
check "release exit 0" 0 "$(release)"
check "the values in no log, call, state or output" "" \
  "$(grep -rlE 'ORIGIN-(sentinel|rotated)' "$R/var/log" "$F/calls" "$F/all-states" "$F/ssm/_factoryos_t_release_state" \
       "$F/ssm/_factoryos_t_release_state.history" "$R/var/lib/factoryos-release" "$R/etc/factoryos/stack.env" "$work/out" 2>/dev/null || true)"
reset_calls

echo "== docker on PATH: failed"
printf '#!/bin/sh\n' > "$stubs/docker" && chmod 0755 "$stubs/docker"
publish v11
handoff >/dev/null
check "release exit 1" 1 "$(release)"
contains "error" "docker is on PATH" "$(state error)"
check "current still v10" factoryos-viewer-linux-arm64-v10 "$(current)"
rm -f "$stubs/docker"
reset_calls

echo "== the flat layout (all at the export's root): ready"
use_layout flat
publish v12
check "handoff exit 0" 0 "$(handoff)"
check "the build root is src/v12" yes "$([ -x "$R/opt/factoryos/src/v12/scripts/build-release.sh" ] && [ ! -e "$R/opt/factoryos/src/v12/source" ] && echo yes)"
check "release exit 0" 0 "$(release)"
check "ready v12, previous v10" "ready v12 v10" "$(state phase) $(state version) $(state previous)"
check "build, package and deploy ran from src/v12" "$(ran v12)" "$(calls | grep '^ran ' | tr '\n' ' ' | sed 's/ $//')"
check "cargo's target dirs deleted (target, cell/target, plan2cell/target)" "./cell-planner/target" \
  "$(cd "$R/opt/factoryos/src/v12" && find . -name target -type d)"
check "BUILD-INFO from the flat VERSION.json" yes "$(grep -q '^build: *v12$' "$R/opt/factoryos/current/BUILD-INFO.txt" && echo yes)"
check "the venvs checked" 2 "$(calls | grep -c '^venv-check')"
use_layout guidance
reset_calls

echo "== the cell planner disabled (workspace.env): its venv is not checked"
echo 'FACTORY_OS_CELL_PLANNER=0' > "$R/etc/factoryos/workspace.env"
echo "cell planner" > "$F/venv-fail"
publish v13
handoff >/dev/null
check "release exit 0" 0 "$(release)"
check "ready v13" "ready v13 v12" "$(state phase) $(state version) $(state previous)"
check "only the planner venv checked" "planner" "$(calls | grep '^venv-check' | cut -d' ' -f2 | tr '\n' ' ' | sed 's/ $//')"
check "BUILD-INFO from source/VERSION.json" yes "$(grep -q '^build: *v13$' "$R/opt/factoryos/current/BUILD-INFO.txt" && echo yes)"
rm -f "$R/etc/factoryos/workspace.env" "$F/venv-fail"
reset_calls

# --- umarell_jam: optional, reported, never a rollback (the user's decision of 2026-11-15)
log_since() { tail -c +"$(( $1 + 1 ))" "$R/var/log/factoryos/release.log"; }
jtar_of() { echo "$R/opt/factoryos/src/$1.umarell-jam.tar.gz"; }
inc_file="$R/etc/factoryos/provision-incomplete"
touch "$F/ship-jam"

echo "== umarell_jam shipped, its tarball carried, set up: checked after the cell planner, ready, nothing incomplete"
publish v13a tar false "" jam
check "handoff exit 0" 0 "$(handoff)"
check "the umarell_jam tarball downloaded next to the export" "umarell-jam v13a" "$(cat "$(jtar_of v13a)" 2>/dev/null)"
check "request.json: the tarball's path and sha256" "$(jtar_of v13a) $(sha256sum "$F/s3/bucket/vendor/umarell-jam-v13a.tar.gz" | cut -d' ' -f1)" \
  "$(python3 -c 'import json, sys; j = json.load(open(sys.argv[1]))["umarellJam"]; print(j["path"], j["sha256"])' "$R/var/lib/factoryos-release/request.json")"
off=$(wc -c < "$R/var/log/factoryos/release.log" | tr -d ' ')
check "release exit 0" 0 "$(release)"
check "ready v13a, previous v13" "ready v13a v13" "$(state phase) $(state version) $(state previous)"
check "nothing incomplete" None "$(state incomplete)"
contains "deploy gets the umarell_jam tarball and its sha256" \
  "jam=$(jtar_of v13a):$(sha256sum "$F/s3/bucket/vendor/umarell-jam-v13a.tar.gz" | cut -d' ' -f1)" "$(calls)"
check "the venvs checked: the planner's, the cell planner's, then umarell_jam's" "planner cell umarell_jam" \
  "$(calls | grep '^venv-check' | cut -d' ' -f2 | tr '\n' ' ' | sed 's/ $//')"
contains "umarell_jam's venv in the cell planner's tree, the pins its cache keys need, no bytecode" \
  "venv-check umarell_jam in $R/opt/factoryos-cell-planner/router/src JAM_PIN=2.7.0 JAM_TOPPRA=0.6.3 bytecode=1" "$(calls)"
contains "the release log says it is set up" "checking: umarell_jam is set up (its venv, umarell 76ea13a)" "$(log_since "$off")"
check "no provision-incomplete" no "$([ -e "$inc_file" ] && echo yes || echo no)"
reset_calls

echo "== umarell_jam's venv does not import: ready all the same, umarell_jam incomplete, the stamp dropped, no rollback"
echo umarell_jam > "$F/venv-fail"
publish v13b
handoff >/dev/null
check "no umarell_jam tarball requested: none kept for v13b" no "$([ -e "$(jtar_of v13b)" ] && echo yes || echo no)"
off=$(wc -c < "$R/var/log/factoryos/release.log" | tr -d ' ')
check "release exit 0" 0 "$(release)"
rm -f "$F/venv-fail"
check "ready v13b (not rolled back)" "ready v13b v13a None" "$(state phase) $(state version) $(state previous) $(state error)"
check "current -> v13b" factoryos-viewer-linux-arm64-v13b "$(current)"
check "the state's incomplete" "['umarell_jam']" "$(state incomplete)"
check "provision-incomplete names umarell_jam" umarell_jam "$(cat "$inc_file" 2>/dev/null)"
check "the provision stamp dropped (the next deploy provisions again)" no "$([ -e "$R/etc/factoryos/provisioned.sha256" ] && echo yes || echo no)"
check "the backend restarted once (the deploy's), no rollback" 1 "$(calls | grep -c 'systemctl restart factoryos-backend')"
contains "the release log's WARNING names the import error" \
  "WARNING: umarell_jam is not set up: the umarell_jam venv does not import (exit 1): ModuleNotFoundError: No module named 'toppra'" "$(log_since "$off")"
contains "the release log's last word: provisioning incomplete" "WARNING: provisioning incomplete: umarell_jam" "$(log_since "$off")"
check "exports kept: v13a with its umarell_jam tarball, v13b" "v13a v13a.umarell-jam.tar.gz v13b" \
  "$(ls "$R/opt/factoryos/src" | grep '^v13[ab]' | tr '\n' ' ' | sed 's/ $//')"
publish v13b
check "the same version again: already installed" 0 "$(handoff)"
check "...and its ready state still says umarell_jam is incomplete" "ready v13b ['umarell_jam']" \
  "$(state phase) $(state version) $(state incomplete)"
reset_calls

echo "== no plain umarell for deploy/umarell-jam.pin: incomplete, the reason names the directory; its venv not run"
echo "1111111111111111111111111111111111111111 https://example.invalid/umarell.git" > "$F/jam-pin"
publish v13c
handoff >/dev/null
off=$(wc -c < "$R/var/log/factoryos/release.log" | tr -d ' ')
check "release exit 0" 0 "$(release)"
rm -f "$F/jam-pin"
check "ready v13c, umarell_jam incomplete" "ready v13c ['umarell_jam']" "$(state phase) $(state version) $(state incomplete)"
contains "the reason" "no plain umarell 1111111 in $R/var/lib/factoryos/umarell-1111111" "$(log_since "$off")"
check "umarell_jam's venv not run" "" "$(calls | grep 'venv-check umarell_jam' || true)"
check "pruned: v13a and its umarell_jam tarball" "v13b v13c" "$(ls "$R/opt/factoryos/src" | grep '^v13' | tr '\n' ' ' | sed 's/ $//')"
reset_calls

echo "== provisioning marked parts incomplete, umarell_jam is set up: the state lists provision.sh's, once each"
printf 'cell-planner\numarell_jam\ncell-planner\n' > "$F/provision-incomplete"
publish v13d
handoff >/dev/null
check "release exit 0" 0 "$(release)"
rm -f "$F/provision-incomplete"
check "ready v13d, incomplete in provision.sh's order" "ready v13d ['cell-planner', 'umarell_jam']" \
  "$(state phase) $(state version) $(state incomplete)"
reset_calls

echo "== the umarell_jam tarball is not what wanted says (sha256): the handoff goes on without it"
publish v13e tar false "" jam-bad
check "handoff exit 0" 0 "$(handoff)"
contains "says so" "WARNING: the umarell_jam tarball s3://bucket/vendor/umarell-jam-v13e.tar.gz cannot be downloaded or is not sha256" "$(cat "$work/out")"
check "no tarball (and no .part) kept" "no no" "$([ -e "$(jtar_of v13e)" ] && echo yes || echo no) $([ -e "$(jtar_of v13e).part" ] && echo yes || echo no)"
check "request.json: no umarellJam" None "$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["umarellJam"])' "$R/var/lib/factoryos-release/request.json")"
check "release exit 0" 0 "$(release)"
check "ready v13e" "ready v13e" "$(state phase) $(state version)"
contains "deploy without the umarell_jam tarball" "jam=:" "$(calls | grep '^deploy ')"
reset_calls

echo "== FACTORY_OS_UMARELL_JAM=0 (workspace.env): umarell_jam is not checked"
echo 'FACTORY_OS_UMARELL_JAM=0' > "$R/etc/factoryos/workspace.env"
echo umarell_jam > "$F/venv-fail"
publish v13f
handoff >/dev/null
off=$(wc -c < "$R/var/log/factoryos/release.log" | tr -d ' ')
check "release exit 0" 0 "$(release)"
check "ready v13f, nothing incomplete" "ready v13f None" "$(state phase) $(state version) $(state incomplete)"
check "umarell_jam's venv not run" "" "$(calls | grep 'venv-check umarell_jam' || true)"
contains "says it is off" "umarell_jam is off on this host" "$(log_since "$off")"
rm -f "$R/etc/factoryos/workspace.env" "$F/venv-fail" "$F/ship-jam"
reset_calls

echo "== a first install whose planner venv does not import: failed (nothing to roll back to)"
rm -f "$R/opt/factoryos/current"
echo planner > "$F/venv-fail"
publish v14
handoff >/dev/null
check "no previous" None "$(state previous)"
check "release exit 1" 1 "$(release)"
rm -f "$F/venv-fail"
check "failed v14" "failed v14" "$(state phase) $(state version)"
check "error" "checking: the planner venv does not import (exit 1): ModuleNotFoundError: No module named 'ompl'; no previous release to roll back to" \
  "$(state error)"
reset_calls

# The deploy's output of the first real test install (2611122014-dc89; shortened: 30 curl lines, 10 restarts): the
# backend's binary was not executable. Its reason is the deploy's own last error and the unit's journal error, not
# provision.sh's informational kiro validate line (which the old summary, the last `error:` line, took).
excerpt() {
  cat <<'EOF'
[provision] packages
[provision] workspace.env (public: workspace default)
[provision] kiro-cli 2.26.1
gzip: stdout: Broken pipe
[provision] planner venv
[provision] kiro agent file: error: You are not logged in, please log in with kiro-cli login
[provision] cell planner venv
[provision] cell planner ok: torch 2.14.1+cpu mujoco 3.15.0 pin 4.1.0
[provision] systemd unit factoryos-backend.service
[provision] systemd unit factoryos-planner-router.service
[provision] done
curl: (7) Failed to connect to 127.0.0.1 port 80 after 0 ms: Couldn't connect to server
curl: (7) Failed to connect to 127.0.0.1 port 80 after 0 ms: Couldn't connect to server
curl: (7) Failed to connect to 127.0.0.1 port 80 after 0 ms: Couldn't connect to server
factoryos-deploy: backend did not become healthy
Nov 12 21:07:31 ip-10-0-2-88 systemd[1]: Started factoryos-backend.service - factoryos-backend (wasm viewer + API + Zenoh over /zenoh on port 80).
Nov 12 21:07:31 ip-10-0-2-88 (-backend)[75583]: factoryos-backend.service: Failed to execute /opt/factoryos/current/bin/factoryos-backend: Permission denied
Nov 12 21:07:31 ip-10-0-2-88 systemd[1]: factoryos-backend.service: Main process exited, code=exited, status=203/EXEC
Nov 12 21:07:31 ip-10-0-2-88 systemd[1]: factoryos-backend.service: Failed with result 'exit-code'.
Nov 12 21:07:34 ip-10-0-2-88 systemd[1]: factoryos-backend.service: Scheduled restart job, restart counter is at 1.
Nov 12 21:08:00 ip-10-0-2-88 systemd[1]: Started factoryos-backend.service - factoryos-backend (wasm viewer + API + Zenoh over /zenoh on port 80).
Nov 12 21:08:00 ip-10-0-2-88 (-backend)[75684]: factoryos-backend.service: Failed to execute /opt/factoryos/current/bin/factoryos-backend: Permission denied
Nov 12 21:08:00 ip-10-0-2-88 systemd[1]: factoryos-backend.service: Main process exited, code=exited, status=203/EXEC
Nov 12 21:08:00 ip-10-0-2-88 systemd[1]: factoryos-backend.service: Failed with result 'exit-code'.
EOF
}
real_reason="factoryos-deploy: backend did not become healthy; factoryos-backend.service: Failed to execute /opt/factoryos/current/bin/factoryos-backend: Permission denied"

echo "== the first real test install's deploy failure (a first install): the deploy's error and the unit's, not kiro's"
rm -f "$R/opt/factoryos/current"
excerpt > "$F/deploy-output"
publish v15
handoff >/dev/null
check "no previous" None "$(state previous)"
check "release exit 1" 1 "$(release)"
check "failed v15" "failed v15" "$(state phase) $(state version)"
check "error: the deploy's own error, then the unit's journal error" \
  "deploying: factoryos-deploy.sh exited 1: $real_reason; no previous release to roll back to" "$(state error)"
check "error: never the kiro validate line" "" "$(state error | grep -i kiro || true)"
reset_calls

echo "== the same after an earlier release: rolled back, the same reason"
rm -f "$F/deploy-output"
publish v16
handoff >/dev/null
check "release exit 0 (v16 ready)" 0 "$(release)"
excerpt > "$F/deploy-output"
publish v17
handoff >/dev/null
check "release exit 1" 1 "$(release)"
check "rolled-back v17, previous v16" "rolled-back v17 v16" "$(state phase) $(state version) $(state previous)"
check "error" "deploying: factoryos-deploy.sh exited 1: $real_reason; rolled back to v16" "$(state error)"
reset_calls

echo "== a deploy whose provisioning failed: its error, then provision.sh's warning (not the kiro line after it)"
cat > "$F/deploy-output" <<'EOF'
[provision] cell planner venv
[provision] WARNING: cell planner setup failed (see above); the planner router offers the umarell planner only
[provision] kiro agent file: WARNING: does not validate: error: unknown field `toolz`
[provision] done
factoryos-deploy: factoryos-viewer-linux-arm64-v18 runs, but its provisioning failed (see above)
EOF
publish v18
handoff >/dev/null
check "release exit 1" 1 "$(release)"
check "error" "deploying: factoryos-deploy.sh exited 1: factoryos-deploy: factoryos-viewer-linux-arm64-v18 runs, but its provisioning failed (see above); [provision] WARNING: cell planner setup failed (see above); the planner router offers the umarell planner only; rolled back to v16" \
  "$(state error)"
rm -f "$F/deploy-output"
reset_calls

echo "== only the kiro line looks like an error: the reason is the step's last line, not kiro's"
printf '%s\n' "[provision] kiro agent file: error: You are not logged in, please log in with kiro-cli login" \
  "[provision] done" "something else went wrong" > "$F/deploy-output"
publish v19
handoff >/dev/null
check "release exit 1" 1 "$(release)"
check "error" "deploying: factoryos-deploy.sh exited 1: something else went wrong; rolled back to v16" "$(state error)"
rm -f "$F/deploy-output"
reset_calls

# --- the first boot: user-data.sh in a root of its own, with stubs for what it changes on the host. The network
# (apt-get, curl) is down for the first .fake/net-down calls ("forever": never up), as behind a new NAT gateway.
echo "== user-data.sh (the first boot)"
U="$work/ud" ustubs="$work/ud-stubs"
mkdir -p "$ustubs"
# tee (user-data.sh's log): the host's (not in the sandbox above).
for d in /usr/bin /bin; do [ -x "$d/tee" ] && { ln -sf "$d/tee" "$ustubs/tee"; break; }; done
# net_down: consumes one call of the outage; true while the network is down.
net_down='n=$(cat "$f/net-down" 2>/dev/null || echo 0); [ "$n" = forever ] && return 0; [ "$n" -gt 0 ] || return 1; echo $(( n - 1 )) > "$f/net-down"'
cat > "$ustubs/apt-get" <<EOF
#!/bin/bash
f="\$FACTORYOS_ROOT/.fake"
echo "apt-get \$*" >> "\$f/calls"
down() { $net_down; }
case " \$* " in
  *" update "*)
    down || exit 0
    echo "W: Failed to fetch http://ports.ubuntu.com/ubuntu-ports/dists/noble/InRelease  Could not connect to ports.ubuntu.com:80 (91.189.91.104), connection timed out" >&2
    case " \$* " in *" --error-on=any "*) echo "E: Some index files failed to download. They have been ignored, or old ones used instead." >&2; exit 100 ;; esac
    echo "W: Some index files failed to download. They have been ignored, or old ones used instead." >&2 ;;
  *".deb "*)
    d="\$FACTORYOS_ROOT/opt/aws/amazon-cloudwatch-agent"; mkdir -p "\$d/bin" "\$d/etc"
    printf '#!/bin/bash\necho "amazon-cloudwatch-agent-ctl \$*" >> "\$FACTORYOS_ROOT/.fake/calls"\n' > "\$d/bin/amazon-cloudwatch-agent-ctl"
    chmod 0755 "\$d/bin/amazon-cloudwatch-agent-ctl" ;;
  *" install "*)
    down || exit 0
    echo "E: Failed to fetch http://ports.ubuntu.com/ubuntu-ports/pool/main/u/unzip/unzip_6.0-28ubuntu4.1_arm64.deb  Could not connect to ports.ubuntu.com:80 (91.189.91.104), connection timed out" >&2
    echo "E: Unable to fetch some archives, maybe run apt-get update or try with --fix-missing?" >&2; exit 100 ;;
esac
EOF
cat > "$ustubs/curl" <<EOF
#!/bin/bash
f="\$FACTORYOS_ROOT/.fake"
echo "curl \$*" >> "\$f/calls"
down() { $net_down; }
out="" url=""
while [ \$# -gt 0 ]; do case "\$1" in -o) out=\$2; shift ;; https://*) url=\$1 ;; esac; shift; done
fail=\$(cat "\$f/curl-fail" 2>/dev/null || true)
if { [ -n "\$fail" ] && [[ "\$url" == *"\$fail"* ]]; } || down; then
  echo "curl: (28) Connection timed out after 20002 milliseconds" >&2; exit 28
fi
echo "from \$url" > "\$out"
EOF
# unzip of the AWS CLI zip: aws/install, which installs an aws whose --version is v2's and which is else the stub.
cat > "$ustubs/unzip" <<EOF
#!/bin/bash
d=""; while [ \$# -gt 0 ]; do [ "\$1" = -d ] && d=\$2; shift; done
mkdir -p "\$d/aws"
printf '#!/bin/bash\nmkdir -p "\$FACTORYOS_ROOT/usr/local/bin"\nprintf "%%s\\\\n" "#!/bin/bash" "[ \\\\"\\\\\$1\\\\" = --version ] && { echo aws-cli/2.99.0 Python/3.13; exit 0; }" "exec $stubs/aws \\\\"\\\\\$@\\\\"" > "\$FACTORYOS_ROOT/usr/local/bin/aws"\nchmod 0755 "\$FACTORYOS_ROOT/usr/local/bin/aws"\n' > "\$d/aws/install"
chmod 0755 "\$d/aws/install"
EOF
cat > "$ustubs/fallocate" <<'EOF'
#!/bin/bash
echo "fallocate $*" >> "$FACTORYOS_ROOT/.fake/calls"
: > "${!#}"
EOF
for c in systemctl mkswap sysctl dd; do
  printf '#!/bin/bash\necho "%s $*" >> "$FACTORYOS_ROOT/.fake/calls"\n' "$c" > "$ustubs/$c"
done
cat > "$ustubs/swapon" <<'EOF'
#!/bin/bash
echo "swapon $*" >> "$FACTORYOS_ROOT/.fake/calls"
case "$*" in *--show=SIZE*) echo 16G ;; esac
EOF
printf '#!/bin/bash\necho arm64\n' > "$ustubs/dpkg"
printf '#!/bin/bash\nexit 1\n' > "$ustubs/pgrep"
chmod 0755 "$ustubs"/*
ud_reset() {
  rm -rf "$U"; mkdir -p "$U/.fake" "$U/var/lib/cloud/data" "$U/etc/sysctl.d"
  echo i-0123456789abcdef0 > "$U/var/lib/cloud/data/instance-id"; : > "$U/etc/fstab"
}
# ud [VAR=value…]: user-data.sh as the CDK prelude runs it (FACTORYOS_STACK t), no AWS CLI on PATH before it installs one.
ud() {
  env PATH="$ustubs:$sandbox" FACTORYOS_ROOT="$U" FACTORYOS_STACK=t AWS_REGION=eu-central-1 AWS_DEFAULT_REGION=eu-central-1 \
    FACTORYOS_SOURCE_BUCKET=bkt FACTORYOS_LOG_GROUP=/factoryos/t FACTORYOS_PUBLIC=1 FACTORYOS_RETRY_SLEEP=0 "$@" \
    bash "$boot/user-data.sh" > "$work/ud.out" 2>&1
  echo $?
}
ud_state() { python3 -I -c 'import json, sys; s = json.load(open(sys.argv[1])); print(s.get(sys.argv[2]))' "$U/.fake/ssm/_factoryos_t_release_state" "$1" 2>/dev/null; }
ud_calls() { cat "$U/.fake/calls" 2>/dev/null; }
ud_log() { cat "$U/var/log/factoryos/bootstrap.log" 2>/dev/null; }
# first_line <text>: the line number of its first call (0: none).
first_line() { local n; n=$(grep -nF -- "$1" "$U/.fake/calls" | head -1 | cut -d: -f1); echo "${n:-0}"; }
last_line() { local n; n=$(grep -nF -- "$1" "$U/.fake/calls" | tail -1 | cut -d: -f1); echo "${n:-0}"; }

# The network is down for the first minutes (a test install of 2026-11-14: apt-get install gave up at once, the
# state was never written): every network step waits for it.
ud_reset; echo 5 > "$U/.fake/net-down"
check "first boot, network down at first: exit 0" 0 "$(ud)"
check "first boot, network down at first: state waiting" "waiting" "$(ud_state phase)"
check "first boot: the state's log stream is the instance's release.log" "i-0123456789abcdef0/release.log" "$(ud_state logStream)"
check "first boot: bootstrap.done" "yes" "$([ -s "$U/var/lib/factoryos-release/bootstrap.done" ] && echo yes)"
contains "first boot: the retries are logged" "retry 1: apt_get update" "$(ud_log)"
check "first boot: apt-get update fails on an index it cannot fetch (--error-on=any)" "yes" \
  "$(grep '^apt-get ' "$U/.fake/calls" | grep ' update ' | grep -vq -- '--error-on=any' && echo no || echo yes)"
check "first boot: every curl is bounded (--connect-timeout, --max-time)" "yes" \
  "$(grep '^curl ' "$U/.fake/calls" | grep -v -- '--connect-timeout' | grep -vq -- '--max-time' && echo no || echo yes)"
check "first boot: the CloudWatch agent is configured (bounded by timeout)" "yes" \
  "$(grep -q '^amazon-cloudwatch-agent-ctl -a fetch-config -m ec2 -s' "$U/.fake/calls" && echo yes)"
check "first boot: reboots masked before the first apt-get, unmasked after the state" "yes" \
  "$(m=$(first_line 'systemctl mask --runtime reboot.target'); a=$(first_line 'apt-get '); p=$(last_line 'ssm put-parameter'); u=$(last_line 'systemctl unmask --runtime reboot.target')
     [ "$m" -gt 0 ] && [ "$m" -lt "$a" ] && [ "$p" -lt "$u" ] && echo yes)"

# A run by hand on a booted instance keeps its own state.
check "run again: exit 0" 0 "$(ud)"
contains "run again: the state is kept" "release state: kept" "$(ud_log)"

# The network never comes up: the script fails, says where, and lets reboots go ahead again. No AWS CLI yet, so no
# state (the installer's console tail shows the log then).
ud_reset; echo forever > "$U/.fake/net-down"
rc=$(ud FACTORYOS_NET_WAIT=1 FACTORYOS_RETRY_SLEEP=0.1)
check "network never up: exit non-zero" "yes" "$([ "$rc" != 0 ] && echo yes)"
contains "network never up: the failure names its command" "FAILED: first-boot setup failed (exit 1, line" "$(ud_log)"
contains "network never up: ... apt-get update" "of user-data.sh): net apt_get update -y -qq --error-on=any" "$(ud_log)"
check "network never up: reboots unmasked at the end" "yes" \
  "$([ "$(last_line 'systemctl unmask --runtime reboot.target')" -gt "$(last_line 'apt-get ')" ] && echo yes)"
check "network never up: no state (no AWS CLI yet)" "" "$(ud_state phase)"
check "network never up: no bootstrap.done" "no" "$([ -e "$U/var/lib/factoryos-release/bootstrap.done" ] && echo yes || echo no)"

# A failure after the AWS CLI is there: the state says so, with the bootstrap log's stream.
ud_reset; echo amazoncloudwatch-agent > "$U/.fake/curl-fail"
rc=$(ud FACTORYOS_NET_WAIT=1 FACTORYOS_RETRY_SLEEP=0.1)
check "CloudWatch agent download fails: exit non-zero" "yes" "$([ "$rc" != 0 ] && echo yes)"
check "CloudWatch agent download fails: state failed" "failed" "$(ud_state phase)"
check "CloudWatch agent download fails: no version (the first boot's)" "" "$(ud_state version)"
check "CloudWatch agent download fails: the log stream is bootstrap.log" "i-0123456789abcdef0/bootstrap.log" "$(ud_state logStream)"
contains "CloudWatch agent download fails: the error names the step" "first-boot setup failed (exit 1, line" "$(ud_state error)"
contains "CloudWatch agent download fails: ... the download" "net download https://amazoncloudwatch-agent.s3.amazonaws.com/" "$(ud_state error)"
check "CloudWatch agent download fails: the error is one line of at most 300 characters" "yes" \
  "$(e=$(ud_state error); [ "${#e}" -le 300 ] && [ "$(printf '%s\n' "$e" | wc -l | tr -d ' ')" = 1 ] && echo yes)"
check "CloudWatch agent download fails: finishedAt set" "yes" "$([ "$(ud_state finishedAt)" != None ] && echo yes)"
# ... and a run by hand once the cause is gone starts over: waiting again.
rm -f "$U/.fake/curl-fail"
check "run again after a first-boot failure: exit 0" 0 "$(ud)"
check "run again after a first-boot failure: state waiting" "waiting" "$(ud_state phase)"
contains "run again after a first-boot failure: logged" "it was this instance's failed first boot" "$(ud_log)"

# A failed run by hand on a serving instance leaves its state alone.
mkdir -p "$U/opt/factoryos/releases/x"; ln -sfn releases/x "$U/opt/factoryos/current"
echo '{"phase":"ready","version":"v1.0.0","previous":null,"startedAt":"2026-01-01T00:00:00Z","updatedAt":"2026-01-01T00:00:00Z","finishedAt":"2026-01-01T00:00:00Z","error":null,"logStream":"i-0123456789abcdef0/release.log"}' \
  > "$U/.fake/ssm/_factoryos_t_release_state"
echo amazoncloudwatch-agent > "$U/.fake/curl-fail"; rm -f "$U/opt/aws/amazon-cloudwatch-agent/bin/amazon-cloudwatch-agent-ctl"
rc=$(ud FACTORYOS_NET_WAIT=1 FACTORYOS_RETRY_SLEEP=0.1)
check "serving instance, failed run by hand: exit non-zero" "yes" "$([ "$rc" != 0 ] && echo yes)"
check "serving instance, failed run by hand: the state is kept" "ready v1.0.0" "$(ud_state phase) $(ud_state version)"

# A replaced instance: the state names another instance; it is taken over (waiting). The other instance is the
# allowlist's second placeholder id (public-export/allowlist.txt): any other made-up id is a deny hit in the export.
ud_reset
mkdir -p "$U/.fake/ssm"
echo '{"phase":"ready","version":"v1.0.0","previous":null,"startedAt":"2026-01-01T00:00:00Z","updatedAt":"2026-01-01T00:00:00Z","finishedAt":"2026-01-01T00:00:00Z","error":null,"logStream":"i-1234567890abcdef0/release.log"}' \
  > "$U/.fake/ssm/_factoryos_t_release_state"
check "replaced instance: exit 0" 0 "$(ud)"
check "replaced instance: state waiting" "waiting" "$(ud_state phase)"

echo "== state JSON: every write has the contract's shape"
check "shapes" "ok" "$(python3 - "$F/all-states" <<'PY'
import json, re, sys, glob
keys = {"phase", "version", "previous", "startedAt", "updatedAt", "finishedAt", "error", "logStream"}
phases = {"waiting", "downloading", "deps", "building", "packaging", "deploying", "checking", "ready", "failed",
          "rolled-back"}
iso = re.compile(r"\d{4}-\d\d-\d\dT\d\d:\d\d:\d\dZ$")
bad = []
for line in open(sys.argv[1]):
    s = json.loads(line)
    if set(s) - {"incomplete"} != keys: bad.append("keys %s" % sorted(s))
    if "incomplete" in s and (s["phase"] != "ready" or not s["incomplete"]
                              or not all(isinstance(x, str) and x for x in s["incomplete"])):
        bad.append("incomplete %r for %s" % (s["incomplete"], s["phase"]))
    if s["phase"] not in phases: bad.append("phase " + s["phase"])
    if not isinstance(s["version"], str): bad.append("version")
    if s["previous"] is not None and not isinstance(s["previous"], str): bad.append("previous")
    for k in ("startedAt", "updatedAt"):
        if not iso.match(s[k]): bad.append(k + " " + s[k])
    terminal = s["phase"] in ("ready", "failed", "rolled-back")
    if terminal != (s["finishedAt"] is not None): bad.append("finishedAt for " + s["phase"])
    if s["finishedAt"] is not None and not iso.match(s["finishedAt"]): bad.append("finishedAt")
    if (s["phase"] in ("failed", "rolled-back")) != (s["error"] is not None): bad.append("error for " + s["phase"])
    if s["error"] is not None and ("\n" in s["error"] or len(s["error"]) > 400): bad.append("error not one line")
    if not s["logStream"].endswith("/release.log"): bad.append("logStream")
print("ok" if not bad else "; ".join(sorted(set(bad))))
PY
)"

echo "== the user's data was never touched"
check "/var/lib/factoryos unchanged" "$data_before" "$(data_snapshot)"

exit "$fail"
