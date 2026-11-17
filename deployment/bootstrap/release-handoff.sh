#!/bin/bash
#
# The release step's handoff (deployment/CONTRACT.md "SSM document ReleaseDocument"): the stack's SSM command
# document runs this as root on the instance (the CDK app inlines this file; its shebang must stay the first line),
# and it returns within a minute or so (the download aside):
#   1. refuses (exit 1) while a release runs (factoryos-release.service active, or its lock held);
#   1b. writes the origin secret (the SecureString /factoryos/<stack>/origin-secret, which the document exports as
#      FACTORYOS_ORIGIN_SECRET_PARAMETER; deployment/CONTRACT.md "Origin secret") into /etc/factoryos/origin.env (0600 root, FACTORY_OS_ORIGIN_SECRET=…; never echoed), which
#      the backend loads (public provisioning's drop-in); when the value changed, a running backend is restarted
#      (only when it is active; a first install has no backend unit yet). A request with `secretsOnly: true` (the installer's rotation) stops here;
#   2. reads /factoryos/<stack>/release/wanted, writes the state `downloading` (version, previous, startedAt);
#   3. downloads the export from the stack's source bucket, checks its sha256, and unpacks it (zip or tar.gz, one top
#      directory, which is stripped) into /opt/factoryos/src/<version>/; an export already unpacked there with the
#      same sha256 is kept (with its build products: a retry builds incrementally) unless `force`;
#   4. downloads the optional umarell tarball to /opt/factoryos/src/<version>.umarell.tar.gz (sha256 checked), and the
#      optional umarell_jam tarball to <version>.umarell-jam.tar.gz (one that cannot be downloaded or whose sha256
#      differs is left out with a WARNING: umarell_jam is optional, the release goes on and marks it incomplete);
#   4c. downloads the optional bundle (`bundle`: the release build in AWS CodeBuild made it of this export) to
#      /opt/factoryos/bundles/factoryos-viewer-linux-arm64-<version>.tar.gz (sha256 checked; its .sha256 next to it):
#      factoryos-release deploys it instead of building;
#   5. installs that export's factoryos-release.sh as /usr/local/sbin/factoryos-release and its unit, and starts the
#      unit with --no-block. The unit does the rest (deps, build, package, deploy, check; with a bundle deploy and check)
#      and writes the state.
# The same version as the one running is "already installed" (state `ready`) unless the request says `force`.
# A failure here writes the state `failed` with a one-line error.
#
# Paths honour FACTORYOS_ROOT (a prefix for /opt, /etc, /var, /run, /usr/local: deployment/bootstrap/tests).
# Should a runner start this with sh (not through its shebang): the same under bash.
if [ -z "${BASH_VERSION:-}" ]; then exec /bin/bash "$0" "$@"; fi
set -euo pipefail
# The stack's SSM document runs this under umask 077 (its own /run/factoryos stays root-only). What this creates is
# not: /opt/factoryos is the parent of the releases the backend runs as the factoryos user, and a 0700 one made every
# release unexecutable for it (status 203/EXEC "Permission denied", test install 2611122014-dc89). The same umask as
# factoryos-release.sh, and the mode of /opt/factoryos spelled out (an earlier handoff's 0700 one repaired).
umask 022

R=${FACTORYOS_ROOT:-}
stack_env="$R/etc/factoryos/stack.env"
opt="$R/opt/factoryos"
srcs="$opt/src"
lib="$R/var/lib/factoryos-release"
logdir="$R/var/log/factoryos"
log="$logdir/release.log"
lock="$R/run/factoryos-release.lock"
unit=factoryos-release.service
prefix=factoryos-viewer-linux-arm64-
mkdir -p "$srcs" "$lib" "$logdir" "$(dirname "$lock")"
chmod 0755 "$opt"

now() { date -u +%Y-%m-%dT%H:%M:%SZ; }
say() { echo "release-handoff: $*"; echo "[$(now) handoff] $*" >> "$log"; }
die() { echo "release-handoff: $*" >&2; echo "[$(now) handoff] ERROR: $*" >> "$log"; exit 1; }
sha256() { sha256sum "$1" | cut -d' ' -f1; }

# --- 1. one release at a time (refused before anything changes)
active=$(systemctl is-active "$unit" 2>/dev/null || true)
case "$active" in
  active|activating|deactivating|reloading)
    die "a release is running ($unit is $active): wait for it (install.sh status --wait)" ;;
esac
exec 9>"$lock"
flock -n 9 || die "a release is running (its lock $lock is held)"

[ -r "$stack_env" ] || die "$stack_env is missing (user data not run?)"
# The stack's settings that may change with an update: the document exports FACTORYOS_KIRO_SECRET (the secret's name,
# possibly empty) and FACTORYOS_BEDROCK (0|1); they go into stack.env on every run (an exported value wins, an empty
# Kiro secret removes the key; one not exported leaves the file's), the other keys stay user data's. factoryos-release
# reads them from there.
if [ -n "${FACTORYOS_KIRO_SECRET+set}" ] || [ -n "${FACTORYOS_BEDROCK+set}" ]; then
  kiro_secret=${FACTORYOS_KIRO_SECRET-}
  bedrock=${FACTORYOS_BEDROCK-}
  [ "${#kiro_secret}" -le 512 ] && [[ "$kiro_secret" =~ ^[A-Za-z0-9/_+=.@-]*$ ]] \
    || die "FACTORYOS_KIRO_SECRET is not a secret name"
  [ -z "${FACTORYOS_BEDROCK+set}" ] || [ "$bedrock" = 0 ] || [ "$bedrock" = 1 ] \
    || die "FACTORYOS_BEDROCK is not 0 or 1"
  drop="^(_none_"
  [ -z "${FACTORYOS_KIRO_SECRET+set}" ] || drop="$drop|FACTORYOS_KIRO_SECRET"
  [ -z "${FACTORYOS_BEDROCK+set}" ] || drop="$drop|FACTORYOS_BEDROCK"
  {
    grep -vE "$drop)=" "$stack_env" || true
    if [ -n "${FACTORYOS_KIRO_SECRET+set}" ] && [ -n "$kiro_secret" ]; then echo "FACTORYOS_KIRO_SECRET=$kiro_secret"; fi
    if [ -n "${FACTORYOS_BEDROCK+set}" ]; then echo "FACTORYOS_BEDROCK=$bedrock"; fi
  } > "$stack_env.new"
  chmod 0644 "$stack_env.new"
  mv -f "$stack_env.new" "$stack_env"
fi
# shellcheck disable=SC1090,SC1091
. "$stack_env"
: "${FACTORYOS_STACK:?}" "${AWS_REGION:?}" "${FACTORYOS_SOURCE_BUCKET:?}"
# The AWS CLI v2 is in /usr/local/bin (user data); a runner's PATH may lack it.
export AWS_DEFAULT_REGION="$AWS_REGION" PATH="$PATH:/usr/local/bin"
param="/factoryos/$FACTORYOS_STACK/release"

# --- 1b. the origin secret: root reads the SecureString into a 0600 EnvironmentFile. The value only ever goes from the
# CLI's stdout through python into the file: no argv, no log, no state. Missing or malformed: refused (exit 1), before
# anything else changes; the release before keeps serving.
origin_env() {
  local f="$R/etc/factoryos/origin.env" name="${FACTORYOS_ORIGIN_SECRET_PARAMETER:-/factoryos/$FACTORYOS_STACK/origin-secret}"
  local err="$lib/origin.err"
  [ "$name" = "/factoryos/$FACTORYOS_STACK/origin-secret" ] \
    || die "FACTORYOS_ORIGIN_SECRET_PARAMETER is not /factoryos/$FACTORYOS_STACK/origin-secret"
  mkdir -p "$R/etc/factoryos"
  rm -f "$f.new"
  if ! (umask 077
        aws ssm get-parameter --region "$AWS_REGION" --name "$name" --with-decryption --query Parameter --output json \
            2>"$err" | python3 -I -c '
import json, re, sys
try:
    p = json.load(sys.stdin)
except ValueError:
    sys.exit("no parameter")
if not isinstance(p, dict) or p.get("Type") != "SecureString":
    sys.exit("it is not a SecureString")
values = [v.strip() for v in str(p.get("Value") or "").strip().split(",")]
if not 1 <= len(values) <= 2 or not all(re.fullmatch(r"[A-Za-z0-9_-]{32,256}", v) for v in values):
    sys.exit("it is not one or two comma-separated base64url values of 32 to 256 characters")
print("FACTORY_OS_ORIGIN_SECRET=" + ",".join(values))' > "$f.new" 2>>"$err"); then
    rm -f "$f.new"
    die "cannot read the origin secret $name: $(tr '\n' ' ' < "$err" | cut -c1-200)"
  fi
  rm -f "$err"
  chmod 0600 "$f.new"
  if [ "$(id -u)" = 0 ]; then chown root:root "$f.new"; fi
  if [ -f "$f" ] && [ "$(sha256 "$f.new")" = "$(sha256 "$f")" ]; then
    rm -f "$f.new"
    say "origin secret: $f is current"
    return 0
  fi
  mv -f "$f.new" "$f"
  say "origin secret: $f written from $name"
  # Only a running backend: on a first install the unit does not exist yet (`systemctl try-restart` of a unit that
  # is not loaded fails), and the deploy starts it with this file anyway. is-active is false for a missing unit.
  if systemctl is-active --quiet factoryos-backend.service; then
    systemctl restart factoryos-backend.service || die "the backend did not restart with the new origin secret"
  else
    say "origin secret: the backend is not running (first install): the deploy starts it with $f"
  fi
}
origin_env

# --- 2. what is wanted
wanted=$(aws ssm get-parameter --region "$AWS_REGION" --name "$param/wanted" --query Parameter.Value --output text) \
  || die "cannot read $param/wanted"
# The installer's origin secret rotation (`secretsOnly: true`): 1b. was all it asked for; the state is not touched.
if python3 -I -c 'import json, sys
try:
    sys.exit(0 if json.loads(sys.argv[1]).get("secretsOnly") is True else 1)
except (ValueError, AttributeError):
    sys.exit(1)' "$wanted"; then
  say "secrets only: the origin secret is refreshed, nothing else was requested"
  exit 0
fi
fields=$(python3 -I -c '
import json, re, sys
try:
    w = json.loads(sys.argv[1])
except ValueError:
    sys.exit("wanted is not JSON")
version, key, sha = w.get("version"), w.get("key"), w.get("sha256")
if not isinstance(version, str) or not re.fullmatch(r"[A-Za-z0-9._+-]{1,64}", version) or version in (".", ".."):
    sys.exit("wanted.version is not [A-Za-z0-9._+-]{1,64}")
if not isinstance(key, str) or not re.fullmatch(r"releases/[A-Za-z0-9._+/-]+\.(zip|tar\.gz)", key) or ".." in key:
    sys.exit("wanted.key is not releases/<name>.zip|.tar.gz")
if not isinstance(sha, str) or not re.fullmatch(r"[0-9a-f]{64}", sha):
    sys.exit("wanted.sha256 is not 64 lower-case hex digits")
u = w.get("umarell")
ukey = usha = ""
if u is not None:
    ukey, usha = u.get("key"), u.get("sha256")
    if not isinstance(ukey, str) or not re.fullmatch(r"vendor/[A-Za-z0-9._+-]+\.tar\.gz", ukey):
        sys.exit("wanted.umarell.key is not vendor/<name>.tar.gz")
    if not isinstance(usha, str) or not re.fullmatch(r"[0-9a-f]{64}", usha):
        sys.exit("wanted.umarell.sha256 is not 64 lower-case hex digits")
j = w.get("umarellJam")
jkey = jsha = ""
if j is not None:
    jkey, jsha = (j.get("key"), j.get("sha256")) if isinstance(j, dict) else (None, None)
    if not isinstance(jkey, str) or not re.fullmatch(r"vendor/[A-Za-z0-9._+-]+\.tar\.gz", jkey):
        sys.exit("wanted.umarellJam.key is not vendor/<name>.tar.gz")
    if not isinstance(jsha, str) or not re.fullmatch(r"[0-9a-f]{64}", jsha):
        sys.exit("wanted.umarellJam.sha256 is not 64 lower-case hex digits")
b = w.get("bundle")
bkey = bsha = ""
if b is not None:
    bkey, bsha = (b.get("key"), b.get("sha256")) if isinstance(b, dict) else (None, None)
    if bkey != "releases/factoryos-viewer-linux-arm64-%s.tar.gz" % version:
        sys.exit("wanted.bundle.key is not releases/factoryos-viewer-linux-arm64-<version>.tar.gz")
    if not isinstance(bsha, str) or not re.fullmatch(r"[0-9a-f]{64}", bsha):
        sys.exit("wanted.bundle.sha256 is not 64 lower-case hex digits")
print(version, key, sha, ukey or "-", usha or "-", jkey or "-", jsha or "-", bkey or "-", bsha or "-",
      "1" if w.get("force") is True else "0")
' "$wanted" 2>&1) || die "$param/wanted: $fields"
read -r version key sha ukey usha jkey jsha bkey bsha force <<<"$fields"
[ "$ukey" = - ] && ukey="" && usha=""
[ "$jkey" = - ] && jkey="" && jsha=""
[ "$bkey" = - ] && bkey="" && bsha=""

previous=""
if [ -L "$opt/current" ]; then
  current_name=$(basename "$(readlink "$opt/current")")
  case "$current_name" in "$prefix"*) previous=${current_name#"$prefix"} ;; esac
fi
stream="$(cat "$R/var/lib/cloud/data/instance-id" 2>/dev/null || echo unknown)/release.log"

# state <phase> <startedAt> <finishedAt|""> <error|"">: the state JSON into $lib/state.json and the parameter; `ready`
# (already installed) with `incomplete` as factoryos-release writes it (/etc/factoryos/provision-incomplete).
state() {
  local json incomplete=""
  [ "$1" != ready ] || incomplete=$(cat "$R/etc/factoryos/provision-incomplete" 2>/dev/null || true)
  json=$(python3 -I -c '
import json, sys
phase, version, previous, started, finished, error, stream, now, incomplete = sys.argv[1:]
s = {"phase": phase, "version": version, "previous": previous or None, "startedAt": started, "updatedAt": now,
     "finishedAt": finished or None, "error": error or None, "logStream": stream}
inc = list(dict.fromkeys(x.strip() for x in incomplete.splitlines() if x.strip()))
if inc:
    s["incomplete"] = inc
print(json.dumps(s, separators=(",", ":")))' "$1" "$version" "$previous" "$2" "$3" "$4" "$stream" "$(now)" "$incomplete")
  printf '%s\n' "$json" > "$lib/state.json.new" && mv -f "$lib/state.json.new" "$lib/state.json"
  aws ssm put-parameter --region "$AWS_REGION" --name "$param/state" --type String --overwrite --value "$json" >/dev/null \
    || say "WARNING: could not write $param/state"
}

# --- already installed?
if [ "$force" != 1 ] && [ "$previous" = "$version" ]; then
  t=$(now)
  previous=$(python3 -I -c 'import json, sys
try:
    s = json.load(open(sys.argv[1]))
    print(s.get("previous") or "" if s.get("version") == sys.argv[2] else "")
except (OSError, ValueError):
    print("")' "$lib/state.json" "$version")
  state ready "$t" "$t" ""
  say "$version is already installed (running); force: true rebuilds it"
  exit 0
fi

started=$(now)
echo "==== $(now) factoryos release $version (previous ${previous:-none}; ${key}; force $force) ====" >> "$log"
# From here on every failure leaves the state `failed` (current is not touched: the running release keeps serving).
in_flight=0
failed() { in_flight=0; state failed "$started" "$(now)" "downloading: $1"; die "$1"; }
on_exit() {
  local rc=$?
  if [ "$rc" != 0 ] && [ "$in_flight" = 1 ]; then
    in_flight=0
    state failed "$started" "$(now)" "downloading: the handoff stopped (exit $rc), see $stream"
  fi
}
trap on_exit EXIT
state downloading "$started" "" ""
in_flight=1
say "downloading $version (previous ${previous:-none})"

# --- 3. the export
dest="$srcs/$version"
marker="$dest/.factoryos-export"
if [ "$force" != 1 ] && [ "$(cat "$marker" 2>/dev/null)" = "$sha" ]; then
  say "$dest is already unpacked (sha256 $sha); kept"
else
  incoming="$srcs/.incoming-$version"
  rm -rf "$incoming" && mkdir -p "$incoming/tree"
  case "$key" in *.zip) archive="$incoming/export.zip" ;; *) archive="$incoming/export.tar.gz" ;; esac
  aws s3 cp "s3://$FACTORYOS_SOURCE_BUCKET/$key" "$archive" --region "$AWS_REGION" --only-show-errors \
    || failed "cannot download s3://$FACTORYOS_SOURCE_BUCKET/$key"
  got=$(sha256 "$archive")
  [ "$got" = "$sha" ] || { rm -rf "$incoming"; failed "$key: sha256 $got, wanted $sha"; }
  case "$archive" in
    *.zip)
      command -v unzip >/dev/null || failed "unzip is not installed"
      unzip -q "$archive" -d "$incoming/tree" || failed "$key: cannot unzip" ;;
    *) tar -xzf "$archive" -C "$incoming/tree" || failed "$key: cannot untar" ;;
  esac
  rm -f "$archive"
  tops=$(find "$incoming/tree" -mindepth 1 -maxdepth 1 | wc -l | tr -d ' ')
  top=$(find "$incoming/tree" -mindepth 1 -maxdepth 1 -type d | head -1)
  [ "$tops" = 1 ] && [ -n "$top" ] || failed "$key: not one top directory ($tops entries)"
  [ -f "$top/deployment/bootstrap/factoryos-release.sh" ] && [ -f "$top/deployment/bootstrap/factoryos-release.service" ] \
    || failed "$key: no deployment/bootstrap/factoryos-release.{sh,service} under $(basename "$top")/"
  rm -rf "$dest"
  mv "$top" "$dest"
  rm -rf "$incoming"
  echo "$sha" > "$marker"
  say "unpacked $(basename "$key") into $dest"
fi

# --- 4. the optional umarell tarball
utar="$srcs/$version.umarell.tar.gz"
if [ -n "$ukey" ]; then
  if [ "$(sha256 "$utar" 2>/dev/null || true)" != "$usha" ]; then
    aws s3 cp "s3://$FACTORYOS_SOURCE_BUCKET/$ukey" "$utar.part" --region "$AWS_REGION" --only-show-errors \
      || failed "cannot download s3://$FACTORYOS_SOURCE_BUCKET/$ukey"
    got=$(sha256 "$utar.part")
    [ "$got" = "$usha" ] || { rm -f "$utar.part"; failed "$ukey: sha256 $got, wanted $usha"; }
    mv -f "$utar.part" "$utar"
  fi
  say "umarell tarball $(basename "$ukey")"
else
  rm -f "$utar"
fi
# --- 4b. the optional umarell_jam tarball: never fails the release (provisioning marks umarell_jam incomplete)
jtar="$srcs/$version.umarell-jam.tar.gz"
if [ -n "$jkey" ]; then
  if [ "$(sha256 "$jtar" 2>/dev/null || true)" != "$jsha" ]; then
    rm -f "$jtar"
    if aws s3 cp "s3://$FACTORYOS_SOURCE_BUCKET/$jkey" "$jtar.part" --region "$AWS_REGION" --only-show-errors \
         && [ "$(sha256 "$jtar.part")" = "$jsha" ]; then
      mv -f "$jtar.part" "$jtar"
    else
      rm -f "$jtar.part"
      say "WARNING: the umarell_jam tarball s3://$FACTORYOS_SOURCE_BUCKET/$jkey cannot be downloaded or is not sha256 $jsha: the release goes on without it (umarell_jam incomplete)"
      jkey="" jsha=""
    fi
  fi
  [ -z "$jkey" ] || say "umarell_jam tarball $(basename "$jkey")"
else
  rm -f "$jtar"
fi
# --- 4c. the optional bundle the release build in AWS CodeBuild made of this export (deployment/CONTRACT.md "Release
# build in AWS CodeBuild"): to /opt/factoryos/bundles/<prefix><version>.tar.gz with its .sha256, where
# factoryos-release deploys it from instead of building
btar="$opt/bundles/$prefix$version.tar.gz"
if [ -n "$bkey" ]; then
  mkdir -p "$opt/bundles"
  if [ "$(cat "$btar.sha256" 2>/dev/null)" != "$bsha" ] || [ "$(sha256 "$btar" 2>/dev/null || true)" != "$bsha" ]; then
    rm -f "$btar" "$btar.sha256"
    aws s3 cp "s3://$FACTORYOS_SOURCE_BUCKET/$bkey" "$btar.part" --region "$AWS_REGION" --only-show-errors \
      || failed "cannot download s3://$FACTORYOS_SOURCE_BUCKET/$bkey"
    got=$(sha256 "$btar.part")
    [ "$got" = "$bsha" ] || { rm -f "$btar.part"; failed "$bkey: sha256 $got, wanted $bsha"; }
    mv -f "$btar.part" "$btar"
    echo "$bsha" > "$btar.sha256"
  fi
  say "bundle $(basename "$bkey") (built in AWS CodeBuild)"
fi
# What the unit releases (its resume after a reboot needs nothing from SSM).
python3 -I -c 'import json, sys
version, sha, utar, usha, jtar, jsha, btar, bsha, force = sys.argv[1:]
print(json.dumps({"version": version, "sha256": sha, "umarell": {"path": utar, "sha256": usha} if usha else None,
                  "umarellJam": {"path": jtar, "sha256": jsha} if jsha else None,
                  "bundle": {"path": btar, "sha256": bsha} if bsha else None,
                  "force": force == "1"}))' "$version" "$sha" "$utar" "$usha" "$jtar" "$jsha" "$btar" "$bsha" "$force" \
  > "$lib/request.json"

# --- 5. the export's release unit, started without waiting
bootstrap="$dest/deployment/bootstrap"
mkdir -p "$R/usr/local/sbin" "$R/etc/systemd/system"
install -m 0755 "$bootstrap/factoryos-release.sh" "$R/usr/local/sbin/factoryos-release"
install -m 0644 "$bootstrap/factoryos-release.service" "$R/etc/systemd/system/$unit"
systemctl daemon-reload || failed "systemctl daemon-reload failed"
# Enabled: started at boot too, where it resumes a release a reboot interrupted (and else does nothing).
systemctl enable --quiet "$unit" || failed "cannot enable $unit"
# The unit takes the lock itself.
exec 9>&-
systemctl start --no-block "$unit" || failed "cannot start $unit"
in_flight=0
say "started $unit for $version; follow it with: install.sh status --wait (log: $stream)"
