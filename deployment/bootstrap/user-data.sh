#!/bin/bash
#
# The public deployment's user data (deployment/CONTRACT.md "The instance"): runs once, as root, at the instance's
# first boot. The CDK app inlines this file behind a prelude that exports
#   FACTORYOS_STACK, AWS_REGION, AWS_DEFAULT_REGION, FACTORYOS_SOURCE_BUCKET, FACTORYOS_LOG_GROUP, FACTORYOS_PUBLIC=1
# and nothing in here depends on the release: a new version never changes the user data, so an update never replaces
# the instance. The release itself (the build toolchain, the build, the deploy) is factoryos-release's, started over
# SSM by the installer (deployment/bootstrap/release-handoff.sh).
#
# It
#   0. masks reboot.target (and kexec, ctrl-alt-del) in /run while it runs (a patch association's reboot would cut
#      it short), and unmasks them when it ends, also on an error,
#   1. writes /etc/factoryos/stack.env (the stack's names; the settings an update may change, FACTORYOS_KIRO_SECRET
#      and FACTORYOS_BEDROCK, are added by every release handoff, from the release document's environment),
#   2. masks ssh (the instance is reached over SSM only),
#   3. creates 16 GiB of swap (rustc / ld peaks of the release build),
#   4. installs the AWS CLI v2 (the official aarch64 zip) and the CloudWatch agent (the official arm64 .deb), which
#      ships /var/log/factoryos/{bootstrap,release}.log to the stack's log group as {instance_id}/<file>,
#   5. writes the release state parameter (/factoryos/<stack>/release/state) as `waiting` when it has none, or when
#      it is another instance's (a replaced instance: its log stream names another instance id), or when it is this
#      instance's own first-boot failure (a run by hand after one).
# Every step that needs the network retries until FACTORYOS_NET_WAIT seconds (default 1800) after the start: a new
# instance behind a new NAT gateway has no way out for its first minutes (up to 3 seen), and apt-get gave up on that
# at once (2026-11-14: the state was never written, the installer waited its full hour). A failure is logged with its
# command and, once the AWS CLI is there, written to the state parameter as `failed` (version "", the error, the
# bootstrap.log stream), so the installer stops at once and names it.
# Idempotent: each step is skipped when its result is there, so it can be run again by hand. Its own log is
# /var/log/factoryos/bootstrap.log. FACTORYOS_ROOT (a path prefix) is for deployment/bootstrap/tests only.
set -euo pipefail

: "${FACTORYOS_STACK:?the CDK prelude exports FACTORYOS_STACK}"
: "${AWS_REGION:?the CDK prelude exports AWS_REGION}"
: "${FACTORYOS_SOURCE_BUCKET:?the CDK prelude exports FACTORYOS_SOURCE_BUCKET}"
: "${FACTORYOS_LOG_GROUP:?the CDK prelude exports FACTORYOS_LOG_GROUP}"
R=${FACTORYOS_ROOT:-}

install -d -m 0755 "$R/var/log/factoryos" "$R/etc/factoryos" "$R/var/lib/factoryos-release"
exec > >(tee -a "$R/var/log/factoryos/bootstrap.log") 2>&1
log() { echo "[bootstrap $(date -u +%FT%TZ)] $*"; }
log "start: stack $FACTORYOS_STACK in $AWS_REGION"
net_deadline=$(( $(date +%s) + ${FACTORYOS_NET_WAIT:-1800} ))
instance_id=$(cat "$R/var/lib/cloud/data/instance-id" 2>/dev/null || echo unknown)
param="/factoryos/$FACTORYOS_STACK/release/state"
export PATH="$R/usr/local/bin:$PATH"

# state_json <phase> <error|"">: the release state of this instance before its first release.
state_json() {
  python3 -I -c 'import json, sys
phase, error, now, stream = sys.argv[1:]
print(json.dumps({"phase": phase, "version": "", "previous": None, "startedAt": now, "updatedAt": now,
                  "finishedAt": now if phase == "failed" else None, "error": error or None, "logStream": stream},
                 separators=(",", ":")))' "$1" "$2" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$instance_id/${3:-release.log}"
}

# --- 0. No reboot while this runs: an account's patch association (AWS-RunPatchBaseline, Operation Install) reboots a
# new instance minutes after it registers with SSM, which would kill this script before step 5 writes the state
# parameter (cloud-init does not run user data again). Masked in /run only (as deploy/planner-router/burst.py does),
# and unmasked when this script ends, also on an error, so later patch reboots go ahead.
no_reboot="reboot.target kexec.target ctrl-alt-del.target"
failed_line="" failed_cmd="" net_failed=""
# released: a release was installed here once (/opt/factoryos/current).
released() { [ -e "$R/opt/factoryos/current" ] || [ -L "$R/opt/factoryos/current" ]; }
on_exit() {
  local rc=$? msg cmd=""
  if [ "$rc" != 0 ]; then
    # The failed command: a network step's own, else its line as written ($BASH_COMMAND of a failed function is
    # its `return`; the line of a failure inside if/for is the block's last), else $BASH_COMMAND.
    if [ -n "$net_failed" ]; then cmd="net $net_failed"
    elif [ -n "$failed_line" ]; then cmd=$(sed -n "${failed_line}p" "$0" 2>/dev/null | sed -e 's/^ *//' -e 's/ *\\$//'); fi
    case "$cmd" in ""|fi|done|esac|"}") cmd=$failed_cmd ;; esac
    msg="first-boot setup failed (exit $rc, line ${failed_line:-?} of $(basename "$0")): ${cmd:0:200}"
    log "FAILED: $msg"
    # Only before a first release: a failed run by hand on a serving instance leaves its state alone.
    if ! released && command -v aws >/dev/null 2>&1 \
       && aws ssm put-parameter --region "$AWS_REGION" --name "$param" --type String --overwrite \
            --value "$(state_json failed "$msg" bootstrap.log)" >/dev/null 2>&1; then
      log "release state: failed"
    fi
  fi
  # shellcheck disable=SC2086 # $no_reboot: a list of unit names
  systemctl unmask --runtime $no_reboot >/dev/null 2>&1 || true
  log "reboots allowed again"
}
trap 'failed_line=$LINENO failed_cmd=$BASH_COMMAND' ERR
trap on_exit EXIT
# shellcheck disable=SC2086
systemctl mask --runtime $no_reboot >/dev/null 2>&1 || true
log "reboots held (until the end of the user data)"

export DEBIAN_FRONTEND=noninteractive NEEDRESTART_SUSPEND=1
export AWS_DEFAULT_REGION="$AWS_REGION"

# apt-get <args>, waiting for the apt/dpkg locks (deploy/provision.sh's helper): at first boot cloud-init,
# unattended-upgrades or a patch association hold them for minutes, and apt-get fails at once on them.
apt_get() {
  local out rc deadline=$(( $(date +%s) + ${FACTORYOS_APT_LOCK_WAIT:-900} ))
  while :; do
    rc=0
    out=$(apt-get -o DPkg::Lock::Timeout=60 -o Acquire::Retries=3 "$@" 2>&1) || rc=$?
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
    printf '%s\n' "$out" | tail -5 >&2
    return "$rc"
  done
}
# net <command…>: a step that needs the network, again every 15 s until it succeeds or the network deadline (the
# start + FACTORYOS_NET_WAIT) has passed.
net() {
  local i=1
  until "$@"; do
    [ "$(date +%s)" -lt "$net_deadline" ] || { net_failed="$*"; log "giving up (the network deadline has passed): $*"; return 1; }
    log "retry $i: $*"
    i=$(( i + 1 ))
    sleep "${FACTORYOS_RETRY_SLEEP:-15}"
  done
}
# download <url> <file>: bounded (a connection that stalls fails, and net tries again).
download() {
  curl --fail --location --show-error --silent --proto '=https' --tlsv1.2 \
    --connect-timeout 20 --speed-limit 1024 --speed-time 60 --max-time 900 -o "$2" "$1"
}

# --- 1. /etc/factoryos/stack.env: read by factoryos-release, the handoff and deploy/provision.sh (public mode).
{
  echo "FACTORYOS_STACK=$FACTORYOS_STACK"
  echo "AWS_REGION=$AWS_REGION"
  echo "AWS_DEFAULT_REGION=$AWS_REGION"
  echo "FACTORYOS_SOURCE_BUCKET=$FACTORYOS_SOURCE_BUCKET"
  echo "FACTORYOS_LOG_GROUP=$FACTORYOS_LOG_GROUP"
  echo "FACTORYOS_PUBLIC=1"
  # A re-run keeps what the handoffs added.
  grep -E '^(FACTORYOS_KIRO_SECRET|FACTORYOS_BEDROCK)=' "$R/etc/factoryos/stack.env" 2>/dev/null || true
} > "$R/etc/factoryos/stack.env.new"
chmod 0644 "$R/etc/factoryos/stack.env.new"
mv -f "$R/etc/factoryos/stack.env.new" "$R/etc/factoryos/stack.env"
log "stack.env written"

# --- 2. No ssh: SSM is the only way in (the security group has no ssh ingress either).
systemctl mask --now ssh.socket ssh.service >/dev/null 2>&1 || true
log "ssh masked"

# --- 3. Swap, a safety net for the release build's rustc / ld peaks.
if [ ! -f "$R/swapfile" ]; then
  fallocate -l 16G "$R/swapfile" 2>/dev/null || dd if=/dev/zero of="$R/swapfile" bs=1M count=16384 status=none
  chmod 600 "$R/swapfile"
  mkswap "$R/swapfile" >/dev/null
fi
grep -q '^/swapfile ' "$R/etc/fstab" 2>/dev/null || echo '/swapfile none swap sw 0 0' >> "$R/etc/fstab"
swapon --show=NAME --noheadings | grep -qx /swapfile || swapon "$R/swapfile"
sysctl -q -w vm.swappiness=10
echo 'vm.swappiness=10' > "$R/etc/sysctl.d/90-factoryos.conf"
log "swap: $(swapon --show=SIZE --noheadings | head -1)"

# --- 4. AWS CLI v2 and the CloudWatch agent. apt-get update fails on any index it cannot fetch (by default it only
# warns, and the install then fails on the AMI's stale lists, or on the archive it still cannot reach).
net apt_get update -y -qq --error-on=any
net apt_get install -y -qq --no-install-recommends ca-certificates curl unzip python3 >/dev/null
case "$(dpkg --print-architecture)" in
  arm64) cli_arch=aarch64; cw_arch=arm64 ;;
  amd64) cli_arch=x86_64;  cw_arch=amd64 ;;
  *) echo "user-data: unsupported architecture $(dpkg --print-architecture)" >&2; exit 1 ;;
esac
if ! "$R/usr/local/bin/aws" --version 2>/dev/null | grep -q '^aws-cli/2\.'; then
  tmp=$(mktemp -d)
  net download "https://awscli.amazonaws.com/awscli-exe-linux-${cli_arch}.zip" "$tmp/awscliv2.zip"
  unzip -q "$tmp/awscliv2.zip" -d "$tmp"
  "$tmp/aws/install" --update
  rm -rf "$tmp"
fi
log "$("$R/usr/local/bin/aws" --version)"

cw_ctl="$R/opt/aws/amazon-cloudwatch-agent/bin/amazon-cloudwatch-agent-ctl"
if [ ! -x "$cw_ctl" ]; then
  tmp=$(mktemp -d)
  net download "https://amazoncloudwatch-agent.s3.amazonaws.com/ubuntu/${cw_arch}/latest/amazon-cloudwatch-agent.deb" \
    "$tmp/amazon-cloudwatch-agent.deb"
  apt_get install -y -qq "$tmp/amazon-cloudwatch-agent.deb" >/dev/null
  rm -rf "$tmp"
fi
touch "$R/var/log/factoryos/release.log"
# One stream per file, named after this instance ({instance_id} is the agent's own placeholder).
cw_config="$R/opt/aws/amazon-cloudwatch-agent/etc/factoryos.json"
python3 -I - "$FACTORYOS_LOG_GROUP" "$cw_config" <<'PY'
import json, sys
group, path = sys.argv[1:]
files = [{"file_path": f"/var/log/factoryos/{name}", "log_group_name": group,
          "log_stream_name": "{instance_id}/" + name, "timezone": "UTC"}
         for name in ("bootstrap.log", "release.log")]
config = {"agent": {"run_as_user": "root"}, "logs": {"logs_collected": {"files": {"collect_list": files}}}}
with open(path, "w") as out:
    json.dump(config, out, indent=2)
PY
timeout 300 "$cw_ctl" -a fetch-config -m ec2 -s -c "file:$cw_config" >/dev/null
log "CloudWatch agent: /var/log/factoryos/{bootstrap,release}.log -> $FACTORYOS_LOG_GROUP"

# --- 5. The release state: `waiting` until the installer's first release step (factoryos-release writes it then).
waiting=$(state_json waiting "")
put_waiting() { # [--overwrite]
  aws ssm put-parameter --region "$AWS_REGION" --name "$param" --type String --value "$waiting" "$@" >/dev/null
}
existing="" err=$(mktemp)
for i in 1 2 3; do
  existing=$(aws ssm get-parameter --region "$AWS_REGION" --name "$param" --query Parameter.Value --output text \
    2>"$err") && break
  existing=""
  grep -q ParameterNotFound "$err" && break
  sleep $(( i * 10 ))
done
if [ -z "$existing" ]; then
  if grep -q ParameterNotFound "$err" 2>/dev/null; then
    # Without --overwrite: created only if still missing.
    put_waiting 2>/dev/null || true
    log "release state: waiting"
  else
    log "WARNING: cannot read $param ($(tr '\n' ' ' < "$err")); not written"
  fi
else
  # owner: the instance its log stream names; first_boot_failed: this script's own failure (phase failed, no version).
  read -r owner first_boot_failed < <(python3 -I -c 'import json, sys
try:
    j = json.loads(sys.argv[1])
except ValueError:
    j = {}
j = j if isinstance(j, dict) else {}
print((j.get("logStream") or "").split("/")[0] or "-", int(j.get("phase") == "failed" and not j.get("version")))' "$existing")
  if [ "$owner" != - ] && [ "$owner" != "$instance_id" ] && ! released; then
    put_waiting --overwrite
    log "release state: waiting (it was instance $owner's)"
  elif [ "$first_boot_failed" = 1 ] && ! released; then
    put_waiting --overwrite
    log "release state: waiting (it was this instance's failed first boot)"
  else
    log "release state: kept (it is this instance's)"
  fi
fi
rm -f "$err"

date -u +%FT%TZ > "$R/var/lib/factoryos-release/bootstrap.done"
log "done"
