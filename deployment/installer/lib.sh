# shellcheck shell=bash
# deployment/installer/lib.sh: argument parsing, output, prompts and small helpers of install.sh.
# Sourced by install.sh (and the tests); defines functions only, except for the defaults below.
# shellcheck disable=SC2034 # the globals set here (defaults, parse_args) are read by the other installer/ parts

STACK="factoryos"
REGION=""
EDGE_REGION="us-east-1"
ASSUME_YES=0
SOURCE_URI=""
FROM_CHECKOUT=0
CONFIRM_STACK=""
BEDROCK_MODELS=""
USERS=""
INSTANCE_TYPE="m7g.2xlarge"
VOLUME_GB="200"
BEDROCK="false"
KIRO_SECRET=""
UMARELL_TARBALL=""
# --umarell-jam-tarball: what was given (s3:// or a local file; '' drops a recorded one), then (release.sh) the staged
# copy, its sha256 and its key in the source bucket; an update without the option keeps the recorded key and sha256.
UMARELL_JAM_TARBALL=""
UMARELL_JAM_FILE=""
UMARELL_JAM_SHA=""
UMARELL_JAM_KEY=""
# --build-project/--build-bucket: the release is built in that AWS CodeBuild project (the CloudFormation installer's
# release-build project), its source staged in that bucket; '' builds it on the instance.
BUILD_PROJECT=""
BUILD_BUCKET=""
FORCE=0
ROTATE_ORIGIN=0
ALLOW_DOWNGRADE=0
ALLOW_REPLACE=0
WAIT=0
SNAPSHOT=0
EDGE_ONLY=0
COMMAND=""
ARGS=()
# The options given on this command line (an update keeps the installed value of every other one).
SET_OPTS=""
WORK_ROOT="${FACTORYOS_WORK:-/tmp/factoryos-install}"
POLL_SECONDS="${FACTORYOS_POLL_SECONDS:-60}"

if [ -t 1 ]; then B=$'\033[1m'; R=$'\033[31m'; G=$'\033[32m'; Y=$'\033[33m'; N=$'\033[0m'; else B=""; R=""; G=""; Y=""; N=""; fi
say()  { printf '%s\n' "$*"; }
step() { printf '\n%s==> %s%s\n' "$B" "$*" "$N"; }
ok()   { printf '%s  ok%s  %s\n' "$G" "$N" "$*"; }
warn() { printf '%sWARN%s  %s\n' "$Y" "$N" "$*" >&2; }
die()  { printf '%sERROR%s %s\n' "$R" "$N" "$*" >&2; exit 1; }

usage() { sed -n '2,/^set -euo/p' "$INSTALLER_DIR/install.sh" | sed -e '$d' -e 's/^# \{0,1\}//'; }

# confirm <question>: yes unless --yes was given or the answer is y/yes.
confirm() {
  [ "$ASSUME_YES" = 1 ] && return 0
  [ -t 0 ] || die "$1 — no terminal to ask on; rerun with --yes"
  local a; read -r -p "$1 [y/N] " a
  case "$a" in y|Y|yes|YES) return 0 ;; *) die "stopped" ;; esac
}

# confirm_typed <what> <expected>: the user must type <expected> (no --yes for this).
confirm_typed() {
  [ -t 0 ] || [ "${FACTORYOS_TEST_TYPED:-}" = "$2" ] || die "$1: needs a terminal to confirm on"
  local a
  if [ -n "${FACTORYOS_TEST_TYPED:-}" ]; then a="$FACTORYOS_TEST_TYPED"; else read -r -p "$1 Type '$2' to confirm: " a; fi
  [ "$a" = "$2" ] || die "not confirmed"
}

# aws in the stack's region; aws_edge in us-east-1.
awsr()  { aws --region "$REGION" "$@"; }
awse()  { aws --region "$EDGE_REGION" "$@"; }

# json <python expression over j> < json: one value of a JSON document (python3 -I: nothing from the cwd).
json() { python3 -I -c 'import json,sys
j=json.load(sys.stdin)
v=eval(sys.argv[1],{"j":j})
print("" if v is None else (json.dumps(v) if isinstance(v,(dict,list)) else v))' "$1"; }

now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }

sha256_file() { if command -v sha256sum >/dev/null; then sha256sum "$1" | cut -d' ' -f1; else shasum -a 256 "$1" | cut -d' ' -f1; fi; }

# version_cmp <a> <b>: prints -1, 0 or 1 for vX.Y.Z versions; "?" when either is not one (dev builds).
version_cmp() {
  local re='^v?([0-9]+)\.([0-9]+)\.([0-9]+)$'
  [[ "$1" =~ $re ]] || { echo "?"; return; }; local a=("${BASH_REMATCH[@]:1}")
  [[ "$2" =~ $re ]] || { echo "?"; return; }; local b=("${BASH_REMATCH[@]:1}")
  local i; for i in 0 1 2; do
    if (( 10#${a[i]} < 10#${b[i]} )); then echo -1; return; fi
    if (( 10#${a[i]} > 10#${b[i]} )); then echo 1; return; fi
  done
  echo 0
}

parse_args() {
  ARGS=()
  case "${1:-}" in
    install|status|add-user|remove-user|uninstall) COMMAND="$1"; shift ;;
    -h|--help|help|"") COMMAND="help" ;;
    s3://*|--from-checkout) COMMAND="install" ;;
    *) die "unknown command '$1' (install.sh --help)" ;;
  esac
  while [ $# -gt 0 ]; do
    case "$1" in
      --region) REGION="${2:?--region needs a value}"; shift 2 ;;
      --stack|--stack-name) STACK="${2:?$1 needs a value}"; shift 2 ;;
      --from-checkout) FROM_CHECKOUT=1; shift ;;
      --confirm-stack) CONFIRM_STACK="${2:?--confirm-stack needs the stack name}"; shift 2 ;;
      --bedrock-models) BEDROCK_MODELS="${2:?--bedrock-models needs a value}"; SET_OPTS+=" bedrockModelIds"; shift 2 ;;
      --users) USERS="${2:?--users needs a value}"; shift 2 ;;
      --instance-type) INSTANCE_TYPE="${2:?}"; SET_OPTS+=" instanceType"; shift 2 ;;
      --volume-gb) VOLUME_GB="${2:?}"; SET_OPTS+=" volumeGb"; shift 2 ;;
      --bedrock) BEDROCK="true"; SET_OPTS+=" bedrock"; shift ;;
      --no-bedrock) BEDROCK="false"; SET_OPTS+=" bedrock"; shift ;;
      --kiro-secret) KIRO_SECRET="${2?}"; SET_OPTS+=" kiroSecretName"; shift 2 ;;
      --umarell-tarball) UMARELL_TARBALL="${2:?}"; shift 2 ;;
      --umarell-jam-tarball) UMARELL_JAM_TARBALL="${2?--umarell-jam-tarball needs a value}"; SET_OPTS+=" umarellJam"; shift 2 ;;
      --build-project) BUILD_PROJECT="${2:?--build-project needs a value}"; shift 2 ;;
      --build-bucket) BUILD_BUCKET="${2:?--build-bucket needs a value}"; shift 2 ;;
      --force) FORCE=1; shift ;;
      --rotate-origin-secret) ROTATE_ORIGIN=1; shift ;;
      --allow-downgrade) ALLOW_DOWNGRADE=1; shift ;;
      --allow-replace) ALLOW_REPLACE=1; shift ;;
      --wait) WAIT=1; shift ;;
      --snapshot) SNAPSHOT=1; shift ;;
      --edge-only) EDGE_ONLY=1; shift ;;
      --yes|-y) ASSUME_YES=1; shift ;;
      -h|--help) COMMAND="help"; shift ;;
      --*) die "unknown option $1 (install.sh --help)" ;;
      *) ARGS+=("$1"); shift ;;
    esac
  done
  [ "$COMMAND" != help ] || return 0
  [[ "$STACK" =~ ^[a-z][a-z0-9-]{0,30}$ ]] || die "--stack must match [a-z][a-z0-9-]{0,30}"
  [[ "$STACK" =~ ^(aws|amazon|amzn) ]] && die "--stack must not start with aws, amazon or amzn (AWS reserves them)"
  [[ "$VOLUME_GB" =~ ^[0-9]+$ ]] && [ "$VOLUME_GB" -ge 100 ] || die "--volume-gb must be a number ≥ 100"
  [[ "$INSTANCE_TYPE" =~ ^[a-z][0-9]+g[a-z]*\.[0-9a-z]+$ ]] || die "--instance-type must be a Graviton type (e.g. m7g.2xlarge)"
  [ "$FROM_CHECKOUT" = 0 ] || [ "$COMMAND" = install ] || die "--from-checkout is an option of install"
  [ -z "$CONFIRM_STACK" ] || [ "$COMMAND" = uninstall ] || die "--confirm-stack is an option of uninstall"
  [ "$ROTATE_ORIGIN" = 0 ] || [ "$COMMAND" = install ] || die "--rotate-origin-secret is an option of install"
  [ -z "$CONFIRM_STACK" ] || [ "$CONFIRM_STACK" = "$STACK" ] \
    || die "--confirm-stack '$CONFIRM_STACK' is not the stack name '$STACK': nothing deleted"
  if [ -n "$BEDROCK_MODELS" ]; then
    # The CDK app's own check (lib/config.ts INFERENCE_PROFILE_ID, MAX_BEDROCK_MODELS), early: no ARNs, no wildcards.
    local id ids n=0 profile_re='^(us|eu|apac|global|jp|au|ca|us-gov)\.[a-z0-9]+\.[a-z0-9][a-z0-9.-]*(:[0-9]+)?$'
    IFS=, read -ra ids <<<"$BEDROCK_MODELS"
    for id in "${ids[@]}"; do
      [[ "$id" =~ $profile_re ]] || die "--bedrock-models: '$id' is not an inference profile id (such as us.<provider>.<model>; no ARNs, no wildcards)"
      n=$((n + 1))
    done
    [ "$n" -ge 1 ] && [ "$n" -le 20 ] || die "--bedrock-models takes 1 to 20 comma-separated inference profile ids"
    [ "$(printf '%s\n' "${ids[@]}" | sort -u | wc -l | tr -d ' ')" = "$n" ] || die "--bedrock-models lists an id twice"
    [[ " $SET_OPTS " == *" bedrock "* ]] && [ "$BEDROCK" = false ] && die "--bedrock-models contradicts --no-bedrock"
    BEDROCK="true"; SET_OPTS+=" bedrock"
  fi
  if [ "$COMMAND" = install ]; then
    if [ "$FROM_CHECKOUT" = 1 ]; then
      [ "${#ARGS[@]}" = 0 ] || die "install --from-checkout takes no export URI (it packages the checkout install.sh runs from)"
    else
      [ "${#ARGS[@]}" = 1 ] || die "install needs exactly one s3:// URI of the export (or --from-checkout)"
      SOURCE_URI="${ARGS[0]}"
      [[ "$SOURCE_URI" =~ ^s3://[a-z0-9.-]{3,63}/.+\.(zip|tar\.gz|tgz)$ ]] || die "the export must be an s3://bucket/key.zip (or .tar.gz)"
    fi
    [ -z "$UMARELL_TARBALL" ] || [[ "$UMARELL_TARBALL" =~ ^s3://[a-z0-9.-]{3,63}/.+\.tar\.gz$ ]] \
      || die "--umarell-tarball must be an s3://bucket/key.tar.gz"
    [ -z "$UMARELL_JAM_TARBALL" ] || [[ "$UMARELL_JAM_TARBALL" =~ ^s3://[a-z0-9.-]{3,63}/.+\.tar\.gz$ ]] \
      || { [[ "$UMARELL_JAM_TARBALL" != s3://* ]] && [[ "$UMARELL_JAM_TARBALL" == *.tar.gz ]] && [ -f "$UMARELL_JAM_TARBALL" ]; } \
      || die "--umarell-jam-tarball must be an s3://bucket/key.tar.gz or a local .tar.gz file ('' drops the installed one)"
  fi
  [[ " $SET_OPTS " != *" umarellJam "* ]] || [ "$COMMAND" = install ] || die "--umarell-jam-tarball is an option of install"
  if [ -n "$BUILD_PROJECT$BUILD_BUCKET" ]; then
    [ "$COMMAND" = install ] || die "--build-project and --build-bucket are options of install"
    [ -n "$BUILD_PROJECT" ] && [ -n "$BUILD_BUCKET" ] || die "--build-project and --build-bucket go together"
    [[ "$BUILD_PROJECT" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{1,254}$ ]] || die "--build-project must be a CodeBuild project name"
    [[ "$BUILD_BUCKET" =~ ^[a-z0-9][a-z0-9.-]{1,61}[a-z0-9]$ ]] || die "--build-bucket must be an S3 bucket name"
  fi
  return 0
}

main() {
  parse_args "$@"
  case "$COMMAND" in
    help) usage ;;
    install) preflight; cmd_install ;;
    status) preflight; cmd_status ;;
    add-user) preflight; [ "${#ARGS[@]}" -ge 1 ] || die "add-user needs an e-mail address"; cmd_add_users "${ARGS[@]}" ;;
    remove-user) preflight; [ "${#ARGS[@]}" -ge 1 ] || die "remove-user needs an e-mail address"; cmd_remove_users "${ARGS[@]}" ;;
    uninstall) preflight; cmd_uninstall ;;
  esac
}
