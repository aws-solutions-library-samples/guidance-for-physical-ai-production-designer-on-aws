#!/usr/bin/env bash
# One-click deployment of the Guidance for Physical AI Production Designer on AWS (Automated Sample Code Validation).
# No input: the parameters are the static values below. Runs on AWS CodeBuild (Amazon Linux 2023, standard:5.0) or
# any bash (Linux, macOS) with Node.js 20+, the AWS CLI v2, git, python3, curl and credentials for the account:
#   bash deploy.sh        # installs into the account, waits for the first release; cleanup.sh removes it again
# install.sh install --from-checkout does the work (and the CDK bootstrap where missing), of the tree this is
# deployment/deploy.sh of (a clone, a CodeBuild source), else of a fresh clone of REPO_URL (a standalone copy).
# The static values below are the ASCV run's. The CloudFormation installer (pai-pd-installer.yaml) overrides them
# with ONECLICK_* variables (a prefix of their own: CodeBuild sets AWS_REGION, which must not move an ASCV run).
set -euo pipefail
export AWS_REGION="us-east-1"
[ -z "${ONECLICK_REGION:-}" ] || AWS_REGION="$ONECLICK_REGION"
export AWS_DEFAULT_REGION="$AWS_REGION"
# This script adds the CDKToolkit and umarell checks, the gate checks and the record of the stack name for cleanup.sh.

# ---- Static parameters ---------------------------------------------------------------------------------------
# A new name for every run (the installer allows [a-z][a-z0-9-]{0,30}: this is 27 characters).
STACK_NAME="${ONECLICK_STACK_NAME:-factoryos-$(date -u +%Y%m%d%H%M)-$(od -An -N2 -tx1 /dev/urandom | tr -d ' \n')}"
# The first user (an invitation with a temporary password goes to this address; MFA is enrolled at first sign-in).
ADMIN_EMAIL="${ONECLICK_ADMIN_EMAIL:-admin@example.com}"
REPO_URL="https://github.com/aws-solutions-library-samples/guidance-for-physical-ai-production-designer-on-aws.git"
# 1: wait for the first release on the instance to finish (ready), and fail if it fails or rolls back.
WAIT_FOR_RELEASE="${ONECLICK_WAIT_FOR_RELEASE:-1}"
# How long to wait for that build, in seconds (6 h).
RELEASE_TIMEOUT_SECONDS="${ONECLICK_RELEASE_TIMEOUT_SECONDS:-21600}"
# Optional: umarell from an s3:// tarball instead of GitHub ('' for GitHub, the default: step 3 checks that the pinned
# public repository answers; an archive of the pinned commit is for an account without a route to GitHub).
UMARELL_TARBALL="${ONECLICK_UMARELL_TARBALL:-}"
# Optional: the plain umarell commit of the umarell_jam planner (deploy/umarell-jam.pin) from an s3:// tarball instead
# of GitHub ('' for GitHub, the default). When GitHub cannot be reached the install goes on: umarell_jam is
# marked incomplete on the instance and its jobs fall back to umarell_u3 (step 3 warns).
UMARELL_JAM_TARBALL="${ONECLICK_UMARELL_JAM_TARBALL:-}"
# Optional: the instance (--instance-type, a Graviton type), its volume (--volume-gb) and Amazon Bedrock (true/false;
# '' for the installer's defaults: m7g.2xlarge, 200 GB, no Bedrock).
INSTANCE_TYPE="${ONECLICK_INSTANCE_TYPE:-}"
VOLUME_GB="${ONECLICK_VOLUME_GB:-}"
BEDROCK="${ONECLICK_BEDROCK:-}"
# Optional: build the release in an AWS CodeBuild project (the CloudFormation installer's release-build project) with
# its source staged in a bucket, instead of on the instance (install.sh --build-project, --build-bucket; '' for the
# instance).
BUILD_PROJECT="${ONECLICK_BUILD_PROJECT:-}"
BUILD_BUCKET="${ONECLICK_BUILD_BUCKET:-}"
# The edge stack (the sign-in function) is always in the CloudFront Region (Lambda@Edge).
EDGE_REGION="us-east-1"
# Where cleanup.sh finds the stack name (the directory this script is run from).
STATE_FILE="${ONECLICK_STATE_FILE:-$PWD/factoryos-oneclick.state}"
# --------------------------------------------------------------------------------------------------------------

step() { printf '\n==> [%s/8] %s\n' "$1" "$2"; }
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

step 1 "Checking the tools (node >= 20, aws v2, git, python3, curl)"
for t in node npm aws git python3 curl; do
  command -v "$t" >/dev/null 2>&1 || fail "$t is not installed"
done
node_version="$(node --version)"
node_major="${node_version#v}"; node_major="${node_major%%.*}"
[[ "$node_major" =~ ^[0-9]+$ ]] && [ "$node_major" -ge 20 ] || fail "Node.js 20 or newer is needed (found $node_version)"
aws_version="$(aws --version 2>&1)"
[[ "$aws_version" == aws-cli/2.* ]] || fail "the AWS CLI v2 is needed (found $aws_version)"
echo "node $node_version, ${aws_version%% *}, $(git --version)"

step 2 "Getting the account"
ACCOUNT_ID="$(aws sts get-caller-identity --query Account --output text)"
[[ "$ACCOUNT_ID" =~ ^[0-9]{12}$ ]] || fail "no AWS account from sts get-caller-identity: $ACCOUNT_ID"
echo "account $ACCOUNT_ID, region $AWS_REGION, stack $STACK_NAME (and $STACK_NAME-edge in $EDGE_REGION)"

step 3 "Getting the installer and checking umarell"
# deployment/deploy.sh of a tree with the installer: install that tree (the revision under test), not REPO_URL's main.
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$(basename "$here")" = deployment ] && [ -x "$here/install.sh" ]; then
  CLONE_DIR="$(dirname "$here")"
  echo "installing the tree this script is in: $CLONE_DIR"
else
  WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/factoryos-oneclick.XXXXXX")"
  CLONE_DIR="$WORK_DIR/repo"
  echo "cloning $REPO_URL"
  git clone --depth 1 "$REPO_URL" "$CLONE_DIR"
fi
cd "$CLONE_DIR"
[ -x deployment/install.sh ] || fail "$CLONE_DIR has no executable deployment/install.sh"
echo "$CLONE_DIR at $(git rev-parse --short HEAD 2>/dev/null || echo '(not a git checkout)')"
if [ -z "$UMARELL_TARBALL" ]; then
  # The release build fetches umarell only at its end, hours from now: find out now that it cannot.
  pin=source/deploy/umarell.pin; [ -f "$pin" ] || pin=deploy/umarell.pin
  [ -f "$pin" ] || fail "no source/deploy/umarell.pin in $CLONE_DIR"
  umarell_commit="$(awk '!/^[[:space:]]*#/ && NF {print $1; exit}' "$pin")"
  umarell_url="$(awk '!/^[[:space:]]*#/ && NF {print $2; exit}' "$pin")"
  [ -n "$umarell_commit" ] && [ -n "$umarell_url" ] || fail "$pin has no '<commit> <url>' line"
  GIT_TERMINAL_PROMPT=0 git -c credential.helper= ls-remote "$umarell_url" >/dev/null 2>&1 \
    || fail "umarell's repository does not answer ($umarell_url): check the network, or set UMARELL_TARBALL to an s3:// archive of $umarell_commit"
  echo "umarell $umarell_commit from $umarell_url (public)"
fi
jam_pin=source/deploy/umarell-jam.pin; [ -f "$jam_pin" ] || jam_pin=deploy/umarell-jam.pin
if [ -z "$UMARELL_JAM_TARBALL" ] && [ -f "$jam_pin" ]; then
  # Optional, so only a warning: the release installs without umarell_jam's plain umarell when it cannot fetch it.
  jam_commit="$(awk '!/^[[:space:]]*#/ && NF {print $1; exit}' "$jam_pin")"
  jam_url="$(awk '!/^[[:space:]]*#/ && NF {print $2; exit}' "$jam_pin")"
  if [ -n "$jam_url" ] && [ "$jam_url" = "${umarell_url:-}" ]; then
    echo "umarell_jam's umarell $jam_commit from $jam_url (public)"
  elif [ -n "$jam_url" ] && GIT_TERMINAL_PROMPT=0 git -c credential.helper= ls-remote "$jam_url" >/dev/null 2>&1; then
    echo "umarell_jam's umarell $jam_commit from $jam_url (public)"
  else
    echo "WARNING: umarell_jam's umarell (${jam_url:-no URL in $jam_pin}) does not answer: the release installs without umarell_jam (marked incomplete, its jobs fall back to umarell_u3) unless UMARELL_JAM_TARBALL is an s3:// archive of ${jam_commit:-its commit}" >&2
  fi
fi

if [ "$EDGE_REGION" = "$AWS_REGION" ]; then where="$AWS_REGION"; else where="$AWS_REGION and $EDGE_REGION"; fi
step 4 "Checking the CDK bootstrap (CDKToolkit) in $where"
# Only checked here: install.sh bootstraps a Region where the bootstrap is missing, with the app's context (a plain
# 'cdk bootstrap' in deployment/ synthesizes the app first, and that fails without the installer's context).
regions=("$AWS_REGION")
[ "$EDGE_REGION" = "$AWS_REGION" ] || regions+=("$EDGE_REGION")
for region in "${regions[@]}"; do
  if out="$(aws cloudformation describe-stacks --stack-name CDKToolkit --region "$region" \
              --query 'Stacks[0].StackStatus' --output text 2>&1)"; then
    case "$out" in
      CREATE_COMPLETE|UPDATE_COMPLETE|UPDATE_ROLLBACK_COMPLETE|IMPORT_COMPLETE) echo "$region: CDKToolkit is $out" ;;
      *) fail "$region: the CDKToolkit stack is $out; fix or delete it, then rerun" ;;
    esac
  elif [[ "$out" == *"does not exist"* ]]; then
    echo "$region: no CDKToolkit stack (install.sh bootstraps it)"
  else
    fail "$region: describe-stacks CDKToolkit: $out"
  fi
done

step 5 "Installing (deployment/install.sh install --from-checkout)"
# Recorded first, so that cleanup.sh also removes a partial installation.
printf 'STACK_NAME=%s\nREGION=%s\nCLONE_DIR=%s\n' "$STACK_NAME" "$AWS_REGION" "$CLONE_DIR" > "$STATE_FILE"
echo "recorded the stack name in $STATE_FILE"
install_args=(install --from-checkout --yes --region "$AWS_REGION" --stack-name "$STACK_NAME" --users "$ADMIN_EMAIL")
[ -z "$UMARELL_TARBALL" ] || install_args+=(--umarell-tarball "$UMARELL_TARBALL")
[ -z "$UMARELL_JAM_TARBALL" ] || install_args+=(--umarell-jam-tarball "$UMARELL_JAM_TARBALL")
[ -z "$INSTANCE_TYPE" ] || install_args+=(--instance-type "$INSTANCE_TYPE")
[ -z "$VOLUME_GB" ] || install_args+=(--volume-gb "$VOLUME_GB")
[ -z "$BUILD_PROJECT" ] || install_args+=(--build-project "$BUILD_PROJECT" --build-bucket "$BUILD_BUCKET")
case "$BEDROCK" in
  true) install_args+=(--bedrock) ;; false) install_args+=(--no-bedrock) ;; '') ;;
  *) fail "ONECLICK_BEDROCK must be true, false or empty (is '$BEDROCK')" ;;
esac
./deployment/install.sh "${install_args[@]}" </dev/null

step 6 "Checking that the sign-in gate was verified (GateVerified) and denies an anonymous client"
gate="$(aws cloudformation describe-stacks --stack-name "$STACK_NAME" --region "$AWS_REGION" \
          --query "Stacks[0].Outputs[?OutputKey=='GateVerified'].OutputValue | [0]" --output text)"
[ "$gate" = "true" ] || fail "the stack output GateVerified is '$gate', not true"
url="$(aws cloudformation describe-stacks --stack-name "$STACK_NAME" --region "$AWS_REGION" \
         --query "Stacks[0].Outputs[?OutputKey=='Url'].OutputValue | [0]" --output text)"
[[ "$url" == https://* ]] || fail "the stack output Url is '$url'"
url="${url%/}"
api="$( { curl -sS -o /dev/null -D - --max-time 30 "$url/api/v1/health" || true; } | tr -d '\r')"
grep -qE '^HTTP/[0-9.]+ 401' <<<"$api" && grep -qi '^www-authenticate:.*realm="factoryos"' <<<"$api" \
  || fail "an anonymous GET of $url/api/v1/health did not get the gate's 401: $(head -1 <<<"$api")"
page="$( { curl -sS -o /dev/null -D - --max-time 30 -H 'Accept: text/html' "$url/" || true; } | tr -d '\r')"
grep -qE '^HTTP/[0-9.]+ 302' <<<"$page" && grep -qi '^location: https://[^/]*/oauth2/authorize' <<<"$page" \
  || fail "an anonymous GET of $url/ was not sent to the sign-in: $(head -1 <<<"$page")"
echo "GateVerified=true; anonymous: /api/v1/health 401, / 302 to the sign-in"

step 7 "Waiting for the release on the instance (install.sh status --wait, at most $((RELEASE_TIMEOUT_SECONDS / 3600)) h)"
if [ "$WAIT_FOR_RELEASE" = 1 ]; then
  # A deadline in bash (macOS has no timeout(1)): status --wait in the background, stopped at the deadline.
  ./deployment/install.sh status --wait --yes --region "$AWS_REGION" --stack-name "$STACK_NAME" </dev/null &
  wait_pid=$!
  deadline=$(( $(date +%s) + RELEASE_TIMEOUT_SECONDS ))
  while kill -0 "$wait_pid" 2>/dev/null; do
    if [ "$(date +%s)" -ge "$deadline" ]; then
      kill "$wait_pid" 2>/dev/null || true
      wait "$wait_pid" 2>/dev/null || true
      fail "the release was not ready after $((RELEASE_TIMEOUT_SECONDS / 3600)) h ($RELEASE_TIMEOUT_SECONDS s)"
    fi
    sleep 10
  done
  rc=0; wait "$wait_pid" || rc=$?
  [ "$rc" = 0 ] || fail "the release failed or was rolled back (install.sh status exited $rc)"
  echo "the release is ready"
else
  echo "skipped (WAIT_FOR_RELEASE=$WAIT_FOR_RELEASE)"
fi

step 8 "Recording the stack name for cleanup.sh"
printf 'STACK_NAME=%s\nREGION=%s\nCLONE_DIR=%s\n' "$STACK_NAME" "$AWS_REGION" "$CLONE_DIR" > "$STATE_FILE"
echo "$STATE_FILE: STACK_NAME=$STACK_NAME"
echo "Deployed: $url (stack $STACK_NAME in $AWS_REGION). Remove it with: bash cleanup.sh"
