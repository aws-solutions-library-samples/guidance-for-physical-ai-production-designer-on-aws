#!/usr/bin/env bash
# One-click cleanup of the Guidance for Physical AI Production Designer on AWS (Automated Sample Code Validation):
# removes what deploy.sh installed. It takes no input: it reads the stack name deploy.sh recorded in the directory
# both are run from, and runs deployment/install.sh uninstall (both stacks, the instance and its data, the user pool,
# the source bucket, the release parameters, the log groups the Lambda functions left) of the tree this script is in
# when it is deployment/cleanup.sh of a checkout, else of deploy.sh's clone, else of a fresh clone of REPO_URL.
# The CDK bootstrap stacks stay: they are the account's.
#
#   bash cleanup.sh
# The CloudFormation installer (pai-pd-installer.yaml) names the stack and its Region with ONECLICK_STACK_NAME and
# ONECLICK_REGION instead of the state file (the same ONECLICK_* prefix as deploy.sh's overrides).
set -euo pipefail
export AWS_REGION="us-east-1"
[ -z "${ONECLICK_REGION:-}" ] || AWS_REGION="$ONECLICK_REGION"
export AWS_DEFAULT_REGION="$AWS_REGION"

# ---- Static parameters ---------------------------------------------------------------------------------------
# Written by deploy.sh (in the directory it was run from).
STATE_FILE="${ONECLICK_STATE_FILE:-$PWD/factoryos-oneclick.state}"
REPO_URL="https://github.com/aws-solutions-library-samples/guidance-for-physical-ai-production-designer-on-aws.git"
# The edge stack (the sign-in function) is in the CloudFront Region (Lambda@Edge); CloudFront deletes its Lambda@Edge replicas only some time
# after the distribution is gone, so its delete is retried every 15 minutes for up to 2 hours.
EDGE_REGION="us-east-1"
EDGE_RETRY_EVERY_SECONDS=900
EDGE_RETRY_FOR_SECONDS=7200
# --------------------------------------------------------------------------------------------------------------

step() { printf '\n==> [%s/6] %s\n' "$1" "$2"; }
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

# stack_state <name> <region>: the stack's status, or NONE when it does not exist.
stack_state() {
  local out
  if out="$(aws cloudformation describe-stacks --stack-name "$1" --region "$2" \
              --query 'Stacks[0].StackStatus' --output text 2>&1)"; then
    echo "$out"
  elif [[ "$out" == *"does not exist"* ]]; then
    echo NONE
  else
    fail "describe-stacks $1 ($2): $out"
  fi
}

if [ -n "${ONECLICK_STACK_NAME:-}" ]; then
  step 1 "Taking the stack name from ONECLICK_STACK_NAME"
  STACK_NAME="$ONECLICK_STACK_NAME"
  CLONE_DIR=""
  NAME_FROM=ONECLICK_STACK_NAME
else
  step 1 "Reading the stack name from $STATE_FILE"
  [ -f "$STATE_FILE" ] || fail "$STATE_FILE is missing: run cleanup.sh from the directory deploy.sh ran in"
  STACK_NAME="$(sed -n 's/^STACK_NAME=//p' "$STATE_FILE" | head -1)"
  CLONE_DIR="$(sed -n 's/^CLONE_DIR=//p' "$STATE_FILE" | head -1)"
  NAME_FROM="$STATE_FILE"
fi
[[ "$STACK_NAME" =~ ^[a-z][a-z0-9-]{0,30}$ ]] || fail "no usable stack name ('$STACK_NAME', from $NAME_FROM)"
EDGE_STACK="$STACK_NAME-edge"
echo "stack $STACK_NAME in $AWS_REGION, $EDGE_STACK in $EDGE_REGION"

step 2 "Checking the tools and the account"
for t in aws git python3; do
  command -v "$t" >/dev/null 2>&1 || fail "$t is not installed"
done
aws_version="$(aws --version 2>&1)"
[[ "$aws_version" == aws-cli/2.* ]] || fail "the AWS CLI v2 is needed (found $aws_version)"
ACCOUNT_ID="$(aws sts get-caller-identity --query Account --output text)"
[[ "$ACCOUNT_ID" =~ ^[0-9]{12}$ ]] || fail "no AWS account from sts get-caller-identity: $ACCOUNT_ID"
echo "account $ACCOUNT_ID"

step 3 "Getting the installer"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$(basename "$here")" = deployment ] && [ -x "$here/install.sh" ]; then
  CLONE_DIR="$(dirname "$here")"
  echo "using the tree this script is in: $CLONE_DIR"
elif [ -n "$CLONE_DIR" ] && [ -x "$CLONE_DIR/deployment/install.sh" ]; then
  echo "using deploy.sh's clone $CLONE_DIR"
else
  CLONE_DIR="$(mktemp -d "${TMPDIR:-/tmp}/factoryos-oneclick.XXXXXX")/repo"
  git clone --depth 1 "$REPO_URL" "$CLONE_DIR"
fi
cd "$CLONE_DIR"
[ -x deployment/install.sh ] || fail "$CLONE_DIR has no executable deployment/install.sh"

step 4 "Uninstalling (deployment/install.sh uninstall)"
rc=0
./deployment/install.sh uninstall --yes --confirm-stack "$STACK_NAME" --region "$AWS_REGION" \
  --stack-name "$STACK_NAME" </dev/null || rc=$?
echo "install.sh uninstall exited $rc"

step 5 "Retrying the edge stack while CloudFront removes the Lambda@Edge replicas"
main="$(stack_state "$STACK_NAME" "$AWS_REGION")"
edge="$(stack_state "$EDGE_STACK" "$EDGE_REGION")"
if [ "$rc" != 0 ] && [ "$main" != NONE ]; then
  fail "the uninstall failed and $STACK_NAME is still there ($main): see its events in the CloudFormation console"
fi
if [ "$edge" = NONE ]; then
  echo "not needed: $EDGE_STACK is gone"
else
  echo "$EDGE_STACK is $edge: retrying every $((EDGE_RETRY_EVERY_SECONDS / 60)) min for up to $((EDGE_RETRY_FOR_SECONDS / 60)) min"
  attempts=$((EDGE_RETRY_FOR_SECONDS / EDGE_RETRY_EVERY_SECONDS))
  attempt=0
  while [ "$edge" != NONE ]; do
    [ "$attempt" -lt "$attempts" ] || fail "$EDGE_STACK is still there ($edge) after $((EDGE_RETRY_FOR_SECONDS / 60)) min"
    attempt=$((attempt + 1))
    echo "waiting $((EDGE_RETRY_EVERY_SECONDS / 60)) min (attempt $attempt of $attempts)"
    sleep "$EDGE_RETRY_EVERY_SECONDS"
    ./deployment/install.sh uninstall --edge-only --yes --confirm-stack "$STACK_NAME" --region "$AWS_REGION" \
      --stack-name "$STACK_NAME" </dev/null || echo "install.sh uninstall --edge-only exited $?"
    edge="$(stack_state "$EDGE_STACK" "$EDGE_REGION")"
  done
  echo "$EDGE_STACK deleted"
fi

step 6 "Verifying that both stacks are gone"
main="$(stack_state "$STACK_NAME" "$AWS_REGION")"
edge="$(stack_state "$EDGE_STACK" "$EDGE_REGION")"
[ "$main" = NONE ] || fail "$STACK_NAME is still there ($main)"
[ "$edge" = NONE ] || fail "$EDGE_STACK is still there ($edge)"
rm -f "$STATE_FILE"
echo "Cleaned up: $STACK_NAME ($AWS_REGION) and $EDGE_STACK ($EDGE_REGION) are deleted."
