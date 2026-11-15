# shellcheck shell=bash
# deployment/installer/preflight.sh: what every command checks first (tools, account, region, that no installation
# of the same stack name in another region owns the sign-in at the edge), and the state of the two stacks.

ACCOUNT=""
CALLER=""

preflight() {
  local t missing=()
  for t in aws python3; do command -v "$t" >/dev/null || missing+=("$t"); done
  if [ "$COMMAND" = install ]; then
    for t in node npm unzip tar; do command -v "$t" >/dev/null || missing+=("$t"); done
    if [ "$FROM_CHECKOUT" = 1 ] && [ -e "$INSTALLER_DIR/../.git" ]; then command -v git >/dev/null || missing+=(git); fi
  fi
  [ "${#missing[@]}" = 0 ] || die "missing tools: ${missing[*]} (AWS CloudShell has them all)"
  aws --version 2>&1 | grep -q '^aws-cli/2\.' || die "the AWS CLI v2 is needed ($(aws --version 2>&1 | head -1))"
  if [ "$COMMAND" = install ]; then
    local nv; nv=$(node -p 'process.versions.node.split(".")[0]' 2>/dev/null || echo 0)
    [ "$nv" -ge 20 ] || die "Node.js 20 or newer is needed (found $(node --version 2>/dev/null || echo none))"
  fi

  if [ -z "$REGION" ]; then
    REGION="${AWS_REGION:-${AWS_DEFAULT_REGION:-}}"
    [ -n "$REGION" ] || REGION=$(aws configure get region 2>/dev/null || true)
    [ -n "$REGION" ] || die "no region: pass --region <region>"
    REGION_FROM="from \$AWS_REGION/your configuration — pass --region to choose another"
  else
    REGION_FROM="--region"
  fi
  [[ "$REGION" =~ ^[a-z]{2}(-[a-z]+)+-[0-9]$ ]] || die "not a region: $REGION"

  local id
  id=$(aws sts get-caller-identity --output json 2>&1) || die "no AWS credentials: $id"
  ACCOUNT=$(json 'j["Account"]' <<<"$id")
  CALLER=$(json 'j["Arn"]' <<<"$id")
  say "${B}Account${N}  $ACCOUNT  ($CALLER)"
  say "${B}Region${N}   $REGION  ($REGION_FROM)"
  say "${B}Stack${N}    $STACK  (and $STACK-edge in $EDGE_REGION for the sign-in at the edge)"
  [ "${AWS_EXECUTION_ENV:-}" = CloudShell ] && say "Running in AWS CloudShell."
  if [ "$REGION_FROM" != --region ] && [ "$COMMAND" != status ]; then
    # The console's region is easy to get wrong, and an install goes where the region says.
    warn "the region $REGION comes from \$AWS_REGION or your configuration, not from --region"
    confirm "Use the region $REGION?"
  fi
  check_edge_owner
  return 0
}

# check_edge_owner: the sign-in at the edge (the stack $STACK-edge and its configuration /factoryos/$STACK/edge-config,
# both in us-east-1) exists once per stack name and account. If an installation of the same name in another region
# owns it, refuse: deploying here would point the first installation's sign-in at this one's user pool.
check_edge_owner() {
  local name="/factoryos/$STACK/edge-config" out owner
  if ! out=$(awse ssm get-parameter --name "$name" --query Parameter.Value --output text 2>&1); then
    grep -q ParameterNotFound <<<"$out" && return 0
    die "cannot read $name in $EDGE_REGION (it tells which region owns the stack name $STACK): $out"
  fi
  [ -n "$out" ] || return 0
  owner=$(json 'j.get("region") or ""' <<<"$out" 2>/dev/null) \
    || die "$name in $EDGE_REGION is not the JSON a FactoryOS stack writes; refusing to use the stack name $STACK"
  [ -n "$owner" ] || die "$name in $EDGE_REGION names no region; refusing to use the stack name $STACK"
  [ "$owner" = "$REGION" ] && return 0
  die "the stack name $STACK is installed in $owner already (its sign-in configuration $name in $EDGE_REGION says so).
    One stack name serves one region: use --region $owner to manage that installation, or choose another name for a
    second one, e.g. --stack-name $STACK-${REGION//-/}. (If $STACK no longer exists in $owner, the parameter is left over:
    check with 'aws cloudformation describe-stacks --region $owner --stack-name $STACK' before deleting it.)"
}

# stack_status <name> <region>: the stack's status, or NONE.
stack_status() {
  local out
  if out=$(aws --region "$2" cloudformation describe-stacks --stack-name "$1" --query 'Stacks[0].StackStatus' --output text 2>&1); then
    echo "$out"
  elif grep -q 'does not exist' <<<"$out"; then
    echo NONE
  else
    die "describe-stacks $1 ($2): $out"
  fi
}

# stack_output <key>: an output of the main stack ("" if missing).
stack_output() {
  awsr cloudformation describe-stacks --stack-name "$STACK" \
    --query "Stacks[0].Outputs[?OutputKey=='$1'].OutputValue | [0]" --output text 2>/dev/null | sed 's/^None$//'
}

# detect_mode: sets MODE to install | update, or stops with what is wrong.
MODE=""
MAIN_STATUS=""
EDGE_STATUS=""
# shellcheck disable=SC2034 # MODE is read by release.sh and infra.sh
detect_mode() {
  MAIN_STATUS=$(stack_status "$STACK" "$REGION")
  EDGE_STATUS=$(stack_status "$STACK-edge" "$EDGE_REGION")
  local healthy='^(CREATE_COMPLETE|UPDATE_COMPLETE|UPDATE_ROLLBACK_COMPLETE|IMPORT_COMPLETE)$'
  local busy='_IN_PROGRESS$'
  if [ "$MAIN_STATUS" = NONE ] && [ "$EDGE_STATUS" = NONE ]; then
    MODE=install
  elif [[ "$MAIN_STATUS" =~ $busy ]] || [[ "$EDGE_STATUS" =~ $busy ]]; then
    die "a stack operation is in progress ($STACK: $MAIN_STATUS, $STACK-edge: $EDGE_STATUS); wait for it to finish and run this again"
  elif [[ "$MAIN_STATUS" =~ $healthy ]] && [[ "$EDGE_STATUS" =~ $healthy ]]; then
    MODE=update
  elif [ "$MAIN_STATUS" = NONE ] && [[ "$EDGE_STATUS" =~ $healthy ]]; then
    # A first install that stopped after the edge stack (deployed first): the next deploy creates the main one.
    MODE=install
  else
    failed_stack_help
  fi
}

failed_stack_help() {
  local s r st
  for s in "$STACK:$REGION:$MAIN_STATUS" "$STACK-edge:$EDGE_REGION:$EDGE_STATUS"; do
    IFS=: read -r name r st <<<"$s"
    [ "$st" = NONE ] && continue
    [[ "$st" =~ (FAILED|ROLLBACK_COMPLETE)$ ]] || continue
    warn "$name ($r) is $st. Its last failures:"
    aws --region "$r" cloudformation describe-stack-events --stack-name "$name" \
      --query 'StackEvents[?contains(ResourceStatus, `FAILED`)].[Timestamp, LogicalResourceId, ResourceStatusReason] | [:5]' \
      --output text >&2 || true
  done
  if [ "$MAIN_STATUS" = ROLLBACK_COMPLETE ] || [ "$EDGE_STATUS" = ROLLBACK_COMPLETE ]; then
    die "a first install failed and was rolled back; it holds no data. Delete it and install again:
    $0 uninstall --stack $STACK --region $REGION"
  fi
  die "the installation is in a state this script does not repair ($STACK: $MAIN_STATUS, $STACK-edge: $EDGE_STATUS).
    See the stack events above (CloudFormation console), fix the cause, then run this again."
}
