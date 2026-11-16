# shellcheck shell=bash
# deployment/installer/uninstall.sh: deletes the installation (both stacks, the source bucket's contents, the
# release parameters, the origin secret's two SecureStrings, the log groups Lambda created for the stacks' functions outside the stacks), after saying what
# goes and a typed confirmation (or --confirm-stack <stack name>, for a run without a terminal, e.g.
# deployment/cleanup.sh). The CDK bootstrap stacks stay (they are the account's, shared with other CDK apps).

cmd_uninstall() {
  local main edge; main=$(stack_status "$STACK" "$REGION"); edge=$(stack_status "$STACK-edge" "$EDGE_REGION")
  # The stack's parameters outside it: an uninstall cut short after the stack was deleted (a closed CloudShell, a
  # reboot) leaves them, and a second run must still remove them (test install fxt-jam, 2026-11-16).
  local left=""; [ "$main" != NONE ] || [ "$EDGE_ONLY" = 1 ] || left=$(stack_params_left)
  if [ "$main" = NONE ] && [ "$edge" = NONE ] && [ -z "$left" ]; then say "Not installed ($STACK in $REGION)."; return 0; fi
  local iid bucket pool url
  if [ "$main" != NONE ]; then
    iid=$(stack_output InstanceId); bucket=$(stack_output SourceBucket); pool=$(stack_output UserPoolId); url=$(stack_output Url)
  fi
  step "Uninstall $STACK from $ACCOUNT"
  if [ "$EDGE_ONLY" = 0 ] && [ "$main" != NONE ]; then
    say "Deletes, in $REGION:"
    say "  - the stack $STACK ($main) — ${url:-no URL}"
    say "  - the instance ${iid:-?} and its volume: ${B}every cell, catalogue item, product and plan on it${N}"
    say "  - the user pool ${pool:-?} and all its users"
    say "  - the source bucket ${bucket:-?} and every export in it"
    say "  - the access-log bucket, the VPC, the NAT gateway, the log groups, the parameters"
    say "    /factoryos/$STACK/release/{wanted,state},"
    say "    /factoryos/$STACK/install-options and /factoryos/$STACK/origin-secret (and in $EDGE_REGION"
    say "    /factoryos/$STACK/edge-origin-secret)"
  fi
  [ -z "$left" ] || say "Deletes, in $REGION: the parameters an earlier uninstall left: $(echo $left)"
  [ "$edge" = NONE ] || say "Deletes, in $EDGE_REGION: the stack $STACK-edge ($edge, the sign-in function), and in every"
  [ "$edge" = NONE ] || say "  Region: the log groups of the sign-in function's Lambda@Edge replicas"
  [ "$SNAPSHOT" = 0 ] || say "Keeps: an EBS snapshot of the instance's volume (it costs storage until you delete it)"
  if [ -n "$CONFIRM_STACK" ]; then
    # parse_args checked it already; again here, next to what it allows.
    [ "$CONFIRM_STACK" = "$STACK" ] || die "--confirm-stack '$CONFIRM_STACK' is not the stack name '$STACK': nothing deleted"
    say "Confirmed by --confirm-stack $CONFIRM_STACK."
  else
    confirm_typed "This cannot be undone." "$STACK"
  fi

  if [ "$EDGE_ONLY" = 0 ] && [ "$main" != NONE ]; then
    if [ "$SNAPSHOT" = 1 ] && [ -n "$iid" ]; then
      local vol snap
      vol=$(awsr ec2 describe-instances --instance-ids "$iid" \
        --query 'Reservations[0].Instances[0].BlockDeviceMappings[0].Ebs.VolumeId' --output text)
      snap=$(awsr ec2 create-snapshot --volume-id "$vol" --description "factoryos $STACK before uninstall $(now_iso)" \
        --tag-specifications "ResourceType=snapshot,Tags=[{Key=factoryos:stack,Value=$STACK}]" --query SnapshotId --output text)
      say "Snapshot $snap of $vol: waiting for it to complete…"
      awsr ec2 wait snapshot-completed --snapshot-ids "$snap" || die "snapshot $snap did not complete; nothing deleted"
      ok "snapshot $snap"
    fi
    if [ -n "$bucket" ]; then empty_bucket "$bucket" || die "s3://$bucket could not be emptied: nothing deleted"; fi
    local fns lgs; fns=$(stack_functions "$REGION" "$STACK"); lgs=$(stack_log_groups "$REGION" "$STACK")
    step "Deleting $STACK"
    delete_main_stack || die "$STACK did not delete; see its events in the CloudFormation console"
    ok "$STACK deleted"
    delete_stack_params
    # The distribution is gone with the stack: nothing sends the edge's copy any more.
    delete_edge_origin_secret
    # The CDK's custom resource providers log to /aws/lambda/<function>, a group Lambda creates outside the stack.
    delete_function_logs "$REGION" "" $fns
    # The stack's own groups: a custom resource runs while the stack is deleted (EdgeConfig deletes its parameter),
    # and Lambda delivers that log line seconds later, after CloudFormation deleted the group — which re-creates it,
    # empty and without retention. Deleted after the wait, so the late write is already in.
    if [ -n "$lgs" ]; then
      sleep "${FACTORYOS_LOG_LAG_SECONDS:-30}"
      delete_log_groups "$REGION" $lgs
    fi
  fi
  if [ -n "$left" ]; then step "Deleting the parameters of $STACK"; delete_stack_params; fi
  if [ "$edge" != NONE ]; then
    local efns elogs
    efns=$(stack_functions "$EDGE_REGION" "$STACK-edge"); elogs=$(stack_log_groups "$EDGE_REGION" "$STACK-edge")
    step "Deleting $STACK-edge"
    awse cloudformation delete-stack --stack-name "$STACK-edge"
    if awse cloudformation wait stack-delete-complete --stack-name "$STACK-edge"; then
      ok "$STACK-edge deleted"
      delete_edge_origin_secret
      delete_function_logs "$EDGE_REGION" "" $efns
      # Lambda@Edge replicas log, in each Region CloudFront ran them in, to the function's custom log group (the
      # edge stack's EdgeAuthLogs: one name in every Region, lib/edge.ts; the stack deletes only its us-east-1 one),
      # or, for an installation from before it, to /aws/lambda/<edge Region>.<function>.
      local r regions
      regions=$(awse ec2 describe-regions --query 'Regions[].RegionName' --output text 2>/dev/null) \
        || warn "cannot list the Regions (ec2:DescribeRegions): delete the log groups $(echo $elogs)" \
             "/aws/lambda/$EDGE_REGION.$STACK-edge-* in every Region yourself"
      for r in $regions; do
        delete_function_logs "$r" "$EDGE_REGION." $efns
        delete_log_groups "$r" $elogs
      done
    else
      warn "$STACK-edge could not be deleted yet: CloudFront removes the sign-in function's replicas only some time"
      warn "after the distribution is gone (up to a few hours). Run this again later:"
      die "  $0 uninstall --edge-only --stack $STACK --region $REGION${CONFIRM_STACK:+ --confirm-stack $CONFIRM_STACK}"
    fi
  fi
  ok "uninstalled"
}

# stack_params_left: those of the stack's parameters in $REGION that exist (names only, nothing decrypted).
stack_params_left() {
  awsr ssm get-parameters --names "$(param_wanted)" "$(param_state)" "$(param_options)" "$(param_origin)" \
    --query 'Parameters[].Name' --output text 2>/dev/null | tr '\t' '\n' | grep -E '^/factoryos/' || true
}

# delete_stack_params: the stack's parameters in $REGION (gone is fine).
delete_stack_params() {
  local p err
  for p in "$(param_wanted)" "$(param_state)" "$(param_options)" "$(param_origin)"; do
    err=$(awsr ssm delete-parameter --name "$p" 2>&1 >/dev/null) || [[ "$err" == *ParameterNotFound* ]] \
      || warn "could not delete the parameter $p: $err"
  done
}

# delete_edge_origin_secret: the edge's SecureString in us-east-1 (also on an --edge-only retry; gone is fine).
delete_edge_origin_secret() {
  local err
  err=$(awse ssm delete-parameter --name "$(param_edge_origin)" 2>&1 >/dev/null) || [[ "$err" == *ParameterNotFound* ]] \
    || warn "could not delete the parameter $(param_edge_origin) in $EDGE_REGION: $err"
}

# stack_log_groups <region> <stack>: the names of the log groups the stack owns (read before it is deleted). Read
# rather than derived from the stack name: a prefix of it would also match another installation's groups
# (<stack>-... matches <stack>-b-...).
stack_log_groups() {
  aws --region "$1" cloudformation list-stack-resources --stack-name "$2" \
    --query "StackResourceSummaries[?ResourceType=='AWS::Logs::LogGroup'].PhysicalResourceId" --output text 2>/dev/null \
    | tr '\t ' '\n\n' | grep -E '^[A-Za-z0-9_./#-]+$' || true
}

# delete_log_groups <region> <group>...: deletes each of them where it exists.
delete_log_groups() {
  local r="$1" g err; shift
  for g in "$@"; do
    if err=$(aws --region "$r" logs delete-log-group --log-group-name "$g" 2>&1 >/dev/null); then ok "log group $g ($r)"
    elif [[ "$err" != *ResourceNotFoundException* ]]; then warn "could not delete the log group $g in $r: $err"
    fi
  done
}

# stack_functions <region> <stack>: the names of the stack's Lambda functions (read before the stack is deleted).
stack_functions() {
  aws --region "$1" cloudformation list-stack-resources --stack-name "$2" \
    --query "StackResourceSummaries[?ResourceType=='AWS::Lambda::Function'].PhysicalResourceId" --output text 2>/dev/null \
    | tr '\t ' '\n\n' | grep -E '^[A-Za-z0-9_-]{1,64}$' || true
}

# delete_function_logs <region> <name prefix> <function>...: deletes /aws/lambda/<prefix><function> in the Region
# where it exists (a group that does not exist is fine: those functions log to a group of the stack, or never ran).
delete_function_logs() {
  local r="$1" pre="$2" f; shift 2
  for f in "$@"; do delete_log_groups "$r" "/aws/lambda/$pre$f"; done
}

# delete_main_stack: deletes $STACK and waits. A bucket of the stack can still fail to delete, not empty: the access
# logs of the distribution and of the source bucket arrive late (CloudFront and S3 deliver them minutes to hours
# after the request), after the stack's auto-delete emptied the bucket and its bucket policy, which denied the writes
# meanwhile, went with the stack. Then, once: the writes to exactly that bucket are denied, it is emptied, and the
# stack is deleted again. Only the buckets the stack itself names as DELETE_FAILED, and only one carrying the stack's
# aws:cloudformation:stack-name tag: never another bucket.
delete_main_stack() {
  awsr cloudformation delete-stack --stack-name "$STACK" || return 1
  awsr cloudformation wait stack-delete-complete --stack-name "$STACK" && return 0
  local failed b
  failed=$(failed_stack_buckets "$STACK")
  if [ -z "$failed" ]; then warn "$STACK: DELETE_FAILED, and not on one of its buckets"; return 1; fi
  # The distribution writes into the access-log bucket until it is gone; CloudFormation deletes it before the bucket
  # (it depends on it), so it is gone here unless that delete failed too, which a second delete does not change.
  local dist; dist=$(stack_resources "$STACK" "AWS::CloudFront::Distribution" "ResourceStatus!='DELETE_COMPLETE'")
  if [ -n "$dist" ]; then warn "$STACK: the distribution $dist is not deleted: not retrying"; return 1; fi
  for b in $failed; do
    if [ "$(bucket_stack_tag "$b")" != "$STACK" ]; then
      warn "s3://$b: not tagged aws:cloudformation:stack-name=$STACK: not emptying it"; return 1
    fi
    warn "s3://$b did not delete, not empty (logs that arrived late): denying writes, emptying it, deleting again"
    deny_writes "$b"
    empty_bucket "$b" || return 1
  done
  step "Deleting $STACK again"
  awsr cloudformation delete-stack --stack-name "$STACK" || return 1
  awsr cloudformation wait stack-delete-complete --stack-name "$STACK"
}

# stack_resources <stack> <type> [<JMESPath condition>]: the physical ids of the stack's resources of <type>.
stack_resources() {
  local q="StackResourceSummaries[?ResourceType=='$2'${3:+ && $3}].PhysicalResourceId"
  awsr cloudformation list-stack-resources --stack-name "$1" --query "$q" --output text 2>/dev/null \
    | tr '\t ' '\n\n' | grep -v -x -e '' -e None || true
}

# failed_stack_buckets <stack>: the names of the stack's buckets in DELETE_FAILED (valid bucket names only).
failed_stack_buckets() {
  stack_resources "$1" "AWS::S3::Bucket" "ResourceStatus=='DELETE_FAILED'" | grep -E '^[a-z0-9][a-z0-9.-]{1,61}[a-z0-9]$' || true
}

# bucket_stack_tag <bucket>: its aws:cloudformation:stack-name tag (CloudFormation tags every bucket it creates).
bucket_stack_tag() {
  awsr s3api get-bucket-tagging --bucket "$1" --query "TagSet[?Key=='aws:cloudformation:stack-name'].Value | [0]" \
    --output text 2>/dev/null || true
}

# deny_writes <bucket>: adds a statement denying s3:PutObject to everyone to its policy (as CDK's auto-delete does
# before it empties a bucket), so a late log delivery cannot refill it; a warning only if that is not possible.
deny_writes() {
  local cur err policy
  cur=$(awsr s3api get-bucket-policy --bucket "$1" --query Policy --output text 2>&1) || {
    [[ "$cur" == *NoSuchBucketPolicy* ]] || { warn "cannot read the policy of s3://$1 (writes not denied): $cur"; return 0; }
    cur=""
  }
  policy=$(python3 -I -c 'import json,sys
p=json.loads(sys.argv[2]) if sys.argv[2].strip() else {"Version":"2012-10-17","Statement":[]}
p.setdefault("Statement",[]).append({"Sid":"FactoryosUninstallDenyWrites","Effect":"Deny","Principal":"*",
  "Action":"s3:PutObject","Resource":"arn:aws:s3:::%s/*" % sys.argv[1]})
print(json.dumps(p))' "$1" "$cur") || { warn "cannot build the policy of s3://$1 (writes not denied)"; return 0; }
  if err=$(awsr s3api put-bucket-policy --bucket "$1" --policy "$policy" 2>&1 >/dev/null); then ok "s3://$1: writes denied"
  else warn "could not deny writes to s3://$1 (emptying it anyway): $err"
  fi
}

# empty_bucket <bucket>: every version and delete marker (the source bucket is versioned), 1000 at a time, until a
# listing is empty. Returns 1 when it cannot list the bucket or a delete fails (a bucket that is gone is empty).
empty_bucket() {
  step "Emptying s3://$1"
  local tmp page n out
  tmp=$(mktemp -d) || return 1
  while :; do
    if ! page=$(awsr s3api list-object-versions --bucket "$1" --max-items 1000 --output json 2>"$tmp/err"); then
      if grep -q NoSuchBucket "$tmp/err"; then rm -rf "$tmp"; ok "s3://$1 is gone"; return 0; fi
      warn "cannot list s3://$1: $(cat "$tmp/err")"; rm -rf "$tmp"; return 1
    fi
    # The batch goes in a file: 1000 keys and version ids on the command line can pass Linux's 128 KiB per argument.
    if ! n=$(python3 -I -c 'import json,sys
t=sys.stdin.read().strip()
j=json.loads(t) if t else {}
objs=[{"Key":o["Key"],"VersionId":o["VersionId"]} for o in (j.get("Versions") or [])+(j.get("DeleteMarkers") or [])]
objs=objs[:1000]  # delete-objects takes at most 1000 keys; --max-items caps Versions only, not DeleteMarkers (the loop lists again)
if objs:
    open(sys.argv[1],"w").write(json.dumps({"Objects":objs,"Quiet":True}))
print(len(objs))' "$tmp/batch.json" <<<"$page"); then
      warn "cannot read the listing of s3://$1"; rm -rf "$tmp"; return 1
    fi
    [ "$n" != 0 ] || break
    if ! out=$(awsr s3api delete-objects --bucket "$1" --delete "file://$tmp/batch.json" --output json 2>&1) \
       || ! python3 -I -c 'import json,sys
t=sys.stdin.read().strip()
e=(json.loads(t).get("Errors") or []) if t else []
e and print("%d not deleted, e.g. %s: %s" % (len(e), e[0].get("Key"), e[0].get("Code")), file=sys.stderr)
sys.exit(1 if e else 0)' <<<"$out"; then
      warn "could not delete from s3://$1${out:+: $out}"; rm -rf "$tmp"; return 1
    fi
    rm -f "$tmp/batch.json"
  done
  rm -rf "$tmp"
  ok "s3://$1 is empty"
}
