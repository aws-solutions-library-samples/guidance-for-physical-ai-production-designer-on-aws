# shellcheck shell=bash
# deployment/installer/release.sh: `install` (a new installation or an update), and the release step both share:
# the export into the stack's source bucket, the wanted release into SSM, the instance told to build it.

param_wanted() { echo "/factoryos/$STACK/release/wanted"; }
param_state()  { echo "/factoryos/$STACK/release/state"; }

# get_param <name>: its value, or "" if it does not exist.
get_param() { awsr ssm get-parameter --name "$1" --query Parameter.Value --output text 2>/dev/null || true; }

# serving_version <state json>: the release that serves now ("" if none).
serving_version() {
  [ -n "$1" ] || { echo ""; return; }
  json '(j.get("version") if j.get("phase")=="ready" else j.get("previous")) or ""' <<<"$1"
}

cmd_install() {
  detect_mode
  inspect_source
  # Checked before anything changes: a wrong archive is refused here, not after the stacks.
  stage_jam_tarball
  local state="" serving="" phase=""
  if [ "$MODE" = update ]; then
    state=$(get_param "$(param_state)")
    serving=$(serving_version "$state")
    [ -n "$state" ] && phase=$(json 'j.get("phase") or ""' <<<"$state")
    case "$phase" in
      downloading|deps|building|packaging|deploying|checking)
        die "a release is in progress on the instance ($(json 'j.get("version")' <<<"$state"): $phase); see install.sh status --wait" ;;
    esac
    if [ "$serving" = "$VERSION" ] && [ "$FORCE" = 0 ]; then
      ok "$VERSION is installed already (--force rebuilds it)"
      [[ " $SET_OPTS " != *" umarellJam "* ]] \
        || warn "--umarell-jam-tarball is not used: $VERSION is installed already (with --force its release provisions umarell_jam with it)"
      if [ "$ROTATE_ORIGIN" = 1 ]; then
        say "$ORIGIN_ROTATION_NOTE"; confirm "Rotate the origin secret?"; rotate_origin_secret
      else
        origin_rotation_check warn
      fi
      [ -z "$USERS" ] || { users_after_gate; cmd_add_users ${USERS//,/ }; }
      print_summary
      return 0
    fi
    if [ -n "$serving" ] && [ "$(version_cmp "$VERSION" "$serving")" = -1 ] && [ "$ALLOW_DOWNGRADE" = 0 ]; then
      die "$serving is installed; $VERSION is older (--allow-downgrade installs it anyway)"
    fi
    if [ -n "$serving" ] && [ "$(version_cmp "$VERSION" "$serving")" = "?" ]; then
      warn "cannot order $VERSION and $serving (not both vX.Y.Z); installing as asked"
    fi
    step "Update ${serving:-(no release yet)} → $VERSION in $ACCOUNT / $REGION"
    say "The running release keeps serving while the new one builds (hours); data on the instance is kept."
  else
    step "New installation of $VERSION in $ACCOUNT / $REGION"
    say "Creates: a VPC with one NAT gateway, one ${INSTANCE_TYPE} instance (${VOLUME_GB} GB), a CloudFront distribution"
    say "with a Lambda@Edge sign-in, a Cognito user pool, an S3 bucket for the exports, and a log group. They cost money"
    say "while they exist (mainly the instance and the NAT gateway); install.sh uninstall removes them."
  fi
  if [ "$ROTATE_ORIGIN" = 1 ] && [ "$MODE" = update ]; then
    say "Then the origin secret is rotated (about 8 minutes; the edge never sends a value the backend refuses)."
    say "$ORIGIN_ROTATION_NOTE"
  fi
  [ "$ROTATE_ORIGIN" = 0 ] || [ "$MODE" = update ] || say "(--rotate-origin-secret: a new installation gets a new origin secret anyway.)"
  confirm "Go on?"

  [ "$MODE" = update ] && load_options
  prepare_app
  choose_ami
  bootstrap
  # Before the stacks: the edge function of this release refuses (503) without it, the release reads it.
  ensure_origin_secret
  if [ "$MODE" = install ]; then
    deploy_stacks
  else
    review_update
    if [ "$INFRA_CHANGED" = 1 ]; then deploy_stacks; fi
  fi
  save_options
  # Before the release: the instance must not be building while the rotation refreshes it.
  if [ "$ROTATE_ORIGIN" = 1 ] && [ "$MODE" = update ]; then rotate_origin_secret; fi
  release_step
  if [ -n "$USERS" ]; then users_after_gate; cmd_add_users ${USERS//,/ }; fi
  print_summary
}

# bucket_region <bucket>
bucket_region() {
  local r; r=$(aws s3api get-bucket-location --bucket "$1" --query LocationConstraint --output text 2>/dev/null || true)
  case "$r" in None|null|"") echo us-east-1 ;; EU) echo eu-west-1 ;; *) echo "$r" ;; esac
}

release_step() {
  step "Release $VERSION"
  local bucket doc iid
  bucket=$(stack_output SourceBucket); doc=$(stack_output ReleaseDocument); iid=$(stack_output InstanceId)
  [ -n "$bucket" ] && [ -n "$doc" ] && [ -n "$iid" ] || die "the stack lacks SourceBucket/ReleaseDocument/InstanceId outputs"

  local key="releases/$VERSION.$SOURCE_EXT"
  if [[ "$SOURCE_URI" == s3://* ]]; then
    # S3 to S3, server side: the bytes do not pass through this machine again.
    local src_bucket="${SOURCE_URI#s3://}"; src_bucket="${src_bucket%%/*}"
    awsr s3 cp "$SOURCE_URI" "s3://$bucket/$key" --source-region "$(bucket_region "$src_bucket")" --only-show-errors \
      || die "could not copy $SOURCE_URI into s3://$bucket/$key"
  else
    # --from-checkout: the archive packaged here.
    [ -f "$SOURCE_FILE" ] || die "the packaged checkout $SOURCE_FILE is gone"
    [ "$(sha256_file "$SOURCE_FILE")" = "$SOURCE_SHA" ] || die "$SOURCE_FILE changed since it was packaged"
    awsr s3 cp "$SOURCE_FILE" "s3://$bucket/$key" --only-show-errors \
      || die "could not upload $SOURCE_FILE to s3://$bucket/$key"
  fi
  ok "s3://$bucket/$key"

  local umarell_json="null"
  if [ -n "$UMARELL_TARBALL" ]; then
    local u="$WORK/umarell.tar.gz" usha
    aws s3 cp "$UMARELL_TARBALL" "$u" --only-show-errors || die "cannot read $UMARELL_TARBALL"
    check_umarell_archive "$u" "$UMARELL_TARBALL"
    usha=$(sha256_file "$u")
    awsr s3 cp "$u" "s3://$bucket/vendor/umarell-${usha:0:12}.tar.gz" --only-show-errors
    umarell_json="{\"key\":\"vendor/umarell-${usha:0:12}.tar.gz\",\"sha256\":\"$usha\"}"
    ok "umarell tarball $usha"
  fi

  # umarell_jam's plain umarell commit: the archive staged by stage_jam_tarball, else the one an earlier install put
  # into vendor/ (recorded in the options) while it is still there. Optional: without one the release goes on.
  local jam_json="null"
  if [ -n "$UMARELL_JAM_FILE" ]; then
    [ "$(sha256_file "$UMARELL_JAM_FILE")" = "$UMARELL_JAM_SHA" ] || die "$UMARELL_JAM_FILE changed since it was checked"
    awsr s3 cp "$UMARELL_JAM_FILE" "s3://$bucket/$UMARELL_JAM_KEY" --only-show-errors \
      || die "could not upload the umarell_jam tarball to s3://$bucket/$UMARELL_JAM_KEY"
    ok "umarell_jam tarball $UMARELL_JAM_SHA"
  elif [ -n "$UMARELL_JAM_KEY" ]; then
    if awsr s3api head-object --bucket "$bucket" --key "$UMARELL_JAM_KEY" >/dev/null 2>&1; then
      ok "umarell_jam tarball $UMARELL_JAM_SHA (installed before: s3://$bucket/$UMARELL_JAM_KEY)"
    else
      warn "the installed umarell_jam tarball s3://$bucket/$UMARELL_JAM_KEY is gone: this release installs without it (umarell_jam incomplete; --umarell-jam-tarball gives it again)"
      UMARELL_JAM_KEY="" UMARELL_JAM_SHA=""
      save_options
    fi
  fi
  [ -z "$UMARELL_JAM_KEY" ] || jam_json="{\"key\":\"$UMARELL_JAM_KEY\",\"sha256\":\"$UMARELL_JAM_SHA\"}"

  local wanted
  wanted=$(python3 -I -c 'import json,sys
v,k,s,u,j,t,f=sys.argv[1:]
print(json.dumps({"version":v,"key":k,"sha256":s,"umarell":json.loads(u),"umarellJam":json.loads(j),"requestedAt":t,"force":f=="1"},separators=(",",":")))' \
    "$VERSION" "$key" "$SOURCE_SHA" "$umarell_json" "$jam_json" "$(now_iso)" "$FORCE")
  awsr ssm put-parameter --name "$(param_wanted)" --type String --overwrite --value "$wanted" >/dev/null
  ok "$(param_wanted)"

  wait_instance_ready "$iid"
  run_release_command "$iid" "$doc" "factoryos release $VERSION" || die "the instance did not start the release ($RELEASE_COMMAND_STATUS)"
  ok "the instance builds $VERSION now (install.sh status --wait follows it)"
}

# check_umarell_archive <file> <what>: a gzip tar whose first entry is under umarell/ (git archive --prefix=umarell/).
# The whole listing is read (awk, not head): GNU tar dies of SIGPIPE when the reader stops early, and pipefail would
# then refuse a good archive.
check_umarell_archive() {
  local first
  first=$(tar -tzf "$1" 2>/dev/null | awk 'NR == 1') || die "$2 is not a gzip tar archive"
  [[ "$first" == umarell/* ]] || die "$2 is not a git archive --prefix=umarell/"
}

# stage_jam_tarball: --umarell-jam-tarball (s3:// or a local file) copied into the work directory and checked as
# --umarell-tarball is; sets UMARELL_JAM_FILE, UMARELL_JAM_SHA and UMARELL_JAM_KEY (vendor/umarell-jam-<sha12>.tar.gz,
# which the release step uploads and the options record). Given as '': the recorded one is dropped.
stage_jam_tarball() {
  [[ " $SET_OPTS " == *" umarellJam "* ]] || return 0
  UMARELL_JAM_FILE="" UMARELL_JAM_SHA="" UMARELL_JAM_KEY=""
  [ -n "$UMARELL_JAM_TARBALL" ] || { say "umarell_jam tarball: none (umarell_jam's plain umarell comes from GitHub)"; return 0; }
  local j="$WORK/umarell-jam.tar.gz"
  mkdir -p "$WORK"
  if [[ "$UMARELL_JAM_TARBALL" == s3://* ]]; then
    aws s3 cp "$UMARELL_JAM_TARBALL" "$j" --only-show-errors || die "cannot read $UMARELL_JAM_TARBALL"
  else
    cp "$UMARELL_JAM_TARBALL" "$j" || die "cannot read $UMARELL_JAM_TARBALL"
  fi
  check_umarell_archive "$j" "$UMARELL_JAM_TARBALL"
  UMARELL_JAM_FILE="$j"
  UMARELL_JAM_SHA=$(sha256_file "$j")
  UMARELL_JAM_KEY="vendor/umarell-jam-${UMARELL_JAM_SHA:0:12}.tar.gz"
  ok "umarell_jam tarball $UMARELL_JAM_TARBALL: sha256 $UMARELL_JAM_SHA"
}

# run_release_command <instance id> <document> <comment>: the release document on the instance, waited for (15
# minutes at most); its output's last lines; returns 1 (its output and errors shown, RELEASE_COMMAND_STATUS says why)
# unless it succeeded. It never stops the installer itself: refresh_instance_secrets puts the wanted release back after
# it, whatever happened.
RELEASE_COMMAND_STATUS=""
run_release_command() {
  local iid="$1" doc="$2" cid
  RELEASE_COMMAND_STATUS=""
  if ! cid=$(awsr ssm send-command --document-name "$doc" --document-version '$LATEST' --instance-ids "$iid" --comment "$3" \
        --query Command.CommandId --output text); then
    RELEASE_COMMAND_STATUS="ssm send-command failed"
    warn "ssm send-command failed"
    return 1
  fi
  local st="" deadline=$(( $(date +%s) + 900 ))
  while :; do
    st=$(awsr ssm get-command-invocation --command-id "$cid" --instance-id "$iid" --query Status --output text 2>/dev/null || echo Pending)
    case "$st" in Pending|InProgress|Delayed) ;; *) break ;; esac
    if [ "$(date +%s)" -ge "$deadline" ]; then
      RELEASE_COMMAND_STATUS="not finished in 15 minutes, command $cid"
      warn "the release command did not finish in 15 minutes (command $cid)"
      return 1
    fi
    sleep 5
  done
  RELEASE_COMMAND_STATUS="$st"
  local out err
  out=$(awsr ssm get-command-invocation --command-id "$cid" --instance-id "$iid" --query StandardOutputContent --output text 2>/dev/null || true)
  if [ "$st" != Success ]; then
    err=$(awsr ssm get-command-invocation --command-id "$cid" --instance-id "$iid" --query StandardErrorContent --output text 2>/dev/null || true)
    printf '%s\n%s\n' "$out" "$err" | tail -25 >&2
    return 1
  fi
  printf '%s\n' "$out" | tail -5
}

# first_boot_error <state json>: the error of the instance's failed first-boot setup (phase failed, no version; written
# by user-data.sh), else "".
first_boot_error() {
  [ -n "$1" ] || { echo ""; return; }
  json '(j.get("error") or "first-boot setup failed") if j.get("phase")=="failed" and not j.get("version") else ""' <<<"$1" 2>/dev/null || echo ""
}

# console_tail <instance id>: the last user-data lines of the instance's console (best effort; a failure before the
# AWS CLI is installed can only be seen there).
console_tail() {
  awsr ec2 get-console-output --instance-id "$1" --latest --query Output --output text 2>/dev/null \
    | grep -E '\[bootstrap |^E: |Failed to run module scripts_user' | tail -8 || true
}

# wait_instance_ready <instance id>: SSM online and its first-boot setup done (it writes the state parameter last).
wait_instance_ready() {
  local deadline=$(( $(date +%s) + ${FACTORYOS_READY_TIMEOUT:-3600} )) said=0
  while :; do
    local ping state err
    ping=$(awsr ssm describe-instance-information --filters "Key=InstanceIds,Values=$1" \
      --query 'InstanceInformationList[0].PingStatus' --output text 2>/dev/null || true)
    state=$(get_param "$(param_state)")
    err=$(first_boot_error "$state")
    [ -z "$err" ] || die "instance $1: $err. Its log: CloudWatch $(stack_output LogGroup), stream $1/bootstrap.log (or /var/log/factoryos/bootstrap.log over SSM); uninstall and install again, or run the user data again on the instance (sudo bash /var/lib/cloud/instance/user-data.txt) and then install.sh install again"
    if [ "$ping" = Online ] && [ -n "$state" ]; then ok "instance $1 is ready"; return 0; fi
    if [ "$(date +%s)" -ge "$deadline" ]; then
      local tail; tail=$(console_tail "$1")
      [ -z "$tail" ] || { warn "the instance's console (its last user-data lines):"; printf '%s\n' "$tail" | sed 's/^/    /' >&2; }
      die "instance $1 is not ready after $(( ${FACTORYOS_READY_TIMEOUT:-3600} / 60 )) minutes (SSM: ${ping:-unknown}; first-boot setup: $([ -n "$state" ] && echo "done" || echo "not finished, no state parameter")). Its log: CloudWatch $(stack_output LogGroup), stream $1/bootstrap.log (or /var/log/factoryos/bootstrap.log over SSM)"
    fi
    [ "$said" = 1 ] || { say "Waiting for the instance's first-boot setup (a few minutes)…"; said=1; }
    sleep 10
  done
}

print_summary() {
  local url; url=$(stack_output Url)
  step "Done"
  say "${B}URL${N}      $url"
  say "${B}Progress${N} $0 status --wait --stack $STACK --region $REGION"
  say "A first build takes about 2 hours; until it is ready the URL signs you in and shows an error page.
Users sign in with their e-mail address and the temporary password Cognito e-mailed them, then set up an
authenticator app (TOTP MFA). Cognito's default e-mail sender sends at most about 50 invitations a day."
}
