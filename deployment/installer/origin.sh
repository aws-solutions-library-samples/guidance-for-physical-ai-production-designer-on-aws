# shellcheck shell=bash
# deployment/installer/origin.sh: the origin secret (deployment/CONTRACT.md "Origin secret"). 48 random bytes,
# base64url (64 characters), made here at install time, never in the CloudFormation template or the user data, kept in
# two SecureStrings (the AWS managed key aws/ssm):
#   /factoryos/<stack>/edge-origin-secret in us-east-1   the edge function reads it and sends it as x-origin-verify
#   /factoryos/<stack>/origin-secret in the stack's Region the instance role reads it; the release handoff writes it
#                                                          into /etc/factoryos/origin.env (FACTORY_OS_ORIGIN_SECRET)
# The value never passes through argv or a log: python3 makes it and writes the put-parameter requests into 0600 files
# in a 0700 directory (aws ssm put-parameter --cli-input-json file://…), deleted at once; a value read back goes from
# the CLI's stdout into python3's stdin.
#
# Before the stacks are deployed (install and update): neither exists (a new installation, or one from before the origin
# secret): a new secret into both; both exist: kept (their Type and the instance copy's Description are read, never a
# value); the instance's alone: both anew (its name is this Region's, so this installation's); the edge copy alone:
# refused, as the stack's OriginSecret custom resource refuses it (the same stack name in another Region of the account,
# or a leftover). That custom resource (lib/factoryos-stack.ts, lambda/custom-resources/origin-secret.mjs) then keeps
# what it finds; it generates the pair itself only for a deployment that did not go through this installer.
# --rotate-origin-secret replaces the secret so that the edge never sends a value the backend refuses:
#   1. instance's parameter := "<new>,<old>" (old: the edge's current value), the instance refreshes origin.env and
#      restarts the backend, which then accepts both;
#   2. edge parameter := <new>; wait until every edge replica has reloaded its configuration (it caches it 5 minutes:
#      FACTORYOS_ORIGIN_EDGE_WAIT seconds, default 420);
#   3. instance's parameter := <new>, refreshed again (a second restart): the old value is refused from then on.
# The refreshes are the stack's release document with a `secretsOnly: true` request (the handoff writes origin.env and
# stops; no build), the wanted release put back afterwards. Each refresh restarts the backend (FACTORY_OS_ORIGIN_SECRET
# is read at its start) and the planner router with it: open terminals close, viewers reconnect, a running planning
# job stops (ORIGIN_ROTATION_NOTE).
# A rotation that stops after step 1 (an error, Ctrl-C, a lost session) leaves the old value accepted: the EXIT trap
# says so and puts back a wanted release left as a secretsOnly request; later runs find the pair by its Description
# ("during a rotation"), without decrypting it: an update refuses until the rotation is run again, others warn.

ORIGIN_EDGE_WAIT="${FACTORYOS_ORIGIN_EDGE_WAIT:-420}"
ORIGIN_ROTATION_NOTE="The backend restarts twice during the rotation (a few seconds each; the planner router
restarts with it): open terminals close, viewers reconnect, and a running planning job is stopped. Run it when
nobody is working."
ORIGIN_ROTATION_RERUN="the old origin secret is still accepted; run install.sh install --rotate-origin-secret again"
ORIGIN_ROTATION_RERUN+=" (with the same source and options)"

# The state of the one EXIT trap the functions below share (bash keeps one per signal; the release build's
# start_release_build sets the same one, and origin_on_exit ends with its part, release_build_on_exit): a request
# directory to delete, a wanted release to put back over a secretsOnly request, and whether a rotation has passed
# step 1 without finishing step 3.
ORIGIN_TMP="" ORIGIN_WANTED_RESTORE="" ORIGIN_ROTATION_OPEN=0
origin_on_exit() {
  [ -z "$ORIGIN_TMP" ] || rm -rf "$ORIGIN_TMP"
  ORIGIN_TMP=""
  if [ -n "$ORIGIN_WANTED_RESTORE" ]; then
    local w="$ORIGIN_WANTED_RESTORE"; ORIGIN_WANTED_RESTORE=""
    if awsr ssm put-parameter --name "$(param_wanted)" --type String --overwrite --value "$w" >/dev/null 2>&1; then
      warn "$(param_wanted) put back as it was (the secretsOnly refresh did not finish)"
    else
      warn "could not put $(param_wanted) back: it still asks for a secretsOnly refresh, so the release document builds
    nothing until install.sh install runs again"
    fi
  fi
  if [ "$ORIGIN_ROTATION_OPEN" = 1 ]; then
    ORIGIN_ROTATION_OPEN=0; warn "the rotation did not finish: $ORIGIN_ROTATION_RERUN"
  fi
  release_build_on_exit
}
origin_trap() { trap origin_on_exit EXIT; }

param_origin()      { echo "/factoryos/$STACK/origin-secret"; }
param_edge_origin() { echo "/factoryos/$STACK/edge-origin-secret"; }

# origin_param_type <region> <name>: SecureString, another type, or NONE; stops on any other error (an unreadable
# parameter is not taken for a missing one, which would be replaced).
origin_param_type() {
  local out
  if out=$(aws --region "$1" ssm get-parameter --name "$2" --query Parameter.Type --output text 2>&1); then
    echo "$out"
  elif grep -q ParameterNotFound <<<"$out"; then
    echo NONE
  else
    die "cannot read the parameter $2 in $1: $out"
  fi
}

# origin_requests <dir> [<old value on stdin>]: makes a new secret and writes the requests into <dir>: edge.json (the
# edge's parameter := new), next.json (the instance's := new) and, with an old value, both.json (the instance's :=
# "new,old"). Prints nothing.
origin_requests() {
  python3 -I -c '
import base64, json, os, re, secrets, sys
d, region_name, edge_name, stack, with_old = sys.argv[1:6]
old = ""
if with_old == "1":
    old = sys.stdin.read().strip()
    if old and not re.fullmatch(r"[A-Za-z0-9_-]{32,256}", old):
        sys.exit("the current origin secret is not one base64url value; rerun without --rotate-origin-secret first")
new = base64.urlsafe_b64encode(secrets.token_bytes(48)).decode().rstrip("=")
def put(name, value, where, file):
    req = {"Name": name, "Type": "SecureString", "Overwrite": True, "Tier": "Standard", "Value": value,
           "Description": "FactoryOS %s: the origin secret (x-origin-verify), %s; written by install.sh" % (stack, where)}
    fd = os.open(os.path.join(d, file), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as f:
        json.dump(req, f)
put(edge_name, new, "read by the CloudFront gate", "edge.json")
put(region_name, new, "read by the instance", "next.json")
if old:
    put(region_name, new + "," + old, "read by the instance (during a rotation: new,old)", "both.json")
' "$1" "$(param_origin)" "$(param_edge_origin)" "$STACK" "${2:-0}"
}

# origin_put <region> <request file>
origin_put() {
  local err
  err=$(aws --region "$1" ssm put-parameter --cli-input-json "file://$2" 2>&1 >/dev/null) \
    || { rm -rf "$(dirname "$2")"; die "could not write the origin secret in $1: $err"; }
}

# origin_dir: a 0700 directory for the requests (removed by the caller, and on exit: ORIGIN_TMP).
origin_dir() {
  local d; d=$(umask 077 && mktemp -d "${TMPDIR:-/tmp}/factoryos-origin.XXXXXX") || die "mktemp failed"
  chmod 0700 "$d"
  echo "$d"
}

# origin_rotation_left: 0 when the instance's parameter still holds the "<new>,<old>" pair of a rotation that did not
# finish (its Description says "during a rotation"; the value is not read), 1 when not, 2 when that cannot be read
# (ssm:DescribeParameters, Resource "*"; warned).
origin_rotation_left() {
  local out
  if ! out=$(awsr ssm describe-parameters --parameter-filters "Key=Name,Values=$(param_origin)" \
               --query 'Parameters[0].Description' --output text 2>&1); then
    warn "cannot check $(param_origin) for an unfinished rotation (ssm:DescribeParameters): $out"
    return 2
  fi
  case "$out" in *"during a rotation"*) return 0 ;; *) return 1 ;; esac
}

# origin_rotation_check: after a rotation that did not finish, an update refuses unless it rotates (and so finishes
# it); a new installation and a run with the version installed already only warn.
origin_rotation_check() {
  local rc=0; origin_rotation_left || rc=$?
  [ "$rc" = 0 ] || return 0
  if [ "$ROTATE_ORIGIN" = 1 ]; then
    say "$(param_origin) holds the two values of a rotation that did not finish: this rotation replaces both."
  elif [ "$MODE" = update ] && [ "${1:-}" != warn ]; then
    die "$(param_origin) holds the two values of a rotation that did not finish: $ORIGIN_ROTATION_RERUN"
  else
    warn "$(param_origin) holds the two values of a rotation that did not finish: $ORIGIN_ROTATION_RERUN"
  fi
}

# ensure_origin_secret: both parameters as SecureStrings before the stacks are deployed (the cases in the header).
ensure_origin_secret() {
  local tr te
  tr=$(origin_param_type "$REGION" "$(param_origin)")
  te=$(origin_param_type "$EDGE_REGION" "$(param_edge_origin)")
  if [ "$tr" = SecureString ] && [ "$te" = SecureString ]; then
    origin_rotation_check
    ok "origin secret: kept ($(param_origin) in $REGION, $(param_edge_origin) in $EDGE_REGION)"
    return 0
  fi
  if [ "$te" != NONE ] && [ "$tr" = NONE ]; then
    die "$(param_edge_origin) exists in $EDGE_REGION without $(param_origin) in $REGION: another installation of the stack
    name $STACK in this account (in another Region) uses it, or an uninstall left it behind. Install with another
    --stack-name, or, if no installation named $STACK exists in any Region, delete the leftover:
    aws ssm delete-parameter --region $EDGE_REGION --name $(param_edge_origin)"
  fi
  step "The origin secret"
  local d; d=$(origin_dir)
  ORIGIN_TMP="$d"; origin_trap
  origin_requests "$d" 0 </dev/null || { rm -rf "$d"; die "could not make the origin secret"; }
  # The edge's first: it is only sent once the edge function of this release is deployed; the instance's is only read
  # by the release.
  origin_put "$EDGE_REGION" "$d/edge.json"
  origin_put "$REGION" "$d/next.json"
  rm -rf "$d"; ORIGIN_TMP=""
  ok "origin secret: a new one in $(param_edge_origin) ($EDGE_REGION) and $(param_origin) ($REGION)"
}

# refresh_instance_secrets <what>: the release document with a secretsOnly request (origin.env written, the backend
# restarted when it changed; nothing built), then the wanted release as it was. The instance is waited for before the
# request is written, and from the write to the put-back the EXIT trap puts it back too: a secretsOnly request never
# outlives this function (left in place, every later run of the release document would build nothing).
refresh_instance_secrets() {
  local iid doc wanted
  iid=$(stack_output InstanceId); doc=$(stack_output ReleaseDocument)
  [ -n "$iid" ] && [ -n "$doc" ] || die "the stack lacks InstanceId/ReleaseDocument outputs"
  wanted=$(get_param "$(param_wanted)")
  [ -n "$wanted" ] || die "no release was requested on this installation yet: install it first"
  # A secretsOnly request found there (left by an installer from before the EXIT trap) is not put back as such.
  wanted=$(python3 -I -c 'import json, sys
w = json.loads(sys.argv[1])
if not isinstance(w, dict): sys.exit(1)
print(json.dumps({k: v for k, v in w.items() if k != "secretsOnly"}, separators=(",", ":")) if "secretsOnly" in w else sys.argv[1])' \
    "$wanted") || die "$(param_wanted) is not a JSON object"
  local refresh
  refresh=$(python3 -I -c 'import json, sys
w = json.loads(sys.argv[1]); w["secretsOnly"] = True; w["requestedAt"] = sys.argv[2]
print(json.dumps(w, separators=(",", ":")))' "$wanted" "$(now_iso)") || die "$(param_wanted) is not JSON"
  wait_instance_ready "$iid"
  ORIGIN_WANTED_RESTORE="$wanted"; origin_trap
  awsr ssm put-parameter --name "$(param_wanted)" --type String --overwrite --value "$refresh" >/dev/null
  local rc=0
  run_release_command "$iid" "$doc" "factoryos origin secret: $1" || rc=$?
  awsr ssm put-parameter --name "$(param_wanted)" --type String --overwrite --value "$wanted" >/dev/null
  ORIGIN_WANTED_RESTORE=""
  if [ "$rc" != 0 ]; then
    local why="the instance did not refresh the origin secret ($1: $RELEASE_COMMAND_STATUS)"
    if [ "$ORIGIN_ROTATION_OPEN" = 1 ]; then ORIGIN_ROTATION_OPEN=0; die "$why; $ORIGIN_ROTATION_RERUN"; fi
    die "$why; the backend keeps the value it had"
  fi
  ok "the instance has the origin secret ($1)"
}

# rotate_origin_secret: --rotate-origin-secret (the three steps in the header).
rotate_origin_secret() {
  step "Rotating the origin secret"
  local te d old=0
  te=$(origin_param_type "$EDGE_REGION" "$(param_edge_origin)")
  d=$(origin_dir)
  ORIGIN_TMP="$d"; origin_trap
  if [ "$te" = SecureString ]; then
    old=1
    awse ssm get-parameter --name "$(param_edge_origin)" --with-decryption --query Parameter.Value --output text \
      2>"$d/err" | origin_requests "$d" 1 || { warn "$(cat "$d/err" 2>/dev/null)"; rm -rf "$d"; die "could not read the current origin secret"; }
  else
    origin_requests "$d" 0 </dev/null || { rm -rf "$d"; die "could not make the origin secret"; }
  fi
  if [ "$old" = 1 ]; then
    origin_put "$REGION" "$d/both.json"
    # From here until step 3 has been refreshed the old value is accepted: stopping says so (origin_on_exit).
    ORIGIN_ROTATION_OPEN=1
    refresh_instance_secrets "the new and the old value accepted"
  fi
  origin_put "$EDGE_REGION" "$d/edge.json"
  if [ "$old" = 1 ]; then
    say "The edge sends the new value once its replicas reload their configuration: waiting $ORIGIN_EDGE_WAIT s."
    sleep "$ORIGIN_EDGE_WAIT"
  fi
  origin_put "$REGION" "$d/next.json"
  rm -rf "$d"; ORIGIN_TMP=""
  refresh_instance_secrets "the new value only"
  ORIGIN_ROTATION_OPEN=0
  ok "origin secret rotated"
}
