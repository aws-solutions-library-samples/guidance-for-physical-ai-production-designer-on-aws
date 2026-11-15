# shellcheck shell=bash
# deployment/installer/infra.sh: the export (download or package the checkout, sha256, version), the CDK app (npm ci in a work directory,
# bootstrap, deploy) and the review of an update's infrastructure change.

VERSION=""
SOURCE_SHA=""
SOURCE_EXT=""
# The archive on this machine (downloaded from SOURCE_URI, or packaged from the checkout).
SOURCE_FILE=""
WORK=""
APP=""
VERSION_RE='^[A-Za-z0-9._+-]{1,64}$'

# release_of_json <file>: the "release" of a VERSION.json ("" if none, or not JSON).
release_of_json() { json 'j.get("release") or ""' < "$1" 2>/dev/null || true; }

# tree_release <root>: the release a tree's VERSION.json names: <root>/source/VERSION.json (the Guidance layout),
# else <root>/VERSION.json (the flat one); "" if neither names one.
tree_release() {
  local f
  for f in "$1/source/VERSION.json" "$1/VERSION.json"; do
    [ -f "$f" ] || continue
    release_of_json "$f"; return 0
  done
  echo ""
}

work_space_check() {
  mkdir -p "$WORK_ROOT"
  local free; free=$(df -Pk "$WORK_ROOT" | awk 'NR==2{print int($4/1024)}')
  [ "${free:-0}" -ge 1500 ] || die "$WORK_ROOT has ${free} MB free; 1.5 GB are needed (set FACTORYOS_WORK to another directory)"
}

# inspect_source: the export (from SOURCE_URI, or the checkout with --from-checkout) into the work directory,
# checked; sets VERSION SOURCE_SHA SOURCE_EXT SOURCE_FILE WORK.
inspect_source() {
  if [ "$FROM_CHECKOUT" = 1 ]; then package_checkout; return; fi
  step "The export"
  local bucket="${SOURCE_URI#s3://}"; bucket="${bucket%%/*}"
  local key="${SOURCE_URI#s3://*/}"
  case "$SOURCE_URI" in *.zip) SOURCE_EXT=zip ;; *) SOURCE_EXT=tar.gz ;; esac
  work_space_check
  local dl="$WORK_ROOT/source.$SOURCE_EXT"
  aws s3 cp "$SOURCE_URI" "$dl" --only-show-errors 2>&1 || die "cannot read $SOURCE_URI (does this account's role have s3:GetObject on it?)"
  SOURCE_SHA=$(sha256_file "$dl")
  local want
  if want=$(aws s3 cp "$SOURCE_URI.sha256" - 2>/dev/null | awk '{print $1; exit}') && [ -n "$want" ]; then
    [ "$want" = "$SOURCE_SHA" ] || die "$SOURCE_URI has sha256 $SOURCE_SHA, but $SOURCE_URI.sha256 says $want"
    ok "sha256 $SOURCE_SHA (matches $key.sha256)"
  else
    ok "sha256 $SOURCE_SHA (no $key.sha256 next to it to compare with)"
  fi
  # The archive's entries: one top directory (the instance refuses anything else), and its VERSION.json at
  # <top>/source/VERSION.json (the Guidance layout) or <top>/VERSION.json (the flat one), source/ first.
  local entries vname vjson="" tops
  if [ "$SOURCE_EXT" = zip ]; then
    entries=$(unzip -Z1 "$dl" 2>/dev/null) || die "$SOURCE_URI is not a zip archive"
  else
    entries=$(tar -tzf "$dl" 2>/dev/null) || die "$SOURCE_URI is not a tar.gz archive"
  fi
  entries=$(sed -e 's|^\./||' <<<"$entries" | grep -v '^$' || true)
  tops=$(cut -d/ -f1 <<<"$entries" | sort -u | grep -c . || true)
  [ "$tops" = 1 ] || die "$SOURCE_URI has $tops top-level entries; the export must have exactly one top directory (factoryos-<release>/)"
  vname=$(grep -E '^[^/]+/(source/)?VERSION\.json$' <<<"$entries" | awk '{print (index($0,"/source/")?0:1) "\t" $0}' | sort | head -1 | cut -f2 || true)
  if [ -n "$vname" ]; then
    if [ "$SOURCE_EXT" = zip ]; then vjson=$(unzip -p "$dl" "$vname" 2>/dev/null || true)
    else vjson=$(tar -xOzf "$dl" "$vname" 2>/dev/null || tar -xOzf "$dl" "./$vname" 2>/dev/null || true); fi
  fi
  if [ -n "$vjson" ]; then
    VERSION=$(json 'j.get("release") or ""' <<<"$vjson" 2>/dev/null) || die "$vname in $SOURCE_URI is not JSON"
  fi
  [ -n "$VERSION" ] || VERSION="dev-${SOURCE_SHA:0:12}"
  [[ "$VERSION" =~ $VERSION_RE ]] || die "unusable version '$VERSION' in the export's VERSION.json"
  local here_version; here_version=$(tree_release "$INSTALLER_DIR/..")
  if [ -n "$here_version" ] && [ "$here_version" != "$VERSION" ]; then
    die "this install.sh is from $here_version, but $SOURCE_URI is $VERSION: run the install.sh of the export you install"
  fi
  ok "version $VERSION"
  WORK="$WORK_ROOT/$VERSION"
  mkdir -p "$WORK"
  SOURCE_FILE="$dl"
}

# package_checkout: --from-checkout. Packages the checkout this install.sh runs from (the directory above
# deployment/) into factoryos-<version>.tar.gz with one top directory factoryos-<version>/:
#   - a clean git checkout: git archive of HEAD (what is committed, nothing else);
#   - a git checkout with changes: its tracked and untracked files except the ignored ones, as they are on disk;
#     the version gets the suffix +dirty-<the first 12 hex digits of a sha256 over the packaged files>;
#   - not a git checkout (e.g. an unpacked zip): every file except .git, node_modules, cdk.out, target, .venv and
#     __pycache__ directories.
# The version: the release of source/VERSION.json, else of VERSION.json, else `git describe --tags --always`, else
# dev-<12 hex digits of the content hash>. Sets VERSION SOURCE_SHA SOURCE_EXT=tar.gz SOURCE_FILE WORK.
package_checkout() {
  step "The checkout"
  local root; root=$(cd "$INSTALLER_DIR/.." && pwd)
  [ -f "$root/deployment/install.sh" ] && [ -f "$root/deployment/bootstrap/factoryos-release.sh" ] \
    || die "$root is not a FactoryOS checkout (no deployment/install.sh and deployment/bootstrap/factoryos-release.sh)"
  SOURCE_EXT=tar.gz
  work_space_check
  local staging; staging=$(mktemp -d "$WORK_ROOT/checkout.XXXXXX")
  local base kind="plain" archive="$staging/source.tar.gz"
  base=$(tree_release "$root")
  if [ -e "$root/.git" ] && command -v git >/dev/null && [ "$(git -C "$root" rev-parse --show-prefix 2>/dev/null)" = "" ] \
     && git -C "$root" rev-parse --verify -q HEAD >/dev/null; then
    if [ -z "$(git -C "$root" status --porcelain --untracked-files=normal 2>/dev/null)" ]; then kind=clean; else kind=dirty; fi
    if [ -z "$base" ]; then
      base=$(git -C "$root" describe --tags --always 2>/dev/null || true)
      [[ "$base" =~ $VERSION_RE ]] || base="dev-$(git -C "$root" rev-parse HEAD | cut -c1-12)"
    fi
  fi
  [ -z "$base" ] || [[ "$base" =~ $VERSION_RE ]] || die "unusable version '$base' in the checkout's VERSION.json"
  case "$kind" in
    clean)
      VERSION="$base"
      git -C "$root" archive --format=tar.gz --prefix="factoryos-$VERSION/" -o "$archive" HEAD \
        || die "git archive of $root failed"
      ok "git archive of HEAD $(git -C "$root" rev-parse --short=12 HEAD) (the checkout is clean)" ;;
    dirty)
      VERSION=$(git -C "$root" ls-files -z --cached --others --exclude-standard \
        | pack_tree "$root" "$base" dirty "$archive") || die "could not package $root"
      warn "the checkout has uncommitted changes: packaged as they are on disk (git status shows them)" ;;
    plain)
      VERSION=$(pack_tree "$root" "$base" plain "$archive" </dev/null) || die "could not package $root"
      ok "packaged $root (not a git checkout)" ;;
  esac
  [[ "$VERSION" =~ $VERSION_RE ]] || die "unusable version '$VERSION'"
  SOURCE_SHA=$(sha256_file "$archive")
  WORK="$WORK_ROOT/$VERSION"
  mkdir -p "$WORK"
  SOURCE_FILE="$WORK/factoryos-$VERSION.tar.gz"
  mv -f "$archive" "$SOURCE_FILE" && rm -rf "$staging"
  ok "sha256 $SOURCE_SHA, $(( $(wc -c < "$SOURCE_FILE") / 1048576 )) MB: $SOURCE_FILE"
  ok "version $VERSION"
}

# pack_tree <root> <base version> dirty|plain <out.tar.gz> [< NUL-separated paths for dirty]: writes the archive with
# the top directory factoryos-<version>/ and prints the version (base+dirty-<sha12>, or base, or dev-<sha12>).
pack_tree() {
  python3 -I -c '
import hashlib, os, sys, tarfile
root, base, kind, out = sys.argv[1:5]
skip = {".git", "node_modules", "cdk.out", "target", ".venv", "__pycache__"}
if kind == "dirty":
    names = [n for n in sys.stdin.buffer.read().decode("utf-8", "surrogateescape").split("\0") if n]
else:
    names = []
    for d, dirs, files in os.walk(root):
        dirs[:] = sorted(x for x in dirs if x not in skip and not os.path.islink(os.path.join(d, x)))
        names += [os.path.relpath(os.path.join(d, f), root) for f in files]
        names += [os.path.relpath(os.path.join(d, x), root) for x in os.listdir(d)
                  if x not in skip and os.path.islink(os.path.join(d, x)) and os.path.isdir(os.path.join(d, x))]
keep = []
for n in sorted(set(names)):
    p = os.path.join(root, n)
    if any(part in skip for part in n.split("/")):
        continue
    if os.path.islink(p) or os.path.isfile(p):   # deleted files and submodule directories are not packaged
        keep.append(n)
h = hashlib.sha256()
for n in keep:
    p = os.path.join(root, n)
    if os.path.islink(p):
        h.update(b"l\0" + n.encode("utf-8", "surrogateescape") + b"\0" + os.readlink(p).encode("utf-8", "surrogateescape") + b"\n")
    else:
        f = hashlib.sha256()
        with open(p, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                f.update(chunk)
        x = "x" if os.stat(p).st_mode & 0o111 else "f"
        h.update(x.encode() + b"\0" + n.encode("utf-8", "surrogateescape") + b"\0" + f.hexdigest().encode() + b"\n")
sha12 = h.hexdigest()[:12]
version = (base or "dev-" + sha12) + ("+dirty-" + sha12 if kind == "dirty" else "")
top = "factoryos-" + version
def clean(ti):
    ti.uid = ti.gid = 0
    ti.uname = ti.gname = ""
    return ti
with tarfile.open(out, "w:gz") as tf:
    d = tarfile.TarInfo(top)
    d.type, d.mode = tarfile.DIRTYPE, 0o755
    tf.addfile(d)
    for n in keep:
        tf.add(os.path.join(root, n), arcname=top + "/" + n, recursive=False, filter=clean)
print(version)
' "$@"
}

# prepare_app: the CDK app with its own pinned CDK CLI, in the work directory (CloudShell's home is only 1 GB).
prepare_app() {
  step "The deployment app"
  APP="$WORK/deployment"
  mkdir -p "$WORK"
  rm -rf "$APP.new"
  mkdir -p "$APP.new"
  (cd "$INSTALLER_DIR" && tar --exclude=./node_modules --exclude=./cdk.out -cf - .) | tar -xf - -C "$APP.new"
  if [ -d "$APP/node_modules" ] && cmp -s "$APP/package-lock.json" "$APP.new/package-lock.json"; then
    mv "$APP/node_modules" "$APP.new/node_modules"
  fi
  rm -rf "$APP" && mv "$APP.new" "$APP"
  if [ ! -d "$APP/node_modules" ]; then
    (cd "$APP" && npm ci --no-audit --no-fund --loglevel=error >"$WORK/npm.log" 2>&1) \
      || { tail -20 "$WORK/npm.log" >&2; die "npm ci failed (log: $WORK/npm.log)"; }
  fi
  ok "$(cd "$APP" && npx --no-install cdk --version 2>/dev/null | head -1) in $APP"
}

# cdk <args>: the app's CDK CLI with this installation's context.
cdk() {
  (cd "$APP" && CDK_DEFAULT_ACCOUNT="$ACCOUNT" CDK_DEFAULT_REGION="$REGION" AWS_REGION="$REGION" AWS_DEFAULT_REGION="$REGION" \
    npx --no-install cdk "$@" "${CDK_CONTEXT[@]}")
}
# cdk_context: this installation's context, into CDK_CONTEXT.
cdk_context() {
  CDK_CONTEXT=(-c stackName="$STACK" -c instanceType="$INSTANCE_TYPE" -c volumeGb="$VOLUME_GB" -c bedrock="$BEDROCK"
               -c kiroSecretName="$KIRO_SECRET" -c amiId="$AMI_ID")
  # Without --bedrock-models the app's own default models apply.
  [ -z "$BEDROCK_MODELS" ] || CDK_CONTEXT+=(-c bedrockModelIds="$BEDROCK_MODELS")
}
CDK_CONTEXT=()

# bootstrap: CDK bootstrap in the stack's region and in us-east-1 (the edge function), where it is missing.
bootstrap() {
  step "CDK bootstrap"
  local r
  for r in $(printf '%s\n' "$REGION" "$EDGE_REGION" | sort -u); do
    if aws --region "$r" ssm get-parameter --name /cdk-bootstrap/hnb659fds/version --query Parameter.Value --output text >/dev/null 2>&1; then
      ok "$r is bootstrapped"
    else
      say "Bootstrapping aws://$ACCOUNT/$r (the CDK's staging bucket and deployment roles; once per account and region)"
      cdk bootstrap "aws://$ACCOUNT/$r" >"$WORK/bootstrap-$r.log" 2>&1 \
        || { tail -30 "$WORK/bootstrap-$r.log" >&2; die "cdk bootstrap $r failed (log: $WORK/bootstrap-$r.log)"; }
      ok "$r bootstrapped"
    fi
  done
}

# The installation's options, kept in SSM so an update deploys the same ones (a changed volume size or AMI would
# replace the instance; a changed instance type stops and starts it).
param_options() { echo "/factoryos/$STACK/install-options"; }
load_options() {
  local o; o=$(awsr ssm get-parameter --name "$(param_options)" --query Parameter.Value --output text 2>/dev/null || true)
  if [ -z "$o" ]; then
    # An installation without the parameter: what the instance itself has.
    local iid; iid=$(stack_output InstanceId)
    o=$(awsr ec2 describe-instances --instance-ids "$iid" --output json \
      --query 'Reservations[0].Instances[0].{t:InstanceType,v:BlockDeviceMappings[0].Ebs.VolumeId}' 2>/dev/null \
      | python3 -I -c 'import json,sys; j=json.load(sys.stdin); print(json.dumps({"instanceType":j["t"],"volume":j["v"]}))' 2>/dev/null || true)
    if [ -n "$o" ]; then
      local vol size; vol=$(json 'j["volume"]' <<<"$o")
      size=$(awsr ec2 describe-volumes --volume-ids "$vol" --query 'Volumes[0].Size' --output text 2>/dev/null || true)
      o=$(json 'json.dumps({"instanceType":j["instanceType"]})' <<<"$o" 2>/dev/null || echo '{}')
      o=$(python3 -I -c 'import json,sys; j=json.loads(sys.argv[1]); j["volumeGb"]=sys.argv[2]; print(json.dumps(j))' "$o" "$size")
    fi
    warn "no $(param_options): using the instance's own type and volume size, Bedrock off, no Kiro secret, unless given"
  fi
  [ -n "$o" ] || return 0
  local k v
  for k in instanceType volumeGb bedrock kiroSecretName bedrockModelIds; do
    [[ " $SET_OPTS " == *" $k "* ]] && continue
    v=$(python3 -I -c 'import json,sys; j=json.loads(sys.argv[1]); v=j.get(sys.argv[2]); print("" if v is None else str(v).lower() if isinstance(v,bool) else v)' "$o" "$k")
    [ -n "$v" ] || [ "$k" = kiroSecretName ] || [ "$k" = bedrockModelIds ] || continue
    case "$k" in
      instanceType) INSTANCE_TYPE="$v" ;; volumeGb) VOLUME_GB="$v" ;; bedrock) BEDROCK="$v" ;; kiroSecretName) KIRO_SECRET="$v" ;;
      bedrockModelIds) BEDROCK_MODELS="$v" ;;
    esac
  done
  # umarellJam: the umarell_jam tarball the source bucket holds (vendor/umarell-jam-<sha12>.tar.gz and its sha256),
  # kept unless --umarell-jam-tarball is given (the release step checks it is still there).
  if [[ " $SET_OPTS " != *" umarellJam "* ]]; then
    local jk js
    { read -r jk; read -r js; } < <(python3 -I -c 'import json,re,sys
j=json.loads(sys.argv[1]).get("umarellJam")
ok=isinstance(j,dict) and re.fullmatch(r"vendor/umarell-jam-[0-9a-f]{12}\.tar\.gz",str(j.get("key"))) and re.fullmatch(r"[0-9a-f]{64}",str(j.get("sha256")))
print(j["key"] if ok else ""); print(j["sha256"] if ok else "")' "$o" 2>/dev/null || printf '\n\n')
    UMARELL_JAM_KEY="$jk"; UMARELL_JAM_SHA="$js"
  fi
  say "Options: instance $INSTANCE_TYPE, volume ${VOLUME_GB} GB, Bedrock $BEDROCK${BEDROCK_MODELS:+ ($BEDROCK_MODELS)}, Kiro secret ${KIRO_SECRET:-none}${UMARELL_JAM_KEY:+, umarell_jam tarball ${UMARELL_JAM_SHA:0:12}}"
}
save_options() {
  local o; o=$(python3 -I -c 'import json,sys; k,s=sys.argv[6:8]; print(json.dumps({"instanceType":sys.argv[1],"volumeGb":sys.argv[2],"bedrock":sys.argv[3]=="true","kiroSecretName":sys.argv[4],"bedrockModelIds":sys.argv[5],"umarellJam":{"key":k,"sha256":s} if k else None},separators=(",",":")))' \
    "$INSTANCE_TYPE" "$VOLUME_GB" "$BEDROCK" "$KIRO_SECRET" "$BEDROCK_MODELS" "$UMARELL_JAM_KEY" "$UMARELL_JAM_SHA")
  awsr ssm put-parameter --name "$(param_options)" --type String --overwrite --value "$o" >/dev/null
}

AMI_ID=""
# choose_ami: a new install takes the newest Ubuntu 24.04 arm64 AMI; an update keeps the instance's own (a newer AMI
# would replace the instance).
choose_ami() {
  if [ "$MODE" = update ]; then
    local iid; iid=$(stack_output InstanceId)
    [ -n "$iid" ] || die "the stack has no InstanceId output"
    AMI_ID=$(awsr ec2 describe-instances --instance-ids "$iid" --query 'Reservations[0].Instances[0].ImageId' --output text)
  else
    AMI_ID=$(awsr ssm get-parameter --name /aws/service/canonical/ubuntu/server/24.04/stable/current/arm64/hvm/ebs-gp3/ami-id \
      --query Parameter.Value --output text)
  fi
  [[ "$AMI_ID" =~ ^ami-[0-9a-f]+$ ]] || die "could not determine the AMI (got '$AMI_ID')"
  cdk_context
}

deploy_stacks() {
  step "Deploying the infrastructure (about 15–25 minutes on a first install: CloudFront and the sign-in check)"
  local log="$WORK/deploy.log"
  if ! cdk deploy --all --require-approval never --progress events --outputs-file "$WORK/outputs.json" 2>&1 | tee "$log" \
       | grep --line-buffered -E 'CREATE_|UPDATE_|DELETE_|✅|❌|Error|error' | grep --line-buffered -vE 'IN_PROGRESS.*(Lambda|Role|Policy)' ; then
    :
  fi
  local st; st=$(stack_status "$STACK" "$REGION")
  [[ "$st" =~ ^(CREATE_COMPLETE|UPDATE_COMPLETE)$ ]] || { tail -40 "$log" >&2; die "the deployment did not complete ($STACK: $st; log: $log)"; }
  ok "$STACK is $st"
}

# review_update: change sets for both stacks; nothing to do, a change to confirm, or a refusal (data would be lost).
INFRA_CHANGED=0
review_update() {
  step "Infrastructure changes"
  local name log="$WORK/diff.log"
  name="factoryos-installer-$(date +%s)"
  cdk deploy --all --no-execute --change-set-name "$name" --require-approval never >"$log" 2>&1 \
    || { tail -30 "$log" >&2; die "could not prepare the change sets (log: $log)"; }
  INFRA_CHANGED=0
  local s r cs danger=() interrupt=()
  for s in "$STACK:$REGION" "$STACK-edge:$EDGE_REGION"; do
    IFS=: read -r cs r <<<"$s"
    local d
    d=$(aws --region "$r" cloudformation describe-change-set --stack-name "$cs" --change-set-name "$name" --output json 2>/dev/null) \
      || { ok "$cs: no change"; continue; }
    local status reason; status=$(json 'j.get("Status")' <<<"$d"); reason=$(json 'j.get("StatusReason") or ""' <<<"$d")
    if [ "$status" = FAILED ] && grep -qiE "didn't contain changes|No updates" <<<"$reason"; then
      ok "$cs: no change"
    elif [ "$status" = CREATE_COMPLETE ]; then
      INFRA_CHANGED=1
      say "$cs:"
      json '"\n".join("  %-8s %-40s %-32s replace=%s" % (c["ResourceChange"]["Action"], c["ResourceChange"]["LogicalResourceId"], c["ResourceChange"]["ResourceType"], c["ResourceChange"].get("Replacement","-")) for c in j["Changes"])' <<<"$d"
      local bad stops
      bad=$(change_set_danger <<<"$d")
      [ -z "$bad" ] || danger+=("$bad")
      stops=$(change_set_stops <<<"$d")
      [ -z "$stops" ] || interrupt+=("$stops")
    else
      warn "$cs: change set $status $reason"
      INFRA_CHANGED=1
    fi
    aws --region "$r" cloudformation delete-change-set --stack-name "$cs" --change-set-name "$name" >/dev/null 2>&1 || true
  done
  if [ "${#danger[@]}" -gt 0 ]; then
    warn "this update would replace or remove resources that hold data:"
    printf '  %s\n' "${danger[@]}" >&2
    say "  The instance and its volume hold every cell, catalogue item and plan; the user pool holds the users."
    [ "$ALLOW_REPLACE" = 1 ] || die "refused. Rerun with --allow-replace only if losing that data is acceptable (install.sh uninstall --snapshot keeps a copy of the volume)."
    confirm "Replace them and lose their data?"
  fi
  if [ "${#interrupt[@]}" -gt 0 ]; then
    warn "this update changes the instance's user data: CloudFormation stops and starts the instance (a few minutes;"
    warn "its volume and data stay, a running build or planning job is interrupted, sessions reconnect):"
    printf '  %s\n' "${interrupt[@]}" >&2
  fi
  if [ "$INFRA_CHANGED" = 1 ]; then confirm "Apply these infrastructure changes?"; fi
}

# The resource types that hold data. A change set marks AWS::EC2::Instance `Conditional` whenever its UserData
# changes (Details: Properties.UserData, RequiresRecreation Conditionally): for an EBS-backed instance, which this one
# is (BlockDeviceMappings /dev/sda1, an EBS root), CloudFormation applies that by stopping and starting it, not by
# replacing it (test install rf-old3, the update to v0.1.22's user data). So an instance whose only change that can
# recreate it is UserData is an interruption (change_set_stops), not a data loss (change_set_danger); anything else
# that is True or Conditional is.
DATA_TYPES='("AWS::EC2::Instance","AWS::EC2::Volume","AWS::S3::Bucket","AWS::Cognito::UserPool")'
USERDATA_ONLY='(c["ResourceChange"]["ResourceType"]=="AWS::EC2::Instance" and c["ResourceChange"]["Action"]=="Modify" and c["ResourceChange"].get("Replacement")=="Conditional" and {d.get("Target",{}).get("Name") for d in c["ResourceChange"].get("Details") or [] if d.get("Target",{}).get("RequiresRecreation") in ("Always","Conditionally")}=={"UserData"})'

# change_set_danger < change set JSON: "<id> (<type>): removed|replaced" for every data resource it loses.
change_set_danger() {
  json '"\n".join("%s (%s): %s" % (c["ResourceChange"]["LogicalResourceId"], c["ResourceChange"]["ResourceType"], "removed" if c["ResourceChange"]["Action"]=="Remove" else "replaced") for c in j["Changes"] if c["ResourceChange"]["ResourceType"] in '"$DATA_TYPES"' and (c["ResourceChange"]["Action"]=="Remove" or c["ResourceChange"].get("Replacement") in ("True","Conditional")) and not '"$USERDATA_ONLY"')'
}

# change_set_stops < change set JSON: "<id> (AWS::EC2::Instance): user data changed" for an instance that is stopped
# and started.
change_set_stops() {
  json '"\n".join("%s (%s): user data changed" % (c["ResourceChange"]["LogicalResourceId"], c["ResourceChange"]["ResourceType"]) for c in j["Changes"] if '"$USERDATA_ONLY"')'
}
