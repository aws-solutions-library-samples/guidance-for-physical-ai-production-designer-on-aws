#!/usr/bin/env bash
# Tests of deployment/install.sh's logic, with a fake `aws`, `node`, `npm` and `npx` (and logging wrappers of `git`
# and `unzip`) on PATH: argument checks, version order, mode detection, the update's refusals (same version,
# downgrade, data-holding resources replaced, a sha256 mismatch), the region confirmation, the edge-config region
# check, install --from-checkout (clean, dirty, not git), the release step's parameter, upload and command, the users'
# gate, uninstall --confirm-stack, the origin secret (made, kept, rotated, deleted; never in argv or output),
# --umarell-jam-tarball (s3:// or a local file: checked before anything changes, uploaded, in wanted and the options,
# kept by an update while the source bucket has it, '' drops it) and a ready state's `incomplete` in status. No AWS
# calls.
#   bash deployment/installer/tests/test-installer.sh
# Needs bash, python3, git, unzip, tar and coreutils (macOS, or Linux: test:deploy's python:3.12-bookworm,
# verify-build's amazonlinux:2023, as root too); no zip, node or npm (the fixtures are zipped with python3's zipfile).
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
dep="$(cd "$here/../.." && pwd)"
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
# The installer under test, in a tree of its own: inside a release export, deployment/../source/VERSION.json names the
# release (v0.1.19), and the installer then refuses the fixtures' other versions ("this install.sh is from ...").
mkdir -p "$T/tree" && (cd "$dep" && tar -cf - --exclude ./node_modules --exclude ./cdk.out .) | (mkdir -p "$T/tree/deployment" && cd "$T/tree/deployment" && tar -xf -) && dep="$T/tree/deployment"
pass=0 fail=0
check() { if [ "$2" = 0 ]; then pass=$((pass+1)); echo "ok    $1"; else fail=$((fail+1)); echo "FAIL  $1"; [ -z "${3:-}" ] || printf '%s\n' "$3" | sed 's/^/      /'; fi; }
# in_order <text>...: each one is in $T/calls, on a line after the one before it.
in_order() { local prev=0 n t; for t in "$@"; do n=$(grep -nF -- "$t" "$T/calls" | head -1 | cut -d: -f1); [ "${n:-0}" -gt "$prev" ] || return 1; prev=$n; done; }

# --- the fake aws: logs its argv to $T/calls, answers from the handler the test writes to $T/handler.sh --------
mkdir -p "$T/bin"
cat > "$T/bin/aws" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$FAKE/calls"
args=("$@"); r=""
# drop --region <r>, remember it
out=(); i=0; while [ $i -lt ${#args[@]} ]; do
  if [ "${args[$i]}" = --region ]; then r="${args[$((i+1))]}"; i=$((i+2)); else out+=("${args[$i]}"); i=$((i+1)); fi
done
set -- "${out[@]}"
REGION_ARG="$r"
. "$FAKE/handler.sh"
handle "$@"
EOF
chmod +x "$T/bin/aws"
# git and unzip: the real ones, their argv logged; node, npm and npx: fakes (the CDK steps are stubbed in run below;
# node answers preflight's version check, so the host needs no Node.js, or not that one).
real_git=$(command -v git) || { echo "FAIL  git is needed for the --from-checkout tests"; exit 1; }
real_unzip=$(command -v unzip) || { echo "FAIL  unzip is needed"; exit 1; }
printf '#!/usr/bin/env bash\nprintf "%%s\\n" "$*" >> "$FAKE/git-calls"\nexec %q "$@"\n' "$real_git" > "$T/bin/git"
printf '#!/usr/bin/env bash\nprintf "%%s\\n" "$*" >> "$FAKE/unzip-calls"\nexec %q "$@"\n' "$real_unzip" > "$T/bin/unzip"
cat > "$T/bin/npx" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$FAKE/npx-calls"
case "$*" in *"cdk --version"*) echo "2.1000.0 (build fake)" ;; esac
EOF
cat > "$T/bin/node" <<'EOF'
#!/usr/bin/env bash
case "$*" in
  --version) echo v22.11.0 ;;
  "-p process.versions.node.split(\".\")[0]") echo 22 ;;
  *) echo "fake node: unhandled: $*" >&2; exit 2 ;;
esac
EOF
cat > "$T/bin/npm" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$FAKE/npm-calls"
EOF
chmod +x "$T/bin/git" "$T/bin/unzip" "$T/bin/npx" "$T/bin/node" "$T/bin/npm"
export FAKE="$T" PATH="$T/bin:$PATH"
# The wait for Lambda's late log delivery (the uninstall's sweep of the stack's own groups): none in the tests.
export FACTORYOS_LOG_LAG_SECONDS=0
# The stack's region in these tests ($AWS_REGION), another one, and the edge's.
reg=eu-central-1 other=us-west-2 edge=us-east-1
# The fixtures' git must not see the developer's own configuration (signing, hooks, templates).
export GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_NOSYSTEM=1 GIT_AUTHOR_NAME=t GIT_AUTHOR_EMAIL=t@example.com \
  GIT_COMMITTER_NAME=t GIT_COMMITTER_EMAIL=t@example.com

# A default handler: account 111122223333, nothing installed.
base_handler() {
cat <<'EOF'
stacks_main=NONE; stacks_edge=NONE
acct=111122223333         # the caller's account: the AWS documentation's example account id
outputs='{"Url":"https://d111111abcdef8.cloudfront.net/","UserPoolId":"eu-central-1_EXAMPLE","SourceBucket":"src-bucket","ReleaseDocument":"factoryos-Release","InstanceId":"i-0123456789abcdef0","LogGroup":"/factoryos/factoryos","GateVerified":"true"}'
state_param=""
edge_config=""            # the value of /factoryos/factoryos/edge-config in the edge region ("" = ParameterNotFound)
edge_config_error=""      # an error reading it instead
sha256_answer=""          # the content of <export uri>.sha256 ("" = there is none)
changeset_main='{"Status":"FAILED","StatusReason":"The submitted information didn'"'"'t contain changes."}'
changeset_edge='{"Status":"FAILED","StatusReason":"No updates are to be performed."}'
instance_offline=""       # 1: SSM does not see the instance (wait_instance_ready runs into its deadline)
send_command_fail=""      # 1: ssm send-command fails
edge_put_fail=""          # 1: writing the origin secret in us-east-1 fails (a rotation stopped in step 2)
build_status="SUCCEEDED"  # the release build's buildStatus (--build-project); $FAKE/bundle.json is its bundle.json
[ -f "$FAKE/overrides.sh" ] && . "$FAKE/overrides.sh"
handle() {
  case "$1 $2" in
    "--version "*|"--version") echo "aws-cli/2.17.0 Python/3.11" ;;
    "sts get-caller-identity") echo "{\"Account\":\"$acct\",\"Arn\":\"arn:aws:sts::$acct:assumed-role/Admin/test\"}" ;;
    "cloudformation describe-stacks")
      local name="" q=""; while [ $# -gt 0 ]; do case "$1" in --stack-name) name="$2"; shift ;; --query) q="$2"; shift ;; esac; shift; done
      local st; if [[ "$name" == *-edge ]]; then st=$stacks_edge; else st=$stacks_main; fi
      [ "$st" = NONE ] && { echo "An error occurred (ValidationError): Stack with id $name does not exist" >&2; exit 254; }
      case "$q" in
        *StackStatus*) echo "$st" ;;
        *OutputKey==*) k=$(sed -E "s/.*OutputKey=='([A-Za-z]+)'.*/\1/" <<<"$q"); python3 -c 'import json,sys;print(json.loads(sys.argv[1]).get(sys.argv[2],"None"))' "$outputs" "$k" ;;
      esac ;;
    "cloudformation describe-change-set")
      local name=""; while [ $# -gt 0 ]; do case "$1" in --stack-name) name="$2"; shift ;; esac; shift; done
      if [[ "$name" == *-edge ]]; then echo "$changeset_edge"; else echo "$changeset_main"; fi ;;
    "cloudformation delete-change-set") ;;
    "ssm get-parameter")
      local all="$*" name=""; while [ $# -gt 0 ]; do case "$1" in --name) name="$2"; shift ;; esac; shift; done
      case "$name" in
        */origin-secret|*/edge-origin-secret)
          # $FAKE/secure/<region><name with _>: the last put-parameter request (JSON); $FAKE/secure-error: an error.
          [ ! -f "$FAKE/secure-error" ] || { cat "$FAKE/secure-error" >&2; exit 254; }
          local sf="$FAKE/secure/$REGION_ARG$(tr / _ <<<"$name")"
          [ -f "$sf" ] || { echo "An error occurred (ParameterNotFound) when calling the GetParameter operation" >&2; exit 254; }
          case "$all" in
            *"--query Parameter.Type"*) python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["Type"])' "$sf" ;;
            *--with-decryption*"--query Parameter.Value"*) python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["Value"])' "$sf" ;;
            *) echo "fake aws: the origin secret read neither for its type nor with decryption: $*" >&2; exit 2 ;;
          esac ;;
        */edge-config)
          [ "$REGION_ARG" = us-east-1 ] || { echo "edge-config read outside the edge region" >&2; exit 3; }
          [ -z "$edge_config_error" ] || { echo "$edge_config_error" >&2; exit 254; }
          [ "$name" = /factoryos/factoryos/edge-config ] && [ -n "$edge_config" ] && echo "$edge_config" || { echo "An error occurred (ParameterNotFound) when calling the GetParameter operation" >&2; exit 254; } ;;
        # $FAKE/state, when it is there, is the instance's (written after the deploy; empty: none yet).
        */release/state)
          if [ -e "$FAKE/state" ]; then [ -s "$FAKE/state" ] && cat "$FAKE/state" || { echo "ParameterNotFound" >&2; exit 254; }
          else [ -n "$state_param" ] && echo "$state_param" || { echo "ParameterNotFound" >&2; exit 254; }; fi ;;
        */release/wanted) [ -f "$FAKE/wanted" ] && cat "$FAKE/wanted" || { echo "ParameterNotFound" >&2; exit 254; } ;;
        /cdk-bootstrap/*) echo 21 ;;
        */install-options) [ -f "$FAKE/options" ] && cat "$FAKE/options" || { echo "ParameterNotFound" >&2; exit 254; } ;;
        /aws/service/canonical/*) echo ami-1234567890abcdef0 ;;
      esac ;;
    "ssm describe-parameters")
      # The Description of the origin secret's last request (the value is not read); None when there is none.
      local all="$*" sf
      [[ "$all" == *"--query Parameters[0].Description"* ]] || { echo "fake aws: describe-parameters: $*" >&2; exit 2; }
      sf="$FAKE/secure/$REGION_ARG$(sed -E 's/.*Key=Name,Values=([^ ]+).*/\1/' <<<"$all" | tr / _)"
      if [ -f "$sf" ]; then python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["Description"])' "$sf"; else echo None; fi ;;
    "ssm put-parameter")
      local a cij=""; for a in "$@"; do case "$a" in file://*) cij="${a#file://}" ;; esac; done
      if [ -n "$cij" ] && [ "$edge_put_fail" = 1 ] && [ "$REGION_ARG" = us-east-1 ]; then
        echo "An error occurred (ThrottlingException) when calling the PutParameter operation: Rate exceeded" >&2; exit 254
      fi
      if [ -n "$cij" ]; then
        # A SecureString from a request file: kept as $FAKE/secure/<region><name>; the file's and its directory's
        # modes and path into secure-files, region, name, type and value into secure-history (the test's view only).
        mkdir -p "$FAKE/secure"
        local n; n=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["Name"])' "$cij")
        cp "$cij" "$FAKE/secure/$REGION_ARG$(tr / _ <<<"$n")"
        python3 -c 'import os,sys;p=sys.argv[1];print(oct(os.stat(p).st_mode&0o777)[2:],oct(os.stat(os.path.dirname(p)).st_mode&0o777)[2:],p)' "$cij" >> "$FAKE/secure-files"
        python3 -c 'import json,sys;j=json.load(open(sys.argv[2]));print(sys.argv[1],j["Name"],j["Type"],j["Overwrite"],j["Value"])' "$REGION_ARG" "$cij" >> "$FAKE/secure-history"
        exit 0
      fi
      local name="" value=""; while [ $# -gt 0 ]; do case "$1" in --name) name="$2"; shift ;; --value) value="$2"; shift ;; esac; shift; done
      [[ "$name" == */wanted ]] && printf '%s' "$value" > "$FAKE/wanted"
      [[ "$name" == */install-options ]] && printf '%s' "$value" > "$FAKE/options"; true ;;
    "ssm describe-instance-information") [ "$instance_offline" = 1 ] && echo ConnectionLost || echo Online ;;
    "ssm send-command")
      [ "$send_command_fail" != 1 ] || { echo "An error occurred (InvalidInstanceId) when calling the SendCommand operation" >&2; exit 254; }
      echo cmd-123 ;;
    "ssm get-command-invocation")
      case "$*" in *Status*) echo Success ;; *StandardOutput*) echo "release started" ;; *) echo "" ;; esac ;;
    "codebuild start-build") echo "pai-pd-installer-build:0f1e2d3c" ;;
    "codebuild batch-get-builds") printf '%s\tBUILD\thttps://console.example/build-log\n' "$build_status" ;;
    "codebuild stop-build") ;;
    "ec2 describe-instances") echo ami-0123456789abcdef0 ;;
    "ec2 get-console-output") [ -f "$FAKE/console" ] && cat "$FAKE/console" ;;
    "s3 cp")
      # The release build: its staged source kept for the test; its bundle.json from $FAKE/bundle.json.
      if [[ "${4:-}" == s3://*/builds/*/source.zip ]]; then cp "$3" "$FAKE/staged-source.zip"; exit 0; fi
      if [[ "$3" == s3://*/builds/*/out/bundle.json ]]; then [ -f "$FAKE/bundle.json" ] || exit 1; cat "$FAKE/bundle.json"; exit 0; fi
      if [[ "$3" == s3://*umarell-jam* ]]; then cp "$FAKE/umarell-jam.tar.gz" "$4"; exit 0; fi
      if [[ "$3" == *umarell* ]]; then cp "$FAKE/umarell.tar.gz" "$4"; exit 0; fi
      if [[ "$*" == *.sha256* ]]; then [ -n "$sha256_answer" ] || exit 1; echo "$sha256_answer  export.zip"; exit 0; fi
      case "$4" in s3://*|-) ;; *) cp "$FAKE/export.zip" "$4" ;; esac ;;
    "s3api get-bucket-location") echo eu-central-1 ;;
    # $FAKE/vendor-gone: the source bucket no longer has the object.
    "s3api head-object") [ ! -f "$FAKE/vendor-gone" ] || { echo "An error occurred (404) when calling the HeadObject operation: Not Found" >&2; exit 254; }; echo '{}' ;;
    "cognito-idp admin-create-user") echo FORCE_CHANGE_PASSWORD ;;
    "s3api list-object-versions") echo '{}' ;;
    "s3api delete-objects"|"cloudformation delete-stack"|"cloudformation wait"|"ssm delete-parameter") ;;
    # The stack's parameters that exist: $FAKE/params-left (one name a line), none without it.
    "ssm get-parameters") [ ! -f "$FAKE/params-left" ] || tr '\n' '\t' < "$FAKE/params-left" ;;
    "cloudformation list-stack-resources")
      local name="" query=""
      while [ $# -gt 0 ]; do case "$1" in --stack-name) name="$2"; shift ;; --query) query="$2"; shift ;; esac; shift; done
      if [[ "$query" == *Logs::LogGroup* ]]; then
        # The groups the stack owns (the CDK names them <stack>-<construct><hash>-<suffix>); the edge stack's is its
        # function's custom log group (EdgeAuthLogs: one name in every Region the replicas ran in).
        if [[ "$name" == *-edge ]]; then printf '%s\n' "$name-EdgeAuthLogs0123ABCD-qRsTuVwXyZaB"
        else printf '%s\t%s\n' "$name-CustomResourceLogsE45D40F0-aAbBcCdDeEfF" "$name-VpcFlowLogsE6FFDEF9-gGhHiIjJkKlL"; fi
      elif [[ "$name" == *-edge ]]; then printf '%s\n' "$name-EdgeAuth0123ABCD-aBcDeFgHiJkL"
      else printf '%s\t%s\n' "$name-CustomS3AutoDeleteObjectsCustomRes-mNoPqRsTuVwX" "$name-VerifyAnonymousDenialFun-yZaBcDeFgHiJ"; fi ;;
    "ec2 describe-regions") printf 'us-east-1\teu-central-1\tus-west-2\n' ;;
    # The release log's events (--query events[].message --output json): $FAKE/log-events, else none.
    "logs get-log-events") [ -f "$FAKE/log-events" ] && cat "$FAKE/log-events" || echo '[]' ;;
    "cognito-idp list-users") printf 'admin@example.com\tCONFIRMED\tTrue\n' ;;
    "logs delete-log-group")
      # Only the groups Lambda created exist: the provider's in the stack's Region, the replicas' everywhere.
      case "$*" in
        *CustomS3AutoDelete*|*"/aws/lambda/us-east-1."*EdgeAuth*|*"-edge-EdgeAuthLogs"*|*CustomResourceLogs*) ;;
        *) echo "An error occurred (ResourceNotFoundException) when calling the DeleteLogGroup operation" >&2; exit 254 ;;
      esac ;;
    *) echo "fake aws: unhandled: $*" >&2; exit 2 ;;
  esac
}
EOF
}
reset() { rm -f "$T/calls" "$T/wanted" "$T/overrides.sh" "$T/options" "$T/state" "$T/console" "$T/git-calls" "$T/unzip-calls" "$T/npx-calls" "$T/npm-calls" \
  "$T/secure-files" "$T/secure-history" "$T/secure-error" "$T/vendor-gone" "$T/bundle.json" "$T/staged-source.zip"; rm -rf "$T/work" "$T/secure"; base_handler > "$T/handler.sh"; }
override() { printf '%s\n' "$@" >> "$T/overrides.sh"; base_handler > "$T/handler.sh"; }

# mkzip <dir> <zip> <entry>...: the entries of <dir> into <zip> (replaced), with python3's zipfile and not zip(1),
# which the Linux hosts that run this suite (test:deploy's python:3.12-bookworm, verify-build's amazonlinux:2023) do
# not have. A fixture that cannot be built stops the suite here, instead of failing every case after it for a reason
# they do not name.
mkzip() { local d=$1 z=$2; shift 2; rm -f "$z"; (cd "$d" && python3 -I -m zipfile -c "$z" "$@") && [ -s "$z" ] \
  || { echo "FAIL  cannot build the fixture $z from $d: $*"; exit 1; }; }
# A tiny export zip with a VERSION.json.
mkdir -p "$T/x/factoryos-v0.2.0/deployment"
echo '{"format":1,"release":"v0.2.0","tree":""}' > "$T/x/factoryos-v0.2.0/VERSION.json"
mkzip "$T/x" "$T/export.zip" factoryos-v0.2.0

# run <args...>: install.sh's functions in a subshell, the stubs for the CDK steps replaced; prints output, sets rc.
run() {
  ( export FACTORYOS_WORK="$T/work" AWS_REGION=$reg FACTORYOS_READY_TIMEOUT=2
    INSTALLER_DIR="${RUN_INSTALLER_DIR:-$dep}"; . "$dep/installer/lib.sh"
    for p in preflight infra release users status uninstall origin; do . "$dep/installer/$p.sh"; done
    prepare_app() { APP="$T/app"; }
    bootstrap() { :; }
    deploy_stacks() { echo "DEPLOY_STACKS"; echo 'stacks_main=CREATE_COMPLETE; stacks_edge=CREATE_COMPLETE; state_param={\"phase\":\"waiting\"}' >> "$FAKE/overrides.sh"; base_handler > "$FAKE/handler.sh"; }
    cdk() { echo "cdk $* ${CDK_CONTEXT[*]}" >> "$FAKE/calls"; }
    sleep() { [ "${1:-}" != "${FACTORYOS_ORIGIN_EDGE_WAIT:-420}" ] || echo "SLEEP $1" >> "$FAKE/calls"; }
    set -euo pipefail
    main "$@" ) </dev/null 2>&1
}

# --- arguments ------------------------------------------------------------------------------------------------
reset
out=$(run install); check "install without a URI is refused" $([ $? != 0 ] && grep -q 'exactly one s3://' <<<"$out"; echo $?) "$out"
out=$(run install s3://exports-bkt/x.txt --yes); check "a non-zip export is refused" $([ $? != 0 ] && grep -q 'must be an s3://' <<<"$out"; echo $?) "$out"
out=$(run status --stack Bad_Name); check "a bad stack name is refused" $([ $? != 0 ] && grep -q 'must match' <<<"$out"; echo $?) "$out"
out=$(run install s3://exports-bkt/x.zip --instance-type m7i.large --yes); check "a non-Graviton instance type is refused" $([ $? != 0 ] && grep -q Graviton <<<"$out"; echo $?) "$out"
out=$(run --help); check "--help prints the usage" $(grep -q 'install.sh status' <<<"$out"; echo $?) "$out"

# --- version order --------------------------------------------------------------------------------------------
v=$( INSTALLER_DIR="$dep"; . "$dep/installer/lib.sh"; echo "$(version_cmp v0.1.9 v0.1.10) $(version_cmp v1.0.0 v0.9.9) $(version_cmp v0.2.0 v0.2.0) $(version_cmp dev-abc v0.1.0)")
check "version_cmp orders numerically, ? for dev builds" $([ "$v" = "-1 1 0 ?" ]; echo $?) "$v"

# --- a fresh install --------------------------------------------------------------------------------------------
reset
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --users a@example.com); rc=$?
check "fresh account: install runs to the end" "$rc" "$out"
check "fresh account: deploys the stacks" $(grep -q DEPLOY_STACKS <<<"$out"; echo $?) "$out"
check "fresh account: the newest Ubuntu AMI is looked up" $(grep -q 'ssm get-parameter --name /aws/service/canonical/ubuntu/server/24.04' "$T/calls"; echo $?)
w=$(cat "$T/wanted" 2>/dev/null)
check "wanted names the version, key and sha256" $(python3 -c 'import json,sys;j=json.loads(sys.argv[1]);assert j["version"]=="v0.2.0" and j["key"]=="releases/v0.2.0.zip" and len(j["sha256"])==64 and j["umarell"] is None and j["umarellJam"] is None' "$w" 2>/dev/null; echo $?) "$w"
check "the export is copied into the source bucket" $(grep -q 's3 cp s3://exports-bkt/factoryos-v0.2.0.zip s3://src-bucket/releases/v0.2.0.zip' "$T/calls"; echo $?) "$(cat "$T/calls")"
check "the release document is sent to the instance" $(grep -q 'ssm send-command --document-name factoryos-Release --document-version $LATEST --instance-ids i-0123456789abcdef0' "$T/calls"; echo $?)
check "a fresh install records its options" $(python3 -c 'import json;j=json.load(open("'"$T"'/options"));assert j=={"instanceType":"m7g.2xlarge","volumeGb":"200","bedrock":False,"kiroSecretName":"","bedrockModelIds":"","umarellJam":None}' 2>/dev/null; echo $?) "$(cat "$T/options" 2>/dev/null)"
check "the user is invited" $(grep -q 'cognito-idp admin-create-user --user-pool-id eu-central-1_EXAMPLE --username a@example.com' "$T/calls"; echo $?)
check "without --build-project: no release build, no bundle in wanted" \
  $(! grep -q '^codebuild ' "$T/calls" && python3 -c 'import json,sys;assert "bundle" not in json.loads(sys.argv[1])' "$w" 2>/dev/null; echo $?) "$w"

# --- the release build in AWS CodeBuild (--build-project, --build-bucket) -----------------------------------------
reset
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --build-project pai-pd-installer-build); check "--build-project without --build-bucket is refused" $([ $? != 0 ] && grep -q 'go together' <<<"$out"; echo $?) "$out"
out=$(run status --build-project p --build-bucket b); check "--build-project is an option of install" $([ $? != 0 ] && grep -q 'options of install' <<<"$out"; echo $?) "$out"
esha=$(sha256sum "$T/export.zip" 2>/dev/null | cut -d' ' -f1 || shasum -a 256 "$T/export.zip" | cut -d' ' -f1)
bsha=$(printf 'bundle' | (sha256sum 2>/dev/null || shasum -a 256) | cut -d' ' -f1)
bundle_json() { printf '{"version":"%s","exportSha256":"%s","sha256":"%s","arch":"linux-arm64","builtBy":"arn:aws:codebuild:x"}\n' "$1" "$2" "$bsha" > "$T/bundle.json"; }
reset; bundle_json v0.2.0 "$esha"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --build-project pai-pd-installer-build --build-bucket inputs-bkt); rc=$?
check "release build: install runs to the end" "$rc" "$out"
check "release build: the export is staged as builds/<version>-<sha12>/source.zip, the archive itself inside" \
  $(grep -qF "s3://inputs-bkt/builds/v0.2.0-${esha:0:12}/source.zip" "$T/calls" \
     && [ "$(python3 -I -c 'import hashlib,sys,zipfile;z=zipfile.ZipFile(sys.argv[1]);assert z.namelist()==["export.zip"];print(hashlib.sha256(z.read("export.zip")).hexdigest())' "$T/staged-source.zip" 2>/dev/null)" = "$esha" ]; echo $?) "$(cat "$T/calls")"
check "release build: started with the staged source, the artifacts' path, the version and the export's sha256" \
  $(grep -F 'codebuild start-build --project-name pai-pd-installer-build --source-type-override S3' "$T/calls" \
     | grep -F "inputs-bkt/builds/v0.2.0-${esha:0:12}/source.zip" | grep -F "path=builds/v0.2.0-${esha:0:12},name=out" \
     | grep -F 'name=RELEASE_VERSION,value=v0.2.0,type=PLAINTEXT' | grep -qF "name=EXPORT_SHA256,value=$esha,type=PLAINTEXT"; echo $?) "$(grep codebuild "$T/calls")"
check "release build: started before the stacks deploy, awaited before the release is sent" \
  $(in_order 'codebuild start-build' 'ssm get-parameter --name /cdk-bootstrap' 'codebuild batch-get-builds' 'ssm send-command' 2>/dev/null \
     || in_order 'codebuild start-build' 'codebuild batch-get-builds' 'ssm send-command'; echo $?) "$(cat "$T/calls")"
check "release build: its bundle is copied next to the export, under releases/" \
  $(grep -qF "s3 cp s3://inputs-bkt/builds/v0.2.0-${esha:0:12}/out/bundle.tar.gz s3://src-bucket/releases/factoryos-viewer-linux-arm64-v0.2.0.tar.gz" "$T/calls"; echo $?) "$(cat "$T/calls")"
w=$(cat "$T/wanted" 2>/dev/null)
check "release build: wanted names the bundle and its sha256" \
  $(python3 -c 'import json,sys;j=json.loads(sys.argv[1]);assert j["bundle"]=={"key":"releases/factoryos-viewer-linux-arm64-v0.2.0.tar.gz","sha256":sys.argv[2]} and j["key"]=="releases/v0.2.0.zip"' "$w" "$bsha" 2>/dev/null; echo $?) "$w"
check "release build: a finished build is not stopped" $(! grep -q 'codebuild stop-build' "$T/calls"; echo $?)
reset; bundle_json v0.2.0 "$(printf 'other' | (sha256sum 2>/dev/null || shasum -a 256) | cut -d' ' -f1)"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --build-project pai-pd-installer-build --build-bucket inputs-bkt); rc=$?
check "release build: a bundle of another export is refused, nothing sent to the instance" \
  $([ "$rc" != 0 ] && grep -q 'another export' <<<"$out" && ! grep -q 'ssm send-command' "$T/calls" && [ ! -f "$T/wanted" ]; echo $?) "$out"
reset; bundle_json v0.2.0 "$esha"; override 'build_status=FAILED'
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --build-project pai-pd-installer-build --build-bucket inputs-bkt); rc=$?
check "release build: a failed build fails the install with its log link, nothing sent, not stopped" \
  $([ "$rc" != 0 ] && grep -q 'ended FAILED' <<<"$out" && grep -q 'https://console.example/build-log' <<<"$out" \
     && ! grep -q 'ssm send-command' "$T/calls" && ! grep -q 'codebuild stop-build' "$T/calls"; echo $?) "$out"
reset; bundle_json v0.2.0 "$esha"; override 'build_status=IN_PROGRESS'
out=$(FACTORYOS_BUILD_TIMEOUT_SECONDS=0 run s3://exports-bkt/factoryos-v0.2.0.zip --yes --build-project pai-pd-installer-build --build-bucket inputs-bkt); rc=$?
check "release build: one still running when install.sh stops is stopped" \
  $([ "$rc" != 0 ] && grep -q 'codebuild stop-build --ids pai-pd-installer-build:0f1e2d3c' "$T/calls" && grep -q 'stopped the release build' <<<"$out"; echo $?) "$out"
check "nothing secret-looking in any argv" $(! grep -qiE 'password|secret-string|KIRO_API_KEY' "$T/calls"; echo $?)

# --- the instance's first boot (2026-11-14: its user data failed, the installer said "running" for an hour) --------
# A first-boot failure in the state (user-data.sh's: phase failed, no version): stopped at once, with its error.
reset
echo '{"phase":"failed","version":"","previous":null,"startedAt":"2026-11-14T00:00:00Z","updatedAt":"2026-11-14T00:00:00Z","finishedAt":"2026-11-14T00:00:00Z","error":"first-boot setup failed (exit 1, line 190 of part-001): net download https://amazoncloudwatch-agent.s3.amazonaws.com/ubuntu/arm64/latest/amazon-cloudwatch-agent.deb","logStream":"i-0123456789abcdef0/bootstrap.log"}' > "$T/state"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "a failed first boot: install stops" $([ $rc != 0 ]; echo $?) "$out"
check "a failed first boot: its error is shown" $(grep -q 'first-boot setup failed (exit 1, line 190 of part-001): net download' <<<"$out"; echo $?) "$out"
check "a failed first boot: where its log is" $(grep -q 'stream i-0123456789abcdef0/bootstrap.log' <<<"$out"; echo $?) "$out"
check "a failed first boot: no release is sent" $(! grep -q 'ssm send-command' "$T/calls"; echo $?) "$(cat "$T/calls")"
# No state when the time is up (a failure before the AWS CLI is on the instance): the console's last user-data lines.
reset; : > "$T/state"
printf '%s\n' '[   66.1] cloud-init[1393]: [bootstrap 2026-11-14T13:26:31Z] FAILED: first-boot setup failed (exit 1, line 165 of part-001): net apt_get update -y -qq --error-on=any' \
  '[   66.2] cloud-init[1393]: 2026-11-14 13:26:31,503 - cc_scripts_user.py[WARNING]: Failed to run module scripts_user (scripts in /var/lib/cloud/instance/scripts)' \
  '[   70.0] systemd[1]: Started something unrelated.' > "$T/console"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "no state in time: install stops" $([ $rc != 0 ]; echo $?) "$out"
check "no state in time: says the setup has not finished (not 'running')" $(grep -q 'first-boot setup: not finished, no state parameter' <<<"$out"; echo $?) "$out"
check "no state in time: the console's user-data lines" $(grep -q 'FAILED: first-boot setup failed (exit 1, line 165 of part-001)' <<<"$out" && grep -q 'Failed to run module scripts_user' <<<"$out" && ! grep -q unrelated <<<"$out"; echo $?) "$out"
check "no state in time: no release is sent" $(! grep -q 'ssm send-command' "$T/calls"; echo $?) "$(cat "$T/calls")"

# --- refusals and states ----------------------------------------------------------------------------------------
reset; override 'stacks_main=ROLLBACK_COMPLETE' 'stacks_edge=CREATE_COMPLETE'
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "a rolled-back first install: refused, uninstall suggested" $([ $? != 0 ] && grep -q 'uninstall' <<<"$out"; echo $?) "$out"
reset; override 'stacks_main=UPDATE_IN_PROGRESS' 'stacks_edge=CREATE_COMPLETE'
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "a stack operation in progress: refused" $([ $? != 0 ] && grep -q 'in progress' <<<"$out"; echo $?) "$out"
reset; override 'stacks_main=CREATE_COMPLETE' 'stacks_edge=NONE'
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "the main stack without the edge stack: refused" $([ $? != 0 ] && grep -q 'does not repair' <<<"$out"; echo $?) "$out"

up='stacks_main=UPDATE_COMPLETE; stacks_edge=CREATE_COMPLETE'
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.2.0","previous":"v0.1.9"}'"'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "the same version: nothing to do" $([ $? = 0 ] && grep -q 'installed already' <<<"$out" && ! grep -q 'send-command' "$T/calls"; echo $?) "$out"
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.3.0"}'"'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "a downgrade: refused" $([ $? != 0 ] && grep -q 'older' <<<"$out"; echo $?) "$out"
reset; override "$up" 'state_param='"'"'{"phase":"building","version":"v0.1.9"}'"'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "a release in progress: refused" $([ $? != 0 ] && grep -q 'in progress on the instance' <<<"$out"; echo $?) "$out"

reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "an update without infrastructure change: no deploy, release sent" $([ $rc = 0 ] && ! grep -q DEPLOY_STACKS <<<"$out" && grep -q 'ssm send-command' "$T/calls"; echo $?) "$out"
check "an update keeps the instance's AMI" $(grep -q 'ec2 describe-instances --instance-ids i-0123456789abcdef0' "$T/calls"; echo $?)

cs_replace='{"Status":"CREATE_COMPLETE","Changes":[{"ResourceChange":{"Action":"Modify","LogicalResourceId":"Instance","ResourceType":"AWS::EC2::Instance","Replacement":"True"}}]}'
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'" "changeset_main='$cs_replace'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "an update replacing the instance: refused" $([ $? != 0 ] && grep -q 'allow-replace' <<<"$out" && ! grep -q send-command "$T/calls"; echo $?) "$out"
# The instance's user data changed (a release that changes user-data.sh): Conditional with UserData its only
# Details entry that can recreate it. CloudFormation stops and starts the EBS-backed instance: deployed, with a warning.
ud='{"Target":{"Attribute":"Properties","Name":"UserData","RequiresRecreation":"Conditionally"},"Evaluation":"Static","ChangeSource":"DirectModification"}'
tags='{"Target":{"Attribute":"Tags","RequiresRecreation":"Never"},"Evaluation":"Static","ChangeSource":"DirectModification"}'
ami='{"Target":{"Attribute":"Properties","Name":"ImageId","RequiresRecreation":"Always"},"Evaluation":"Static","ChangeSource":"DirectModification"}'
cs_ud='{"Status":"CREATE_COMPLETE","Changes":[{"ResourceChange":{"Action":"Modify","LogicalResourceId":"Instance","ResourceType":"AWS::EC2::Instance","Replacement":"Conditional","Details":['"$ud,$tags"']}}]}'
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'" "changeset_main='$cs_ud'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "an update changing the instance's user data only: deployed, the stop and start announced" \
  $([ $rc = 0 ] && grep -q DEPLOY_STACKS <<<"$out" && grep -q 'stops and starts the instance' <<<"$out" \
     && grep -q 'Instance (AWS::EC2::Instance): user data changed' <<<"$out" && ! grep -q 'allow-replace' <<<"$out"; echo $?) "$out"
cs_ud_ami='{"Status":"CREATE_COMPLETE","Changes":[{"ResourceChange":{"Action":"Modify","LogicalResourceId":"Instance","ResourceType":"AWS::EC2::Instance","Replacement":"Conditional","Details":['"$ud,$ami"']}}]}'
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'" "changeset_main='$cs_ud_ami'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes)
check "user data and another property that recreates the instance: refused" $([ $? != 0 ] && grep -q 'allow-replace' <<<"$out" && ! grep -q DEPLOY_STACKS <<<"$out"; echo $?) "$out"
cs_cond='{"Status":"CREATE_COMPLETE","Changes":[{"ResourceChange":{"Action":"Modify","LogicalResourceId":"Instance","ResourceType":"AWS::EC2::Instance","Replacement":"Conditional"}}]}'
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'" "changeset_main='$cs_cond'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes)
check "Conditional without Details: refused (nothing shows it is the user data)" $([ $? != 0 ] && grep -q 'allow-replace' <<<"$out"; echo $?) "$out"
cs_modify='{"Status":"CREATE_COMPLETE","Changes":[{"ResourceChange":{"Action":"Modify","LogicalResourceId":"EdgeFn","ResourceType":"AWS::Lambda::Function","Replacement":"False"}}]}'
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'" "changeset_edge='$cs_modify'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "an update with a harmless change: deployed" $([ $? = 0 ] && grep -q DEPLOY_STACKS <<<"$out"; echo $?) "$out"

reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'"
echo '{"instanceType":"m7g.4xlarge","volumeGb":"300","bedrock":true,"kiroSecretName":"kiro/key"}' > "$T/options" # pragma: allowlist secret (kiroSecretName: a Secrets Manager secret's name)
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --no-bedrock)
check "an update keeps the installed options, except the ones given" $(grep -q 'cdk deploy --all --no-execute.* -c instanceType=m7g.4xlarge -c volumeGb=300 -c bedrock=false -c kiroSecretName=kiro/key' "$T/calls"; echo $?) "$(grep '^cdk' "$T/calls")"
out=$(run status --stack amzn-x); check "a reserved stack prefix is refused" $([ $? != 0 ] && grep -q reserves <<<"$out"; echo $?) "$out"

# --- users and the gate -------------------------------------------------------------------------------------------
reset; override "$up" "outputs='{\"UserPoolId\":\"p\",\"GateVerified\":\"false\"}'"
out=$(run add-user b@example.com --region $reg); check "no users before the gate is verified" $([ $? != 0 ] && grep -q GateVerified <<<"$out" && ! grep -q admin-create-user "$T/calls"; echo $?) "$out"
reset; override "$up"
out=$(run add-user not-an-email --region $reg); check "a bad e-mail address is refused" $([ $? != 0 ] && grep -q 'not an e-mail' <<<"$out"; echo $?) "$out"

# --- uninstall needs the typed stack name ----------------------------------------------------------------------------
reset; override "$up"
out=$(FACTORYOS_TEST_TYPED=wrong run uninstall --yes); check "uninstall: a wrong confirmation stops it" $([ $? != 0 ] && ! grep -q delete-stack "$T/calls"; echo $?) "$out"

# --- the region: from $AWS_REGION it is printed and asked about ------------------------------------------------------
reset
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip); rc=$?
check "region from \$AWS_REGION: printed, and without --yes and a terminal it stops before anything" $([ $rc != 0 ] && grep -q "Region   $reg" <<<"$out" && grep -q 'not from --region' <<<"$out" && grep -q "Use the region $reg?" <<<"$out" && ! grep -q 'cloudformation' "$T/calls"; echo $?) "$out"
reset
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --region $reg); rc=$?
check "region from --region: not asked about (stops only at 'Go on?' without a terminal)" $([ $rc != 0 ] && ! grep -q 'Use the region' <<<"$out" && grep -q 'Go on?' <<<"$out" && ! grep -q 's3 cp s3://exports-bkt/factoryos-v0.2.0.zip s3://' "$T/calls"; echo $?) "$out"
reset
out=$(run status); check "status: the region is printed but not asked about" $([ $? = 0 ] && grep -q "Region   $reg" <<<"$out" && ! grep -q 'Use the region' <<<"$out"; echo $?) "$out"

# --- status of a failed release (test install 2611122014-dc89): the error a part a line, where the log is, and the
# log's tail with the restart loop collapsed so the deploy's own error stays in view ------------------------------------
reset
st_err="deploying: factoryos-deploy.sh exited 1: factoryos-deploy: backend did not become healthy; factoryos-backend.service: Failed to execute /opt/factoryos/current/bin/factoryos-backend: Permission denied; no previous release to roll back to"
override "$up" "state_param='{\"phase\":\"failed\",\"version\":\"dev-346f9851fc2e\",\"previous\":null,\"startedAt\":\"2026-11-12T20:41:39Z\",\"updatedAt\":\"2026-11-12T21:08:01Z\",\"finishedAt\":\"2026-11-12T21:08:01Z\",\"error\":\"$st_err\",\"logStream\":\"i-0123456789abcdef0/release.log\"}'"
python3 -I - "$T/log-events" <<'PY'
import json, sys
j = "Nov 12 21:%s ip-10-0-2-88 "
ev = ["[provision] kiro agent file: not validated (kiro-cli has no login here; expected without one)", "[provision] done"]
ev += ["curl: (7) Failed to connect to 127.0.0.1 port 80 after %d ms: Couldn't connect to server" % (i % 2) for i in range(30)]
ev += ["factoryos-deploy: backend did not become healthy"]
for i in range(10):
    t = "07:%02d" % (31 + 3 * i)
    ev += [j % t + "systemd[1]: Started factoryos-backend.service - factoryos-backend.",
           j % t + "(-backend)[%d]: factoryos-backend.service: Failed to execute /opt/factoryos/current/bin/factoryos-backend: Permission denied" % (75583 + 10 * i),
           j % t + "systemd[1]: factoryos-backend.service: Main process exited, code=exited, status=203/EXEC",
           j % t + "systemd[1]: factoryos-backend.service: Failed with result 'exit-code'."]
    if i < 9:
        ev.append(j % t + "systemd[1]: factoryos-backend.service: Scheduled restart job, restart counter is at %d." % (i + 1))
ev.append("[2026-11-12T21:08:02Z release] state: failed (deploying: …)")
json.dump(ev, open(sys.argv[1], "w"))
PY
out=$(run status); rc=$?
check "status, failed: the error's parts a line each" \
  $([ $rc = 0 ] && grep -qxF "  error: deploying: factoryos-deploy.sh exited 1: factoryos-deploy: backend did not become healthy" <<<"$out" \
     && grep -qxF "         factoryos-backend.service: Failed to execute /opt/factoryos/current/bin/factoryos-backend: Permission denied" <<<"$out" \
     && grep -qxF "         no previous release to roll back to" <<<"$out"; echo $?) "$out"
check "status, failed: names the log (group and stream)" \
  $(grep -qxF "  log:   /factoryos/factoryos i-0123456789abcdef0/release.log (CloudWatch Logs)" <<<"$out"; echo $?) "$out"
check "status, failed: the log tail keeps the deploy's own error in view (the restart loop and curl collapsed)" \
  $(grep -qxF "  │ factoryos-deploy: backend did not become healthy" <<<"$out" && grep -qxF "  │ … the line above: 30 times" <<<"$out" \
     && grep -qxF "  │ … the 5 lines above: 9 times" <<<"$out" && [ "$(grep -c 'Permission denied' <<<"$out")" = 3 ]; echo $?) "$out"
check "status, failed: the log tail is read as JSON, from the instance's stream" \
  $(grep -q -- "--region $reg logs get-log-events --log-group-name /factoryos/factoryos --log-stream-name i-0123456789abcdef0/release.log --limit 200 --no-start-from-head --query events\[\].message --output json" "$T/calls"; echo $?) "$(grep get-log-events "$T/calls")"
reset; override "$up" "state_param='{\"phase\":\"building\",\"version\":\"v0.3.0\",\"previous\":\"v0.2.0\",\"error\":null,\"logStream\":\"i-0123456789abcdef0/release.log\"}'"
out=$(run status); rc=$?
check "status, in progress: phase, version and the release serving; no error, no log line" \
  $([ $rc = 0 ] && grep -qF "Release  building  v0.3.0  (serving: v0.2.0)" <<<"$out" && ! grep -qE '^  (error|log): ' <<<"$out"; echo $?) "$out"
rm -f "$T/log-events"

# --- the edge-config collision (a second region must not take over the first one's sign-in) ---------------------------
reset; override "edge_config='{\"format\":1,\"region\":\"$other\",\"userPoolId\":\"${other}_X\"}'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "edge-config owned by another region: install refused, --stack-name suggested" $([ $rc != 0 ] && grep -q "installed in $other already" <<<"$out" && grep -q -- '--stack-name' <<<"$out" && ! grep -qE 'cloudformation|s3 cp' "$T/calls"; echo $?) "$out"
check "the edge-config is read in the edge region with get-parameter" $(grep -q -- "--region $edge ssm get-parameter --name /factoryos/factoryos/edge-config" "$T/calls"; echo $?) "$(cat "$T/calls")"
out=$(run uninstall --yes --confirm-stack factoryos); check "edge-config owned by another region: uninstall refused too" $([ $? != 0 ] && grep -q "$other" <<<"$out" && ! grep -q delete-stack "$T/calls"; echo $?) "$out"
reset; override "edge_config='{\"format\":1,\"region\":\"$reg\"}'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "edge-config of this region: install goes on" $(grep -q DEPLOY_STACKS <<<"$out"; echo $?) "$out"
reset; override "edge_config_error='An error occurred (AccessDeniedException) when calling the GetParameter operation'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "edge-config unreadable: refused (fails closed)" $([ $? != 0 ] && grep -q 'cannot read /factoryos/factoryos/edge-config' <<<"$out" && ! grep -q DEPLOY_STACKS <<<"$out"; echo $?) "$out"
reset; override "edge_config='{\"format\":1,\"region\":\"$other\"}'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --stack-name factoryos-two); check "another --stack-name: its own edge-config is checked" $(grep -q DEPLOY_STACKS <<<"$out" && grep -q 'get-parameter --name /factoryos/factoryos-two/edge-config' "$T/calls"; echo $?) "$out"

# --- a sha256 mismatch -------------------------------------------------------------------------------------------------
reset; override "sha256_answer=$(printf '0%.0s' {1..64})"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "the export's sha256 differs from <uri>.sha256: refused before any deploy" $([ $? != 0 ] && grep -q 'factoryos-v0.2.0.zip.sha256 says 0000' <<<"$out" && ! grep -q DEPLOY_STACKS <<<"$out" && ! grep -q send-command "$T/calls"; echo $?) "$out"
reset; override "sha256_answer=$(python3 -I -c 'import hashlib,sys;print(hashlib.sha256(open(sys.argv[1],"rb").read()).hexdigest())' "$T/export.zip")"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "the export's sha256 matches <uri>.sha256: accepted" $([ $? = 0 ] && grep -q 'matches factoryos-v0.2.0.zip.sha256' <<<"$out"; echo $?) "$out"

# --- the archive: one top directory, VERSION.json in source/ (the Guidance layout) -------------------------------------
mkdir -p "$T/g/factoryos-v0.5.0/source" "$T/g/factoryos-v0.5.0/deployment"
echo '{"format":1,"release":"v0.5.0","tree":"t","umarell":"u"}' > "$T/g/factoryos-v0.5.0/source/VERSION.json"
mkzip "$T/g" "$T/guidance.zip" factoryos-v0.5.0
reset; cp "$T/export.zip" "$T/export.zip.orig"; cp "$T/guidance.zip" "$T/export.zip"
out=$(run s3://exports-bkt/factoryos-v0.5.0.zip --yes); check "an export with source/VERSION.json: its release is the version" $([ $? = 0 ] && grep -q 'releases/v0.5.0.zip' "$T/calls"; echo $?) "$out"
mkdir -p "$T/g2/a" "$T/g2/b"; echo x > "$T/g2/a/f"; echo y > "$T/g2/b/f"; mkzip "$T/g2" "$T/export.zip" a b
reset
out=$(run s3://exports-bkt/factoryos-v0.5.0.zip --yes); check "an export with two top directories: refused" $([ $? != 0 ] && grep -q 'exactly one top directory' <<<"$out" && ! grep -q DEPLOY_STACKS <<<"$out"; echo $?) "$out"
mv "$T/export.zip.orig" "$T/export.zip"

# --- install --from-checkout --------------------------------------------------------------------------------------------
# A checkout in the Guidance layout: deployment/ (install.sh and the bootstrap) and source/VERSION.json.
mk_checkout() { # <dir>
  mkdir -p "$1/deployment/bootstrap" "$1/deployment/node_modules/x" "$1/source/scripts"
  echo '#!/usr/bin/env bash' > "$1/deployment/install.sh"; chmod +x "$1/deployment/install.sh"
  echo 'echo release' > "$1/deployment/bootstrap/factoryos-release.sh"
  echo 'build' > "$1/source/scripts/build-release.sh"; chmod +x "$1/source/scripts/build-release.sh"
  echo 'module' > "$1/deployment/node_modules/x/index.js"
  printf 'node_modules/\n*.log\n' > "$1/.gitignore"
}
tar_list() { tar -tzf "$1" | sort; }
co="$T/co"; mk_checkout "$co"
echo '{"format":1,"release":"v0.3.0","tree":"t","umarell":"u"}' > "$co/source/VERSION.json"
git -C "$co" init -q && git -C "$co" add -A && git -C "$co" commit -qm init

reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg); rc=$?
a="$T/work/v0.3.0/factoryos-v0.3.0.tar.gz"
check "from-checkout clean: install runs to the end" "$rc" "$out"
check "from-checkout clean: git archive of HEAD with the prefix factoryos-<version>/" $(grep -q 'archive --format=tar.gz --prefix=factoryos-v0.3.0/ -o .* HEAD' "$T/git-calls"; echo $?) "$(cat "$T/git-calls" 2>/dev/null)"
check "from-checkout clean: one top directory, committed files only" $(tar_list "$a" | grep -qx 'factoryos-v0.3.0/source/VERSION.json' && [ "$(tar_list "$a" | cut -d/ -f1 | sort -u)" = factoryos-v0.3.0 ] && ! tar_list "$a" | grep -q node_modules; echo $?) "$(tar_list "$a" 2>&1)"
check "from-checkout clean: the local archive is uploaded to releases/<version>.tar.gz" $(grep -qxF -- "--region $reg s3 cp $a s3://src-bucket/releases/v0.3.0.tar.gz --only-show-errors" "$T/calls"; echo $?) "$(grep 's3 cp' "$T/calls")"
w=$(cat "$T/wanted" 2>/dev/null)
check "from-checkout clean: wanted has the key and the archive's sha256" $(python3 -c 'import json,sys,hashlib;j=json.loads(sys.argv[1]);assert j["version"]=="v0.3.0" and j["key"]=="releases/v0.3.0.tar.gz" and j["sha256"]==hashlib.sha256(open(sys.argv[2],"rb").read()).hexdigest()' "$w" "$a" 2>/dev/null; echo $?) "$w"

rm "$co/source/VERSION.json"; git -C "$co" commit -qam 'no VERSION.json' && git -C "$co" tag v0.4.0
reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg)
check "from-checkout without VERSION.json: the version is git describe --tags" $(grep -q 'releases/v0.4.0.tar.gz' "$T/calls"; echo $?) "$out"

echo '{"format":1,"release":"v0.3.0","tree":"t","umarell":"u"}' > "$co/source/VERSION.json"; git -C "$co" add -A; git -C "$co" commit -qm v
echo 'changed' >> "$co/source/scripts/build-release.sh"; echo new > "$co/source/untracked.txt"; echo ignored > "$co/source/debug.log"
reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg); rc=$?
v=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["version"])' "$T/wanted" 2>/dev/null)
a="$T/work/$v/factoryos-$v.tar.gz"
check "from-checkout dirty: version <release>+dirty-<sha12>, warned" $([ $rc = 0 ] && [[ "$v" =~ ^v0\.3\.0\+dirty-[0-9a-f]{12}$ ]] && grep -q 'uncommitted changes' <<<"$out"; echo $?) "$v
$out"
check "from-checkout dirty: changed and untracked files as on disk; ignored files, node_modules and .git left out" $(tar_list "$a" | grep -qx "factoryos-$v/source/untracked.txt" && tar -xOzf "$a" "factoryos-$v/source/scripts/build-release.sh" | grep -q changed && ! tar_list "$a" | grep -qE 'debug\.log|node_modules|/\.git/' && [ "$(tar_list "$a" | cut -d/ -f1 | sort -u)" = "factoryos-$v" ]; echo $?) "$(tar_list "$a" 2>&1)"
check "from-checkout dirty: modes kept (build-release.sh stays executable)" $(tar -tvzf "$a" "factoryos-$v/source/scripts/build-release.sh" | grep -q '^-rwx'; echo $?) "$(tar -tvzf "$a" 2>&1 | head)"
check "from-checkout dirty: uploaded as releases/<version>.tar.gz" $(grep -q "s3 cp $a s3://src-bucket/releases/$v.tar.gz" "$T/calls"; echo $?) "$(grep 's3 cp' "$T/calls")"
v1="$v"; reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg)
v=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["version"])' "$T/wanted" 2>/dev/null)
echo more >> "$co/source/untracked.txt"; reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg)
v2=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["version"])' "$T/wanted" 2>/dev/null)
check "from-checkout dirty: the same content gives the same version, other content another" $([ "$v" = "$v1" ] && [ "$v2" != "$v1" ]; echo $?) "$v1 $v $v2"

pc="$T/plain"; mk_checkout "$pc"; mkdir -p "$pc/source/.git"; echo x > "$pc/source/.git/HEAD"
reset
out=$(RUN_INSTALLER_DIR="$pc/deployment" run install --from-checkout --yes --region $reg); rc=$?
v=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["version"])' "$T/wanted" 2>/dev/null)
a="$T/work/$v/factoryos-$v.tar.gz"
check "from-checkout of a directory that is no git checkout: dev-<sha12>, no node_modules or .git" $([ $rc = 0 ] && [[ "$v" =~ ^dev-[0-9a-f]{12}$ ]] && tar_list "$a" | grep -qx "factoryos-$v/source/scripts/build-release.sh" && ! tar_list "$a" | grep -qE 'node_modules|\.git/'; echo $?) "$v
$out"

reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout s3://exports-bkt/x.zip --yes); check "from-checkout with an export URI as well: refused" $([ $? != 0 ] && grep -q 'takes no export URI' <<<"$out"; echo $?) "$out"
out=$(run status --from-checkout); check "--from-checkout outside install: refused" $([ $? != 0 ] && grep -q 'option of install' <<<"$out"; echo $?) "$out"

# --- --umarell-tarball: a real archive with a listing larger than a pipe buffer (64 KiB), and a GNU-like tar ---------
# GNU tar (CloudShell, CodeBuild) dies of SIGPIPE when its reader stops early; bsdtar (macOS) does not. The fake tar
# behaves like GNU tar for -t (cat dies of SIGPIPE), so the check is exercised the same way on every platform.
mkdir -p "$T/um/umarell/src"; for i in $(seq 1 2500); do : > "$T/um/umarell/src/file-with-a-long-name-$i.py"; done
(cd "$T/um" && tar -czf "$T/umarell.tar.gz" umarell); mkdir -p "$T/um/bad"; : > "$T/um/bad/x"; (cd "$T/um" && tar -czf "$T/bad.tar.gz" bad)
listing=$(tar -tzf "$T/umarell.tar.gz" | wc -c | tr -d ' ')
mkdir -p "$T/gnubin"; real_tar=$(command -v tar)
printf '#!/usr/bin/env bash
if [ "$1" = -tzf ]; then %q -tzf "$2" > "$FAKE/tar-list" || exit 2; exec cat "$FAKE/tar-list"; fi
exec %q "$@"
' "$real_tar" "$real_tar" > "$T/gnubin/tar"
chmod +x "$T/gnubin/tar"
reset
out=$(PATH="$T/gnubin:$PATH" RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg --umarell-tarball s3://exports-bkt/umarell.tar.gz); rc=$?
w=$(cat "$T/wanted" 2>/dev/null)
check "--umarell-tarball with a ${listing}-byte listing and a GNU-like tar: accepted, recorded in wanted" \
  $([ $rc = 0 ] && [ "$listing" -gt 65536 ] && python3 -c 'import json,sys;u=json.loads(sys.argv[1])["umarell"];assert u["key"].startswith("vendor/umarell-") and len(u["sha256"])==64' "$w" 2>/dev/null; echo $?) "$rc $listing $out"
cp "$T/bad.tar.gz" "$T/umarell.tar.gz"; reset
out=$(PATH="$T/gnubin:$PATH" RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg --umarell-tarball s3://exports-bkt/umarell.tar.gz); rc=$?
check "--umarell-tarball without the umarell/ prefix: refused" $([ $rc != 0 ] && grep -q 'not a git archive --prefix=umarell/' <<<"$out"; echo $?) "$out"
echo 'not a tarball' > "$T/umarell.tar.gz"; reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg --umarell-tarball s3://exports-bkt/umarell.tar.gz); rc=$?
check "--umarell-tarball that is no tar archive: refused" $([ $rc != 0 ] && grep -qE 'not a gzip tar archive|not a git archive' <<<"$out"; echo $?) "$out"

# --- --umarell-jam-tarball: s3:// or a local file, checked before anything changes, uploaded, kept by an update -------
mkdir -p "$T/jm/umarell/agents"; : > "$T/jm/umarell/agents/__init__.py"
(cd "$T/jm" && tar -czf "$T/umarell-jam.tar.gz" umarell); cp "$T/umarell-jam.tar.gz" "$T/jam-local.tar.gz"
jsha=$(sha256sum < "$T/umarell-jam.tar.gz" | cut -d' ' -f1); jkey="vendor/umarell-jam-${jsha:0:12}.tar.gz"
jam_of() { python3 -c 'import json,sys;j=json.load(open(sys.argv[1])).get("umarellJam");print("null" if j is None else j["key"]+" "+j["sha256"])' "$1" 2>/dev/null; }
reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg --umarell-jam-tarball s3://exports-bkt/umarell-jam.tar.gz); rc=$?
check "--umarell-jam-tarball s3://: in wanted as vendor/umarell-jam-<sha12>.tar.gz with its sha256" \
  $([ $rc = 0 ] && [ "$(jam_of "$T/wanted")" = "$jkey $jsha" ]; echo $?) "$rc $out"$'\n'"$(cat "$T/wanted" 2>/dev/null)"
check "--umarell-jam-tarball s3://: uploaded into the source bucket's vendor/" \
  $(grep -qE -- "--region $reg s3 cp .*/umarell-jam.tar.gz s3://src-bucket/$jkey --only-show-errors" "$T/calls"; echo $?) "$(grep 's3 cp' "$T/calls")"
check "--umarell-jam-tarball s3://: recorded in the options" $([ "$(jam_of "$T/options")" = "$jkey $jsha" ]; echo $?) "$(cat "$T/options" 2>/dev/null)"
check "--umarell-jam-tarball: the umarell tarball untouched (umarell null)" \
  $(python3 -c 'import json,sys;assert json.load(open(sys.argv[1]))["umarell"] is None' "$T/wanted" 2>/dev/null; echo $?) "$(cat "$T/wanted")"
reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg --umarell-jam-tarball "$T/jam-local.tar.gz"); rc=$?
check "--umarell-jam-tarball <local file>: copied, uploaded, in wanted and the options" \
  $([ $rc = 0 ] && [ "$(jam_of "$T/wanted")" = "$jkey $jsha" ] && [ "$(jam_of "$T/options")" = "$jkey $jsha" ] \
     && grep -q "s3 cp .*/umarell-jam.tar.gz s3://src-bucket/$jkey" "$T/calls" && ! grep -q "s3 cp $T/jam-local.tar.gz" "$T/calls"; echo $?) "$rc $out"
cp "$T/bad.tar.gz" "$T/jam-bad.tar.gz"; reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg --umarell-jam-tarball "$T/jam-bad.tar.gz"); rc=$?
check "--umarell-jam-tarball without the umarell/ prefix: refused before the stacks, nothing written" \
  $([ $rc != 0 ] && grep -q 'jam-bad.tar.gz is not a git archive --prefix=umarell/' <<<"$out" && ! grep -q DEPLOY_STACKS <<<"$out" \
     && [ ! -e "$T/wanted" ] && [ ! -e "$T/options" ]; echo $?) "$out"
reset
out=$(RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg --umarell-jam-tarball "$T/no-such.tar.gz"); rc=$?
check "--umarell-jam-tarball naming no file: refused" $([ $rc != 0 ] && grep -q 'must be an s3://bucket/key.tar.gz or a local .tar.gz file' <<<"$out"; echo $?) "$out"
out=$(run status --umarell-jam-tarball s3://exports-bkt/umarell-jam.tar.gz); check "--umarell-jam-tarball outside install: refused" $([ $? != 0 ] && grep -q 'option of install' <<<"$out"; echo $?) "$out"
# An update: the recorded tarball is kept (while the bucket has it), '' drops it, a gone one is dropped with a warning.
jopts='{"instanceType":"m7g.2xlarge","volumeGb":"200","bedrock":false,"kiroSecretName":"","bedrockModelIds":"","umarellJam":{"key":"'"$jkey"'","sha256":"'"$jsha"'"}}'
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'"; printf '%s' "$jopts" > "$T/options"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "update without --umarell-jam-tarball: the installed one in wanted, checked in the bucket, not uploaded again" \
  $([ $rc = 0 ] && [ "$(jam_of "$T/wanted")" = "$jkey $jsha" ] && grep -q "s3api head-object --bucket src-bucket --key $jkey" "$T/calls" \
     && ! grep -q "s3://src-bucket/$jkey" "$T/calls" && [ "$(jam_of "$T/options")" = "$jkey $jsha" ]; echo $?) "$rc $out"
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'"; printf '%s' "$jopts" > "$T/options"; touch "$T/vendor-gone"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "update, the installed one gone from the bucket: installs without it, warned, dropped from the options" \
  $([ $rc = 0 ] && [ "$(jam_of "$T/wanted")" = null ] && grep -q 'umarell_jam incomplete' <<<"$out" && [ "$(jam_of "$T/options")" = null ]; echo $?) "$rc $out"
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'"; printf '%s' "$jopts" > "$T/options"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --umarell-jam-tarball ''); rc=$?
check "update with --umarell-jam-tarball '': dropped (wanted and options), the bucket not asked" \
  $([ $rc = 0 ] && [ "$(jam_of "$T/wanted")" = null ] && [ "$(jam_of "$T/options")" = null ] && ! grep -q head-object "$T/calls"; echo $?) "$rc $out"
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.2.0","previous":"v0.1.9"}'"'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --umarell-jam-tarball s3://exports-bkt/umarell-jam.tar.gz); rc=$?
check "the version installed already: --umarell-jam-tarball is not used, and says so" \
  $([ $rc = 0 ] && grep -q 'umarell-jam-tarball is not used' <<<"$out" && ! grep -q send-command "$T/calls"; echo $?) "$rc $out"
# status: a ready release that provisioned without umarell_jam says so, and why to look.
reset; override "$up" "state_param='{\"phase\":\"ready\",\"version\":\"v0.2.0\",\"previous\":\"v0.1.9\",\"error\":null,\"logStream\":\"i-0123456789abcdef0/release.log\",\"incomplete\":[\"umarell_jam\"]}'"
out=$(run status --wait); rc=$?
check "status: a ready release's incomplete umarell_jam, with the fallback and the tarball named; --wait still ready" \
  $([ $rc = 0 ] && grep -qF "Release  ready  v0.2.0" <<<"$out" && grep -qxF "  incomplete: umarell_jam (provisioned without it; the next update provisions it again)" <<<"$out" \
     && grep -q 'fall back to umarell_u3' <<<"$out" && grep -q -- '--umarell-jam-tarball' <<<"$out" && grep -q 'provisioning incomplete: umarell_jam' <<<"$out"; echo $?) "$out"
reset; override "$up" "state_param='{\"phase\":\"ready\",\"version\":\"v0.2.0\",\"error\":null,\"logStream\":\"i-0123456789abcdef0/release.log\"}'"
out=$(run status); check "status: nothing incomplete, no incomplete line" $([ $? = 0 ] && ! grep -q incomplete <<<"$out"; echo $?) "$out"

# --- no credential in any argv during the release step ----------------------------------------------------------------------
reset
# The AWS documentation's example key pair and a sentinel session token, no account's: in no argv below.
# pragma: allowlist nextline secret
out=$(AWS_ACCESS_KEY_ID=AKIAIOSFODNN7EXAMPLE AWS_SECRET_ACCESS_KEY=wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY AWS_SESSION_TOKEN=sentinelSessionToken0123456789 \
      RUN_INSTALLER_DIR="$co/deployment" run install --from-checkout --yes --region $reg --kiro-secret factoryos/kiro --users a@example.com); rc=$?
all=$(cat "$T/calls" "$T/git-calls" "$T/npx-calls" "$T/npm-calls" "$T/unzip-calls" 2>/dev/null)
check "release step: no credential in the argv of aws, git, npm, npx or unzip" $([ $rc = 0 ] && grep -q 'ssm send-command' "$T/calls" && ! grep -qE 'AKIAIOSFODNN7EXAMPLE|bPxRfiCYEXAMPLEKEY|sentinelSessionToken|password|secret-string|KIRO_API_KEY' <<<"$all"; echo $?) "$rc $out" # pragma: allowlist secret (the example key id above, searched for)
w=$(cat "$T/wanted" 2>/dev/null)
check "release step: the wanted parameter holds only version, key, sha256, umarell, umarellJam, time and force" $(python3 -c 'import json,sys;j=json.loads(sys.argv[1]);assert set(j)=={"version","key","sha256","umarell","umarellJam","requestedAt","force"}' "$w" 2>/dev/null; echo $?) "$w"

# --- --bedrock-models ----------------------------------------------------------------------------------------------------------
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --bedrock-models 'us.example.model-a-v1:0,eu.example.model-b-v1:0')
check "--bedrock-models: -c bedrockModelIds=… (and bedrock on) to the CDK app, recorded in the options" $(grep -q -- '-c bedrock=true .*-c bedrockModelIds=us.example.model-a-v1:0,eu.example.model-b-v1:0' "$T/calls" && grep -q '"bedrockModelIds":"us.example.model-a' "$T/options"; echo $?) "$(grep '^cdk' "$T/calls")"
reset; override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); check "without --bedrock-models: no bedrockModelIds context (the app's default)" $(grep -q '^cdk deploy' "$T/calls" && ! grep '^cdk' "$T/calls" | grep -q bedrockModelIds; echo $?) "$(grep '^cdk' "$T/calls")"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --no-bedrock --bedrock-models us.example.model-a); check "--bedrock-models with --no-bedrock: refused" $([ $? != 0 ] && grep -q contradicts <<<"$out"; echo $?) "$out"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --bedrock-models 'us.example.model-a,arn:aws:bedrock:*'); check "--bedrock-models with an ARN or a wildcard: refused" $([ $? != 0 ] && grep -q 'not an inference profile id' <<<"$out"; echo $?) "$out"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes --bedrock-models 'us.example.model-a,us.example.model-a'); check "--bedrock-models with an id twice: refused" $([ $? != 0 ] && grep -q 'twice' <<<"$out"; echo $?) "$out"

# --- uninstall --confirm-stack: without a terminal --------------------------------------------------------------------------
reset; override "$up"
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall --confirm-stack <the stack name>: runs without a terminal" $([ $rc = 0 ] && grep -q "$reg cloudformation delete-stack --stack-name factoryos\$" "$T/calls" && grep -q "$edge cloudformation delete-stack --stack-name factoryos-edge" "$T/calls" && grep -q uninstalled <<<"$out"; echo $?) "$out"
check "uninstall: the stacks' functions are listed before the stacks are deleted" \
  $(in_order "$reg cloudformation list-stack-resources --stack-name factoryos " "$reg cloudformation delete-stack --stack-name factoryos" \
     "$edge cloudformation list-stack-resources --stack-name factoryos-edge " "$edge cloudformation delete-stack --stack-name factoryos-edge"; echo $?) "$(cat "$T/calls")"
check "uninstall: deletes the provider's log group in the stack's Region" \
  $(grep -q -- "--region $reg logs delete-log-group --log-group-name /aws/lambda/factoryos-CustomS3AutoDeleteObjectsCustomRes-mNoPqRsTuVwX" "$T/calls" \
     && grep -q 'ok.*/aws/lambda/factoryos-CustomS3AutoDelete' <<<"$out"; echo $?) "$(cat "$T/calls")"
check "uninstall: deletes the Lambda@Edge replicas' log groups in every Region" \
  $(for r in us-east-1 eu-central-1 us-west-2; do grep -q -- "--region $r logs delete-log-group --log-group-name /aws/lambda/us-east-1.factoryos-edge-EdgeAuth0123ABCD-aBcDeFgHiJkL" "$T/calls" || exit 1; done; echo 0) "$(cat "$T/calls")"
check "uninstall: deletes the edge function's custom log group (the stack's EdgeAuthLogs) in every Region" \
  $(grep -q -- "--region $edge cloudformation list-stack-resources --stack-name factoryos-edge --query .*AWS::Logs::LogGroup" "$T/calls" \
     && for r in us-east-1 eu-central-1 us-west-2; do grep -q -- "--region $r logs delete-log-group --log-group-name factoryos-edge-EdgeAuthLogs0123ABCD-qRsTuVwXyZaB\$" "$T/calls" || exit 1; done; echo $?) "$(cat "$T/calls")"
check "uninstall: no log group of the main stack is deleted by name (CloudFormation deletes them)" \
  $(! grep -q -- "logs delete-log-group --log-group-name factoryos-FlowLogs" "$T/calls"; echo $?) "$(cat "$T/calls")"
check "uninstall: a log group that does not exist is no warning" $(! grep -q 'could not delete the log group' <<<"$out"; echo $?) "$out"
check "uninstall: deletes the stack's own log groups, after the stack (a log line of its delete re-creates one)" \
  $(in_order "$reg cloudformation delete-stack --stack-name factoryos" \
     "$reg logs delete-log-group --log-group-name factoryos-CustomResourceLogsE45D40F0-aAbBcCdDeEfF"; echo $?) "$(cat "$T/calls")"
check "uninstall: reads those groups from the stack, never by a name prefix (which matches another installation's)" \
  $(grep -qF "ResourceType=='AWS::Logs::LogGroup'" "$T/calls" && ! grep -q 'logs describe-log-groups' "$T/calls"; echo $?) "$(cat "$T/calls")"
# A repetition count above 255 is an error in BSD grep ("maximum repetition exceeds 255"), not a long match: the
# pipeline then yields nothing at all. It cost the sweep below its whole list once.
check "no grep -E repetition above 255 (BSD grep refuses it)" \
  $(! grep -nE '\{[0-9]+,([3-9][0-9][0-9]|[0-9]{4,})\}' "$dep"/installer/*.sh "$dep"/*.sh; echo $?) \
  "$(grep -nE '\{[0-9]+,([3-9][0-9][0-9]|[0-9]{4,})\}' "$dep"/installer/*.sh "$dep"/*.sh)"
check "uninstall: the wait for the late delivery defaults to 30 s" \
  $(grep -qF 'sleep "${FACTORYOS_LOG_LAG_SECONDS:-30}"' "$dep/installer/uninstall.sh"; echo $?)
reset; override "$up"
out=$(run uninstall --region $reg --confirm-stack factoryos --edge-only); rc=$?
check "uninstall --edge-only: only the edge stack and its log groups" \
  $([ $rc = 0 ] && ! grep -q "$reg cloudformation delete-stack --stack-name factoryos\$" "$T/calls" && ! grep -q CustomS3AutoDelete "$T/calls" \
     && grep -q "logs delete-log-group --log-group-name /aws/lambda/us-east-1.factoryos-edge-EdgeAuth" "$T/calls"; echo $?) "$out"$'\n'"$(cat "$T/calls")"
# An uninstall cut short after the main stack was deleted (a closed CloudShell, a reboot) leaves the stack's
# parameters; running it again removes them (test install fxt-jam, 2026-11-16).
reset; printf '%s\n' /factoryos/factoryos/release/wanted /factoryos/factoryos/install-options > "$T/params-left"
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall with both stacks gone but parameters left: lists and deletes them" \
  $([ $rc = 0 ] && grep -q 'parameters an earlier uninstall left: /factoryos/factoryos/release/wanted /factoryos/factoryos/install-options' <<<"$out" \
     && grep -q -- "--region $reg ssm delete-parameter --name /factoryos/factoryos/release/wanted" "$T/calls" \
     && grep -q -- "--region $reg ssm delete-parameter --name /factoryos/factoryos/install-options" "$T/calls" \
     && ! grep -q 'Not installed' <<<"$out" && ! grep -q 'delete-stack' "$T/calls"; echo $?) "$out"$'\n'"$(cat "$T/calls")"
rm -f "$T/params-left"; reset
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall with nothing left: 'Not installed', nothing deleted" \
  $([ $rc = 0 ] && grep -q 'Not installed' <<<"$out" && ! grep -q 'delete-parameter' "$T/calls"; echo $?) "$out"
reset; override "$up"
out=$(run uninstall --region $reg --confirm-stack factoryos-other); rc=$?
check "uninstall --confirm-stack <another name>: refused before any AWS call" $([ $rc != 0 ] && grep -q 'is not the stack name' <<<"$out" && [ ! -s "$T/calls" ]; echo $?) "$out"
reset; override "$up"
out=$(run uninstall --region $reg --yes); rc=$?
check "uninstall without --confirm-stack and without a terminal: refused as before (--yes is not enough)" $([ $rc != 0 ] && grep -q 'needs a terminal' <<<"$out" && ! grep -q delete-stack "$T/calls"; echo $?) "$out"
out=$(run status --confirm-stack factoryos); check "--confirm-stack outside uninstall: refused" $([ $? != 0 ] && grep -q 'option of uninstall' <<<"$out"; echo $?) "$out"

# --- uninstall: a bucket of the stack DELETE_FAILED, not empty (test install 2611122014-dc89) -------------------------
# The fake, on top of the default handler: the main stack's first $fail_waits deletes end DELETE_FAILED on the
# buckets $failed_buckets; a bucket's objects (versions and delete markers) are counted in $FAKE/objs-<bucket>
# (no file: none); delete-objects reads its file://, records "<bucket> <n>" in $FAKE/deleted and removes them.
lb=factoryos-accesslogbucketda470295-si8diff1fwst
fail_handler() {
  base_handler
  cat <<'EOF'
fail_waits=1; failed_buckets=""; bucket_tag=factoryos; dist_left=""; policy_now='{"Version":"2012-10-17","Statement":[{"Sid":"Ssl","Effect":"Deny","Principal":"*","Action":"s3:*","Resource":"*","Condition":{"Bool":{"aws:SecureTransport":"false"}}}]}'
delete_errors=""; list_error=""
[ -f "$FAKE/fail-overrides.sh" ] && . "$FAKE/fail-overrides.sh"
eval "$(declare -f handle | sed '1s/^handle/base_handle/')"
handle() {
  local bucket="" name="" q="" del=""; local a=("$@") i
  for ((i = 0; i < ${#a[@]}; i++)); do case "${a[$i]}" in --bucket) bucket="${a[$((i+1))]}" ;; --stack-name) name="${a[$((i+1))]}" ;;
    --query) q="${a[$((i+1))]}" ;; --delete) del="${a[$((i+1))]}" ;; esac; done
  case "$1 $2" in
    "cloudformation wait")
      [[ "$name" == *-edge ]] && return 0
      local n; n=$(( $(cat "$FAKE/waits" 2>/dev/null || echo 0) + 1 )); echo $n > "$FAKE/waits"
      [ "$n" -gt "$fail_waits" ] || { echo 'Waiter StackDeleteComplete failed: "DELETE_FAILED"' >&2; exit 255; } ;;
    "cloudformation list-stack-resources")
      case "$q" in
        *AWS::S3::Bucket*DELETE_FAILED*) [ -z "$failed_buckets" ] || printf '%s\n' "$failed_buckets" | tr ' ' '\t' ;;
        *AWS::CloudFront::Distribution*) [ -z "$dist_left" ] || echo "$dist_left" ;;
        *) base_handle "$@" ;;
      esac ;;
    "s3api get-bucket-tagging") echo "$bucket_tag" ;;
    "s3api get-bucket-policy") echo "$policy_now" ;;
    "s3api put-bucket-policy") printf '%s %s\n' "$bucket" "${a[$(( ${#a[@]} - 1 ))]}" >> "$FAKE/put-policy" ;;
    "s3api list-object-versions")
      [ -z "$list_error" ] || { echo "$list_error" >&2; exit 254; }
      local left; left=$(cat "$FAKE/objs-$bucket" 2>/dev/null || echo 0)
      python3 -I -c 'import json,sys
n=min(int(sys.argv[1]),1000); v=[{"Key":"cloudfront/E1.%d.gz" % i,"VersionId":"v%d" % i} for i in range(n)]
print(json.dumps({"Versions":v[:n//2],"DeleteMarkers":v[n//2:]} if n else {}))' "$left" ;;
    "s3api delete-objects")
      [[ "$del" == file://* ]] || { echo "delete-objects without a file://" >&2; exit 2; }
      [ -z "$delete_errors" ] || { echo "$delete_errors"; return 0; }
      local k; k=$(python3 -I -c 'import json,sys;print(len(json.load(open(sys.argv[1]))["Objects"]))' "${del#file://}")
      echo "$bucket $k" >> "$FAKE/deleted"
      local left; left=$(cat "$FAKE/objs-$bucket" 2>/dev/null || echo 0); echo $((left - k)) > "$FAKE/objs-$bucket" ;;
    *) base_handle "$@" ;;
  esac
}
EOF
}
fail_reset() { reset; override "$up"; rm -f "$T"/objs-* "$T/deleted" "$T/put-policy" "$T/waits" "$T/fail-overrides.sh"
  printf '%s\n' "$@" > "$T/fail-overrides.sh"; fail_handler > "$T/handler.sh"; }
# s3api_buckets: the buckets of every s3api call, one per line, sorted, unique.
s3api_buckets() { grep ' s3api ' "$T/calls" | sed -E 's/.*--bucket ([^ ]+).*/\1/' | sort -u; }

fail_reset "failed_buckets=$lb"; echo 2500 > "$T/objs-$lb"
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall, the access-log bucket DELETE_FAILED: emptied, the stack deleted again, uninstalled" \
  $([ $rc = 0 ] && [ "$(grep -c "$reg cloudformation delete-stack --stack-name factoryos\$" "$T/calls")" = 2 ] && grep -q uninstalled <<<"$out" \
     && [ "$(cat "$T/objs-$lb")" = 0 ]; echo $?) "$out"$'\n'"$(cat "$T/calls")"
check "uninstall, bucket DELETE_FAILED: 1000 versions and delete markers a batch, from a file" \
  $([ "$(grep -c "^$lb " "$T/deleted")" = 3 ] && grep -q "^$lb 1000\$" "$T/deleted" && grep -q "^$lb 500\$" "$T/deleted" \
     && ! grep "s3api delete-objects" "$T/calls" | grep -qv 'file://'; echo $?) "$(cat "$T/deleted" 2>/dev/null)"
check "uninstall, bucket DELETE_FAILED: writes denied first, the bucket's own policy kept, then emptied, then deleted again" \
  $(in_order "delete-stack --stack-name factoryos" "s3api get-bucket-tagging --bucket $lb" "s3api put-bucket-policy --bucket $lb" \
       "s3api list-object-versions --bucket $lb" \
     && [ "$(grep -n "delete-stack --stack-name factoryos\$" "$T/calls" | tail -1 | cut -d: -f1)" -gt "$(grep -n "s3api delete-objects --bucket $lb" "$T/calls" | tail -1 | cut -d: -f1)" ] \
     && python3 -I -c 'import json,sys
b,p=open(sys.argv[1]).read().split(" ",1); p=json.loads(p); s=p["Statement"]
assert b==sys.argv[2] and [x.get("Sid") for x in s]==["Ssl","FactoryosUninstallDenyWrites"], s
assert s[1]["Effect"]=="Deny" and s[1]["Action"]=="s3:PutObject" and s[1]["Resource"]=="arn:aws:s3:::%s/*" % b' "$T/put-policy" "$lb"; echo $?) \
  "$(cat "$T/calls")"$'\n'"$(cat "$T/put-policy" 2>/dev/null)"
check "uninstall, bucket DELETE_FAILED: no s3api call on a bucket other than the source bucket and the failed one" \
  $([ "$(s3api_buckets | tr '\n' ' ')" = "$(printf '%s\n' "$lb" src-bucket | sort | tr '\n' ' ')" ]; echo $?) "$(s3api_buckets)"

fail_reset "failed_buckets=$lb" "bucket_tag=someone-else"; echo 10 > "$T/objs-$lb"
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall, the failed bucket tagged by another stack: not touched, no second delete, fails" \
  $([ $rc != 0 ] && grep -q "not tagged aws:cloudformation:stack-name=factoryos" <<<"$out" && [ "$(cat "$T/objs-$lb")" = 10 ] \
     && ! grep -q -e put-bucket-policy -e "list-object-versions --bucket $lb" "$T/calls" \
     && [ "$(grep -c "delete-stack --stack-name factoryos\$" "$T/calls")" = 1 ]; echo $?) "$out"$'\n'"$(cat "$T/calls")"

fail_reset "failed_buckets=$lb" "fail_waits=2"; echo 10 > "$T/objs-$lb"
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall, the second delete fails too: retried once only, fails with the console hint" \
  $([ $rc != 0 ] && [ "$(grep -c "delete-stack --stack-name factoryos\$" "$T/calls")" = 2 ] && grep -q 'did not delete; see its events' <<<"$out" \
     && ! grep -q 'delete-stack --stack-name factoryos-edge' "$T/calls"; echo $?) "$out"

fail_reset "failed_buckets="
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall, DELETE_FAILED not on a bucket: no bucket emptied, no second delete, fails" \
  $([ $rc != 0 ] && [ "$(grep -c "delete-stack --stack-name factoryos\$" "$T/calls")" = 1 ] && [ "$(s3api_buckets)" = src-bucket ]; echo $?) "$out"

fail_reset "failed_buckets=$lb" "dist_left=E2EXAMPLE"; echo 10 > "$T/objs-$lb"
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall, the distribution not deleted: the bucket it logs to is not emptied, fails" \
  $([ $rc != 0 ] && grep -q 'distribution E2EXAMPLE is not deleted' <<<"$out" && [ "$(cat "$T/objs-$lb")" = 10 ] \
     && [ "$(grep -c "delete-stack --stack-name factoryos\$" "$T/calls")" = 1 ]; echo $?) "$out"

fail_reset "failed_buckets=$lb" "delete_errors='{\"Errors\":[{\"Key\":\"cloudfront/E1.0.gz\",\"Code\":\"AccessDenied\"}]}'"; echo 10 > "$T/objs-$lb"
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall, a delete-objects with errors: no second delete, fails" \
  $([ $rc != 0 ] && grep -q 'could not delete from s3://' <<<"$out" && [ "$(grep -c "delete-stack --stack-name factoryos\$" "$T/calls")" = 1 ]; echo $?) "$out"

fail_reset "list_error='An error occurred (AccessDenied) when calling the ListObjectVersions operation'"
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall, the source bucket cannot be listed: nothing deleted (was: 'is empty')" \
  $([ $rc != 0 ] && grep -q 'could not be emptied: nothing deleted' <<<"$out" && ! grep -q 'is empty' <<<"$out" && ! grep -q delete-stack "$T/calls"; echo $?) "$out"

# --- the origin secret ------------------------------------------------------------------------------------------------------
# hist_field <line> <field>: a field of secure-history (1 region, 2 name, 3 type, 4 overwrite, 5 value).
hist() { sed -n "${1}p" "$T/secure-history" 2>/dev/null | cut -d' ' -f"$2"; }
# Coreutils only (verify-build runs this suite on bare amazonlinux:2023: no cmp, diff, find or xargs).
# same_content <a> <b>: the two files have the same bytes.
same_content() { [ -f "$1" ] && [ -f "$2" ] && [ "$(sha256sum < "$1")" = "$(sha256sum < "$2")" ]; }
# request_files_left: 0 when one of the request files secure-files names still exists.
request_files_left() { local f; while read -r f; do [ ! -e "$f" ] || return 0; done < <(awk '{print $3}' "$T/secure-files"); return 1; }
out=$(grep -nE '(^|[^A-Za-z_-])(find|xargs|cmp|diff)( |$)' "$0" | grep -vE '^[0-9]+:\s*#|grep -nE|^[0-9]+:check "this suite uses no' || true)
check "this suite uses no find, xargs, cmp or diff (verify-build's amazonlinux:2023 has none; v0.1.24's export failed on cmp)" $([ -z "$out" ]; echo $?) "$out"
no_secret_shown() { # no_secret_shown <output>: no value of secure-history in argv, git/npm calls or the output
  local v; for v in $(cut -d' ' -f5 "$T/secure-history" 2>/dev/null | tr ',' ' '); do
    grep -qF -- "$v" "$T/calls" && return 1; grep -qF -- "$v" <<<"$1" && return 1
  done; return 0; }
reset
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "install: a new origin secret into both SecureStrings, the edge's (us-east-1) first, overwriting" \
  $([ $rc = 0 ] && [ "$(wc -l < "$T/secure-history" | tr -d ' ')" = 2 ] \
     && [ "$(hist 1 1) $(hist 1 2) $(hist 1 3) $(hist 1 4)" = "$edge /factoryos/factoryos/edge-origin-secret SecureString True" ] \
     && [ "$(hist 2 1) $(hist 2 2) $(hist 2 3) $(hist 2 4)" = "$reg /factoryos/factoryos/origin-secret SecureString True" ]; echo $?) "$out"$'\n'"$(cat "$T/secure-history" 2>/dev/null)"
check "install: the same value in both, 48 random bytes in base64url (64 characters)" \
  $([ "$(hist 1 5)" = "$(hist 2 5)" ] && [[ "$(hist 1 5)" =~ ^[A-Za-z0-9_-]{64}$ ]]; echo $?) "$(cat "$T/secure-history")"
check "install: before the stacks are deployed" $(python3 -c 'import sys;o=sys.argv[1];a=o.find("origin secret: a new one");b=o.find("DEPLOY_STACKS");sys.exit(0 if 0<=a<b else 1)' "$out"; echo $?) "$out"
check "install: the value in no argv and no output" $(no_secret_shown "$out"; echo $?) "$out"
check "install: put-parameter --cli-input-json from 0600 files in a 0700 directory, deleted after" \
  $(awk '{print $1, $2}' "$T/secure-files" | sort -u | grep -qx '600 700' && [ "$(awk '{print $1, $2}' "$T/secure-files" | sort -u | wc -l | tr -d ' ')" = 1 ] \
     && ! request_files_left \
     && [ "$(grep -c 'ssm put-parameter --cli-input-json file://' "$T/calls")" = 2 ]; echo $?) "$(cat "$T/secure-files")"
first=$(hist 1 5)
# An update: both exist, kept.
override "$up" 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'"
rm -f "$T/secure-history" "$T/calls"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "update: the origin secret kept (read for its type only, never its value)" \
  $([ $rc = 0 ] && [ ! -s "$T/secure-history" ] && grep -q 'origin secret: kept' <<<"$out" && ! grep -q 'with-decryption' "$T/calls" \
     && grep -q 'ssm send-command' "$T/calls"; echo $?) "$out"
# An update of an installation from before the origin secret (neither exists): both anew, before the deployment.
rm -rf "$T/secure" "$T/calls"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "update of an installation from before the origin secret: a new value into both" \
  $([ $rc = 0 ] && [ "$(wc -l < "$T/secure-history" | tr -d ' ')" = 2 ] && [ "$(hist 1 5)" = "$(hist 2 5)" ] && [ "$(hist 1 5)" != "$first" ]; echo $?) "$(cat "$T/secure-history")"
second=$(hist 1 5)
# The instance's copy alone (its name is this Region's: this installation's): both anew.
rm -f "$T/secure/${edge}_factoryos_factoryos_edge-origin-secret" "$T/secure-history" "$T/calls"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "update, the edge copy missing: a new value into both" \
  $([ $rc = 0 ] && [ "$(wc -l < "$T/secure-history" | tr -d ' ')" = 2 ] && [ "$(hist 1 5)" = "$(hist 2 5)" ] && [ "$(hist 1 5)" != "$second" ]; echo $?) "$(cat "$T/secure-history")"
# Unreadable (not missing): stop, nothing replaced.
echo "An error occurred (AccessDeniedException) when calling the GetParameter operation" > "$T/secure-error"
rm -f "$T/secure-history" "$T/calls"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "an unreadable origin secret parameter: stops, nothing written, nothing deployed or released" \
  $([ $rc != 0 ] && grep -q 'cannot read the parameter /factoryos/factoryos/origin-secret' <<<"$out" && [ ! -s "$T/secure-history" ] \
     && ! grep -q 'send-command' "$T/calls"; echo $?) "$out"
rm -f "$T/secure-error"
# The edge copy alone (another Region's installation of the name, or a leftover): refused, never overwritten.
rf="$T/secure/${reg}_factoryos_factoryos_origin-secret"; mv "$rf" "$rf.keep"
rm -f "$T/secure-history" "$T/calls"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "the edge copy alone: refused (another installation or a leftover), nothing written, the delete command named" \
  $([ $rc != 0 ] && grep -q 'exists in us-east-1 without' <<<"$out" && grep -q 'aws ssm delete-parameter --region us-east-1 --name /factoryos/factoryos/edge-origin-secret' <<<"$out" \
     && [ ! -s "$T/secure-history" ] && ! grep -q 'send-command' "$T/calls"; echo $?) "$out"
mv "$rf.keep" "$rf"
# --rotate-origin-secret with the version installed already: new,old -> refresh -> edge := new -> wait -> new -> refresh.
override 'state_param='"'"'{"phase":"ready","version":"v0.2.0","previous":"v0.1.9"}'"'"
printf '%s' '{"version":"v0.2.0","key":"releases/v0.2.0.zip","sha256":"'"$(printf '%064d' 0)"'","umarell":null,"requestedAt":"2026-11-01T00:00:00Z","force":false}' > "$T/wanted"
cp "$T/wanted" "$T/wanted.before"
old=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["Value"])' "$T/secure/${edge}_factoryos_factoryos_edge-origin-secret")
rm -f "$T/secure-history" "$T/calls"
out=$(FACTORYOS_ORIGIN_EDGE_WAIT=33 run s3://exports-bkt/factoryos-v0.2.0.zip --yes --rotate-origin-secret); rc=$?
new=$(hist 2 5)
check "rotate: three writes: the instance's new,old, the edge's new, the instance's new" \
  $([ $rc = 0 ] && [ "$(wc -l < "$T/secure-history" | tr -d ' ')" = 3 ] && [[ "$new" =~ ^[A-Za-z0-9_-]{64}$ ]] && [ "$new" != "$old" ] \
     && [ "$(hist 1 1) $(hist 1 2) $(hist 1 5)" = "$reg /factoryos/factoryos/origin-secret $new,$old" ] \
     && [ "$(hist 2 1) $(hist 2 2)" = "$edge /factoryos/factoryos/edge-origin-secret" ] \
     && [ "$(hist 3 1) $(hist 3 2) $(hist 3 5)" = "$reg /factoryos/factoryos/origin-secret $new" ]; echo $?) "$out"$'\n'"$(cat "$T/secure-history")"
check "rotate: the old value read with decryption in the edge Region (into python, not argv)" \
  $(grep -q -- "--region $edge ssm get-parameter --name /factoryos/factoryos/edge-origin-secret --with-decryption --query Parameter.Value --output text" "$T/calls"; echo $?) "$(cat "$T/calls")"
check "rotate: refresh (secretsOnly), the edge, the wait, then the new value alone and a refresh" \
  $(python3 -I - "$T/calls" "$reg" "$edge" <<'PY'
import sys
lines, reg, edge = open(sys.argv[1]).read().splitlines(), sys.argv[2], sys.argv[3]
steps = [f"--region {reg} ssm put-parameter --cli-input-json file://", 'secretsOnly":true', "ssm send-command --document-name factoryos-Release",
         f"--region {edge} ssm put-parameter --cli-input-json file://", "SLEEP 33", f"--region {reg} ssm put-parameter --cli-input-json file://",
         'secretsOnly":true', "ssm send-command --document-name factoryos-Release"]
files = ["both.json", None, None, "edge.json", None, "next.json", None, None]
i = 0
for step, f in zip(steps, files):
    while i < len(lines) and not (step in lines[i] and (f is None or lines[i].endswith(f))):
        i += 1
    if i == len(lines):
        sys.exit(1)
    i += 1
PY
echo $?) "$(cat "$T/calls")"
check "rotate: two secretsOnly refreshes, no release (the version is installed)" \
  $([ "$(grep -c 'ssm send-command' "$T/calls")" = 2 ] && [ "$(grep -c 'comment factoryos origin secret' "$T/calls")" = 2 ]; echo $?) "$(grep send-command "$T/calls")"
check "rotate: the wanted release put back as it was" $(same_content "$T/wanted" "$T/wanted.before"; echo $?) "$(cat "$T/wanted")"
check "rotate: no value in argv or output" $(no_secret_shown "$out"; echo $?) "$out"
check "rotate: the request files deleted" $(! request_files_left; echo $?) "$(cat "$T/secure-files")"
# A rotation that stops in step 2 (here: the edge's write fails, as a Ctrl-C or a lost session during the wait would
# stop it): the instance's parameter keeps "<new>,<old>", the trap says so and how to finish it.
override 'edge_put_fail=1'; rm -f "$T/secure-history" "$T/calls"
out=$(FACTORYOS_ORIGIN_EDGE_WAIT=33 run s3://exports-bkt/factoryos-v0.2.0.zip --yes --rotate-origin-secret); rc=$?
check "rotate stopped in step 2: says the old value is still accepted and to run the rotation again; wanted put back" \
  $([ $rc != 0 ] && grep -q 'could not write the origin secret in us-east-1' <<<"$out" \
     && grep -q 'the rotation did not finish: the old origin secret is still accepted; run install.sh install --rotate-origin-secret again' <<<"$out" \
     && grep -q 'during a rotation' "$T/secure/${reg}_factoryos_factoryos_origin-secret" && same_content "$T/wanted" "$T/wanted.before"; echo $?) "$out"
check "rotate stopped: the request files deleted, no value in argv or output" \
  $(! request_files_left && no_secret_shown "$out"; echo $?) "$out"
override 'edge_put_fail=""'
# The next runs find the pair by its Description, without decrypting it.
override 'state_param='"'"'{"phase":"ready","version":"v0.1.9"}'"'"; rm -f "$T/secure-history" "$T/calls"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "after a rotation that did not finish: an update without --rotate-origin-secret refuses, nothing deployed or released" \
  $([ $rc != 0 ] && grep -q 'holds the two values of a rotation that did not finish: the old origin secret is still accepted' <<<"$out" \
     && grep -q -- "--region $reg ssm describe-parameters --parameter-filters Key=Name,Values=/factoryos/factoryos/origin-secret" "$T/calls" \
     && ! grep -q DEPLOY_STACKS <<<"$out" && ! grep -q send-command "$T/calls" && ! grep -q with-decryption "$T/calls" \
     && [ ! -s "$T/secure-history" ]; echo $?) "$out"
override 'state_param='"'"'{"phase":"ready","version":"v0.2.0","previous":"v0.1.9"}'"'"; rm -f "$T/calls"
out=$(run s3://exports-bkt/factoryos-v0.2.0.zip --yes); rc=$?
check "after a rotation that did not finish: the version installed already warns, changes nothing" \
  $([ $rc = 0 ] && grep -q 'WARN.*holds the two values of a rotation that did not finish' <<<"$out" && ! grep -q send-command "$T/calls"; echo $?) "$out"
rm -f "$T/secure-history" "$T/calls"
out=$(FACTORYOS_ORIGIN_EDGE_WAIT=33 run s3://exports-bkt/factoryos-v0.2.0.zip --yes --rotate-origin-secret); rc=$?
check "after a rotation that did not finish: --rotate-origin-secret again finishes it (one value, no pair left)" \
  $([ $rc = 0 ] && [ "$(wc -l < "$T/secure-history" | tr -d ' ')" = 3 ] && [ "$(hist 2 5)" = "$(hist 3 5)" ] \
     && ! grep -q 'during a rotation' "$T/secure/${reg}_factoryos_factoryos_origin-secret" && grep -q 'restarts twice' <<<"$out" \
     && ! grep -q 'still accepted' <<<"$out" && same_content "$T/wanted" "$T/wanted.before"; echo $?) "$out"
# The refresh: the release command fails (it no longer stops the installer itself): wanted put back, then the stop.
override 'send_command_fail=1'; rm -f "$T/calls"
out=$(FACTORYOS_ORIGIN_EDGE_WAIT=33 run s3://exports-bkt/factoryos-v0.2.0.zip --yes --rotate-origin-secret); rc=$?
check "rotate, the refresh's send-command fails: wanted put back, the stop names the rerun once" \
  $([ $rc != 0 ] && grep -q 'did not refresh the origin secret (the new and the old value accepted: ssm send-command failed); the old origin secret is still accepted' <<<"$out" \
     && [ "$(grep -c 'still accepted' <<<"$out")" = 1 ] && same_content "$T/wanted" "$T/wanted.before" \
     && in_order 'secretsOnly":true' 'ssm send-command' "put-parameter --name /factoryos/factoryos/release/wanted --type String --overwrite --value $(cat "$T/wanted.before")"; echo $?) "$out"$'\n'"$(cat "$T/calls")"
override 'send_command_fail=""' 'instance_offline=1'; rm -f "$T/calls"
out=$(FACTORYOS_ORIGIN_EDGE_WAIT=33 run s3://exports-bkt/factoryos-v0.2.0.zip --yes --rotate-origin-secret); rc=$?
check "rotate, the instance not ready: no secretsOnly request written, the rerun named" \
  $([ $rc != 0 ] && grep -q 'is not ready after' <<<"$out" && ! grep -q 'secretsOnly' "$T/calls" && same_content "$T/wanted" "$T/wanted.before" \
     && grep -q 'the rotation did not finish: the old origin secret is still accepted' <<<"$out"; echo $?) "$out"
override 'instance_offline=""'
# A wanted release left as a secretsOnly request (an installer from before the trap): put back without it.
printf '%s' "$(cat "$T/wanted.before" | sed 's/}$/,"secretsOnly":true}/')" > "$T/wanted"
out=$(FACTORYOS_ORIGIN_EDGE_WAIT=33 run s3://exports-bkt/factoryos-v0.2.0.zip --yes --rotate-origin-secret); rc=$?
check "rotate over a wanted left as a secretsOnly request: put back as the release request it was" \
  $([ $rc = 0 ] && same_content "$T/wanted" "$T/wanted.before"; echo $?) "$out"$'\n'"$(cat "$T/wanted")"
out=$(run status --rotate-origin-secret); check "--rotate-origin-secret outside install: refused" $([ $? != 0 ] && grep -q 'option of install' <<<"$out"; echo $?) "$out"
# uninstall deletes both.
reset; override "$up"
out=$(run uninstall --region $reg --confirm-stack factoryos); rc=$?
check "uninstall: deletes both origin secret parameters" \
  $([ $rc = 0 ] && grep -q -- "--region $reg ssm delete-parameter --name /factoryos/factoryos/origin-secret\$" "$T/calls" \
     && grep -q -- "--region $edge ssm delete-parameter --name /factoryos/factoryos/edge-origin-secret\$" "$T/calls"; echo $?) "$(cat "$T/calls")"

# --- the usage documents the new options -----------------------------------------------------------------------------------
out=$(run --help); check "--help documents --from-checkout, --confirm-stack, --bedrock-models, --stack-name, --rotate-origin-secret and --umarell-jam-tarball" $(grep -q -- '--from-checkout' <<<"$out" && grep -q -- '--confirm-stack' <<<"$out" && grep -q -- '--bedrock-models' <<<"$out" && grep -q -- '--stack-name' <<<"$out" && grep -q -- '--rotate-origin-secret' <<<"$out" && grep -q -- '--umarell-jam-tarball' <<<"$out"; echo $?) "$out"

echo; echo "$pass passed, $fail failed"
[ "$fail" = 0 ]
