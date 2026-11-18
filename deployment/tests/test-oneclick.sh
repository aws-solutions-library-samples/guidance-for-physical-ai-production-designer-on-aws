#!/usr/bin/env bash
# Tests of the one-click scripts deployment/deploy.sh and deployment/cleanup.sh (Automated Sample Code Validation),
# with fake aws, git, npm, npx, node, curl and sleep on PATH; the fake git clone holds a fake
# deployment/install.sh that logs its arguments and plays the stacks, and a umarell.pin. Most cases run standalone
# copies of the scripts (they clone REPO_URL); the in-tree case runs them as deployment/*.sh of a tree with the
# installer (they use that tree, no clone). No AWS calls, no network.
#   bash deployment/tests/test-oneclick.sh </dev/null
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
dep="$(cd "$here/.." && pwd)"
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
pass=0 fail=0
check() { if [ "$2" = 0 ]; then pass=$((pass+1)); echo "ok    $1"; else fail=$((fail+1)); echo "FAIL  $1"; [ -z "${3:-}" ] || printf '%s\n' "$3" | sed 's/^/      /'; fi; }
# The one-click region (the scripts pin it; the edge stack is always there too).
export FAKE="$T" FAKE_R="us-east-1"
mkdir -p "$T/bin"

# --- fakes ------------------------------------------------------------------------------------------------------
# Stacks are files $FAKE/stacks/<region>/<name> holding the status. Every call goes to $FAKE/calls.
cat > "$T/bin/aws" <<'EOF'
#!/usr/bin/env bash
printf 'aws %s\n' "$*" >> "$FAKE/calls"
[ "${1:-}" = --version ] && { echo "${FAKE_AWS_VERSION:-aws-cli/2.17.0 Python/3.11.9 Linux/6.1 exe/x86_64.amzn.2023}"; exit 0; }
name="" region="" query=""; args=()
while [ $# -gt 0 ]; do
  case "$1" in
    --stack-name) name="$2"; shift 2 ;; --region) region="$2"; shift 2 ;; --query) query="$2"; shift 2 ;;
    --output) shift 2 ;; *) args+=("$1"); shift ;;
  esac
done
case "${args[*]}" in
  "sts get-caller-identity") echo 111122223333 ;;
  "cloudformation describe-stacks")
    f="$FAKE/stacks/${region:-$AWS_REGION}/$name"
    [ -f "$f" ] || { echo "An error occurred (ValidationError) when calling the DescribeStacks operation: Stack with id $name does not exist" >&2; exit 254; }
    case "$query" in
      *StackStatus*) cat "$f" ;;
      *"'GateVerified'"*) cat "$FAKE/gate" 2>/dev/null || echo true ;;
      *"'Url'"*) echo "https://d111111abcdef8.cloudfront.net/" ;;
      *) echo "fake aws: unhandled query $query" >&2; exit 2 ;;
    esac ;;
  *) echo "fake aws: unhandled: ${args[*]}" >&2; exit 2 ;;
esac
EOF
cat > "$T/bin/git" <<'EOF'
#!/usr/bin/env bash
printf 'git %s\n' "$*" >> "$FAKE/calls"
while [ "${1:-}" = -c ]; do shift 2; done
case "$1" in
  --version) echo "git version 2.47.1" ;;
  clone) dir="${!#}"; mkdir -p "$dir/deployment" "$dir/source/deploy" && cp "$FAKE/install.sh" "$dir/deployment/install.sh" \
           && cp "$FAKE/umarell.pin" "$dir/source/deploy/umarell.pin" \
           && { [ ! -f "$FAKE/umarell-jam.pin" ] || cp "$FAKE/umarell-jam.pin" "$dir/source/deploy/umarell-jam.pin"; } ;;
  rev-parse) echo abc1234 ;;
  # FAKE_UMARELL_PRIVATE=1: the umarell repository asks for credentials (it is not public).
  ls-remote) [ "${FAKE_UMARELL_PRIVATE:-0}" = 1 ] && { echo "fatal: could not read Username" >&2; exit 128; }; echo "08d5 HEAD" ;;
  *) echo "fake git: unhandled: $*" >&2; exit 2 ;;
esac
EOF
cat > "$T/bin/node" <<'EOF'
#!/usr/bin/env bash
[ "$1" = --version ] && echo "v${FAKE_NODE:-22.11.0}"
EOF
cat > "$T/bin/npm" <<'EOF'
#!/usr/bin/env bash
printf 'npm %s (in %s)\n' "$*" "$(basename "$PWD")" >> "$FAKE/calls"
EOF
cat > "$T/bin/npx" <<'EOF'
#!/usr/bin/env bash
printf 'npx %s (in %s)\n' "$*" "$(basename "$PWD")" >> "$FAKE/calls"
EOF
# The gate: 401 + WWW-Authenticate on the API, 302 to the hosted sign-in for a page; FAKE_OPEN=1 serves 200.
cat > "$T/bin/curl" <<'EOF'
#!/usr/bin/env bash
printf 'curl %s\n' "$*" >> "$FAKE/calls"
url="${!#}"
if [ "${FAKE_OPEN:-0}" = 1 ]; then printf 'HTTP/2 200\r\ncontent-type: text/html\r\n\r\n'; exit 0; fi
case "$url" in
  */api/v1/health) printf 'HTTP/2 401\r\nwww-authenticate: Bearer realm="factoryos"\r\n\r\n' ;;
  */) printf 'HTTP/2 302\r\nlocation: https://sign-in.example.com/oauth2/authorize?client_id=x\r\n\r\n' ;;
esac
EOF
cat > "$T/bin/sleep" <<'EOF'
#!/usr/bin/env bash
printf 'sleep %s\n' "$*" >> "$FAKE/calls"
EOF
printf '# <commit> <url>\n08d5a390a498cef123cfe36becfc45ee61876cd9 https://github.com/aws-samples/sample-umarell.git\n' > "$T/umarell.pin"
# The installer in the fake clone. FAKE_STATUS_RC: status --wait's exit; FAKE_STATUS_HANG=1: status --wait does not
# return (a real sleep, so that the deadline's kill ends it); FAKE_EDGE_STUCK=<n>: the edge stack's
# delete fails until the n-th --edge-only retry; FAKE_MAIN_STUCK=1: the main stack does not delete.
cat > "$T/install.sh" <<'EOF'
#!/usr/bin/env bash
printf 'install.sh %s\n' "$*" >> "$FAKE/calls"
if IFS= read -r line; then printf 'install.sh STDIN-DATA %s\n' "$line" >> "$FAKE/calls"; fi
cmd="$1"; stack="" region="" edge_only=0
while [ $# -gt 0 ]; do
  case "$1" in --stack-name) stack="$2"; shift ;; --region) region="$2"; shift ;; --edge-only) edge_only=1 ;; esac; shift
done
s="$FAKE/stacks"; mkdir -p "$s/$region" "$s/$FAKE_R"
case "$cmd" in
  install) echo CREATE_COMPLETE > "$s/$region/$stack"; echo CREATE_COMPLETE > "$s/$FAKE_R/$stack-edge"; exit "${FAKE_INSTALL_RC:-0}" ;;
  status) [ "${FAKE_STATUS_HANG:-0}" = 1 ] && exec /bin/sleep 30; exit "${FAKE_STATUS_RC:-0}" ;;
  uninstall)
    if [ "$edge_only" = 0 ]; then
      if [ "${FAKE_MAIN_STUCK:-0}" = 1 ]; then echo DELETE_FAILED > "$s/$region/$stack"; exit 1; fi
      rm -f "$s/$region/$stack"
    fi
    n=$(cat "$FAKE/edge-tries" 2>/dev/null || echo 0)
    [ "$edge_only" = 1 ] && n=$((n + 1)) && echo "$n" > "$FAKE/edge-tries"
    if [ "$n" -lt "${FAKE_EDGE_STUCK:-0}" ]; then echo DELETE_FAILED > "$s/$FAKE_R/$stack-edge"; exit 1; fi
    rm -f "$s/$FAKE_R/$stack-edge" ;;
esac
EOF
chmod +x "$T/bin/"* "$T/install.sh"
export PATH="$T/bin:$PATH"

reset() {
  rm -rf "$T/calls" "$T/stacks" "$T/gate" "$T/edge-tries" "$T/cwd" "$T/umarell-jam.pin"
  mkdir -p "$T/stacks/$FAKE_R" "$T/cwd"; : > "$T/calls"
  unset FAKE_NODE FAKE_AWS_VERSION FAKE_STATUS_RC FAKE_INSTALL_RC FAKE_EDGE_STUCK FAKE_MAIN_STUCK FAKE_OPEN FAKE_STATUS_HANG \
    FAKE_UMARELL_PRIVATE
}
# Standalone copies of the scripts (not deployment/*.sh of a tree with the installer): they clone REPO_URL.
SD="$T/standalone"; mkdir -p "$SD"; cp "$dep/deploy.sh" "$dep/cleanup.sh" "$SD/"
# run <script> [stdin file]: runs it from $T/cwd (where the state file goes); prints its output, returns its exit.
run() { (cd "$T/cwd" && TMPDIR="$T/tmp" bash "$1" < "${2:-/dev/null}" 2>&1); }
mkdir -p "$T/tmp"
# line_of <fixed text>: the first line of $T/calls containing it (0 when none).
line_of() { local n; n=$(grep -nF -- "$1" "$T/calls" | head -1 | cut -d: -f1); echo "${n:-0}"; }
# in_order <text>...: each one appears in $T/calls, after the one before it.
in_order() { local prev=0 n t; for t in "$@"; do n=$(line_of "$t"); [ "$n" -gt "$prev" ] || return 1; prev=$n; done; }
# no_clone: nothing was cloned.
no_clone() { ! grep -q '^git clone' "$T/calls"; }
stack_of() { sed -n 's/^STACK_NAME=//p' "$T/cwd/factoryos-oneclick.state"; }

# --- static rules (ASCV) ----------------------------------------------------------------------------------------
out=$(bash -n "$dep/deploy.sh" 2>&1 && bash -n "$dep/cleanup.sh" 2>&1); check "bash -n deploy.sh cleanup.sh" $?  "$out"
out=$(grep -nE '^\s*read\b|--template-body|\bservice\s' "$dep/deploy.sh" "$dep/cleanup.sh")
check "no read, --template-body or service" $([ -z "$out" ]; echo $?) "$out"
out=$(grep -nE '\bcdk\s+deploy\b' "$dep/deploy.sh" "$dep/cleanup.sh" | grep -v -- '--require-approval never')
check "no cdk deploy without --require-approval never" $([ -z "$out" ]; echo $?) "$out"
for f in deploy.sh cleanup.sh; do
  out=$(sed -n '2,16p' "$dep/$f")
  check "$f: set -euo pipefail and export AWS_REGION=$FAKE_R" \
    $(grep -qx 'set -euo pipefail' <<<"$out" && grep -qxF "export AWS_REGION=\"$FAKE_R\"" <<<"$out"; echo $?)
done

# --- deploy.sh: a fresh account (no CDKToolkit) -------------------------------------------------------------------
reset
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: a fresh install succeeds" "$rc" "$out"$'\n'"$(cat "$T/calls")"
check "deploy.sh: the 8 steps are echoed in order" \
  $(in_order(){ local prev=0 n i; for i in 1 2 3 4 5 6 7 8; do n=$(grep -n "^==> \[$i/8\]" <<<"$out" | head -1 | cut -d: -f1); [ "${n:-0}" -gt "$prev" ] || return 1; prev=$n; done; }; in_order; echo $?) "$out"
S=$(stack_of)
check "deploy.sh: the stack name is dynamic and fits the installer's limit" \
  $([[ "$S" =~ ^factoryos-[0-9]{12}-[0-9a-f]{4}$ ]] && [ "${#S}" -le 31 ] && [[ "$S" =~ ^[a-z][a-z0-9-]{0,30}$ ]]; echo $?) "$S"
check "deploy.sh: the commands run in order (account, clone, umarell, CDKToolkit, install, gate, status)" \
  $(in_order "aws sts get-caller-identity" "git clone --depth 1 https://github.com/aws-solutions-library-samples/guidance-for-physical-ai-production-designer-on-aws.git" \
     "git -c credential.helper= ls-remote https://github.com/aws-samples/sample-umarell.git" \
     "aws cloudformation describe-stacks --stack-name CDKToolkit --region $FAKE_R" "install.sh install" \
     "OutputKey=='GateVerified'" "curl" "install.sh status"; echo $?) "$(cat "$T/calls")"
check "deploy.sh: the tools are checked before the first AWS call" \
  $([ "$(line_of "aws --version")" -lt "$(line_of "aws sts")" ] && [ "$(line_of "git --version")" -lt "$(line_of "aws sts")" ]; echo $?)
check "deploy.sh: no CDKToolkit: no npm/npx/cdk bootstrap of its own (install.sh bootstraps, with the app's context)" \
  $(! grep -qE '^(npx|npm) |cdk bootstrap' "$T/calls" && grep -q 'install.sh bootstraps it' <<<"$out"; echo $?) "$(cat "$T/calls")"
want="install.sh install --from-checkout --yes --region $FAKE_R --stack-name $S --users admin@example.com"
check "deploy.sh: install.sh gets exactly the one-click arguments" $(grep -qxF "$want" "$T/calls"; echo $?) "$(grep '^install.sh install' "$T/calls")"
check "deploy.sh: status --wait for the same stack" \
  $(grep -qxF "install.sh status --wait --yes --region $FAKE_R --stack-name $S" "$T/calls"; echo $?) "$(grep '^install.sh status' "$T/calls")"
check "deploy.sh: the state file names the stack and the clone" \
  $(grep -qx "REGION=$FAKE_R" "$T/cwd/factoryos-oneclick.state" && [ -x "$(sed -n 's/^CLONE_DIR=//p' "$T/cwd/factoryos-oneclick.state")/deployment/install.sh" ]; echo $?) \
  "$(cat "$T/cwd/factoryos-oneclick.state")"
check "deploy.sh: no direct cdk deploy" $(! grep -q 'cdk deploy' "$T/calls"; echo $?)

# --- deploy.sh: CDKToolkit exists ---------------------------------------------------------------------------------
reset; echo UPDATE_COMPLETE > "$T/stacks/$FAKE_R/CDKToolkit"
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: bootstrap is skipped when CDKToolkit exists" \
  $([ "$rc" = 0 ] && ! grep -qE '^(npx|npm) ' "$T/calls" && grep -q 'CDKToolkit is UPDATE_COMPLETE' <<<"$out"; echo $?) "$out"
reset; echo ROLLBACK_COMPLETE > "$T/stacks/$FAKE_R/CDKToolkit"
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: a broken CDKToolkit stops before the install" \
  $([ "$rc" != 0 ] && ! grep -q '^install.sh' "$T/calls" && grep -q ROLLBACK_COMPLETE <<<"$out"; echo $?) "$out"

# --- deploy.sh: failures ------------------------------------------------------------------------------------------
reset; export FAKE_STATUS_RC=1
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: a failed or rolled-back release exits non-zero" \
  $([ "$rc" != 0 ] && grep -q 'failed or was rolled back' <<<"$out" && ! grep -q '^==> \[8/8\]' <<<"$out"; echo $?) "$out"
check "deploy.sh: the stack name is recorded even when the install does not finish" $([ -n "$(stack_of)" ]; echo $?)
reset; export FAKE_STATUS_HANG=1
out=$(ONECLICK_RELEASE_TIMEOUT_SECONDS=0 run "$SD/deploy.sh"); rc=$?
check "deploy.sh: a release still building at the deadline is stopped and exits non-zero (no timeout(1))" \
  $([ "$rc" != 0 ] && grep -q 'not ready after 0 h' <<<"$out" && ! grep -q '^==> \[8/8\]' <<<"$out" \
     && ! grep -qE '^\s*timeout\b|for t in .*\btimeout\b' "$SD/deploy.sh"; echo $?) "$out"
check "deploy.sh: the deadline's default is 6 h" \
  $(grep -qxF 'RELEASE_TIMEOUT_SECONDS="${ONECLICK_RELEASE_TIMEOUT_SECONDS:-21600}"' "$dep/deploy.sh"; echo $?)
reset; export FAKE_UMARELL_PRIVATE=1
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: umarell not public and no UMARELL_TARBALL: stops at step 3, before any install" \
  $([ "$rc" != 0 ] && grep -q "umarell's repository does not answer" <<<"$out" && grep -q 'UMARELL_TARBALL' <<<"$out" \
     && ! grep -q '^install.sh' "$T/calls" && ! grep -q CDKToolkit "$T/calls"; echo $?) "$out"
reset; export FAKE_INSTALL_RC=3
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: a failed install exits non-zero before the gate check" \
  $([ "$rc" != 0 ] && ! grep -q "GateVerified" "$T/calls"; echo $?) "$out"
reset; echo false > "$T/gate"
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: GateVerified=false exits non-zero, no wait" \
  $([ "$rc" != 0 ] && grep -q "GateVerified is 'false'" <<<"$out" && ! grep -q '^install.sh status' "$T/calls"; echo $?) "$out"
reset; export FAKE_OPEN=1
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: an anonymous 200 exits non-zero, no wait" \
  $([ "$rc" != 0 ] && grep -q "did not get the gate's 401" <<<"$out" && ! grep -q '^install.sh status' "$T/calls"; echo $?) "$out"
reset; export FAKE_NODE=18.20.4
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: Node.js 18 is refused before any AWS call" \
  $([ "$rc" != 0 ] && grep -q 'Node.js 20' <<<"$out" && ! grep -q '^aws sts' "$T/calls"; echo $?) "$out"
reset; export FAKE_AWS_VERSION="aws-cli/1.33.0 Python/3.9"
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: AWS CLI v1 is refused" $([ "$rc" != 0 ] && grep -q 'CLI v2' <<<"$out"; echo $?) "$out"

# --- deploy.sh: the other static parameters (a copy with them changed) ----------------------------------------------
reset
printf '#!/usr/bin/env bash\nONECLICK_WAIT_FOR_RELEASE=0 ONECLICK_UMARELL_TARBALL=s3://bkt/umarell.tar.gz exec bash %q "$@"\n' \
  "$SD/deploy.sh" > "$T/deploy-params.sh"
out=$(run "$T/deploy-params.sh"); rc=$?
check "deploy.sh: UMARELL_TARBALL is passed, WAIT_FOR_RELEASE=0 skips the wait" \
  $([ "$rc" = 0 ] && grep -q '^install.sh install .* --umarell-tarball s3://bkt/umarell.tar.gz$' "$T/calls" && ! grep -q '^install.sh status' "$T/calls"; echo $?) \
  "$out"$'\n'"$(cat "$T/calls")"
check "deploy.sh: with UMARELL_TARBALL, GitHub is not asked about umarell" $(! grep -q 'ls-remote' "$T/calls"; echo $?) "$(cat "$T/calls")"
reset; export FAKE_UMARELL_PRIVATE=1
out=$(run "$T/deploy-params.sh"); rc=$?
check "deploy.sh: with UMARELL_TARBALL, a private umarell repository does not stop it" $([ "$rc" = 0 ]; echo $?) "$out"

# --- deploy.sh: umarell_jam's umarell (deploy/umarell-jam.pin): optional, a warning at most ---------------------------
jam_pin='# <commit> <url>
76ea13aafdeede10be11936cf0830613e8717907 https://github.com/aws-samples/sample-umarell.git'
reset; printf '%s\n' "$jam_pin" > "$T/umarell-jam.pin"
out=$(run "$SD/deploy.sh"); rc=$?
check "deploy.sh: umarell-jam.pin in umarell's public repository: not asked again, no warning, no jam option" \
  $([ "$rc" = 0 ] && [ "$(grep -c 'ls-remote' "$T/calls")" = 1 ] && grep -q "umarell_jam's umarell 76ea13a" <<<"$out" \
     && ! grep -q WARNING <<<"$out" && ! grep -q -- '--umarell-jam-tarball' "$T/calls"; echo $?) "$out"$'\n'"$(cat "$T/calls")"
reset; printf '%s\n' "$jam_pin" > "$T/umarell-jam.pin"; export FAKE_UMARELL_PRIVATE=1
out=$(run "$T/deploy-params.sh"); rc=$?
check "deploy.sh: UMARELL_TARBALL, a private repository, no UMARELL_JAM_TARBALL: warns and installs (umarell_jam incomplete)" \
  $([ "$rc" = 0 ] && grep -q "WARNING: umarell_jam's umarell (https://github.com/aws-samples/sample-umarell.git) does not answer" <<<"$out" \
     && grep -q 'UMARELL_JAM_TARBALL' <<<"$out" && grep -q '^install.sh install ' "$T/calls" && ! grep -q -- '--umarell-jam-tarball' "$T/calls"; echo $?) "$out"
reset; printf '%s\n' "$jam_pin" > "$T/umarell-jam.pin"; export FAKE_UMARELL_PRIVATE=1
out=$(ONECLICK_UMARELL_JAM_TARBALL=s3://bkt/umarell-jam.tar.gz run "$T/deploy-params.sh"); rc=$?
check "deploy.sh: UMARELL_JAM_TARBALL is passed (after --umarell-tarball), GitHub not asked, no warning" \
  $([ "$rc" = 0 ] && grep -q '^install.sh install .* --umarell-tarball s3://bkt/umarell.tar.gz --umarell-jam-tarball s3://bkt/umarell-jam.tar.gz$' "$T/calls" \
     && ! grep -q 'ls-remote' "$T/calls" && ! grep -q WARNING <<<"$out"; echo $?) "$out"$'\n'"$(cat "$T/calls")"

# --- nothing reads stdin --------------------------------------------------------------------------------------------
reset; yes y | head -50 > "$T/stdin"
out=$(run "$SD/deploy.sh" "$T/stdin"); rc=$?
check "deploy.sh: the installer never gets the caller's stdin" $([ "$rc" = 0 ] && ! grep -q STDIN-DATA "$T/calls"; echo $?) "$(grep STDIN "$T/calls")"
cp "$T/cwd/factoryos-oneclick.state" "$T/state.keep"
out=$(run "$SD/cleanup.sh" "$T/stdin"); rc=$?
check "cleanup.sh: the installer never gets the caller's stdin" $([ "$rc" = 0 ] && ! grep -q STDIN-DATA "$T/calls"; echo $?) "$out"

# --- cleanup.sh -----------------------------------------------------------------------------------------------------
deployed() { reset; run "$SD/deploy.sh" >/dev/null; S=$(stack_of); : > "$T/calls"; }
deployed
out=$(run "$SD/cleanup.sh"); rc=$?
check "cleanup.sh: uninstalls the recorded stack" \
  $([ "$rc" = 0 ] && grep -qxF "install.sh uninstall --yes --confirm-stack $S --region $FAKE_R --stack-name $S" "$T/calls"; echo $?) "$out"$'\n'"$(cat "$T/calls")"
check "cleanup.sh: the 6 stages are echoed, and both stacks are verified gone" \
  $(for i in 1 2 3 4 5 6; do grep -q "^==> \[$i/6\]" <<<"$out" || exit 1; done
    in_order "install.sh uninstall" "describe-stacks --stack-name $S --region $FAKE_R" "describe-stacks --stack-name $S-edge --region $FAKE_R"; echo $?) "$out"
check "cleanup.sh: no edge retry when the edge stack is gone; the state file is removed" \
  $(! grep -q 'edge-only' "$T/calls" && ! grep -q '^sleep' "$T/calls" && [ ! -f "$T/cwd/factoryos-oneclick.state" ]; echo $?)
check "cleanup.sh: reuses deploy.sh's clone" $(no_clone; echo $?)

deployed; export FAKE_EDGE_STUCK=3
out=$(run "$SD/cleanup.sh"); rc=$?
check "cleanup.sh: retries uninstall --edge-only every 15 min until the edge stack is gone" \
  $([ "$rc" = 0 ] && [ "$(grep -c "^install.sh uninstall --edge-only --yes --confirm-stack $S --region $FAKE_R --stack-name $S$" "$T/calls")" = 3 ] \
     && [ "$(grep -c '^sleep 900$' "$T/calls")" = 3 ] && [ ! -f "$T/stacks/$FAKE_R/$S-edge" ]; echo $?) "$out"$'\n'"$(cat "$T/calls")"
check "cleanup.sh: each retry waits first" $(in_order "sleep 900" "install.sh uninstall --edge-only"; echo $?)

deployed; export FAKE_EDGE_STUCK=100
out=$(run "$SD/cleanup.sh"); rc=$?
check "cleanup.sh: gives up after 2 h (8 retries) with a non-zero exit" \
  $([ "$rc" != 0 ] && [ "$(grep -c '^install.sh uninstall --edge-only' "$T/calls")" = 8 ] && grep -q 'still there' <<<"$out" \
     && [ -f "$T/cwd/factoryos-oneclick.state" ]; echo $?) "$out"

deployed; export FAKE_MAIN_STUCK=1
out=$(run "$SD/cleanup.sh"); rc=$?
check "cleanup.sh: a main stack that does not delete exits non-zero without edge retries" \
  $([ "$rc" != 0 ] && ! grep -q 'edge-only' "$T/calls" && grep -q DELETE_FAILED <<<"$out"; echo $?) "$out"

deployed; rm -rf "$(sed -n 's/^CLONE_DIR=//p' "$T/cwd/factoryos-oneclick.state")"
out=$(run "$SD/cleanup.sh"); rc=$?
check "cleanup.sh: clones again when deploy.sh's clone is gone" \
  $([ "$rc" = 0 ] && in_order "git clone --depth 1 https://github.com/aws-solutions-library-samples/" "install.sh uninstall"; echo $?) "$out"

# --- in a tree: deployment/deploy.sh and cleanup.sh next to the installer use that tree, not a clone ------------------
TR="$T/tree"; mkdir -p "$TR/deployment" "$TR/source/deploy"
cp "$dep/deploy.sh" "$dep/cleanup.sh" "$TR/deployment/"; cp "$T/install.sh" "$TR/deployment/install.sh"
cp "$T/umarell.pin" "$TR/source/deploy/umarell.pin"
reset
out=$(run "$TR/deployment/deploy.sh"); rc=$?
check "deploy.sh in a tree: installs that tree, no clone" \
  $([ "$rc" = 0 ] && no_clone && grep -q '^install.sh install --from-checkout' "$T/calls" \
     && grep -qx "CLONE_DIR=$TR" "$T/cwd/factoryos-oneclick.state" && grep -q "the tree this script is in: $TR" <<<"$out"; echo $?) \
  "$out"$'\n'"$(cat "$T/calls")"
# The origin secret (deployment/CONTRACT.md): install.sh makes a new one for a new installation itself; the one-click
# install passes no secret and asks for no rotation, and uninstall deletes both of its parameters (test-installer.sh).
check "deploy.sh: no origin secret option or value passed to the installer" \
  $(grep '^install.sh install' "$T/calls" | grep -qiE 'origin|secret|verify'; [ $? = 1 ]; echo $?) "$(grep '^install.sh' "$T/calls")"
: > "$T/calls"
out=$(run "$TR/deployment/cleanup.sh"); rc=$?
check "cleanup.sh in a tree: uninstalls with that tree's installer, no clone" \
  $([ "$rc" = 0 ] && no_clone && grep -q '^install.sh uninstall' "$T/calls" && grep -q "the tree this script is in: $TR" <<<"$out"; echo $?) "$out"

reset
out=$(run "$SD/cleanup.sh"); rc=$?
check "cleanup.sh: without deploy.sh's state file it exits non-zero and calls nothing" \
  $([ "$rc" != 0 ] && grep -q 'factoryos-oneclick.state is missing' <<<"$out" && [ ! -s "$T/calls" ]; echo $?) "$out"
reset; printf 'STACK_NAME=Bad;rm -rf /\n' > "$T/cwd/factoryos-oneclick.state"
out=$(run "$SD/cleanup.sh"); rc=$?
check "cleanup.sh: a malformed stack name is refused" $([ "$rc" != 0 ] && [ ! -s "$T/calls" ]; echo $?) "$out"

# --- the CloudFormation installer's overrides (ONECLICK_*) -----------------------------------------------------------
reset
out=$(ONECLICK_REGION=eu-central-1 ONECLICK_STACK_NAME=pai-pd ONECLICK_ADMIN_EMAIL=ops@example.org \
  ONECLICK_INSTANCE_TYPE=c7g.4xlarge ONECLICK_VOLUME_GB=300 ONECLICK_BEDROCK=true ONECLICK_STATE_FILE="$T/ov.state" \
  ONECLICK_UMARELL_TARBALL=s3://bkt/u.tar.gz ONECLICK_WAIT_FOR_RELEASE=1 run "$SD/deploy.sh"); rc=$?
want="install.sh install --from-checkout --yes --region eu-central-1 --stack-name pai-pd --users ops@example.org --umarell-tarball s3://bkt/u.tar.gz --instance-type c7g.4xlarge --volume-gb 300 --bedrock"
check "deploy.sh: ONECLICK_* set the region, stack, user, umarell, instance, volume and Bedrock" \
  $([ "$rc" = 0 ] && grep -qxF "$want" "$T/calls"; echo $?) "$out"$'\n'"$(grep '^install.sh' "$T/calls")"
check "deploy.sh: ONECLICK_*: CDKToolkit is checked in the stack's region and the edge region" \
  $(grep -qF "describe-stacks --stack-name CDKToolkit --region eu-central-1" "$T/calls" \
     && grep -qF "describe-stacks --stack-name CDKToolkit --region $FAKE_R" "$T/calls"; echo $?) "$(cat "$T/calls")"
check "deploy.sh: ONECLICK_STATE_FILE is where the state goes" \
  $(grep -qx 'STACK_NAME=pai-pd' "$T/ov.state" && grep -qx 'REGION=eu-central-1' "$T/ov.state" && [ ! -f "$T/cwd/factoryos-oneclick.state" ]; echo $?)
reset
out=$(ONECLICK_BEDROCK=false run "$SD/deploy.sh"); rc=$?
check "deploy.sh: ONECLICK_BEDROCK=false passes --no-bedrock" $([ "$rc" = 0 ] && grep -q '^install.sh install .* --no-bedrock$' "$T/calls"; echo $?) "$(grep '^install.sh install' "$T/calls")"
reset
out=$(ONECLICK_BEDROCK=maybe run "$SD/deploy.sh"); rc=$?
check "deploy.sh: ONECLICK_BEDROCK=maybe is refused before any install" \
  $([ "$rc" != 0 ] && grep -q 'ONECLICK_BEDROCK must be' <<<"$out" && ! grep -q '^install.sh' "$T/calls"; echo $?) "$out"
reset
out=$(AWS_REGION=eu-west-1 AWS_DEFAULT_REGION=eu-west-1 run "$SD/deploy.sh"); rc=$?
check "deploy.sh: a caller's AWS_REGION (CodeBuild sets it) does not move an ASCV run from $FAKE_R" \
  $([ "$rc" = 0 ] && grep -q "^install.sh install --from-checkout --yes --region $FAKE_R " "$T/calls" && ! grep -q eu-west-1 "$T/calls"; echo $?) "$(cat "$T/calls")"
reset
ONECLICK_REGION=eu-central-1 ONECLICK_STACK_NAME=pai-pd ONECLICK_STATE_FILE="$T/ov.state" run "$SD/deploy.sh" >/dev/null
: > "$T/calls"; rm -f "$T/ov.state"
out=$(ONECLICK_REGION=eu-central-1 ONECLICK_STACK_NAME=pai-pd run "$SD/cleanup.sh"); rc=$?
check "cleanup.sh: ONECLICK_STACK_NAME and ONECLICK_REGION, no state file: uninstalls that stack" \
  $([ "$rc" = 0 ] && grep -qxF "install.sh uninstall --yes --confirm-stack pai-pd --region eu-central-1 --stack-name pai-pd" "$T/calls" \
     && [ ! -f "$T/stacks/eu-central-1/pai-pd" ] && [ ! -f "$T/stacks/$FAKE_R/pai-pd-edge" ]; echo $?) "$out"$'\n'"$(cat "$T/calls")"
reset
out=$(ONECLICK_STACK_NAME='Bad;x' run "$SD/cleanup.sh"); rc=$?
check "cleanup.sh: a malformed ONECLICK_STACK_NAME is refused" $([ "$rc" != 0 ] && [ ! -s "$T/calls" ]; echo $?) "$out"

echo
echo "$pass passed, $fail failed"
[ "$fail" = 0 ]
