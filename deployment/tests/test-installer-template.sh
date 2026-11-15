#!/usr/bin/env bash
# Tests of the CloudFormation installer deployment/pai-pd-installer.yaml: its build script (the buildspec's
# /tmp/pai-pd-build.sh, cut out of the template as text) with fake aws and curl on PATH and a fake export whose
# deploy.sh / cleanup.sh log their ONECLICK_* environment; and its custom resource function (the inline ZipFile) with
# a fake boto3 and a recorded response. cfn-lint, where installed, lints the template. No AWS calls, no network.
#   bash deployment/tests/test-installer-template.sh </dev/null
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
dep="$(cd "$here/.." && pwd)"
tpl="$dep/pai-pd-installer.yaml"
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
pass=0 fail=0
check() { if [ "$2" = 0 ]; then pass=$((pass+1)); echo "ok    $1"; else fail=$((fail+1)); echo "FAIL  $1"; [ -z "${3:-}" ] || printf '%s\n' "$3" | sed 's/^/      /'; fi; }

# --- the template's embedded code ----------------------------------------------------------------------------------
# The build script: the heredoc's body, without the block's indentation.
awk '/<<.PAIPD_BUILD_EOF.$/ {on=1; match($0, /^ */); ind=RLENGTH; next}
     on && $0 ~ /^ *PAIPD_BUILD_EOF$/ {exit}
     on {print substr($0, ind + 1)}' "$tpl" > "$T/build.sh"
# The function: the ZipFile block, without its indentation. The block ends at the first non-blank line indented less
# than its 10 columns. Counted with match(), not an interval regex: mawk (Debian's awk, test:deploy's) has no {m,n}.
awk '/ZipFile: \|$/ {on=1; next}
     on && NF { match($0, /^ */); if (RLENGTH < 10) exit }
     on {print substr($0, 11)}' "$tpl" > "$T/index.py"
# public-export:verify-build runs this file on bare amazonlinux:2023, which has neither findutils (find, xargs) nor
# diffutils (cmp, diff): both passed test:deploy (Debian) and failed v0.1.23's export.
out=$(grep -nE '(^|[^A-Za-z_-])(find|xargs|cmp|diff)( |$)' "$0" | grep -vE '^[0-9]+:\s*#|grep -nE|^[0-9]+:check "this test uses no' || true)
check "this test uses no find, xargs, cmp or diff (verify-build's amazonlinux:2023 has none)" $([ -z "$out" ]; echo $?) "$out"
check "the build script is in the template" $([ "$(wc -l < "$T/build.sh")" -gt 100 ] && head -1 "$T/build.sh" | grep -qx '#!/usr/bin/env bash'; echo $?) "$(head -3 "$T/build.sh")"
out=$(bash -n "$T/build.sh" 2>&1); check "bash -n of the build script" $? "$out"
check "the function is in the template" $(grep -q '^def handler(e, context):' "$T/index.py"; echo $?)
check "the function fits an inline ZipFile (4096 characters)" $([ "$(wc -c < "$T/index.py")" -lt 4096 ]; echo $?) "$(wc -c < "$T/index.py")"
out=$(python3 -c 'import sys; compile(open(sys.argv[1]).read(), "index.py", "exec")' "$T/index.py" 2>&1); check "the function compiles" $? "$out"
if command -v cfn-lint >/dev/null 2>&1; then
  out=$(cfn-lint "$tpl" 2>&1); check "cfn-lint" $? "$out"
else
  echo "skip  cfn-lint (not installed)"
fi
# (Internal names are the public export's denylist gate's job: scripts/export-public.py scans every exported file.)
out=$(grep -nE '[0-9]{12}' "$tpl" || true)
check "no account ids in the template" $([ -z "$out" ]; echo $?) "$out"
check "the build role is assumable by CodeBuild only, for this project" \
  $(grep -q 'Principal: {Service: codebuild.amazonaws.com}' "$tpl" && grep -q "aws:SourceArn: !Sub 'arn:\${AWS::Partition}:codebuild" "$tpl"; echo $?)
check "UmarellJamTarballUri: a parameter, passed to the build, in the build's Config only when set" \
  $(grep -q '^  UmarellJamTarballUri:$' "$tpl" && grep -qF -- '- {Name: UMARELL_JAM_TARBALL_URI, Value: !Ref UmarellJamTarballUri}' "$tpl" \
     && grep -qF "!If [HasJamTarball, !Sub '|jam=\${UmarellJamTarballUri}', '']" "$tpl"; echo $?)
check "the inputs bucket blocks public access and denies plain HTTP" \
  $(for k in BlockPublicAcls BlockPublicPolicy IgnorePublicAcls RestrictPublicBuckets; do grep -q "$k: true" "$tpl" || exit 1; done
    grep -q "aws:SecureTransport: 'false'" "$tpl"; echo $?)

# --- fakes for the build script ------------------------------------------------------------------------------------
# S3 is $T/s3/<bucket>/<key>; https://files.example.com/<path> is $T/http/<path>. Every call goes to $T/calls.
export FAKE="$T"
mkdir -p "$T/bin" "$T/s3" "$T/http"
cat > "$T/bin/aws" <<'EOF'
#!/usr/bin/env bash
printf 'aws %s\n' "$*" >> "$FAKE/calls"
path() { local u="${1#s3://}"; printf '%s/s3/%s' "$FAKE" "$u"; }
case "$1 $2" in
  "s3 cp")
    shift 2; args=(); for a in "$@"; do [ "$a" = --only-show-errors ] || args+=("$a"); done
    src="${args[0]}" dst="${args[1]}"
    if [[ "$src" == s3://* ]]; then
      [ -f "$(path "$src")" ] || { echo "fatal error: An error occurred (404) when calling the HeadObject operation: Key not found" >&2; exit 1; }
      cp "$(path "$src")" "$dst"
    else
      mkdir -p "$(dirname "$(path "$dst")")"; cp "$src" "$(path "$dst")"
    fi ;;
  "s3 ls") [ -f "$(path "${!#}")" ] && echo "2026-11-13 12:00:00 1 $(basename "${!#}")" ;;
  "s3 rm") args=(); for a in "$@"; do [ "$a" = --only-show-errors ] || args+=("$a"); done; rm -f "$(path "${args[2]}")" ;;
  "s3api list-objects-v2")
    shift 2; bucket="" prefix=""
    while [ $# -gt 0 ]; do case "$1" in --bucket) bucket="$2"; shift ;; --prefix) prefix="$2"; shift ;; esac; shift; done
    d="$FAKE/s3/$bucket/$prefix"
    # Bash globs, no find(1): public-export:verify-build runs this on bare amazonlinux:2023 (no findutils, no diffutils).
    keys=""; for f in "$d"/*; do [ -f "$f" ] && keys+="${keys:+$'\t'}$prefix${f##*/}"; done
    echo "${keys:-None}" ;;
  "cloudformation describe-stacks") echo "https://d111111abcdef8.cloudfront.net/" ;;
  *) echo "fake aws: unhandled: $*" >&2; exit 2 ;;
esac
EOF
cat > "$T/bin/curl" <<'EOF'
#!/usr/bin/env bash
printf 'curl %s\n' "$*" >> "$FAKE/calls"
out="" head=0 put=0 data="" url="${!#}"
while [ $# -gt 0 ]; do
  case "$1" in -o) out="$2"; shift ;; -X) [ "$2" = PUT ] && put=1; shift ;; --data-binary) data="$2"; shift ;;
    -H|--retry) shift ;; -*I*) head=1 ;; esac; shift
done
if [ "$put" = 1 ]; then printf '%s\n' "$data" >> "$FAKE/signals"; exit 0; fi
case "$url" in
  */releases/latest)
    [ -n "${FAKE_LATEST:-}" ] || { printf 'HTTP/2 302\r\nlocation: %s\r\n\r\n' "${url%/latest}"; exit 0; }
    printf 'HTTP/2 302\r\nlocation: %s/tag/%s\r\n\r\n' "${url%/latest}" "$FAKE_LATEST" ;;
  https://github.com/*/archive/refs/tags/*) f="$FAKE/http/github/${url##*/}"; [ -f "$f" ] || exit 22; cp "$f" "$out" ;;
  https://files.example.com/*) p="${url#https://files.example.com/}"; f="$FAKE/http/${p%%\?*}"; [ -f "$f" ] || exit 22; cp "$f" "$out" ;;
  *) echo "fake curl: unhandled $url" >&2; exit 2 ;;
esac
EOF
if ! command -v sha256sum >/dev/null 2>&1; then
  printf '#!/usr/bin/env bash\nexec shasum -a 256 "$@"\n' > "$T/bin/sha256sum"
fi
chmod +x "$T/bin/"*
export PATH="$T/bin:$PATH"

# The fake export: deployment/{install,deploy,cleanup}.sh (deploy.sh / cleanup.sh log their ONECLICK_* environment;
# FAKE_DEPLOY_RC makes deploy.sh fail with an install.sh-style ERROR line) and source/VERSION.json.
mk_tree() {  # mk_tree <dir> <top>
  local t="$1/$2"; mkdir -p "$t/deployment" "$t/source"
  printf '#!/usr/bin/env bash\necho install.sh\n' > "$t/deployment/install.sh"
  for s in deploy cleanup; do
    cat > "$t/deployment/$s.sh" <<EOF
#!/usr/bin/env bash
{ printf '$s.sh in %s\n' "\$PWD"; env | grep -E '^(ONECLICK_|FACTORYOS_WORK=)' | sort | sed 's/^/  /'; } >> "\$FAKE/calls"
[ -x "\$(dirname "\$0")/install.sh" ] || { echo "install.sh not executable" >> "\$FAKE/calls"; exit 9; }
if [ "\${FAKE_DEPLOY_RC:-0}" != 0 ]; then echo "some output"; echo "ERROR the instance did not come up" >&2; exit "\$FAKE_DEPLOY_RC"; fi
echo "$s done"
EOF
  done
  printf '{"format":1,"release":"v0.1.20"}\n' > "$t/source/VERSION.json"
}
SRC="$T/src"; mk_tree "$SRC" pai-pd-v0.1.20
python3 - "$SRC" "$T/export.zip" <<'EOF'
import os, sys, zipfile
root, out = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(out, "w") as z:
    for d, _, fs in os.walk(root):
        for f in fs:
            p = os.path.join(d, f); z.write(p, os.path.relpath(p, root))   # no exec bits kept: like a zip from Windows
EOF
tar -czf "$T/export.tar.gz" -C "$SRC" pai-pd-v0.1.20
echo "umarell bytes" > "$T/umarell.tar.gz"
echo "umarell 76ea13a bytes" > "$T/umarell-jam.tar.gz"
sha() { (sha256sum "$1" 2>/dev/null || shasum -a 256 "$1") | cut -d' ' -f1; }

reset() {
  rm -rf "$T/calls" "$T/signals" "$T/s3" "$T/http" "$T/work"; mkdir -p "$T/s3/inputs-bkt" "$T/http/github"; : > "$T/calls"
  unset FAKE_LATEST FAKE_DEPLOY_RC WAIT_HANDLE_URL
  export ACTION=install SOURCE_MODE=upload RELEASE=latest \
    REPO_URL=https://github.com/aws-solutions-library-samples/guidance-for-physical-ai-production-designer-on-aws \
    ARTIFACTS_URI="" SOURCE_ARCHIVE_URI="" UMARELL_TARBALL_URI="" UMARELL_JAM_TARBALL_URI="" APP_STACK_NAME=pai-pd ADMIN_EMAIL=ops@example.org \
    INSTANCE_TYPE=m7g.2xlarge VOLUME_GB=200 BEDROCK=false INPUTS_BUCKET=inputs-bkt AWS_REGION=eu-central-1 \
    CODEBUILD_BUILD_ID=pai-pd-installer:1234 PAIPD_WORK="$T/work"
}
put() { mkdir -p "$(dirname "$T/s3/$2")"; cp "$1" "$T/s3/$2"; }   # put <file> <bucket/key>
build() { bash "$T/build.sh" </dev/null 2>&1; }
signal_field() { python3 -c 'import json, sys; print(json.loads(open(sys.argv[1]).read().splitlines()[-1])[sys.argv[2]])' "$T/signals" "$1"; }
env_has() { grep -qxF "  $1" "$T/calls"; }

# --- upload mode ---------------------------------------------------------------------------------------------------
reset; put "$T/export.zip" inputs-bkt/pai-pd.zip; put "$T/umarell.tar.gz" inputs-bkt/umarell.tar.gz
export WAIT_HANDLE_URL="https://cloudformation-waitcondition-eu-central-1.s3.amazonaws.com/handle?X-Amz-Signature=x"
out=$(build); rc=$?
check "upload: installs the uploaded export with the uploaded umarell" \
  $([ "$rc" = 0 ] && grep -q '^deploy.sh in ' "$T/calls" && env_has "ONECLICK_UMARELL_TARBALL=s3://inputs-bkt/umarell.tar.gz"; echo $?) "$out"$'\n'"$(cat "$T/calls")"
check "upload: deploy.sh gets the region, stack, user, instance, volume, Bedrock, state file and the 5 h wait" \
  $(env_has ONECLICK_REGION=eu-central-1 && env_has ONECLICK_STACK_NAME=pai-pd && env_has ONECLICK_ADMIN_EMAIL=ops@example.org \
     && env_has ONECLICK_INSTANCE_TYPE=m7g.2xlarge && env_has ONECLICK_VOLUME_GB=200 && env_has ONECLICK_BEDROCK=false \
     && env_has "ONECLICK_STATE_FILE=$T/work/oneclick.state" && env_has ONECLICK_WAIT_FOR_RELEASE=1 \
     && env_has ONECLICK_RELEASE_TIMEOUT_SECONDS=18000 && env_has "FACTORYOS_WORK=$T/work/install"; echo $?) "$(cat "$T/calls")"
check "upload: a zip without exec bits still runs deploy.sh in its own tree (install.sh made executable)" \
  $(! grep -q 'install.sh not executable' "$T/calls"; echo $?)
check "upload: signals SUCCESS with the application's URL" \
  $([ "$(signal_field Status)" = SUCCESS ] && [ "$(signal_field Data)" = https://d111111abcdef8.cloudfront.net/ ] \
     && [ "$(signal_field UniqueId)" = install ]; echo $?) "$(cat "$T/signals" 2>/dev/null)"
check "upload: the signal is a PUT with an empty Content-Type" $(grep -q "^curl .*-X PUT -H Content-Type: " "$T/calls"; echo $?) "$(grep PUT "$T/calls")"
check "upload: the presigned signal URL's query is not printed" $(! grep -q 'X-Amz-Signature' <<<"$out"; echo $?)
check "upload: the installed export is kept for the uninstall (installed/<stack>.zip)" \
  $([ "$(sha "$T/export.zip")" = "$(sha "$T/s3/inputs-bkt/installed/pai-pd.zip" 2>/dev/null)" ]; echo $?) "$(ls -R "$T/s3")"

check "upload: no umarell_jam archive uploaded: deploy.sh gets none" $(env_has ONECLICK_UMARELL_JAM_TARBALL=; echo $?) "$(cat "$T/calls")"
reset; put "$T/export.zip" inputs-bkt/pai-pd.zip; put "$T/umarell.tar.gz" inputs-bkt/umarell.tar.gz; put "$T/umarell-jam.tar.gz" inputs-bkt/umarell-jam.tar.gz
out=$(build); rc=$?
check "upload: umarell-jam.tar.gz is umarell_jam's archive, not a second umarell one" \
  $([ "$rc" = 0 ] && env_has "ONECLICK_UMARELL_TARBALL=s3://inputs-bkt/umarell.tar.gz" \
     && env_has "ONECLICK_UMARELL_JAM_TARBALL=s3://inputs-bkt/umarell-jam.tar.gz"; echo $?) "$out"$'\n'"$(cat "$T/calls")"
reset; put "$T/export.zip" inputs-bkt/pai-pd.zip; put "$T/umarell-jam.tar.gz" inputs-bkt/umarell-jam-a.tar.gz; put "$T/umarell-jam.tar.gz" inputs-bkt/umarell-jam-b.tar.gz
out=$(build); rc=$?
check "upload: two umarell_jam archives without artifacts.json: a clear error naming both" \
  $([ "$rc" != 0 ] && grep -q 'more than one umarell_jam archive in s3://inputs-bkt/ (umarell-jam-a.tar.gz umarell-jam-b.tar.gz' <<<"$out" && ! grep -q '^deploy.sh' "$T/calls"; echo $?) "$out"

reset
out=$(build); rc=$?
check "upload: an empty bucket fails with what to upload" \
  $([ "$rc" != 0 ] && grep -q 'no export in s3://inputs-bkt/: upload pai-pd.zip' <<<"$out" && ! grep -q '^deploy.sh' "$T/calls"; echo $?) "$out"
check "upload: no wait handle, no signal" $([ ! -f "$T/signals" ]; echo $?)

# --- s3 mode: the artifact set ---------------------------------------------------------------------------------------
reset; export SOURCE_MODE=s3 ARTIFACTS_URI=s3://set-bkt/sets/v20/ WAIT_HANDLE_URL=https://handle.example.com/h
put "$T/export.zip" set-bkt/sets/v20/factoryos-v0.1.20.zip; sha "$T/export.zip" > "$T/s3/set-bkt/sets/v20/factoryos-v0.1.20.zip.sha256"
put "$T/umarell.tar.gz" set-bkt/sets/v20/umarell-5d4b0ea0a39b.tar.gz
out=$(build); rc=$?
check "s3: the set's export (the factoryos*.zip name) with a matching .sha256, and its umarell" \
  $([ "$rc" = 0 ] && grep -q 'sha256 ok: s3://set-bkt/sets/v20/factoryos-v0.1.20.zip' <<<"$out" \
     && env_has ONECLICK_UMARELL_TARBALL=s3://set-bkt/sets/v20/umarell-5d4b0ea0a39b.tar.gz; echo $?) "$out"
reset; export SOURCE_MODE=s3 ARTIFACTS_URI=s3://set-bkt/sets/v20 WAIT_HANDLE_URL=https://handle.example.com/h
put "$T/export.zip" set-bkt/sets/v20/pai-pd.zip; echo "0000000000000000000000000000000000000000000000000000000000000000  pai-pd.zip" > "$T/s3/set-bkt/sets/v20/pai-pd.zip.sha256"
out=$(build); rc=$?
check "s3: a wrong .sha256 fails before deploy.sh, and the stack gets the reason" \
  $([ "$rc" != 0 ] && ! grep -q '^deploy.sh' "$T/calls" && [ "$(signal_field Status)" = FAILURE ] \
     && signal_field Reason | grep -q 'sha256 mismatch: s3://set-bkt/sets/v20/pai-pd.zip'; echo $?) "$out"$'\n'"$(cat "$T/signals")"
reset; export SOURCE_MODE=s3 ARTIFACTS_URI=s3://set-bkt/a/
put "$T/export.zip" set-bkt/a/pai-pd-v0.1.19.zip; put "$T/export.zip" set-bkt/a/pai-pd-v0.1.20.zip
out=$(build); rc=$?
check "s3: two exports without artifacts.json: a clear error naming both" \
  $([ "$rc" != 0 ] && grep -q 'more than one export in s3://set-bkt/a/ (pai-pd-v0.1.19.zip pai-pd-v0.1.20.zip' <<<"$out" && ! grep -q '^deploy.sh' "$T/calls"; echo $?) "$out"
printf '{"source": "pai-pd-v0.1.20.zip", "umarell": "u/umarell.tar.gz"}\n' > "$T/s3/set-bkt/a/artifacts.json"; put "$T/umarell.tar.gz" set-bkt/a/u/umarell.tar.gz
out=$(build); rc=$?
check "s3: artifacts.json picks the export and umarell" \
  $([ "$rc" = 0 ] && grep -q 'got s3://set-bkt/a/pai-pd-v0.1.20.zip' <<<"$out" && env_has ONECLICK_UMARELL_TARBALL=s3://set-bkt/a/u/umarell.tar.gz \
     && env_has ONECLICK_UMARELL_JAM_TARBALL=; echo $?) "$out"
printf '{"source": "pai-pd-v0.1.20.zip", "umarellJam": "u/umarell-jam.tar.gz"}\n' > "$T/s3/set-bkt/a/artifacts.json"; put "$T/umarell-jam.tar.gz" set-bkt/a/u/umarell-jam.tar.gz
: > "$T/calls"; out=$(build); rc=$?
check "s3: artifacts.json's umarellJam (no umarell)" \
  $([ "$rc" = 0 ] && env_has ONECLICK_UMARELL_JAM_TARBALL=s3://set-bkt/a/u/umarell-jam.tar.gz && env_has ONECLICK_UMARELL_TARBALL= \
     && grep -q 'artifacts.json: source pai-pd-v0.1.20.zip, umarell_jam u/umarell-jam.tar.gz' <<<"$out"; echo $?) "$out"
printf '{"source": "../x.zip"}\n' > "$T/s3/set-bkt/a/artifacts.json"; : > "$T/calls"
out=$(build); rc=$?
check "s3: an artifacts.json naming ../ is refused" $([ "$rc" != 0 ] && grep -q 'artifacts.json' <<<"$out" && ! grep -q '^deploy.sh' "$T/calls"; echo $?) "$out"
reset; export SOURCE_MODE=s3
out=$(build); rc=$?
check "s3: neither ArtifactsUri nor SourceArchiveUri: refused" $([ "$rc" != 0 ] && grep -q 'SourceMode s3 needs ArtifactsUri' <<<"$out"; echo $?) "$out"

# --- explicit URIs (https, presigned) ----------------------------------------------------------------------------------
reset; export SOURCE_MODE=github SOURCE_ARCHIVE_URI="https://files.example.com/exp/pai-pd.tar.gz?X-Amz-Signature=secret" \
  UMARELL_TARBALL_URI="https://files.example.com/u/umarell.tar.gz?X-Amz-Signature=secret" \
  UMARELL_JAM_TARBALL_URI="https://files.example.com/u/umarell-jam.tar.gz?X-Amz-Signature=secret"
mkdir -p "$T/http/exp" "$T/http/u"; cp "$T/export.tar.gz" "$T/http/exp/pai-pd.tar.gz"; cp "$T/umarell.tar.gz" "$T/http/u/umarell.tar.gz"
cp "$T/umarell-jam.tar.gz" "$T/http/u/umarell-jam.tar.gz"
out=$(build); rc=$?
u12=$(sha "$T/umarell.tar.gz" | cut -c1-12); j12=$(sha "$T/umarell-jam.tar.gz" | cut -c1-12)
check "https: the export and umarell from presigned URLs; umarell is staged to the inputs bucket for install.sh" \
  $([ "$rc" = 0 ] && env_has "ONECLICK_UMARELL_TARBALL=s3://inputs-bkt/staging/umarell-$u12.tar.gz" \
     && [ -f "$T/s3/inputs-bkt/staging/umarell-$u12.tar.gz" ] && ! grep -q 'releases/latest' "$T/calls"; echo $?) "$out"$'\n'"$(cat "$T/calls")"
check "https: umarell_jam's archive staged the same way" \
  $(env_has "ONECLICK_UMARELL_JAM_TARBALL=s3://inputs-bkt/staging/umarell-jam-$j12.tar.gz" \
     && [ -f "$T/s3/inputs-bkt/staging/umarell-jam-$j12.tar.gz" ]; echo $?) "$out"$'\n'"$(cat "$T/calls")"
check "https: the presigned query strings are not printed" $(! grep -q 'secret' <<<"$out"; echo $?) "$out"
reset; export SOURCE_MODE=upload SOURCE_ARCHIVE_URI="https://files.example.com/missing.zip"
put "$T/export.zip" inputs-bkt/pai-pd.zip
out=$(build); rc=$?
check "https: SourceArchiveUri wins over the uploaded set, and a 404 fails" \
  $([ "$rc" != 0 ] && grep -q 'could not download https://files.example.com/missing.zip' <<<"$out" && ! grep -q '^deploy.sh' "$T/calls"; echo $?) "$out"

# --- github ------------------------------------------------------------------------------------------------------------
reset; export SOURCE_MODE=github FAKE_LATEST=v0.1.20; cp "$T/export.tar.gz" "$T/http/github/v0.1.20.tar.gz"
out=$(build); rc=$?
check "github: latest is resolved through the releases/latest redirect (not api.github.com) and installed" \
  $([ "$rc" = 0 ] && grep -q 'release v0.1.20 of https://github.com/aws-solutions-library-samples/' <<<"$out" \
     && grep -q "^curl .*$REPO_URL/archive/refs/tags/v0.1.20.tar.gz$" "$T/calls" && ! grep -q api.github.com "$T/calls" \
     && grep -q '^deploy.sh' "$T/calls" && env_has ONECLICK_UMARELL_TARBALL=; echo $?) "$out"$'\n'"$(cat "$T/calls")"
reset; export SOURCE_MODE=github RELEASE=v0.1.19; cp "$T/export.tar.gz" "$T/http/github/v0.1.19.tar.gz"
out=$(build); rc=$?
check "github: a pinned tag is downloaded as is" $([ "$rc" = 0 ] && ! grep -q 'releases/latest' "$T/calls" && grep -q 'tags/v0.1.19.tar.gz$' "$T/calls"; echo $?) "$out"
reset; export SOURCE_MODE=github WAIT_HANDLE_URL=https://handle.example.com/h
out=$(build); rc=$?
check "github: no release yet: a clear failure, signalled" \
  $([ "$rc" != 0 ] && grep -q 'has no published release yet' <<<"$out" && [ "$(signal_field Status)" = FAILURE ]; echo $?) "$out"

# --- failures, the export's shape, uninstall -------------------------------------------------------------------------
reset; put "$T/export.zip" inputs-bkt/pai-pd.zip; export FAKE_DEPLOY_RC=1 WAIT_HANDLE_URL=https://handle.example.com/h
out=$(build); rc=$?
check "a failing deploy.sh fails the build; the reason is its last ERROR line plus the build id" \
  $([ "$rc" != 0 ] && [ "$(signal_field Status)" = FAILURE ] && signal_field Reason | grep -q '^ERROR the instance did not come up. Build pai-pd-installer:1234'; echo $?) \
  "$out"$'\n'"$(cat "$T/signals" 2>/dev/null)"
reset; mk_tree "$T/two" a; mk_tree "$T/two" b; tar -czf "$T/two.tar.gz" -C "$T/two" a b; put "$T/two.tar.gz" inputs-bkt/pai-pd.tar.gz
out=$(build); rc=$?
check "an export with two top directories is refused" $([ "$rc" != 0 ] && grep -q 'exactly one top directory' <<<"$out"; echo $?) "$out"
reset; echo "not a guidance" > "$T/readme"; mkdir -p "$T/other/x"; cp "$T/readme" "$T/other/x/"; tar -czf "$T/other.tar.gz" -C "$T/other" x
put "$T/other.tar.gz" inputs-bkt/pai-pd.tar.gz
out=$(build); rc=$?
check "an archive without deployment/ is refused" $([ "$rc" != 0 ] && grep -q 'is it the guidance' <<<"$out"; echo $?) "$out"
reset; put "$T/export.zip" inputs-bkt/pai-pd.zip; put "$T/umarell.tar.gz" inputs-bkt/umarell.tar.gz; export ACTION=uninstall WAIT_HANDLE_URL=https://handle.example.com/h
out=$(build); rc=$?
check "uninstall: runs cleanup.sh of the export with the stack and region, no umarell, no deploy.sh, no signal" \
  $([ "$rc" = 0 ] && grep -q '^cleanup.sh in ' "$T/calls" && env_has ONECLICK_STACK_NAME=pai-pd && env_has ONECLICK_REGION=eu-central-1 \
     && ! grep -q '^deploy.sh' "$T/calls" && ! env_has ONECLICK_UMARELL_TARBALL=s3://inputs-bkt/umarell.tar.gz && [ ! -f "$T/signals" ]; echo $?) "$out"$'\n'"$(cat "$T/calls")"
reset; mkdir -p "$T/s3/inputs-bkt/installed"; cp "$T/export.tar.gz" "$T/s3/inputs-bkt/installed/pai-pd.tar.gz"
export ACTION=uninstall SOURCE_MODE=github SOURCE_ARCHIVE_URI="https://files.example.com/expired.zip?X-Amz-Expires=60"
out=$(build); rc=$?
check "uninstall: uses the export the last install kept, not the configured source (an expired URL, a moved latest)" \
  $([ "$rc" = 0 ] && grep -q 'the export the last install installed: s3://inputs-bkt/installed/pai-pd.tar.gz' <<<"$out" \
     && grep -q '^cleanup.sh in ' "$T/calls" && ! grep -q 'expired.zip' "$T/calls" && ! grep -q 'releases/latest' "$T/calls"; echo $?) "$out"
reset; export ACTION=reinstall
out=$(build); rc=$?
check "an unknown ACTION is refused" $([ "$rc" != 0 ] && grep -q 'ACTION must be install or uninstall' <<<"$out"; echo $?) "$out"
reset; export APP_STACK_NAME='Bad;x'
out=$(build); rc=$?
check "an unusable stack name is refused" $([ "$rc" != 0 ] && grep -q 'unusable AppStackName' <<<"$out"; echo $?) "$out"

# --- deployment/make-artifacts.sh: a set the build installs as is ---------------------------------------------------
reset; export WAIT_HANDLE_URL=https://handle.example.com/h
out=$(bash "$dep/make-artifacts.sh" "$T/export.zip" --umarell "$T/umarell.tar.gz" --out "$T/set" 2>&1); rc=$?
u12=$(sha "$T/umarell.tar.gz" | cut -c1-12)
check "make-artifacts.sh: names the files by the release, writes their .sha256 and artifacts.json" \
  $([ "$rc" = 0 ] && [ -f "$T/set/pai-pd-v0.1.20.zip" ] && [ -f "$T/set/umarell-$u12.tar.gz" ] \
     && [ "$(cut -d' ' -f1 "$T/set/pai-pd-v0.1.20.zip.sha256")" = "$(sha "$T/export.zip")" ] \
     && python3 -c 'import json, sys; d = json.load(open(sys.argv[1])); sys.exit(d != {"source": "pai-pd-v0.1.20.zip", "umarell": sys.argv[2]})' \
          "$T/set/artifacts.json" "umarell-$u12.tar.gz"; echo $?) "$out"
cp "$T/set/"* "$T/s3/inputs-bkt/"; put "$T/export.zip" inputs-bkt/factoryos-old.zip
out=$(build); rc=$?
check "make-artifacts.sh: the uploaded set installs (artifacts.json wins over a stray zip; both .sha256 checked)" \
  $([ "$rc" = 0 ] && grep -q 'sha256 ok: s3://inputs-bkt/pai-pd-v0.1.20.zip' <<<"$out" && grep -q "sha256 ok: s3://inputs-bkt/umarell-$u12.tar.gz" <<<"$out" \
     && env_has "ONECLICK_UMARELL_TARBALL=s3://inputs-bkt/umarell-$u12.tar.gz" && [ "$(signal_field Status)" = SUCCESS ]; echo $?) "$out"
out=$(bash "$dep/make-artifacts.sh" "$T/readme" --out "$T/set2" 2>&1); rc=$?
check "make-artifacts.sh: refuses a file that is not a .zip or .tar.gz" $([ "$rc" != 0 ] && grep -q 'must be a .zip or .tar.gz' <<<"$out"; echo $?) "$out"
out=$(bash "$dep/make-artifacts.sh" "$T/other.tar.gz" --out "$T/set2" 2>&1); rc=$?
check "make-artifacts.sh: refuses an archive without source/VERSION.json" $([ "$rc" != 0 ] && grep -q 'VERSION.json' <<<"$out"; echo $?) "$out"
out=$(bash "$dep/make-artifacts.sh" "$T/export.tar.gz" --out "$T/set3" 2>&1); rc=$?
check "make-artifacts.sh: a .tar.gz export, no umarell: artifacts.json names the source only" \
  $([ "$rc" = 0 ] && [ -f "$T/set3/pai-pd-v0.1.20.tar.gz" ] && ! grep -q umarell "$T/set3/artifacts.json"; echo $?) "$out"
reset; export WAIT_HANDLE_URL=https://handle.example.com/h
out=$(bash "$dep/make-artifacts.sh" "$T/export.zip" --umarell "$T/umarell.tar.gz" --umarell-jam "$T/umarell-jam.tar.gz" --out "$T/set5" 2>&1); rc=$?
j12=$(sha "$T/umarell-jam.tar.gz" | cut -c1-12)
check "make-artifacts.sh --umarell-jam: umarell-jam-<sha12>.tar.gz, its .sha256, artifacts.json's umarellJam" \
  $([ "$rc" = 0 ] && [ -f "$T/set5/umarell-jam-$j12.tar.gz" ] && [ "$(cut -d' ' -f1 "$T/set5/umarell-jam-$j12.tar.gz.sha256")" = "$(sha "$T/umarell-jam.tar.gz")" ] \
     && python3 -c 'import json, sys; d = json.load(open(sys.argv[1])); sys.exit(d != {"source": "pai-pd-v0.1.20.zip", "umarell": sys.argv[2], "umarellJam": sys.argv[3]})' \
          "$T/set5/artifacts.json" "umarell-$u12.tar.gz" "umarell-jam-$j12.tar.gz"; echo $?) "$out"
cp "$T/set5/"* "$T/s3/inputs-bkt/"
out=$(build); rc=$?
check "make-artifacts.sh --umarell-jam: the uploaded set installs with both archives (both .sha256 checked)" \
  $([ "$rc" = 0 ] && grep -q "sha256 ok: s3://inputs-bkt/umarell-jam-$j12.tar.gz" <<<"$out" \
     && env_has "ONECLICK_UMARELL_TARBALL=s3://inputs-bkt/umarell-$u12.tar.gz" && env_has "ONECLICK_UMARELL_JAM_TARBALL=s3://inputs-bkt/umarell-jam-$j12.tar.gz"; echo $?) "$out"

mk_tree "$T/dev" pai-pd-dev; printf '{"format":1,"tree":"1111111111111111111111111111111111111111"}\n' > "$T/dev/pai-pd-dev/source/VERSION.json"
tar -czf "$T/dev.tar.gz" -C "$T/dev" pai-pd-dev
out=$(bash "$dep/make-artifacts.sh" "$T/dev.tar.gz" --out "$T/set4" 2>&1); rc=$?
check "make-artifacts.sh: an export of a commit that is no release is named dev-<tree, 12 hex>" \
  $([ "$rc" = 0 ] && [ -f "$T/set4/pai-pd-dev-111111111111.tar.gz" ]; echo $?) "$out"

# --- the custom resource function --------------------------------------------------------------------------------------
out=$(python3 - "$T/index.py" <<'EOF' 2>&1
import importlib.util, json, sys, types
calls, responses = [], []
class CB:
    running = []
    def list_builds_for_project(self, **k): calls.append(("list", k)); return {"ids": [b["id"] for b in CB.running] or []}
    def batch_get_builds(self, ids): return {"builds": [b for b in CB.running if b["id"] in ids]}
    def stop_build(self, id): calls.append(("stop", id))
    def start_build(self, **k): calls.append(("start", k)); return {"build": {"id": "proj:42"}}
class CF:
    def describe_stacks(self, StackName):
        calls.append(("describe", StackName))
        return {"Stacks": [{"Outputs": [{"OutputKey": "Url", "OutputValue": "https://x.cloudfront.net/"},
                                        {"OutputKey": "GateVerified", "OutputValue": "true"},
                                        {"OutputKey": "InstanceId", "OutputValue": "i-1"}]}]}
class Objs:
    def __init__(s, b): s.b = b
    def all(s): return s
    def delete(s):
        calls.append(("empty", s.b))
        if s.b == "boom": raise RuntimeError("AccessDenied")
class S3:
    def Bucket(s, b): return types.SimpleNamespace(objects=Objs(b))
boto3 = types.ModuleType("boto3")
boto3.client = lambda n: {"codebuild": CB(), "cloudformation": CF()}[n]
boto3.resource = lambda n: S3()
sys.modules["boto3"] = boto3
spec = importlib.util.spec_from_file_location("index", sys.argv[1]); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
import urllib.request
urllib.request.urlopen = lambda req, timeout: responses.append((req.get_method(), req.headers, json.loads(req.data)))
def ev(kind, **props):
    return {"RequestType": kind, "ResponseURL": "https://cfn-response.example.com/r", "StackId": "s", "RequestId": "r",
            "LogicalResourceId": "StartBuild", "ResourceProperties": props}
def last(): return responses[-1][2]
fails = []
def expect(name, cond):
    print(("ok    " if cond else "FAIL  ") + name)
    if not cond: fails.append(name)
sb = dict(Mode="start-build", ProjectName="proj", WaitHandle="https://handle/h")
m.handler(ev("Create", **sb), None)
start = [c for c in calls if c[0] == "start"][-1][1]
expect("function: Create starts the build with WAIT_HANDLE_URL", start["environmentVariablesOverride"] == [{"name": "WAIT_HANDLE_URL", "value": "https://handle/h", "type": "PLAINTEXT"}])
expect("function: Create answers SUCCESS with the build id, PUT with an empty Content-Type",
       last()["Status"] == "SUCCESS" and last()["Data"] == {"BuildId": "proj:42"} and responses[-1][0] == "PUT" and responses[-1][1].get("Content-type") == "")
calls.clear(); m.handler(ev("Update", **sb), None)
start = [c for c in calls if c[0] == "start"][-1][1]
expect("function: Update starts a build without the wait handle", start["environmentVariablesOverride"] == [] and last()["Status"] == "SUCCESS")
CB.running = [{"id": "proj:41", "buildStatus": "IN_PROGRESS"}]
calls.clear(); m.handler(ev("Update", **sb), None)
expect("function: Update while a build runs: FAILED, no new build", last()["Status"] == "FAILED" and "still running" in last()["Reason"] and not [c for c in calls if c[0] == "start"])
calls.clear(); m.handler(ev("Delete", **sb), None)
expect("function: Delete stops the running build and answers SUCCESS", ("stop", "proj:41") in calls and last()["Status"] == "SUCCESS")
CB.running = []
calls.clear(); m.handler(ev("Delete", **sb), None)
expect("function: Delete starts nothing", not [c for c in calls if c[0] in ("start", "stop")] and last()["Status"] == "SUCCESS")
m.handler(ev("Create", Mode="read-outputs", AppStackName="pai-pd"), None)
expect("function: read-outputs returns Url and GateVerified only", last()["Data"] == {"Url": "https://x.cloudfront.net/", "GateVerified": "true"})
calls.clear(); m.handler(ev("Delete", Mode="read-outputs", AppStackName="pai-pd"), None)
expect("function: read-outputs on Delete reads nothing", not calls and last()["Status"] == "SUCCESS")
calls.clear(); m.handler(ev("Create", Mode="empty-bucket", Bucket="b"), None)
expect("function: empty-bucket empties nothing on Create", not calls and last()["Status"] == "SUCCESS")
m.handler(ev("Delete", Mode="empty-bucket", Bucket="b"), None)
expect("function: empty-bucket empties the bucket on Delete", ("empty", "b") in calls and last()["Status"] == "SUCCESS")
m.handler(ev("Delete", Mode="empty-bucket", Bucket="boom"), None)
expect("function: an error on Delete never stalls the delete (SUCCESS, the error in the reason)", last()["Status"] == "SUCCESS" and "AccessDenied" in last()["Reason"])
m.handler(ev("Create", Mode="nonsense"), None)
expect("function: an unknown mode answers (SUCCESS, nothing done)", last()["Status"] == "SUCCESS")
m.handler({**ev("Create"), "ResourceProperties": {}}, None)
expect("function: a missing Mode on Create answers FAILED", last()["Status"] == "FAILED")
sys.exit(1 if fails else 0)
EOF
); rc=$?
printf '%s\n' "$out"
pass=$((pass + $(grep -c '^ok ' <<<"$out"))); fail=$((fail + $(grep -c '^FAIL' <<<"$out")))
[ "$rc" = 0 ] || [ "$(grep -c '^FAIL' <<<"$out")" -gt 0 ] || { fail=$((fail+1)); echo "FAIL  the function's tests did not run"; }

echo
echo "$pass passed, $fail failed"
[ "$fail" = 0 ]
