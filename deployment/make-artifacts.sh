#!/usr/bin/env bash
# Writes an artifact set for the CloudFormation installer (pai-pd-installer.yaml, SourceMode s3 or upload): the
# export archive and, optionally, the umarell archive and umarell_jam's (the plain commit of deploy/umarell-jam.pin),
# each with a <file>.sha256, and an artifacts.json naming them.
# The installer's build checks the .sha256 files and installs exactly the files artifacts.json names.
#
#   deployment/make-artifacts.sh <export.zip|.tar.gz> [--umarell <umarell.tar.gz>] [--umarell-jam <umarell-jam.tar.gz>]
#                                --out <dir | s3://bucket/prefix/>
#
# --out s3://… uploads the set there (the AWS CLI v2 and credentials for that bucket); give the same URI as the
# stack's ArtifactsUri. --out <dir> writes it to a local directory: upload its files to the stack's UploadTo bucket.
set -euo pipefail
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
sha() { if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | cut -d' ' -f1; else shasum -a 256 "$1" | cut -d' ' -f1; fi; }

src="" umarell="" jam="" out=""
while [ $# -gt 0 ]; do
  case "$1" in
    --umarell) umarell="${2:?--umarell needs a file}"; shift 2 ;;
    --umarell-jam) jam="${2:?--umarell-jam needs a file}"; shift 2 ;;
    --out) out="${2:?--out needs a directory or an s3:// URI}"; shift 2 ;;
    -h|--help) usage 0 ;;
    -*) die "unknown option $1 (--help)" ;;
    *) [ -z "$src" ] || die "one export archive only (--help)"; src="$1"; shift ;;
  esac
done
[ -n "$src" ] && [ -n "$out" ] || usage 2
[ -f "$src" ] || die "no such file: $src"
case "$src" in *.zip) ext=zip ;; *.tar.gz|*.tgz) ext=tar.gz ;; *) die "the export must be a .zip or .tar.gz: $src" ;; esac
[ -z "$umarell" ] || [ -f "$umarell" ] || die "no such file: $umarell"
[ -z "$umarell" ] || [[ "$umarell" == *.tar.gz ]] || die "the umarell archive must be a .tar.gz: $umarell"
[ -z "$jam" ] || [ -f "$jam" ] || die "no such file: $jam"
[ -z "$jam" ] || [[ "$jam" == *.tar.gz ]] || die "the umarell_jam archive must be a .tar.gz: $jam"

# The export's version names the set's files: source/VERSION.json's release, else dev-<its tree, 12 hex> (an export of
# a commit that is no release, as install.sh names it), else dev-<the archive's sha256, 12 hex>.
VERSION_OF='import json, sys
v = json.load(sys.stdin)
print(v.get("release") or ("dev-" + v["tree"][:12] if v.get("tree") else ""))'
case "$ext" in
  zip) vj=$(unzip -Z1 "$src" | grep -m1 -E '^[^/]+/source/VERSION\.json$' || true)
       version=$([ -z "$vj" ] || unzip -p "$src" "$vj" | python3 -c "$VERSION_OF") ;;
  *)   vj=$(tar -tzf "$src" | grep -m1 -E '^(\./)?[^/]+/source/VERSION\.json$' || true)
       version=$([ -z "$vj" ] || tar -xzOf "$src" "$vj" | python3 -c "$VERSION_OF") ;;
esac
[ -n "$vj" ] || die "$src has no <top>/source/VERSION.json: is it the guidance's export?"
[ -n "$version" ] || version="dev-$(sha "$src" | cut -c1-12)"
[[ "$version" =~ ^[A-Za-z0-9._+-]+$ ]] || die "unusable release '$version' in $vj"
name="pai-pd-$version.$ext"

stage=$(mktemp -d); trap 'rm -rf "$stage"' EXIT
cp "$src" "$stage/$name"; printf '%s  %s\n' "$(sha "$stage/$name")" "$name" > "$stage/$name.sha256"
uname=""
if [ -n "$umarell" ]; then
  uname="umarell-$(sha "$umarell" | cut -c1-12).tar.gz"
  cp "$umarell" "$stage/$uname"; printf '%s  %s\n' "$(sha "$stage/$uname")" "$uname" > "$stage/$uname.sha256"
fi
jname=""
if [ -n "$jam" ]; then
  jname="umarell-jam-$(sha "$jam" | cut -c1-12).tar.gz"
  cp "$jam" "$stage/$jname"; printf '%s  %s\n' "$(sha "$stage/$jname")" "$jname" > "$stage/$jname.sha256"
fi
python3 -c 'import json, sys
d = {"source": sys.argv[1]}
d.update({"umarell": sys.argv[2]} if sys.argv[2] else {})
d.update({"umarellJam": sys.argv[3]} if sys.argv[3] else {})
print(json.dumps(d, indent=2))' "$name" "$uname" "$jname" > "$stage/artifacts.json"

case "$out" in
  s3://*)
    out="${out%/}/"
    for f in "$stage"/*; do aws s3 cp --only-show-errors "$f" "$out$(basename "$f")"; done
    echo "the artifact set $version is at $out: create the stack with SourceMode s3 and ArtifactsUri $out" ;;
  *)
    mkdir -p "$out"; cp "$stage"/* "$out"/
    echo "the artifact set $version is in $out: upload its files to the stack's UploadTo bucket (SourceMode upload)" ;;
esac
ls -1 "$stage"
