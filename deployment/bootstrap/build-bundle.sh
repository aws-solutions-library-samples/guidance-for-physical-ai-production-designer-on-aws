#!/bin/bash
#
# build-bundle.sh <export dir> <version> <export sha256> <out dir>
#
# The release build off the instance (deployment/CONTRACT.md "Release build in AWS CodeBuild"): the installer's
# release-build CodeBuild project (pai-pd-installer.yaml, <installer stack>-build: Arm, Ubuntu 24.04, as root) runs it
# from the export it builds, unpacked in <export dir>. The same phases factoryos-release.sh runs on the instance, on
# the same OS and architecture:
#   deps        deployment/bootstrap/install-build-deps.sh (the instance's toolchain)
#   building    <tree>/scripts/build-release.sh, FACTORYOS_BUILD_ID=<version>
#   packaging   <tree>/scripts/package-bundle.sh <out dir>/bundle.tar.gz
# and writes <out dir>/bundle.json: {"version", "exportSha256", "sha256" (the bundle's), "arch", "builtBy"}. install.sh
# hands that bundle to the instance, whose factoryos-release deploys it instead of building (it still downloads and
# unpacks the export: deploy/factoryos-deploy.sh, provisioning and the checks are the export's).
# <tree>, the build root: <export dir>/source in the Guidance layout, else <export dir> (factoryos-release.sh's rule).
set -euo pipefail
umask 022

src="${1:?usage: build-bundle.sh <export dir> <version> <export sha256> <out dir>}"
version="${2:?}" export_sha="${3:?}" out="${4:?}"
say() { echo "build-bundle: $*"; }
fail() { echo "build-bundle: ERROR: $*" >&2; exit 1; }

[[ "$version" =~ ^[A-Za-z0-9._+-]{1,64}$ ]] || fail "the version '$version' is not [A-Za-z0-9._+-]{1,64}"
[[ "$export_sha" =~ ^[0-9a-f]{64}$ ]] || fail "the export sha256 is not 64 lower-case hex digits"
[ "$(id -u)" = 0 ] || fail "runs as root (install-build-deps.sh installs the toolchain)"
[ "$(uname -m)" = aarch64 ] || fail "builds the linux-arm64 bundle on arm64 (this is $(uname -m))"
# shellcheck disable=SC1091
[ "$(. /etc/os-release && echo "$ID $VERSION_ID")" = "ubuntu 24.04" ] \
  || fail "builds on Ubuntu 24.04, the instance's OS (this is $(. /etc/os-release && echo "$PRETTY_NAME"))"
if [ -f "$src/source/scripts/build-release.sh" ]; then tree="$src/source"; else tree="$src"; fi
[ -f "$tree/scripts/build-release.sh" ] && [ -f "$tree/scripts/package-bundle.sh" ] \
  || fail "$src has no scripts/build-release.sh and scripts/package-bundle.sh (nor under source/)"
mkdir -p "$out"
out=$(cd "$out" && pwd)
export DEBIAN_FRONTEND=noninteractive NEEDRESTART_SUSPEND=1
start=$(date +%s)
phase() { say "$1 ($(( ($(date +%s) - start) / 60 )) min)"; }

phase "deps: install-build-deps.sh"
bash "$src/deployment/bootstrap/install-build-deps.sh"
# The toolchain's environment (install-build-deps.sh writes it).
# shellcheck disable=SC1091
. /etc/profile.d/factoryos-build.sh

phase "building: scripts/build-release.sh $version"
(cd "$tree" && FACTORYOS_BUILD_ID="$version" bash scripts/build-release.sh)

phase "packaging: scripts/package-bundle.sh"
rm -f "$out/bundle.tar.gz" "$out/bundle.json"
(cd "$tree" && bash scripts/package-bundle.sh "$out/bundle.tar.gz")
sha=$(sha256sum "$out/bundle.tar.gz" | cut -d' ' -f1)
printf '{"version":"%s","exportSha256":"%s","sha256":"%s","arch":"linux-arm64","builtBy":"%s"}\n' \
  "$version" "$export_sha" "$sha" "${CODEBUILD_BUILD_ARN:-$(hostname)}" > "$out/bundle.json"
phase "done: bundle.tar.gz $(( $(wc -c < "$out/bundle.tar.gz") / 1048576 )) MB, sha256 $sha"
