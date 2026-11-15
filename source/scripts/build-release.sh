#!/usr/bin/env bash
#
# scripts/build-release.sh: the Factory OS release build of the public deployment (deployment/CONTRACT.md), natively
# on its instance, in an unpacked export (or a checkout of this repo). The build of ci/build-in-container.sh, which CI
# runs on every merge and which public-export:verify-build runs on every export, minus what a deployment does not
# need by default:
#   1. rerun at third_party/rerun/PIN with our patches (scripts/fetch-rerun.sh; from GitHub)
#   2. factoryos-backend (no Rerun in its tree: scripts/check-backend-no-rerun.sh), factoryos-terminal-agent,
#      factoryos-mcp
#   3. cell-rerun-server with cell-runner (Coal collision checking) and its libraries embedded
#      (cell/scripts/build-cell-rerun-server.sh)
#   4. plan2cell (its own workspace and lock) with --features sim,cell
#   5. the wasm viewer into web/dist (scripts/build-web.sh)
# then out/build-release.info, which scripts/package-bundle.sh puts into the bundle's BUILD-INFO.txt.
#
# Env:
#   FACTORYOS_BUILD_ID       the release (default: VERSION.json's, else "dev")
#   FACTORYOS_BUILD_TESTS=1  also run the test suites ci/build-in-container.sh runs (an hour more); default: none
#   FACTORYOS_FULL_PROFILE=1 the release profiles as committed (LTO, one codegen unit: slower to build, somewhat
#                            faster binaries); default: no LTO and 16 codegen units, as CI's merge request builds
#   CARGO_BUILD_JOBS         default: one job per ~4 GiB of RAM, at most one per vCPU (as ci/aws/instance-build.sh;
#                            the deployment's 8 vCPU / 32 GiB instance gets 7; it has 16 GiB of swap besides)
# Needs the toolchain of deployment/bootstrap/install-build-deps.sh (or infra/docker/app/Dockerfile), and network
# access to GitHub, crates.io and the npm registry. Works in the exported tree: no crate's `internal` feature is
# named, and every cargo call is --locked.
set -euo pipefail
# Build products readable (and binaries executable) by others whatever the caller's umask; scripts/package-bundle.sh
# sets the bundle's modes itself too.
umask 022

root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"

build_id=${FACTORYOS_BUILD_ID:-}
if [ -z "$build_id" ] && [ -f VERSION.json ]; then
  build_id=$(python3 -I -c 'import json, sys; print(json.load(open(sys.argv[1])).get("release") or "")' VERSION.json)
fi
build_id=${build_id:-dev}
tests=${FACTORYOS_BUILD_TESTS:-0}

step() { echo; echo "=== $(date -u +%FT%TZ) build-release: $* ==="; }
run() { echo "+ $*"; "$@"; }

profile="release"
if [ "${FACTORYOS_FULL_PROFILE:-0}" != 1 ]; then
  profile="release, fast (no LTO, 16 codegen units)"
  # Both workspaces' `release` and the wasm viewer's `web-release` (which inherits it), as FAST_BUILD=1 in CI.
  export CARGO_PROFILE_RELEASE_LTO=false CARGO_PROFILE_RELEASE_CODEGEN_UNITS=16
  export CARGO_PROFILE_WEB_RELEASE_LTO=false CARGO_PROFILE_WEB_RELEASE_CODEGEN_UNITS=16
fi

# Parallelism by memory as well as by CPU: ~4 GiB per concurrent rustc (the viewer's and plan2cell's largest crates),
# never more jobs than vCPUs, at least 2.
if [ -z "${CARGO_BUILD_JOBS:-}" ] && [ -r /proc/meminfo ]; then
  mem_gb=$(awk '/^MemTotal:/ {printf "%d", $2/1048576}' /proc/meminfo)
  cpus=$(nproc)
  CARGO_BUILD_JOBS=$(( mem_gb / 4 ))
  [ "$CARGO_BUILD_JOBS" -gt "$cpus" ] && CARGO_BUILD_JOBS=$cpus
  [ "$CARGO_BUILD_JOBS" -lt 2 ] && CARGO_BUILD_JOBS=2
  export CARGO_BUILD_JOBS
fi
export CARGO_TERM_COLOR=never CARGO_TERM_PROGRESS_WHEN=never
echo "build-release: $build_id, profile $profile, tests $([ "$tests" = 1 ] && echo on || echo off)," \
     "cargo jobs ${CARGO_BUILD_JOBS:-default}, $(rustc --version)"

# scripts/build-web.sh needs wasm-bindgen-cli at exactly the wasm-bindgen of Cargo.lock (install-build-deps.sh
# installs the CI image's; a newer lock is caught up here, as scripts/dev-build/box-build.sh does).
wb=$(awk '/^name = "wasm-bindgen"$/ { getline; gsub(/"/, "", $3); print $3; exit }' Cargo.lock)
if [ "$(wasm-bindgen --version 2>/dev/null | awk '{ print $2 }')" != "$wb" ]; then
  step "wasm-bindgen-cli $wb (Cargo.lock)"
  run cargo install --locked wasm-bindgen-cli --version "$wb"
fi

# 1. rerun at the pinned release, with our patches.
step "rerun"
run bash scripts/fetch-rerun.sh
rerun_ref=$(tr -d '[:space:]' < third_party/rerun/PIN)
rerun_sha=$(git -C third_party/rerun/src rev-parse HEAD)

# 2. The backend and the binaries it ships with. Renderer-neutral scenes only: no Rerun in its dependency tree.
step "backend, terminal agent, MCP server"
run bash scripts/check-backend-no-rerun.sh
run cargo build --release --locked -p factoryos-backend -p factoryos-terminal-agent -p factoryos-mcp
if [ "$tests" = 1 ]; then
  run cargo test --release --locked -p factoryos-protocol -p factoryos-client -p factoryos-backend -p factoryos-pty \
    -p factoryos-terminal-agent -p factoryos-urdf-materials -p urdf-robo-dresser -p robo-fitting-room -p openapi-mcp \
    -p factoryos-mcp
  PYTHONDONTWRITEBYTECODE=1 run python3 -m unittest discover -s deploy/planner-router/tests
fi

# 3. The cell runtime (cell/ workspace): cell-rerun-server with cell-runner (Coal collision checking) embedded.
step "cell-rerun-server"
if [ "$tests" = 1 ]; then
  (cd cell && run cargo test --release --locked --workspace --features cell-runner/collision)
fi
run bash cell/scripts/build-cell-rerun-server.sh

# 4. plan2cell (own workspace and lock): the CLI the backend runs for code export and the virtual cell.
step "plan2cell"
if [ "$tests" = 1 ]; then
  (cd plan2cell && run cargo test --release --locked --workspace)
fi
(cd plan2cell && run cargo build --release --locked -p plan2cell-cli --features sim,cell)
if [ "$tests" = 1 ]; then
  # The backend's contract with that CLI (as ci/build-in-container.sh).
  FACTORY_OS_PLAN2CELL="$root/plan2cell/target/release/plan2cell" \
    run cargo test --release --locked -p factoryos-backend --bins -- --ignored real_plan2cell
fi

# 5. The wasm viewer.
step "web viewer"
run bash scripts/build-web.sh

mkdir -p out
{
  echo "build:                  $build_id"
  echo "arch:                   linux-$(case "$(uname -m)" in aarch64|arm64) echo arm64 ;; x86_64) echo x64 ;; *) uname -m ;; esac)"
  echo "rerun:                  $rerun_ref ($rerun_sha)"
  echo "profile:                $profile"
  echo "binaries:               built on $(hostname) $(date -u +%FT%TZ) (scripts/build-release.sh, tests $([ "$tests" = 1 ] && echo run || echo skipped))"
  echo "rustc:                  $(rustc --version)"
} > out/build-release.info
step "done"
cat out/build-release.info
