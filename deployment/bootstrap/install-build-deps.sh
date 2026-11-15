#!/bin/bash
#
# install-build-deps.sh [--stamp]
#
# The build toolchain of the public deployment's instance (deployment/CONTRACT.md), natively on its Ubuntu 24.04:
# what the CI deps image infra/docker/app/Dockerfile has, at the same versions (keep them in step, as
# scripts/dev-build/box-setup.sh does), so scripts/build-release.sh builds the release the way CI builds it:
#   - the apt packages the image installs (+ unzip),
#   - Coal and Pinocchio from source into /usr/local (cell/: cell-runner's C++ wrapper, collision checking),
#   - node (zenoh-ts bundle of the web viewer), binaryen's wasm-opt,
#   - rustup with the toolchain(s) of rust-toolchain.toml (and plan2cell/'s) and the wasm32 target; both are read from
#     the export's build root: source/ in the Guidance layout (deployment/ at the root, the rest under source/), else
#     the export's root (the flat layout),
#   - wasm-bindgen-cli at the image's version (scripts/build-release.sh installs another one when Cargo.lock asks).
# Not sccache: the instance builds once per release and deletes its target dirs afterwards.
#
# Run as root by factoryos-release (deployment/bootstrap/factoryos-release.sh) in its `deps` phase, from the export
# being released. Idempotent: every step is skipped when what it installs is already there at its version, so a run
# cut short (a reboot) is finished by the next. At the end it records its stamp (`--stamp` prints it: the sha256 of
# this file and the toolchain files it reads, then the toolchain channels) in /var/lib/factoryos-release/deps.sha256;
# factoryos-release skips the phase while the stamp matches. /etc/profile.d/factoryos-build.sh is the build
# environment (the image's ENV).
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
# The build root: source/ in the Guidance layout, else the root (factoryos-release.sh chooses the same way).
if [ -f "$root/source/rust-toolchain.toml" ]; then tree="$root/source"; else tree="$root"; fi
[ -f "$tree/rust-toolchain.toml" ] \
  || { echo "install-build-deps.sh: no rust-toolchain.toml in $root/source or $root" >&2; exit 1; }
toolchains=("$tree/rust-toolchain.toml")
[ -f "$tree/plan2cell/rust-toolchain.toml" ] && toolchains+=("$tree/plan2cell/rust-toolchain.toml")
channels=$(sed -n 's/^channel *= *"\([^"]*\)".*/\1/p' "${toolchains[@]}" | sort -u)
[ -n "$channels" ] || { echo "install-build-deps.sh: no channel in ${toolchains[*]}" >&2; exit 1; }

# The stamp: one line, "<sha256> rust <channel>[,<channel>…]".
stamp() {
  local sum
  sum=$(cat "$0" "${toolchains[@]}" | sha256sum | cut -d' ' -f1)
  echo "$sum rust $(printf '%s\n' "$channels" | paste -sd, -)"
}
if [ "${1:-}" = --stamp ]; then stamp; exit 0; fi

R=${FACTORYOS_ROOT:-}
state="$R/var/lib/factoryos-release"

# The versions of infra/docker/app/Dockerfile (its ARGs).
WASM_BINDGEN_VERSION=0.2.126
BINARYEN_VERSION=133
BINARYEN_SHA256_X64=2dc9c7813f5375db93d96ead4b78222fcc3e2677bbb832297af4797782a37489
BINARYEN_SHA256_ARM64=89c07ea56faf38d0fbecf36ca8ec0721756716185f265b568e133d427f299bf8
NODE_VERSION=20.20.2
NODE_SHA256_X64=df770b2a6f130ed8627c9782c988fda9669fa23898329a61a871e32f965e007d
NODE_SHA256_ARM64=73093db209e4e9e09dd7d15a47aeaab1b74833830df03efa5f942a1122c5fa71
COAL_VERSION=v3.0.1
PINOCCHIO_VERSION=v3.9.0

log() { echo "[build-deps] $*"; }
[ "$(id -u)" = 0 ] || { echo "install-build-deps.sh: run as root" >&2; exit 1; }
case "$(dpkg --print-architecture)" in
  arm64) node_arch=arm64; node_sha=$NODE_SHA256_ARM64; b_arch=aarch64; b_sha=$BINARYEN_SHA256_ARM64 ;;
  amd64) node_arch=x64;   node_sha=$NODE_SHA256_X64;   b_arch=x86_64;  b_sha=$BINARYEN_SHA256_X64 ;;
  *) echo "install-build-deps.sh: unsupported architecture $(dpkg --print-architecture)" >&2; exit 1 ;;
esac
export DEBIAN_FRONTEND=noninteractive NEEDRESTART_SUSPEND=1

# apt-get <args>, waiting for the apt/dpkg locks (deploy/provision.sh's helper): at boot SSM Patch Manager,
# unattended-upgrades or cloud-init can hold them for minutes, and apt-get fails at once on them.
# Waits up to FACTORYOS_APT_LOCK_WAIT seconds (default 900); any other failure fails as before.
apt_get() {
  local out rc deadline=$(( $(date +%s) + ${FACTORYOS_APT_LOCK_WAIT:-900} ))
  while :; do
    rc=0
    out=$(apt-get -o DPkg::Lock::Timeout=60 "$@" 2>&1) || rc=$?
    if [ "$rc" = 0 ]; then
      [ -z "$out" ] || printf '%s\n' "$out"
      return 0
    fi
    if grep -qE 'Could not get lock|Unable to lock|Unable to acquire the dpkg frontend lock' <<<"$out" \
       && [ "$(date +%s)" -lt "$deadline" ]; then
      log "apt is locked ($(pgrep -af 'apt|dpkg|unattended-upgr|main_entrance' 2>/dev/null | head -3 | tr '\n' ';' || true)); waiting"
      sleep 10
      continue
    fi
    printf '%s\n' "$out" >&2
    return "$rc"
  done
}
# fetch <url> <file>: a download with retries (a NAT gateway's first minutes, GitHub's occasional 5xx).
fetch() {
  curl --fail --location --show-error --silent --proto '=https' --tlsv1.2 \
    --retry 5 --retry-delay 5 --retry-all-errors -o "$2" "$1"
}

log "packages"
apt_get update -y -qq
# As the deps image (unzip: for the AWS CLI's installer and the release's zip exports).
apt_get install -y -qq --no-install-recommends \
  build-essential ca-certificates clang cmake curl file git libssl-dev pkg-config xz-utils zip unzip \
  libeigen3-dev libboost-all-dev \
  liburdfdom-dev libconsole-bridge-dev libtinyxml-dev libtinyxml2-dev \
  libassimp-dev liboctomap-dev libqhull-dev >/dev/null

export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/local/share/pkgconfig CMAKE_PREFIX_PATH=/usr/local
# ~4 GiB per job, at most one per vCPU (as the Dockerfile).
jobs=$(awk '/MemTotal/{m=int($2/1024/1024/4)} END{n=m; ("nproc"|getline p); if(n>p)n=p; if(n<1)n=1; print n}' /proc/meminfo)

# Coal (formerly hpp-fcl), Pinocchio's collision backend; Pinocchio with collision and URDF, no Python.
if [ "$(pkg-config --modversion coal 2>/dev/null)" != "${COAL_VERSION#v}" ]; then
  log "coal $COAL_VERSION"
  rm -rf /tmp/coal
  git clone --quiet --branch "$COAL_VERSION" --depth 1 --recursive https://github.com/coal-library/coal.git /tmp/coal
  cmake -S /tmp/coal -B /tmp/coal/build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_PYTHON_INTERFACE=OFF -DCOAL_BUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_TESTING=OFF -DCOAL_HAS_QHULL=ON \
    -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON
  cmake --build /tmp/coal/build -j"$jobs"
  cmake --install /tmp/coal/build
  rm -rf /tmp/coal
fi
if [ "$(pkg-config --modversion pinocchio 2>/dev/null)" != "${PINOCCHIO_VERSION#v}" ]; then
  log "pinocchio $PINOCCHIO_VERSION"
  rm -rf /tmp/pinocchio
  git clone --quiet --branch "$PINOCCHIO_VERSION" --depth 1 --recursive https://github.com/stack-of-tasks/pinocchio.git /tmp/pinocchio
  cmake -S /tmp/pinocchio -B /tmp/pinocchio/build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_TESTING=OFF \
    -DBUILD_WITH_COLLISION_SUPPORT=ON -DBUILD_WITH_URDF_SUPPORT=ON \
    -DBUILD_WITH_CASADI_SUPPORT=OFF -DBUILD_WITH_AUTODIFF_SUPPORT=OFF
  cmake --build /tmp/pinocchio/build -j"$jobs"
  cmake --install /tmp/pinocchio/build
  rm -rf /tmp/pinocchio
fi
ldconfig

# Node (zenoh-ts bundle of the web viewer; scripts/build-web.sh also reads cargo metadata with it).
if [ "$(/usr/local/bin/node --version 2>/dev/null)" != "v$NODE_VERSION" ]; then
  log "node $NODE_VERSION"
  fetch "https://nodejs.org/dist/v${NODE_VERSION}/node-v${NODE_VERSION}-linux-${node_arch}.tar.xz" /tmp/node.tar.xz
  echo "${node_sha}  /tmp/node.tar.xz" | sha256sum -c - >/dev/null
  tar -xJf /tmp/node.tar.xz -C /usr/local --strip-components=1 \
    --exclude CHANGELOG.md --exclude LICENSE --exclude README.md
  rm -f /tmp/node.tar.xz
fi

# binaryen (wasm-opt); the distro's is years older than the wasm features the viewer uses.
if ! /usr/local/bin/wasm-opt --version 2>/dev/null | grep -q "version $BINARYEN_VERSION"; then
  log "binaryen $BINARYEN_VERSION"
  fetch "https://github.com/WebAssembly/binaryen/releases/download/version_${BINARYEN_VERSION}/binaryen-version_${BINARYEN_VERSION}-${b_arch}-linux.tar.gz" /tmp/binaryen.tar.gz
  echo "${b_sha}  /tmp/binaryen.tar.gz" | sha256sum -c - >/dev/null
  tar -xzf /tmp/binaryen.tar.gz -C /tmp
  install -m 0755 "/tmp/binaryen-version_${BINARYEN_VERSION}/bin/wasm-opt" /usr/local/bin/wasm-opt
  cp -a "/tmp/binaryen-version_${BINARYEN_VERSION}/lib/." /usr/local/lib/ 2>/dev/null || true
  ldconfig
  rm -rf /tmp/binaryen.tar.gz "/tmp/binaryen-version_${BINARYEN_VERSION}"
fi

# Rust exactly as the rust-toolchain.toml files ask (so rustup never downloads at build time), with the wasm32 target.
export RUSTUP_HOME=/usr/local/rustup CARGO_HOME=/usr/local/cargo
export PATH=/usr/local/cargo/bin:$PATH
first=$(printf '%s\n' "$channels" | head -1)
if [ ! -x "$CARGO_HOME/bin/rustup" ]; then
  log "rustup ($first)"
  fetch https://sh.rustup.rs /tmp/rustup-init.sh
  sh /tmp/rustup-init.sh -y --no-modify-path --profile minimal --default-toolchain "$first" \
    --component clippy,rustfmt --target wasm32-unknown-unknown
  rm -f /tmp/rustup-init.sh
fi
for channel in $channels; do
  rustup toolchain install "$channel" --profile minimal --component clippy,rustfmt --target wasm32-unknown-unknown
done
rustup default "$first" >/dev/null

# wasm-bindgen-cli: must match the wasm-bindgen crate of Cargo.lock exactly (scripts/build-web.sh checks it).
if [ "$(wasm-bindgen --version 2>/dev/null | awk '{ print $2 }')" != "$WASM_BINDGEN_VERSION" ]; then
  log "wasm-bindgen-cli $WASM_BINDGEN_VERSION"
  (cd / && cargo +"$first" install --locked --quiet wasm-bindgen-cli --version "$WASM_BINDGEN_VERSION")
fi
rm -rf "$CARGO_HOME/registry/cache"

# The deps image's environment, for scripts/build-release.sh (factoryos-release sources it) and `sudo -i` over SSM.
cat > /etc/profile.d/factoryos-build.sh <<'EOF'
export RUSTUP_HOME=/usr/local/rustup CARGO_HOME=/usr/local/cargo
export PATH=/usr/local/cargo/bin:$PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/local/share/pkgconfig CMAKE_PREFIX_PATH=/usr/local
export LD_LIBRARY_PATH=/usr/local/lib
EOF

# Self-check, as the Dockerfile's.
for c in bash cargo clang cmake curl file git node npm pkg-config readelf unzip wasm-bindgen wasm-opt zip; do
  command -v "$c" >/dev/null || { echo "install-build-deps.sh: missing $c" >&2; exit 1; }
done
rustup target list --installed | grep -qx wasm32-unknown-unknown
pkg-config --exists pinocchio coal || { echo "install-build-deps.sh: pinocchio/coal not visible to pkg-config" >&2; exit 1; }
log "pinocchio $(pkg-config --modversion pinocchio) / coal $(pkg-config --modversion coal) / $(cargo --version)" \
    "/ $(wasm-bindgen --version) / $(wasm-opt --version) / node $(node --version)"

mkdir -p "$state"
stamp > "$state/deps.sha256"
log "done $(date -u +%FT%TZ)"
