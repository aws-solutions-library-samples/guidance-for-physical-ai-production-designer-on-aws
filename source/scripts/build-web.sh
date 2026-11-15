#!/usr/bin/env bash
# Builds the wasm viewer into web/dist/, which factoryos-backend serves with `--web-root web/dist`.
#
#   scripts/build-web.sh            # web-release profile (lto, size-optimized, wasm-opt)
#   scripts/build-web.sh --debug    # dev profile, no wasm-opt (much faster)
#
# Needs: the wasm32-unknown-unknown target (rust-toolchain.toml), wasm-bindgen-cli at exactly the
# version of the wasm-bindgen crate in Cargo.lock, node/npm, and wasm-opt (binaryen) for release.
# The rerun patches must be applied to ../rerun first (scripts/build.sh --sync-only does that).
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"

profile=web-release
if [ "${1:-}" = --debug ]; then
    profile=dev
fi
out="$root/web/dist"
crate=factoryos_viewer

# wasm-bindgen's CLI and the crate must match exactly, or the generated JS doesn't fit the wasm.
want=$(awk '/^name = "wasm-bindgen"$/ { getline; gsub(/"/, "", $3); print $3; exit }' Cargo.lock)
have=$(wasm-bindgen --version 2>/dev/null | awk '{ print $2 }' || true)
if [ "$have" != "$want" ]; then
    echo "wasm-bindgen-cli $want is needed (found: ${have:-none}):" >&2
    echo "  cargo install wasm-bindgen-cli --locked --version $want" >&2
    exit 1
fi

echo "=== cargo build ($profile, wasm32) ==="
cargo build -p "$crate" --lib --target wasm32-unknown-unknown --profile "$profile"
target_dir=$(cargo metadata --format-version 1 --no-deps | node -e 'process.stdout.write(JSON.parse(require("fs").readFileSync(0)).target_directory)')
profile_dir=$profile
[ "$profile" = dev ] && profile_dir=debug
wasm="$target_dir/wasm32-unknown-unknown/$profile_dir/$crate.wasm"

rm -rf "$out"
mkdir -p "$out/pkg" "$out/vendor"

echo "=== wasm-bindgen ==="
# --target web: ES modules, so js/zenoh_bridge.js's bare zenoh-ts import resolves via the import map.
wasm-bindgen "$wasm" --target web --no-typescript --out-dir "$out/pkg" --out-name "$crate"

if [ "$profile" = web-release ]; then
    echo "=== wasm-opt ==="
    # The same feature set as .cargo/config.toml's target-feature flags (and rerun's web viewer).
    wasm-opt "$out/pkg/${crate}_bg.wasm" -O2 --output "$out/pkg/${crate}_bg.wasm" \
        --enable-reference-types --enable-simd --enable-bulk-memory \
        --enable-nontrapping-float-to-int --enable-multivalue --vacuum --strip-debug
fi

echo "=== zenoh-ts bundle ==="
npm ci --prefix web --no-audit --no-fund --silent
node web/bundle-zenoh-ts.mjs web/node_modules "$out/vendor/zenoh-ts.js"

cp web/index.html "$out/"
cp crates/factoryos-viewer/assets/app_icon.png "$out/"
# The loader's progress bar needs the uncompressed size: served compressed, Content-Length is not it.
wasm_bytes=$(wc -c < "$out/pkg/${crate}_bg.wasm" | tr -d ' ')
printf '{"wasm":"%s_bg.wasm","bytes":%s}\n' "$crate" "$wasm_bytes" > "$out/pkg/manifest.json"

echo "=== web viewer in $out ==="
ls -l "$out" "$out/pkg"
