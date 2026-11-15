#!/usr/bin/env bash
# Build a universal (Apple Silicon + Intel) btgui.app and zip it with the sample trees and plans.
#
#   crates/btgui/scripts/package-macos.sh   → dist/btgui-macos-<version>.zip (at the repository root)
#
# Needs `rustup target add x86_64-apple-darwin`. The app is ad-hoc signed, not notarised.
set -euo pipefail
cd "$(dirname "$0")/../../.."   # the plan2cell repository root (workspace)

version=$(grep -m1 '^version' Cargo.toml | sed 's/.*"\(.*\)"/\1/')
export MACOSX_DEPLOYMENT_TARGET=11.0

cargo build --release -p btgui --target aarch64-apple-darwin
cargo build --release -p btgui --target x86_64-apple-darwin

out=dist/btgui-macos-$version
app=$out/btgui.app
rm -rf "$out" "$out.zip"
mkdir -p "$app/Contents/MacOS"
lipo -create -output "$app/Contents/MacOS/btgui" \
  target/aarch64-apple-darwin/release/btgui \
  target/x86_64-apple-darwin/release/btgui

cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>btgui</string>
  <key>CFBundleDisplayName</key><string>btgui</string>
  <key>CFBundleIdentifier</key><string>dev.btgui</string>
  <key>CFBundleVersion</key><string>$version</string>
  <key>CFBundleShortVersionString</key><string>$version</string>
  <key>CFBundleExecutable</key><string>btgui</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>LSMinimumSystemVersion</key><string>$MACOSX_DEPLOYMENT_TARGET</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>CFBundleDocumentTypes</key>
  <array>
    <dict>
      <key>CFBundleTypeName</key><string>BehaviorTree XML</string>
      <key>CFBundleTypeRole</key><string>Viewer</string>
      <key>CFBundleTypeExtensions</key><array><string>xml</string><string>json</string></array>
    </dict>
  </array>
</dict>
</plist>
PLIST

# A copy inside the app, for the picker of an app run on its own (moved, or translocated by
# Gatekeeper); it is sealed by the signature, so before codesign.
mkdir -p "$app/Contents/Resources"
cp -R test_data "$app/Contents/Resources/examples"
codesign --force --deep --sign - "$app"
# Next to the app, where btgui's startup picker opens (crates/btgui/src/files.rs).
cp -R test_data "$out/examples"
cp crates/btgui/packaging/README.txt "$out/README.txt"
(cd dist && zip -qry "btgui-macos-$version.zip" "btgui-macos-$version")
echo "dist/btgui-macos-$version.zip"
