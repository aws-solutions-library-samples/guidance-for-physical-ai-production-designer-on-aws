#!/usr/bin/env bash
# Builds cell-rerun-server with its backend embedded: cell-runner (with Coal collision checking) and
# the shared libraries it loads (Pinocchio, Coal, urdfdom, boost, …), so the single binary runs on a
# host without any of them installed.
#
#   cell/scripts/build-cell-rerun-server.sh                 # -> cell/target/release/cell-rerun-server
#   cell/scripts/build-cell-rerun-server.sh --no-collision  # dev only: cell-runner without Coal
#
# Needs Pinocchio + Coal discoverable by pkg-config (the infra `cell` image has them).
# On Linux the libraries are collected with ldd and the archive is gated before it is embedded:
# every NEEDED soname must be in it (or be part of glibc), and nothing may need a glibc newer than
# GLIBC_FLOOR (default 2.39, Ubuntu 24.04), the oldest host the bundle promises to run on.
# On macOS (development) the libraries are not bundled: the binary loads them from where Homebrew
# put them, so the result only runs on the machine that built it.
#
# Ported from the Kiro extension bundle's build-release.sh (stage_cell_rerun_supervisor).
set -euo pipefail

cell=$(cd "$(dirname "$0")/.." && pwd)
GLIBC_FLOOR="${GLIBC_FLOOR:-2.39}"
features=(--features cell-runner/collision)
if [ "${1:-}" = --no-collision ]; then
    echo "WARNING: building cell-runner WITHOUT collision checking: trajectories load as UNCHECKED," >&2
    echo "         and preview_trajectory is unavailable. Not for releases." >&2
    features=()
fi
log() { echo "[cell] $*" >&2; }

cd "$cell"
log "building cell-runner (release)"
cargo build --release --locked -p cell-runner "${features[@]}"
cell_bin="$cell/target/release/cell-runner"

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/bin" "$work/lib"
install -m 0755 "$cell_bin" "$work/bin/cell-runner"

# glibc itself and the loader come from the host; everything else must be in lib/.
host_provided_soname() {
    case "$1" in
        ld-linux*|ld.so.*|linux-vdso.so.*)                              return 0 ;;
        libc.so.6|libm.so.6|libdl.so.2|libpthread.so.0)                 return 0 ;;
        librt.so.1|libresolv.so.2|libutil.so.1|libnsl.so.1|libanl.so.1) return 0 ;;
    esac
    return 1
}

bundle_elf_files() {
    local f
    while IFS= read -r f; do
        file -b "$f" 2>/dev/null | grep -q '^ELF' && printf '%s\n' "$f"
    done < <(find "$work/bin" "$work/lib" -type f 2>/dev/null)
}

# Copy every non-glibc library the cell-runner resolves (ldd follows the whole closure).
collect_runtime_libs() {
    local bin="$cell_bin" soname _arrow path _rest copied=0 missing=0
    while read -r soname _arrow path _rest; do
        if [ "$path" = "not" ] || [ -z "$path" ]; then
            case "$soname" in linux-vdso.so*|/lib64/ld-linux*|/lib/ld-linux*) continue ;; esac
            echo "ERROR: $(basename "$bin"): '$soname' could not be resolved" >&2
            missing=$((missing + 1))
            continue
        fi
        host_provided_soname "$(basename "$soname")" && continue
        [ -f "$path" ] && cp -L "$path" "$work/lib/$(basename "$soname")" && copied=$((copied + 1))
    done < <(ldd "$bin" 2>/dev/null)
    log "collected $copied shared library reference(s)"
    [ "$missing" -eq 0 ] || { echo "ERROR: $missing unresolved library reference(s)" >&2; exit 1; }
}

# Every NEEDED of every ELF in the bundle is in lib/ or provided by glibc (readelf, not ldd: ldd
# resolves through this host's ld.so.cache and so passes exactly where it does not matter).
verify_bundle_closure() {
    local f n missing=0 checked=0
    while IFS= read -r f; do
        checked=$((checked + 1))
        while IFS= read -r n; do
            [ -n "$n" ] || continue
            host_provided_soname "$n" && continue
            [ -e "$work/lib/$n" ] && continue
            echo "ERROR: $(basename "$f") needs '$n', which is not in the bundle" >&2
            missing=$((missing + 1))
        done < <(readelf -d "$f" 2>/dev/null | sed -n 's/.*(NEEDED).*\[\(.*\)\]/\1/p')
    done < <(bundle_elf_files)
    [ "$missing" -eq 0 ] || { echo "ERROR: bundle not self-contained ($missing missing)" >&2; exit 1; }
    log "bundle closure verified: $checked ELF file(s)"
}

verify_bundle_glibc_floor() {
    local f worst
    # Numeric compare: GLIBC_2.10 is newer than GLIBC_2.9.
    worst=$(
        while IFS= read -r f; do readelf -V "$f" 2>/dev/null | grep -o 'GLIBC_[0-9][0-9.]*' || true; done \
            < <(bundle_elf_files) |
            sed 's/^GLIBC_//' |
            awk -F. 'NF>=2 { v=$1*1000+$2; if (v>m) { m=v; s=$0 } } END { if (s) print s }'
    )
    [ -n "$worst" ] || return 0
    if awk -v a="$worst" -v b="$GLIBC_FLOOR" 'BEGIN { split(a,x,"."); split(b,y,"."); exit !(x[1]*1000+x[2] > y[1]*1000+y[2]) }'; then
        echo "ERROR: the bundle needs glibc $worst, newer than the floor $GLIBC_FLOOR" >&2
        exit 1
    fi
    log "glibc floor verified: needs <= $worst, floor $GLIBC_FLOOR"
}

case "$(uname -s)" in
    Linux)
        collect_runtime_libs
        verify_bundle_closure
        verify_bundle_glibc_floor
        ;;
    *)
        log "$(uname -s): shared libraries are not bundled (development build for this machine only)"
        ;;
esac

(cd "$work" && zip -r -q backend.zip bin lib)
log "building cell-rerun-server with the backend archive embedded"
CELL_RERUN_BACKEND_ARCHIVE="$work/backend.zip" cargo build --release --locked -p cell-rerun-server

server="$cell/target/release/cell-rerun-server"
"$server" --help 2>&1 | grep -Fq 'backends embedded: true' || {
    echo "ERROR: $server does not report embedded backends" >&2
    exit 1
}
log "ok: $server ($(du -h "$server" | cut -f1), $(find "$work/lib" -type f | wc -l | tr -d ' ') libraries)"
