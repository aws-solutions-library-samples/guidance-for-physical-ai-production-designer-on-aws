#!/usr/bin/env bash
# Puts rerun at the pinned release into third_party/rerun/src and applies our patches to it.
# The release is third_party/rerun/PIN (a tag), checked against its commit in third_party/rerun/PIN.commit.
# The workspace's crates depend on it by path; it is not committed (see third_party/rerun/).
#
#   scripts/fetch-rerun.sh                  # clone third_party/rerun/PIN (shallow), apply patches
#   RERUN_SRC=~/src/rerun scripts/fetch-rerun.sh   # use an existing checkout instead (symlinked);
#                                           # it must be at the pinned tag, patches are applied to it
#
# Idempotent: an existing checkout at the pinned tag is kept, and patches already applied are not
# applied again; when the patches change, they are re-applied from the release. A checkout at
# another version, or with local changes of its own, is an error, not silently reused.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
dir="$root/third_party/rerun"
src="$dir/src"
pin=$(tr -d '[:space:]' < "$dir/PIN")
# The commit of that tag (a tag can be moved; the commit cannot): git ls-remote --tags "$url" "$pin"
pin_commit=$(tr -d '[:space:]' < "$dir/PIN.commit")
url=https://github.com/rerun-io/rerun.git

if [ ! -e "$src" ]; then
    if [ -n "${RERUN_SRC:-}" ]; then
        ln -s "$(cd "$RERUN_SRC" && pwd)" "$src"
        echo "rerun: linked $RERUN_SRC"
    else
        echo "rerun: cloning $pin (shallow)"
        # No LFS: only the Rust sources are needed.
        GIT_LFS_SKIP_SMUDGE=1 git clone --quiet --depth 1 --branch "$pin" "$url" "$src"
    fi
fi

# The checkout must be the pinned release (a tag, so `describe --exact-match` finds it).
have=$(git -C "$src" describe --tags --exact-match 2>/dev/null || git -C "$src" rev-parse --short HEAD)
if [ "$have" != "$pin" ]; then
    echo "rerun: $src is at $have, but third_party/rerun/PIN says $pin" >&2
    echo "  remove it and re-run, or check out $pin there" >&2
    exit 1
fi
head_commit=$(git -C "$src" rev-parse HEAD)
if [ "$head_commit" != "$pin_commit" ]; then
    echo "rerun: $src is at commit $head_commit, but third_party/rerun/PIN.commit says $pin_commit" >&2
    echo "  (the $pin tag moved upstream, or PIN.commit was not updated with PIN)" >&2
    exit 1
fi

# The patches are a series (later ones may touch what earlier ones changed), so what counts is
# whether the checkout is the pinned release plus the whole series, recorded in a stamp in its
# git dir once applied.
shopt -s nullglob
patches=("$dir"/patches/*.patch)
series=$(cat "${patches[@]}" /dev/null | git hash-object --stdin)
stamp="$(git -C "$src" rev-parse --absolute-git-dir)/factoryos-patches"

# Is the working tree exactly the checkout plus every patch? (Checked in a scratch index.)
is_patched() {
    local index rc=0
    index=$(mktemp)
    GIT_INDEX_FILE=$index git -C "$src" read-tree HEAD
    for patch in "${patches[@]}"; do
        GIT_INDEX_FILE=$index git -C "$src" apply --cached "$patch" 2>/dev/null || rc=1
    done
    [ $rc = 0 ] && { GIT_INDEX_FILE=$index git -C "$src" diff --quiet || rc=1; }
    rm -f "$index"
    return $rc
}

if [ "$(cat "$stamp" 2>/dev/null)" = "$series" ]; then
    : # up to date
elif is_patched; then
    echo "$series" > "$stamp" # patched before the stamp existed
else
    if ! git -C "$src" diff --quiet; then
        if [ ! -f "$stamp" ]; then
            echo "rerun: $src has local changes that aren't our patches; commit or stash them there," >&2
            echo "  or remove the checkout and re-run" >&2
            exit 1
        fi
        echo "rerun: the patches changed; re-applying them"
        git -C "$src" checkout -- .
    fi
    for patch in "${patches[@]}"; do
        name=$(basename "$patch")
        if ! git -C "$src" apply --check "$patch" 2>/dev/null; then
            echo "rerun: $name does not apply to rerun $pin; update the patch for this version" >&2
            exit 1
        fi
        git -C "$src" apply "$patch"
        echo "rerun: applied $name"
    done
    echo "$series" > "$stamp"
fi
echo "rerun: $pin ready in third_party/rerun/src"
