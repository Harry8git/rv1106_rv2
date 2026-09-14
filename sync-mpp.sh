#!/bin/bash
set -e

TOPDIR="/home/hspencer/rv2"
SRCDIR="$TOPDIR/mpp-src"
PATCH_FILE="$TOPDIR/meta-luckfox/recipes-multimedia/rockchip-mpp/rockchip-mpp/0001-mpp_soc-hal-add-rv1106-and-rv1103-encoder-only-SoC-s.patch"

if [ ! -d "$SRCDIR" ]; then
    echo "ERROR: Source directory not found: $SRCDIR"
    exit 1
fi

cd "$SRCDIR"

# Check for modified tracked files
MODS=$(git status --porcelain -uno)
if [ -n "$MODS" ]; then
    echo "==> Detected modifications in MPP source files:"
    echo "$MODS"
    
    # Locate patch 0001 commit (first commit after upstream)
    TARGET_COMMIT=$(git rev-list origin/develop..HEAD | tail -n 1)
    
    if [ -z "$TARGET_COMMIT" ]; then
        echo "ERROR: Could not find target commit! Patch file NOT touched."
        exit 1
    fi

    echo "==> Squashing changes into patch 0001 commit ($TARGET_COMMIT)..."
    git commit -a --no-verify --fixup "$TARGET_COMMIT"
    GIT_SEQUENCE_EDITOR=true git rebase -i --autosquash origin/develop
    
    # Get the updated commit hash
    NEW_COMMIT=$(git rev-list origin/develop..HEAD | tail -n 1)
    
    TMP_PATCH="/tmp/mpp-0001-safe.patch"
    echo "==> Formatting patch to temporary file..."
    git format-patch -1 "$NEW_COMMIT" --stdout > "$TMP_PATCH"
    
    # Safety check: ensure file is non-empty and has reasonable size
    FILE_SIZE=$(wc -c < "$TMP_PATCH" 2>/dev/null || echo 0)
    if [ "$FILE_SIZE" -gt 5000 ]; then
        cp "$TMP_PATCH" "$PATCH_FILE"
        echo "==> Patch 0001 successfully updated ($FILE_SIZE bytes)!"
    else
        echo "ERROR: Generated patch was too small ($FILE_SIZE bytes). Original patch preserved!"
        exit 1
    fi
else
    echo "==> No uncommitted file changes detected in MPP source."
fi

cd "$TOPDIR"
echo "==> Recompiling rockchip-mpp in Yocto..."
bitbake -C compile rockchip-mpp
echo "==> SUCCESS! rockchip-mpp compiled and ready."
