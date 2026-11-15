#!/bin/bash
set -euo pipefail

TOPDIR="/home/hspencer/rv2"
SRCDIR="$TOPDIR/mpp-src"
RECIPE="$TOPDIR/meta-luckfox/recipes-multimedia/rockchip-mpp/rockchip-mpp.bb"
PATCH_DIR="$TOPDIR/meta-luckfox/recipes-multimedia/rockchip-mpp/rockchip-mpp"
DO_REBASE=0
DO_BUILD=1

for arg in "$@"; do
    case "$arg" in
        --rebase)   DO_REBASE=1 ;;
        --no-build) DO_BUILD=0 ;;
        -h|--help)
            echo "Usage: $0 [--rebase] [--no-build]"
            echo "  --rebase    Fetch upstream origin/develop, rebase mpp-src, and bump SRCREV"
            echo "  --no-build  Export patches and sync recipe without running bitbake"
            exit 0
            ;;
        *)
            echo "Unknown option: $arg" >&2
            exit 1
            ;;
    esac
done

[ -d "$SRCDIR/.git" ] || { echo "ERROR: $SRCDIR is not a git repo"; exit 1; }
[ -f "$RECIPE" ] || { echo "ERROR: Recipe not found: $RECIPE"; exit 1; }

# 1. Require clean working tree in mpp-src (never auto-squash blindly)
if [ -n "$(git -C "$SRCDIR" status --porcelain)" ]; then
    echo "ERROR: $SRCDIR has uncommitted changes. Commit/amend them cleanly first:"
    git -C "$SRCDIR" status -s
    exit 1
fi

CURRENT_SRCREV=$(sed -n 's/^SRCREV = "\([^"]*\)"/\1/p' "$RECIPE" | head -n 1)
[ -n "$CURRENT_SRCREV" ] || { echo "ERROR: Could not read SRCREV from $RECIPE"; exit 1; }

# 2. Optional upstream rebase
if [ "$DO_REBASE" -eq 1 ]; then
    echo "==> Fetching upstream origin..."
    git -C "$SRCDIR" fetch origin
    NEW_SRCREV=$(git -C "$SRCDIR" rev-parse origin/develop)
    if [ "$CURRENT_SRCREV" != "$NEW_SRCREV" ]; then
        echo "==> Rebasing local commits from $CURRENT_SRCREV onto $NEW_SRCREV..."
        git -C "$SRCDIR" rebase --onto "$NEW_SRCREV" "$CURRENT_SRCREV"
        CURRENT_SRCREV="$NEW_SRCREV"
        python3 - "$RECIPE" "$CURRENT_SRCREV" <<'PY'
import re, sys
p, rev = sys.argv[1:3]
s = open(p).read()
s, n = re.subn(r'^SRCREV\s*=\s*"[^"]+"', f'SRCREV = "{rev}"', s, count=1, flags=re.M)
assert n == 1
open(p, "w").write(s)
PY
    else
        echo "==> Already at latest origin/develop ($CURRENT_SRCREV)."
    fi
fi

# 3. Verify SRCREV is an ancestor of HEAD
if ! git -C "$SRCDIR" merge-base --is-ancestor "$CURRENT_SRCREV" HEAD; then
    echo "ERROR: SRCREV ($CURRENT_SRCREV) is not an ancestor of mpp-src HEAD."
    echo "Run '$0 --rebase' or align mpp-src with SRCREV."
    exit 1
fi

COMMIT_COUNT=$(git -C "$SRCDIR" rev-list --count "${CURRENT_SRCREV}..HEAD")
echo "==> Exporting $COMMIT_COUNT patch(es) above $CURRENT_SRCREV..."

# 4. Export to staging dir and validate every patch before touching meta-luckfox
TMP_DIR=$(mktemp -d)
WT_DIR=$(mktemp -d)
cleanup() {
    git -C "$SRCDIR" worktree remove --force "$WT_DIR" 2>/dev/null || true
    rm -rf "$TMP_DIR" "$WT_DIR"
}
trap cleanup EXIT

if [ "$COMMIT_COUNT" -gt 0 ]; then
    git -C "$SRCDIR" format-patch -N "${CURRENT_SRCREV}..HEAD" -o "$TMP_DIR" >/dev/null
    for p in "$TMP_DIR"/*.patch; do
        if ! grep -q "^Upstream-Status:" "$p"; then
            echo "ERROR: Missing 'Upstream-Status:' line in commit for $(basename "$p")"
            exit 1
        fi
    done

    # Verify patches apply cleanly in sequence on a detached worktree at SRCREV
    git -C "$SRCDIR" worktree add --detach "$WT_DIR" "$CURRENT_SRCREV" >/dev/null 2>&1
    git -C "$WT_DIR" am --keep-cr "$TMP_DIR"/*.patch >/dev/null
    echo "==> Verified all $COMMIT_COUNT patch(es) apply cleanly to $CURRENT_SRCREV."
fi

# 5. Replace patches in meta-luckfox and sync SRC_URI block
rm -f "$PATCH_DIR"/*.patch
if [ "$COMMIT_COUNT" -gt 0 ]; then
    cp "$TMP_DIR"/*.patch "$PATCH_DIR/"
fi

python3 - "$RECIPE" "$PATCH_DIR" <<'PY'
import glob, os, re, sys
recipe_path, patch_dir = sys.argv[1:3]
patches = sorted(os.path.basename(p) for p in glob.glob(os.path.join(patch_dir, "*.patch")))
lines = "".join(f'SRC_URI += "file://{p}"\n' for p in patches)
block = (
    "# BEGIN MPP PATCHES (managed by sync-mpp.sh)\n"
    + lines +
    "# END MPP PATCHES"
)
text = open(recipe_path).read()
text, n = re.subn(
    r'# BEGIN MPP PATCHES.*?# END MPP PATCHES',
    block,
    text,
    count=1,
    flags=re.S,
)
assert n == 1, "Missing BEGIN/END MPP PATCHES markers in recipe"
open(recipe_path, "w").write(text)
PY

echo "==> Synced $(ls -1 "$PATCH_DIR"/*.patch 2>/dev/null | wc -l) patch(es) and updated $RECIPE"

# 6. Build in Yocto (full task graph, not -C compile, so do_patch runs properly)
if [ "$DO_BUILD" -eq 1 ]; then
    cd "$TOPDIR"
    set +u
    source openembedded-core/oe-init-build-env build >/dev/null
    set -u
    bitbake rockchip-mpp
fi
