#!/bin/bash
set -e

MSG="$1"
if [ -z "$MSG" ]; then
    read -p "Enter commit message (press Enter for timestamp): " MSG
    if [ -z "$MSG" ]; then
        MSG="Update: $(date '+%Y-%m-%d %H:%M:%S')"
    fi
fi

# 1. Commit and push ArduPilot repo if it has changes
if [ -d "apps/ardupilot/.git" ]; then
    echo "🔍 Checking apps/ardupilot..."
    cd apps/ardupilot
    if [ -n "$(git status --porcelain)" ]; then
        echo "📦 Staging ArduPilot changes..."
        git add .
        echo "💾 Committing ArduPilot: $MSG"
        git commit -m "$MSG"
        echo "🚀 Pushing ArduPilot to GitHub..."
        git push
        echo "✅ ArduPilot pushed!"
    else
        echo "ℹ️ ArduPilot has no changes."
    fi
    cd ../..
fi

# 2. Commit and push main rv2 repo
echo "📦 Staging rv2 changes..."
git add .
echo "💾 Committing rv2: $MSG"
git commit -m "$MSG"
echo "🚀 Pushing rv2 to GitHub..."
git push

echo "✅ All repos successfully pushed to GitHub!"
