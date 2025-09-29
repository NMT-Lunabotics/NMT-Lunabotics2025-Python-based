#!/bin/bash
set -e

# Default: do everything
PULL_FROM_GITHUB=true
BUILD_PROJECT=false

# Parse flags
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -p|--pull) PULL_FROM_GITHUB=false ;;
        -b|--build) BUILD_PROJECT=true ;;
        *) echo "Unknown flag: $1"; exit 1 ;;
    esac
    shift
done

# Git setup
git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

# Sync code to branch
if [ "$PULL_FROM_GITHUB" = true ]; then
    git fetch origin
    git reset --hard origin/main
    git clean -fdx
fi

# Build containor if it is required
if [[ "$BUILD_PROJECT" = true || "$REBUILD_IMAGE" = true ]]; then
    # Build workspace
    colcon build --symlink-install
    source install/setup.bash
fi