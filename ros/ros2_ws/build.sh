#!/bin/bash
set -e

WORKDIR="/ros2_ws"

# Default: do everything
DO_SYNC_ONLY=false

# Parse flags
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -s|--sync) DO_SYNC_ONLY=true ;;
        *) echo "Unknown flag: $1"; exit 1 ;;
    esac
    shift
done

# Git setup
git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

cd "$WORKDIR"

# Sync branch
git fetch origin
git checkout -B benjaminstestbranch origin/benjaminstestbranch
git merge origin/benjaminstestbranch || echo "Already up to date"

# If only syncing, exit here
if [ "$DO_SYNC_ONLY" = true ]; then
    echo "Sync complete. Exiting."
    exit 0
fi

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install
source install/setup.bash

# Commit and push changes
git add -A
git commit -m "Update ROS workspace" || echo "Nothing to commit"
git push origin benjaminstestbranch
