#!/bin/bash
set -e

WORKDIR="/ros2_ws"

# Default: do everything
sync_to_github=false
push_to_github=false
build_project=false

# Parse flags
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -s|-sync) sync_to_github=true ;;
        -p|-push) push_to_github=true ;;
        -b|-build) build_project=true ;;
        *) echo "Unknown flag: $1"; exit 1 ;;
    esac
    shift
done

# Git setup
git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

cd "$WORKDIR"

# Sync code to branch
if [ "$sync_to_github" = true ]; then
    git fetch origin
    git checkout -B benjaminstestbranch origin/benjaminstestbranch
    git merge origin/benjaminstestbranch || echo "Already up to date"
fi


# Sync code to branch
if [[ "$build_project" = true || ( "$build_project" = false && "$sync_to_github" = false && "$push_to_github" = false ) ]]; then
    # Source ROS 2
    source /opt/ros/humble/setup.bash

    # Build workspace
    colcon build --symlink-install
    source install/setup.bash
fi

# Commit and push changes
if [ "$push_to_github" = true ]; then
    git add -A
    git commit -m "Update ROS workspace" || echo "Nothing to commit"
    git push origin benjaminstestbranch
fi