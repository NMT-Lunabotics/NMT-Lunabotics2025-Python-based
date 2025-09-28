#!/bin/bash
set -e

WORKDIR="/ros2_ws"

# Default: do everything
PULL_FROM_GITHUB=true
BUILD_PROJECT=true

# Parse flags
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -s|-sync) sync_to_github=false ;;
        -b|-build) build_project=false ;;
        *) echo "Unknown flag: $1"; exit 1 ;;
    esac
    shift
done

# Git setup
git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git config --global --add safe.directory /ros2_ws
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

# Sync code to branch
if [ "$sync_to_github" = true ]; then
    git fetch origin
    git checkout benjaminstestbranch 2>/dev/null || git checkout -b benjaminstestbranch
    git reset --hard origin/benjaminstestbranch
    git clean -fdx
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