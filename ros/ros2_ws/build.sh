#!/bin/bash
set -e

# Go to workspace
cd /ros2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Git setup (first time only)
git config user.email "benjamin.peterson@student.nmt.edu"
git config user.name "benjamin-p15"
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

# Sync branch
git fetch origin
git checkout -B benjaminstestbranch origin/benjaminstestbranch

# Build workspace
colcon build --symlink-install

# Commit and push changes
git add -A
git commit -m "Update ROS workspace" || echo "Nothing to commit"
git push origin benjaminstestbranch
