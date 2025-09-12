#!/bin/bash
set -e

# Where your repo lives
WORKDIR="$HOME/ros2_ws"
git config --global --add safe.directory /ros2_ws


# Clone if it doesn't exist
if [ ! -d "$WORKDIR/.git" ]; then
    if [ -z "$GITHUB_TOKEN" ]; then
        echo "Error: GITHUB_TOKEN is not set."
        exit 1
    fi

    git clone -b benjaminstestbranch \
      https://benjamin-p15:$GITHUB_TOKEN@github.com/NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git \
      "$WORKDIR"
fi

cd "$WORKDIR"

# Pull latest changes
git fetch origin
git merge origin/benjaminstestbranch

# Build project
if [ -f Makefile ]; then
    make
elif [ -f package.json ]; then
    npm install && npm run build
elif [ -f CMakeLists.txt ]; then
    mkdir -p build && cd build && cmake .. && make
else
    echo "No known build system found."
fi

git remote set-url origin https://benjamin-p15:$GITHUB_TOKEN@github.com/NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git

# Commit & push any local changes
git fetch origin
git config user.name "benjamin-p15"
git config user.email "benjamin.peterson@student.nmt.edu"
git commit -m "Update ROS workspace" || true
git push origin benjaminstestbranch