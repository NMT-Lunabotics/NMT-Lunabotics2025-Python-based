#!/bin/bash
set -e

# Where your repo lives
WORKDIR="$HOME/ros2_ws"
git config --global --add safe.directory "$WORKDIR"

# Make sure GitHub host key is known (avoid first-time prompts)
mkdir -p ~/.ssh
ssh-keyscan github.com >> ~/.ssh/known_hosts 2>/dev/null || true

# Clone if it doesn't exist
if [ ! -d "$WORKDIR/.git" ]; then
    git clone -b benjaminstestbranch \
      git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git \
      "$WORKDIR"
fi

cd "$WORKDIR"

# Pull latest changes
git fetch origin
git reset --hard origin/benjaminstestbranch
git clean -fd

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

# Commit & push any local changes
git add -A
git commit -m "Update ROS workspace" || echo "Nothing to commit"
git push origin benjaminstestbranch
