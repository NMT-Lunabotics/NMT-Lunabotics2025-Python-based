#!/bin/bash
set -e

WORKDIR="$HOME/ros2_ws"

# Git config
git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git config --global --add safe.directory "$WORKDIR"

sudo chown -R $(id -u):$(id -g) /ros2_ws

# Ensure SSH known hosts
mkdir -p ~/.ssh
ssh-keyscan github.com >> ~/.ssh/known_hosts 2>/dev/null || true

# Clone repo if it doesn't exist
if [ ! -d "$WORKDIR/.git" ]; then
    git clone -b benjaminstestbranch \
      git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git \
      "$WORKDIR"
fi

cd "$WORKDIR"

# Ensure branch exists and matches remote
git fetch origin
git checkout -B benjaminstestbranch origin/benjaminstestbranch

# Build project (adjust to your actual build folder)
<<<<<<< HEAD
=======
cd package_name  # if your build is inside this folder
>>>>>>> origin/benjaminstestbranch
if [ -f Makefile ]; then
    make
elif [ -f package.json ]; then
    npm install && npm run build
elif [ -f CMakeLists.txt ]; then
    mkdir -p build && cd build && cmake .. && make
else
    echo "No known build system found."
fi
cd "$WORKDIR"  # return to repo root

# Commit & push changes
git add -A
git commit -m "Update ROS workspace" || echo "Nothing to commit"
git push origin benjaminstestbranch
