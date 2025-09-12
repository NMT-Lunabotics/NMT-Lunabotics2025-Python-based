#!/bin/bash
set -e

cd ~/ros2_ws

if [ -z "$GITHUB_TOKEN" ]; then
  echo "Error: GITHUB_TOKEN is not set."
  exit 1
fi

# Set Git username/email for commits
git config user.name "benjamin-p15"
git config user.email "benjamin.peterson@student.nmt.edu"

git remote set-url origin https://$GITHUB_TOKEN@github.com/NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git

git fetch origin benjaminstestbranch
git checkout -B benjaminstestbranch origin/benjaminstestbranch
git pull origin benjaminstestbranch

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

git add .
git commit -m "Auto update from container" || true
git push origin benjaminstestbranch
