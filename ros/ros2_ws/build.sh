#!/bin/bash
set -e

# Where you want the repo
WORKDIR=~/ros2_ws

# Remove old repo if needed
rm -rf $WORKDIR
mkdir -p $WORKDIR

# Clone the repo
if [ -z "$GITHUB_TOKEN" ]; then
  echo "Error: GITHUB_TOKEN is not set."
  exit 1
fi

git clone -b benjaminstestbranch https://benjamin-p15:$GITHUB_TOKEN@github.com/NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git $WORKDIR

cd $WORKDIR

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

# Commit & push changes
git config user.name "benjamin-p15"
git config user.email "benjamin.peterson@student.nmt.edu"
git add .
git commit -m "Auto update from container" || true
git push origin benjaminstestbranch
