#!/bin/bash
set -e

EXCLUDE_FROM_PULL=("ros/" "DriverStation/" "README.md" "LiDAR_Project/" "SLAM/")

git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

git fetch origin
git sparse-checkout init --no-cone
ALL_FILES=$(git ls-tree -r --name-only origin/main)
INCLUDE=()
while IFS= read -r f; do
    f="${f#./}"
    skip=false
    for excl in "${EXCLUDE_FROM_PULL[@]}"; do
        [[ "$f" == "$excl"* || "$f" == "$excl" ]] && skip=true && break
    done
    if [ "$skip" = false ]; then
        INCLUDE+=("$f")
    fi
done <<< "$ALL_FILES"
git sparse-checkout set "${INCLUDE[@]}"
git reset --hard origin/main
git clean -fdx