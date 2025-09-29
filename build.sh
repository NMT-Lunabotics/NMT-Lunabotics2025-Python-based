#!/bin/bash
set -e

read -a EXCLUDE_FROM_PULL <<< "$EXCLUDE_FROM_PULL_STR"

# Git setup
git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

# Git pull and overwrite local files, excluding specified files defined in ./start_docker.sh
git fetch origin
git sparse-checkout init --no-cone
ALL_FILES=$(git ls-tree -r --name-only origin/main)
INCLUDE=()
while IFS= read -r f; do
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
RESTART_CONTAINER=true
BUILD_IMAGE=true