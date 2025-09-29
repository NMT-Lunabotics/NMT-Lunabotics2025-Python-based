#!/bin/bash
set -e

read -a EXCLUDE_FROM_PULL <<< "$EXCLUDE_FROM_PULL_STR"

# Git setup
git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

# Git pull and overwrite local files, excluding specified files defined in ./start_docker.sh
git fetch origin
git sparse-checkout init --cone
ALL_FILES=$(git ls-tree -r --name-only origin/main)
INCLUDE=()
for f in $ALL_FILES; do
    skip=false
    for excl in "${EXCLUDE_FROM_PULL[@]}"; do
        [[ "$f" == "$excl"* ]] && skip=true && break
    done
    $skip || INCLUDE+=("$f")
done
git sparse-checkout set "${INCLUDE[@]}"
git reset --hard origin/main
git clean -fdx