#!/bin/bash
set -e

TRIPWIRE_FILE=".tripwire"

EXCLUDE_FROM_PULL=(
    "README.md" 
    "LiDAR_Project/"  
    "system_operations/component_tests/" 
    "system_operations/documents/" 
    ".gitgnore"
    "Map.png"
    "ros/guides-documents/"
    "ros/ros1_ws"
    ".tripwire"
    )

if [[ -f "$TRIPWIRE_FILE" ]]; then
    echo "Tripwire file detected, skipping pull. (Are you sure you are not on your local pc's files?)"
else
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
fi