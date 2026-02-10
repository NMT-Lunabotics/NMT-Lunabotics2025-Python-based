#!/bin/bash
set -e

LOCAL=${1}
LOCAL_USERNAME=${2}

TRIPWIRE_FILE=".tripwire"
REPO="NMT-Lunabotics2025-Python-based"

EXCLUDE_FROM_PULL=(
    "README.md" 
    "LiDAR_Project/"  
    "system_operations/component_tests/" 
    "system_operations/documents/" 
    "Map.png"
    "ros/guides-documents/"
    "ros/ros1_ws"
    ".tripwire"
    ".git/"
    )

if [[ -f "$TRIPWIRE_FILE" ]]; then
    echo -e "\e[31mERROR\e[0m Tripwire file detected, skipping pull. (Are you sure you are not on your local pc's files?)"
elif [[ "$LOCAL" == true ]]; then
    if [[ "$LOCAL_USERNAME" == "unknown" ]]; then
        echo -e "\e[31mERROR\e[0m Username for --pull not given"
        exit 1
    fi
    PC_IP=$(echo "$SSH_CLIENT" | awk '{print $1}')

    RSYNC_EXCLUDES=()
    for item in "${EXCLUDE_FROM_PULL[@]}"; do
        RSYNC_EXCLUDES+=(--exclude="$item")
    done

    rsync -avz --delete --exclude-from=<(printf "%s\n" "${EXCLUDE_FROM_PULL[@]}") --exclude-from=.gitignore $LOCAL_USERNAME@$PC_IP:/home/$LOCAL_USERNAME/$REPO/ "$(pwd)/"
    exit 0
else
    exit 0
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