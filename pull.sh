#!/bin/bash
set -e

LOCAL=${1}
LOCAL_USERNAME=${2}

TRIPWIRE_FILE=".tripwire"
REPO="NMT-Lunabotics2025-Python-based"
BRANCH="simulated-nav2"

EXCLUDE_FROM_PULL=(
    ".venv/"
    ".vscode/"
    "DriverStation/"
    "old/"
    #"system_operations/arduino_cli/"
    #"system_operations/arduino_serial_commuication/" 
    "system_operations/autonomous/" 
    "system_operations/component_tests/" 
    "system_operations/documents/" 
    "system_operations/jetson_networking/" 
    "system_operations/main/" 
    "info.txt" 
    ".tripwire"
    ".env_file.txt"
    "README.md"
    ".git/"
    "info.md"
    )

if [[ -f "$TRIPWIRE_FILE" ]]; then
    echo -e "\e[31m[ERROR]\e[0m Tripwire file detected, skipping pull. (Are you sure you are not on your local pc's files?)"
elif [[ "$LOCAL" == true ]]; then
    if [[ "$LOCAL_USERNAME" == "unknown" ]]; then
        echo -e "\e[31m[ERROR]\e[0m Username for --pull not given"
        exit 1
    fi
    PC_IP=$(echo "$SSH_CLIENT" | awk '{print $1}')
    if [ "$PC_IP" == "192.168.10.1" ]; then
        PC_IP="192.168.10.2"
    elif [ "$PC_IP" == "192.168.10.2" ]; then
        PC_IP="192.168.10.1"
    fi

    RSYNC_EXCLUDES=()
    for item in "${EXCLUDE_FROM_PULL[@]}"; do
        RSYNC_EXCLUDES+=(--exclude="$item")
    done

    rsync -avz --delete --exclude-from=<(printf "%s\n" "${EXCLUDE_FROM_PULL[@]}") --exclude-from=.gitignore $LOCAL_USERNAME@$PC_IP:~/Documents/$REPO/ "$(pwd)/"
else
    git config --global user.email "benjamin.peterson@student.nmt.edu"
    git config --global user.name "benjamin-p15"
    git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

    git fetch origin $BRANCH
    git reset --hard origin/$BRANCH
    git clean -fdx
    git checkout -B $BRANCH origin/$BRANCH
    git sparse-checkout init --no-cone
    ALL_FILES=$(git ls-tree -r --name-only origin/$BRANCH)
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
fi