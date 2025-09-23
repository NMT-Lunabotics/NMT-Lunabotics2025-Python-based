#!/bin/bash
set -e

# Set working directory
: "${WORKING_DIR:=/home/luna/app}"
cd "$WORKING_DIR"

# --- Execute the command passed to the container ---
if [ $# -eq 0 ]; then
    # default command if none passed: run main.py
    exec python -m app.main
else
    exec "$@"
fi
