#!/bin/bash
set -e

# Set working directory
: "${WORKING_DIR:=/home/luna/app}"
cd "$WORKING_DIR"

# --- Execute the command passed to the container ---
if [ $# -eq 0 ]; then
    # No default command for now, just keep container interactive
    exec bash
else
    exec "$@"
fi
