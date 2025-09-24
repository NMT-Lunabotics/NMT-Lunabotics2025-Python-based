#!/bin/bash
# entry_point.sh

# Environment variables expected:
# HOST_USER - the username on the host machine
# HOST_IP   - the IP of the host machine
# HOST_PATH - path to NMT-Lunabotics2025-Python-based on the host

# Temporary directory for copying host files
mkdir -p $HOME_DIR/NMT-Lunabotics2025-Python-based

echo "Copying host files into container..."
rsync -avz --delete $HOST_USER@$HOST_IP:$HOST_PATH/ $HOME_DIR/NMT-Lunabotics2025-Python-based/

echo "Files copied successfully."

# Start your normal container behavior
cd $WORKING_DIR

# If you have additional commands to start your robot environment, put them here
exec "$@"
