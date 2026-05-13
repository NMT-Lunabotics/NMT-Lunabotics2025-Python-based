#!/bin/bash
set -e

# Fix camera
[ -e /dev/video0 ] && sudo chmod 666 /dev/video0

# Always source ROS 2 first
source /opt/ros/humble/setup.bash

# Then source workspace if built
if [ -f /home/luna/NMT-Lunabotics2025-Python-based/ros2_ws/install/setup.bash ]; then
    source /home/luna/NMT-Lunabotics2025-Python-based/ros2_ws/install/setup.bash
fi

# Then bashrc if needed
if [ -f /home/luna/.bashrc ]; then
    source /home/luna/.bashrc
fi

# Go to working directory (ensure WORKING_DIR is set in your docker/env)
cd "${WORKING_DIR:-/home/luna/NMT-Lunabotics2025-Python-based}"

# Execute the command passed to the container
exec "$@"
