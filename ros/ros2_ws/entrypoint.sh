#!/bin/bash
set -e

# Git setup
cd /ros2_ws
if [ ! -d .git ]; then
    echo "Initializing git repository..."
    git init
    git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git
    git fetch origin
    git checkout -b benjaminstestbranch --track origin/benjaminstestbranch
    git config user.name "benjamin-p15"
    git config user.email "benjamin.peterson@student.nmt.edu"
    echo "Git repository initialized!"
fi

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace if built
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"