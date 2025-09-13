#!/bin/bash
set -e

: "${WORKING_DIR:=/home/luna/ros2_ws}"
cd "$WORKING_DIR"

# Source ROS 2
source /opt/ros/humble/setup.bash
if [ -f $WORKING_DIR/install/setup.bash ]; then
    source $WORKING_DIR/install/setup.bash
fi

# If workspace hasn't been built yet, build it
if [ ! -f "$WORKING_DIR/install/setup.bash" ]; then
    colcon build
fi

# Source the workspace
source "$WORKING_DIR/install/setup.bash"

# Git setup
cd "$WORKING_DIR"
if [ ! -d .git ]; then
    echo "Initializing git repository..."
    git init
    git remote add origin "$GIT_REPO"
    git fetch origin
    git checkout -b "$GIT_BRANCH" --track "origin/$GIT_BRANCH"
    git config user.name "$GIT_USER"
    git config user.email "$GIT_EMAIL"
    echo "Git repository initialized!"
fi

# Execute the command passed to the entrypoint
exec "$@"