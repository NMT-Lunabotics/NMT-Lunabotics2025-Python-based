#!/bin/bash
set -e

# Default: do everything
PULL_FROM_GITHUB=true
BUILD_PROJECT=false
REBUILD_IMAGE=false

CONTAINER_NAME="lunabotics_container"
IMAGE_NAME="luna/python-robot:latest"
WORKDIR="/home/luna/NMT-Lunabotics2025-Python-based"

# Parse flags
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -p|--pull) PULL_FROM_GITHUB=false ;;
        -b|--build) BUILD_PROJECT=true ;;
        *) echo "Unknown flag: $1"; exit 1 ;;
    esac
    shift
done

cd "$WORKDIR"

git config --global user.email "benjamin.peterson@student.nmt.edu"
git config --global user.name "benjamin-p15"
git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true

if [ "$PULL_FROM_GITHUB" = true ]; then
    git fetch origin
    git reset --hard origin/main
    git clean -fdx
fi

if [[ "$BUILD_PROJECT" = true || "$REBUILD_IMAGE" = true ]]; then

    if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    fi

    if [ "$BUILD_PROJECT" = true ]; then
        docker build -t $IMAGE_NAME .
    fi

    colcon build --symlink-install
    source install/setup.bash

    docker run -it --name $CONTAINER_NAME -v "$WORKDIR":/ros2_ws $IMAGE_NAME bash
fi
