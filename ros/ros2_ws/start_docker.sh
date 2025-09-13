#!/bin/bash
set -e

# Default variables
ROS_DISTRO="humble"
IMAGE_NAME="luna/ros2:$ROS_DISTRO"
DISPLAY_ENABLED=false
BUILD_IMAGE=false
RESTART_CONTAINER=false
WORKING_DIR_CONTAINER="/home/luna/ros2_ws"
WORKING_DIR_HOST="$HOME/NMT-Lunabotics2025-Python-based/ros/ros2_ws"
WORKSPACE="/home/luna/ros2_ws"

# Parse flags
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -d|--display) DISPLAY_ENABLED=true; shift ;;
        -b|--build) BUILD_IMAGE=true; shift ;;
        -r|--restart) RESTART_CONTAINER=true; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
done

# Build image if needed
if [ "$BUILD_IMAGE" = true ] || ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
    echo "Building Docker image: $IMAGE_NAME"
    docker build -t $IMAGE_NAME .
fi

# Remove old containers if restarting
if [ "$RESTART_CONTAINER" = true ]; then
    OLD_CONTAINERS=$(docker ps -aq -f ancestor=$IMAGE_NAME)
    if [ -n "$OLD_CONTAINERS" ]; then
        docker rm -f $OLD_CONTAINERS
    fi
fi

# Check if container is already running
CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME)
if [ -z "$CONTAINER_ID" ]; then
    echo "Starting Docker container..."
    DOCKER_FLAGS=(
    --privileged
    --net=host
    --group-add video
    --volume=/dev:/dev:rw
    --volume $HOME_DIR/.ssh:/home/$USER/.ssh:ro
    --volume $WORKING_DIR_HOST:$WORKING_DIR_CONTAINER
    --env WORKING_DIR=$WORKING_DIR_CONTAINER
    -w $WORKING_DIR_CONTAINER
)


    if [ "$DISPLAY_ENABLED" = true ]; then
        DOCKER_FLAGS+=(
            -e DISPLAY=$DISPLAY
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        )
        xhost +local:docker
    fi

    docker run -dit "${DOCKER_FLAGS[@]}" $IMAGE_NAME /entrypoint.sh bash
    CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME)
fi

echo "Container ID: $CONTAINER_ID"

# Attach to container and start bash (or ROS camera test)
docker exec -it $CONTAINER_ID bash