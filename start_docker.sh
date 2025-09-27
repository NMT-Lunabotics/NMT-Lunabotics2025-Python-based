#!/bin/bash
set -e

# Set default flags/settings of container
IMAGE_NAME="luna/python-robot:latest"
DISPLAY_ENABLED=false
BUILD_IMAGE=false
RESTART_CONTAINER=false
START_SYSTEM_CONTROL=false

# Container working directory and mount
WORKING_DIR_CONTAINER="/home/luna/NMT-Lunabotics2025-Python-based"
WORKING_DIR_HOST="$(pwd)" 

# Use current environment variables if available, fallback to defaults
: "${DISPLAY:=$DISPLAY}"
: "${XAUTHORITY:=$HOME/.Xauthority}"

# Parse input flags
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -d|--display) DISPLAY_ENABLED=true; shift ;;   # Enable GUI forwarding
        -b|--build) BUILD_IMAGE=true; shift ;;         # Force image rebuild
        -r|--restart) RESTART_CONTAINER=true; shift ;; # Remove old containers first
        -s|--start) START_SYSTEM_CONTROL=true; shift ;; # Start system control script
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
done

# Build image if needed
if [ "$BUILD_IMAGE" = true ] || ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
    echo "Building Docker image: $IMAGE_NAME"
    docker build -t $IMAGE_NAME .
fi

# Remove old containers if requested
if [ "$RESTART_CONTAINER" = true ]; then
    OLD_CONTAINERS=$(docker ps -aq -f ancestor=$IMAGE_NAME)
    if [ -n "$OLD_CONTAINERS" ]; then
        docker rm -f $OLD_CONTAINERS
    fi
fi

# Check if container already exists
CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME | head -n1)
if [ -z "$CONTAINER_ID" ]; then
    echo "Starting Docker container..."

    # Base Docker flags for privileged access, devices, working directory
    DOCKER_FLAGS=(
        --network=host                                   # Use host networking
        --privileged                                     # Give container privileged access
        --group-add video                                # Give video group access
        --device /dev:/dev                               # Map devices  
        -v $WORKING_DIR_HOST:$WORKING_DIR_CONTAINER      # Mount host folder
        -w $WORKING_DIR_CONTAINER                        # Set working directory                 
    )

    # Enable display if requested and $DISPLAY is set
    if [ "$DISPLAY_ENABLED" = true ] && [ -n "$DISPLAY" ]; then
        DOCKER_FLAGS+=(
            -e DISPLAY=$DISPLAY                          # Forward display
            -v $XAUTHORITY:$XAUTHORITY:ro                # Forward Xauthority
            -e XAUTHORITY=$XAUTHORITY
        )
    fi

    # Start container detached
    CONTAINER_ID=$(docker run -dit "${DOCKER_FLAGS[@]}" $IMAGE_NAME bash)
fi

# Start system control script if requested 
if [ "$START_SYSTEM_CONTROL" = true ]; then
    echo "Starting heartbeat script inside container..."
    docker exec -it $CONTAINER_ID python3 system_operations/main/main.py
    exit 0
fi

# Attach to container shell
docker exec -it $CONTAINER_ID bash
