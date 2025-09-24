#!/bin/bash
set -e

# Set default flags/settings of containor
IMAGE_NAME="luna/python-robot:latest"
DISPLAY_ENABLED=false
BUILD_IMAGE=false
RESTART_CONTAINER=false

WORKING_DIR_CONTAINER="/home/luna/NMT-Lunabotics2025-Python-based"
WORKING_DIR_HOST="$(pwd)" 
: "${DISPLAY:=:0}"
# should be the root of NMT-Lunabotics2025-Python-based

# Set up phrase flags for running the script
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -d|--display) DISPLAY_ENABLED=true; shift ;;
        -b|--build) BUILD_IMAGE=true; shift ;;
        -r|--restart) RESTART_CONTAINER=true; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
done

# Build image if image is not built yet.
if [ "$BUILD_IMAGE" = true ] || ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
    echo "Building Docker image: $IMAGE_NAME"
    docker build -t $IMAGE_NAME .
fi

# If the image is being rebuilt stop all other containors
if [ "$RESTART_CONTAINER" = true ]; then
    OLD_CONTAINERS=$(docker ps -aq -f ancestor=$IMAGE_NAME)
    if [ -n "$OLD_CONTAINERS" ]; then
        docker rm -f $OLD_CONTAINERS
    fi
fi

# Checks if a container is already running, if it is it attaches to it, if not it starts a new one.
CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME)
if [ -z "$CONTAINER_ID" ]; then
    echo "Starting Docker container..."
if [[ "$OS" != "Windows_NT" ]]; then
    DOCKER_FLAGS=(
        --net=host
        --privileged
        --group-add video
        --device /dev:/dev        
        --env WORKING_DIR=$WORKING_DIR_CONTAINER
        -w /home/luna
    )
fi
    if [ "$DISPLAY_ENABLED" = true ]; then
        DOCKER_FLAGS+=(
            -e DISPLAY=$DISPLAY
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        )
        xhost +local:docker
    fi

    # Start container detached so we can copy files
    docker run -dit "${DOCKER_FLAGS[@]}" $IMAGE_NAME bash
    CONTAINER_ID=$(docker run -dit "${DOCKER_FLAGS[@]}" $IMAGE_NAME bash)

    # Copy host project into a temp folder inside the container
    docker exec "$CONTAINER_ID" rm -rf /tmp/NMT_temp
    docker cp "$WORKING_DIR_HOST" "$CONTAINER_ID":/tmp/NMT_temp

    # Remove old project folder safely
    docker exec "$CONTAINER_ID" rm -rf "$WORKING_DIR_CONTAINER"

    # Move temp folder into the final location
    docker exec "$CONTAINER_ID" mv /tmp/NMT_temp/NMT-Lunabotics2025-Python-based "$WORKING_DIR_CONTAINER"
    docker exec "$CONTAINER_ID" chown -R luna:luna "$WORKING_DIR_CONTAINER"

    echo "Files copied successfully."
fi

echo "Container ID: $CONTAINER_ID"

# attach containor to shell
docker exec -it $CONTAINER_ID bash
