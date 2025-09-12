#!/bin/bash
set -e

# Sets the default values for running the containor from the env_file list of variables.
ROS_DISTRO="humble"
ROS_DOMAIN_ID=42
IMAGE_NAME="luna/ros2:$ROS_DISTRO"
RESTART_CONTAINER=false


#Variables set by flags
DISPLAY_ENABLED=false
BUILD_IMAGE=false
export DISPLAY=$DISPLAY






#Check flags that were set, and update variables. 
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -d|-display) DISPLAY_ENABLED=true; shift ;;
        -b|-build) BUILD_IMAGE=true; shift ;;
        -r|-restart) RESTART_CONTAINER=true; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
done

#Run flags that docker containor needs.
DOCKER_RUN_FLAGS=(
    --privileged 
    --net=host
    --volume=/home/masterpi/NMT-Lunabotics2025-Python-based/ros/ros2_ws:/ros2_ws
    --volume=$HOME/.ssh:/root/.ssh
    -e DISPLAY=$DISPLAY \
    -w /ros2_ws 
    )

# Enable X11 display if DISPLAY is set
if [ "$DISPLAY_ENABLED" = true ]; then
    DOCKER_RUN_FLAGS+=("-e" "DISPLAY=$DISPLAY")
fi

# If image does not exist build it if the architecture is supported.
if [ "$BUILD_IMAGE" = true ] || ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
    echo "Building Docker image: $IMAGE_NAME"
    ARCH=$(uname -m)
    TARGET_ARCH=""
    if [[ "$ARCH" == "x86_64" ]]; then
        TARGET_ARCH="amd64"
    elif [[ "$ARCH" == "aarch64" ]]; then
        TARGET_ARCH="arm64"
    else
        echo "Unsupported architecture: $ARCH"
        exit 1
    fi

    docker build -t $IMAGE_NAME --build-arg TARGET_ARCH=$TARGET_ARCH .
fi

# Restart container if requested
if [ "$RESTART_CONTAINER" = true ]; then
    echo "Restarting: stopping and removing all containers for $IMAGE_NAME..."
    OLD_CONTAINERS=$(docker ps -aq -f ancestor=$IMAGE_NAME)
    if [ -n "$OLD_CONTAINERS" ]; then
        docker rm -f $OLD_CONTAINERS
    fi
    RUNNING=false
fi

# Check if a container is already running for this image
RUNNING=false
CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME)
if [ -n "$CONTAINER_ID" ]; then
    echo "Docker container is already running."
    RUNNING=true
fi

# Start container if not running
if [ "$RUNNING" = false ]; then
    echo "Starting Docker container..."
    docker run -dit "${DOCKER_RUN_FLAGS[@]}" $IMAGE_NAME bash
    CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME)
fi


# Exec into the container with bash
docker run -it "${DOCKER_RUN_FLAGS[@]}" $IMAGE_NAME bash