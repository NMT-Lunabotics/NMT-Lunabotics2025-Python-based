#!/bin/bash
set -e

# Sets the default values for running the container from the env_file list of variables.
ROS_DISTRO="humble"
ROS_DOMAIN_ID=42
IMAGE_NAME="luna/ros2:$ROS_DISTRO"
PI_WS="/home/masterpi/NMT-Lunabotics2025-Python-based"
REPO_ROOT="/home/masterpi/NMT-Lunabotics2025-Python-based"
GIT_BRANCH="benjaminstestbranch"
GIT_REPO="git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git"

# Variables set by flags
DISPLAY_ENABLED=false
BUILD_IMAGE=false

# Check flags that were set, and update variables. 
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -d|-display) DISPLAY_ENABLED=true; shift ;;
        -b|-build) BUILD_IMAGE=true; shift ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
done

# Run flags that docker container needs.
DOCKER_RUN_FLAGS=(
    --privileged 
    --net=host
    --volume=/home/masterpi/.ssh:/root/.ssh:ro 
    -w /workspace
)

# Mount the repository directory into the container
DOCKER_RUN_FLAGS+=("--volume=$PI_WS:/workspace")

# Enable X11 display if DISPLAY is set
if [ "$DISPLAY_ENABLED" = true ]; then
    DOCKER_RUN_FLAGS+=("--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw")
    DOCKER_RUN_FLAGS+=("-e DISPLAY=$DISPLAY")
    xhost +local:docker
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

# Check if a container is already running for this image
RUNNING=false
CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME)
if [ -n "$CONTAINER_ID" ]; then
    echo "Docker container is already running."
    RUNNING=true
fi

# Start container if not running
if [ "$RUNNING" = false ]; then
    # Remove old containers from this image
    OLD_CONTAINERS=$(docker ps -aq -f ancestor=$IMAGE_NAME)
    if [ -n "$OLD_CONTAINERS" ]; then
        echo "Removing old containers..."
        docker rm -f $OLD_CONTAINERS
    fi
    echo "Starting Docker container..."
    docker run -dit "${DOCKER_RUN_FLAGS[@]}" $IMAGE_NAME bash
    CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME)
fi

# Fix SSH permissions inside the container
docker exec $CONTAINER_ID bash -c "\
    chmod 600 /root/.ssh/id_ed25519 2>/dev/null || true; \
    chmod 644 /root/.ssh/known_hosts 2>/dev/null || true; \
    chmod 644 /root/.ssh/config 2>/dev/null || true; \
    chmod 700 /root/.ssh 2>/dev/null || true"

# Test SSH connection first
docker exec $CONTAINER_ID bash -c "\
    echo 'Testing SSH connection to GitHub...'; \
    ssh -T git@github.com 2>&1 | grep -q 'successfully authenticated' && echo 'SSH connection successful' || echo 'SSH connection test completed'"

# Exec into the container with bash
docker exec -it $CONTAINER_ID bash -c "\
    set -e; \
    echo 'Starting SSH agent and adding key...'; \
    eval \"\$(ssh-agent -s)\"; \
    ssh-add /root/.ssh/id_ed25519; \
    echo 'Checking if Git repository exists...'; \
    if [ ! -d /workspace/.git ]; then \
        echo 'Git repository not found. Cloning repository...'; \
        cd /; \
        rm -rf /workspace_temp 2>/dev/null || true; \
        git clone $GIT_REPO /workspace_temp; \
        cd /workspace_temp; \
        git checkout $GIT_BRANCH; \
        echo 'Copying repository to mounted volume...'; \
        cp -r . /workspace/; \
        cd /workspace; \
        rm -rf /workspace_temp; \
    else \
        echo 'Pulling latest Git changes...'; \
        cd /workspace; \
        git config --global --add safe.directory /workspace; \
        git fetch origin; \
        git checkout $GIT_BRANCH; \
        git reset --hard origin/$GIT_BRANCH; \
        git clean -fd; \
        git pull origin $GIT_BRANCH; \
    fi; \
    echo 'Building workspace...'; \
    cd ros/ros2_ws; \
    colcon build; \
    echo 'Committing any new changes...'; \
    cd /workspace; \
    git add .; \
    git commit -m 'Auto-sync: updated workspace after build' || echo 'Nothing to commit'; \
    git push origin $GIT_BRANCH || echo 'Push failed or nothing to push'; \
    echo 'Build complete. Dropping into shell...'; \
    bash"