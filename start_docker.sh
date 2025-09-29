#!/bin/bash
set -e

# Set default flags/settings of container
IMAGE_NAME="luna/python-robot:latest"
DISPLAY_ENABLED=false
BUILD_IMAGE=false
RESTART_CONTAINER=false
START_SYSTEM_CONTROL=false
GITHUB_PULL=false

# Container working directory and mount
MOUNT_USERNAME=false
MOUNT_HOST_PATH=false
REPOSITORY_NAME="NMT-Lunabotics2025-Python-based"
WORKING_DIR_CONTAINER="/home/luna/$REPOSITORY_NAME"
WORKING_DIR_HOST="$(pwd)" 

# Use current environment variables if available, fallback to defaults
: "${DISPLAY:=$DISPLAY}"
: "${XAUTHORITY:=$HOME/.Xauthority}"

# Parse input flags
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -d|--display) DISPLAY_ENABLED=true; shift ;;    # Enable GUI forwarding
        -b|--build) BUILD_IMAGE=true; shift ;;          # Force image rebuild
        -r|--restart) RESTART_CONTAINER=true; shift ;;  # Remove old containers first
        -s|--start) START_SYSTEM_CONTROL=true; shift ;; # Start system control script
        -p|--pull) GITHUB_PULL=true; shift ;;           # Pull github changes before building
        -mm|--mount) [[ "$#" -lt 3 ]] && { echo "Error: --mount <username> <host_path>"; exit 1; }; MOUNT_USERNAME="$2"; MOUNT_HOST_PATH="$3"; shift 3 ;; # Custom mount point
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
done

# Mount to host repository --mount flag is used
if [ -n "$MOUNT_USERNAME" ] && [ -n "$MOUNT_HOST_PATH" ] && [ "$MOUNT_USERNAME" != "false" ] && [ "$MOUNT_HOST_PATH" != "false" ]; then
    PC_IP=$(echo $SSH_CLIENT | awk '{print $1}')
    MOUNT_POINT=~/$REPOSITORY_NAME
    mkdir -p "$MOUNT_POINT"
    HOST_REPO_PATH="$MOUNT_HOST_PATH/$REPOSITORY_NAME"
    if mountpoint -q "$MOUNT_POINT"; then
        echo "[Info] Host repository already mounted at $MOUNT_POINT"
    else
        sshfs -o reconnect,allow_other "$MOUNT_USERNAME@$PC_IP:$HOST_REPO_PATH" "$MOUNT_POINT" || echo "[Error] Failed to mount host repo"
    fi
    #rsync -av --exclude-from='exclude-list.txt' "$MOUNT_POINT/" /path/in/container/
fi

# Pull latest changes from GitHub if --pull flag is used
if [ "$GITHUB_PULL" = true ]; then
    git config --global user.email "benjamin.peterson@student.nmt.edu"
    git config --global user.name "benjamin-p15"
    git remote add origin git@github.com:NMT-Lunabotics/NMT-Lunabotics2025-Python-based.git 2>/dev/null || true
    git fetch origin
    git reset --hard origin/main
    git clean -fdx
    RESTART_CONTAINER=true
    BUILD_IMAGE=true
fi


# Build image if needed or if --build flag is used
if [ "$BUILD_IMAGE" = true ] || ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
    echo "Building Docker image: $IMAGE_NAME"
    docker build -t $IMAGE_NAME .
fi

# If --restart flag is used, restart all containors
if [ "$RESTART_CONTAINER" = true ]; then
    OLD_CONTAINERS=$(docker ps -aq)  
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

# Start main system control python script if --start flag is used 
if [ "$START_SYSTEM_CONTROL" = true ]; then
    echo "Starting main system control script..."
    docker exec -it $CONTAINER_ID python3 system_operations/main/main.py
    exit 0
fi

# Attach to container shell
docker exec -it $CONTAINER_ID bash