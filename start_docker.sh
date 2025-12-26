#!/bin/bash
set -e

# Set default flags/settings of container
PYTHON_IMAGE_NAME="luna/python-robot:latest"
PYTHON_DOCKERFILE="Dockerfile.python"

ROS_IMAGE_NAME="luna/ros2-humble:latest"
ROS_DOCKERFILE="Dockerfile.ros"

# Containor settings
DISPLAY_ENABLED=false
BUILD_IMAGE=false
RESTART_CONTAINER=false
START_SYSTEM_CONTROL=false
GITHUB_PULL=false

# Determin what containors to run
RUN_PYTHON_IMAGE=false
RUN_ROS_IMAGE=true
ARDUINO_UPDATER_IMAGE=false
CONTAINER_MODE=2 #//1=python, 2=ros


# Container directorys and mount point
MOUNT_USERNAME=false
MOUNT_HOST_PATH=false
REPOSITORY_NAME="NMT-Lunabotics2025-Python-based"
WORKING_DIR_CONTAINER="/home/luna/$REPOSITORY_NAME"
WORKING_DIR_HOST="$(pwd)"
ROS_DIR="ros2_ws"

# Ros stuff to start
APRIAL_TAG_POSE=false
APRIAL_TAG_POSE_DISPLAY=false

# Use current environment variables if available, fallback to defaults
: "${DISPLAY:=$DISPLAY}"
: "${XAUTHORITY:=$HOME/.Xauthority}"


usage() {
    echo "Usage: $0 [commands]"
    echo "This script is used to start and manage a Docker and running the whole system controller"

    echo "Options:"
    echo "  --display (-d)                         Enable display support (forward X11 display)"
    echo "  --build (-b)                           Build the Docker container (will stop the running container if any)"
    echo "  --restart (-r)                         Restart all Docker containers"
    echo "  --start (-s)                           Start the main system control loop"
    echo "  --mount (-m)                           Mounts a directory into jetson across wifi"
    echo "  --pull (-p)                            Pulls the most recent files from github"
    echo "  --containor (-c)                       Switches between entering the ros or python container with bash"
    echo "  --aprial_tag (-tag) [display|d]       Starts the aprial tag position system, and realsense camera IMU sensor"
    echo "  --help (-h)                            Show this help message"
    exit 1
}

# Parse input flags
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -d|--display) DISPLAY_ENABLED=true; shift ;;                                     # Enable GUI forwarding
        -b|--build) BUILD_IMAGE=true; shift ;;                                           # Force image rebuild
        -r|--restart) RESTART_CONTAINER=true; shift ;;                                   # Remove old containers first
        -s|--start) START_SYSTEM_CONTROL=true; shift ;;                                  # Start system control script
        -p|--pull) GITHUB_PULL=true; shift ;;                                            # Pull github changes before building
        -tag|--aprial_tag) APRIAL_TAG_POSE=true; if [[ "$2" == "display" || "$2" == "d" ]]; then APRIAL_TAG_POSE_DISPLAY=true; shift; fi; shift ;;  # Starts node which publishs april tag to ros topic
        -h|--help) usage; shift ;;                                                       # Shows help infomation about the system
        -mm|--mount) [[ "$#" -lt 3 ]] && { echo "Error: --mount <username> <host_path>"; exit 1; }; MOUNT_USERNAME="$2"; MOUNT_HOST_PATH="$3"; shift 3 ;; # Custom mount point
        -c|--container) [[ -n "$2" && ! "$2" =~ ^- ]] && case "$2" in ros) CONTAINER_MODE=2 ;; python) CONTAINER_MODE=1 ;; *) CONTAINER_MODE=0 ;; esac && shift 2 || { CONTAINER_MODE=0; shift; } ;; # Container mount mode
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
    ./pull.sh
    RESTART_CONTAINER=true
    BUILD_IMAGE=true
fi

# If --restart flag is used, restart all containors
if [ "$RESTART_CONTAINER" = true ] || [ "$BUILD_IMAGE" = true ]; then
    docker rm -f $(docker ps -aq)
fi

# Build image if needed or if --build flag is used
if [ "$BUILD_IMAGE" = true ] || ! docker image inspect $PYTHON_IMAGE_NAME >/dev/null 2>&1; then
    if [ "$RUN_PYTHON_IMAGE" = true ]; then
        echo "Building Docker image: $PYTHON_IMAGE_NAME"
        docker build -t $PYTHON_IMAGE_NAME -f $PYTHON_DOCKERFILE .
    fi
    if [ "$RUN_ROS_IMAGE" = true ]; then
        echo "Building Docker image: $ROS_IMAGE_NAME"
        docker build -t $ROS_IMAGE_NAME -f $ROS_DOCKERFILE .
    fi
    docker image prune -f
fi

start_container() {
    local IMAGE_NAME=$1
    local CONTAINER_ID
    # Check if container already exists
    CONTAINER_ID=$(docker ps -q -f ancestor="$IMAGE_NAME" | head -n1)
    if [ -z "$CONTAINER_ID" ]; then
        #echo "Starting container from image $IMAGE_NAME..."

        # Base Docker flags for privileged access, devices, working directory
        DOCKER_FLAGS=(
            --network=host                                   # Use host networking
            --privileged                                     # Give container privileged access
            --group-add video                                # Give video group access
            --group-add dialout                              # Ensure USB serial access
            --device /dev:/dev                               # Map devices  
            -v $WORKING_DIR_HOST:$WORKING_DIR_CONTAINER      # Mount host folder
            -w $WORKING_DIR_CONTAINER                        # Set working directory                 
        )

        # Enable display if requested and $DISPLAY is set
        if [ "$DISPLAY_ENABLED" = true ] && [ -n "$DISPLAY" ]; then
            DOCKER_FLAGS+=(
                -e DISPLAY=$DISPLAY
                -v $XAUTHORITY:$XAUTHORITY:ro
                -e XAUTHORITY=$XAUTHORITY
            )
        fi

        # Start container detached
        CONTAINER_ID=$(docker run -dit "${DOCKER_FLAGS[@]}" $IMAGE_NAME bash)
    fi
    echo "$CONTAINER_ID"
}

# Start Python and ROS containers if requested
if [ "$RUN_PYTHON_IMAGE" = true ]; then
    PY_CONTAINER_ID=$(start_container $PYTHON_IMAGE_NAME)
fi

if [ "$RUN_ROS_IMAGE" = true ]; then
    ROS_CONTAINER_ID=$(start_container $ROS_IMAGE_NAME)
    docker exec -it $ROS_CONTAINER_ID bash -c "source /opt/ros/humble/setup.bash"
fi

# Start main system control python script if --start flag is used 
if [ "$START_SYSTEM_CONTROL" = true ]; then
    echo "Starting main system control script..."
    docker exec -it $CONTAINER_ID python3 system_operations/main/main.py
    exit 0
fi

# Attempt arduino update cycle
if [ "$ARDUINO_UPDATER_IMAGE" = true ] || [ "$BUILD_IMAGE" = true ]; then
    ./system_operations/system_control/arduino_updater.sh
fi

# Attach to container shell
if [ "$CONTAINER_MODE" = 1 ]; then
    docker exec -it $PY_CONTAINER_ID bash
fi
if [ "$CONTAINER_MODE" = 2 ]; then
    # Create a bashrc file for auto source in ros
    docker exec -it $ROS_CONTAINER_ID bash -c "\
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash' >> ~/.bashrc"

    # If containor is being build rebuild all packages at same time
    if [ "$BUILD_IMAGE" = true ]; then
        docker exec -it $ROS_CONTAINER_ID bash -c \
        "cd $ROS_DIR && \
        rm -rf build/ install/ log/ && \
        source /opt/ros/humble/setup.bash && \
        colcon build --symlink-install --continue-on-error"
    fi

    if [ "$APRIAL_TAG_POSE" = true ]; then
        if [ "$APRIAL_TAG_POSE_DISPLAY" = true ]; then
            docker exec -it $ROS_CONTAINER_ID bash -c "\
            source /opt/ros/humble/setup.bash && \
            source $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash && \
            ros2 run aprial_tag_pose aprial_tag_pose_node.py --ros-args -p visual_display:=True"
        else
            docker exec -it $ROS_CONTAINER_ID bash -c "\
            source /opt/ros/humble/setup.bash && \
            source $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash && \
            ros2 run aprial_tag_pose aprial_tag_pose_node.py"
        fi
    fi

    # Open interactive shell terminal
    docker exec -it $ROS_CONTAINER_ID bash -c \
    "cd $ROS_DIR && \
    source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    bash"
fi