#!/bin/bash
set -e

# The diffrent containors core image and docker file path
PYTHON_IMAGE_NAME="luna/python-robot:latest"
PYTHON_DOCKERFILE="Dockerfile.python"

ROS_IMAGE_NAME="luna/ros2-humble:latest"
ROS_DOCKERFILE="Dockerfile.ros"

ARDUINO_IMAGE_UPDATER="./system_operations/system_control/arduino_updater.sh"

# Variables which handle what containors run
RUN_PYTHON_IMAGE=false
RUN_ROS_IMAGE=true
ARDUINO_UPDATER_IMAGE=false
CONTAINER_MODE=2 #//1=python, 2=ros

# Containor settings and operations
DISPLAY_ENABLED=false               # Enable display forwording
BUILD_IMAGE=false                   # Flag tells system if it should attempt to rebuild dependencies
BUILD_SKIP=false                    # Skip the rebuild of dependencies
RESTART_CONTAINER=false             # Stop all running docker containors   
STOP_CONTAINER=false                # Stop all running containors and do not start new ones              
GITHUB_PULL=false                   # Attempt to do a github pull
LOCAL_PULL=false                    # Attempt to pull over ssh
LOCAL_USERNAME=unknown              # Default username used for ssh pull             
COMMAND_STRING=""                   # Command to execute in containor                  
QUIET_MODE=false                    # Supresse command output
ROS_DOMAIN_ID=42                    # Set domain id for network passthrough for controller 
#sudo apt install docker-buildx-plugin
export DOCKER_BUILDKIT=1 

# Ros launch files
RUN_SYSTEM_CONTROL_LAUNCH=false          # Start main system control 
RUN_TELEOP_LAUNCH=false
RUN_USB_CAMERA_NODE=false
RUN_VIEW_CAMERA_LAUNCH=false
RUN_SERIAL_LAUNCH=false

# Ros systems
APRIAL_TAG_POSE=false               # Start aprial tag system
APRIAL_TAG_POSE_DISPLAY=false       # Determines if a special video feed should be displayed

# Setup a containor mount point
MOUNT_USERNAME=false
MOUNT_HOST_PATH=false
REPOSITORY_NAME="NMT-Lunabotics2025-Python-based"
WORKING_DIR_CONTAINER="/home/luna/$REPOSITORY_NAME"
WORKING_DIR_HOST="$(pwd)"
ROS_DIR="ros2_ws"

# Use current environment variables if available, fallback to defaults
: "${DISPLAY:=$DISPLAY}"
: "${XAUTHORITY:=$HOME/.Xauthority}"

ENV_FILE="env_file.txt"

usage() {
    echo "Usage: $0 [--display | --build | --restart | --mount | --pull | --arduino | --containor | --quiet | --stop | --help] [--start | --aprial_tag | --usb-cam | --command]"
    echo "This script is used to start and manage Docker and run the whole system. You can start the script by just using $0, adding other parameters that change how the container is run and what starts up on its own."
    
    echo "Actions (pick ONE):"
    echo "  --start (-s)                           Start all processes on the robot"
    echo "  --teleop (-t)                          Run joystick control"
    echo "  --usb-cam (-u)                         Launch camera devices"
    echo "  --video-stream (-v)                    View camera video stream"

    echo "  --aprial_tag (-tag) [display|d]        Starts the aprial tag position system, and realsense camera IMU sensor"
    echo "  --command (-cmd) <command>             Execute a command inside the container without entering the container"
    echo ""
    echo "Options:"
    echo "  --display (-d)                         Enable display support (forward X11 display)"
    echo "  --build (-b) [skip|s]                  Build the Docker container (will stop the running container if any)"
    echo "  --restart (-r)                         Restart all Docker containers"
    echo "  --mount (-m) <username> <host_path>    Mounts a directory into jetson across wifi"
    echo "  --pull (-p) [local|l] <username>       Pulls the most recent files from github or pulls local files with local connection"
    echo "  --arduino (-sys)                       Force updates arduino, does not do full build"
    echo "  --container (-c) [ros|python]          Switches between entering the ros or python container with bash"
    echo "  --quiet (-q)                           Suppress bash messages"
    echo "  --stop (-x)                            Stop the running Docker container"
    echo "  --help (-h)                            Show this help message"
    exit 1
}

# Parse input flags
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -s|--start) RUN_SYSTEM_CONTROL_LAUNCH=true; shift ;;                                                                                                                                              # Start system control script
        -t|--teleop) RUN_TELEOP_LAUNCH=true; shift ;;
        -u|--usb-cam) RUN_USB_CAMERA_NODE=true; shift ;;
        -ser|--serial) RUN_SERIAL_LAUNCH=true; shift ;;
        -v|--video-stream) RUN_VIEW_CAMERA_LAUNCH=true; shift ;;
        -tag|--aprial_tag) APRIAL_TAG_POSE=true; if [[ "$2" == "display" || "$2" == "d" ]]; then APRIAL_TAG_POSE_DISPLAY=true; shift; fi; shift ;;                                                   # Starts node which publishs april tag to ros topic

        -d|--display) DISPLAY_ENABLED=true; shift ;;                                                                                                                                                 # Enable GUI forwarding
        -b|--build) BUILD_IMAGE=true; if [[ "$2" == "skip" || "$2" == "s" ]]; then BUILD_SKIP=true; shift; fi; shift ;;                                                                              # Force image rebuild
        -r|--restart) RESTART_CONTAINER=true; shift ;;                                                                                                                                               # Remove old containers first
        -p|--pull) GITHUB_PULL=true; [[ "$2" == "local" || "$2" == "l" ]] && LOCAL_PULL=true && shift; [[ -n "$2" && "$2" != -* ]] && LOCAL_USERNAME="$2" && shift; shift ;;                         # Pull github changes before building
        -h|--help) usage; shift ;;                                                                                                                                                                   # Shows help startupmation about the system
        -mm|--mount) [[ "$#" -lt 3 ]] && { echo "Error: --mount <username> <host_path>"; exit 1; }; MOUNT_USERNAME="$2"; MOUNT_HOST_PATH="$3"; shift 3 ;;                                            # Custom mount point
        -c|--container) [[ -n "$2" && ! "$2" =~ ^- ]] && case "$2" in ros) CONTAINER_MODE=2 ;; python) CONTAINER_MODE=1 ;; *) CONTAINER_MODE=0 ;; esac && shift 2 || { CONTAINER_MODE=0; shift; } ;; # Container mount mode
        -cmd|--command) COMMAND_STRING="$*"; break ;;                                                                                                                   # Execute command in container without entering
        -x|--stop) STOP_CONTAINER=true; break ;;                                                                                                                                                     # Stop all running containers
        -q|--quite) QUIET_MODE=true; shift ;;   
        -sys|--arduino) ARDUINO_UPDATER_IMAGE=true; shift ;;                                                                                                                                                       # Start bash in quite mode
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
done

# Script which handles command execution, suppress the output while in quite mode
run_cmd() {
    if [ "$QUIET_MODE" = true ]; then
        "$@" > /dev/null 2>&1
    else
        "$@"
    fi
}

# ------------------------------------------------------------
# Update container code
# ------------------------------------------------------------

# Mount to host repository --mount flag is used
if [ -n "$MOUNT_USERNAME" ] && [ -n "$MOUNT_HOST_PATH" ] && [ "$MOUNT_USERNAME" != "false" ] && [ "$MOUNT_HOST_PATH" != "false" ]; then
    PC_IP=$(echo $SSH_CLIENT | awk '{print $1}')
    MOUNT_POINT=~/$REPOSITORY_NAME
    mkdir -p "$MOUNT_POINT"
    HOST_REPO_PATH="$MOUNT_HOST_PATH/$REPOSITORY_NAME"
    if mountpoint -q "$MOUNT_POINT"; then
        echo "[STARTUP] Host repository already mounted at $MOUNT_POINT"
    else
        sshfs -o reconnect,allow_other "$MOUNT_USERNAME@$PC_IP:$HOST_REPO_PATH" "$MOUNT_POINT" || echo "[Error] Failed to mount host repo"
    fi
    #rsync -av --exclude-from='exclude-list.txt' "$MOUNT_POINT/" /path/in/container/
fi

# Pull latest changes from GitHub if --pull flag is used
if [ "$GITHUB_PULL" = true ]; then
        echo "[STARTUP] Pulling new files..."
        ./pull.sh $LOCAL_PULL $LOCAL_USERNAME
        RESTART_CONTAINER=true
        BUILD_IMAGE=true
fi

# ------------------------------------------------------------
# Build containor
# ------------------------------------------------------------

# Build image if needed or if --build flag is used
if [ "$BUILD_IMAGE" = true ] || ! docker image inspect $PYTHON_IMAGE_NAME >/dev/null 2>&1; then
    if [ "$RUN_PYTHON_IMAGE" = true ]; then
        echo "[STARTUP] Building Docker image: $PYTHON_IMAGE_NAME"
        run_cmd docker build -t $PYTHON_IMAGE_NAME -f $PYTHON_DOCKERFILE .
    fi
    if [ "$RUN_ROS_IMAGE" = true ] && [ "$BUILD_SKIP" = false ]; then
        echo "[STARTUP] Building Docker image: $ROS_IMAGE_NAME"
        run_cmd docker build --target final -t $ROS_IMAGE_NAME -f $ROS_DOCKERFILE .
    fi
    run_cmd docker image prune -f
fi

# Stop running containor when containor is stopped, restarted, or built
if [[ "$STOP_CONTAINER" == true || "$RESTART_CONTAINER" == true || ( "$BUILD_IMAGE" == true && "$BUILD_SKIP" == false ) ]]; then
    echo "[STARTUP] Stopping running containers..."
    CONTAINERS=$(docker ps -aq)
    if [ -n "$CONTAINERS" ]; then
        run_cmd docker rm -f $CONTAINERS
    fi
fi
if [ "$STOP_CONTAINER" == true ]; then
    echo "[STARTUP] Exiting..."
    exit 0
fi

# Script that can start one or more containers
start_container() {
    local IMAGE_NAME=$1
    local CONTAINER_ID
    # Check if container already exists
    CONTAINER_ID=$(docker ps -q -f ancestor="$IMAGE_NAME" | head -n1)
    if [ -z "$CONTAINER_ID" ]; then
        #echo "Starting container from image $IMAGE_NAME..."

        # Base Docker flags for privileged access, devices, working directory
        DOCKER_FLAGS=(
            --privileged                                     # Give container privileged access
            --network=host                                   # Use host networking
            #--volume=/dev:/dev:rw

            --group-add video                                # Give video group access
            --group-add dialout                              # Ensure USB serial access
            --group-add plugdev
            #--device /dev:/dev
            #--device=/dev/video:/dev/video
            -v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:ro
            -v /lib/modules:/lib/modules:ro
            -v /dev:/dev
            --device=/dev/video0
            -v /sys:/sys:ro 
            -v /run/udev:/run/udev:ro
            --device-cgroup-rule='c 81:* rmw' 
            --device-cgroup-rule='c 189:* rmw' 
            --device-cgroup-rule='c 13:* rmw'
            --device /dev/bus/usb:/dev/bus/usb                              # Map devices  
            -v $WORKING_DIR_HOST:$WORKING_DIR_CONTAINER      # Mount host folder
            -w $WORKING_DIR_CONTAINER                        # Set working directory               
        )
        

        # Enable display if requested and $DISPLAY is set
        if [ "$DISPLAY_ENABLED" = true ] && [ -n "$DISPLAY" ]; then
            # Check if we are in windows and if we are use wsl variables instead of linux ones
            if grep -qi microsoft /proc/version 2>/dev/null || [[ -n "$WSL_DISTRO_NAME" ]]; then
                DOCKER_FLAGS+=(
                    -v /tmp/.X11-unix:/tmp/.X11-unix
                    -e DISPLAY=unix:0
                )
            else
                DOCKER_FLAGS+=(
                    -e DISPLAY=$DISPLAY
                    -v $XAUTHORITY:$XAUTHORITY:ro
                    -e XAUTHORITY=$XAUTHORITY
                )
            fi
        fi

        # Start container detached
        CONTAINER_ID=$(docker run -dit -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID "${DOCKER_FLAGS[@]}" $IMAGE_NAME tail -f /dev/null)
    fi
    echo "$CONTAINER_ID"
}

# Start Python and ROS containers if requested
if [ "$RUN_PYTHON_IMAGE" = true ]; then
    echo "[STARTUP] Starting python container..."
    PY_CONTAINER_ID=$(start_container $PYTHON_IMAGE_NAME)
fi

if [ "$RUN_ROS_IMAGE" = true ]; then
    CONTAINER_ID=$(start_container $ROS_IMAGE_NAME)
    echo "[STARTUP] Starting ros container..."
    #run_cmd docker exec -it $CONTAINER_ID bash -c "source /opt/ros/humble/setup.bash"
fi

# Run command in containor
if [ -n "$COMMAND_STRING" ]; then
    echo "[STARTUP] Executing container command..."
    run_cmd docker exec -it "$CONTAINER_ID" bash -lc \
    "source /opt/ros/humble/setup.bash && \
    [ -f $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash ] && \
    source $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash || true && \
    $COMMAND_STRING"
    exit 0
fi

# Attempt arduino update cycle
if [ "$ARDUINO_UPDATER_IMAGE" = true ] || [ "$BUILD_IMAGE" = true ]; then
    echo "[STARTUP] Updating arduino code..."
    "$ARDUINO_IMAGE_UPDATER"
fi

# ------------------------------------------------------------
# Launch container and bash terminal/start commands
# ------------------------------------------------------------
if [ "$CONTAINER_MODE" = 1 ]; then
    echo "[STARTUP] Opening interactive bash terminal..."
    run_cmd docker exec -it $PY_CONTAINER_ID bash
fi
if [ "$CONTAINER_MODE" = 2 ]; then
    # Create a bashrc file for auto source in ros
    run_cmd docker exec -u 0 -it $CONTAINER_ID bash -c "\
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash' >> ~/.bashrc"

    # If containor is being build rebuild all packages at same time
    if [ "$BUILD_IMAGE" = true ] && [ "$ARDUINO_UPDATER_IMAGE" = false ]; then
        run_cmd docker exec -u 0 -it $CONTAINER_ID bash -c \
        "cd $ROS_DIR && \
        rm -rf build/ install/ log/ && \
        source /opt/ros/humble/setup.bash && \
        colcon build --symlink-install --continue-on-error"
    fi
    #--packages-skip rplidar_slam"

    # Start

    # Start teleop
    if [ "$RUN_TELEOP_LAUNCH" == true ]; then
        echo "[STARTUP] Joystick control started..."
        docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch controller_input teleop_launch.py
    # Start camera /topic
    elif [ "$RUN_USB_CAMERA_NODE" == true ]; then
        echo "[STARTUP] Camera started..."
        docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch camera usb_camera_launch.py
    # Start camera viewer
    elif [ "$RUN_VIEW_CAMERA_LAUNCH" == true ]; then
        echo "[STARTUP] Camera feed started..."
        docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch camera view_camera_launch.py
    # Start all system operations
    elif [ "$RUN_SYSTEM_CONTROL_LAUNCH" = true ]; then
        echo "[STARTUP] All robot operations started..."
        docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch system_start system_launch.py
    # Start serial communications
    elif [ "$RUN_SERIAL_LAUNCH" = true ]; then
        echo "[STARTUP] Serial communications started..."
        docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch serial_commands serial_launch.py
    # Start aprial tag system
    elif [ "$APRIAL_TAG_POSE" = true ]; then
        if [ "$APRIAL_TAG_POSE_DISPLAY" = true ]; then
            run_cmd docker exec -u 0 -it $CONTAINER_ID bash -c "\
            source /opt/ros/humble/setup.bash && \
            source $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash && \
            ros2 run aprial_tag_pose aprial_tag_pose_node.py --ros-args -p visual_display:=True"
        else
            run_cmd docker exec -u 0 -it $CONTAINER_ID bash -c "\
            source /opt/ros/humble/setup.bash && \
            source $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash && \
            ros2 run aprial_tag_pose aprial_tag_pose_node.py"
        fi
    # Open interactive bash terminal
    else
        echo "[STARTUP] Opening interactive bash terminal in the container..."
        docker exec -it -u root $CONTAINER_ID bash -c "\
        cd $ROS_DIR && \
        [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash && \
        [ -f install/setup.bash ] && source install/setup.bash && \
        exec bash"
    fi
fi