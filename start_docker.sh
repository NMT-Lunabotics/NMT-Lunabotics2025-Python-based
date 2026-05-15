#!/bin/bash
set -e

# --------------------------------------------------------------------------------
# Settings, handles states and file paths of everything
# --------------------------------------------------------------------------------

# Variables that handle docker images/docker file paths and state
ROS_IMAGE_NAME="luna/ros2-humble:latest"
ROS_DOCKERFILE="Dockerfile.ros"
ARDUINO_IMAGE_UPDATER="./system_operations/system_control/arduino_updater.sh"
ARDUINO_UPDATER_IMAGE=false

# Containor settings and operations
DISPLAY_ENABLED=false                                       # Enable display forwording
BUILD_IMAGE=false                                           # Flag tells system if it should attempt to rebuild dependencies
BUILD_SKIP=false                                            # Skip the rebuild of dependencies
RESTART_CONTAINER=false                                     # Stop all running docker containors   
STOP_CONTAINER=false                                        # Stop all running containors and do not start new ones              
GITHUB_PULL=false                                           # Attempt to do a github pull
LOCAL_PULL=false                                            # Attempt to pull over ssh
LOCAL_USERNAME=unknown                                      # Default username used for ssh pull             
COMMAND_STRING=""                                           # Command to execute in containor                  
QUIET_MODE=false                                            # Supresse command output
ROS_DOMAIN_ID=42                                            # Set domain id for network passthrough for controller 
FIND_IP=false                                               # Find ip of robot
IP_FIND_MODE=0                                              # Settings for ip finder
PORT=10000                                                  # Port used for ip finder
TIMEOUT=10                                                  # Timeout for ip finder
INTERACTIVE_HOST=false                                      # Run on host system instead of docker

# Ros2 launch files to launch diffrent system aspects
RUN_SYSTEM_CONTROL_LAUNCH=false                             # Start whole system
RUN_TELEOP_LAUNCH=false                                     # Start controller for sending velocity commands (local)
RUN_USB_CAMERA_NODE=false                                   # Launch system cameras
RUN_VIEW_CAMERA_LAUNCH=false                                # View camera stream (local)
RUN_SERIAL_LAUNCH=false                                     # Launch serial writter 
LAUNCH_FILE_TYPE=0;

# Settings for containor mount point, TODO depricate --> replaced by local pull system
MOUNT_USERNAME=false
MOUNT_HOST_PATH=false
REPOSITORY_NAME="NMT-Lunabotics2025-Python-based"
WORKING_DIR_CONTAINER="/home/luna/$REPOSITORY_NAME"
WORKING_DIR_HOST="$(pwd)"
ROS_DIR="ros2_ws"



#sudo apt install docker-buildx-plugin
export DOCKER_BUILDKIT=1 
# Use current environment variables if available, fallback to defaults
: "${DISPLAY:=$DISPLAY}"
: "${XAUTHORITY:=$HOME/.Xauthority}"
# Store system variables
ENV_FILE="env_file.txt"

# --------------------------------------------------------------------------------
# System command code, commands and help info starting diffrent system aspects
# --------------------------------------------------------------------------------

usage() {
    echo "Usage: $0 [--display | --build | --restart | --mount | --pull | --arduino | --containor | --quiet | --stop | --help] [--start | --aprial_tag | --usb-cam | --command]"
    echo "This script is used to start and manage Docker and run the whole system. You can start the script by just using $0, adding other parameters that change how the container is run and what starts up on its own."
    
    echo "Actions (pick ONE):"
    echo "  --start (-s) [l|local]                 Start all processes on the robot"
    echo "  --teleop (-t)                          Run joystick control"
    echo "  --usb-cam (-u)                         Launch camera devices"
    echo "  --video-stream (-v)                    View camera video stream"
    echo "  --command (-cmd) <command>             Execute a command inside the container without entering the container"
    echo " "
    echo "Options:"
    echo "  --host (-i)                            Runs system without docker, makes system less modual but allows it to bypass SDK issues"
    echo "  --display (-d)                         Enable display support (forward X11 display)"
    echo "  --build (-b) [skip|s]                  Build the Docker container (will stop the running container if any)"
    echo "  --restart (-r)                         Restart all Docker containers"
    echo "  --pull (-p) [local|l] <username>       Pulls the most recent files from github or pulls local files with local connection"
    echo "  --ip_address (-ip) [f|find|a|all|p|ping]      Automaticlly finds robot ip using pinging system and ssh's into it"
    echo "  --mount (-m) <username> <host_path>    Mounts a directory into jetson across wifi"
    echo "  --arduino (-sys)                       Force updates arduino, does not do full build"
    echo "  --quiet (-q)                           Suppress bash messages"
    echo "  --stop (-x)                            Stop the running Docker container"
    echo "  --help (-h)                            Show this help message"
    exit 1
}

# Parse input flags
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -s|--start) RUN_SYSTEM_CONTROL_LAUNCH=true; [[ -n "$2" && ( "$2" == "local" || "$2" == "l" ) ]] && { LAUNCH_FILE_TYPE=1; shift 2; } || { [[ -n "$2" && ( "$2" == "nav" || "$2" == "n" ) ]] && { LAUNCH_FILE_TYPE=2; shift 2; } || shift 1; } ;;                                          # Launches whole system 
        -t|--teleop) RUN_TELEOP_LAUNCH=true; shift ;;                                                                                                                                                # Launches ros joystick (local)
        -u|--usb-cam) RUN_USB_CAMERA_NODE=true; shift ;;                                                                                                                                             # Launches all system cameras
        -ser|--serial) RUN_SERIAL_LAUNCH=true; shift ;;                                                                                                                                              # Launches serial talker
        -v|--video-stream) RUN_VIEW_CAMERA_LAUNCH=true; shift ;;                                                                                                                                     # Launches camera stream viewer (local)

        -i|--host) INTERACTIVE_HOST=true; shift ;;   
        -ip|--ip_address) FIND_IP=true; case "$2" in find|f) IP_FIND_MODE=1; shift ;; all|a) IP_FIND_MODE=2; shift ;; ping|p) IP_FIND_MODE=3; shift ;; *) IP_FIND_MODE=0 ;; esac; shift ;;           # Enabled system to auto ssh into system, list robot ip, or all ips
        -d|--display) DISPLAY_ENABLED=true; shift ;;                                                                                                                                                 # Enable GUI forwarding
        -b|--build) BUILD_IMAGE=true; if [[ "$2" == "skip" || "$2" == "s" ]]; then BUILD_SKIP=true; shift; fi; shift ;;                                                                              # Force image rebuild
        -r|--restart) RESTART_CONTAINER=true; shift ;;                                                                                                                                               # Remove old containers first
        -p|--pull) GITHUB_PULL=true; [[ "$2" == "local" || "$2" == "l" ]] && LOCAL_PULL=true && shift; [[ -n "$2" && "$2" != -* ]] && LOCAL_USERNAME="$2" && shift; shift ;;                         # Pull github changes before building
        -h|--help) usage; shift ;;                                                                                                                                                                   # Shows help startupmation about the system
        -mm|--mount) [[ "$#" -lt 3 ]] && { echo "Error: --mount <username> <host_path>"; exit 1; }; MOUNT_USERNAME="$2"; MOUNT_HOST_PATH="$3"; shift 3 ;;                                            # Custom mount point
        -cmd|--command) COMMAND_STRING="$2"; shift 2 ;;                                                                                                                                              # Execute command in container without entering
        -x|--stop) STOP_CONTAINER=true; shift ;;                                                                                                                                                     # Stop all running containers
        -q|--quite) QUIET_MODE=true; shift ;;                                                                                                                                                        # Start bash in quite mode
        -sys|--arduino) ARDUINO_UPDATER_IMAGE=true; shift ;;                                                                                                                                         # Update arduino code
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

# --------------------------------------------------------------------------------
# Container connection code, handles networking
# --------------------------------------------------------------------------------

if [ "$FIND_IP" = true ]; then
    if [ "$IP_FIND_MODE" == 2 ]; then
        echo -e "\e[36m[STARTUP]\e[0m Scanning network for devices..."
        sudo arp-scan --localnet | grep -E '([0-9]{1,3}\.){3}[0-9]{1,3}' | while read line; do
            ip=$(echo "$line" | awk '{print $1}')
            mac=$(echo "$line" | awk '{print $2}')
            echo "Device: $ip - $mac"
        done
        exit 0
    elif [ "$IP_FIND_MODE" == 3 ]; then
        echo -e "\e[36m[STARTUP]\e[0m Listening for response to discovery ping..."

        RESULT=$(python3 -c 'import socket;PORT=11010;sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1);sock.settimeout(5);sock.sendto(b"B-NMT26",("<broadcast>",PORT));exec("try:\n data,addr=sock.recvfrom(1024)\n print(addr[0],data.decode())\nexcept:\n pass")')

        [ -z "$RESULT" ] && echo -e "\e[31m[ERROR]\e[0m No response to ping recived." && exit 1

        IP=${RESULT%% *}
        DATA=${RESULT#* }

        echo -e "\e[36m[STARTUP]\e[0m Robot found at: $IP"
        echo -e "\e[36m[STARTUP]\e[0m Response: $DATA"

        exit 0
    else
        IP=""
        USERNAME=""
        echo -e "\e[36m[STARTUP]\e[0m Listening for heatbeat ping..."
        # Listion to network heartbeat from jetson
        while read -r line; do
            if echo "$line" | grep -q "UDP"; then
                src_ip=$(echo "$line" | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}\.[0-9]+' | cut -d. -f1-4 | head -1)
                data=$(sudo tcpdump -n -i any udp port $PORT -c 1 -A 2>/dev/null)
                if echo "$data" | grep -q "NMT26"; then
                    if [[ "$data" =~ (NMT26)[^:]*:([A-Za-z0-9]+) ]]; then
                        USERNAME="${BASH_REMATCH[2]}"
                        IP="$src_ip"
                        break
                    fi
                fi
            fi
        done < <(sudo timeout $TIMEOUT tcpdump -n -i any udp port $PORT -l 2>/dev/null)
        if [ -z "$USERNAME" ]; then
            echo -e "\e[31m[ERROR]\e[0m Robot heatbeat not detected."
            exit 1
        else
            echo -e "\e[36m[STARTUP]\e[0m robot found at: $USERNAME@$IP"
            if [ "$IP_FIND_MODE" == 1 ]; then
                exit 1
            fi
            # Remove -ip argument
            NEW_ARGS=()
            for arg in "$@"; do
                if [[ "$arg" != "-ip" && "$arg" != "--ip_address" ]]; then
                    NEW_ARGS+=("$arg")
                fi
            done
            # Execute modifyeid start docker file inside of terminal
            # ssh -X -t "$USERNAME@$IP" "cd $REPOSITORY_NAME && ./$(basename "$0") ${NEW_ARGS[@]}"
        fi
        exit 0
    fi
fi

# --------------------------------------------------------------------------------
# Container update code, handles updating packages
# --------------------------------------------------------------------------------

# Mount to host repository --mount flag is used
if [ -n "$MOUNT_USERNAME" ] && [ -n "$MOUNT_HOST_PATH" ] && [ "$MOUNT_USERNAME" != "false" ] && [ "$MOUNT_HOST_PATH" != "false" ]; then
    PC_IP=$(echo $SSH_CLIENT | awk '{print $1}')
    MOUNT_POINT=~/$REPOSITORY_NAME
    mkdir -p "$MOUNT_POINT"
    HOST_REPO_PATH="$MOUNT_HOST_PATH/$REPOSITORY_NAME"
    if mountpoint -q "$MOUNT_POINT"; then
        echo -e "\e[36m[STARTUP]\e[0m Host repository already mounted at $MOUNT_POINT"
    else
        sshfs -o reconnect,allow_other "$MOUNT_USERNAME@$PC_IP:$HOST_REPO_PATH" "$MOUNT_POINT" || echo -e "\e[31m[ERROR]\e[0m Failed to mount host repo."
    fi
    #rsync -av --exclude-from='exclude-list.txt' "$MOUNT_POINT/" /path/in/container/
fi

# Pull latest changes from GitHub if --pull flag is used
if [ "$GITHUB_PULL" == true ]; then
    if [ -d "ros2_ws" ]; then
        echo -e "\e[36m[STARTUP]\e[0m Fixing file permissions for pull..."
        sudo chown -R $USER:$USER ros2_ws/ 2>/dev/null || true
        find ros2_ws -type d -exec chmod 755 {} \; 2>/dev/null || true
        find ros2_ws -type f -exec chmod 644 {} \; 2>/dev/null || true
    fi
    echo -e "\e[36m[STARTUP]\e[0m Pulling new files..."
    ./pull.sh $LOCAL_PULL $LOCAL_USERNAME
    RESTART_CONTAINER=true
    BUILD_IMAGE=true
fi

# --------------------------------------------------------------------------------
# Containor build code, handles rebuilding ros packages
# --------------------------------------------------------------------------------

if [ "$INTERACTIVE_HOST" = false ]; then
    # Build image if needed or if --build flag is used
    if [ "$BUILD_IMAGE" = true ]; then
        if [ "$BUILD_SKIP" = false ]; then
            echo -e "\e[36m[STARTUP]\e[0m Building Docker image: $ROS_IMAGE_NAME..."
            run_cmd docker build -t $ROS_IMAGE_NAME -f $ROS_DOCKERFILE . #--target final
        fi
        run_cmd docker builder prune -f
    fi


    # Stop running containor when containor is stopped, restarted, or built
    if [[ "$STOP_CONTAINER" == true || "$RESTART_CONTAINER" == true || ( "$BUILD_IMAGE" == true && "$BUILD_SKIP" == false ) ]]; then
        echo -e "\e[36m[STARTUP]\e[0m Stopping running containers..."
        CONTAINERS=$(docker ps -aq)
        if [ -n "$CONTAINERS" ]; then
            run_cmd docker rm -f $CONTAINERS
        fi
    fi
    if [ "$STOP_CONTAINER" == true ]; then
        echo -e "\e[36m[STARTUP]\e[0m Exiting..."
        exit 0
    fi
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
            --privileged                                     # Give container full privileged access
            --network=host                                   # Use host networking
            #--volume=/dev:/dev:rw

            --group-add video                                # Give video group access
            --group-add dialout                              # Ensure USB serial access
            --group-add plugdev     
            #--device /dev:/dev
            #--device=/dev/video:/dev/video
            -v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:rw
            -v /lib/modules:/lib/modules:rw
            -v /dev:/dev
            --device=/dev/video0
            -v /sys:/sys:rw
            -v /run/udev:/run/udev:rw
            --device-cgroup-rule='c 81:* rmw' 
            --device-cgroup-rule='c 189:* rmw' 
            --device-cgroup-rule='c 13:* rmw'
            --device /dev/bus/usb:/dev/bus/usb               # Map devices  
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

if [ "$INTERACTIVE_HOST" = true ]; then
    echo -e "\e[36m[STARTUP]\e[0m Running ros system on host..."

    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
    ROS_DIR_ABS="$WORKING_DIR_HOST/$ROS_DIR"

    if [ -f "$ROS_DIR_ABS/install/setup.bash" ]; then
        source "$ROS_DIR_ABS/install/setup.bash"
    fi

    if [ "$BUILD_IMAGE" = true ]; then
        cd "$ROS_DIR_ABS"
        sudo rm -rf build/ install/ log/
        source /opt/ros/humble/setup.bash
        colcon build --continue-on-error --packages-select automated_operations camera controller_input point_navigation robot_interfaces serial_commands system_monitor system_start
        source "$ROS_DIR_ABS/install/setup.bash"
        cd "$WORKING_DIR_HOST" 
    fi

    if [ -n "$COMMAND_STRING" ]; then
        echo -e "\e[36m[STARTUP]\e[0m Executing command..."
        eval "$COMMAND_STRING"
        exit 0
    fi

    if [ "$ARDUINO_UPDATER_IMAGE" == true ] || [ "$BUILD_IMAGE" == true ]; then
        echo -e "\e[36m[STARTUP]\e[0m Updating arduino code..."
        #"$ARDUINO_IMAGE_UPDATER"
    fi

    if [ "$RUN_TELEOP_LAUNCH" == true ]; then
        #export ROS_DOMAIN_ID=0
        #ros2 run domain_bridge domain_bridge /home/benjamin/NMT-Lunabotics2025-Python-based/ros2_ws/src/controller_input/config/camera_bridge.yaml & 
        
        ros2 launch controller_input teleop_launch.py 
    elif [ "$RUN_USB_CAMERA_NODE" == true ]; then
        ros2 launch camera usb_camera_launch.py
    elif [ "$RUN_VIEW_CAMERA_LAUNCH" == true ]; then
        ros2 launch camera view_camera_launch.py
    elif [ "$RUN_SYSTEM_CONTROL_LAUNCH" == true ]; then
        if [ "$LAUNCH_FILE_TYPE" == 0 ]; then
            ros2 launch system_start system_launch.py
        elif [ "$LAUNCH_FILE_TYPE" == 1 ]; then
            ros2 launch system_start system_user_interface_launch.py
        elif [ "$LAUNCH_FILE_TYPE" == 2 ]; then
            ros2 launch system_start system_navigation_launch.py nav_stream:="true"
        fi
    elif [ "$RUN_SERIAL_LAUNCH" == true ]; then
        ros2 launch serial_commands serial_launch.py
    else
        exec bash
    fi

    exit 0
fi

# Attempt arduino update cycle
if [ "$ARDUINO_UPDATER_IMAGE" = true ] || [ "$BUILD_IMAGE" = true ]; then
    echo -e "\e[36m[STARTUP]\e[0m Updating arduino code..."
    "$ARDUINO_IMAGE_UPDATER"
fi

CONTAINER_ID=$(start_container $ROS_IMAGE_NAME)
echo -e "\e[36m[STARTUP]\e[0m Starting ros container..."

# If containor is being build rebuild all packages at same time
if [ "$BUILD_IMAGE" = true ] && [ "$ARDUINO_UPDATER_IMAGE" = false ] && [ "$INTERACTIVE_HOST" = false ]; then
    run_cmd docker exec -u 0 -it $CONTAINER_ID bash -c \
    "cd $ROS_DIR && \
    rm -rf build/ install/ log/ && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --continue-on-error"
fi
#run_cmd docker exec -it $CONTAINER_ID bash -c "source /opt/ros/humble/setup.bash"

# Run command in containor
if [ -n "$COMMAND_STRING" ]; then
    echo -e "\e[36m[STARTUP]\e[0m Executing container command..."
    run_cmd docker exec -it "$CONTAINER_ID" bash -lc \
    "source /opt/ros/humble/setup.bash && \
    [ -f $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash ] && \
    source $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash || true && \
    $COMMAND_STRING"
    exit 0
fi

# Create a bashrc file for auto source in ros
run_cmd docker exec -u 0 -it $CONTAINER_ID bash -c "\
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
echo 'source $WORKING_DIR_CONTAINER/$ROS_DIR/install/setup.bash' >> ~/.bashrc"

#--packages-skip rplidar_slam"

# --------------------------------------------------------------------------------
# Launch commands, starts ros system components
# --------------------------------------------------------------------------------

# All telop/serial topics use on DOMAIN_ID=10, All camera topics use on DOMAIN_ID=11

# Start teleop
if [ "$RUN_TELEOP_LAUNCH" == true ]; then
    echo -e "\e[36m[STARTUP]\e[0m Joystick control started..."
    docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch controller_input teleop_launch.py
# Start camera /topic
elif [ "$RUN_USB_CAMERA_NODE" == true ]; then
    echo -e "\e[36m[STARTUP]\e[0m Camera started..."
    docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch camera usb_camera_launch.py
# Start camera viewer
elif [ "$RUN_VIEW_CAMERA_LAUNCH" == true ]; then
    echo -e "\e[36m[STARTUP]\e[0m Camera feed started..."
    docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch camera view_camera_launch.py
# Start all system operations
elif [ "$RUN_SYSTEM_CONTROL_LAUNCH" = true ]; then
    echo -e "\e[36m[STARTUP]\e[0m All robot operations started..."
    if [ "$LAUNCH_FILE_TYPE" == 0 ]; then
        docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch system_start system_launch.py
    else 
        docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch system_start system_user_interface_launch.py
    fi
# Start serial communications
elif [ "$RUN_SERIAL_LAUNCH" = true ]; then
    echo -e "\e[36m[STARTUP]\e[0m Serial communications started..."
    docker exec -it -u root --env-file $ENV_FILE $CONTAINER_ID /entry_point.sh ros2 launch serial_commands serial_launch.py
# Open interactive bash terminal
else
    echo -e "\e[36m[STARTUP]\e[0m Opening interactive bash terminal in the container..."
    docker exec -it -u root $CONTAINER_ID bash -c "\
    cd $ROS_DIR && \
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash && \
    [ -f install/setup.bash ] && source install/setup.bash && \
    exec bash"
fi