#!/bin/bash
set -e

# config
SKETCH_PATH="$HOME/NMT-Lunabotics2025-Python-based/system_operations/system_control/system_control.ino"
#SKETCH_PATH="$HOME/NMT-Lunabotics2025-Python-based/system_operations/component_tests/system_control_led/system_control_led.ino"
IMAGE_NAME="luna/python-arduino-upload:latest"
CONTAINER_NAME="temp_arduino_containor"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
HASH_FILE="$SCRIPT_DIR/arduino.sketch_hash"
DOCKERFILE_NAME="Dockerfile.arduino"

SKETCH_NAME=$(basename "$SKETCH_PATH" .ino)
SKETCH_DIR=$(dirname "$SKETCH_PATH")
ORIGINAL_SKETCH="$SKETCH_NAME.ino"

# Compute current hash of all sketch files
TEMP_HASH_FILE=$(mktemp)
find "$SKETCH_DIR" -type f \( -name "*.ino" -o -name "*.h" -o -name "*.hpp" -o -name "*.cpp" \) -exec sha256sum {} \; | sort > "$TEMP_HASH_FILE"
CURRENT_HASH=$(sha256sum "$TEMP_HASH_FILE" | awk '{print $1}')

# Check if previous hash exists, if one does and matches do not recompile arduino code
if [ -f "$HASH_FILE" ]; then
    PREV_HASH=$(cat "$HASH_FILE")
else
    PREV_HASH=""
fi

if [ "$CURRENT_HASH" == "$PREV_HASH" ]; then
    echo "Arduino sketch up to date."
    exit 0
fi

# Automaticlly detect arduino port
PORT=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n1)
if [ -z "$PORT" ]; then
    echo "No Arduino detected on port: $PORT!"
    exit 0
fi

# Automaticlly detect arduino board
BOARD_FQBN="arduino:avr:mega"
VIDPID=$(udevadm info -q property -n "$PORT" | grep ID_MODEL_ID || true)
if [[ "$VIDPID" == *"0043"* ]] || [[ "$VIDPID" == *"6001"* ]]; then
    BOARD_FQBN="arduino:avr:uno"
fi

# Create a temp folder for arduino files
TEMP_DIR=$(mktemp -d)
TEMP_SKETCH_DIR="$TEMP_DIR/$SKETCH_NAME"
mkdir -p "$TEMP_SKETCH_DIR"

# Copy files to blank folder
cp "$SKETCH_PATH" "$TEMP_SKETCH_DIR/"
find "$SKETCH_DIR" -type f \( -name "*.h" -o -name "*.hpp" -o -name "*.cpp" \) -exec cp {} "$TEMP_SKETCH_DIR/" \;

# Depending on board type auto update script mode
if [[ "$BOARD_FQBN" == "arduino:avr:uno" ]]; then
    NEW_VALUE=0
else
    NEW_VALUE=1
fi
sed -i "s/#define MAIN_ROBOT [01]/#define MAIN_ROBOT $NEW_VALUE/" "$TEMP_SKETCH_DIR/$ORIGINAL_SKETCH"

# Build docker image
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
    docker build -t $IMAGE_NAME -f "$SCRIPT_DIR/$DOCKERFILE_NAME" "$SCRIPT_DIR"
fi

# Upload sketch with timeout
set +e
CONTAINER_ID=$(docker run --rm \
  --name "$CONTAINER_NAME" \
  --device="$PORT:$PORT" \
  -v "$TEMP_DIR":/workspace \
  -w /workspace/$SKETCH_NAME \
  "$IMAGE_NAME" \
  bash -c "set -e; arduino-cli compile --fqbn $BOARD_FQBN $ORIGINAL_SKETCH; arduino-cli upload -p $PORT --fqbn $BOARD_FQBN $ORIGINAL_SKETCH")

# Wait up to 10s, kill if still running
for i in {1..10}; do
    sleep 1
    if ! docker ps -q --no-trunc | grep -q "$CONTAINER_ID"; then break; fi
done
docker kill "$CONTAINER_ID" >/dev/null 2>&1

UPLOAD_EXIT=$(docker inspect "$CONTAINER_ID" --format='{{.State.ExitCode}}' 2>/dev/null || echo 1)
set -e

# Print result and update hash
if [ $UPLOAD_EXIT -eq 0 ]; then
    echo "Arduino upload complete."
else
    echo "Arduino upload failed (connection to board cannot be established), uploude skipped, hash updated..."
fi
echo "$CURRENT_HASH" > "$HASH_FILE"
