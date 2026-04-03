#!/bin/bash

PORT=10000
INTERVAL=1

# Get username from /home directory or whoami command
SSH_USER=$(ls /home | head -1)
if [ -z "$SSH_USER" ]; then
    SSH_USER=$(whoami)
fi

# Create heartbeat message and publish UDP message
HEARTBEAT="NMT26:$SSH_USER"
while true; do
    echo -n "$HEARTBEAT" | socat - UDP-DATAGRAM:255.255.255.255:$PORT,broadcast
    echo "Sent: $HEARTBEAT"
    sleep $INTERVAL
done