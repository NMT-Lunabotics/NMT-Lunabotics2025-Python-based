#!/bin/bash

PORT=10000
INTERVAL=1

# Assign SSH user to the pc's username
SSH_USER="luna"

# Create heartbeat message and publish UDP message
HEARTBEAT="NMT26:$SSH_USER"
while true; do
    echo -n "$HEARTBEAT" | socat - UDP-DATAGRAM:255.255.255.255:$PORT,broadcast
    echo "Sent: $HEARTBEAT"
    sleep $INTERVAL
done