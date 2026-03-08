#!/bin/bash

PORT=10000
TIMEOUT=3

# Capture the first UDP packet on the port with a timeout
data=$(sudo timeout $TIMEOUT tcpdump -n -i any udp port $PORT -c 1 -A 2>/dev/null)

# Try to extract the IP if the heartbeat message exists
if echo "$data" | grep -q "NMT26"; then
    # Extract source IP
    robot_ip=$(echo "$data" | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}' | head -1)
    echo "$robot_ip"
fi

# Exit silently if not found (no output)
exit 0