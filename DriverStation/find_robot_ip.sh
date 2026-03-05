#!/bin/bash

PORT=10000
TIMEOUT=1

# Capture the first UDP packet on the port
data=$(sudo timeout $TIMEOUT tcpdump -n -i any udp port $PORT -c 1 -A 2>/dev/null)

if echo "$data" | grep -q "NMT26"; then
    # Extract source IP
    IP=$(echo "$data" | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}' | head -1)
    echo "$IP"
    exit 0
else
    echo "NOT_FOUND"
    exit 1
fi