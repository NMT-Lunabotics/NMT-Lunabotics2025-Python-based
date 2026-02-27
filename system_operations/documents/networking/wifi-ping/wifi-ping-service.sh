#!/bin/bash

PORT=10000
TRIGGER="NMT_IP_PING"
RESPONSE="NMT_ROBOT_2026"

socat -T5 UDP-RECVFROM:$PORT,reuseaddr,fork SYSTEM:'
read MSG
if [ "$MSG" = "'"$TRIGGER"'" ]; then
    echo -n "'"$RESPONSE"'"
fi
'