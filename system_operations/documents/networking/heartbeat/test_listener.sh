#!/bin/bash

PORT=10000

echo "Listening for UDP broadcasts on port $PORT..."
echo "Press Ctrl+C to stop"
echo "----------------------------------------"

sudo tcpdump -n -i any udp port $PORT -l 2>/dev/null | while read line; do
    if echo "$line" | grep -q "UDP"; then
        src_ip=$(echo "$line" | grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}\.[0-9]+' | cut -d. -f1-4 | head -1)
        sudo tcpdump -n -i any udp port $PORT -c 1 -A 2>/dev/null | while read data; do
            if echo "$data" | grep -q "NMT2026"; then
                if [[ "$data" =~ (NMT2026)[^:]*:([A-Za-z0-9]+) ]]; then
                    code="${BASH_REMATCH[2]}"
                    printf "%-15s | %-15s | %-5s | %-12s | %s\n" "$(date +"%H:%M:%S")" "$src_ip" "48473" "NMT2026" "$code"
                fi
                break
            fi
        done
    fi
done