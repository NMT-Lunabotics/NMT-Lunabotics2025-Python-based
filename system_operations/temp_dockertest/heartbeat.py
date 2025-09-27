#!/usr/bin/env python3
import socket

PORT = 10001  # must match COMMAND_DESTINATION port in GUI

# Create UDP socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', PORT))  # listen on all interfaces

print(f"[Heartbeat] Listening for UDP on port {PORT}...")

while True:
    try:
        data, addr = s.recvfrom(1024)  # receive up to 1024 bytes
        if data:
            print(f"[Heartbeat] Received from {addr}: {data.decode().strip()}")
    except Exception as e:
        print(f"[Heartbeat] Error: {e}")
