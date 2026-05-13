#!/usr/bin/env python3
import socket
import json
import signal
from pathlib import Path

CONFIG_FILE = Path(__file__).parent / "wifi-ping-config.json"

# Load settings
with open(CONFIG_FILE, "r") as f:
    cfg = json.load(f)

DISCOVERY_PORT = cfg["discovery_port"]
UDP_PORT = cfg["udp_port"]
COMMAND_PORT = cfg["command_port"]
TELEMETRY_PORT = cfg["telemetry_port"]
ROLE = cfg["role"]
DISCOVERY_MAGIC = cfg["discovery_magic"].encode("utf-8")

running = True
def signal_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def discovery_server():
    global running
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", DISCOVERY_PORT))  # Listen on all interfaces
    sock.settimeout(0.5)

    print(f"Robot discovery server running on UDP port {DISCOVERY_PORT}...")

    while running:
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            continue
        except OSError:
            break

        if data.strip() != DISCOVERY_MAGIC:
            continue

        response = {
            "role": ROLE,
            "udp_port": UDP_PORT,
            "command_port": COMMAND_PORT,
            "telemetry_port": TELEMETRY_PORT,
        }
        sock.sendto(json.dumps(response).encode("utf-8"), addr)
        print(f"Responded to discovery request from {addr[0]}")

    sock.close()
    print("Server stopped.")

if __name__ == "__main__":
    discovery_server()