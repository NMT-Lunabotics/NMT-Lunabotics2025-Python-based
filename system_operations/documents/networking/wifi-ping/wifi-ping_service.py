#!/usr/bin/env python3

import socket
import json

DISCOVERY_PORT = 10000
DISCOVERY_MAGIC = "NMT_IP_PING"

def get_local_ip():
    sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
    finally: sock.close()
    return ip

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", DISCOVERY_PORT))
    print("IP ping service started...")
    while True:
        data, addr = sock.recvfrom(1024)
        message = data.decode().strip()
        if message == DISCOVERY_MAGIC:
            response = {
                "ip": get_local_ip(),
                "hostname": socket.gethostname()
            }
            sock.sendto(json.dumps(response).encode(), addr)

if __name__ == "__main__":
    main()