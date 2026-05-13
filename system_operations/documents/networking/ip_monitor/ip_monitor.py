#!/usr/bin/env python3

import socket
import time

CHECK_INTERVAL = 15

# Get current up by opening socket and then closing it
def get_ip():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except Exception:
        return None

# Run loop checking ip every CHECK_INTERVAL, printing out result 
def main():
    last_ip = None
    while True:
        current_ip = get_ip()
        if current_ip != last_ip and current_ip is not None:
            print(f"IP Address: {current_ip}", flush=True)
            last_ip = current_ip
        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()