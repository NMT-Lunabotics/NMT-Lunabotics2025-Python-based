# listener.py
import socket
import time

PORT = 5000  # must match heartbeat

while True:
    try:
        s = socket.socket()
        s.connect(('127.0.0.1', PORT))
        data = s.recv(1024)
        print(f"[Listener] Received: {data.decode().strip()}")
        s.close()
        time.sleep(1)
    except ConnectionRefusedError:
        print("[Listener] Heartbeat not running yet...")
        time.sleep(1)
