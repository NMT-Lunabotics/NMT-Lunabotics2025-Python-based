# heartbeat.py
import socket
import time

PORT = 5000  # fixed port for single heartbeat

s = socket.socket()
s.bind(('0.0.0.0', PORT))
s.listen(1)

print(f"[Heartbeat] Listening on port {PORT}...")

while True:
    conn, addr = s.accept()
    conn.send(b"Heartbeat alive\n")
    conn.close()
    time.sleep(1)
