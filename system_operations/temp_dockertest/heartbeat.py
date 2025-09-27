import socket

PORT = 5000  # fixed port for heartbeat

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('0.0.0.0', PORT))
s.listen(1)

print(f"[Heartbeat] Listening on port {PORT}...")

while True:
    conn, addr = s.accept()
    print(f"[Heartbeat] Connection from {addr}")
    try:
        data = conn.recv(1024)  # Receive up to 1024 bytes
        if data:
            print(f"[Heartbeat] Received: {data.decode().strip()}")
    except Exception as e:
        print(f"[Heartbeat] Error: {e}")
    finally:
        conn.close()
