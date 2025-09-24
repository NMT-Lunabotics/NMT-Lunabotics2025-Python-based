import os
import time

# Get the USER_ID from environment
USER_ID = int(os.environ.get("USER_ID", 0))

# Each user has their own port (optional if you want network isolation)
PORT = 5000 + USER_ID

print(f"[USER {USER_ID}] Starting heartbeat on port {PORT}...")

try:
    while True:
        print(f"[USER {USER_ID}] Alive at {time.strftime('%H:%M:%S')}")
        time.sleep(2)
except KeyboardInterrupt:
    print(f"[USER {USER_ID}] Stopping heartbeat.")
