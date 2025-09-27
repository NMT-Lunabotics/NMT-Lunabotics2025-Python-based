import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.jetson_networking import NetworkingOperations

network = NetworkingOperations()

# Main loop
while True:
    message=network.receive_data()
    if message: print(f"[message] {message}")
    time.sleep(0.01)
