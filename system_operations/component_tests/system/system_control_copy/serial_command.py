import sys
import time
from pathlib import Path

# -------------------------
# DYNAMIC PATH SETUP
# -------------------------
SYSTEM_OPS_DIR = Path("/home/benjamin/NMT-Lunabotics2025-Python-based/system_operations")

if not SYSTEM_OPS_DIR.exists():
    raise RuntimeError(f"system_operations folder not found at {SYSTEM_OPS_DIR}")

sys.path.insert(0, str(SYSTEM_OPS_DIR))

# -------------------------
# IMPORT MODULES
# -------------------------
try:
    from arduino_cli.cli import arduinoConsole
    from arduino_serial_commuication import serialCommands
except ModuleNotFoundError as e:
    print("Module not found:", e)
    sys.exit(1)

# -------------------------
# INITIALIZE ARDUINO
# -------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
sketch_path = SCRIPT_DIR / "system_control_copy.ino"
arduino = arduinoConsole(sketch_path=sketch_path,board="arduino:avr:mega")

arduino.compile_and_upload()

# -------------------------
# INITIALIZE SERIAL
# -------------------------
serial = serialCommands()

# -------------------------
# MAIN LOOP
# -------------------------
print("Starting serial communication loop...")

while True:
    # Example commands (uncomment to use)
    # serial.send_command("M", [1, -1])
    # serial.send_command("A", [-1, -1, -1, -1, 0, -10])
    # serial.send_command("R", [10, 90, 1, True])

    # Read serial response
    value = serial.read_serial()
    if value:
        print(value)

    time.sleep(0.01)
