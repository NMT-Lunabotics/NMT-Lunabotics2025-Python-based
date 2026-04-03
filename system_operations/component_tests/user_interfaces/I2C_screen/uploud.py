import sys
import time
from pathlib import Path

SYSTEM_OPS_DIR = Path("/home/benjamin/NMT-Lunabotics2025-Python-based/system_operations")
if not SYSTEM_OPS_DIR.exists():
    raise RuntimeError(f"system_operations folder not found at {SYSTEM_OPS_DIR}")

sys.path.insert(0, str(SYSTEM_OPS_DIR))
from arduino_cli.cli import arduinoConsole
from arduino_serial_commuication import serialCommands
SCRIPT_DIR = Path(__file__).resolve().parent
sketch_path = SCRIPT_DIR/"I2C_screen.ino"
arduino = arduinoConsole(sketch_path=sketch_path)
arduino.compile_and_upload()
serial = serialCommands()




while True:
    #value = serial.read_serial()
    #if value:
    #    print(value)
    time.sleep(0.01)
