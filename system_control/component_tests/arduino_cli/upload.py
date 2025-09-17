import subprocess
import time

sketch_path = "/home/masterpi/NMT-Lunabotics2025-Python-based/system_control/component_tests/serial_connection/recieve_message"
port = "/dev/ttyACM0"
 
# Compile the sketch
subprocess.run(
    f"arduino-cli compile --fqbn arduino:avr:uno {sketch_path}",
    shell=True,
    check=True
)

# Short delay to ensure the board is ready
time.sleep(2)

# Upload the sketch
subprocess.run(
    f"arduino-cli upload -p {port} --fqbn arduino:avr:uno --verbose {sketch_path}",
    shell=True,
    check=True
)
