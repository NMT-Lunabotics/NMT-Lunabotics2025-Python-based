# Example code to send command to arduino 

#import required packages and moduals and setup serial and cli tools.
import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.arduino_cli import arduinoConsole
from system_operations.arduino_serial_commuication import serialCommands

# Initialize arduino and serial classes with our deseried settings, and run our compile/upload function.
arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"system_control"/"system_control.ino")

arduino.compile_and_upload()
serial = serialCommands()

# Loop that sends our command and then reads the response for debugging.
while True:
    #<motorCommand, leftMotorSpeed, rightMotorSpeed>
    serial.send_command("M", [50, 50])
    time.sleep(0.05)
    value = serial.read_serial()
    if value: print(value)
