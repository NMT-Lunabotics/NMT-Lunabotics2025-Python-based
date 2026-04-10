# Example code to send command to arduino 

#import required packages and moduals and setup serial and cli tools.
import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.arduino_cli import arduinoConsole
from system_operations.arduino_serial_commuication import serialCommands

# Initialize arduino and serial classes with our deseried settings, and run our compile/upload function.
arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"component_tests"/"system"/"command_rate_checker"/"command_rate_checker.ino", board="arduino:avr:mega")
serial = serialCommands()
arduino.compile_and_upload()

while True:
    #serial.send_command("M", [10,10])


    value = serial.read_serial()
    if value: print(value)