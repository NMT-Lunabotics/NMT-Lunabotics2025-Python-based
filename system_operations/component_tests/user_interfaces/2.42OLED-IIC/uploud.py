import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.arduino_cli import arduinoConsole
from system_operations.arduino_serial_commuication import serialCommands

#arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"component_tests"/"user_interfaces"/"2.42OLED-IIC"/"screen_reset"/"screen_reset.ino", board="arduino:avr:mega")
arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"component_tests"/"user_interfaces"/"2.42OLED-IIC"/"text_example"/"text_example.ino", board="arduino:avr:mega")
serial = serialCommands()
arduino.compile_and_upload()

while True:
    value = serial.read_serial()
    if value: print(value)