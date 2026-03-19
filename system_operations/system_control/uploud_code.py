# Example code to send command to arduino 

#import required packages and moduals and setup serial and cli tools.
import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.arduino_cli import arduinoConsole
from system_operations.arduino_serial_commuication import serialCommands

# Initialize arduino and serial classes with our deseried settings, and run our compile/upload function.
arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"system_control"/"system_control.ino", board="arduino:avr:mega")
#arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"remote_rc_serial"/"host_sheild_test"/"host_sheild_test.ino", board="arduino:avr:mega")
#arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"component_tests"/"system_control_led"/"system_control_led.ino", board="arduino:avr:mega")
serial = serialCommands()
arduino.compile_and_upload()

# NOTE serial feedback is blocking, you will not recive serial error messages
#feedback = serial.read_command_feedback()
#if feedback:
#    for packet in feedback: 
#        print(packet)

time.sleep(1)
serial.send_command('D', [1, 0] + list("Example payload for an E-Stop message!".encode()))
time.sleep(0.05)
serial.send_command('D', [2, 0] + list("Simple payload".encode()))
time.sleep(0.05)
serial.send_command('D', [3, 0] + list("Less ciritcal error".encode()))
time.sleep(0.05)
serial.send_command('D', [4, 0] + list("Storage of comms data".encode()))
time.sleep(0.05)
serial.send_command('D', [8, 0] + list("Bootup status of the system".encode()))
time.sleep(0.05)
serial.send_command('D', [9, 0] + list("Extra data that the system is able to display like voltage".encode()))

time.sleep(1)
serial.send_command('D', [0, 6] + list("Example screen show message".encode()))

while True:
    #serial.send_command("S", [180])
    #time.sleep(0.5)
    #serial.send_command("S", [0])
    #time.sleep(0.5)



    value = serial.read_serial()
    if value: print(value)