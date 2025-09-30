import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.jetson_networking import NetworkingOperations                                        # Import jetson networking package for gui
from system_operations.arduino_cli import arduinoConsole                                                    # Import arduino cli package for compiling and uploading code to arduino
from system_operations.arduino_serial_commuication import serialCommands                                    # Import arduino serial package commucation with the arduino

# Initialize classes with default options
arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"system_control"/"system_control.ino",baord="arduino:avr:uno")      
network = NetworkingOperations()

# Uplude latest pulled code to arduino. We uploude the code here intead of start_docker.sh or entery_point.sh to allow us to see compile errors.
arduino.compile_and_upload()
serial = serialCommands()

# Main loop
while True:
    #console_message=network.receive_data()
    #if console_message: print(f"[message] {console_message}")
    serial_message = serial.read_serial()
    if serial_message: print(serial_message)  
    time.sleep(0.01)