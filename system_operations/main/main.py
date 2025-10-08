import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.jetson_networking import NetworkingOperations                                        # Import jetson networking package for gui
from system_operations.arduino_cli import arduinoConsole                                                    # Import arduino cli package for compiling and uploading code to arduino
from system_operations.arduino_serial_commuication import serialCommands                                    # Import arduino serial package commucation with the arduino

arduino1_port = "/dev/ttyUSB0"  # Arduino 1 (system control)
arduino2_port = "/dev/ttyUSB1"  # Arduino 2 (rc transmitsion)

# Initialize classes with default options
arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"system_control"/"system_control.ino",board="arduino:avr:mega", port=arduino1_port)    
network = NetworkingOperations()

# Uplude latest pulled code to arduino. We uploude the code here intead of start_docker.sh or entery_point.sh to allow us to see compile errors.
arduino.compile_and_upload()
serial = serialCommands(port=arduino1_port)
serial2 = serialCommands(port=arduino2_port)

# Main loop

# Primary commands for sending data between the jetson and arduino
#-----------------------------------------------------------------
# serial.send_command("A", [-1, -1, -1, -1, 0, -10]) 
# <command, [arm_act_max_pos, arm_act_min_pos, bucket_act_max_pos, bucket_act_min_pos, arm_act_speed, bucket_act_speed]>

# serial.send_command("M", [1, -1])
# <command, [left_motor_speed, right_motor_speed]>

# serial.send_command("R", [10, 90, 1, true])
# <command, [rotation_speed, rotation_angle, rotation_radius, reset home position]>
#-----------------------------------------------------------------

#rotate=True
while True:
    #TODO robot cannot revice feedback data at same time as serial messages
    #console_message=network.receive_data()
    #if console_message: print(f"[message] {console_message}")

    #feedback = serial.read_command_feedback()
    #    if feedback:
    #        for packet in feedback:
    #            if packet["command"] == 'R': 
    #                print("Robot rotation compleated")
    #                rotate=False

    #if rotate: serial.send_command("R", [1, -90, 0, True])
    #serial.send_command("L", [0, 255, 255])

    packets = serial2.read_command_feedback()
    if packets:
        for packet in packets:
            serial.send_command(packet["command"], packet["data"])



    # Read and print out error arduino serial messages
    time.sleep(0.01)
    #serial_message = serial.read_serial()
    #if serial_message: print(serial_message)  