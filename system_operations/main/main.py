import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.jetson_networking import NetworkingOperations                                        # Import jetson networking package for gui
from system_operations.arduino_cli import arduinoConsole                                                    # Import arduino cli package for compiling and uploading code to arduino
from system_operations.arduino_serial_commuication import serialCommands                                    # Import arduino serial package commucation with the arduino

# Initialize classes with default options
arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"system_control"/"system_control.ino")      
network = NetworkingOperations()

# Uplude latest pulled code to arduino. We uploude the code here intead of start_docker.sh or entery_point.sh to allow us to see compile errors.
arduino.compile_and_upload()
serial = serialCommands()

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

rotate=True
while True:
    #console_message=network.receive_data()
    #if console_message: print(f"[message] {console_message}")

    feedback = serial.read_command_feedback()
    if feedback:
        for packet in feedback:
            if packet["command"] == 'R': 
                print("Robot rotation compleated")
                rotate=False

    if rotate: serial.send_command("R", [1, 5, 0, True])

    # Read and print out error arduino serial messages
    serial_message = serial.read_serial()
    if serial_message: print(serial_message)  
    
    time.sleep(0.01)