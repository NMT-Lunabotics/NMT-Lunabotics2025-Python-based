import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.jetson_networking import NetworkingOperations                                        # Import jetson networking package for gui
from system_operations.arduino_cli import arduinoConsole                                                    # Import arduino cli package for compiling and uploading code to arduino
from system_operations.arduino_serial_commuication import serialCommands                                    # Import arduino serial package commucation with the arduino
from system_operations.autonomous import AutonomousRunner                                                   # Import Riley's autos system(converted into package) to run system autos

arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"system_control"/"system_control.ino")    

network = NetworkingOperations()
arduino.compile_and_upload()
serial = serialCommands()
runner = AutonomousRunner(serial)
runner.load_sequence("transverse")

def run_test():
    while True:
        start_time = time.time()
        while time.time() - start_time < 1:
            serial.send_command("M", [5, 5])
        time.sleep(0.1)
        start_time = time.time()
        while time.time() - start_time < 1:
            serial.send_command("M", [-5, -5])
        time.sleep(0.5)
        start_time = time.time()
        while time.time() - start_time < 1:
            serial.send_command("M", [5, -5])
        time.sleep(0.5)

def run():
    while True:
        serial.send_command("M", [5, 5])
        time.sleep(0.01)

def run_and_log():
    while True:
        serial.send_command("M", [5, 5])
        value = serial.read_serial()
        if value: print(value) 
        time.sleep(0.01)
        
def rotate():
    rotate=True
    while True:
        feedback = serial.read_command_feedback()
        if feedback:
            for packet in feedback:
                if packet["command"] == 'R': 
                    print("Robot rotation compleated")
                    rotate=False
        if rotate: serial.send_command("R", [0, 360, 0, True])

def read_log():
    while True:
        value = serial.read_serial()
        if value: print(value) 

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

#while True:
#    runner.update()

#run()
#run_and_log()
#run_test()
#rotate()
#read_log()