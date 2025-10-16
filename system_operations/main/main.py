import sys, time; from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent.parent; sys.path.insert(0, str(ROOT))
from system_operations.jetson_networking import NetworkingOperations                                        # Import jetson networking package for gui
from system_operations.arduino_cli import arduinoConsole                                                    # Import arduino cli package for compiling and uploading code to arduino
from system_operations.arduino_serial_commuication import serialCommands                                    # Import arduino serial package commucation with the arduino

arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"system_control"/"system_control.ino")    
#arduino = arduinoConsole(sketch_path = ROOT/"system_operations"/"nuc_motor"/"nuc_motor.ino") 

network = NetworkingOperations()
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

#while True:
    #console_message=network.receive_data()
    #if console_message: print(f"[message] {console_message}")
    #serial.send_command("M", [1, 1])

    #value = serial.read_serial()
    #if value: print(value) 

# Motor speeds
FORWARD_SPEED = 30    # Adjust to your system max speed
BACKWARD_SPEED = -30
SPIN_SPEED = 30       # For spinning, left and right are opposite

# Timing
STEP_DURATION = 1.0   # seconds
SEND_INTERVAL = 0.1   # seconds between commands

while True:
    start_time = time.time()
    
    # --- Move forward ---
    while time.time() - start_time < STEP_DURATION:
        serial.send_command("M", [FORWARD_SPEED, FORWARD_SPEED])
        time.sleep(SEND_INTERVAL)
    
    start_time = time.time()
    # --- Spin in place (right turn) ---
    while time.time() - start_time < STEP_DURATION:
        serial.send_command("M", [SPIN_SPEED, -SPIN_SPEED])
        time.sleep(SEND_INTERVAL)
    
    start_time = time.time()
    # --- Move backward ---
    while time.time() - start_time < STEP_DURATION:
        serial.send_command("M", [BACKWARD_SPEED, BACKWARD_SPEED])
        time.sleep(SEND_INTERVAL)