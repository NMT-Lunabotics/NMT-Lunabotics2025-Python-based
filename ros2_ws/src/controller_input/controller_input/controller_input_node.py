#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robot_interfaces.msg import Camera, Command
import subprocess
import time
import sys

# Run on local system copy and then if robot and local domain id matches they can connect

# Axes, Buttons
#Xbox_controller_map = {
#    "LEFT_JOY_X": 0, "LEFT_JOY_Y": 1, "RIGHT_JOY_X": 2, "RIGHT_JOY_Y": 3, "LEFT_TRIGGER": 4, "RIGHT_TRIGGER": 5, "HORIZONTAL_DPAD": 6, "VERTICAL_DPAD": 7,
#    "BUTTON_A": 0, "BUTTON_B": 1, "BUTTON_X": 2, "BUTTON_Y": 3, "LEFT_BUMPER": 4, "RIGHT_BUMPER": 5, "BACK": 6, "START": 7, "MODE": 8, "LEFT_STICK_BUTTON": 9, "RIGHT_STICK_BUTTON": 10, "GUIDE": 11,
#}

Xbox_controller_map = {
    "LEFT_JOY_X": 0, "LEFT_JOY_Y": 1, "RIGHT_JOY_X": 2, "RIGHT_JOY_Y": 3, "LEFT_TRIGGER": 5, "RIGHT_TRIGGER": 4, "HORIZONTAL_DPAD": 6, "VERTICAL_DPAD": 7,
    "BUTTON_A": 0, "BUTTON_B": 1, "BUTTON_X": 3, "BUTTON_Y": 4, "LEFT_BUMPER": 6, "RIGHT_BUMPER": 7, "BACK": 10, "START": 11, "GUIDE": 15, "LEFT_STICK_BUTTON": 13, "RIGHT_STICK_BUTTON": 14,
}


Logictech_controller_map = {
    "LEFT_JOY_X": 0, "LEFT_JOY_Y": 1, "RIGHT_JOY_X": 3, "RIGHT_JOY_Y": 4, "LEFT_TRIGGER": 2, "RIGHT_TRIGGER": 5, "HORIZONTAL_DPAD": 6, "VERTICAL_DPAD": 7,
    "BUTTON_A": 0, "BUTTON_B": 1, "BUTTON_X": 2, "BUTTON_Y": 3, "LEFT_BUMPER": 4, "RIGHT_BUMPER": 5, "BACK": 6, "START": 7, "LEFT_STICK_BUTTON": 9, "RIGHT_STICK_BUTTON": 10,
}

# Settings
deadzone=0.4                        # Deadzone of actuator joystick
Schematic=Xbox_controller_map       # Defines what controler schematic to use

# Button mappings
Buttons = {
    "MOTOR_X": {"type": "axis", "input": Schematic["RIGHT_JOY_X"]},                 # Turn left/right
    "MOTOR_Y": {"type": "axis", "input": Schematic["RIGHT_JOY_Y"]},                 # Drive forword/backwords
    "ACTUATOR_X": {"type": "axis", "input": Schematic["LEFT_JOY_X"]},               # Move bucket up/down
    "ACTUATOR_Y": {"type": "axis", "input": Schematic["LEFT_JOY_Y"]},               # Move main actuator up/down  
    "ARM": {"type": "axis", "input": Schematic["LEFT_TRIGGER"]},                    # Arming button
    "CAMERA_TOGGLE": {"type": "button", "input": Schematic["RIGHT_STICK_BUTTON"]},  # Toggle between camera views
    "CAMERA_SWITCH": {"type": "button", "input": Schematic["START"]},               # Switch between camera views
}

# Button combos: 
# RIGHT_BUMPER+[BUTTON_A,BUTTON_B,BUTTON_X,BUTTON_Y] triggers automation
# LEFT_BUMPER+(LEFT_STICK_BUTTON and RIGHT_STICK_BUTTON) gets robot out of error mode
# LEFT_BUMPER+(BUTTON_Y) shows ip address on screen
# LEFT_BUMPER+(BUTTON_X) Shows last error arduino if any

# GOOD IDEA?
# Save current slam map button. A way to start/stop slam mapping?

# Variables used
timeout=1
default_camera_view=0
second_camera_view=1

# Handles controller inputs
class ControllerNode(Node):
    """This acts as an data interpretation layer, all controller actions and combos are converted into topics used by the rest of teh system"""
    def __init__(self):
        super().__init__('controller_input_node')
        self.connected = False
        self.time=self.get_clock().now()
        self.last_camera_state_change=self.time
        self.last_msg_time = self.time

        # Initalize used message types
        self.cmd_vel_msg = Command()
        self.actuator_msg = Command()
        
        self.camera_msg = Camera()
        self.camera_msg.camera_view = default_camera_view

        # Start the /joy node to talk to controller
        try:
            self.joy_process = subprocess.Popen(['ros2', 'run', 'joy', 'joy_node'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"/joy node failed to start: {e}")
            sys.exit(1)

        # Create subsciption to the /joy topic, and a timer to ensure controller is connected
        self.joy = self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.create_timer(0.1, self.check_connection)

        self.create_timer(0.05, self.publish_cmd_vel)       
        self.create_timer(0.05, self.publish_actuators)    
        self.create_timer(0.1, self.publish_camera_state) 

        # Create publishers for robot commands
        self.robot_command_publisher = self.create_publisher(Command, '/robot_commands', 5)
        self.camera_state_publisher = self.create_publisher(Camera, '/camera/toggle_view', 5)

    # Handle joystick logic
    def joy_callback(self, msg: Joy):
        global default_camera_view, second_camera_view, Schematic
        self.time = self.get_clock().now()
        
        # Connection code that helps handle timeouts
        self.last_msg_time = self.get_clock().now()

        if not self.connected:
            controller=""
            # Switch to correct button layout
            out = subprocess.check_output(["cat", "/proc/bus/input/devices"]).decode()
            for line in out.splitlines():
                lower = line.lower()
                if "name=" in lower:
                    if "logitech" in lower: 
                        Schematic=Logictech_controller_map 
                        controller="Logictech "
                    elif "xbox" in lower: 
                        Schematic=Xbox_controller_map 
                        controller="Xbox "
            self.get_logger().info(f"{controller}controller connected.")
            self.connected = True

        # Send x and y joystick inputs as linear and angular velocity
        motor=Command()
        ang_vel=self.get_input_values(msg, "MOTOR_X")
        vel=self.get_input_values(msg, "MOTOR_Y")

        # Deadzone mapping for actuator to prevent double inputs and movment issues
        arm_act_vel=self.get_input_values(msg, "ACTUATOR_Y")
        if(arm_act_vel<deadzone and arm_act_vel>-deadzone): arm_act_vel=0.0
        elif(arm_act_vel > 0): arm_act_vel=self.map_value(arm_act_vel,deadzone,1.0,0.0,1.0)
        else: arm_act_vel=self.map_value(arm_act_vel,-1.0,-deadzone,-1.0,0.0)

        bucket_act_vel=self.get_input_values(msg, "ACTUATOR_X")
        if(bucket_act_vel<deadzone and bucket_act_vel>-deadzone): bucket_act_vel=0.0
        elif(bucket_act_vel > 0): bucket_act_vel=self.map_value(bucket_act_vel,deadzone,1.0,0.0,1.0)
        else: bucket_act_vel=self.map_value(bucket_act_vel,-1.0,-deadzone,-1.0,0.0)

        # If unarmed or controller is disconnected send a speed of 0
        if(self.get_input_values(msg, "ARM") >= 0 or not self.connected): 
            vel=0.0
            ang_vel=0.0
            arm_act_vel=0.0
            bucket_act_vel=0.0
            
        # Motor velocity data
        motor.command="M"
        motor.data=[float(vel),float(ang_vel)]
        motor.blocking_id=-1
        self.cmd_vel_msg=motor

        # Publish actuator data
        actuator_msg = Command()
        actuator_msg.command="A"
        actuator_msg.data=[float(-1), float(-1), float(-1), float(-1), float(arm_act_vel), float(bucket_act_vel)]
        actuator_msg.blocking_id=-1
        self.actuator_msg=actuator_msg

        # Publish camera data
        camera_state_actiave=self.get_input_values(msg, "CAMERA_TOGGLE")
        camera_state = self.get_input_values(msg, "CAMERA_SWITCH")
        
        camera_msg = Camera()
        if(camera_state_actiave==0): camera_msg.camera_view = default_camera_view
        else: camera_msg.camera_view = second_camera_view

        if(camera_state==1 and (self.time-self.last_camera_state_change).nanoseconds*1e-9>0.3):
            default=default_camera_view
            default_camera_view=second_camera_view
            second_camera_view=default
            self.last_camera_state_change=self.time
        self.camera_msg=camera_msg
    
    def publish_cmd_vel(self):
        self.robot_command_publisher.publish(self.cmd_vel_msg)

    def publish_actuators(self):
        self.robot_command_publisher.publish(self.actuator_msg)

    def publish_camera_state(self):
        self.camera_state_publisher.publish(self.camera_msg)

    # Get data dynamiclly from joystick using input type
    def get_input_values(self, msg, input):
        button = Buttons[input]
        if(button["type"]=="axis"): return msg.axes[button["input"]]
        else: return msg.buttons[button["input"]]

    # Constantly check to make sure controller is connected
    def check_connection(self):
        self.time=self.get_clock().now()
        elapsed = (self.time - self.last_msg_time).nanoseconds * 1e-9

        # If controler is detected as timed out, set /vel to 0 and only send velocity=0 cmds
        if elapsed > timeout:
            if self.connected:  
                # Warn when controller disconnects once, and set all velocityies to 0
                self.get_logger().warn("Controller disconnected!")

                # Motor velocity
                motor=Command()
                motor.command="M"
                motor.data=[float(0),float(0)]
                motor.blocking_id=-1
                self.robot_command_publisher.publish(motor)
                self.connected = False
                # Actuator velocity
                actuator=Command()
                actuator.command="A" 
                actuator.data=[float(-1),float(-1),float(-1),float(-1),float(0),float(0)]
                actuator.blocking_id=-1
                self.robot_command_publisher.publish(actuator)
    
    # Map values to match range
    def map_value(self, x, in_min, in_max, out_min, out_max):
        x = max(min(x, in_max), in_min)
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # Destory started sub-process
    def destroy_node(self):
        if hasattr(self, 'joy_process') and self.joy_process: self.joy_process.terminate()
        super().destroy_node()

# Spin up controller_node with handling for the related situations
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()