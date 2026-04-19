#!/usr/bin/env python3
import time, sys, yaml, subprocess, rclpy, glob
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robot_interfaces.msg import Camera, Command, Sequence
from geometry_msgs.msg import Twist

# Settings
deadzone=0.4  # Deadzone of actuator joystick

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
        self.controller_schematic=[]
        self.active_controller={}
        self.controller_name="None"
        self.triggered_automation=None
        self.automation_timeout=15

        self.name_remapping = {
            "MOTOR_X": "RIGHT_JOY_X",                     # Turn left/right
            "MOTOR_Y": "RIGHT_JOY_Y",                     # Drive forword/backwords
            "ACTUATOR_X": "LEFT_JOY_X",                   # Move bucket actuator up/down
            "ACTUATOR_Y": "LEFT_JOY_Y",                   # Move arm actuators up/down  
            "ARM": "LEFT_TRIGGER",                        # Arming button
            "CAMERA_TOGGLE": "RIGHT_STICK_BUTTON",        # Toggle between camera views
            "CAMERA_SWITCH": "START",                     # Switch between cameras
            "AUTOMATED": "BACK",                          # Trigger automation 

            "A_BUTTON": "BUTTON_A",                       # Button A used for diffrent things
            "B_BUTTON": "BUTTON_B",                       # Button B used for diffrent things
            "X_BUTTON": "BUTTON_X",                       # Button X used for diffrent things
            "Y_BUTTON": "BUTTON_Y",                       # Button Y used for diffrent things
        }

        # Initalize used message types
        self.cmd_vel_msg = Twist()
        self.actuator_msg = Command()
        self.camera_msg = Camera()
        self.camera_msg.camera_view = default_camera_view

        # Load the controller schematic
        self.declare_parameter('controller_schematic', '')
        controller_schematic_path = self.get_parameter('controller_schematic').get_parameter_value().string_value
        with open(controller_schematic_path, 'r') as f: self.controller_schematic = yaml.safe_load(f).get('controller_input_node', {}).get('ros__parameters', {}).get('controllers', [])
        self.controller_schematic=list(self.controller_schematic.values())

        # Start the /joy node to talk to controller
        try:
            self.joy_process = subprocess.Popen(['ros2', 'run', 'joy_linux', 'joy_linux_node'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"/joy node failed to start: {e}")
            sys.exit(1)

        # Create subsciption to the /joy topic, and a timer to ensure controller is connected
        self.joy = self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.create_timer(0.1, self.check_connection)

        self.create_timer(0.005, self.publish_cmd_vel)       
        self.create_timer(0.005, self.publish_actuators)    
        self.create_timer(0.1, self.publish_camera_state) 

        # Create publishers for robot commands
        self.robot_command_publisher = self.create_publisher(Command, '/robot_commands', 5)
        self.camera_state_publisher = self.create_publisher(Camera, '/camera/toggle_view', 5)
        self.automation_publisher = self.create_publisher(Sequence, '/automation_trigger', 1)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)

        self.devices = glob.glob("/dev/input/by-id/*-event-joystick")

        self.get_logger().info("\033[34mController input node started.\033[0m")

    def select_controller_schematic(self):
        for device in self.devices:
            name = device.lower()
            for schematic_device in self.controller_schematic:
                if any(key.lower() in name for key in schematic_device["identifying_keys"]):
                    self.active_controller = schematic_device
                    self.controller_name=schematic_device["identifying_keys"][0]
                    return

    # Handle joystick logic
    def joy_callback(self, msg: Joy):
        global default_camera_view, second_camera_view
        self.time = self.get_clock().now()
        
        # Connection code that helps handle timeouts
        self.last_msg_time = self.get_clock().now()

        if not self.connected:
            self.select_controller_schematic()
            self.get_logger().info(f"\033[92m{self.controller_name} controller connected.\033[0m")
            self.connected = True

        # Send x and y joystick inputs as linear and angular velocity
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

        # If unarmed or controller is disconnected send a speed of 0, 
        if self.triggered_automation != None and (self.get_input_values(msg, "ARM") == 0 or self.connected == False):
            if time.time()-self.triggered_automation>=self.automation_timeout: self.triggered_automation=None
            else:
                self.trigger_sequence("interrupt")
                self.triggered_automation=None

        if(self.get_input_values(msg, "ARM") == 0 or self.connected == False): 
            vel=0.0
            ang_vel=0.0
            arm_act_vel=0.0
            bucket_act_vel=0.0
        elif self.get_input_values(msg, "AUTOMATED") == 1:
            if self.get_input_values(msg, 'X_BUTTON') == 1: self.trigger_sequence("dig2")
            #elif self.get_input_values(msg, 'A_BUTTON') == 1: self.trigger_sequence("lateral_traverse")
            #elif self.get_input_values(msg, 'B_BUTTON') == 1: self.trigger_sequence("excavation")
            #elif self.get_input_values(msg, 'Y_BUTTON') == 1: self.trigger_sequence("dumping")
            self.triggered_automation=time.time()
            
        # Motor velocity data
        motor=Twist()
        motor.linear.x = float(vel) 
        motor.angular.z = float(ang_vel)
        self.cmd_vel_msg=motor

        # Publish actuator data
        actuator_msg = Command()
        actuator_msg.command="A"
        actuator_msg.data=[float(-1), float(-1), float(-1), float(-1), float(arm_act_vel), float(bucket_act_vel)]
        actuator_msg.blocking_id=0
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

    def trigger_sequence(self, name):
        msg = Sequence()
        msg.name = name
        msg.timestamp = time.time()
        self.automation_publisher.publish(msg)
    
    def publish_cmd_vel(self):
        self.vel_publisher.publish(self.cmd_vel_msg)

    def publish_actuators(self):
        self.robot_command_publisher.publish(self.actuator_msg)

    def publish_camera_state(self):
        self.camera_state_publisher.publish(self.camera_msg)

    # Get data dynamiclly from joystick using input type
    def get_input_values(self, msg, input):
        button=self.name_remapping[input]
        if button in self.active_controller["axes"]: return msg.axes[self.active_controller["axes"][button]]
        elif button in self.active_controller["buttons"]: return msg.buttons[self.active_controller["buttons"][button]]
        else: return 0

    # Constantly check to make sure controller is connected
    def check_connection(self):
        self.time=self.get_clock().now()
        elapsed = (self.time - self.last_msg_time).nanoseconds * 1e-9

        # If controler is detected as timed out, set /vel to 0 and only send velocity=0 cmds
        if elapsed > timeout:
            if self.connected:  
                # Warn when controller disconnects once, and set all velocityies to 0
                self.get_logger().warn("\033[93mController disconnected!\033[0m")

                # Motor velocity
                motor=Twist()
                motor.linear.x = float(0)
                motor.angular.z = float(0)
                self.cmd_vel_msg=motor
                self.vel_publisher.publish(motor)
                
                # Actuator velocity
                actuator=Command()
                actuator.command="A" 
                actuator.data=[float(-1),float(-1),float(-1),float(-1),float(0),float(0)]
                actuator.blocking_id=0
                self.actuator_msg=actuator
                self.robot_command_publisher.publish(actuator)

                self.connected = False
    
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