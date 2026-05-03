#!/usr/bin/env python3
import time, sys, yaml, subprocess, rclpy, glob
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robot_interfaces.msg import Camera, Command, Sequence
from geometry_msgs.msg import Twist

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
        self.deadzone=0.4  # Deadzone of actuator joystick
        self.timeout=1
        self.triggered_settings_update=0
        self.last_save_time=0

        self.default_camera_view=0
        self.second_camera_view=1

        self.arm_actuator_direction=0
        self.bucket_actuator_direction=0
        self.motor_lateral_direction=0
        self.motor_sideways_direction=0

        self.name_remapping = {
            "MOTOR_X": "RIGHT_JOY_X",                     # Turn left/right
            "MOTOR_Y": "RIGHT_JOY_Y",                     # Drive forword/backwords
            "ACTUATOR_X": "LEFT_JOY_X",                   # Move bucket actuator up/down
            "ACTUATOR_Y": "LEFT_JOY_Y",                   # Move arm actuators up/down  
            "ARM": "LEFT_BUMPER",                         # Arming button
            "POS1_SAVE": "RIGHT_BUMPER",                  # Save pos for mapping 
            "NAV_TRIGGER": "RIGHT_TRIGGER",               # Trigger navigation cycle 
            "CAMERA_TOGGLE": "RIGHT_STICK_BUTTON",        # Toggle between camera views
            "LEFT_STICK": "LEFT_STICK_BUTTON",            # Extra
            "CAMERA_SWITCH": "START",                     # Switch between cameras
            "AUTOMATED": "BACK",                          # Trigger automation 

            "A_BUTTON": "BUTTON_A",                       # Button A automated functions
            "B_BUTTON": "BUTTON_B",                       # Button B automated functions
            "X_BUTTON": "BUTTON_X",                       # Button X automated functions
            "Y_BUTTON": "BUTTON_Y",                       # Button Y automated functions
            "H_DPAD": "HORIZONTAL_DPAD",                  # H D-pad for automated functions
            "V_DPAD": "VERTICAL_DPAD",                    # V D-pad for automated functions
        }

        # Initalize used message types
        self.cmd_vel_msg = Twist()
        self.actuator_msg = Command()
        self.camera_msg = Camera()
        self.servo_msg = Command()
        self.estop_msg = Command()
        self.camera_msg.camera_view = self.default_camera_view

        # Load the controller schematic
        self.declare_parameter('controllers', '')
        controller_schematic=self.get_parameter('controllers').get_parameter_value().string_value
        with open(controller_schematic, 'r') as f: self.controller_schematic = yaml.safe_load(f).get('controller_input_node', {}).get('ros__parameters', {}).get('controllers', [])
        self.controller_schematic=list(self.controller_schematic.values())

        self.declare_parameter('config', '')
        controller_config=self.get_parameter('config').get_parameter_value().string_value
        with open(controller_config, 'r') as f: self.controller_config = yaml.safe_load(f).get('controller_input_node', {}).get('ros__parameters', {}).get('config', [])
        self.controller_config=list(self.controller_config.values())

        self.declare_parameter('automations', '')
        controller_automations=self.get_parameter('automations').get_parameter_value().string_value
        with open(controller_automations, 'r') as f: self.controller_automations=yaml.safe_load(f).get('controller_input_node', {}).get('ros__parameters', {}).get('automations', [])

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
        self.create_timer(0.1, self.publish_extra)    

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
        if(arm_act_vel<self.deadzone and arm_act_vel>-self.deadzone): arm_act_vel=0.0
        elif(arm_act_vel > 0): arm_act_vel=self.map_value(arm_act_vel,self.deadzone,1.0,0.0,1.0)
        else: arm_act_vel=self.map_value(arm_act_vel,-1.0,-self.deadzone,-1.0,0.0)

        bucket_act_vel=self.get_input_values(msg, "ACTUATOR_X")
        if(bucket_act_vel<self.deadzone and bucket_act_vel>-self.deadzone): bucket_act_vel=0.0
        elif(bucket_act_vel > 0): bucket_act_vel=self.map_value(bucket_act_vel,self.deadzone,1.0,0.0,1.0)
        else: bucket_act_vel=self.map_value(bucket_act_vel,-1.0,-self.deadzone,-1.0,0.0)

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
                # Triggers automated functions
                if self.get_input_values(msg, 'X_BUTTON') == 1: self.trigger_sequence(self.controller_automations["X_BUTTON"])
                elif self.get_input_values(msg, 'A_BUTTON') == 1: self.trigger_sequence(self.controller_automations["A_BUTTON"])
                elif self.get_input_values(msg, 'B_BUTTON') == 1: self.trigger_sequence(self.controller_automations["B_BUTTON"])
                elif self.get_input_values(msg, 'Y_BUTTON') == 1: self.trigger_sequence(self.controller_automations["Y_BUTTON"])
                elif self.get_input_values(msg, 'H_DPAD') == 1: self.trigger_sequence(self.controller_automations["HORIZONTAL_DPAD_LEFT"])
                elif self.get_input_values(msg, 'H_DPAD') == -1: self.trigger_sequence(self.controller_automations["HORIZONTAL_DPAD_RIGHT"])
                elif self.get_input_values(msg, 'V_DPAD') == 1: self.trigger_sequence(self.controller_automations["VERTICAL_DPAD_UP"])
                elif self.get_input_values(msg, 'V_DPAD') == -1: self.trigger_sequence(self.controller_automations["VERTICAL_DPAD_DOWN"])
                self.triggered_automation=time.time()

        if(self.get_input_values(msg, "ARM")==0 and self.get_input_values(msg, 'CAMERA_TOGGLE')==1 and self.get_input_values(msg, 'LEFT_STICK')==1):
            estop_msg = Command()
            estop_msg.command="B"
            estop_msg.data=[]
            estop_msg.blocking_id=0
            self.estop_msg=estop_msg

        if self.get_input_values(msg, "AUTOMATED") == 1 and time.time()-self.triggered_settings_update>0.3:
            # Swaps indavidual controls
            if self.get_input_values(msg, "CAMERA_SWITCH") == 1:
                if self.get_input_values(msg, 'X_BUTTON') == 1: 
                    self.motor_sideways_direction=1-self.motor_sideways_direction
                    self.triggered_settings_update=time.time()
                elif self.get_input_values(msg, 'A_BUTTON') == 1: 
                    self.arm_actuator_direction=1-self.arm_actuator_direction
                    self.triggered_settings_update=time.time()
                elif self.get_input_values(msg, 'B_BUTTON') == 1: 
                    self.bucket_actuator_direction=1-self.bucket_actuator_direction
                    self.triggered_settings_update=time.time()
                elif self.get_input_values(msg, 'Y_BUTTON') == 1: 
                    self.motor_lateral_direction=1-self.motor_lateral_direction
                    self.triggered_settings_update=time.time()

            # Swaps motor directions for backwords driving
            elif self.get_input_values(msg, 'CAMERA_TOGGLE') == 1:
                self.motor_lateral_direction=1-self.motor_lateral_direction
                self.triggered_settings_update=time.time()

            elif self.get_input_values(msg, 'MOTOR_X') != 0:
                servo_msg = Command()
                servo_msg.command="S"
                angle=self.map_value(self.get_input_values(msg, "MOTOR_X"), -1, 1, 0, 180)
                servo_msg.data=[angle]
                servo_msg.blocking_id=0
                self.servo_msg=servo_msg

            elif self.get_input_values(msg, 'POS1_SAVE') == 1 and time.time() - self.last_save_time > 0.5:
                self.last_save_time = time.time()
                subprocess.Popen(["ros2", "param", "set","/waypoint", "target_name", "berm"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.Popen(["ros2", "service", "call","/save_target_location","std_srvs/srv/SetBool","{data: true}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            elif self.get_input_values(msg, 'NAV_TRIGGER') == -1 and time.time() - self.last_save_time > 0.5:
                self.last_save_time = time.time()
                subprocess.Popen(["ros2", "param", "set","/waypoint", "nav_target_name", "berm"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.Popen(["ros2", "service", "call","/navigation_goal_target","std_srvs/srv/SetBool","{data: true}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
        # Motor velocity data
        motor=Twist()
        if self.motor_lateral_direction==1: vel=-vel
        if self.motor_sideways_direction==1: ang_vel=-ang_vel
        motor.linear.x = float(vel) 
        motor.angular.z = float(ang_vel)
        self.cmd_vel_msg=motor

        # Publish actuator data
        actuator_msg = Command()
        actuator_msg.command="A"
        if self.arm_actuator_direction==1: arm_act_vel=-arm_act_vel
        if self.bucket_actuator_direction==1: bucket_act_vel=-bucket_act_vel
        actuator_msg.data=[float(-1), float(-1), float(-1), float(-1), float(arm_act_vel), float(bucket_act_vel)]
        actuator_msg.blocking_id=0
        self.actuator_msg=actuator_msg

        # Publish camera data
        camera_msg = Camera()

        self.get_logger().error(f"{self.get_input_values(msg, 'LEFT_STICK')}")

        camera_state_actiave=self.get_input_values(msg, "CAMERA_TOGGLE")
        camera_state = self.get_input_values(msg, "CAMERA_SWITCH")
        if self.get_input_values(msg, "LEFT_STICK") == 1: camera_msg.camera_increment=True 
        
        if(camera_state_actiave==0): camera_msg.camera_view = self.default_camera_view
        else: camera_msg.camera_view = self.second_camera_view

        if(camera_state==1 and (self.time-self.last_camera_state_change).nanoseconds*1e-9>0.3):
            default=self.default_camera_view
            self.default_camera_view=self.second_camera_view
            self.second_camera_view=default
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

    def publish_extra(self):
        if self.servo_msg != None:
            self.robot_command_publisher.publish(self.servo_msg)
            self.servo_msg=None
        if self.estop_msg != None:
            self.robot_command_publisher.publish(self.estop_msg)
            self.estop_msg=None
        self.camera_state_publisher.publish(self.camera_msg)

    # Get data dynamiclly from joystick using input type
    def get_input_values(self, msg, input):
        button = self.name_remapping[input]
        if button in self.active_controller["axes"]:
            idx = self.active_controller["axes"][button]
            return msg.axes[idx] if idx < len(msg.axes) else 0.0
        elif button in self.active_controller["buttons"]:
            idx = self.active_controller["buttons"][button]
            return msg.buttons[idx] if idx < len(msg.buttons) else 0
        else: return 0

    # Constantly check to make sure controller is connected
    def check_connection(self):
        self.time=self.get_clock().now()
        elapsed = (self.time - self.last_msg_time).nanoseconds * 1e-9

        # If controler is detected as timed out, set /vel to 0 and only send velocity=0 cmds
        if elapsed > self.timeout:
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