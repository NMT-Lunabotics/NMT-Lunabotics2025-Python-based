#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from rclpy.duration import Duration
from controller_input.msg import Actuators, Camera
import subprocess
import time
import sys

# Settings
max_vel=30.0
max_ang_vel=30.0
act_max_vel=25.0

# Mapping for xbox controller, tested on linux
Axis_map = {
    "LEFT_JOY_X": 0,        # Move bucket up/down
    "LEFT_JOY_Y": 1,        # Move main actuator up/down   
    "LEFT_TRIGGER": 2,      # Arming button
    "RIGHT_JOY_X": 3,       # Turn left/right
    "RIGHT_JOY_Y": 4,       # Drive forword/backwords
    "RIGHT_TRIGGER": 5,
    "HORIZONTAL_DPAD": 6,
    "VERTICAL_DPAD": 7,
}
Button_map = {
    "BUTTON_A": 0,
    "BUTTON_B": 1,
    "BUTTON_X": 2,
    "BUTTON_Y": 3,
    "LEFT_BUMPER": 4,
    "RIGHT_BUMPER": 5,      
    "BACK": 6,
    "START": 7,
    "MODE": 8,
    "LEFT_STICK_BUTTON": 9,
    "RIGHT_STICK_BUTTON": 10,   # Switch between camera views (hold held switches pov, double click switches main view type)
    "GUIDE": 11,
}

# Button combos: 
# RIGHT_BUMPER+[BUTTON_A,BUTTON_B,BUTTON_X,BUTTON_Y] triggers automation
# LEFT_BUMPER+(LEFT_STICK_BUTTON and RIGHT_STICK_BUTTON) gets robot out of error mode
# LEFT_BUMPER+(BUTTON_Y) shows ip address on screen
# LEFT_BUMPER+(BUTTON_X) Shows last error arduino if any

# GOOD IDEA?
# Save current slam map button. A way to start/stop slam mapping?

# Variables used
timeout=0.5
default_camera_view=0
second_camera_view=1

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_input_node')
        self.connected = False
        self.time=self.get_clock().now()

        self.last_msg_time=self.time
        self.last_camera_click=self.time
        self.last_camera_click2=0
        self.last_camera_release=self.time

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

        # Create publishers for robot commands
        self.robot_velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.actuator_velocity_publisher = self.create_publisher(Actuators, '/actuators', 10)
        self.camera_state_publisher = self.create_publisher(Camera, '/camera/toggle_view', 10)

    def joy_callback(self, msg: Joy):
        self.time = self.get_clock().now()
        
        global default_camera_view, second_camera_view
        # Connection code that helps handle timeouts
        self.last_msg_time = self.get_clock().now()
        if not self.connected:
            self.get_logger().info("Controller connected.")
            self.connected = True

        # Send x and y joystick inputs as linear and angular velocity
        twist = Twist()
        vel=msg.axes[Axis_map['RIGHT_JOY_X']]
        ang_vel=msg.axes[Axis_map['RIGHT_JOY_Y']]
        arm_act_vel=msg.axes[Axis_map['LEFT_JOY_Y']]
        bucket_act_vel=msg.axes[Axis_map['LEFT_JOY_X']]
        camera_state=msg.buttons[Button_map['RIGHT_STICK_BUTTON']]

        # If unarmed or controller is disconnected send a speed of 0
        if(msg.axes[Axis_map['LEFT_TRIGGER']] >= 0 or not self.connected): 
            vel=0
            ang_vel=0
            arm_act_vel=0
            bucket_act_vel=0
            
        # Motor velocity data
        twist.linear.x = vel*act_max_vel
        twist.angular.z = ang_vel*act_max_vel
        self.robot_velocity_publisher.publish(twist)

        # Publish actuator data
        actuator_msg = Actuators()
        actuator_msg.arm = arm_act_vel * max_vel  
        actuator_msg.bucket = bucket_act_vel * max_ang_vel
        self.actuator_velocity_publisher.publish(actuator_msg)

        # Publish camera data
        camera_msg = Camera()
        if(camera_state==0): 
            camera_msg.camera_view = default_camera_view
            self.last_camera_release=self.time

            release_time=(self.time - self.last_camera_release).nanoseconds*1e-9
            if(release_time>0.1): self.last_camera_click2=self.last_camera_click
        else: 
            release_time=(self.time - self.last_camera_release).nanoseconds*1e-9
            click_time=(self.time - self.last_camera_click).nanoseconds*1e-9
            if (release_time>0.02 and release_time<0.1) and (click_time>0.02 and click_time<0.1):
                default=default_camera_view
                default_camera_view=second_camera_view
                second_camera_view=default
            camera_msg.camera_view = second_camera_view
            self.last_camera_click=self.time
        self.camera_state_publisher.publish(camera_msg)

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
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.robot_velocity_publisher.publish(twist)
                self.connected = False
                # Actuator velocity
                actuator = Actuators()
                actuator.arm = 0.0  
                actuator.bucket = 0.0  
                self.actuator_velocity_publisher.publish(actuator)


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