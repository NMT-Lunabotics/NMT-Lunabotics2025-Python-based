#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSHistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist,TransformStamped
from tf2_ros import TransformBroadcaster
import math

class ImuTFPublisher(Node):
 def __init__(self):
  super().__init__('imu_tf_publisher')

  qos=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=10)

  self.sub_imu=self.create_subscription(Imu,'/camera/camera_1/imu',self.imu_cb,qos)
  self.sub_cmd=self.create_subscription(Twist,'/cmd_vel',self.cmd_cb,10)

  self.br=TransformBroadcaster(self)

  self.yaw=0.0
  self.vx=0.0
  self.vy=0.0
  self.x=0.0
  self.y=0.0

  self.last=None

  self.cmd_lin=0.0
  self.cmd_ang=0.0

  self.bias_g=0.0
  self.bias_ax=0.0
  self.bias_ay=0.0
  self.bias_count=0
  self.bias_time=None

 def cmd_cb(self,msg):
  self.cmd_lin=msg.linear.x
  self.cmd_ang=msg.angular.z

 def imu_cb(self,msg):
  now=self.get_clock().now().nanoseconds/1e9

  if self.last is None:
   self.last=now
   return

  dt=now-self.last
  self.last=now

  if dt<=0.0 or dt>0.1:
   return

  gx=msg.angular_velocity.z
  ax=msg.linear_acceleration.x
  ay=msg.linear_acceleration.y

  # ===== BIAS CALIBRATION WHEN STOPPED =====
  if abs(self.cmd_lin)<0.01 and abs(self.cmd_ang)<0.01:
   self.bias_g+=gx
   self.bias_ax+=ax
   self.bias_ay+=ay
   self.bias_count+=1
   return

  if self.bias_count>0:
   self.bias_g/=self.bias_count
   self.bias_ax/=self.bias_count
   self.bias_ay/=self.bias_count
   self.bias_count=0
   self.get_logger().info("bias updated")

  gx-=self.bias_g
  ax-=self.bias_ax
  ay-=self.bias_ay

  # ===== YAW =====
  self.yaw+=gx*dt

  # ===== MOTION RULES =====

  if abs(self.cmd_lin)<0.01 and abs(self.cmd_ang)<0.01:
   return

  # forward-only mode (ignore y drift if no rotation)
  if abs(self.cmd_ang)<0.01 and abs(self.cmd_lin)>0.01:
   ay=0.0

  # threshold noise
  if abs(ax)<0.15: ax=0.0
  if abs(ay)<0.15: ay=0.0

  ax2=ax*math.cos(self.yaw)-ay*math.sin(self.yaw)
  ay2=ax*math.sin(self.yaw)+ay*math.cos(self.yaw)

  self.vx+=ax2*dt
  self.vy+=ay2*dt

  self.vx*=0.98
  self.vy*=0.98

  self.x+=self.vx*dt
  self.y+=self.vy*dt

  qz=math.sin(self.yaw/2)
  qw=math.cos(self.yaw/2)

  t=TransformStamped()
  t.header.stamp=self.get_clock().now().to_msg()
  t.header.frame_id='odom'
  t.child_frame_id='base_link'
  t.transform.translation.x=self.x
  t.transform.translation.y=self.y
  t.transform.translation.z=0.0
  t.transform.rotation.x=0.0
  t.transform.rotation.y=0.0
  t.transform.rotation.z=qz
  t.transform.rotation.w=qw

  self.br.sendTransform(t)

def main():
 rclpy.init()
 n=ImuTFPublisher()
 rclpy.spin(n)
 n.destroy_node()
 rclpy.shutdown()

if __name__=='__main__':
 main()