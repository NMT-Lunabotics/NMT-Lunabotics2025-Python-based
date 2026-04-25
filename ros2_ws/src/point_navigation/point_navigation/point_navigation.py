#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer,TransformListener
import math

class SimpleNavigator(Node):
 def __init__(self):
  super().__init__('simple_navigator')

  self.cmd_pub=self.create_publisher(Twist,'/cmd_vel',10)

  self.tf_buffer=Buffer()
  self.tf_listener=TransformListener(self.tf_buffer,self)

  self.points=[(10.0,0.0),(10.0,10.0)]
  self.index=0

  self.timer=self.create_timer(0.1,self.loop)

 def get_pose(self):
  try:
   t=self.tf_buffer.lookup_transform('odom','base_link',rclpy.time.Time())
   x=t.transform.translation.x
   y=t.transform.translation.y
   return x,y
  except:
   return None

 def loop(self):
  pose=self.get_pose()
  if pose is None:
   return

  x,y=pose
  gx,gy=self.points[self.index]

  dx=gx-x
  dy=gy-y
  dist=math.sqrt(dx*dx+dy*dy)

  cmd=Twist()

  if dist>0.3:
   cmd.linear.x=0.4
   cmd.angular.z=0.0
  else:
   cmd.linear.x=0.0
   cmd.angular.z=0.0
   self.index+=1
   if self.index>=len(self.points):
    self.get_logger().info("DONE")
    self.cmd_pub.publish(Twist())
    rclpy.shutdown()

  self.cmd_pub.publish(cmd)

def main():
 rclpy.init()
 n=SimpleNavigator()
 rclpy.spin(n)
 n.destroy_node()
 rclpy.shutdown()

if __name__=='__main__':
 main()