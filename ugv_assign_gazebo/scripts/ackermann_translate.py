#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/bicycle_robot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)

def transform_callback(data):
  speed = data.drive.speed
  angle = data.drive.steering_angle
  msg = Twist()
  msg.linear.x = speed
  msg.angular.z = angle
  msg.linear.y = 0.0
  msg.linear.z = 0.0
  msg.angular.x = 0.0
  msg.angular.y = 0.0
  pub.publish(msg)
    
def ackermann_translate():
  rospy.init_node('ackermann_translate', anonymous=False)
  rospy.Subscriber("ackermann_cmd", AckermannDriveStamped, transform_callback)
  rospy.spin()

if __name__ == '__main__':
  try:
    ackermann_translate()
  except rospy.ROSInterruptException:
    pass