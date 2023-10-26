#!/usr/bin/env python

import rospy
import time
import math
from ackermann_msgs.msg import AckermannDriveStamped

def send_ackermann_commands(duration):

    rospy.init_node("ackermann_publisher")
    
    ackermann_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=10)
    zero = False

    rate = rospy.Rate(10)  
    
    start_time = time.time()
    
    while not rospy.is_shutdown() and (time.time() - start_time) < duration:
        # Calculate elapsed time and adjust speed
        elapsed_time = time.time() - start_time
        speed = 0.0 + 27.0 * min(elapsed_time / duration, 1.0)  # Speed from 0.0 to 27.0 m/s

        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.speed = speed
        
        # Set the steering angle
        steering_amplitude = 0.3  # Maximum steering angle (radians)
        steering_frequency = 0.05  # Frequency of the sine wave (Hz)
        steering_angle = steering_amplitude * math.sin(2 * math.pi * steering_frequency * elapsed_time)
        zero = zero or (speed >= 10 and abs(steering_angle) < 0.025)
        steering_angle = 0 if zero else steering_angle

        
        ackermann_msg.drive.steering_angle = steering_angle
        
        ackermann_pub.publish(ackermann_msg)

        direction = "left" if steering_angle > 0 else "right"

        rospy.loginfo("Speed: %.2f m/s | Direction: %s | Steering angle: %.2f", speed, direction, steering_angle)

        
        rate.sleep()

if __name__ == "__main__":
    try:
        duration = 60.0  # Duration (s)
        send_ackermann_commands(duration)
    except rospy.ROSInterruptException:
        pass
