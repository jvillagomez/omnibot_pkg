#!/usr/bin/env python
import time
import rospy
from omnibot.msg import MotorArray

# Starts the motors at 30 rpm.
# Causes the motors to alternate directions every 5 seconds.

def VelocityPublisher_talker():
    pub = rospy.Publisher('setStepMotorVelocity_topic', MotorArray, queue_size=10)
    rospy.init_node('velocityPublisher_node')
    rate = rospy.Rate(1)
    velocity = 30.00
    while not rospy.is_shutdown():
        velocity = velocity * (-1) 
        pub.publish(velocity,velocity,velocity)
        time.sleep(5)

if __name__ == '__main__':
    try:
        VelocityPublisher_talker()
    except rospy.ROSInterruptException:
        pass
