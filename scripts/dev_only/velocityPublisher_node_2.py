#!/usr/bin/env python
import time
import rospy
from omnibot.msg import MotorArray

# Displays both directions for each motor in the following order.
# Velocities in either direction ar set to 30 rpm.
# Scenarios alternate every 5 seconds.

# Set -> 1
# Motor1 = CCW; Motor2 = CCW; Motor3= CCW

# Set -> 2
# Motor1 = 0; Motor2 = 0; Motor3= 0

# Set -> 3
# Motor1 = CW; Motor2 = CW; Motor3= CW

# Set -> 4
# Motor1 = CCW; Motor2 = 0; Motor3= CW

def VelocityPublisher_talker():
    pub = rospy.Publisher('setStepMotorVelocity_topic', MotorArray, queue_size=10)
    rospy.init_node('velocityPublisher_node')
    rate = rospy.Rate(1) # 10hz
    pos_velocity = 30.00
    no_velocity = 0
    neg_velocity = -30.00
    while not rospy.is_shutdown():
        time.sleep(5)
        pub.publish(pos_velocity,pos_velocity,pos_velocity)
        time.sleep(5)
        pub.publish(no_velocity,no_velocity,no_velocity)
        time.sleep(5)
        pub.publish(neg_velocity,neg_velocity,neg_velocity)
        time.sleep(5)
        pub.publish(pos_velocity,no_velocity,neg_velocity)

if __name__ == '__main__':
    try:
        VelocityPublisher_talker()
    except rospy.ROSInterruptException:
        pass
