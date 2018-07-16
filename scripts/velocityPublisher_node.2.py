#!/usr/bin/env python
import time
import rospy
from omnibot.msg import MotorArray


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
