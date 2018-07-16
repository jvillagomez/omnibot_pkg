#!/usr/bin/env python
import time
import rospy
from omnibot.msg import MotorArray


def VelocityPublisher_talker():
    pub = rospy.Publisher('setStepMotorVelocity_topic', MotorArray, queue_size=10)
    rospy.init_node('velocityPublisher_node')
    rate = rospy.Rate(1) # 10hz
    velocity = 30.00
    while not rospy.is_shutdown():
        velocity = velocity * (-1.5)
        pub.publish(velocity,velocity,velocity)
        time.sleep(5)
        # velocity -= 1.0
        # slee
        # rospy.loginfo([velocity,velocity,velocity])
        # pub.publish(velocity,velocity,velocity)
        # rate.sleep()

if __name__ == '__main__':
    try:
        VelocityPublisher_talker()
    except rospy.ROSInterruptException:
        pass
