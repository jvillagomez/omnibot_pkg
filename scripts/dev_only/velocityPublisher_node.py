#!/usr/bin/env python
import rospy #import ROS pip package for using ROS library
from omnibot.msg import MotorArray # import our custom ROS msg types

def VelocityPublisher_talker():
    # Declare publisher
    # Transmitting to => setStepMotorVelocity_topic
    # Using the 'MotorArray' message type
    # A maximnum of 10 messages will be held in the message queue.
    pub = rospy.Publisher('setStepMotorVelocity_topic', MotorArray, queue_size=10)

    # Initialize node
    rospy.init_node('velocityPublisher_node')

    # Sets ROS to cycle at frequency of 1Hz
    rate = rospy.Rate(1) 

    # Set uinitial velocity to 100 rpms
    velocity=100.00

    # This keeps the node running until the script is shut down manually.
    # node will keep cycling at the frequncy set above.
    while not rospy.is_shutdown(): 
        # Velocity decreases by 1 rpm every cycle
        velocity -= 1.0
        # Logs our MotorArray to the terminal
        rospy.loginfo([velocity,velocity,velocity])
        # Publishes our motorArray to the 'setStepMotorVelocity_topic'
        pub.publish(velocity,velocity,velocity)
        # Causes the while loop to pause until its time to cycle again.
        # How long it sleeps (pauses) depends on our declared frequnecy above.
        rate.sleep()

if __name__ == '__main__':
    try:
        VelocityPublisher_talker()
    except rospy.ROSInterruptException:
        pass
