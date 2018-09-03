#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from subprocess import Popen, PIPE

def publisher():
    pub = rospy.Publisher('pico_board_temp_topic', String, queue_size=10)
    rospy.init_node('pico_board_temp_node')
    rate = rospy.Rate(0.5) 
    while not rospy.is_shutdown():
        process = Popen(['sudo', 'i2cget', '-y', '1', '0x69', '0x1b', 'b'], stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()
        process.wait()

        temp = stdout.replace('0x','')
        board_temp = "Board Temp: %s" % temp
        rospy.loginfo(board_temp)
        pub.publish(board_temp)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
