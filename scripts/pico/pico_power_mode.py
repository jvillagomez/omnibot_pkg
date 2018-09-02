#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from subprocess import Popen, PIPE

def publisher():
    pub = rospy.Publisher('Power_mode_topic', String, queue_size=10)
    rospy.init_node('PicoPower_Mode')
    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():
        process = Popen(['sudo', 'i2cget', '-y', '1', '0x69', '0x00', 'b'], stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()
        process.wait()

        powerMode_hex = stdout.replace('0x','')
        rospy.loginfo(powerMode_hex)
        mode = 'Bat' if '02' in powerMode_hex else 'Charger'
        
        power_mode = "Power Mode : %s" % mode
        rospy.loginfo(power_mode)
        pub.publish(power_mode)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass



print stdout
