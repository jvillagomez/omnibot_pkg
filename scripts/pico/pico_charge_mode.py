#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from subprocess import Popen, PIPE

def publisher():
    pub = rospy.Publisher('IC_CHG_mode_topic', String, queue_size=10)
    rospy.init_node('IC_charge_Mode')
    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():
        process = Popen(['sudo', 'i2cget', '-y', '1', '0x69', '0x20', 'b'], stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()
        process.wait()

        chgMode_hex = stdout.replace('0x','')
        mode = 'Open' if '01' in chgMode_hex else 'Closed'
        
        
        power_mode = "Charge IC : %s" % mode
        rospy.loginfo(power_mode)
        pub.publish(power_mode)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass



print stdout
