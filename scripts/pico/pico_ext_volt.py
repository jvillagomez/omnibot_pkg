#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from subprocess import Popen, PIPE

def publisher():
    pub = rospy.Publisher('external_voltage_topic', String, queue_size=10)
    rospy.init_node('external_voltage')
    rate = rospy.Rate(5) 

    while not rospy.is_shutdown():
        process = Popen(['sudo', 'i2cget', '-y', '1', '0x69', '0x0c', 'w'], stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()
        process.wait()

        ext_volt = stdout.replace('0x','')
        voltage = "Ext Volt Level: %s" % ext_volt
        rospy.loginfo(voltage)
        pub.publish(voltage)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

