#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from subprocess import Popen, PIPE

def publisher():
    pub = rospy.Publisher('pico_battery_voltage_topic', String, queue_size=10)
    rospy.init_node('pico_battery_voltage_node')
    rate = rospy.Rate(5) 
    while not rospy.is_shutdown():
        process = Popen(['sudo', 'i2cget', '-y', '1', '0x69', '0x08', 'w'], stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()
        process.wait()

        battery_chg = stdout.replace('0x','').replace("\n",'')
        # battery_chg = "Bat Level: %s" % battery_chg
        pub.publish(battery_chg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
