#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from subprocess import Popen, PIPE
import os

orange, green, blue = '0x09','0x0a','0x0b'

def kill_ros():
    os.system("killall roslaunch")
    os.system("rosnode kill -a")
    os.system("killall roscore")
    return 0

def toggleLight(color):
    process = Popen(['sudo', 'i2cget', '-y', '1', '0x6b', color, 'b'], stdout=PIPE, stderr=PIPE)
    stdout, stderr = process.communicate()
    process.wait()
    if '0x01' in stdout:
        process = Popen(['sudo', 'i2cset', '-y', '1', '0x6b', color, '0x00'])
        process.wait()
        return
    process = Popen(['sudo', 'i2cset', '-y', '1', '0x6b', color, '0x01'])
    process.wait()
    return

def publisher():
    pub = rospy.Publisher('pico_key_watcher_topic', String, queue_size=5)
    rospy.init_node('pico_key_watcher_node')
    rate = rospy.Rate(5) 
    while not rospy.is_shutdown():
        process = Popen(['sudo', 'i2cget', '-y', '1', '0x69', '0x1a', 'b'], stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()
        process.wait()
        # rospy.loginfo(stdout)
        
        if '0x00' in stdout:
            continue
        elif '0x01' in stdout:
            action = 'kill_ros' # Do action A
            kill_ros()
            # toggleLight(orange)            
        elif '0x02' in stdout:
            action = 'action-B' # Do action B
            toggleLight(green)            
        else:
            action = 'action-C' # Do action C
            toggleLight(blue)            

        process = Popen(['sudo', 'i2cset', '-y', '1', '0x69', '0x1a', '0x00'])
        process.wait()
        # action = "Perform: %s" % action
        # rospy.loginfo(action)
        pub.publish(action)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
