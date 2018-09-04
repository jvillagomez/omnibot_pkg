#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from subprocess import Popen, PIPE

voltage = 0
power_mode = 0
below_nominal = 0
above_nominal = 0
nominal_voltage = 3.2
fssd_voltage = 2.7

def notify_of_nominal():
    for i in range(5):
        toggle_buzzer()
        toggle_nominal_light()
        time.sleep(0.5)
        toggle_buzzer()
        toggle_nominal_light()
        time.sleep(0.5)    
    return

def notify_of_fssd():
    for i in range(15):
        toggle_buzzer()
        toggle_fssd_light()
        time.sleep(0.5)
        toggle_buzzer()
        toggle_fssd_light()
        time.sleep(0.5)    
    return

def update_voltage(data):
    voltage = int(data.data)

def update_power_mode(data):
    power_mode = data

def evaluate_depletion_status():
    if voltage<=nominal_voltage and below_nominal==0:
        below_nominal = 1
        notify_of_nominal()
        return
    elif voltage <= fssd_voltage:
        notify_of_fssd()
        return
    return

def evaluate_charge_status():
    if voltage>=nominal_voltage and above_nominal==0:
        below_nominal = 1
        notify_of_nominal()
    return
    
def listener():
    # Initialize node to handle Buzzer+LED notifications
    rospy.init_node('notification_listener')

    # Update Voltage and power mode variables
    rospy.Subscriber("pico_battery_voltage_topic", String, update_voltage)
    rospy.Subscriber("pico_power_mode_topic", String, update_power_mode)

    if(power_mode=='battery'):
        above_nominal = 0
        evaluate_depletion_status()
    else:
        below_nominal = 0
        evaluate_charge_status()
    rospy.spin()

if __name__ == '__main__':
    listener()
