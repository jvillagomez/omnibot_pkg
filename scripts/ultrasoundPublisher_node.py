#!/usr/bin/python
# Publishes an integer value representing distance to target in millimeters

from time import time
from serial import Serial
import rospy
from std_msgs.msg import Int8

serialDevice = "/dev/ttyAMA0" # default for RaspberryPi
maxwait = 5 # seconds to try for a good reading before quitting


def get_measurement(portName):
    ser = Serial(portName, 9600, 8, 'N', 1, timeout=1)
    timeStart = time()
    valueCount = 0

    while time() < timeStart + maxwait:
        if ser.inWaiting():
            bytesToRead = ser.inWaiting()
            valueCount += 1
            if valueCount < 2: # 1st reading may be partial number; throw it out
                continue
            testData = ser.read(bytesToRead)
            if not testData.startswith(b'R'):
                # data received did not start with R
                continue
            try:
                sensorData = testData.decode('utf-8').lstrip('R')
            except UnicodeDecodeError:
                # data received could not be decoded properly
                continue
            try:
                mm = int(sensorData)
            except ValueError:
                # value is not a number
                continue
            ser.close()
            return(mm)

    ser.close()
    raise RuntimeError("Expected serial data not received")

def publish_measurement(portName):
    pub = rospy.Publisher('ultrasound_distance', Int8, queue_size=10)
    rospy.init_node('ultrasoundPublisher_node')
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        measurement = get_measurement(portName)

        rospy.loginfo(measurement)
        pub.publish(measurement)
        rate.sleep()

if __name__ == '__main__':
    publish_measurement(serialDevice)
    # print("distance =",measurement)
