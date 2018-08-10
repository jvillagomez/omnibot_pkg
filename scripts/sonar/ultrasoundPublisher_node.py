#!/usr/bin/python
# Publishes an integer value representing distance to target in millimeters

from time import time
from serial import Serial # import library allowing python to interact with serial port
import rospy
from std_msgs.msg import Int8 # distance acquired will be fit into an 8bit variable

serialDevice = "/dev/ttyAMA0" # default for RaspberryPi
maxwait = 5 # seconds to try for a good reading before quitting

# Function was obtained from the Maxbotix website.
# https://www.maxbotix.com/wp-content/uploads/2017/09/074_raspPi.txt
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
    # Declare publisher
    # Transmitting to => ultrasoundDistance_topic
    # Using the 'Int8' message type
    # A maximnum of 10 messages will be held in the message queue.
    pub = rospy.Publisher('ultrasoundDistance_topic', Int8, queue_size=10)
    rospy.init_node('ultrasoundPublisher_node')
    # 10 readings will be published per second
    rate = rospy.Rate(10) # 10hz

    # This keeps the node running until the script is shut down manually.
    # node will keep cycling at the frequncy set above.
    while not rospy.is_shutdown():
        # obtain measurement
        measurement = get_measurement(portName)

        # Log measurment to the terminal.
        rospy.loginfo(measurement)
        # publish measurmeent to the 'ultrasoundDistance_topic'.
        pub.publish(measurement)
        # Causes ROS to pause, to ensure its cycleing at the frequency set above.
        rate.sleep()

if __name__ == '__main__':
    publish_measurement(serialDevice)