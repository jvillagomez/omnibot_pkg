#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
import numpy as np
import tf
from math import cos, sin

l=1.0  # the length from the center to the wheels
class estimator():
    def __init__(self):
        self.motorstep = None
        self.estimate_state = None
        self.state = Pose()

        rospy.init_node('estimator', anonymous=False)

    def estimator_publish(self):
        a=estimator()
        pub = rospy.Publisher('state_estimate', Pose, queue_size=10)

        global estimate_state
        estimate_state = Pose()

        pub.publish(estimate_state)

        rospy.Subscriber("state_estimate", Pose, a.callback_estimator)
        rospy.Subscriber("motor_angular_displacement", Vector3, a.callback_motorstep)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            pub.publish(estimate_state)
            rate.sleep()

    def callback_estimator(self,msg):
        self.state = msg

    def callback_motorstep(self,msg):
        global estimate_state
        estimate_state = Pose()
        D=np.matrix([[msg.x],[msg.y],[msg.z]])  # the motor's angular displacements
        k1=[[-1.0/3,2.0/3,-1.0/3],
            [-np.sqrt(3)/3,0,np.sqrt(3)/3],
            [1.0/(3*l),1.0/(3*l),1.0/(3*l)]]
        quaternion=(self.state.orientation.x,
                    self.state.orientation.y,
                    self.state.orientation.z,
                    self.state.orientation.w)
                    
        orientation = list(tf.transformations.euler_from_quaternion(quaternion)) # the orientation of Pose is in quaternion, but the kinematic model is in euler angles
        theta=orientation[2] #
        R=[[cos(theta),-sin(theta),0],
            [sin(theta),cos(theta),0],
            [0,0,1]]
        mk1 = np.matrix(k1)*(0.1016/2) # first make k1 into a matrix, then times the radius of the wheel
        mR = np.matrix(R)               # make R to a matrix
        state_change=mk1*D
        state_change=mR*state_change   # pre-multiple the rotation matrix
        # kinematic model of the robot, transfer the distance travelled of each wheel to the state change of the robot
        
        temp0 = theta + state_change[2]
        temp1 = np.asarray(temp0)       # make the change in orientation to an array
        orientation[2] = float(temp1)  # make the array to a float number because we can only assign float numbers to orientation[2]
        new_quaternion = tf.transformations.quaternion_from_euler(orientation[0],orientation[1],orientation[2])
        
        estimate_state.position.x = self.state.position.x+state_change[0]
        estimate_state.position.y = self.state.position.y+state_change[1]

        estimate_state.orientation.x = new_quaternion[0]
        estimate_state.orientation.y = new_quaternion[1]
        estimate_state.orientation.z = new_quaternion[2]
        estimate_state.orientation.w = new_quaternion[3]

if __name__ == '__main__':
    try:
        msg = "hello %s"
        rospy.loginfo(msg)
        a=estimator()
        a.estimator_publish()

    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("estimator terminated.")
