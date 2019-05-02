#!/usr/bin/env python
import time
import numpy as np
import rospy
from phase1.msg import Pose
from phase1.msg import Cmd
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point




class controller():
    
    def __init__(self):
        rospy.init_node('controller2d')
        self.g = 9.80665  # [meters/sec^2]
        self.m = 0.030  # [kilograms]
        self.l = 0.046  # [meters]
        self.Ixx = 1.43e-5  # [kilogram*meters^2]
        self.Kd = [15,15,0.5]
        self.Kp = [0,0,0]
        self.yd = [5,15,0,0,0,0,0,0,0]
        self.currentpose = np.zeros(6)
        self.waypoint = Point()
        self.initialpose = Pose()
        self.desiredpose = Pose()
        self.controllerinput = cmd()
        self.controller_publisher = rospy.Publisher('phase1/controller', Cmd, queue_size=100)
        self.controller_subscriber = rospy.Subscriber('phase1/quadrotor2d', Pose, self.currentpose_callback)
        self.rate = rospy.Rate(10)

    def currentpose_callback(self,config):
        self.currentpose = self.pose2nparray(config)


    def getdistance(self,goalpose):
        return np.linalg.norm(self.currentpose-goalpose)

    def pose2array(config):
        #converts ros twist data to numpy array
        a = np.zeros(6) #empty numpy array
        a[0] = config.x
        a[1] = config.y
        a[2] = config.z
        a[3] = config.phi
        a[4] = config.theta
        a[5] = config.psi
        return a

    '''def twist2nparray(vel):
        #converts ros twist data to numpy array
        a = np.zeros(6) #empty numpy array
        a[0] = vel.linear.x
        a[1] = vel.linear.y
        a[2] = vel.linear.z
        a[3] = vel.angular.x
        a[4] = vel.angular.y
        a[5] = vel.angular.z
        return a
        '''

    def publisher(self,goalpose):
        Kd = self.kd
        yd = self.yd
        while True:
            y = self.currentpose
            e = yd - y
            u1 = self.m * (yd[7] + Kd[1] * e[4] + Kp[1] * e[1] + g)
            phid = -1 / self.g * (yd[6] + Kd[0] * e[3] + Kp[0] * e[0])
            u2 = self.Ixx * (yd[8] + Kd[2] * e[5] + Kp[2] * (phid - y[2]))
            self.controllerinput.u1 = u1
            self.controllerinput.u2 = u2
            self.controller_publisher.publish(self.controllerinput)
            self.rate.sleep()
   
    
'''    while not rospy.is_shutdown():
        rospy.spin()'''


# LIMIT ROS DATA TYPES TO CALL BACK FUNCTIONS AS MUCH AS POSSIBLE
# USE NUMPY DATA TYPES IN REST OF PROCEDURES

if __name__ == '__main__':
    try:
        Controller1 = controller()

    except rospy.ROSInterruptException:
            pass
