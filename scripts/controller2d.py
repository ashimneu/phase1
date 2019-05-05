#!/usr/bin/env python
import time
import rospy
import numpy as np
from phase1.msg import Pose2d
from phase1.msg import Cmd2d



class controller():
    
    def __init__(self):
        rospy.init_node('controller2d')
        self.g = 9.80665  # [meters/sec^2]
        self.m = 0.030  # [kilograms]
        self.l = 0.046  # [meters]
        self.Ixx = 1.43e-5  # [kilogram*meters^2]
        self.Kd = np.array([15,15,0.5])
        self.Kp = np.array([0,0,0])
        self.yd = np.array([5,15,0,0,0,0,0,0,0])
        self.currentpose = np.array([0,0,0,0,0,0])
        self.waypoint = np.array([0,0])
        self.initialpose = np.array([0,0,0,0,0,0])
        self.desiredpose = np.array([5,15,0,0,0,0,0,0,0])
        self.controllerinput = Cmd2d()
        self.controller_publisher = rospy.Publisher('phase1/controller', Cmd2d, queue_size=10)
        self.controller_subscriber = rospy.Subscriber('phase1/quadrotor2d', Pose2d, self.currentpose_callback)
        self.rate = rospy.Rate(10)

    def currentpose_callback(self,config):
        print('inside currentpose_callback')
        self.currentpose = self.pose2nparray(config)


    def getdistance(self,goalpose):
        return np.linalg.norm(self.currentpose-goalpose)

    def pose2nparray(self, config):
        #converts Pose2d data to numpy array
        a = np.zeros(6) #empty numpy array
        a[0] = config.y
        a[1] = config.z
        a[2] = config.phi
        a[3] = config.ydot
        a[4] = config.zdot
        a[5] = config.phidot
        return a

    def Twist2nparray(self, vel):
        #converts ros twist data to numpy array
        a = np.zeros(6) #empty numpy array
        a[0] = vel.linear.x
        a[1] = vel.linear.y
        a[2] = vel.linear.z
        a[3] = vel.angular.x
        a[4] = vel.angular.y
        a[5] = vel.angular.z
        return a



    def start(self):
        Kd = self.Kd
        Kp = self.Kp
        yd = self.yd
        while (not rospy.is_shutdown()):
            y = self.currentpose
            e = yd[0:6] - y
            u1 = self.m * (yd[7] + Kd[1] * e[4] + Kp[1] * e[1])
            phid = -1 / self.g * (yd[6] + Kd[0] * e[3] + Kp[0] * e[0])
            u2 = self.Ixx * (yd[8] + Kd[2] * e[5] + Kp[2] * (phid - y[2]))
            print('type u1 = ',type(u1))
            self.controllerinput.u1 = u1
            self.controllerinput.u2 = u2
            self.controller_publisher.publish(self.controllerinput)
            #self.rate.sleep()

        rospy.spin()
   
    
'''    while not rospy.is_shutdown():
        rospy.spin()'''


# LIMIT ROS DATA TYPES TO CALL BACK FUNCTIONS AS MUCH AS POSSIBLE
# USE NUMPY DATA TYPES IN REST OF PROCEDURES

if __name__ == '__main__':
    try:
        Controller1 = controller()
        Controller1.start()

    except rospy.ROSInterruptException:
            pass
