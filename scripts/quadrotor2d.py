#!/usr/bin/env python
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import rospy
from phase1.msg import Pose
from phase1.msg import Cmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point



''' commentss.. '''

class quadrotor():
    def __init__(self):
        rospy.init_node('quadrotor2d')
        self.sub = rospy.Subscriber('phase1/controller', Cmd, self.controllerinput_callback())
        self.quadrotor_publisher = rospy.Publisher('phase1/quadrotor2d', Pose, queue_size=100)

        self.g = 9.80665  # [meters/sec^2]
        self.m = 0.030  # [kilograms]
        self.l = 0.046  # [meters]
        self.Ixx = 1.43e-5  # [kilogram*meters^2]
        self.Kd = [15,15,0.5]
        self.Kp = [0,0,0]
        self.yd = [5,15,0,0,0,0,0,0,0]
        self.currentpose = np.zeros(6)
        self.waypoint = Point()
        self.initialpose = Twist()
        self.desiredpose = Twist()
        self.controllerinput = np.array([[0],[0]])
        self.rate = rospy.Rate(10)


    def controllerinput_callback(self,command):
        self.controllerinput[0] = command.u1
        self.controllerinput[1] = command.u2

    def propagate_once(self):
        def xdot_2d(y,t,yd):
            # CONTROLLER
            e = yd[0:6] - y
            # position controller
            u1 = m * (yd[7] + Kd[1] * e[4] + Kp[1] * e[1] + g)
            theta_d = -1 / g * (yd[6] + Kd[0] * e[3] + Kp[0] * e[0])
            # attitude controller
            u2 = Ixx * (yd[8] + Kd[2] * e[5] + Kp[2] * (theta_d - y[2]))  # add theta_d
            u = np.array([[u1], [u2]]) + u0

            u0 = np.array([[m*g],[0]])
            u = self.controllerinput + self.u0

            # CLOSED-LOOP DYNAMICS
            F = np.array([[y[3]], [y[4]], [y[5]], [0], [-g], [0]])
            G = np.array(
                [[0, 0], [0, 0], [0, 0], [(-1 / m) * np.sin(y[2]), 0], [(1 / m) * np.cos(y[2]), 0], [0, 1 / Ixx]])
            return np.squeeze(F + np.matmul(G,u)).tolist()

        x = odeint(xdot_2d, x0, t, args=(yd,))



        

   
'''    while not rospy.is_shutdown():
        rospy.spin()'''


if __name__ == '__main__':
    try:
        Quad = quadrotor()

    except rospy.ROSInterruptException:
            pass