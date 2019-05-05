#!/usr/bin/env python
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from phase1.msg import Pose2d
from phase1.msg import Cmd2d



class quadrotor():
    def __init__(self):
        rospy.init_node('quadrotor2d')
        self.sub = rospy.Subscriber('phase1/controller', Cmd2d, self.controllerinput_callback)
        self.quadrotor_publisher = rospy.Publisher('phase1/quadrotor2d', Pose2d, queue_size=100)

        self.g = 9.80665  # [meters/sec^2]
        self.m = 0.030  # [kilograms]
        self.l = 0.046  # [meters]
        self.Ixx = 1.43e-5  # [kilogram*meters^2]
        self.Kd = np.array([15.0,15.0,0.5])
        self.Kp = np.array([0.0,0.0,0.0])
        self.yd = np.array([5,15,0,0,0,0,0,0,0])
        self.currentpose = np.zeros(6).tolist()
        self.waypoint = np.array([0.0,0.0])
        self.initialpose = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.desiredpose = np.array([5.0,15.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

        tf = 0.005
        tstep = 0.005
        t = np.arange(start=0, stop=tf, step=tstep)
        self.t = t

        self.dt = 0.005
        self.controllerinput = np.array([[0],[0]])
        self.rate = rospy.Rate(10)


    def controllerinput_callback(self,command):
        print("A command input has been received.")
        self.controllerinput[0] = command.u1
        self.controllerinput[1] = command.u2



    def pose2nparray(config):
        #converts ros twist data to numpy array
        arr = np.zeros(6)
        arr[0] = config.x
        arr[1] = config.y
        arr[2] = config.z
        arr[3] = config.phi
        arr[4] = config.theta
        arr[5] = config.psi
        return arr

    def pose2list(config):
        #converts ros twist data to numpy array
        ls = np.zeros(6) #empty numpy array
        ls[0] = config.x
        ls[1] = config.y
        ls[2] = config.z
        ls[3] = config.phi
        ls[4] = config.theta
        ls[5] = config.psi
        return ls

    def list2pose(self, ls):
        config = Pose2d()
        config.y = ls[0]
        config.z = ls[1]
        config.phi = ls[2]
        config.ydot = ls[3]
        config.zdot = ls[4]
        config.phidot = ls[5]
        return config

    def start(self):
        x0 = self.currentpose
        t = self.t
        m = self.m
        g = self.g
        Ixx = self.Ixx
        u0 = np.array([[m * g], [0]])
        yd = self.desiredpose # <------ (note to self) its location could change depending on nature of yd



        def xdot_2d(y,t,yd,u):
            # CLOSED-LOOP DYNAMICS
            F = np.array([[y[3]], [y[4]], [y[5]], [0], [-g], [0]])
            G = np.array([[0,0], [0,0], [0,0], [(-1/m)*np.sin(y[2]),0], [(1/m) * np.cos(y[2]),0], [0,1/Ixx]])
            return np.squeeze(F + np.matmul(G,u)).tolist()

        while (not rospy.is_shutdown()):
            u = self.controllerinput + u0
            x = odeint(xdot_2d, x0, t, args=(yd, u))
            x = np.squeeze(x).tolist()  # convert nested list x to single list x
            self.t = self.t + self.dt
            self.currentpose = x
            self.publishcurrentpose()

        rospy.spin()



    def publishcurrentpose(self):
        print('inside publishcurrentpose ')
        currentpose = self.list2pose(self.currentpose)
        self.quadrotor_publisher.publish(currentpose)
        return

   
'''    while not rospy.is_shutdown():
        rospy.spin()'''


if __name__ == '__main__':
    try:
        Robot = quadrotor()
        Robot.start()


    except rospy.ROSInterruptException:
            pass