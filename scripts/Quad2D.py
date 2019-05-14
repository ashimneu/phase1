#!/usr/bin/env python
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from phase1.msg import Pose2d


class Quad2D():
    def __init__(self):
        rospy.init_node('quad2d')
        self.quadrotor_publisher = rospy.Publisher('quad2d/pose', Pose2d, queue_size=10, latch = True)

        self.rate = rospy.Rate(10)
        self.g = 9.80665  # [meters/sec^2]
        self.m = 0.030  # [kilograms]
        self.l = 0.046  # [meters]
        self.Ixx = 1.43e-5  # [kilogram*meters^2]
        self.Kd = np.array([15.0,15.0,0.5])
        self.Kp = np.array([0.0,0.0,0.0])
        self.yd = np.array([5,15,0,0,0,0,0,0,0])
        self.initialpose = np.asarray([5.0,5.0,np.pi/8,0.0,0.0,0.0])
        self.currentpose = self.initialpose
        self.desiredpose = np.array([15,15,0,0,0,0,0,0,0])
        self.waypoint = np.array([0.0,0.0])
        self.controllerinput = np.asarray([[0],[0]])
        self.time_new = 0.0 # clock time at current step
        self.time_old = 0.0 # clock time at previous step
        self.t = 0.0 # time lapse since clock_start
        self.dt = 0.1
        self.got_timelapse_updated = False # second time lapse update yet?
        self.got_new_input = False
        self.pose_trajectory = None
        self.A = np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[0,0,-g,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]])   #linearized system matrix
        self.B = = np.array([[0,0],[0,0],[0,0],[0,0],[1/m,0],[0,1/Ixx]])    #linearized input matrix


    def xdot_2d(self,y,t,yd,u0,m,g,Ixx,Kp,Kd):

        # CONTROLLER
        e = yd[0:6] - y
        # position controller
        u1 = m * (yd[7] + Kd[1] * e[4] + Kp[1] * e[1])
        theta_d = -1 / g * (yd[6] + Kd[0] * e[3] + Kp[0] * e[0])
        # attitude controller
        u2 = Ixx * (yd[8] + Kd[2] * e[5] + Kp[2] * (theta_d - y[2]))  # add theta_d
        u = np.array([[u1], [u2]]) + u0
        # CLOSED-LOOP DYNAMICS
        #F = np.array([[y[3]], [y[4]], [y[5]], [0], [-self.g], [0]])
        #G = np.array([[0,0], [0,0], [0,0], [(-1/m)*np.sin(y[2]),0], [(1/m)*np.cos(y[2]),0], [0,1/self.Ixx]])
        return np.squeeze(np.matmul(self.A,y) + np.matmul(self.B,u)).tolist()

    def start(self):
        x0 = self.currentpose
        u0 = np.array([[self.m * self.g], [0]])
        yd = self.desiredpose
        m = self.m

        # simulation parameters
        tf = 1
        tstep = 0.005
        t = np.arange(start=0, stop=tf, step=tstep)


        #print('time lapse = ', np.round(self.t,4))
        #print('Quad: Inside if statement')
        x = odeint(self.xdot_2d, x0, t, args=(yd,u0,self.m,self.g,self.Ixx,self.Kp,self.Kd))


        print('dim(x)', x.shape)

        # extract for plotting
        # extract for plotting
        Y = np.array(x)[:,0]
        Z = np.array(x)[:,1]
        Theta = np.array(x)[:,2]

        # plot results
        plt.subplot(2,2,1)
        plt.plot(Y,Z,'b')
        plt.xlabel(r'$y(t)$')
        plt.ylabel(r'$z(t)$')
        plt.xlim(-10,10)
        plt.ylim(-1,20)
        plt.grid(True)

        plt.subplot(2,2,2)
        plt.plot(t,Y,'b')
        plt.xlabel(r'$t$')
        plt.ylabel(r'$y(t)$')
        plt.xlim(-1,15)
        plt.ylim(-5,15)
        plt.grid(True)

        plt.subplot(2,2,3)
        plt.plot(t,Z,'b')
        plt.xlabel(r'$t$')
        plt.ylabel(r'$z(t)$')
        plt.xlim(-1,15)
        plt.ylim(-5,15)
        plt.grid(True)

        plt.subplot(2,2,4)
        plt.plot(t,Theta,'b')
        plt.xlabel(r'$t$')
        plt.ylabel(r'$\phi$')
        plt.xlim(-1,15)
        plt.ylim(-4,4)
        plt.grid(True)
        #figManager = plt.get_current_fig_manager()
        #figManager.window.showMaximized()
        #figManager.window.state('zoomed')
        plt.show()


if __name__ == '__main__':
    try:
        Robot = Quad2D()
        Robot.start()


    except rospy.ROSInterruptException:
            pass

