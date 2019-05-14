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
        self.Kp = np.array([1,50,1200])
        self.Kd = np.array([1.5,12,self.Kp[2]/8])
        self.Ku = 1500
        self.yd = np.array([5,15,0,0,0,0,0,0,0])
        self.initialpose = np.asarray([5.0,5.0,np.pi/8,0.0,0.0,0.0])
        self.currentpose = self.initialpose
        self.desiredpose = np.array([15,15,0,0,0,0,0,0,0])
        self.dt = 0.1
        self.time_trajectory = None
        self.A = np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[0,0,-self.g,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]])   #linearized system matrix
        self.B = np.array([[0,0],[0,0],[0,0],[0,0],[1/self.m,0],[0,1/self.Ixx]])    #linearized input matrix

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
        return np.squeeze(np.matmul(self.A,y.reshape(6,1)) + np.matmul(self.B,u)).tolist()

    def simulate(self):
        x0 = self.currentpose
        u0 = np.array([[self.m * self.g], [0]])
        yd = self.desiredpose
        m = self.m

        # simulation parameters
        ts = self.start_time
        tf = self.end_time
        tstep = self.dt
        t = np.arange(start=ts, stop=tf, step=tstep)
        x = odeint(self.xdot_2d, x0, t, args=(yd,u0,self.m,self.g,self.Ixx,self.Kp,self.Kd))
        self.pose_trajectory= x
        self.time_trajectory = t

    def run_once(self,currentpose,desiredpose,start_time,end_time):
        self.currentpose = currentpose
        self.desiredpose = desiredpose
        self.start_time = start_time
        self.end_time = end_time
        self.simulate()
        return


def fly_quad2d(ip,tkh,hl,ht):
    # ip -initial pose, tkh -takeoff height, hl -hover location, ht -hover time
    Robot = Quad2D()
    dt = 0.005
    cp1 = np.asarray([ip[0], ip[1], ip[2], 0.0, 0.0, 0.0])
    dp1 = np.array([0, tkh, 0, 0, 0, 0, 0, 0, 0])
    t0 = 0
    t1 = 5
    Robot.run_once(cp1, dp1, t0, t0 + t1)
    trajectory_1 = Robot.pose_trajectory
    time_1 = Robot.time_trajectory

    # fly from takeoff 2 hover pose
    cp2 = trajectory_1[-1, 0:6]
    dp2 = np.array([hl[0], hl[1], 0, 0, 0, 0, 0, 0, 0])
    t2 = 5
    Robot.run_once(cp2, dp2, t1 + dt, t1 + t2)
    trajectory_2 = Robot.pose_trajectory
    time_2 = Robot.time_trajectory

    # Hover for some time
    cp3 = trajectory_2[-1, 0:6]
    dp3 = np.array([hl[0], hl[0], 0, 0, 0, 0, 0, 0, 0])
    t3 = ht
    Robot.run_once(cp3, dp3, t2 + dt, t2 + t3)

    trajectory_3 = Robot.pose_trajectory
    time_3 = Robot.time_trajectory

    # Land from Hover pose
    cp4 = trajectory_3[-1, 0:6]
    dp4 = np.array([hl[0], 0, 0, 0, 0, 0, 0, 0, 0])
    t4 = 5
    Robot.run_once(cp4, dp4, t3 + dt, t3 + t4)

    trajectory_4 = Robot.pose_trajectory
    time_4 = Robot.time_trajectory

    x = np.concatenate((trajectory_1, trajectory_2, trajectory_3, trajectory_4), axis=0)
    t = np.concatenate((time_1, time_2, time_3, time_4), axis=0)

    # extract for plotting
    Y = np.array(x)[:, 0]
    Z = np.array(x)[:, 1]
    Theta = np.array(x)[:, 2]

    # plot results
    #plt.subplot(2, 2, 1)
    plt.plot(Y, Z, 'b')
    plt.xlabel(r'$y(t)$')
    plt.ylabel(r'$z(t)$')
    plt.xlim(-10, 10)
    plt.ylim(-1, 20)
    plt.grid(True)

    '''plt.subplot(2, 2, 2)
    plt.plot(t, Y, 'b')
    plt.xlabel(r'$t$')
    plt.ylabel(r'$y(t)$')
    plt.xlim(-1, 15)
    plt.ylim(-5, 15)
    plt.grid(True)

    plt.subplot(2, 2, 3)
    plt.plot(t, Z, 'b')
    plt.xlabel(r'$t$')
    plt.ylabel(r'$z(t)$')
    plt.xlim(-1, 15)
    plt.ylim(-5, 15)
    plt.grid(True)

    plt.subplot(2, 2, 4)
    plt.plot(t, Theta, 'b')
    plt.xlabel(r'$t$')
    plt.ylabel(r'$\phi$')
    plt.xlim(-1, 15)
    plt.ylim(-4, 4)
    plt.grid(True)'''
    plt.show()


if __name__ == '__main__':
    try:
        #fly_quad2d([0,0,0],5,[5,15],3)
        fly_quad2d([0, 0, 0], 5, [5, 15], 3)


    except rospy.ROSInterruptException:
            pass

