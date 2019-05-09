#!/usr/bin/env python
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from geometry_msgs.msg import Twist


class Quad3D():
    def __init__(self):
        # ROS Node & Publisher Initializaition
        rospy.init_node('quad3d')
        self.quadrotor_publisher = rospy.Publisher('quad3d/pose', Twist, queue_size=10, latch = True)
        self.rate = rospy.Rate(10)

        # References ------------------------------------------
        # [1] Powers, Mellinger, Kumar 2014
        # -----------------------------------------------------
        # physical parameters
        self.g = 9.80665  # [meters/sec^2]
        self.m = 0.030  # [kilograms]
        self.l = 0.046  # [meters]
        self.Ixx = 1.43e-5  # [kilogram*meters^2]
        self.Iyy = 1.43e-5  # [kilogram*meters^2]
        self.Izz = 2.89e-5   # [kilogram*meters^2]
        self.kF = 6.11e-8  # [Newton/(rpm)^2]
        self.kM = 1.59e-9  # [(Newton*meter)/(rpm)^2]
        self.gamma = self.kM / self.kF  # [meter]
        self.M = np.linalg.inv(np.array([[1, 1, 1, 1], [0, self.l, 0, -self.l], [-self.l, 0, self.l, 0], [self.gamma, -self.gamma, self.gamma, self.gamma]]))
        self.I = np.array([[self.Ixx, 0.0, 0.0], [0.0, self.Iyy, 0.0], [0.0, 0.0, self.Izz]])
        self.Iinv = np.array([[1 / self.Ixx, 0.0, 0.0], [0.0, 1 / self.Iyy, 0.0], [0.0, 0.0, 1 / self.Izz]])

        # position and velocity gains
        # position and velocity gains
        self.Kp = {'x': 2e1, 'y': 2.2e1, 'z': 1e2, 'phi': 5e0, 'theta': 1e2, 'psi': 0.0}
        self.Kd = {'x_dot': 3e1, 'y_dot': 3.8e1, 'z_dot': 15, 'phi_dot': 2e0, 'theta_dot': 1e1, 'psi_dot': 0.0}

        #self.Kd = np.array([15.0,15.0,0.5])
        #self.Kp = np.array([0.0,0.0,0.0])

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

    def world2body_vel(self,euler, omega_dot_w):
        # euler = [phi, theta, psi] the roll, pitch, and yaw angles in world frame
        # euler_dot = [phi_dot theta_dot psi_dot] world frame angular velocities
        T = np.array([[np.cos(euler[1]), 0.0, -np.cos(euler[0]) * np.sin(euler[1])],
                      [0.0, 1, np.sin(euler[0])],
                      [np.sin(euler[1]), 0.0, np.cos(euler[0] * np.cos(euler[1]))]])
        return np.matmul(T, omega_dot_w)

    def body2world_vel(self,euler, omega_b):
        T = np.array([[np.cos(euler[1]), 0.0, -np.cos(euler[0]) * np.sin(euler[1])],
                      [0.0, 1, np.sin(euler[0])],
                      [np.sin(euler[1]), 0.0, np.cos(euler[0] * np.cos(euler[1]))]])
        TT = np.linalg.inv(T)
        return np.matmul(TT, omega_b)

    def Rot(self,euler):
        # rotation matrix from body to world frame
        R = np.array([[np.cos(euler[1]) * np.cos(euler[2]) - np.sin(euler[0]) * np.sin(euler[1]) * np.sin(euler[2]),
                       -np.cos(euler[0]) * np.sin(euler[2]),
                       np.cos(euler[2]) * np.sin(euler[1]) + np.cos(euler[2]) * np.sin(euler[0]) * np.sin(euler[2])],
                      [np.cos(euler[1]) * np.sin(euler[2]) + np.cos(euler[1]) * np.sin(euler[1]) * np.sin(euler[0]),
                       np.cos(euler[0]) * np.cos(euler[2]),
                       np.sin(euler[1]) * np.sin(euler[2]) - np.cos(euler[2]) * np.cos(euler[1]) * np.sin(euler[0])],
                      [-np.cos(euler[0]) * np.sin(euler[1]), np.sin(euler[0]), np.cos(euler[0]) * np.cos(euler[1])]])
        return R


            # dynamics & control of planar quadrotor
    def xdot_3d(self,x,t,xd):
        Kd = self.Kd
        Kp = self.Kp
        m = self.m
        g = self.g

        # CONTROLLER
        e = xd[0:12] - x
        # position controller
        x_ddot_c = xd[12] + Kd['x_dot']*e[6] + Kp['x']*e[0]
        y_ddot_c = xd[13] + Kd['y_dot']*e[7] + Kp['y']*e[1]
        z_ddot_c = xd[14] + Kd['z_dot']*e[8] + Kp['z']*e[2]
        psi_c = xd[5]
        phi_c = 1/g*(x_ddot_c*np.sin(psi_c) - y_ddot_c*np.cos(psi_c))
        theta_c = 1/g*(x_ddot_c*np.cos(psi_c) - y_ddot_c*np.sin(psi_c))
        u1 = z_ddot_c + m*g
        # attitude controller
        p = x[9]
        q = x[10]
        r = x[11]
        p_c = 0.0   # as per [1]
        q_c = 0.0
        r_c = 0.0
        u2 = np.array([Kp['phi']*(phi_c - x[3]) +     Kd['phi_dot']*(p_c - p),
                       Kp['theta']*(theta_c - x[4]) + Kd['theta_dot']*(q_c - q),
                       Kp['psi']*(psi_c - x[5]) +     Kd['psi_dot']*(r_c - r)])

        # CLOSED-LOOP DYNAMICS
        r_dot = x[6:9]
        omega_dot_w = self.body2world_vel(x[3:6],x[9:12])
        r_ddot = np.squeeze(np.array([[0.0],[0.0],[-m*g]]) + np.matmul(self.Rot(x[3:6]),np.array([[0.0],[0.0],[u1]])))
        omega_ddot_b = np.matmul(self.Iinv,(u2 - np.cross(x[9:12], np.matmul(self.I,x[9:12]))))
        #breakpoint()
        return np.concatenate((r_dot,omega_dot_w,r_ddot,omega_ddot_b),axis=0)


    def start(self):

        # simulation parameters
        tf = 10
        tstep = 0.1
        t = np.arange(start=0, stop=tf, step=tstep)

        # equillibrium input
        u0 = np.array([[self.m * self.g], [0], [0], [0]])

        # initial pose
        q0 = [0.0, 0.0, 0.0, np.pi / 180 * 30.0, np.pi / 180 * 60.0, np.pi / 180 * 0.0]
        q0_dot = [0] * 6
        x0 = np.array(q0 + q0_dot)

        # desired pose
        qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        qd_dot = [0] * 6
        qd_ddot = [0] * 6
        xd = np.array(qd + qd_dot + qd_ddot)

        # solve ODE
        x = odeint(self.xdot_3d, x0, t, args=(xd,))

        # extract for plotting
        X = np.array(x)[:, 0]
        Y = np.array(x)[:, 1]
        Z = np.array(x)[:, 2]
        Phi = np.array(x)[:, 3]
        Theta = np.array(x)[:, 4]
        Psi = np.array(x)[:, 5]

        # plot results
        # fig = plt.figure()
        # ax = plt.axes(projection='3d')
        # ax.plot3D(X,Y,Z)
        # plt.xlabel(r'$x(t)$')
        # plt.ylabel(r'$y(t)$')
        # plt.zlabel(r'$z(t)$')
        # plt.plot(t,Z,'g')
        # plt.plot(t,X,'b')
        # plt.plot(t,Y,'r')
        plt.plot(t, Phi, 'w')
        plt.plot(t, Theta, 'c')
        # plt.plot(t,Psi,'y')
        # plt.grid()
        plt.show()




if __name__ == '__main__':
    try:
        Robot = Quad3D()
        Robot.start()


    except rospy.ROSInterruptException:
            pass

