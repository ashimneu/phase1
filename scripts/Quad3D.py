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
        self.Ku = 10
        self.Tu = 12
        self.Kp = {'x': 0.8*self.Ku, 'y': 0.8*self.Ku, 'z': 1e2,
                   'phi': 5e0, 'theta': 5e0, 'psi': 0.0}
        self.Kd = {'x_dot': 0.1*self.Ku*self.Tu, 'y_dot': 0.1*self.Ku*self.Tu, 'z_dot': 15,
                   'phi_dot': 2e0, 'theta_dot': 2e0, 'psi_dot': 0.0}
        self.yd = np.array([5,15,0,0,0,0,0,0,0])
        self.initialpose = np.asarray([0,0,0,0,0,0]+[0]*6)
        self.currentpose = self.initialpose
        self.desiredpose = np.array([0,0,5,0,0,0]+[0]*6 + [0]*6)
        self.start_time = 0
        self.end_time = None
        self.dt = 0.07
        self.pose_trajectory= None
        self.time_trajectory = None



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
        x_ddot_c = xd[12] + Kd['x_dot'] * e[6] + Kp['x'] * e[0]
        y_ddot_c = xd[13] + Kd['y_dot'] * e[7] + Kp['y'] * e[1]
        z_ddot_c = xd[14] + Kd['z_dot'] * e[8] + Kp['z'] * e[2]
        psi_c = xd[5]
        phi_c = 1 / g * (x_ddot_c * np.sin(psi_c) - y_ddot_c * np.cos(psi_c))
        theta_c = 1 / g * (x_ddot_c * np.cos(psi_c) - y_ddot_c * np.sin(psi_c))
        u1 = z_ddot_c + m * g
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


    def simulate(self):
        # simulation parameters
        t = np.arange(start=self.start_time, stop=self.end_time, step=self.dt)

        # equillibrium input
        u0 = np.array([[self.m * self.g], [0], [0], [0]])

        # initial pose
        #q0 = [0.0,0.0,0.0,np.pi/180*30.0, np.pi/180*60.0, np.pi/180*0.0]
        #q0_dot = [0] * 6
        #x0 = np.array(q0 + q0_dot)
        x0 = self.initialpose

        # desired pose
        #qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #qd_dot = [0]*6
        #qd_ddot = [0]*6
        #xd = np.array(qd + qd_dot + qd_ddot)
        xd = self.desiredpose

        # solve ODE
        x = odeint(self.xdot_3d, x0, t, args=(xd,))
        self.pose_trajectory = x
        self.time_trajectory = t


    def run_once(self,currentpose,desiredpose,start_time,end_time):
        self.currentpose = currentpose
        self.desiredpose = desiredpose
        self.start_time = start_time
        self.end_time = end_time
        self.simulate()
        return

def launch_quad3d(ip,tkh,hl,ht):
    # ip -initial pose, tkh -takeoff height, hl -hover location, ht -hover time
    ip = np.array(ip)
    hl = np.array(hl)
    tkh = float(tkh)
    ht = float(ht)
    Robot = Quad3D()
    dt = 0.07
    cp1 = np.array([ip[0],ip[1],ip[2],ip[3], ip[4], ip[5]] + [0]*6)
    dp1 = np.array([ip[0], ip[0],tkh,0, 0, 0] + [0]*6 + [0]*6)
    t0 = 0
    t1 = 5
    Robot.run_once(cp1, dp1, t0, t0 + t1)
    trajectory_1 = Robot.pose_trajectory
    time_1 = Robot.time_trajectory

    # fly from takeoff 2 hover pose
    cp2 = trajectory_1[-1, 0:12]
    dp2 = np.array([hl[0], hl[1], hl[2], 0, 0, hl[3]] + [0]*6 + [0]*6)
    t2 = 5
    Robot.run_once(cp2, dp2, t1 + dt, t1 + t2)
    trajectory_2 = Robot.pose_trajectory
    time_2 = Robot.time_trajectory

    # Hover for some time
    cp3 = trajectory_2[-1, 0:12]
    dp3 = np.array([hl[0], hl[0], hl[2], 0, 0, hl[3]] + [0]*6 + [0]*6)
    t3 = ht
    Robot.run_once(cp3, dp3, t2 + dt, t2 + t3)

    trajectory_3 = Robot.pose_trajectory
    time_3 = Robot.time_trajectory

    # Land from Hover pose
    cp4 = trajectory_3[-1, 0:12]
    dp4 = np.array([hl[0], hl[1], 0, 0, 0, 0, 0, 0, hl[3]] + [0]*6 + [0]*6)
    t4 = 5
    Robot.run_once(cp4, dp4, t3 + dt, t3 + t4)

    trajectory_4 = Robot.pose_trajectory
    time_4 = Robot.time_trajectory

    x = np.concatenate((trajectory_1, trajectory_2, trajectory_3, trajectory_4), axis=0)
    t = np.concatenate((time_1, time_2, time_3, time_4), axis=0)

    # extract for plotting
    X = np.array(x)[:, 0]
    Y = np.array(x)[:, 1]
    Z = np.array(x)[:, 2]
    Theta = np.array(x)[:, 3]

    # plot results

    plt.plot(t, Z, 'g')
    plt.plot(t, X, 'b')
    plt.plot(t, Y, 'r')
    # plt.plot(t,Phi,'w')
    # plt.plot(t,Theta,'c')
    # plt.plot(t,Psi,'y')
    plt.grid(linestyle='--', linewidth='0.5', color='white')
    plt.show()

    #plt.subplot(2, 2, 1)
    #plt.plot(X, Y, 'b')
    #plt.xlabel(r'$x(t)$')
    #plt.ylabel(r'$y(t)$')
    #plt.xlim(-10, 10)
    #plt.ylim(-1, 20)
    #plt.grid(True)
    #plt.show()




if __name__ == '__main__':
    try:
        #launch_quad3d([0.0,0.0,0.0,0.0],5.0,[15.0,15.0,15.0],5.0)
        initialpose = rospy.get_param('/quad3d/initialpose')
        tkfheight = rospy.get_param('/quad3d/tkfheight')
        hoverpose = rospy.get_param('/quad3d/hoverpose')
        hovertime = rospy.get_param('/quad3d/hovertime')
        #print(initialpose,tkfheight,hoverpose,hovertime)
        #print(type(initialpose),type(tkfheight),type(hoverpose),type(hovertime))
        launch_quad3d(initialpose, tkfheight, hoverpose, hovertime)




    except rospy.ROSInterruptException:
            pass

