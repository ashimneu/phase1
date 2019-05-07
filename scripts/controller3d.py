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
        self.Iyy = 1.43e-5  # [kilogram*meters^2]
        self.Izz = 2.89e-5  # [kilogram*meters^2]
        self.kF = 6.11e-8  # [Newton/(rpm)^2]
        self.kM = 1.59e-9  # [(Newton*meter)/(rpm)^2]
        self.gamma = self.kM / self.kF  # [meter]
        self.M = inv(np.array([[1, 1, 1, 1], [0, l, 0, -l], [-l, 0, l, 0], [self.gamma, -self.gamma, self.gamma, self.gamma]]))
        self.Kd = np.array([15,15,0.5])
        self.Kp = np.array([0,0,0])

        # equillibrium input
        u0 = np.array([[m * g], [0], [0], [0]])

        # initial pose
        q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q0_dot = [0] * 6
        x0 = np.array(q0 + q0_dot)

        # desired pose
        qd = [0.0, 0.0, 10.0, 0.0, 0.0, np.pi / 2]
        qd_dot = [0] * 6
        qd_ddot = [0] * 6
        self.xd = np.array(qd + qd_dot + qd_ddot)


        self.currentpose = np.array([0,0,0,0,0,0])
        self.waypoint = np.array([0,0])
        self.initialpose = np.array([0,0,0,0,0,0])
        self.desiredpose = np.array([5,15,0,0,0,0,0,0,0])
        self.controllerinput = Cmd2d()
        self.controller_publisher = rospy.Publisher('phase1/controller', Cmd2d, queue_size=10)
        self.controller_subscriber = rospy.Subscriber('phase1/quadrotor2d', Pose2d, self.currentpose_callback)
        self.rate = rospy.Rate(10)

    def currentpose_callback(self,config):
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

    def world2body_vel(euler, euler_dot)
        # euler = [phi, theta, psi] the roll, pitch, and yaw angles in world frame
        # euler_dot = [phi_dot theta_dot psi_dot] world frame angular velocities
        T = np.array([[np.cos(euler[1]), 0.0, -np.cos(euler[0]) * np.sin(euler[1])],
                      [0.0, 1, np.sin(euler[0])],
                      [np.sin(euler[1]), 0.0, np.cos(euler[0]) * np.cos(euler[1])]])

        return np.matmul(T,euler_dot)

# ADD LOGIC TO EXTRACT FORCES/MOMENTS FROM MODEL

    def start(self):
        Kd = self.Kd
        Kp = self.Kp
        xd = self.xd
        while (not rospy.is_shutdown()):
            x = self.currentpose
            e = xd[0:11] - x
            # position controller
            x_ddot_c = xd[12] + Kd['x_dot'] * e[6] + Kp['x'] * e[0]
            y_ddot_c = xd[13] + Kd['y_dot'] * e[7] + Kp['y'] * e[1]
            z_ddot_c = xd[14] + Kd['z_dot'] * e[8] + Kp['z'] * e[2]

            psi_c = xd[5]
            phi_c = 1 / g * (x_ddot_c * np.sin(psi_c) - y_ddot_c * np.cos(psi_c)
            theta_c = 1 / g * (x_ddot_c * np.cos(psi_c) - y_ddot_c * np.sin(psi_c)
            u1 = z_ddot_c
            # attitude controller
            body = world2body_vel(x[4:6], x[6:12])
            p = body[0]
            q = body[1]
            r = body[2]
            u2 = np.array([[Kp['phi'] * (phi_c - x[3]) + Kd['phi'] * (p_c - p)],
                           [Kp['theta'] * (theta_c - x[4]) + Kd['theta'] * (q_c - q)],
                           [Kp['psi'] * (psi_c - x[5]) + Kd['psi'] * (r_c - r)]])

            u = np.array([[u1], [u2]]) + u0


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
