#!/usr/bin/env python
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from phase1.msg import Pose2d
from phase1.msg import Cmd2d
from rosgraph_msgs.msg import Clock



class quadrotor():
    def __init__(self):
        rospy.init_node('quadrotor2d')
        self.input_subscriber = rospy.Subscriber('controller', Cmd2d, self.controllerinput_callback)
        self.quadrotor_publisher = rospy.Publisher('quadrotor', Pose2d, queue_size=10, latch = True)
        self.clock_subscriber = rospy.Subscriber('/clock', Clock, self.clock_callback)
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


    def clock_callback(self,clock_time):
        #print('type(clock_time) =  ', type(clock_time.clock.to_sec()) )
        #print('Quad: New clock msg received.')
        time_new = clock_time.clock.to_sec()# float data type
        print('Quad: New clock msg received: ', np.round(time_new,2))
        if (self.time_old == 0.0):
            self.time_old = time_new
        else:
            self.t = self.t + (time_new - self.time_old) # make a time lapse update
            self.time_new = time_new
            self.got_timelapse_updated = True

    def controllerinput_callback(self,command):
        print("Quad: New input cmd received.")
        self.controllerinput[0] = command.u1
        self.controllerinput[1] = command.u2
        self.got_new_input = True



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
        m = self.m
        g = self.g
        Ixx = self.Ixx
        u0 = np.array([[m*g], [0]])
        yd = self.desiredpose # <------ (note: location of yd might change

        def xdot_2d(y,t,yd,u):
            # CLOSED-LOOP DYNAMICS
            F = np.array([[y[3]], [y[4]], [y[5]], [0], [-g], [0]])
            G = np.array([[0,0], [0,0], [0,0], [(-1/m)*np.sin(y[2]),0], [(1/m)*np.cos(y[2]),0], [0,1/Ixx]])
            return np.squeeze(F + np.matmul(G,u)).tolist()

        while (not rospy.is_shutdown()):
            if self.got_timelapse_updated and self.got_new_input:
                #print('time lapse = ', np.round(self.t,4))
                #print('Quad: Inside if statement')
                u = self.controllerinput + u0
                x = odeint(xdot_2d, x0, [self.t, self.t + self.dt], args=(yd, u))
                q = x[1,:]
                x = np.squeeze(q).tolist()  # converts nested list to single list
                self.currentpose = x
                #print('currentpose= ', self.currentpose)
                self.time_old = self.time_new
                self.got_new_input = False
                self.got_timelapse_updated = False
                self.publishcurrentpose()
            self.rate.sleep()
        #rospy.spin()



    def publishcurrentpose(self):
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