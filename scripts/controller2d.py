#!/usr/bin/env python
import rospy
import numpy as np
from phase1.msg import Pose2d
from phase1.msg import Cmd2d
from rosgraph_msgs.msg import Clock



class controller():
    
    def __init__(self):
        rospy.init_node('controller2d')
        self.controller_publisher = rospy.Publisher('controller', Cmd2d, queue_size=10, latch = True)
        self.controller_subscriber = rospy.Subscriber('quadrotor', Pose2d, self.currentpose_callback)
        self.clock_subscriber = rospy.Subscriber('/clock', Clock, self.clock_callback)
        self.rate = rospy.Rate(1)
        self.g = 9.80665  # [meters/sec^2]
        self.m = 0.030  # [kilograms]
        self.l = 0.046  # [meters]
        self.Ixx = 1.43e-5  # [kilogram*meters^2]
        self.Kd = np.array([15,15,0.5])
        self.Kp = np.array([0,0,0])
        self.yd = np.array([5,15,0,0,0,0,0,0,0])
        self.currentpose = np.array([0,0,0,0,0,0])
        self.initialpose = np.array([0,0,0,0,0,0])
        self.desiredpose = np.array([5,15,0,0,0,0,0,0,0])
        self.waypoint = np.array([0,0])
        self.controllerinput = Cmd2d()
        self.time_new = 0.0 # time at current step
        self.time_old = 0.0 # time at previous step
        self.t = 0.0 # time lapse since clock_start
        self.got_new_pose = True
        #self.got_new_clock_msg = True



    def clock_callback(self,clock_time):
        #print('type(clock_time) =  ', type(clock_time.clock.to_sec()) )
        #print('Controller: New clock msg received.')
        time_new = clock_time.clock.to_sec() # float data type
        print('Controller: New clock msg received: ', np.round(time_new, 2))
        if (self.time_old == 0.0):
            self.time_old = time_new
        else:
            self.t = self.t + (time_new - self.time_old) # make time lapse update
            self.time_new = time_new
            #self.got_new_clock_msg = False

    def currentpose_callback(self,config):
        print('Controller: new pose received')
        self.currentpose = self.pose2nparray(config)
        self.got_new_pose = True


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

    def start(self):
        Kd = self.Kd
        Kp = self.Kp
        yd = self.yd
        while (not rospy.is_shutdown()):
            #time = rospy.get_time()
            if (self.got_new_pose):
                print('inside if statement')
                y = self.currentpose
                e = yd[0:6] - y
                u1 = self.m * (yd[7] + Kd[1] * e[4] + Kp[1] * e[1])
                phid = -1 / self.g * (yd[6] + Kd[0] * e[3] + Kp[0] * e[0])
                u2 = self.Ixx * (yd[8] + Kd[2] * e[5] + Kp[2] * (phid - y[2]))
                self.controllerinput.u1 = u1
                self.controllerinput.u2 = u2
                self.time_old = self.time_new
                self.got_new_pose = False
                self.controller_publisher.publish(self.controllerinput)
                print('Controller: A cmd is published.')
            self.rate.sleep()

        #rospy.spin()
   
    
'''    while not rospy.is_shutdown():
        rospy.spin()'''



if __name__ == '__main__':
    try:
        Controller1 = controller()
        Controller1.start()

    except rospy.ROSInterruptException:
            pass
