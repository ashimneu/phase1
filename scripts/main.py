#!/usr/bin/env python
import time
import rospy
#from geometry_msgs.msg import Twist 
from phase1.msg import Pose



''' commentss.. '''

class quadrotor():
    def __init__(self):
        rospy.init_node('quadrotor')
        self.Pose = Pose()
        self.sub = rospy.Subscriber('phase1/controller', Pose, self.pose_callback())

    def pose_callback(self,pose_msg):
        

   
'''    while not rospy.is_shutdown():
        rospy.spin()'''


if __name__ == '__main__':
    try:
        Quad = quadrotor()

    except rospy.ROSInterruptException:
            pass