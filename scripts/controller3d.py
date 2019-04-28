#!/usr/bin/env python
import time
import rospy
#from geometry_msgs.msg import Twist
from phase1.msg import Pose 




'''commented out example '''

class controller():
    
    def __init__(self):
        rospy.init_node('controller')
        self.pub = rospy.Publisher('phase1/controller', Pose, queue_size=100)
        # example publisher pub.publish(std_msgs.msg.String("foo"))


    def publisher(self,goalpose):
         

       self.control_publisher.publish(goalpose)
       #self.rate.sleep()
   
    
'''    while not rospy.is_shutdown():
        rospy.spin()'''


if __name__ == '__main__':
    try:
        Controller1 = controller()

    except rospy.ROSInterruptException:
            pass
