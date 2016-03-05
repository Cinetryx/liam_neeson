#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point

class object:
    def __init__(self):
        self.pub = rospy.Publisher('/obj_position', Point, queue_size=10)
        rospy.sleep(2)
        self.freq=10
    def go_nuts(self):
        i=0.0
        while not rospy.is_shutdown():
            rate=rospy.Rate(self.freq)
            point_msg=Point()
            point_msg.x=math.sin(2*math.pi*(i/self.freq))
            print point_msg.x
            point_msg.y=point_msg.x
            point_msg.z=point_msg.x
            self.pub.publish(point_msg)
            i+=0.1
            if i>self.freq:
                i=0.0
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('my_hacky_object')
        this=object()
        this.go_nuts()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass