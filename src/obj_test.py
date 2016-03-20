#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point

class object:
    def __init__(self):
        self.pub = rospy.Publisher('/obj_position', Point, queue_size=2)
        rospy.sleep(2)
        self.freq=5.0
    def go_nuts(self):
        i=0.0
        rate=rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            point_msg=Point()
            fig1=math.sin(2*math.pi*(i/self.freq))
            fig2=math.cos(2*math.pi*(i/self.freq))
            point_msg.x=2
            point_msg.y=fig2+1
            point_msg.z=fig1
            print fig1
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