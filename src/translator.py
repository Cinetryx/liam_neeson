#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

class translator:
    def __init__(self):
        self.sub = rospy.Subscriber('/robot/joint_states', JointState, self.callback)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    def callback(self,data):
        self.pub.publish(data)

if __name__ == '__main__':
    try:
        rospy.init_node('my_hacky_translator')
        this=translator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass