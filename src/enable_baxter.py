#!/usr/bin/env python
import rospy
import baxter_interface

# from math import pi, sqrt

def initialize():
    print "=== Enabling robot..."
    rs = baxter_interface.RobotEnable()
    if not rs.state().enabled:
        rs.enable()
        rospy.sleep(0.5)
    print "=== Robot is enabled!"
    return True



def main():
    print "=== Reaching robot..."

    flag = 0
    while not flag:
        try:
            flag = initialize()
        except:
            rospy.sleep(0.5)

if __name__=='__main__':
    try:
        rospy.init_node('enable_baxter')
        main()
    except rospy.ROSInterruptException:
        pass