#!/usr/bin/env python


################
# ROS IMPORTS: #
################
import rospy
import tf
from tf import transformations as TR
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg as GM
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg as VM

###################
# NON-ROS IMPORTS #
###################
import numpy as np
import copy

import rospy
import math



####################
# GLOBAL CONSTANTS #
####################
SIMFRAME = "camera_link1"
MASSFRAME = "mass"

class DummyObject:
    def __init__(self):
        self.pub = rospy.Publisher('/obj_position', Point, queue_size=2)
        rospy.sleep(2)
        self.link_len = 1.5 
        self.A = 2
        self.period = 10.0
        self.speed = 0.75
        self.t=0.0

        self.setup_markers()
        # self.mass_pub = rospy.Publisher("mass_point", PointStamped, queue_size=2)
        self.marker_pub = rospy.Publisher("visualization_marker_array", VM.MarkerArray, queue_size=2)
        self.br = tf.TransformBroadcaster()
        self.sim_timer = rospy.Timer(rospy.Duration(1/self.period), self.go_nuts)


    def setup_markers(self):
        self.markers = VM.MarkerArray()
        # mass marker
        self.mass_marker = VM.Marker()
        self.mass_marker.action = VM.Marker.ADD
        self.mass_marker.color = ColorRGBA(*[1.0, 1.0, 1.0, 1.0])
        self.mass_marker.header.frame_id = rospy.get_namespace() + SIMFRAME # for some reason rviz warns about non-qualified names without namespace
        self.mass_marker.lifetime = rospy.Duration(5*(1/self.period))
        self.mass_marker.scale = GM.Vector3(*[0.05, 0.05, 0.05])
        self.mass_marker.type = VM.Marker.SPHERE
        self.mass_marker.id = 0
        self.markers.markers.append(self.mass_marker)
        return

    def go_nuts(self,event):
        
        point_msg=Point()
        v_t = self.A*math.sin((2*math.pi/self.period)*self.t)
        fig1=self.link_len*math.cos(v_t)
        fig2=-self.link_len*math.sin(v_t)
        point_msg.x = -fig2
        point_msg.y = fig1
        point_msg.z = 2
        print -fig2, fig1
        self.pub.publish(point_msg)
        self.t += 1/self.period
        if (self.t > self.period):
            self.t = 0.0

        for m in self.markers.markers:
            m.header.stamp = rospy.Time.now()
            self.mass_marker.pose = GM.Pose(position=point_msg)
            self.marker_pub.publish(self.markers)



if __name__ == '__main__':
    try:
        rospy.init_node('my_dummy_object')
        this = DummyObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass