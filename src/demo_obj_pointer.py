#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import Point
import tf
import numpy as np

from obj_velocity_control import Limb_Command

class Controller:
    def __init__(self,name):
        #by default, assuming camera faces straight forward from baxter
        self.tt = tf.TransformListener()
        rospy.sleep(1)

        self.camera_trans = np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]]) #default value
        self.get_camera_trans()
        self.sub = [] #subscriber to be defined after startup is complete

        self.limb_command = Limb_Command(name)

    def get_camera_trans(self):
        #tries to get static transform of camera for a few seconds
        transformer = tf.Transformer()
        for i in range(100):
            try:
                print "===== Searching for camera transform"
                t=self.tt.getLatestCommonTime("base","camera_link1")
                (trans,orien) = self.tt.lookupTransform("base","camera_link1",t)  #orn is presumably quaternion
                euler = tf.transformations.euler_from_quaternion(orien)
                self.camera_trans=tf.transformations.compose_matrix(angles=euler, translate=trans)
                print "===== Successfully obtained camera transform."
                print "===== The camera is located at: ",trans, euler, " ...in the robot frame.\n"
                break
            except:
                rospy.sleep(0.05)

    def init_subscriber(self):
        self.sub = rospy.Subscriber('obj_position', Point, self.callback)

    def callback(self,data):
        #data is 3-element list of xyz coord in camera frame
        pos_cam = [data.x,data.y,data.z]
        pos_base = self.get_obj_coord(pos_cam)
        self.limb_command.set_obj_pos_trans(pos_base) #set Limb_Command's obj position parameter

    def get_obj_coord(self,xyz):
        #given position of object in camera frame, returns position in robot base frame
        pt=tf.transformations.compose_matrix(translate=(xyz[0],xyz[1],xyz[2]))
        obj_transform=np.dot(self.camera_trans,pt)
        pt_base=tf.transformations.decompose_matrix(obj_transform)
        translation = pt_base[3]
        # quaternion=tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])
        return (translation)

    def clean_shutdown(self):
        self.limb_command.clean_shutdown()


def main():
    rospy.init_node('obj_pointer')

    print "Enabling robot"
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()

    print "\n=====OBJ_POINTER: Initializing Limb_Command"
    arm_controller=Controller("left")
    rospy.sleep(2)

    print "\n=====OBJ_POINTER: Initializing Subscriber\n"
    rospy.sleep(2)
    arm_controller.init_subscriber()

    rospy.on_shutdown(arm_controller.clean_shutdown)
    rospy.spin()



if __name__=='__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass