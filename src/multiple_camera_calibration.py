#!/usr/bin/env python

#This node takes transforms from two cameras and calculates the transform from the Baxter base frame to the ASUS camera
#run beforehand: roslaunch openni_launch openni.launch depth_registration:=true rgb_camera_info_url:=file:///home/hongalan/skelws/src/liam_neeson/ost.yaml



import rospy
import os
import sys
import subprocess
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

from baxter_interface.camera import CameraController

import rospkg

class Calibrator:
    def __init__(self,output_frame,image_topic, info_topic, marker_frame="ar_marker_0"):
        # self.sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size=1)
        self.tt = tf.TransformListener()

        self.output_frame = output_frame
        self.image_topic = image_topic
        self.info_topic = info_topic
        self.marker_frame = marker_frame
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('liam_neeson')
        self.transform = []
        print "===== %s calibrator initialized" %self.output_frame
        rospy.sleep(1)

    def set_transform(self):
        self.transform = []
        for i in range(100):
            try:
                print "===== Checking for ar_pose_marker"
                t=self.tt.getLatestCommonTime(self.marker_frame,self.output_frame)
                (trans,orien) = self.tt.lookupTransform(self.marker_frame,self.output_frame,t)  #orn is presumably quaternion
                euler = tf.transformations.euler_from_quaternion(orien)
                self.transform=tf.transformations.compose_matrix(angles=euler, translate=trans)
                print "===== Successfully obtained %s transform." %self.output_frame
                print "===== The ar code is located at: ",trans, euler, " ...in the %s frame.\n" %self.output_frame
                break
            except:
                rospy.sleep(0.05)

        if (self.transform == []):
            return False
        else:
            return True


def get_trans(frame1,frame2):
        #tries to get static transform of camera for a few seconds
        tt = tf.TransformListener()
        frame_transform = []
        print "===== Searching for robot transform"
        transformer = tf.Transformer()
        for i in range(100):
            try:
                t=tt.getLatestCommonTime(frame1,frame2)
                (trans,orien) = tt.lookupTransform(frame1,frame2,t)
                euler = tf.transformations.euler_from_quaternion(orien)
                frame_transform=tf.transformations.compose_matrix(angles=euler, translate=trans)
                # print "===== Successfully obtained transform from %s to %s: " %(frame1,frame2), trans, euler
                # print "===== The camera is located at: ",trans, euler, " ...in the robot frame.\n"
                break
            except:
                rospy.sleep(0.05)
        return frame_transform

def main():

    rospy.init_node('multiple_camera_calibration')
        
    print "===== Initialized multiple camera calibration"

    # print "Starting camera calibration"
    if len(sys.argv)>8:
        c1_output_frame = sys.argv[1]
        c1_image_topic = sys.argv[2]
        c1_info_topic = sys.argv[3]

        c2_output_frame = sys.argv[4]
        c2_image_topic = sys.argv[5]
        c2_info_topic = sys.argv[6]

        marker_frame = sys.argv[7] #such as "ar_marker_0"
        marker_size = sys.argv[8]
    else:
        print "=====obj_dregistered node requires 8 arguments: \ncamera1: output_frame, image_topic, info_topic \ncamera2: output_frame, image_topic, info_topic \nmarker_frame, marker_size"

    c1 = Calibrator(c1_output_frame,c1_image_topic,c1_info_topic,marker_frame)
    subprocess.Popen(["roslaunch", "liam_neeson","multiple_camera_calibration_aux.launch","c1:=true","c1_image_topic:="+c1_image_topic,"c1_info_topic:="+c1_info_topic,"c1_output_frame:="+c1_output_frame, "marker_frame:="+marker_frame, "marker_size:="+marker_size]) ##should pass in other arguments as well
    #be sure openni is launched
    # os.system("rosrun dynamic_reconfigure dynparam set /camera/driver image_mode 4")
    
    rospy.sleep(1)
    raw_input("===== Ready to search for AR code from first camera. Press Enter to continue.")

    while (not c1.set_transform() and not rospy.is_shutdown()):
        raw_input("===== AR code transform not found. Press Enter to try again.")

    raw_input("\n===== Press Enter to continue.\n")

    os.system('rosnode kill c1_ar_track_alvar')
    # os.system("rosrun dynamic_reconfigure dynparam set /camera/driver image_mode 2") #set back to default

    try:
        cam_control_right = CameraController('right_hand_camera')
        cam_control_right.close()
    except:
        pass
    cam_control_left = CameraController('left_hand_camera')
    res = (1280, 800) #apparently this resolution is necessary to properly derive AR transform
    cam_control_left.resolution = res
    print "Opening left-hand camera."
    cam_control_left.open()

    c2 = Calibrator(c2_output_frame,c2_image_topic,c2_info_topic,marker_frame)
    subprocess.Popen(["roslaunch", "liam_neeson","multiple_camera_calibration_aux.launch","c2:=true","c2_image_topic:="+c2_image_topic,"c2_info_topic:="+c2_info_topic,"c2_output_frame:="+c2_output_frame, "marker_frame:="+marker_frame, "marker_size:="+marker_size]) ##should pass in other arguments as well
    
    rospy.sleep(1)
    raw_input("===== Ready to search for AR code from second camera. Press Enter to continue.")

    while (not c2.set_transform() and not rospy.is_shutdown()):
        raw_input("===== AR code transform not found. Press Enter to try again.")
    # os.system('rosnode kill c2_ar_track_alvar')

    rospy.sleep(1)
    raw_input("===== Both AR code transforms found. Press Enter to continue.")

    # tf2_ros.StaticTransformBroadcaster()

    # ASSUMING c2 is Baxter's left hand camera frame
    print "Waiting for transform from Baxter camera to base frame"
    left_in_base = get_trans("base","left_hand_camera")
    while (left_in_base == [] and not rospy.is_shutdown()):
        print "Transform not found..."
        left_in_base = get_trans("base","left_hand_camera")
    print "Transform found!"
    left_in_base = tf.transformations.inverse_matrix(left_in_base)

    print "Base to Left: ", tf.transformations.decompose_matrix(left_in_base)[3]
    ar_in_base = np.dot(c2.transform,left_in_base)
    print "Base to AR: ", tf.transformations.decompose_matrix(ar_in_base)[3]
    #since the frame doesn't seem right, we need to roll -90deg, yaw -90deg
    frame_adjustment=tf.transformations.compose_matrix(angles=[-1.57,0,-1.57], translate=[0,0,0])
    c1.transform = np.dot(c1.transform,frame_adjustment)

    cam_in_ar = tf.transformations.inverse_matrix(c1.transform)
    print "AR to Camera: ", tf.transformations.decompose_matrix(cam_in_ar)[3]
    cam_in_base = np.dot(cam_in_ar, ar_in_base)
    print "Base to Camera: ",tf.transformations.decompose_matrix(cam_in_base)[3]
    


    cam_in_base_decomp = tf.transformations.decompose_matrix(cam_in_base)
    euler = cam_in_base_decomp[2]
    pos = cam_in_base_decomp[3]
    print "Euler: ", euler, " Position: ", pos
    quat = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])

    #Let's publish the static transform!
    broadcaster = tf.TransformBroadcaster()
    # static_transformStamped = TransformStamped()
    # # static_transformStamped.header.stamp = rospy.Time.now()
    # static_transformStamped.header.frame_id = "base"
    # static_transformStamped.child_frame_id = "camera_link1" ###########SUBJECT TO CHANGE
    # static_transformStamped.transform.translation.x = pos[0]
    # static_transformStamped.transform.translation.y = pos[1]
    # static_transformStamped.transform.translation.z = pos[2]
    # static_transformStamped.transform.rotation.x = quat[0]
    # static_transformStamped.transform.rotation.y = quat[1]
    # static_transformStamped.transform.rotation.z = quat[2]
    # static_transformStamped.transform.rotation.w = quat[3]

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        broadcaster.sendTransform(pos,quat,rospy.Time.now(),"base","camera_link1")
        # static_transformStamped.header.stamp = rospy.Time.now()
        # broadcaster.sendTransform(static_transformStamped)
        rate.sleep()


    # if ('baxter' in message):
    #     if ('find' in message) & (('noodles' in message) | ('lunch' in message)):
    #         print "\n==Let's have lunch."
    #         os.system('rosnode kill obj_dregistered')
    #         rospy.sleep(2)
            
    #         # cmd = 'rosrun liam_neeson obj_dregistered.py '+path+'/src/classifier_data/maruchan_sample.png '+path+'/src/classifier_data/maruchan_asus15.xml'
    #         subprocess.Popen(['rosrun', 'liam_neeson','obj_dregistered.py',self.package_path+'/src/classifier_data/maruchan_sample.png',self.package_path+'/src/classifier_data/maruchan_asus15.xml'])
    #         # os.system('gnome-terminal -x '+cmd)
    #         # os.system('gnome-terminal -x sh -c "sleep 1; bash"')
    #         # os.system('rosrun liam_neeson obj_dregistered.py '+path+'/src/classifier_data/maruchan_sample.png '+path+'/src/classifier_data/maruchan_asus15.xml')
    #         return
    #     elif ('find' in message) & ('coffee' in message):
    #         print "\n==Time for a coffee break."
    #         os.system('rosnode kill obj_dregistered')
    #         rospy.sleep(2)
    #         subprocess.Popen(['rosrun', 'liam_neeson','obj_dregistered.py',self.package_path+'/src/classifier_data/coffee_sample.png',self.package_path+'/src/classifier_data/coffee_asus15.xml'])
    #         return
    #     elif ('find' in message) & ('fruit' in message):
    #         print "\n==Fiber is good for you."
    #         os.system('rosnode kill obj_dregistered')
    #         rospy.sleep(2)
    #         subprocess.Popen(['rosrun', 'liam_neeson','obj_dregistered.py',self.package_path+'/src/classifier_data/mapear_sample.png',self.package_path+'/src/classifier_data/mapear_asus15.xml'])
    #         return
    #     elif ('hello' in message):
    #         print "\n==Hi there!"
    #         os.system('rosnode kill baxter_display_left')
    #         return
    #     elif ('good' in message) & ('job' in message):
    #         os.system('rosnode kill obj_dregistered')
    #         rospy.sleep(2)
    #         os.system('rosnode kill baxter_cam_disp_left')
    #         print "\n==Thank you."
    #         return
    #     elif ('open' in message) & ('display' in message):
    #         print "\n==Opening left-hand camera."
    #         os.system('rosnode kill baxter_display_left')
    #         rospy.sleep(2)
    #         subprocess.Popen(['rosrun', 'liam_neeson','baxter_display_left.py'])
    #         # os.system('rosrun liam_neeson baxter_display_left.py')
    #         return
    #     else:
    #         print "\n==pocketsphinx_controller: Incomplete message: '%s'" %message

    # else:
    #     print "\n==pocketsphinx_controller: Message not recognized: '%s' \n(Be sure to say 'Baxter' in command.)" %message


if __name__ == '__main__':
    # try:
        main()
        
        # rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass