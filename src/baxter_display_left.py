#!/usr/bin/env python

#This node publishes image data from the left hand of the Baxter robot to the display on the robot's head.

import rospy
from sensor_msgs.msg import Image
from baxter_interface.camera import CameraController

from baxter_core_msgs.srv import ListCameras

import cv2
import cv_bridge

import rospkg

class translator:
    def __init__(self):
        self.sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=1)
    def callback(self,data):
        self.pub.publish(data)
        rospy.sleep(0.01)

def send_image(path):
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)

def cleanup():
    print "Closing left-hand camera."
    cam_control_left = CameraController('left_hand_camera')
    cam_control_left.close()
    print "Left-hand camera closed."
    rospack = rospkg.RosPack()
    path = rospack.get_path('liam_neeson')
    img_path = path+"/src/resources/honhon.jpg"
    send_image(img_path)
    print "Hon hon!"


if __name__ == '__main__':
    # try:
        rospy.init_node('baxter_display_left')
        rospy.on_shutdown(cleanup)
        print "Initialized '/cameras/left_hand_camera/image' --> '/robot/xdisplay' remapping node"

        list_svc = rospy.ServiceProxy('/cameras/list', ListCameras)
        rospy.wait_for_service('/cameras/list', timeout=10)
        print "Cameras: ",list_svc().cameras
        cam_control_right = CameraController('right_hand_camera')
        cam_control_right.close()
        cam_control_left = CameraController('left_hand_camera')
        cam_control_left.mirror = True
        res = (1280, 800)
        cam_control_left.resolution = res
        print "Opening left-hand camera."
        cam_control_left.open()
        print "Left-hand camera opened."
        this=translator()
        rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass