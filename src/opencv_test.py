#!/usr/bin/python

import sys
import numpy as np
import rospy
import cv2
# import math
# import tf
# from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
# import roslib

# T= rospy.get_param('~T', 4)

#using global variable for now, but consider accessing values using classes
node_name='opencv_test_node'
win1_name='RGB_feed'
win2_name='Edges'
win3_name='Depth'
freq = 2


cvbridge = CvBridge()

def initialize():
    pass

def cleanup():
    print ("Have a nice day.")
    cv2.destroyAllWindows()
    pass

def callback_rgb(data):
    # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
        frame = cvbridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
        print e
    np_array = np.array(frame, dtype=np.uint8)
    # image = process_rgb(np_array)

    cv2.imshow(win1_name, np_array)
    # cv2.imshow(win2_name, np_array)
    keystroke = cv.WaitKey(5)
    if 32 <= keystroke and keystroke < 128:
        cc = chr(self.keystroke).lower()
        if cc == 'q':
            # The user has press the q key, so exit
            rospy.signal_shutdown("User hit q key to quit.")

def callback_depth(data):
    try:
        # The depth image is a single-channel float32 image
        frame = self.bridge.imgmsg_to_cv(data, "32FC1")
    except CvBridgeError, e:
        print e
    np_array = np.array(frame, dtype=np.float32)
    # Normalize the depth image to fall between 0 (black) and 1 (white)
    cv2.normalize(np_array, np_array, 0, 1, cv2.NORM_MINMAX)
    # Process the depth image
    image = process_depth(np_array)
    # Display the result
    cv2.imshow(win2_name, image)

def process_image(frame):
    # Convert to grayscale
    grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
    # Blur the image
    grey = cv2.blur(grey, (7, 7))
    # Compute edges using the Canny edge filter
    edges = cv2.Canny(grey, 15.0, 30.0)
    return edges

def main():
    
    rospy.init_node(node_name) # check to see if launch file remaps this
    try:
        cv2.namedWindow(win1_name, flags=cv2.cv.WINDOW_NORMAL) #see if flag is necessary
        # cv2.NamedWindow(win2_name, flags=WINDOW_NORMAL)
        cv2.moveWindow(win1_name,200,0)
        # cv2.MoveWindow(win2_name,200,300)
        cv2.resizeWindow(win1_name,100,300)
        # cv2.ResizeWindow(win2_name,100,300)

        cv2.startWindowThread()
        rospy.on_shutdown(cleanup)
        
        
        # rate = rospy.Rate(freq)
        sub_RBG = rospy.Subscriber("/camera/rgb/image_color", Image, callback_rbg)
        # sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)

        print('==========Initializing Subscribers=====')
        rospy.sleep(1)

        # while not rospy.is_shutdown():
        #     rospy.rate=1
        #     rospy.wait_for_message("/camera/rgb/image_color", Image, timeout=None)
        #     k = cv2.waitKey(0) & 0xFF
        #     if k==27:
        #         cleanup()
        #     # rate.sleep()
        rospy.spin()

    except KeyboardInterrupt:
        print "Shutting down"
        cleanup()


if __name__=='__main__':
    try:
        main()
        cleanup()
    except rospy.ROSInterruptException:
        cleanup()