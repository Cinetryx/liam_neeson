#!/usr/bin/env python
# import numpy as np
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


win1_name='rgb_image'
win2_name='edge_image'
win3_name='param'
win4_name='depth'
node_name='opencv_test_node'
rgb_flag=0
param_flag=0
depth_flag=0

def cleanup():
    print "Have a nice day."
    cv2.destroyAllWindows()

def nothing(x):
    # print "Hello"
    pass

def rgb2edges(frame):
    global param_flag

    # Convert to grayscale
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Blur the image
    grey = cv2.blur(grey,(7,7))
    # Compute edges using the Canny edge filter
    if param_flag:
        param1=cv2.getTrackbarPos('param1',win3_name)
        param2=cv2.getTrackbarPos('param2',win3_name)
        edges = cv2.Canny(grey,param1, param2)
    else:
        edges = cv2.Canny(grey, 15.0, 30.0)

    return grey, edges

def setup_rgb():
    cv2.namedWindow(win1_name,flags=cv2.WINDOW_NORMAL)
    cv2.namedWindow(win2_name,flags=cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win1_name,640,480)
    cv2.resizeWindow(win2_name,640,480)
    cv2.moveWindow(win1_name,0,0)
    cv2.moveWindow(win2_name,0,850)
    

def callback_rgb(data):    
    global rgb_flag
    if rgb_flag == 0:
        setup_rgb()
        rgb_flag = 1
    # Use cv_bridge() to convert the ROS image to OpenCV format
    frame = cvbridge.imgmsg_to_cv2(data, "bgr8")
    frame = np.array(frame, dtype=np.uint8)
    cv2.imshow(win1_name, frame)

    grey,edges=rgb2edges(frame)
    # cv2.imshow('blur', grey)
    cv2.imshow(win2_name, edges)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:         # wait for ESC key to exit
        rospy.signal_shutdown("User hit ESC key to quit.")

def setup_param():
    cv2.namedWindow(win3_name,flags=cv2.WINDOW_NORMAL)
    cv2.createTrackbar('param1',win3_name,20,50,nothing)
    cv2.createTrackbar('param2',win3_name,30,50,nothing)
    cv2.resizeWindow(win3_name,640,240)
    cv2.moveWindow(win3_name,700,850)

def setup_depth():
    cv2.namedWindow(win4_name,flags=cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win4_name,640,480)
    cv2.moveWindow(win4_name,700,0)
    # cv2.setMouseCallback(win4_name,nothing)

def callback_depth(data):
    global depth_flag, param_flag
    if depth_flag == 0:
        setup_depth()
        depth_flag = 1

    # Use cv_bridge() to convert the ROS image to OpenCV format
    frame = cvbridge.imgmsg_to_cv2(data) #, "32FC1"
    frame = np.array(frame, dtype=np.float32)
    cv2.setMouseCallback(win4_name,mouse_callback,frame)
    # frame=np.nan_to_num(frame)
    maxim = np.nanmax(frame)
    if param_flag:
        param1=cv2.getTrackbarPos('param1',win3_name)
        maxim = (param1/25.)*maxim
    frame=cv2.convertScaleAbs(frame,frame,255/maxim)
    
    cv2.imshow(win4_name, frame)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:         # wait for ESC key to exit
        rospy.signal_shutdown("User hit ESC key to quit.")

def mouse_callback(event,x,y,flags,param):
    # print param.__class__
    if event == cv2.EVENT_LBUTTONDOWN:
        # print np.shape(param)
        print "X value: ", x
        print "Y value:", y
        print "Depth in meters:", param[y,x]

rospy.init_node(node_name)
rospy.on_shutdown(cleanup)
# cv2.startWindowThread()
cvbridge = CvBridge()

if param_flag:
    setup_param()

print('==========Initializing Subscribers=====')
sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, callback_rgb)
sub_depth = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, callback_depth)
rospy.sleep(1)

rospy.spin()