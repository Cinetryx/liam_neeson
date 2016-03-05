#!/usr/bin/env python
# import numpy as np
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from math import sqrt


win1_name='rgb_image'

node_name='opencv_test_node'
rgb_flag=0
param_flag=0
depth_flag=0

def hough_rgb(img):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(img,5)
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,2.0,150,param1=9,param2=24,minRadius=15,maxRadius=55)
    circles = np.round(circles[0, :]).astype("int")
    return circles

def hough_mask(img):
    a0=4.5
    a1=150
    a=20
    b=10
    c=15
    d=55
    g_lo=0
    b_lo=0
    r_lo=113
    g_hi=67
    b_hi=44
    r_hi=255
    lower= np.array([g_lo, b_lo, r_lo], dtype="uint8")
    upper= np.array([g_hi, b_hi, r_hi], dtype="uint8")
    mask = cv2.inRange(img, lower, upper)
    circles = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,a0,a1,param1=a,param2=b,minRadius=c,maxRadius=d)
    circles = np.round(circles[0, :]).astype("int")
    return circles

def circle_compare(circles_grey,circles_mask):
    # global circles_mask, circles_rgb
    for (x1,y1,r1) in circles_mask:
        for (x2,y2,r2) in circles_grey:
            if (sqrt(((x1-x2)**2) + ((y1-y2)**2)) < 20) and (abs(r1-r2) < 20):
                # print sqrt(((x1-x2)**2) + ((y1-y2)**2)),abs(r1-r2)
                return (x1+x2)/2, (y1+y2)/2, (r1+r2)/2
    return 0,0,0

def setup_rgb():
    cv2.namedWindow(win1_name,flags=cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win1_name,640,480)
    cv2.moveWindow(win1_name,0,0)

def callback_rgb(data):    
    global rgb_flag
    if rgb_flag == 0:
        setup_rgb()
        rgb_flag = 1
    # Use cv_bridge() to convert the ROS image to OpenCV format
    frame = cvbridge.imgmsg_to_cv2(data, "bgr8")
    frame = np.array(frame, dtype=np.uint8)
    circles_grey= hough_rgb(frame)
    circles_mask= hough_mask(frame)
    if (circles_grey is not None) and (circles_mask is not None):
        x,y,r=circle_compare(circles_grey,circles_mask)
        if x:
            cv2.circle(frame,(x,y),r,(0,255,0),2)
            cv2.circle(frame,(x,y),2,(0,0,255),3)
    cv2.imshow(win1_name, frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:         # wait for ESC key to exit
        rospy.signal_shutdown("User hit ESC key to quit.")

def cleanup():
    print "Have a nice day."
    cv2.destroyAllWindows()

rospy.init_node(node_name)
rospy.on_shutdown(cleanup)
# cv2.startWindowThread()
cvbridge = CvBridge()

print('==========Initializing Subscribers=====')
sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, callback_rgb)

rospy.spin()