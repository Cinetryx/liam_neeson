#!/usr/bin/env python
# import numpy as np
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from math import sqrt
from image_geometry import PinholeCameraModel as PC
from time import sleep


def hough_rgb(img):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(img,5)
    if (depth_flag ==1):
        # img= cv2.bitwise_and(img, img, mask = depth_mask[...,np.newaxis]) #testing out depth mask
        img = img * (depth_mask/255)
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,2.0,150,param1=9,param2=24,minRadius=15,maxRadius=55)
    circles = np.round(circles[0, :]).astype("int")
    if circles is not None:
        for (x,y,r) in circles:
            # print col_img[x,y]
            cv2.circle(img,(x,y),r,(0,255,0),2)
            cv2.circle(img,(x,y),2,(0,0,255),3)
    cv2.imshow('hough_rgb',img)
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
    if circles is not None:
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

def dep_mask(img):
    lower = 0
    upper = 4
    mask = cv2.inRange(img, lower, upper)
    # mask = (np.dstack((mask,mask,mask)))
    return mask

def setup_feed():
    cv2.namedWindow(win1_name,flags=cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win1_name,640,480)
    cv2.moveWindow(win1_name,500,0)

def callback_rgb(data):    
    global rgb_image,circle_coord
    global rgb_flag
    if rgb_flag == 0:
        rgb_flag = 1
    # Use cv_bridge() to convert the ROS image to OpenCV format
    frame = cvbridge.imgmsg_to_cv2(data, "bgr8")
    frame = np.array(frame, dtype=np.uint8)
    circles_grey= hough_rgb(frame)
    circles_mask= hough_mask(frame)
    if (circles_grey is not None) and (circles_mask is not None):
        x,y,r=circle_compare(circles_grey,circles_mask)
        circle_coord=[x,y,r]
        if x:
            cv2.circle(frame,(x,y),r,(0,255,0),2)
            cv2.circle(frame,(x,y),2,(0,0,255),3)
    else:
        circle_coord=[0,0,0]
    rgb_image=frame

def callback_depth(data):    
    global circle_coord, obj_depth
    global depth_mask
    global depth_image
    global depth_flag
    if (rgb_flag == 1) and (not circle_coord[0]==0):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        frame = cvbridge.imgmsg_to_cv2(data) #, "32FC1"
        frame = np.array(frame, dtype=np.float32)
        # frame=np.nan_to_num(frame)
        # maxim = np.nanmax(frame)
        # frame=cv2.convertScaleAbs(frame,frame,255/maxim)

        depth_image= frame
        depth_mask = dep_mask(frame)
        if depth_flag == 0:
            depth_flag = 1
        measured_depth = frame[circle_coord[0],circle_coord[1]]
        if (measured_depth>0):
            obj_depth=measured_depth
        # print obj_depth



def cleanup():
    print "Have a nice day."
    cv2.destroyAllWindows()


win1_name='Feed'

circle_coord=[0,0,0]
obj_depth=[]
obj_coord=[]
rgb_image=np.array([0])
depth_image=np.array([0])
depth_mask=np.array([0])


node_name='opencv_depth_registered_test_node'
rgb_flag=0
depth_flag=0

rospy.init_node(node_name)
rospy.on_shutdown(cleanup)
# cv2.startWindowThread()
cvbridge = CvBridge()

print('==========Initializing Subscribers=====')

cam_info=rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=None)
sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, callback_rgb)
sub_depth = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, callback_depth)

sleep(2)

setup_feed()
imgproc=PC()
imgproc.fromCameraInfo(cam_info)
# print cam_info.P
# print (np.dstack((depth_mask,depth_mask,depth_mask))).shape
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    # cv2.imshow(win1_name, np.hstack([rgb_image,depth_image]))

    cv2.imshow(win1_name,rgb_image)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:         # wait for ESC key to exit
        rospy.signal_shutdown("User hit ESC key to quit.")
    obj_coord= np.array(imgproc.projectPixelTo3dRay((circle_coord[0],circle_coord[1])))
    # scale=obj_depth/obj_coord[2] #scale the unit vector according to measured depth
    # obj_coord=obj_coord*scale
    # print obj_coord
    rate.sleep()


rospy.spin()