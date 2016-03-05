#!/usr/bin/env python

import cv2
import numpy as np
import sys
from time import sleep
from math import sqrt

win3_name='mask parameters'
win4_name='rgb parameters'

a0=2.7
a1=163
a=20
b=17
c=29
d=47
g_lo=0
b_lo=0
r_lo=113
g_hi=67
b_hi=44
r_hi=255

i=0.4
j=147
k=9
l=48
m=23
n=55

circles_mask=0
circles_rgb=0
flag_mask=1
flag_rgb=1

def param_update(x):
    global g_lo,b_lo,r_lo,g_hi,b_hi,r_hi,a0,a1,a,b,c,d,i,j,k,l,m
    a0=cv2.getTrackbarPos('param00',win3_name)/10
    a1=cv2.getTrackbarPos('param01',win3_name)
    a=cv2.getTrackbarPos('param1',win3_name)
    b=cv2.getTrackbarPos('param2',win3_name)
    c=cv2.getTrackbarPos('param3',win3_name)
    d=cv2.getTrackbarPos('param4',win3_name)
    g_lo=cv2.getTrackbarPos('green_lower',win3_name)
    b_lo=cv2.getTrackbarPos('blue_lower',win3_name)
    r_lo=cv2.getTrackbarPos('red_lower',win3_name)
    g_hi=cv2.getTrackbarPos('green_upper',win3_name)
    b_hi=cv2.getTrackbarPos('blue_upper',win3_name)
    r_hi=cv2.getTrackbarPos('red_upper',win3_name)
    i=cv2.getTrackbarPos('param_i',win4_name)/10
    j=cv2.getTrackbarPos('param_j',win4_name)
    k=cv2.getTrackbarPos('param_k',win4_name)
    l=cv2.getTrackbarPos('param_l',win4_name)
    m=cv2.getTrackbarPos('param_m',win4_name)
    n=cv2.getTrackbarPos('param_n',win4_name)

    if a0==0: a0=1
    if a1==0: a1=1
    if a==0: a=1
    if b==0: b=1
    if i==0: i=1
    if j==0: j=1
    if k==0: k=1
    if l==0: l=1

def setup_mask_param():
    cv2.namedWindow(win3_name,flags=cv2.WINDOW_NORMAL)
    cv2.createTrackbar('param00',win3_name,27,100,param_update)
    cv2.createTrackbar('param01',win3_name,163,400,param_update)
    cv2.createTrackbar('param1',win3_name,30,100,param_update)
    cv2.createTrackbar('param2',win3_name,17,100,param_update)
    cv2.createTrackbar('param3',win3_name,29,500,param_update)
    cv2.createTrackbar('param4',win3_name,47,1500,param_update)
    cv2.createTrackbar('green_lower',win3_name,0,255,param_update)
    cv2.createTrackbar('blue_lower',win3_name,0,255,param_update)
    cv2.createTrackbar('red_lower',win3_name,113,255,param_update)
    cv2.createTrackbar('green_upper',win3_name,67,255,param_update)
    cv2.createTrackbar('blue_upper',win3_name,44,255,param_update)
    cv2.createTrackbar('red_upper',win3_name,255,255,param_update)
    cv2.resizeWindow(win3_name,640,240)
    cv2.moveWindow(win3_name,700,850)

def setup_rgb_param():
    cv2.namedWindow(win4_name,flags=cv2.WINDOW_NORMAL)
    cv2.createTrackbar('param_i',win4_name,4,100,param_update)
    cv2.createTrackbar('param_j',win4_name,147,400,param_update)
    cv2.createTrackbar('param_k',win4_name,9,100,param_update)
    cv2.createTrackbar('param_l',win4_name,48,100,param_update)
    cv2.createTrackbar('param_m',win4_name,23,500,param_update)
    cv2.createTrackbar('param_n',win4_name,55,500,param_update)
    cv2.resizeWindow(win4_name,640,240)
    cv2.moveWindow(win4_name,1200,850)

def circle_compare(circles_mask,circles_rgb):
    # global circles_mask, circles_rgb
    for (x1,y1,r1) in circles_mask:
        for (x2,y2,r2) in circles_rgb:
            if (sqrt(((x1-x2)**2) + ((y1-y2)**2)) < 20) and (abs(r1-r2) < 20):
                # print sqrt(((x1-x2)**2) + ((y1-y2)**2)),abs(r1-r2)
                return (x1+x2)/2, (y1+y2)/2, (r1+r2)/2
    return 0,0,0


print "Here goes..."



# print sys.argv
if len(sys.argv)>1:
    if sys.argv[1] == '1':
        flag_rgb=0
    elif sys.argv[1] == '2':
        flag_mask=0

if flag_mask:
    setup_mask_param()
if flag_rgb:
    setup_rgb_param()

if flag_mask and flag_rgb:
    cv2.namedWindow('common',flags=cv2.WINDOW_NORMAL)
    cv2.resizeWindow('common',1920,480)

while True:   
    if flag_mask:
        col_img = cv2.imread('/home/hongalan/Desktop/Courses/d_Winter_Project/Progress/1_a1_camera_rgb_image_color.jpg')
        lower= np.array([g_lo, b_lo, r_lo], dtype="uint8")
        upper= np.array([g_hi, b_hi, r_hi], dtype="uint8")
        mask = cv2.inRange(col_img, lower, upper)
        output = cv2.bitwise_and(col_img, col_img, mask = mask)
        # cv2.imshow("images", np.hstack([col_img, output]))
        circles = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,a0,a1,param1=a,param2=b,minRadius=c,maxRadius=d)
            # param1=50,param2=20,minRadius=0,maxRadius=100)
        if circles is not None:
            # print "yay for masks"
            circles = np.round(circles[0, :]).astype("int")
            for (x,y,r) in circles:
                # print col_img[x,y]
                cv2.circle(output,(x,y),r,(0,255,0),2)
                cv2.circle(output,(x,y),2,(0,0,255),3)
        global circles_mask
        circles_mask=circles
        # cv2.imshow("mask",output)
  
    if flag_rgb:
        img = cv2.imread('/home/hongalan/Desktop/Courses/d_Winter_Project/Progress/1_a1_camera_rgb_image_color.jpg',0)
        img = cv2.medianBlur(img,5)
        # img = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,i,j,param1=k,param2=l,minRadius=m,maxRadius=n)
        # circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,0.4,147,param1=9,param2=48,minRadius=23,maxRadius=55)
        img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
        # param1=100,param2=100,minRadius=0,maxRadius=800
        if circles is not None:
            # print "yay for rgb"
            # circles = np.uint16(np.around(circles))
            circles = np.round(circles[0, :]).astype("int")
            # print circles
            for (x,y,r) in circles:
                # if mask[x,y]>0:
                # print col_img[x,y]
                # print x,y,r
                # draw the outer circle
                cv2.circle(img,(x,y),r,(0,255,0),2)
                # draw the center of the circle
                cv2.circle(img,(x,y),2,(0,0,255),3)
        global circles_rgb
        circles_rgb=circles
        # cv2.imshow('rgb',img)
        
    if flag_mask and flag_rgb:
        common_img = cv2.imread('/home/hongalan/Desktop/Courses/d_Winter_Project/Progress/1_a1_camera_rgb_image_color.jpg')
        if (circles_mask is not None) and (circles_rgb is not None):
            x,y,r = circle_compare(circles_mask,circles_rgb)
            if x:
                cv2.circle(common_img,(x,y),r,(0,255,0),2)
                cv2.circle(common_img,(x,y),2,(0,0,255),3)
        # cv2.imshow('common',common_img)
        cv2.imshow("common", np.hstack([output,common_img,img,]))
    elif flag_mask:
        cv2.imshow("mask",output)
    elif flag_rgb:
        cv2.imshow('rgb',img)

    key = cv2.waitKey(100) & 0xFF
    if key == 27:         # wait for ESC key to exit
        break

# cv2.destroyAllWindows()