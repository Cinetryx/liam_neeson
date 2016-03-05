#!/usr/bin/env python

import cv2
import numpy as np
import sys
from copy import copy

filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/negative_images/20160210_155139.jpg"

win4_name = "Parameters"

def setup_param():
    global win4_name
    cv2.namedWindow(win4_name,flags=cv2.WINDOW_NORMAL)
    cv2.createTrackbar('param_i',win4_name,105,200,param_update)
    cv2.createTrackbar('param_j',win4_name,2,200,param_update)
    cv2.createTrackbar('param_k',win4_name,20,300,param_update)
    cv2.createTrackbar('param_l',win4_name,100,300,param_update)
    cv2.createTrackbar('param_m',win4_name,0,1,param_update)
    cv2.createTrackbar('param_n',win4_name,10,10,param_update)
    cv2.resizeWindow(win4_name,640,240)
    cv2.moveWindow(win4_name,1200,850)

def param_update(data):
    pass

if len(sys.argv)>1:
    filename = sys.argv[1]
img = cv2.imread(filename,1)


cascade = cv2.CascadeClassifier("/home/hongalan/skelws/src/opencv-haar-classifier-training/trained_classifiers/maruchan.xml")
if not cascade.empty():
    print "Let's get started!"
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gray = cv2.equalizeHist(gray)

    setup_param()

    while True:
        i=max(cv2.getTrackbarPos('param_i',win4_name)*0.01, 1.001)
        j=cv2.getTrackbarPos('param_j',win4_name)
        k=cv2.getTrackbarPos('param_k',win4_name)
        l=cv2.getTrackbarPos('param_l',win4_name)
        m=cv2.getTrackbarPos('param_m',win4_name)
        n=max(cv2.getTrackbarPos('param_n',win4_name)*0.1,0.1)
        if m==0:
            input_img = copy(img)
        else:
            input_img = copy(gray)
        input_img = res = cv2.resize(input_img,None,fx=n, fy=n, interpolation = cv2.INTER_AREA)
        rects = cascade.detectMultiScale(input_img, scaleFactor=i, minNeighbors=j, minSize=(k, k), maxSize=(l, l))
            # cv2.CASCADE_SCALE_IMAGE)
        
        if not(len(rects) == 1):
            for x1, y1, x2, y2 in rects:
                cv2.rectangle(input_img, (x1, y1), (x1+x2, y1+y2), (255, 0, 0), 2)

        print "Hello"

        cv2.imshow('Classifier',input_img)
        key = cv2.waitKey(0) & 0xFF
        if key == 27:         # wait for ESC key to exit
            break

    cv2.destroyAllWindows()
else:
    print "Classifier not found"