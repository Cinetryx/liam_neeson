#!/usr/bin/env python

import cv2
import numpy as np
import sys

filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/Positiv/20160210_155220.jpg"

if len(sys.argv)>1:
    filename = sys.argv[1]
img = cv2.imread(filename,1)


cascade = cv2.CascadeClassifier("/home/hongalan/skelws/src/opencv-haar-classifier-training/trained_classifiers/cascade.xml")
if not cascade.empty():
    print "Let's get started!"
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gray = cv2.equalizeHist(gray)
    rects = cascade.detectMultiScale(img, scaleFactor=1.05, minNeighbors=2, minSize=(20, 20), flags=cv2.CASCADE_SCALE_IMAGE)
    if not(len(rects) == 0):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
cv2.imshow('Classifier',img)
key = cv2.waitKey(0) & 0xFF
if key == 27:         # wait for ESC key to exit
    cv2.destroyAllWindows()