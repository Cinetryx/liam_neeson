#!/usr/bin/env python

#Given a trained classifier (using Haar-like features) and reference image of the object (for color comparison),
#publishes the calculated position of the object using rgb and depth camera images

#Currently in development: 
#using different classifiers 
#object tracking (allows for faster object location after initial search)

import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from math import sqrt
from image_geometry import PinholeCameraModel as PC
from time import sleep
from obj_classifier_timed import Classifier
import random


class Seeker():
    def __init__(self,classifier,cvbridge):
        self.classifier = classifier
        self.cvbridge = cvbridge
        cam_info=rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=None)
        self.imgproc=PC()
        self.imgproc.fromCameraInfo(cam_info)
        self.rect = np.array([np.empty((0,4))]) #Array of most recent valid rectangles
        self.xyz = np.array([np.empty((0,3))])  #Array of most recent valid xyz position
        self.vel = np.array([np.zeros(3)])
    def seek(self):
        self.initial_seek()
    def track(self):
        if len(self.xyz==0):
            print "Seeker: track() called before position history exists"
            self.initial_seek() #might as well do so here
            return
        avg_rect = np.zeros(4)
        avg_vel = np.zeros(3)
        n = len(self.rect)
        for i in range(n):
            avg_rect += self.rect[i]*(i+1) #greater weight given to more recent values
            if i>0:
                vel = (self.xyz[i]-self.xyz[i-1])*i
                avg_vel+=vel
        avg_rect = np.int32(avg_rect / (n*(n+1)/2))
        avg_vel = avg_vel / (n*(n+1)/2)
        obj_coord = np.array(self.imgproc.projectPixelTo3dRay((centroid_x,centroid_y)))


    def append_history(rect,xyz):
        #newest valid values are last in the list
        self.rect = np.append(self.rect,rect,0)
        self.xyz = np.append(self.xyz,xyz,0)
        if len(self.rect)>5: #both history arrays should be of the same length, so no need to check both
            #remove values beyond the 5th
            self.rect = self.rect[-5:]
            self.xyz = self.xyz[-5:]

    def initial_seek(self):
        # print "Seeker: Running initial_seek()"
        rect = self.classifier.initial_search_timed(2)
        if not len(rect) == 0:
            xyz = self.get_xyz(rect)
            if len(xyz) > 0:
                # print xyz
                #initializes history
                self.rect = np.array([rect])
                self.xyz = np.array([xyz])
                self.vel = np.array([np.zeros(3)])
            else:
                print "Couldn't retrieve depth info. Make sure object is at least 3 ft from camera and unobstructed."

    def get_xyz(self,rect):
        # data = rospy.wait_for_message("/camera/depth_registered/hw_registered/image_rect", Image, timeout=None)
        data = self.classifier.subscriber.depth_img #depth image should now match up with whatever rgb_image was last used in the classifier
        img = self.cvbridge.imgmsg_to_cv2(data) #, "32FC1"
        img = np.array(img, dtype=np.float32)
        x1,y1,x2,y2 = rect
        frame = img[y1:(y1+y2),x1:(x1+x2)]
        mask = np.nan_to_num(frame)
        #takes mean of center pixel values, assuming object exists there and there's no hole in the object's depth map
        seg = mask[int(0.45*y2):int(0.55*y2),int(0.45*x2):int(0.55*x2)]
        seg_mean = np.mean(seg) #approximate depth value of the object. Take mean of remaining object pixels
        # print "Seg_mean: ", seg_mean
        if (seg_mean > 0.25): #if there's no hole in the map. Minimum operating distance is about 0.5 meters
            seg_std = np.std(seg)
            ind = np.nonzero(np.abs(mask-seg_mean) < seg_std) # Valid region should be within one standard deviation of the approximate mean[?]
            mask_depth = mask[ind]
            obj_depth = np.mean(mask_depth)
            # print "Mean depth: ", obj_depth

        else: #backup method in case hole exists
            ind_depth_calc = np.nonzero((mask>0.25) & (mask<5)) #
            mask_depth = mask[ind_depth_calc]
            mask_mean = np.mean(mask_depth)
            mask_std = np.std(mask_depth)
            # print "Mean depth and std: ", mask_mean, mask_std
            ind = np.nonzero((np.abs(mask-mask_mean) < (max(0.25,0.5*mask_std))))
            if np.size(ind) < 0.5 *(np.size(mask)):
                return np.empty((0,3))
            obj_depth = np.mean(mask[ind]) # mean depth value of pixels representing object

        # mask_depth = np.uint8(mask_depth*(255/np.max(mask_depth))) #normalize to greyscale
        # mask = np.uint8(mask*(255/np.max(mask_depth)))
        # retval,mask = cv2.threshold(mask,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU) #threshold was designed for greyscale, but this works too.
        
        # kernel = np.ones((5,5),np.uint8)
        # mask = cv2.erode(mask,kernel,iterations = 1) #[?]
        # ind = np.nonzero(mask)

        # print "Object Depth: ", obj_depth

        row = ind[0]+y1
        column = ind[1]+x1
        # blah = np.zeros(np.shape(img))
        # blah[(row,column)] = 255
        # cv2.imshow("Blah",blah)
        # cv2.waitKey(0)

        centroid_y = np.mean(row)
        centroid_x = np.mean(column)



        obj_coord = np.array(self.imgproc.projectPixelTo3dRay((centroid_x,centroid_y)))
        scale=obj_depth/obj_coord[2] #scale the unit vector according to measured depth
        # print "Centroid y,x,ray,depth: ", centroid_y,centroid_x,obj_coord,obj_depth
        obj_coord_sc=obj_coord*scale
    
        return obj_coord_sc

def setup_feed(win_name):
    cv2.namedWindow(win_name,flags=cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name,640,480)
    cv2.moveWindow(win_name,500,0)


def cleanup():
    print "Have a nice day."
    cv2.destroyAllWindows()


def main():
    rospy.init_node("obj_dregistered")
    rospy.on_shutdown(cleanup)
    
    # cascade = cv2.CascadeClassifier("/home/hongalan/skelws/src/opencv-haar-classifier-training/trained_classifiers/maruchan_asus15.xml")
    # ref_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/maruchan_train2.avi_9765_0000_0266_0148_0143_0148.png"
    if len(sys.argv)>2:
        ref_img = cv2.imread(sys.argv[1],1)
        cascade = cv2.CascadeClassifier(sys.argv[2])
    else:
        print "=====obj_dregistered node requires 2 arguments: ref image and classifier"
        return
    if cascade.empty():
        print "Classifier not found"
        return

    print "Let's get started!"
    cvbridge = CvBridge()
    classifier=Classifier(cascade,cvbridge,ref_img)
    # classifier.initial_search_timed(5)
    seeker = Seeker(classifier,cvbridge)

    pub = rospy.Publisher('/obj_position', Point, queue_size=2)
    
    while not rospy.is_shutdown():
        rate=rospy.Rate(1)
        try:
            seeker.initial_seek()
            if len(seeker.xyz>0):
                x,y,z = seeker.xyz[-1] #get latest point found
                print "Point: ", seeker.xyz[-1]
                point_msg=Point()
                point_msg.x=x
                point_msg.y=y
                point_msg.z=z
                pub.publish(point_msg)

            rate.sleep()
        except:
            pass

    # rospy.spin()

# print('==========Initializing Subscribers=====')


# sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, callback_rgb)
# sub_depth = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, callback_depth)
# cam_info=rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=None)
# sleep(2)

# setup_feed()
# imgproc=PC()
# imgproc.fromCameraInfo(cam_info)
# # print cam_info.P
# rate = rospy.Rate(20)
# while not rospy.is_shutdown():
#     # cv2.imshow(win1_name, np.hstack([rgb_image,depth_image]))
#     cv2.imshow(win1_name,rgb_image)
#     key = cv2.waitKey(1) & 0xFF
#     if key == 27:         # wait for ESC key to exit
#         rospy.signal_shutdown("User hit ESC key to quit.")
#     print obj_coord, obj_depth
#     if not (np.isnan(obj_depth) or obj_depth==[]):
#         obj_coord= np.array(imgproc.projectPixelTo3dRay((circle_coord[0],circle_coord[1])))
#         scale=obj_depth/obj_coord[2] #scale the unit vector according to measured depth
#         obj_coord_sc=obj_coord*scale
#         print obj_coord_sc
#     else:
#         obj_coord = []
#     rate.sleep()



if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        cleanup()