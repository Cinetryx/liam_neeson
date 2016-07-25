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
from copy import copy

from time import clock


class Seeker():
    def __init__(self,classifier,cvbridge):
        self.classifier = classifier
        self.cvbridge = cvbridge
        cam_info=rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=None)
        # print "Waiting for image data"
        # while not (len(self.classifier.subscriber.depth_img)>0):
        #     if not rospy.is_shutdown():
        #         rospy.sleep(1)
        #         print self.classifier.subscriber.depth_img
        #     else: break
        # data = self.classifier.subscriber.depth_img #depth image should now match up with whatever rgb_image was last used in the classifier
        # img = self.cvbridge.imgmsg_to_cv2(data) #, "32FC1"
        # img = np.array(img, dtype=np.float32)
        self.img_shape = (480,640) #np.shape(img)
        self.imgproc=PC()
        self.imgproc.fromCameraInfo(cam_info)
        self.rect = np.array([np.empty((0,4))]) #Array of most recent valid rectangles
        self.xyz = np.array([np.empty((0,3))])  #Array of most recent valid xyz position
        self.vel = np.array([np.zeros(3)])

    def seek(self):
        t = 0.25
        scale = 1.5
        # rospy.sleep(1)
        rect = self.track(t,scale)
        if not np.size(rect)>0:
            print "Trying again with larger rectangle"
            for i in range(3): #let's try a couple more times...
                t += 1
                scale = scale * 1.5
                rect = self.track(t,scale)
                if np.size(rect)>0:
                    break
            if not np.size(rect)>0:
                print "seek: Object lost. Re-initializing search"
                self.rect = np.array([np.empty((0,4))])
                self.xyz = np.array([np.empty((0,3))])
                self.vel = np.array([np.zeros(3)])
                return

        xyz = self.get_xyz(rect)

        if len(xyz) > 0:
            self.append_history(rect,xyz)
        else:
            print "Couldn't retrieve depth info. Make sure object is at least 3 ft from camera and unobstructed."
            print "Trying again with larger rectangle"
            for i in range(3): #let's try a couple more times...
                t += 1
                scale = scale * 1.5
                rect = self.track(t,scale)
                if len(rect)>0:
                    break
            if not len(rect)>0:
                print "\n ===== seek: Object lost. Re-initializing search \n"
                self.rect = np.array([np.empty((0,4))])
                self.xyz = np.array([np.empty((0,3))])
                self.vel = np.array([np.zeros(3)])


        #displaying feed
        disp_img = self.classifier.subscriber.rgb_img
        disp_img = self.cvbridge.imgmsg_to_cv2(disp_img, "bgr8")
        disp_img = np.array(disp_img, dtype=np.uint8)
        #overlaying result if available
        if len(rect)>0:
            # print "rect: ",rect
            (x1,y1,x2,y2)=np.int32(rect[0])
            cv2.rectangle(disp_img, (x1, y1), (x1+x2, y1+y2), (0, 255, 0), 2)
            text = "X: %.1f Y: %.1f Z: %.1f" %(xyz[0],xyz[1],xyz[2])
            cv2.putText(disp_img,text, (x1-10,y1-10),cv2.FONT_HERSHEY_PLAIN,1,(0, 255, 0),1)
        cv2.imshow("Seek",disp_img)
        cv2.waitKey(1)

    def calc_avg(self):
        avg_rect = np.zeros(4)
        avg_xyz = np.zeros(3)
        avg_vel = np.zeros(3)
        n = len(self.rect)
        for i in range(n):
            avg_rect += self.rect[i]*(i+1) #greater weight given to more recent values
            avg_xyz += self.xyz[i]*(i+1)
            if i>0:
                vel = (self.xyz[i]-self.xyz[i-1])*i
                avg_vel+=vel
        avg_rect = np.int32(avg_rect / (n*(n+1)/2))
        avg_xyz = avg_xyz / (n*(n+1)/2)
        if n>1:
            avg_vel = avg_vel / (n*(n+1)/2)
        return (avg_rect, avg_xyz, avg_vel)

    def track(self,t,scale):
        # t represents estimated time elapsed between the last image obtained and the coming one... one might guess 1 second... this can be made adaptive with time history data... [?]
        if not len(self.xyz[0])>0:
            # print "Seeker: track() called before position history exists"
            self.initial_seek() #might as well do so here
            return self.rect
        
        (avg_rect, avg_xyz, avg_vel) = self.calc_avg()
        
        guess_xyz = avg_xyz + t * avg_vel
        gx,gy,gz = guess_xyz
        (g_centroid_x, g_centroid_y) = np.array(self.imgproc.project3dToPixel((gx,gy,gz)))

        ax1,ay1,ax2,ay2 = avg_rect
        ax2 = ax2 * scale
        ay2 = ay2 * scale
        a_centroid_x = ax1 + (ax2/2)
        a_centroid_y = ay1 + (ay2/2)

        new_centroid_x = np.int((a_centroid_x + g_centroid_x)/2) #centroid of new rectangle encompassing both current rectangle and guessed rectangle
        new_centroid_y = np.int((a_centroid_y + g_centroid_y)/2)
        new_x2 = np.abs(a_centroid_x - g_centroid_x) + ax2  #proposed rectangle width
        new_y2 = np.abs(a_centroid_y - g_centroid_y) + ay2  #proposed rectangle height
        new_x1 = max(0,new_centroid_x - np.int(new_x2/2))   #make sure new rectangle's top & left edges are within image
        new_y1 = max(0,new_centroid_y - np.int(new_y2/2))
        new_x2 = min(new_x2, self.img_shape[1] - new_x1) #make sure new rectangle's bottom & right edges are within image
        new_y2 = min(new_y2, self.img_shape[0] - new_y1)

        new_rect = np.array([new_x1,new_y1,new_x2,new_y2])
        # print "track: new_rect: ", new_rect
        #######REPLACE THIS WITH SOMETHING FASTER TT__TT
        # rect = self.classifier.seg_search(new_rect)
        rect = self.seg_track(new_rect)
        # print "track: rect: ",rect
        return rect

    def append_history(self,rect,xyz):
        # newest valid values are last in the list
        # print "Appending: ",rect, xyz
        # print "Current Rect, Current xyz: ", self.rect,self.xyz
        rect = np.array(rect).reshape(4)
        xyz = np.array(xyz).reshape(3)
        self.rect = np.append(self.rect,[rect],0)
        self.xyz = np.append(self.xyz,[xyz],0)
        # print "Stored Rect, Stored xyz: ", self.rect,self.xyz
        if len(self.rect)>5: #both history arrays should be of the same length, so no need to check both
            #remove values beyond the 5th
            self.rect = self.rect[-5:]
            self.xyz = self.xyz[-5:]

    def initial_seek(self):
        print "Searching for object. Don't move!"
        rect = self.classifier.initial_search_timed(2)
        if len(rect) > 0:
            # print "Rect: ", rect
            xyz = self.get_xyz(rect)
            if len(xyz) > 0:
                # print xyz
                #initializes history
                self.rect = np.array([rect])
                self.xyz = np.array([xyz])
                self.vel = np.array([np.zeros(3)])
            else:
                print "Couldn't retrieve depth info. Make sure object is at least 3 ft from camera and unobstructed."

    def seg_track(self,seg_rect):
        # A much faster but potentially less accurate tracking function using depth data instead of trained classifier
        # Given rect coord potentially containing object, returns rect around object if found

        (rgb_frame,depth_frame) = self.classifier.get_feed() #retrieves updated rgb and depth images
        # disp_frame = copy(rgb_frame)
        
        # cv2.rectangle(disp_frame, (x1, y1), (x1+x2, y1+y2), (0, 255, 0), 2)
        # cv2.imshow('Searching',disp_frame)
        # cv2.waitKey(0)

        seg_rect = np.array(seg_rect).reshape(4)
        x1,y1,x2,y2 = seg_rect

        depth_seg = depth_frame[y1:(y1+y2),x1:(x1+x2)]

        (avg_rect, avg_xyz, avg_vel) = self.calc_avg()
        
        if (avg_vel[2]>0):
            depth_min = avg_xyz[2]
            depth_max = depth_min + avg_vel[2] * 0.5 #predicts how far the object may have travelled in... 0.5 sec [?]
        else:
            depth_max = avg_xyz[2]
            depth_min = depth_max + avg_vel[2] * 0.5

        mask = np.nan_to_num(depth_seg)

        ind = np.nonzero(( (mask>(depth_min-0.05)) & (mask<(depth_max+0.05)) ))
        mask = np.zeros(np.shape(mask))

        mask[ind] = 1

        kernel = np.ones((10,10),np.uint8)
        # mask = cv2.erode(mask,kernel,iterations = 1)
        # mask = cv2.dilate(mask,kernel,iterations = 1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) #clean noise
        # cv2.imshow('Searching',mask)
        # cv2.waitKey(0)
        ind = np.nonzero(mask)

        if (len(ind[0])==0):
            return np.empty((0,4))

        # calculation accuracy rely on there being no other objects of similar depth in the segment
        dx2 = np.int32(0.5 * ((max(ind[1]) - min(ind[1])) + avg_rect[2]))
        dy2 = np.int32(0.5 * ((max(ind[0]) - min(ind[0])) + avg_rect[3]))
        dcentroid_x = np.int32(np.mean(ind[1]))
        dcentroid_y = np.int32(np.mean(ind[0]))
        # dx2 = dy2 = (dx2+dy2)/2
        dx1 = max(dcentroid_x - (dx2/2),0)
        dy1 = max(dcentroid_y - (dy2/2),0)
        dx2 = min(dx2,self.img_shape[1]-dx1)
        dy2 = min(dy2,self.img_shape[0]-dy1)

        sdx1 = x1+dx1
        sdy1 = y1+dy1
        dseg = rgb_frame[sdy1:(sdy1+dy2),sdx1:(sdx1+dx2)]
        # cv2.imshow('Searching',dseg)
        # cv2.waitKey(0)
        #if the image segment doesn't pass the color histogram test, object isn't in the segment
        if self.classifier.hist_test.compare(dseg):
            rect = np.array([[sdx1,sdy1,dx2,dy2]])
        else:
            rect = np.empty((0,4))

        return rect

    def get_xyz(self,rect):
        # data = rospy.wait_for_message("/camera/depth_registered/hw_registered/image_rect", Image, timeout=None)

        data = self.classifier.subscriber.depth_img #depth image should now match up with whatever rgb_image was last used in the classifier
        img = self.cvbridge.imgmsg_to_cv2(data) #, "32FC1"
        img = np.array(img, dtype=np.float32)
        rect = np.array(rect).reshape(4,)
        x1,y1,x2,y2 = rect
        frame = img[y1:(y1+y2),x1:(x1+x2)]
        mask = np.nan_to_num(frame)
        #takes mean of center pixel values, assuming object exists there and there's no hole in the object's depth map
        seg = mask[int(0.45*y2):int(0.55*y2),int(0.45*x2):int(0.55*x2)]
        seg_mean = np.mean(seg) #approximate depth value of the object. Take mean of remaining object pixels

        if (seg_mean > 0.25): #if there's no hole in the map. Minimum operating distance is about 0.5 meters
            seg_std = np.std(seg)
            ind = np.nonzero(np.abs(mask-seg_mean) < ((1.5 * seg_std) + 0.025)) # Valid region should be within one standard deviation of the approximate mean[?]
            if np.size(ind) == 0:
                return np.empty((0,3))
            mask_depth = mask[ind]
            obj_depth = np.mean(mask_depth)

        else: #backup method in case hole exists
            ind_depth_calc = np.nonzero((mask>0.25) & (mask<5)) #
            mask_depth = mask[ind_depth_calc]
            mask_mean = np.mean(mask_depth)
            mask_std = np.std(mask_depth)
            ind = np.nonzero((np.abs(mask-mask_mean) < (max(0.25,0.5*mask_std))))
            if np.size(ind) < np.int(0.5 *(np.size(mask))):
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
        # cv2.waitKey(1)

        centroid_y = np.mean(row)
        centroid_x = np.mean(column)


        obj_coord = np.array(self.imgproc.projectPixelTo3dRay((centroid_x,centroid_y)))
        scale=obj_depth/obj_coord[2] #scale the unit vector according to measured depth
        # print "Centroid y,x,ray,depth: ", centroid_y,centroid_x,obj_coord,obj_depth
        obj_coord_sc=obj_coord*scale
        return obj_coord_sc

def setup_feed(win_name):
    cv2.namedWindow(win_name,flags=cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name,1280,960)
    cv2.moveWindow(win_name,500,0)


def cleanup():
    print "Have a nice day."
    cv2.destroyAllWindows()

def publish_xyz(pub, xyz):
    #given publisher and 3-element array of xyz real-world coordinates, publishes coordinates as Point message
    # print "\nPoint: ", xyz
    x,y,z = xyz #get latest point found
    point_msg=Point()

    point_msg.x=x
    point_msg.y=y
    point_msg.z=z
    pub.publish(point_msg)

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
    setup_feed("Seek")
    cvbridge = CvBridge()
    classifier=Classifier(cascade,cvbridge,ref_img)
    # classifier.initial_search_timed(5)
    seeker = Seeker(classifier,cvbridge)

    pub = rospy.Publisher('obj_position', Point, queue_size=2)
    
    # seeker.initial_seek()
    while not rospy.is_shutdown():
        # print "===Start timer"
        # start = rospy.Time.now()
        try:
            seeker.seek()
            if len(seeker.xyz[-1]>0):
                publish_xyz(pub,seeker.xyz[-1])
        except:
            pass
        # print "===Elapsed time: ", clock()-start
        # rospy.loginfo("===Elapsed time: %f", (rospy.Time.now() - start).to_sec())

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