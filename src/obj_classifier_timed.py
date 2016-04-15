#!/usr/bin/env python

import sys
from copy import copy
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from math import sqrt
from hsv_histogram import HistogramTest
from obj_subscriber import Subscriber
from copy import copy

class Classifier:
    def __init__(self,cascade,cvbridge,ref_img):
        self.cascade = cascade
        self.cvbridge = cvbridge
        self.hist_test = HistogramTest(ref_img)
        self.subscriber = Subscriber() #each object classifier gets its own set of subscribers!

    def callback_rgb(data):    # more intensive than using wait_for_message, but necessary to keep camera running
        
        if self.rgb_flag == 0:
            pass
        else:
            # Use cv_bridge() to convert the ROS image to OpenCV format
            frame = cvbridge.imgmsg_to_cv2(data, "bgr8")
            frame = np.array(frame, dtype=np.uint8)
            
            rgb_image=frame

    def cv_size(self,img):
        return tuple(img.shape[1::-1])
    def initial_search_timed(self,max_iterations):
        combined_candidates=np.empty((0,4))
        combined_occurrences = []
        for i in range(max_iterations):
            frame = self.get_feed()[0]
            # cv2.imshow('Classifier',frame)
            # key = cv2.waitKey(0) & 0xFF
            (candidates, occurrences) = self.initial_search(frame)
            if (len(occurrences)==0):
                continue
            combined_candidates = np.append(combined_candidates,candidates,0)
            combined_occurrences=combined_occurrences + occurrences #concatenate the lists
            if (np.max(occurrences)>250): #if finds a candidate of sufficiently high score [?], end search
                break

        print "=====Initial Search Complete"
        if (len(combined_occurrences)==0):
            print "No matches found."
            return np.empty((0,4))
        # print combined_candidates, combined_occurrences
        combined_occurrences = self.scoring_combine(combined_candidates,combined_occurrences)
        (combined_candidates, combined_occurrences) = self.candidate_sort_filter(combined_candidates,combined_occurrences)
        # frame = self.get_feed()[0]
        frame_disp = copy(frame)
        combined_candidates = np.int32(combined_candidates)
        # print combined_occurrences
        # for x1, y1, x2, y2 in combined_candidates:
        #     cv2.rectangle(frame, (x1, y1), (x1+x2, y1+y2), (255, 0, 0), 2)
        (x1,y1,x2,y2)=combined_candidates[np.argmax(combined_occurrences)]
        cv2.rectangle(frame_disp, (x1, y1), (x1+x2, y1+y2), (0, 255, 0), 2) #assuming object hasn't moved significantly
        
        # cv2.imshow('Classifier',frame_disp)
        # key = cv2.waitKey(1) & 0xFF
        # cv2.destroyAllWindows()
        return (np.array([x1,y1,x2,y2])) #return best rectangle! 

    def seg_search(self,seg_rect): #given rect coord potentially containing object, returns rect around object if found
        frame = self.get_feed()[0]
        x1,y1,x2,y2 = seg_rect
        seg = frame[y1:(y1+y2),x1:(x1+x2)]
        # cv2.imshow("blah",seg)
        # cv2.waitKey(10)
        (candidates,candidates_counts) = self.initial_search(seg)  #checks segment of the original image within the rect
        if (len(candidates_counts)>0):
            rect = candidates[np.argmax(candidates_counts)] #takes the most likely candidate
            rect[0] = rect[0]+x1    #corrects the coordinates to correspond to original image
            rect[1] = rect[1]+y1
            return rect
        else:

            print "seg_search: Object lost"
            return np.empty((0,4))

    def get_feed(self):
        # data = rospy.wait_for_message("/camera/rgb/image_raw", Image, timeout=None)
        self.subscriber.update_feed()
        rgb_data = self.subscriber.rgb_img
        rgb_frame = self.cvbridge.imgmsg_to_cv2(rgb_data, "bgr8")
        rgb_frame = np.array(rgb_frame, dtype=np.uint8)

        depth_data = self.subscriber.depth_img
        depth_frame = self.cvbridge.imgmsg_to_cv2(depth_data)
        depth_frame = np.array(depth_frame, dtype=np.float32)
        return rgb_frame, depth_frame

    def initial_search(self,original_image):
        #print "Searching for object; don't move." Display image being processed
        #read image dimensions
        #pass whole image to detect, guessed size=50px, nearest_neighbors=1.
        #pass initial_candidate sections to detect, nearest_neighbors=0
        #convert these second-round results to np matrix, each row containing rectangle center and size (in float). 
        #kmeans cluster the matrix; combine similar results (average and combine if within... 10% of total image dimension... from each other); if one contains significant majority (...>75% of total), add to list of serious_candidates along with # of members
        #repeat: perform initial detect for increasing guessed sizes (all resized to ...120px...or 50px?)
        #take all serious candidates, combine like ones, pass to depth filter, seek cluster of similar values.
        #(If desired) take histogram of these pixels' colors, see if it's on par with histogram of trained data.
        
        # cv2.imshow('Classifier',original_image)
        shape=self.cv_size(original_image)
        candidates=np.empty((0,4),int)
        candidates_counts=[]
        candidates_total_counts=[]
        for i in shape:
            if i<=50:
                print "The image is too small."
                return ([],[])
        for guess in range(50,min(shape),40):
            # print "Now guessing: ", guess
            rects1=self.detect(original_image,guess-30,guess+30,neighbors=1,initial=True)
            if len(rects1)==0:
                continue
            for x1, y1, x2, y2 in rects1:
                x2_=x1+x2
                y2_=y1+y2

                segment=original_image[y1:y2_,x1:x2_]

                #if the image doesn't pass the color histogram test, don't consider it
                if not self.hist_test.compare(segment):
                    continue

                segment_scaled=cv2.resize(segment,(140,140), interpolation = cv2.INTER_AREA) #[?]
                segment_scaled = cv2.GaussianBlur(segment_scaled,(5,5),0)
                dim_scaled=self.cv_size(segment_scaled)[0]
                (candidate_rect,count,total_count)=self.detect(segment_scaled,int(dim_scaled*0.5),dim_scaled,neighbors=0,initial=False) #check sections of interest of guessed size
                if candidate_rect.size>0:
                    dim=self.cv_size(segment)[0]
                    scale=np.float(dim)/dim_scaled
                    # print "Located at",x1,y1
                    candidate_rect[0]=np.int(candidate_rect[0]*scale)+x1
                    candidate_rect[1]=np.int(candidate_rect[1]*scale)+y1
                    candidate_rect[2]=np.int(candidate_rect[2]*scale)
                    candidate_rect[3]=np.int(candidate_rect[3]*scale)
                    candidates=np.append(candidates,[candidate_rect],0)
                    candidates_counts.append(count)
                    candidates_total_counts.append(total_count)
  
        if (len(candidates) > 0):
            # Sort data by candidates_total_counts confidence level
            # (candidates,candidates_counts,candidates_total_counts) = candidate_sort_filter(candidates,candidates_counts,candidates_total=candidates_total_counts)
            for x1, y1, x2, y2 in candidates:
                cv2.rectangle(original_image, (x1, y1), (x1+x2, y1+y2), (255, 0, 0), 2)
            (x1,y1,x2,y2)=candidates[np.argmax(candidates_counts)]
            cv2.rectangle(original_image, (x1, y1), (x1+x2, y1+y2), (0, 255, 0), 2)
            # cv2.rectangle(input_image, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (255, 0, 0), 2)
            # cv2.imshow('Classifier',original_image)
            # key = cv2.waitKey(50) & 0xFF
        
            #return only candidates within top 25% score of this size class [?]
            (candidates,candidates_counts) = self.candidate_sort_filter(candidates,candidates_counts)
        return (candidates,candidates_counts) #(candidates[-record:],candidates_counts[-record:])

    def combine(self,labels,centers,max_label,max_center):
        #convert to width and centroid values
        tolerance=np.mean(centers[:,2])*0.5 #tolerance set to ...25%?... of mean width [?]
        centroids=np.column_stack((centers[:,0]+(centers[:,1])*0.5,centers[:,2]+(centers[:,3])*0.5)) #nx2 list of centroid positions x and y
        # combined_centers=np.empty((0,4))
        distances=np.sqrt(((centroids[:,0]-centroids[max_label,0])**2)+((centroids[:,1]-centroids[max_label,1])**2))
        ind=np.nonzero(distances<tolerance)[0] #at the very least will be populated by index max_label
            # combined_centers=np.append(combined_centers,[mean_center],0)
        for j in ind:
            labels[labels==j] = max_label
        return labels
        # centroids=np.column_stack((centers[:,0]+(centers[:,1])*0.5,centers[:,2]+(centers[:,3])*0.5))

    def detect(self,input_image,min_size,max_size,neighbors=0,initial=False):
        #read image dimensions
        #run classifier, neighbors=1, minimum rectangle=size px, maximum rectangle=size+20px
        #kmeans cluster, find representative rectangle... that has more than 1/k members
        #return position in image

        # shape=self.cv_size(input_image)
        rects = self.cascade.detectMultiScale(input_image, scaleFactor=1.05,minNeighbors=neighbors, minSize=(min_size, min_size), maxSize=(max_size, max_size))

        if initial:
            return rects
        else:
            if len(rects)>50:
                # Apply KMeans
                rects=np.float32(rects)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
                flags = cv2.KMEANS_RANDOM_CENTERS
                k=10 #[?]
                compactness,labels,centers = cv2.kmeans(rects,k,criteria,10,flags)
                (index,count)=self.get_max_occurrence(labels)
                labels=self.combine(labels,centers,index,centers[index]) #combine centers that are close to the optimal center at index
                count=np.sum(labels==index) #updates count value to include newly combined centers
                
                # (x1,y1,x2,y2)=centers[index]
                # cv2.rectangle(input_image, (x1, y1), (x1+x2, y1+y2), (255, 0, 0), 2)
                # temp=copy(input_image)
                # if (len(rects) > 0): #not(len(rects) == 1)
                #     for x1, y1, x2, y2 in rects:
                #         cv2.rectangle(temp, (x1, y1), (x1+x2, y1+y2), (255, 0, 0), 2)
                # cv2.namedWindow("Detect", cv2.WINDOW_NORMAL)
                # cv2.imshow('Detect',temp)
                # key = cv2.waitKey(0) & 0xFF

                if (count>=(0.55*len(labels))): #[?]
                    # print "Total Rects: ",len(rects)
                    # print "Qualified Rects: ",count
                    # print (np.int64(centers[index]))
                    # print "Candidate found!\n"

                    return (np.int64(centers[index]),count,len(rects))
            return (np.empty((0,4)),[],[])

    def get_max_occurrence(self,array):
        #takes numpy array, finds unique value that occurs the most often, along with # of occurrences
        unique_values=np.unique(array)
        count=[]
        #if I could reliably use np.unique's "return_counts" option, that would be nice. It's only available to v.1.90+...
        for value in unique_values:
            count.append(sum(array==value))
        return (unique_values[np.argmax(count)],max(count))

    def candidate_sort_filter(self,candidates,candidates_counts): #,candidates_total=[]
        #takes list of candidate rectangles and their scores, sorts according to score, and removes candidates below %75 of max score [?]
        # if len(candidates_total)>0:
        #     sorting_order = np.argsort(candidates_total)
        #     candidates = candidates[sorting_order]
        #     candidates_total = np.asarray(candidates_total)[sorting_order].tolist()
        #     candidates_counts = np.asarray(candidates_counts)[sorting_order].tolist()
        #     return (candidates,candidates_counts,candidates_total)
        # else:
        sorting_order = np.argsort(candidates_counts)
        sorting_order=np.fliplr([sorting_order])[0] #order is max to min
        candidates = candidates[sorting_order]
        candidates_counts = np.asarray(candidates_counts)[sorting_order].tolist()
        # print candidates
        for i in range(len(candidates_counts)):
            if candidates_counts[i] < (0.75*candidates_counts[0]):
                # print "Candidates", candidates[:i,:], candidates_counts[:i]
                return (candidates[:i,:], candidates_counts[:i])
        return (candidates, candidates_counts)

        

    def scoring_combine(self,rects, scores):
        new_rects = np.empty((0,4))
        new_scores = []
        centroids=np.column_stack((rects[:,0]+(rects[:,2])*0.5,rects[:,1]+(rects[:,3])*0.5)) #nx2 list of centroid positions x and y
        for i in range(len(scores)):
            tolerance=rects[i,2]*0.25 #tolerance set to ...50%?... of width [?]
            distances=np.sqrt(((centroids[:,0]-centroids[i,0])**2)+((centroids[:,1]-centroids[i,1])**2))
            ind=np.nonzero((distances<tolerance) & (np.abs(rects[:,2]-rects[i,2])<tolerance))[0]
            sum_scores = scores[i]
            for j in range(len(ind)):
                sum_scores = sum_scores + (0.25*scores[j])     #scores are weighted such that the original has weight 1.5 and additional scores have weight 0.5
                # mean_rects += rects[j]
            # mean_rects = np.int32(mean_rects/len(ind))
            new_scores.append(sum_scores) #concatenate
            # new_rects=np.append(new_rects,mean_rects,0)
        return np.int32(new_scores).tolist() #new_rects,


def cleanup():
    print "Have a nice day."
    cv2.destroyAllWindows()

def demo():
    rospy.init_node("obj_classifier")
    rospy.on_shutdown(cleanup)
    # cv2.startWindowThread()
    # cascade = cv2.CascadeClassifier("/home/hongalan/skelws/src/opencv-haar-classifier-training/trained_classifiers/maruchan_asus15.xml")
    # ref_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/maruchan_train2.avi_9765_0000_0266_0148_0143_0148.png"
    if len(sys.argv)>2:
        ref_img = cv2.imread(sys.argv[1],1)
        cascade = cv2.CascadeClassifier(sys.argv[2])
    else:
        print "obj_dregistered node requires 2 arguments: ref image and classifier"
        return
    if cascade.empty():
        print "Classifier not found"
        return

    print "Let's get started!"
    cvbridge = CvBridge()
    classifier=Classifier(cascade,cvbridge,ref_img)

    for i in range(5):
        classifier.initial_search_timed(5)
    



if __name__=='__main__':
    try:
        demo()
    except rospy.ROSInterruptException:
        cleanup()