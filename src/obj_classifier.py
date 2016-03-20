#!/usr/bin/env python

import cv2
import numpy as np
import sys
from copy import copy

class Classifier:
    def __init__(self,cascade):
        self.cascade=cascade

    def cv_size(self,img):
        return tuple(img.shape[1::-1])
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
        print "Searching for object. Don't move!"
        cv2.imshow('Classifier',original_image)
        shape=self.cv_size(original_image)
        candidates=np.empty((0,4),int)
        candidates_counts=[]
        candidates_total_counts=[]
        for i in shape:
            if i<=50:
                print "The image is too small."
                return
        for guess in range(50,min(shape),40):
            print "Now guessing: ", guess
            rects1=self.detect(original_image,guess-30,guess+30,neighbors=1,initial=True)
            
            # temp=copy(original_image)
            # for x1, y1, x2, y2 in rects1:
            #     x2_=x1+x2
            #     y2_=y1+y2
            #     cv2.rectangle(temp, (x1, y1), (x2_, y2_), (255, 0, 0), 2)
            # cv2.imshow('Classifier',temp)
            # key = cv2.waitKey(0) & 0xFF

            for x1, y1, x2, y2 in rects1:
                x2_=x1+x2
                y2_=y1+y2

                # segment_image = copy(original_image)
                # cv2.rectangle(segment_image, (x1, y1), (x2_, y2_), (255, 0, 0), 2)
                # cv2.imshow('Classifier',segment_image)
                # key = cv2.waitKey(100) & 0xFF

                # print x1,x2_,y1,y2_
                segment=original_image[y1:y2_,x1:x2_]

                if not segment.size>4:
                    continue
                segment_scaled=cv2.resize(segment,(140,140), interpolation = cv2.INTER_AREA) #[?]
                segment_scaled = cv2.GaussianBlur(segment_scaled,(5,5),0)
                dim_scaled=self.cv_size(segment_scaled)[0]
                (candidate_rect,count,total_count)=self.detect(segment_scaled,int(dim_scaled*0.5),dim_scaled,neighbors=0,initial=False) #check sections of interest of guessed size
                if candidate_rect.size>0:
                    dim=self.cv_size(segment)[0]
                    scale=np.float(dim)/dim_scaled
                    print "Located at",x1,y1
                    candidate_rect[0]=np.int(candidate_rect[0]*scale)+x1
                    candidate_rect[1]=np.int(candidate_rect[1]*scale)+y1
                    candidate_rect[2]=np.int(candidate_rect[2]*scale)
                    candidate_rect[3]=np.int(candidate_rect[3]*scale)
                    candidates=np.append(candidates,[candidate_rect],0)
                    candidates_counts.append(count)
                    candidates_total_counts.append(total_count)

        for i in range(len(candidates)):
            print candidates[i], candidates_counts[i], candidates_total_counts[i]
        if (len(candidates) > 0):
            print "Original: "
            copy_candidates_counts = copy(candidates_counts)
            print candidates, copy_candidates_counts
            candidates_counts = self.scoring_combine(candidates,candidates_counts) #previous version took mean of combined rects
            print "Combined: "
            print np.asarray(candidates_counts)-np.asarray(copy_candidates_counts)
            candidates = np.int32(candidates)
            for x1, y1, x2, y2 in candidates:
                cv2.rectangle(original_image, (x1, y1), (x1+x2, y1+y2), (255, 0, 0), 2)
            (x1,y1,x2,y2)=candidates[np.argmax(candidates_counts)]
            cv2.rectangle(original_image, (x1, y1), (x1+x2, y1+y2), (0, 255, 0), 2)
        # cv2.rectangle(input_image, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (255, 0, 0), 2)
        cv2.imshow('Classifier',original_image)
        key = cv2.waitKey(0) & 0xFF

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
            new_scores.append(sum_scores)
            # new_rects=np.append(new_rects,mean_rects,0)
        return new_scores #new_rects,

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
                    print "Total Rects: ",len(rects)
                    print "Qualified Rects: ",count
                    print (np.int64(centers[index]))
                    print "Candidate found!\n"

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




def cleanup():
    print "Have a nice day."
    cv2.destroyAllWindows()

def main():
    # rospy.init_node(node_name)
    # rospy.on_shutdown(cleanup)
    # cv2.startWindowThread()
    cascade = cv2.CascadeClassifier("/home/hongalan/skelws/src/opencv-haar-classifier-training/trained_classifiers/maruchan.xml")
    filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/Positiv/20160225_165224.jpg"
    if len(sys.argv)>1:
        filename = sys.argv[1]
        cascade = cv2.CascadeClassifier(sys.argv[2])
    win1_name='Feed'
    win2_name='Filtered'
    img = cv2.imread(filename,1)
    if not cascade.empty():
        print "Let's get started!"

        cv2.imshow('Classifier',img)
        key = cv2.waitKey(0) & 0xFF

        classifier=Classifier(cascade)

        classifier.initial_search(img)
    else:
        print "Classifier not found"


    # while True:
    #     input_img = copy(img)
    #     input_img = res = cv2.resize(input_img,None,fx=n, fy=n, interpolation = cv2.INTER_AREA)
        
    #         # cv2.CASCADE_SCALE_IMAGE)
        
    #     if not(len(rects) == 1):
    #         for x1, y1, x2, y2 in rects:
    #             cv2.rectangle(input_img, (x1, y1), (x1+x2, y1+y2), (255, 0, 0), 2)

    #     print "Hello"

        # cv2.imshow('Classifier',input_img)
        # key = cv2.waitKey(0) & 0xFF
        # if key == 27:         # wait for ESC key to exit
        #     break


if __name__=='__main__':
    # try:
    main()
    # except rospy.ROSInterruptException:
    #     pass