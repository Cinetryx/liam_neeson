#!/usr/bin/env python

import sys
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from math import sqrt


class HistogramTest:
    def __init__(self,ref_image):
        self.km = 2
        self.ref_image = ref_image
        self.ref_hsv = self.prep_image(self.ref_image)
        self.ref_color_content = float(np.size(self.ref_hsv))/np.size(ref_image)
        (self.ref_centers,self.ref_percentage) = self.histogram_clusters(self.ref_hsv)
         #[?]
        # print "======================", self.km
    def train_reference(self,images_list):
        # NOT YET SUCCESSFULLY IMPLEMENTED. Need to account for HSV wrapping and find a way to average percentages such that they add up to 1 in the end...

        print "Original REF color content: ",self.ref_color_content
        print "Original REF centers: ", self.ref_centers
        print "Original REF percentages: ", self.ref_percentage, self.ref_percentage.__class__
        avg_color_content = 0
        avg_centers = np.zeros((self.km,3))
        avg_percentage = np.zeros(self.km)
        for img in images_list:
            cv2.imshow("REF TRAINING",img)
            key = cv2.waitKey(0) & 0xFF
            hsv = self.prep_image(img)
            avg_color_content += float(np.size(hsv))/np.size(img)
            centers,percentage = self.histogram_clusters(hsv)
            ind = np.zeros(self.km)
            for i in range(self.km):
                #find the nearest center
                dist = np.zeros((self.km,1))
                for j in range(self.km):
                    dist[j]=sqrt(((self.ref_centers[i,0]-centers[j,0])**2) + ((self.ref_centers[i,1]-centers[j,1])**2) + ((self.ref_centers[i,2]-centers[j,2])**2))
                ind[i] = np.argmin(dist)
            ind = np.int32(ind)
            print ind
            centers = centers[ind]
            percentage = np.asarray(percentage)[ind]
            avg_centers += centers
            avg_percentage += percentage
        avg_color_content /= len(images_list)
        avg_centers /= len(images_list)
        avg_percentage /= len(images_list)
        self.ref_color_content = avg_color_content
        self.ref_centers = avg_centers
        self.ref_percentage = avg_percentage.tolist()
        print "New REF color content: ",self.ref_color_content
        print "New REF centers: ", self.ref_centers
        print "New REF percentages: ", self.ref_percentage

        bar = np.zeros((50, 300, 3), dtype = "uint8")
        startX = 0
        print "Percentage: ", percentage
        print "HSV: ",centers
        for (percent, color) in zip(self.ref_percentage, self.ref_centers):
            # plot the relative percentage of each cluster
            endX = startX + (percent * 300)
            color = [color[0],255,255] #just show the hue
            color = np.uint8([[color]])
            color = cv2.cvtColor(color, cv2.COLOR_HSV2BGR)
            color = color[0,0,:]
            cv2.rectangle(bar, (int(startX), 0), (int(endX), 50),
                color.astype("uint8").tolist(), -1)
            startX = endX
        cv2.imshow("REF TRAINING",bar)
        key = cv2.waitKey(0) & 0xFF



    def test_color_content(self,rgb_image):
        hsv = self.prep_image(rgb_image)
        #test if color content distribution is within 50% that of reference image [?]
        # print "Color Content: ", float(np.size(hsv))/np.size(rgb_image)
        # print "Ref Color Content: ",self.ref_color_content
        if ((float(np.size(hsv))/np.size(rgb_image) < 1.5*(self.ref_color_content)) & (float(np.size(hsv))/np.size(rgb_image) > 0.5*(self.ref_color_content))):
            return hsv
        else:
            return np.empty((0,3))

    def prep_image(self,rgb_image):
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        hsv = hsv.reshape((hsv.shape[0] * hsv.shape[1], 3)) # reshape the image to be a list of pixels
        
        #remove black and white pixels
        ind = np.nonzero(np.invert((hsv[:,2]<(0.5*255)) | ((hsv[:,2]>(0.5*255)) & (hsv[:,1]<(0.20*255))) ))[0] #tuned for vivid colors [?]
        hsv = hsv[ind]
        return hsv

    def histogram_clusters(self,hsv):
        #Given rgb image, takes kmeans clusters of color distribution according to hsv colorspace
        
        # Apply KMeans
        hsv=np.float32(hsv)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        flags = cv2.KMEANS_RANDOM_CENTERS
        compactness,labels,centers = cv2.kmeans(hsv,self.km,criteria,10,flags)
        percentage = []
        for i in range(0,self.km):
            percentage.append(float(sum(labels==i))/len(labels))
        # print "Histogram percentage distribution: ", percentage
        return centers,percentage

    def histogram_compare(self,centers1,percentage1, centers2, percentage2):
        #given clusters of kx3 hsv values and their respective percentage distributions, test if they match within a certain tolerance
        #innocent until proven guilty
        for i in range(0,self.km):
            #find the nearest center
            dist = np.zeros(self.km)
            for j in range(self.km):
                dist[j]=sqrt(((centers1[i,0]-centers2[j,0])**2) + ((centers1[i,1]-centers2[j,1])**2) + ((centers1[i,2]-centers2[j,2])**2))
            ind = np.argmin(dist)
            #test hue similarity
            if (abs(centers1[i,0]-centers2[ind,0]) < 45) | (abs(centers1[i,0]-centers2[ind,0]) > 90): #hue ranges from 0 to 180. If difference >120, they are still within 60 of each other as the value wraps around [?]
                #test percentage distribution
                if abs(percentage1[i] - percentage2[ind]) < 0.5: #[?]
                    continue
            return False
        return True

    def plot_colors(self,img):

        hsv = self.prep_image(img)
        (centers,percentage) = self.histogram_clusters(hsv)

        # initialize the bar chart representing the relative frequency
        # of each of the colors
        bar = np.zeros((50, 300, 3), dtype = "uint8")
        startX = 0
        # loop over the percentage of each cluster and the color of
        # each cluster
        # print "Percentage: ", percentage
        # print "HSV: ",centers
        for (percent, color) in zip(percentage, centers):
            # plot the relative percentage of each cluster
            endX = startX + (percent * 300)
            color = [color[0],255,255] #just show the hue
            color = np.uint8([[color]])
            color = cv2.cvtColor(color, cv2.COLOR_HSV2BGR)
            color = color[0,0,:]
            cv2.rectangle(bar, (int(startX), 0), (int(endX), 50),
                color.astype("uint8").tolist(), -1)
            startX = endX
        
        # return the bar chart
        return bar

    def demo_bar(self,window_name,file_name):
        img = cv2.imread(file_name,1)
        cv2.imshow(file_name,img)
        key = cv2.waitKey(0) & 0xFF
        
        bar=self.plot_colors(img)
        cv2.imshow(window_name,bar)
        key = cv2.waitKey(0) & 0xFF

    def compare(self,rgb_image):
        #given rgb image, compares it to reference image, returns True if passes color content test and hue distribution test
        hsv=self.test_color_content(rgb_image)
        if not hsv.any():
            return False #hsv will be empty if the color content percentage varies drastically from that of reference image
        (centers,percentage) = self.histogram_clusters(hsv)
        return self.histogram_compare(self.ref_centers,self.ref_percentage,centers,percentage)

def demo():
    ref1_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/maruchan_train2.avi_9765_0000_0266_0148_0143_0148.png"
    ref2_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/test_positive2.jpg"
    ref3_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/maruchan_train.avi_4367_0354_0299_0224_0028_0033.png"
    ref4_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/20160223_183351.jpg"
    ref5_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/20160223_183459.mp4_2501_0090_0038_0107_0093_0095.png"
    
    test1_filename = "/home/hongalan/skelws/src/liam_neeson/src/test_hsv.jpg"
    test2_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/maruchan_train2.avi_16673_0000_0209_0133_0217_0222.png"
    test3_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/maruchan_train2.avi_8782_0356_0257_0150_0113_0118.png"
    # test4_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/maruchan_train.avi_4759_0001_0310_0258_0028_0033.png"
    test4_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/negative_images/94b35a728230533d0d859a81dc4dc025.jpg"
    test5_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/negative_images/20160210_155130.jpg"
    
    # print calc_hist("REF",ref_filename)
    ref_img1 = cv2.imread(ref1_filename,1)
    ref_img2 = cv2.imread(ref2_filename,1)
    ref_img3 = cv2.imread(ref3_filename,1)
    ref_img4 = cv2.imread(ref4_filename,1)
    ref_img5 = cv2.imread(ref5_filename,1)

    ht = HistogramTest(ref_img1)
    # ht.train_reference([ref_img1,ref_img2,ref_img3,ref_img4,ref_img5])

    ht.demo_bar("REF",ref1_filename)
    ht.demo_bar("IMG1",test1_filename)
    ht.demo_bar("IMG2",test2_filename)
    ht.demo_bar("IMG3",test3_filename)
    ht.demo_bar("IMG4",test4_filename)
    ht.demo_bar("IMG5",test5_filename)

    img1 = cv2.imread(test1_filename,1)
    img2 = cv2.imread(test2_filename,1)
    img3 = cv2.imread(test3_filename,1)
    img4 = cv2.imread(test1_filename,1)
    neg_img = cv2.imread(test5_filename,1)

    (centers1,percentage1) = ht.histogram_clusters(ht.prep_image(img1))
    (centers2,percentage2) = ht.histogram_clusters(ht.prep_image(img2))
    (centers3,percentage3) = ht.histogram_clusters(ht.prep_image(img3))
    (centers4,percentage4) = ht.histogram_clusters(ht.prep_image(img4))
    (centers5,percentage5) = ht.histogram_clusters(ht.prep_image(neg_img))

    result1_2 = ht.histogram_compare(centers1,percentage1, centers2,percentage2)
    result1_3 = ht.histogram_compare(centers1,percentage1, centers3,percentage3)
    result1_4 = ht.histogram_compare(centers1,percentage1, centers4,percentage4)

    result2_3 = ht.histogram_compare(centers2,percentage2, centers3,percentage3)
    result2_4 = ht.histogram_compare(centers2,percentage2, centers4,percentage4)

    result3_4 = ht.histogram_compare(centers3,percentage3, centers4,percentage4)

    result1_5 = ht.histogram_compare(centers1,percentage1, centers5,percentage5)    
    # print "Match Results 1: ", result1_2, result1_3, result1_4
    # print "Match Results 2: ", result2_3, result2_4
    # print "Match Results 3: ", result3_4

    # print "Negative Result: ", result1_5

    
    # print "With color content comparison... "
    # print "Match Result 1: ", ht.compare(img1)
    # print "Match Result 2: ", ht.compare(img2)
    # print "Match Result 3: ", ht.compare(img3)
    # print "Match Result 4: ", ht.compare(img4)

    # print "Negative Result: ", ht.compare(neg_img)


    key = cv2.waitKey(0) & 0xFF
def main():
    demo()
    # ref_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/maruchan_train2.avi_9765_0000_0266_0148_0143_0148.png"
    # test1_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/maruchan_training_data/positive_images/maruchan_train2.avi_16673_0000_0209_0133_0217_0222.png"
    # test2_filename = "/home/hongalan/skelws/src/opencv-haar-classifier-training/negative_images/20160210_155130.jpg"
    # ref_img = cv2.imread(ref_filename,1)
    # ht = HistogramTest(ref_img)
    # result1 = ht.compare(test2_img)

    # cv2.imshow(ref_filename,ref_img)
    # key = cv2.waitKey(0) & 0xFF
    # hsv = ht.prep_image(ref_img)
    # (centers,percentage) = self.histogram_clusters(hsv)
    # bar=self.plot_colors(centers,percentage)
    # cv2.imshow(window_name,bar)
    # key = cv2.waitKey(0) & 0xFF
    # test1_img = cv2.imread(test1_filename,1)
    # test2_img = cv2.imread(test2_filename,1)
    # ht = HistogramTest(ref_img)
    # result1 = ht.compare(test2_img)

    # print result1


if __name__=='__main__':
    main()