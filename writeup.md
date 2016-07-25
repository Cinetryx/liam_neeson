# Alan Hong - Winter Project 2016

~~~
Table of Contents
1. Goal and Motivation
2. Project Overview
3. Progress History
4. Lessons and Challenges

~~~


### **1. Goal and Motivation:**  

Computer vision is an essential tool in robotics. Whenever a robot plans to interact with an object or its environment, it must first obtain an understanding of it to determine how to best interact with it. Cameras are relatively inexpensive and versatile sensors and are often used in such situations. 

The goal of this project is to visually determine the real-world coordinates of a known object and use this information to interact with the object using a robot. This is accomplished using RGBD data from an ASUS Xtion PRO LIVE camera and the Baxter robot.


### **2. Project Overview:**  

The object is first detected in the RGB image by means of a Haar classifier, which has been trained using a set of images containing the object. The classifier proceeds to search the image for features it has determined to most likely represent the object. Once the classifier returns a region of the image which is suffifiently likely to contain the object, the script then determines the distance to the object in that region using the corresponding depth map (where each pixel's value represents the distance from the camera) supplied by the ASUS Xtion PRO LIVE camera. Using this depth information and the pixel coordinates of the object's center of mass, the real-world coordinates of the object relative to the camera can be calculated.

A video demonstration of the project as of April 2016 [can be seen here](https://www.youtube.com/watch?v=c6ZZvMBOk0I). In this demonstration, the Baxter is commanded to point its end-effector at the object as the script uses the RGBD data to determine its position. This is performed by first calculating the transform of the ASUS Xtion PRO LIVE camera's frame relative to that of the robot through the Baxter's built-in camera and multiple camera calibration. Then, when the object's position relative to the Asus Xtion PRO LIVE is determined, the object's position relative to the robot can likewise be determined. The robot is then commanded to point at the object's position by calculating the joint positions that will accomplish this and actuating each joint at corresponding velocities.


### **3. Progress History:**

- The RGB and depth images supplied by the camera were rectified to align the pixels.

- As a proof of concept, the pixel coordinates of a red ball were found using Hough circles and color. The real-world coordinates were then extracted using an approximated region in the depth image.

- The red ball was located using a Haar classifier. Implemented through OpenCV, the classifier returns a set of rectangular areas that may contain the trained object. The Haar classifier was chosen for its relatively fast operation, though it requires a lengthy training process in advance.

- The classifier was trained to locate a more complex object: a cup of instant noodles.

- The robustness of the classifier was improved by training it with images of the object under various conditions (such as lighting, orientation, and camera resolution). By doing so, the classifier searches for features that are held common throughout these conditions.

- Classifier parameters such as the range of permissible rectangle dimensions were tuned to allow for a wider margin of error, but resulted in more false positives.

- False positives were removed by assigning a score to each positive. This score is assigned using the number of positives located in the same vicinity, which was determined using k-means clustering. Further false positives were removed by filtering the ROI across time and testing color composition against a reference image.

- The RGB and depth images were aligned in time so that the depth data retrieved corresponds to the object found in the RGB image, even if the object had moved in the interim between the two processes.

- Noise in the depth data was accounted for through the binarization of the depth image to isolate the pixels that represent the object; the representative depth of the object was then calculated as the mean of depth values at each pixel location.

- Passed the real-world object coordinates to Baxter, transforming between camera and planning frames.

- Several methods for instructing Baxter to point to the object were attempted. For the time being, the arm is essentially reduced to two joints, allowing the robot to more quickly track the object.

- The real-world coordinates of a trained object can be determined at nearly 2 Hz and Baxter is able to point its end effector towards the object at intervals of 1-2 seconds. [**See video here**](https://youtu.be/LnhNJadnGo0).

- Once the position of the object is determined, the subsequent search for the object is restricted to the immediate vicinity as opposed to the entire image, reducing the search time.

- Control of the robot arm is perfomred using the robot's Jacobian and error optimization to determine the desired joint velocities.

- An expanded set of trained classifiers and reference images is provided for a variety of objects.

- The RGBD camera's position and orientation relative to robot's camera is determined by the calculated transforms to a QR code present in both cameras' fields of view.

- A reference image can be supplied to the script to specify the color distribution of the object. This allows object tracking to be more robust to varied lighting and the camera's exposure setting.


### **4. Lessons and Challenges:**  

* It was found that, while the classifier can be made more accurate with a large training set, this slows the classfier at runtime. This may be the case because a larger training set provides more criteria that must be checked when detecting an object. A balance was reached that returned reasonably accurate results.

* The robustness of the classifier was seen to be dependent on the diversity of images chosen for training the classifer. For instance, the camera resolution, object proximity, lighting, and orientation could be varied. However, varying these qualities too much was seen to reduce accuracy. For instance, the classifier trained on images of the object from multiple perspectives resulted in lower accuracy than that which was trained on as many images from a single perspective. I suspect similar accuracy may be accomplished with a sufficiently large dataset, but this slows the performance.

* It was seen that using a single method of classification is not as reliable as combining several methods. For instance, the Haar classifier can be very accurate in a given setting provided the parameters are tuned accordingly. However, the same parameters would fail to find the object in a different setting. To achieve robust performance across settings, the parameters were chosen to allow for a wider margin of error, producing more false positives. These false positives were addressed by comparing these results to results from the past and by comparing the color composition to a reference image.

* The primary limiting factor of the object location process was seen to be the classifier. In order to reduce the number of times that it must run, one can reduce the number of false positives early on and obviate the need to run classifier. One can also pass only a portion of the image to the classifier; this can be done anticipating the location of the object from its past locations and estimated trajectory and assigning a corresponding ROI.

* The speed of Baxter's performance is seen to suffer for relying on a path-planner to calculate the trajectory of the end-effector. It is concluded that the path-planner is not needed and can be replaced by working with the robot's Jacobian in joint-velocity space.

