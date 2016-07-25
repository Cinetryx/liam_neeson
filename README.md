# Object Tracking with an RGBD Camera
#### *liam_neeson ROS package README*

Owner: Alan Hong, Northwestern University, MSR 2016

Project overview available here: [ *site currently in development* ]

~~~
Table of Contents
1. Package Description
2. ROS Package Dependencies
3. Launch Files
4. Notes on Package Usage
5. Ongoing Development
~~~

### 1. Package Description

The liam_neeson package combines an [ASUS Xtion PRO LIVE](https://www.asus.com/3D-Sensor/Xtion_PRO/) camera (or equivalent, e.g. Kinect) and [Baxter robot](http://www.rethinkrobotics.com/baxter/) to visually track and interact with a pre-determined object. The demo implemented commands the robot to continually point at the object as the user moves the object around in the camera's field of view.

A trained Haar classifier, in conjunction with time and color filters, is used to locate the object in the camera's RGB image feed, which is combined with a rectified depth image to determine the object's real-world coordinates. These coordinates are then published through the `/obj_position` topic. 

The coordinates, once recieved by the robot controller, are adjusted to be expressed from the camera frame to the robot frame. The robot may then use these adjusted coordinates to perform tasks.

This package includes the following major components:
- Script that performs robust object detection and segmentation, and extracts the object's real-world coordinates.
- Demo that points the robot's end-effector towards the object at regular intervals.
- Launch file to set up the environment and run all of the above. 

### 2. ROS Package Dependencies

In order to successfully run the launch files, you must have the following sets of packages available to the workspace:

* **[Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Baxter_Setup)** - Contains necessary robot interface packages such as baxter_moveit_config, baxter_interface, baxter_examples, etc.
* **OpenNI** - Contains RGBD camera interface packages


### 3. Launch Files

#### obj_pointer.launch

- The overarching launch file of the project. Calls the necessary robot and camera launch files, runs the object classifier and robot control scripts.

#### baxter_setup.launch

- Launches nodes relevant to Baxter and its interface


### 4. Notes on Package Usage

- openni.launch will start multiple nodes under the `/camera` namespace
  


### 5. Ongoing Development

*as of March 19, 2016*

-Implementing object tracking, to much more quickly determine position of object once its initial position is determined.

-Velocity control of robot for more continuous operation

-Interactive user interface (including voice commands)

-Include set of trained classifiers and reference images for variety of objects

-Determine camera position and orientation relative to robot through transforms to a common object.