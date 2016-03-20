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


replace grippers
    baxter_description/urdf/

obj_pointer.launch
    arguments:
    contents of launch file and purposes:
    relevant nodes:
        


    topics of interest:
        /obj_position


### 1. Package Description

The liam_neeson package combines an [ASUS Xtion Pro](https://www.asus.com/3D-Sensor/Xtion_PRO/) camera (or equivalent, e.g. Kinect) and [Baxter robot](http://www.rethinkrobotics.com/baxter/) to visually track and interact with a pre-determined object.

A trained Haar classifier, in conjunction with time and color filters, is used to locate the object in the camera's RGB image feed, which is combined with a rectified depth image to determine the object's real-world coordinates. These coordinates are then published through the `/obj_position` topic. 

The coordinates, once recieved by the robot controller, are adjusted to be expressed from the camera frame to the robot frame. The robot may then use these adjusted coordinates to perform tasks.

This package includes the following major components:
- Script that performs robust object detection and segmentation, and extracts the object's real-world coordinates.
- Demo that points the robot's end-effector towards the object at regular intervals.
- Launch file to set up the environment and run all of the above. 

### 2. ROS Package Dependencies

In order to successfully run the launch files, you must have the following sets of packages available to the workspace:

* **[Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Baxter_Setup)** - Contains necessary robot interface packages such as baxter_moveit_config, baxter_interface, baxter_examples, etc.
* **OpenNI SDK** - Contains RGBD camera interface packages


### 3. Launch Files

#### obj_pointer.launch

- The over-arching launch file of the project. Calls the necessary robot and camera launch files, runs the object classifier and robot control scripts.

- Arguments:

    `gazebo_sim` *(default="false")* Set to "true" if simulating robot.

    `gazebo_gui` *(default="false")* Set to "true" to view the virtual robot in Gazebo.

    `rviz` *(default="false")* Set to "true" to view the virtual robot in RViz.

    `update_gripper` *(default="true")* Flag to incorporate electric grippers in the URDF.

    `bag` *(default="false")* Set to "true" to play a rosbag file.

    `bag_file` *(default="depth_registered1.bag")* Filename of the desired rosbag file.

    `openni` *(default="false")* Set to "true" if RGBD camera is connected.

    `camera_trans` *(default="1. 0. 0. -0.5, 0.5, -0.5, 0.5")* Contains static transform from the robot to the camera. *By default the camera is set to 1 meter in front of the robot and oriented straight forward.*

    `classifier_xml1` *(default="maruchan_asus15.xml")* Filename of the trained classifier.

    `classifier_img1` *(default="maruchan_sample.png")* Filename of the object reference image.

- Contents:
    
    Publishes static transform from the robot to the camera.
    ~~~
    <node pkg="tf" type="static_transform_publisher" name="link1_broadcaster" args="$(arg camera_trans) base camera_link1 1000" />
    ~~~

    Starts up the simulated robot if desired
    ~~~
    <group if="$(arg gazebo_sim)">
        <include
            file="$(find liam_neeson)/launch/baxter_sim.launch">
            <arg name="gazebo_gui" value="$(arg gazebo_gui)" />
            <arg name="update_gripper" value="$(arg update_gripper)" />
        </include>
        <!-- Start remaining nodes manually once simulation is completely loaded -->
    </group>
    ~~~

    Sets up the Baxter robot interface
    ~~~
    <include
        file="$(find liam_neeson)/launch/baxter_setup.launch">
        <arg name="update_gripper" value="$(arg update_gripper)" />
    </include>
    ~~~

    Opens RViz configured to view the virtual robot
    ~~~
    <include if="$(arg rviz)"
        file="$(find liam_neeson)/launch/baxter_rviz.launch">
    </include>
    ~~~

    Runs the demo script for the robot pointer
    ~~~
    <node pkg="liam_neeson" 
        type="demo_obj_pointer.py" 
        output="screen" 
        name="demo_obj_pointer"/>
    ~~~

    Runs the object locator script
    ~~~
    <node pkg="liam_neeson"
        type="obj_dregistered.py"
        output="screen"
        name="obj_dregistered"
        args="$(find liam_neeson)/src/classifier_data/$(arg classifier_img1) $(find liam_neeson)/src/classifier_data/$(arg classifier_xml1)"/>
    </group>
    ~~~

    Sets up the RGBD camera
    ~~~
    <include if="$(arg openni)" 
        file="$(find openni_launch)/launch/openni.launch" >
        <arg name="depth_registration" value="true" />
    </include>
    ~~~

    Plays the bag file if desired
    ~~~
    <node if="$(arg bag)"
        pkg="rosbag" type="play" name="rosbag" args="-l $(find liam_neeson)/bag/$(arg bag_file)"/>
    ~~~

#### baxter_setup.launch

- Launches nodes relevant to Baxter and its interface

- Arguments:

    `update_gripper` *(default="true")* Flag to incorporate electric grippers in the URDF.


- Contents:

    Sets up the grippers
    ~~~
    <node if="$(arg update_gripper)"
        name="gripper_modifier"
        pkg="liam_neeson" 
        type="replace_grippers.sh" 
        output="screen" 
        respawn="false"/>    
    ~~~

    Enables Baxter
    ~~~
    <node name="enable_robot"
        pkg="liam_neeson" 
        type="enable_baxter.py"
        output="screen"/>
    ~~~

    Starts the joint trajectory action server
    ~~~
    <node name="joint_trajectory_action_server" 
        pkg="baxter_interface" 
        type="joint_trajectory_action_server.py" 
        output="screen"/>
    ~~~

    A last resort remapping node that subscribes to `/joint_states` and publishes to `/robot/joint_states`
    ~~~
    <node name="joint_states_remapper" 
        pkg="liam_neeson" 
        type="translator.py"
        output="screen"/>
    ~~~

    Sets up the MoveGroup interface for robot path planning
    ~~~
    <group>
        <remap from="joint_states" to="/robot/joint_states"/>

        <include file="$(find baxter_moveit_config)/launch/move_group.launch">
            <arg name="allow_trajectory_execution" value="true"/>
        </include>

        <include file="$(find baxter_moveit_config)/launch/planning_context.launch">
            <arg name="load_robot_description" value="true"/>
            <arg name="right_electric_gripper" value="true"/>
            <arg name="left_electric_gripper" value="true"/>
        </include>
    </group>
    ~~~

### 4. Notes on Package Usage

* Relevant Nodes:

    `/base_to_world`
    `/baxter_emulator`
    `/baxter_sim_io`
    `/baxter_sim_kinematics_left`
    `/baxter_sim_kinematics_right`
    `/gazebo`
    `/joint_states_remapper`
    `/joint_trajectory_action_server`
    `/link1_broadcaster`
    `/move_group`
    `/moveit_python_wrappers`
    `/obj_dregistered`
    `/point_and_point_some_more`
    `/robot/controller_spawner`
    `/robot_state_publisher`
    `/rosbag`

    openni.launch will start multiple nodes under the `/camera` namespace
  


<!-- #### 5. Useful Resources
 -->
