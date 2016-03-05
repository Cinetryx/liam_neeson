#!/bin/bash          
#
# Script to delay the launch of a roslaunch file
# 
# Koen Lekkerkerker
# Thu 24 Apr 2014 
#
# Use: ./timed_roslaunch.sh [number of seconds to delay] [rospkg] [script file]
#

function showHelp(){
    echo 
    echo "This script can delay the execution of a rosrun command"
    echo "Place it in the 'scripts' folder of your catkin package"
    echo "and make sure that the file is executable (chmod +x timed_rosrun.sh)"
    echo 
    echo "Run it from command line:"
    echo 
    echo "Use: ./timed_rosrun.sh [number of seconds to delay] [rospkg] [roslaunch file] [arguments (optional)]"
    echo "Or: rosrun [yourpackage] time_roslaunch.sh [number of seconds to delay] [rospkg] [roslaunch file] [arguments (optional)]"
    echo "Example: ./timed_rosrun.sh 2 turtlebot_navigation amcl_demo.py initial_pose_x:=17.0 initial_pose_y:=17.0"
    echo 
    echo "Or run it from a roslaunch file:"
    echo 
    echo '<launch>'
    echo '  <arg name="initial_pose_y" default="17.0" />'
    echo '  <node pkg="semantic_turtle_test" type="timed_rosrun.sh"'
    echo '    args="2 turtlebot_navigation amcl_demo.py initial_pose_x:=17.0 initial_pose_y:=$(arg initial_pose_y)"'
    echo '    name="timed_rosrun" output="screen">'
    echo '  </node>'
    echo '</launch>'
}

if [ "$1" = "-h" ]; then
    showHelp
else 
    echo "start wait for $1 seconds"
    sleep $1
    echo "end wait for $1 seconds"
    shift
        echo "now running 'rosrun $@'"
    rosrun baxter_tools enable_robot.py -e
fi