#!/bin/bash          


function showHelp(){
    echo 
    echo "This script runs the send_urdf_fragment.py script"
    echo "provided by Rethink Robotics's Mutable Robot State Publisher"
    echo
    echo "http://sdk.rethinkrobotics.com/wiki/URDF_Configuration_Example"
    echo
}

if [ "$1" = "-h" ]; then
    showHelp
else 
    echo "Pulling end-effector configuration files: left_end_effector.urdf.xacro and right_end_effector.urdf.xacro, from `rospack find baxter_description`/urdf/ "
    rosrun baxter_examples send_urdf_fragment.py -f `rospack find baxter_description`/urdf/left_end_effector.urdf.xacro -l left_hand -j left_gripper_base & pidleft=$!
    rosrun baxter_examples send_urdf_fragment.py -f `rospack find baxter_description`/urdf/right_end_effector.urdf.xacro -l right_hand -j right_gripper_base & pidright=$!
    echo "Sleeping"
    sleep 20
    echo "Waking up"
    echo "Nodes started:"
    echo $(rosnode list |grep --color=always configure_urdf)
    kill -2 $pidleft
    kill -2 $pidright
    echo "Nodes after kill:" 
    echo $(rosnode list |grep --color=always configure_urdf)
fi