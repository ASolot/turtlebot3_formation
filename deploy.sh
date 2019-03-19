#!/bin/sh

# This script is intended to auto-start all nodes and launch files

if [ "$1" = "real" ]; then
    echo "here it goes the real stuff"
else
    echo "launching deployment sequence for simulation"

    echo "launching robots in gazebo"
    gnome-terminal -x sh -c "echo 'launching robots in gazebo'; roslaunch turtlebot3_formation main.launch; bash"
    sleep 5

    echo "launching slam"
    gnome-terminal -x sh -c "echo 'launching slam'; roslaunch turtlebot3_formation multirobot_slam.launch"
    sleep 5

    # echo "launching leader navigation"
    # gnome-terminal -x sh -c "echo 'leader navigation'; bash"
    # sleep 0.5

    # echo "launching follower1 navigation"
    # gnome-terminal -x sh -c "echo 'follower1 navigation'; bash"
    # sleep 0.5

    # echo "launching follower2 navigation"
    # gnome-terminal -x sh -c "echo 'follower2 navigation'; bash"
    # sleep 0.5
fi 