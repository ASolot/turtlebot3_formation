#!/bin/sh

# This script is intended to auto-start all nodes and launch files

if [ "$1" = "real" ]; then
    echo "here it goes the real stuff"

    # we have 3 robots, we request their IDs 
    echo "Please input the robot IDs"
    
    echo "tb3_0 ID"
    read tb3_0_ID

    echo "tb3_1 ID"
    read tb3_1_ID

    echo "tb3_2 ID"
    read tb3_2_ID

    # we configure the network parameters 

    host_address=$(hostname -I)
    host_address=$(echo ${host_address} | tr -d ' ')

    ROS_MASTER_URI="http://${host_address}:11311"
    ROS_HOSTNAME=$host_address

    export ROS_MASTER_URI
    export ROS_HOSTNAME

    # base_IP="192.168.2." 
    # tb3_0_IP=$base_IP$((150 + tb3_0_ID))
    # tb3_1_IP=$base_IP$((150 + tb3_1_ID))
    # tb3_2_IP=$base_IP$((150 + tb3_2_ID))

    # export tb3_0_IP
    # export tb3_1_IP
    # export tb3_2_IP

    # any child we launch inherits the bash varibles :) 
    # bring the beasts alive + clock time

    gnome-terminal -x sh -c "roslaunch exercises send_clock.launch"

    ./bash/beast_wake_up.sh ${tb3_0_ID}
    ./bash/beast_wake_up.sh ${tb3_1_ID}
    ./bash/beast_wake_up.sh ${tb3_2_ID}

    # have all the networks configured properly



else
    echo "launching deployment sequence for simulation"

    echo "launching robots in gazebo"
    gnome-terminal -x sh -c "echo 'launching robots in gazebo'; roslaunch turtlebot3_formation main.launch; bash"
    sleep 8

    echo "launching slam"
    gnome-terminal -x sh -c "echo 'launching slam'; roslaunch turtlebot3_formation multirobot_slam.launch"
    sleep 2

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