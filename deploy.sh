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

    echo "launching the master"
    gnome-terminal -x sh -c "ROS_MASTER_URI=$ROS_MASTER_URI; ROS_HOSTNAME=$host_address; roscore"
    sleep 5

    echo "launching the clock sync script"
    gnome-terminal -x sh -c "ROS_MASTER_URI=$ROS_MASTER_URI; ROS_HOSTNAME=$host_address; roslaunch turtlebot3_formation send_clock.launch"

    ./bash/beast_wake_up.sh ${tb3_0_ID} tb3_0
    ./bash/beast_wake_up.sh ${tb3_1_ID} tb3_1
    ./bash/beast_wake_up.sh ${tb3_2_ID} tb3_2

    # launch navigation -> set up ros master and hostname

    echo "launching tb3_0 navigation"
    gnome-terminal -x sh -c "ROS_MASTER_URI=$ROS_MASTER_URI; ROS_HOSTNAME=$host_address; python ros/navigation.py --robot=tb3_0"
    sleep 1

    echo "launching tb3_1 navigation"
    gnome-terminal -x sh -c "ROS_MASTER_URI=$ROS_MASTER_URI; ROS_HOSTNAME=$host_address; python ros/navigation.py --robot=tb3_1"
    sleep 1

    echo "launching tb3_2 navigation"
    gnome-terminal -x sh -c "ROS_MASTER_URI=$ROS_MASTER_URI; ROS_HOSTNAME=$host_address; python ros/navigation.py --robot=tb3_2"
    sleep 1


else
    echo "launching deployment sequence for simulation"

    echo "launching robots in gazebo"
    gnome-terminal -x sh -c "echo 'launching robots in gazebo'; roslaunch turtlebot3_formation main.launch; bash"
    sleep 8

    echo "launching slam"
    gnome-terminal -x sh -c "echo 'launching slam'; roslaunch turtlebot3_formation multirobot_slam.launch; bash"
    sleep 2

    echo "launching tb3_0 navigation"
    gnome-terminal -x sh -c "python ros/navigation.py --robot=tb3_0 --detector=average --controller=dynamic; bash"
    sleep 1

    echo "launching tb3_1 navigation"
    gnome-terminal -x sh -c "python ros/navigation.py --robot=tb3_1 --detector=average --controller=dynamic; bash"
    sleep 1

    echo "launching tb3_2 navigation"
    gnome-terminal -x sh -c "python ros/navigation.py --robot=tb3_2 --detector=average --controller=dynamic; bash"
    sleep 1
fi 