#!/bin/sh

ID=$1
base_IP="192.168.2." 
ROBOT_IP=$base_IP$((150 + ID))

echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_HOSTNAME: $ROS_HOSTNAME"
echo "\n"

echo "Awakening beast $ID with IP: $ROBOT_IP"

REMOTE_COMMAND="
ROS_MASTER_URI=$ROS_MASTER_URI;
python receive_clock.py;
roslaunch turtlebot3_bringup turtlebot3_robot.launch"

# better to use expect 
ssh robotlab@$ROBOT_IP
sleep 2
echo "r0b0t\t"
sleep 2
echo $REMOTE_COMMAND
echo "exit"

echo "DONE"