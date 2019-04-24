#!/bin/sh

ID=$1
NAME=$2
base_IP="192.168.2." 
ROBOT_IP=$base_IP$((150 + ID))

# echo "ROS_MASTER_URI: $ROS_MASTER_URI"
# echo "ROS_HOSTNAME: $ROS_HOSTNAME"
# echo "\n"

echo "Awakening beast $ID with name $NAME and IP: $ROBOT_IP"

REMOTE_COMMAND="
ROS_MASTER_URI=$ROS_MASTER_URI;
python receive_clock.py;
ROS_NAMESPACE=$NAME roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=$NAME"

gnome-terminal -x sh -c "ssh -t robotlab@$ROBOT_IP; echo 'r0b0t'; "


echo "\n\nType the following command in the robot terminal: \n"
echo $REMOTE_COMMAND

# better to use expect 
# ssh robotlab@$ROBOT_IP $REMOTE_COMMAND
# ssh robotlab@$ROBOT_IP

# $REMOTE_COMMAND
# sleep 2
# echo "r0b0t\t"
# sleep 2
# echo $REMOTE_COMMAND
# echo "exit"

# echo "DONE"