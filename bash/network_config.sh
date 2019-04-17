#!/bin/sh

host_address=$(hostname -I)

host_address=$(echo ${host_address} | tr -d ' ')

ROS_MASTER_URI="http://${host_address}:11311"
ROS_HOSTNAME=host_address

