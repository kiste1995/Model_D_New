#!/usr/bin/env bash

sleep 60

source /opt/ros/melodic/setup.bash
source /home/zetabank/catkin_ws/devel/setup.bash

#roscore

#sleep 2

roslaunch zetabank_bringup zetabank_robot_serial.launch
