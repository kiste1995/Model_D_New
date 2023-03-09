#!/usr/bin/env bash

until rosnode info amcl grep Pid; do sleep 1; done

source /opt/ros/melodic/setup.bash
source /home/zetabank/catkin_ws/devel/setup.bash

sleep 4

roslaunch robot_control_gui ZBMRCS.launch