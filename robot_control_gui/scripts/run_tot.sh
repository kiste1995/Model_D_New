#!/usr/bin/env bash

until rosnode info STM grep Pid; do sleep 1; done

source /opt/ros/melodic/setup.bash
source /home/zetabank/catkin_ws/devel/setup.bash

sleep 30

roslaunch robot_control_gui ZBMRCS_Tot.launch