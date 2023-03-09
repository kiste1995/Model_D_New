#!/usr/bin/env bash

until rosnode info initial_pos_srv  grep Pid; do sleep 1; done

source /opt/ros/melodic/setup.bash
source /home/zetabank/catkin_ws/devel/setup.bash

sleep 4

roslaunch zetabank_navigation normal_navigation.launch
