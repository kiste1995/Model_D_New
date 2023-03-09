#!/usr/bin/env bash

until rosnode info sick_tim551_2050001 grep Pid; do sleep 1; done

source /opt/ros/melodic/setup.bash
source /home/zetabank/catkin_ws/devel/setup.bash

sleep 4

roslaunch ira_laser_tools laserscan_multi_merger.launch
