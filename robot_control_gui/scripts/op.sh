#!/usr/bin/env bash

until rosnode info laserscan_multi_merger  grep Pid; do sleep 1; done

source /opt/ros/melodic/setup.bash
source /home/zetabank/catkin_ws/devel/setup.bash

sleep 4

roslaunch zetabot_main operate_robot.launch
