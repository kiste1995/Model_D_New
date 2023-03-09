#!/usr/bin/env bash

until rosnode info rosout grep Pid; do sleep 1; done

source /opt/ros/melodic/setup.bash
source /home/zetabank/catkin_ws/devel/setup.bash

sleep 2

roslaunch zetabank_bringup zetabank_robot_serial.launch

