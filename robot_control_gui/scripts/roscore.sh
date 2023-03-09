#!/usr/bin/env bash

until rosnode info rosout  grep Pid; do sleep 1; done
source /home/zetabank/catkin_ws/devel/setup.bash

sleep 1

roscore