#!bin/bash

cd /home/zetabank/catkin_ws/src/robot_control_gui/scripts

roscore & 
python ZBMRGS.py --rootdir=/home/zetabank/catkin_ws/src/robot_control_gui/ --wpdir=/home/zetabank/catkin_ws/src/navi_waypoint/ --robotid=DI_1 --autostart=True



