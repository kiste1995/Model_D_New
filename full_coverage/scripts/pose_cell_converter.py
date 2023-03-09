#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import json
import threading
import math
import actionlib
import time
import os
from full_coverage.srv import Cell2pose
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

pose = Pose()
boundary_pixcel = 30
resolution = 0.025
boundary = resolution * (boundary_pixcel/2)

cell_name_topic = "/cell_name"
cell_name_pub = rospy.Publisher(cell_name_topic,String,queue_size=10)

cell_data = {}

def recv_robot_pose(msg) :
    global pose
    global cell_name_pub
    global cell_data

    cell_name = "empty"

    pose = msg
    for i in cell_data :
            if abs(i['x'] - pose.position.x) < boundary/2 :
                if abs(i['y'] - pose.position.y) < boundary/2 :
                    cell_name = i['cell_name']
                    

    cell_name_pub.publish(cell_name)
        
def cell2pose(msg) :
    global cell_data
    
    for i in cell_data :
        if i["cell_name"] == msg.cell_name :
            return i["x"],i["y"]
    return 9999,9999

def main() :
    global cell_data

    rospy.init_node("pose_cell_converter")

    json_dir = os.path.dirname( os.path.abspath( __file__ ) ) + "/map/cell_info.json"
    with open(json_dir,'r') as json_file:
        cell_data = json.load(json_file)

    robot_pose_topic = "/robot_pose"
    rospy.Subscriber(robot_pose_topic,Pose,recv_robot_pose)

    cell2pose_srv = rospy.Service('cell2pose', Cell2pose, cell2pose)


    rospy.spin()



if __name__ == "__main__":
    main()