#!/usr/bin/env python

import cv2
import numpy as np
import math 
import sys
import json
import os
import commands
import re
import struct

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from full_coverage.srv import Fullpath
from full_coverage.msg import Fullpathmsgs


def serviceproccess(msg):
    print(msg.a[0].x)
    return 1


def main():
    rospy.init_node('line_tracer')
    
    s = rospy.Service('fullpathmove', Fullpath, serviceproccess)

    rospy.spin()

    

if __name__ == '__main__':
    main()