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

testsrv =Fullpathmsgs()



def main():
    '''
    a = Fullpathmsgs(1,2,-1)
    b = Fullpathmsgs(2,2,-1)
    c = Fullpathmsgs(3,2,-1)
    d = Fullpathmsgs(9,9,9)
    ll = [a,b,c]
    ll.append(d)

    print(len(ll))'''

    print (testsrv)


    testsrv.x = 1
    testsrv.y = 2
    testsrv.rotation_flag = 3
    testsrv.wall = [False,False,False,False]
    testsrv.direction = 4
    #
    print (testsrv)

    #rospy.wait_for_service('fullpathmove')

    #fullpath = rospy.ServiceProxy('fullpathmove', Fullpath)
    #fullpath(ll)

    #rospy.spin()

if __name__ == '__main__':
    main()