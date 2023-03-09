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
import copy

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from full_coverage.srv import Fullpath
from full_coverage.msg import Fullpathmsgs

from std_srvs.srv import Empty

g_map =None
qq =None
c = 0
obstacle_flag = False
fullpath_info = Fullpathmsgs()
coverage_list = []

testcnt = 0

blue_color = (255, 0, 0)
green_color = (0, 255, 0)
red_color = (0, 0, 255)
white_color = (255, 255, 255)

p_file = None
p_file2 = None
p_num = 0
p_num2 = 0
pre_dest = 0

wall_check = True
check_num = 0

distance = 30
hdistance = distance/2

coverage_list = []

def execute_remaining_area():
    global coverage_list
    #remain_list = copy.deepcopy(coverage_list)
    idx = 0
    while idx < len(coverage_list):
        try:
            if coverage_list[idx].arrival == False :
                fullpath = rospy.ServiceProxy('fullpathmove', Fullpath)
                print ("remain false")
                coverage_list[idx].x_pre = coverage_list[idx-1].x
                coverage_list[idx].y_pre = coverage_list[idx - 1].y
                resp1 = fullpath(coverage_list[idx-1])
                print (coverage_list[idx-1])
                while (not (resp1.result)):
                    None
                if resp1.result == 1:   
                    print("image_dodge_fail")
                    if idx < len(coverage_list)-4:
                        result = movebase_client(coverage_list[idx+4].x, coverage_list[idx+4].y,1)
                        coverage_list[idx].arrival = False
                        coverage_list[idx].start_flag = False
                        coverage_list[idx+1].arrival = False
                        coverage_list[idx+1].start_flag = False
                        coverage_list[idx+2].arrival = False
                        coverage_list[idx+2].start_flag = False
                        coverage_list[idx+3].arrival = False
                        coverage_list[idx+3].start_flag = False
                        coverage_list[idx+4].arrival = False
                        coverage_list[idx+4].start_flag = False

                        idx=idx+5
                    else :
                        result = movebase_client(coverage_list[len(coverage_list) + 5 - len(coverage_list)])

                elif resp1.result ==100:
                    coverage_list[idx-1].arrival = True
                    coverage_list[idx-1].start_flag = True
                    idx=idx+1
            else:
                idx= idx+1    

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
    #execute_remaining_area()






def execute_fulcoverage():
    global coverage_list
    idx = 0
    

    #coverage_list.append(Fullpathmsgs())
    first_point = Fullpathmsgs()
    
    first_point.start_flag = True
    first_point.x = coverage_list[0].x
    first_point.y = coverage_list[0].y + (15*0.025)
    first_point.rotation_flag = coverage_list[0].rotation_flag
    first_point.wall = coverage_list[0].wall
    first_point.direction = coverage_list[0].direction
    first_point.arrival = coverage_list[0].arrival
    
    coverage_list[0].x_pre  = -0.222259953061
    coverage_list[0].y_pre  = -0.00213585657632

    first_point.x_pre  = coverage_list[-2].x
    first_point.y_pre  = coverage_list[-2].y

    coverage_list.insert(0,first_point)

    coverage_list[0].start_flag = True
    coverage_list[-1].start_flag = False
    coverage_list[-1].x = coverage_list[0].x
    coverage_list[-1].y = coverage_list[0].y
    coverage_list[-1].rotation_flag = coverage_list[0].rotation_flag
    coverage_list[-1].wall[0] = True
    coverage_list[-1].wall[1] = True
    coverage_list[-1].direction = 2
    coverage_list[-1].arrival = coverage_list[0].arrival
    

    while idx < len(coverage_list):
        if idx ==0 :
            result = movebase_client(coverage_list[idx].x, coverage_list[idx].y, 1)
            coverage_list[idx].arrival = False
            coverage_list[idx].start_flag = False
            idx = idx + 1

        try:
            print ("////////////////////////////////////////////////////////",coverage_list[idx])
            fullpath = rospy.ServiceProxy('fullpathmove', Fullpath)
            resp1 = fullpath(coverage_list[idx])
            while (not (resp1.result)):
                None
            if resp1.result == 1:
                print("image_dodge_fail")
                if idx < len(coverage_list)-4:
                    result = movebase_client(coverage_list[idx+4].x, coverage_list[idx+4].y,1)
                    coverage_list[idx].arrival = False
                    coverage_list[idx].start_flag = False
                    coverage_list[idx+1].arrival = False
                    coverage_list[idx+1].start_flag = False
                    coverage_list[idx+2].arrival = False
                    coverage_list[idx+2].start_flag = False
                    coverage_list[idx+3].arrival = False
                    coverage_list[idx+3].start_flag = False
                    coverage_list[idx+4].arrival = False
                    coverage_list[idx+4].start_flag = False

                    idx=idx+5
                else :
                    result = movebase_client(coverage_list[len(coverage_list) + 5 - len(coverage_list)])

            elif resp1.result ==100:
                coverage_list[idx].arrival = True
                coverage_list[idx].start_flag = True
                idx=idx+1

            print(resp1.result)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    execute_remaining_area()



def movebase_client(x,y,z):

    clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
    clear_costmaps_srv()

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = z

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        # rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def calcPixel2Pos(x,y,th):
    global g_map
    tfs=TransformStamped()
    tfs.header.frame_id = 'map'
    tfs.child_frame_id = 'goal'
    tfs.transform.translation.x = g_map.origin.position.x + x * g_map.resolution
    tfs.transform.translation.y = g_map.origin.position.y + g_map.height * g_map.resolution - y * g_map.resolution
    tfs.transform.translation.z = th
    quaternion = tf.transformations.quaternion_from_euler(0, 0, th)
    tfs.transform.rotation.x = quaternion[0]
    tfs.transform.rotation.y = quaternion[1]
    tfs.transform.rotation.z = quaternion[2]
    tfs.transform.rotation.w = quaternion[3]
    return tfs

def mapcb(data):
    global g_map
    g_map = data
    print ("dddddddddddddd",g_map)

def drowline1(pt1,pt2,x,y) :
    global p_num
    global p_num2
    global p_file
    global p_file2
    global pre_dest
    global obstacle_flag
    global ll
    global fullpath_info
    global testcnt
    
    x=x+hdistance
    y=y+hdistance
    p_num = p_num + 1

    if pt1 == 0 and pt2 == 1:
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2, y), blue_color, 2)
    elif pt1 == 0 and pt2 == 2:
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2-hdistance/2,y-hdistance/2), blue_color, 2)
    elif pt1 == 0 and pt2 == 3:
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x,y-hdistance/2 ), blue_color, 2)
    elif pt1 == 0 and pt2 == 4:
        cv2.line(original, (x-hdistance/2, x-hdistance/2), (x-hdistance/2,y-hdistance/2+hdistance/2 ), blue_color, 2)
        
    elif pt1 == 1 and pt2 == 0 :
        cv2.line(original, (x-hdistance/2, y-hdistance/2-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
    elif pt1 == 1 and pt2 == 2:        
        cv2.line(original, (x-hdistance/2, y-hdistance/2-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2-hdistance/2, y-hdistance/2), blue_color, 2)
    elif pt1 == 1 and pt2 == 3:
    
        cv2.line(original, (x-hdistance/2, y-hdistance/2-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x+hdistance/2, y-hdistance/2), blue_color, 2)
    elif pt1 == 1 and pt2 == 4:
        cv2.line(original, (x-hdistance/2, y-hdistance/2-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2, y), blue_color, 2)
    
    elif pt1 == 2 and pt2 == 0 :
        cv2.line(original, (x-hdistance/2-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
    elif pt1 == 2 and pt2 == 1:
        cv2.line(original, (x-hdistance/2-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2-hdistance/2), blue_color, 2)
    elif pt1 == 2 and pt2 == 3:
        cv2.line(original, (x-hdistance/2-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2+hdistance/2, y-hdistance/2), blue_color, 2)
    elif pt1 == 2 and pt2 == 4:
        cv2.line(original, (x-hdistance/2-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2+hdistance/2), blue_color, 2)
    
    elif pt1 == 3 and pt2 == 0 :
        cv2.line(original, (x-hdistance/2+hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
    elif pt1 == 3 and pt2 == 1:
        cv2.line(original, (x-hdistance/2+hdistance/2, y-8), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2-hdistance/2), blue_color, 2)
    elif pt1 == 3 and pt2 == 2:        
        cv2.line(original, (x-hdistance/2-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2+hdistance/2, y-hdistance/2), blue_color, 2)
    elif pt1 == 3 and pt2 == 4:
        cv2.line(original, (x, y-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2+hdistance/2), blue_color, 2)

    elif pt1 == 4 and pt2 == 0 :
        cv2.line(original, (x-hdistance/2, y-hdistance/2+hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
    elif pt1 == 4 and pt2 == 1:
        cv2.line(original, (x-hdistance/2, y-hdistance/2+hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2-hdistance/2), blue_color, 2)
    elif pt1 == 4 and pt2 == 2:        
        cv2.line(original, (x-hdistance/2, y-hdistance/2+hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2-hdistance/2, y-hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
    elif pt1 == 4 and pt2 == 3:
        cv2.line(original, (x-hdistance/2, y-hdistance/2+hdistance/2), (x-hdistance/2, y-hdistance/2), blue_color, 2)
        cv2.line(original, (x-hdistance/2, y-hdistance/2), (x-hdistance/2+hdistance/2, y-hdistance/2), blue_color, 2)
    p_file.write('no' + str(p_num) +'  x: ' + str(x) + '  y: ' +str(y) + '\n') 
    #cv2.waitKey(0)
    tfs = calcPixel2Pos(x, y, 1)

    fullpath_info.x = tfs.transform.translation.x
    fullpath_info.y = tfs.transform.translation.y
    fullpath_info_temp = Fullpathmsgs()
    fullpath_info_temp.x = fullpath_info.x
    fullpath_info_temp.y = fullpath_info.y
    fullpath_info_temp.rotation_flag = fullpath_info.rotation_flag
    fullpath_info_temp.wall = fullpath_info.wall
    fullpath_info_temp.direction = fullpath_info.direction
    fullpath_info_temp.arrival = fullpath_info.arrival
    fullpath_info_temp.start_flag = fullpath_info.start_flag    
    #ll.append(Fullpathmsgs(tfs.transform.translation.x,tfs.transform.translation.y,c))




    coverage_list.append(fullpath_info_temp)

    coverage_list[0].x_pre = coverage_list[0].x
    coverage_list[0].y_pre = coverage_list[0].y - (15*0.025)

    idx =0
    while idx < len(coverage_list):
        if idx !=0:
            coverage_list[idx].x_pre = coverage_list[idx-1].x
            coverage_list[idx].y_pre = coverage_list[idx - 1].y
        idx = idx+1    
    '''
    if pre_dest != pt2  :
        p_num2 = p_num2 + 1
        tfs= calcPixel2Pos(x,y,1)

        print (tfs)
        #result = movebase_client(tfs.transform.translation.x,tfs.transform.translation.y,tfs.transform.translation.z)
        #if result:
        #    rospy.loginfo("Goal execution done!")
        #cv2.waitKey(0)
    
        p_file2.write('no' + str(p_num2) +'  x: ' + str(x) + '  y: ' +str(y) + '\n')
        pre_dest = pt2
        print ("tests0")
        rospy.wait_for_service('fullpathmove')
        print ("tests1")
        try:
            fullpath = rospy.ServiceProxy('fullpathmove', Fullpath)
            print (ll)
            resp1 = fullpath(ll)
            while(not(resp1.result)):
                None
            if resp1.result == 1 :
                print("image_dodge_fail")
                result = movebase_client(tfs.transform.translation.x,tfs.transform.translation.y,tfs.transform.translation.z)
                if result:
                    rospy.loginfo("Goal execution done!")
            print ("tests3")
            print(resp1.result)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        obstacle_flag = False
        ll =[]'

    #result = movebase_client(tfs.transform.translation.x,tfs.transform.translation.y,tfs.transform.translation.z)
    #if result:
    #   rospy.loginfo("Goal execution done!")'''
        
if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        rospy.Subscriber("/map_metadata", MapMetaData ,mapcb)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

    fname = '/home/zetabank/catkin_ws/src/full_coverage/scripts/map/gong_full0517.pgm'

    original = cv2.imread(fname, cv2.IMREAD_COLOR)

    s_arr = ""
    #original = cv2.line(original, (150, 150), (150, 150), red_color, 5)
    # cv2.waitKey(10)
    cv2.imshow('Original', original)
    cv2.waitKey(10)


    height, width, channel = original.shape

    arrw = int(round(width  / distance))
    arrh = int(round(float(height) / distance))
    print (float(height) / distance)
    print (width   , height )

    matrix = [[1 for col in range(arrh)] for row in range(arrw)]
    matrixpxx = [[0 for col in range(arrh)] for row in range(arrw)] 
    matrixpxy = [[0 for col in range(arrh)] for row in range(arrw)] 
    matrix2x = [[0 for col in range(arrh*2)] for row in range(arrw*2)]
    matrix2y = [[0 for col in range(arrh*2)] for row in range(arrw*2)]

    indexlist = list()

    p_file = open('test.txt', mode='wt')
    p_file2 = open('test2.txt', mode='wt')

    print (matrix[arrw-1][arrh-1])

    px = original[height-1,width-1]
    print(arrw , " " , arrh)

    for i in range(distance/2,width,distance):
        for j in range(distance/2,height,distance):
            px = original[j,i]
            if px[0] > 220 and px[1] > 220 :
                for k in range (i-hdistance, i+hdistance,1):
                    if wall_check:
                        for l in range (j-hdistance, j+ hdistance, 1):
                            if i > hdistance and j > hdistance and i < width- hdistance and j < height - hdistance:
                                px = original[l,k]
                                if px[0] <200 and px[1]<200 :
                                    check_num = 0
                                    wall_check = False
                                    break
                                else:
                                    check_num = check_num + 1
                                    print (check_num)
                    else:
                        wall_check = True
                        break            
                if check_num == (hdistance*2)*(hdistance*2) :
                    original = cv2.line(original, (i, j), (i, j), red_color, 4)
                    matrix[i/distance][j/distance] = 0
                    matrixpxx[i/distance][j/distance] = i
                    matrixpxy[i/distance][j/distance] = j
                    check_num = 0

                    print (check_num)
                elif (hdistance*2)*(hdistance*2) / 3 < check_num < (hdistance*2) * (hdistance*2) :
                    original = cv2.line(original, (i, j), (i, j), green_color, 4)
                    matrix[i/distance][j/distance] = 0
                    matrixpxx[i/distance][i/distance] = i
                    matrixpxy[i/distance][j/distance] = j
                    print (j,i)
                    check_num = 0
                    indexlist.append((j/hdistance,i/hdistance))
                    indexlist.append((j/hdistance+1,i/hdistance))
                    indexlist.append((j / hdistance , i / hdistance+1))
                    indexlist.append((j / hdistance+1 , i / hdistance+1))
                    #indexlist.append((j / 15+1, i / 15))
                    print (check_num)

                else :
                    matrix[i/distance][j/distance] = 1
                    matrixpxx[i/distance][j/distance] = i
                    matrixpxy[i/distance][j/distance] = j
                    check_num = 0

    cv2.imshow('Original', original)
    cv2.waitKey(0)

    for i in range( 0 , arrh, 1) :
        print("")
        for j in range( 0, arrw,1):
            print matrix[j][i],;

    for i in range( 0 , arrh, 1) :
        print("")
        for j in range( 0, arrw,1):
            s_arr = s_arr + str(matrix[j][i])
            
    s_arr = s_arr.replace("0","2",1)
    print s_arr
    cv2.imshow('Original', original)
    cv2.waitKey(0)
    #os.system('cd {}'.format(os.getcowd()))) 
    cmd = 'java -jar /home/zetabank/catkin_ws/src/full_coverage/scripts/test.jar ' + str(arrh) + ' ' + str(arrw) +' ' + s_arr 
    output = os.popen(cmd).read() 
    #print(output)

    jsontxt = output
    pkt = json.loads( re.sub('"', '', jsontxt))
    '''
    for i in range( 0 , arrh, 1) :
        print("")
        for j in range( 0, arrw,1):
            print matrixpxx[j][i],;

    cv2.imshow('Original', original)
    '''
    #drowline1(1,3,30,30)


    '''
    for i in range( 1 , arrh*2, 1) :
        print("")
        for j in range( 1, arrw*2,1):
            print pkt[i-1][j-1][1],;
    '''
    '''
    for i in range( 1 , arrh*2, 1) :
        print("")
        for j in range( 1, arrw*2,1):
            drowline1(pkt[i-1][j-1][0],pkt[i-1][j-1][1],j*15,i*15)
                #drowline1(pkt[i*2+1][j*2+1][0],pkt[i*2+1][j*2+1][1],j*30+15,i*30)
    '''
    cv2.imshow('Original', original)

        
    xx = 0
    yy = 0
    paintloopidx = 2
    startflag = True
    for i in range( 1 , arrh*2, 1) :
        print("")
        for j in range( 1, arrw*2, 1):
            if pkt[i-1][j-1][0] == 4  and pkt[i-1][j-1][1] == 0 and startflag :
                startflag = False
                xx = j
                yy = i
            if pkt[i-1][j-1][0] != 0  and pkt[i-1][j-1][1] != 0 :
                paintloopidx = paintloopidx + 1
            #print pkt[i-1][j-1][1]

    print paintloopidx
    print xx , yy 
    xx= xx -1
    yy= yy -1
    #drowline1(1,4,2*15,2*15)
    cv2.imshow('Original', original)

    print pkt[6][4][1],pkt[6][4][0]
    for i in range( 0 , paintloopidx, 1) :
        
        #print pkt[yy-1][xx-1][0],pkt[xx-1][yy-1][1],xx*15,xx*15, xx, yy

        print pkt[yy][xx][0],pkt[yy][xx][1],xx*hdistance,yy*hdistance, xx, yy
        if pkt[yy][xx][0] == 0 and pkt[yy][xx][1] == 4:
            #drowline1(pkt[xx][yy][0],pkt[xx][yy][1],xx*15,xx*15)
            yy=yy+1
            fullpath_info.rotation_flag = 0
            fullpath_info.direction = 4

        elif pkt[yy][xx][0] == 1 and pkt[yy][xx][1] == 4:
            #drowline1(pkt[xx][yy][0],pkt[xx][yy][1],xx*15,xx*15)
            yy=yy+1
            fullpath_info.rotation_flag = 0
            fullpath_info.direction = 4
        elif pkt[yy][xx][0] == 1 and pkt[yy][xx][1] == 2:
            xx = xx - 1
            #c
            fullpath_info.rotation_flag = -1
            fullpath_info.direction = 2
        elif pkt[yy][xx][0] == 3 and pkt[yy][xx][1] == 2:
            xx = xx - 1
            fullpath_info.rotation_flag = 0
            fullpath_info.direction = 2
        elif pkt[yy][xx][0] == 3 and pkt[yy][xx][1] == 4:
            yy = yy + 1
            #cc
            fullpath_info.rotation_flag = 2
            fullpath_info.direction = 4
        elif pkt[yy][xx][0] == 1 and pkt[yy][xx][1] == 3:
            xx = xx + 1
            #cc
            fullpath_info.rotation_flag = 1
            fullpath_info.direction = 3
        elif pkt[yy][xx][0] == 2 and pkt[yy][xx][1] == 3:
            xx = xx + 1
            fullpath_info.rotation_flag = 0
            fullpath_info.direction = 3
        elif pkt[yy][xx][0] == 2 and pkt[yy][xx][1] == 4:
            yy = yy + 1
            #c
            fullpath_info.rotation_flag = -1
            fullpath_info.direction = 4
        elif pkt[yy][xx][0] == 4 and pkt[yy][xx][1] == 0:
            yy= yy + 1
            #s
            fullpath_info.rotation_flag = 100
            fullpath_info.direction = 4
        elif pkt[yy][xx][0] == 2 and pkt[yy][xx][1] == 1:
            yy = yy - 1
            #cc
            fullpath_info.rotation_flag = 1
            fullpath_info.direction = 1
        elif pkt[yy][xx][0] == 4 and pkt[yy][xx][1] == 1:
            yy = yy - 1
            #
            fullpath_info.rotation_flag = 0
            fullpath_info.direction = 1
        elif pkt[yy][xx][0] == 4 and pkt[yy][xx][1] == 3:
            xx = xx + 1
            #c
            fullpath_info.rotation_flag = -1
            fullpath_info.direction = 3
        elif pkt[yy][xx][0] == 4 and pkt[yy][xx][1] == 2:
            xx = xx - 1
            #cc
            fullpath_info.rotation_flag = 1
            fullpath_info.direction = 2
        elif pkt[yy][xx][0] == 3 and pkt[yy][xx][1] == 1:
            yy = yy - 1
            #c
            fullpath_info.rotation_flag = -1
            fullpath_info.direction = 1

        xx_ = xx * hdistance
        yy_ = yy * hdistance

        for (first, last) in indexlist:
            if first-1 == yy and last-1 == xx :
                obstacle_flag = True

                if pkt[yy][xx-1][0] == 0 and pkt[yy][xx-1][1] == 0 :
                    xx_ = xx_+ hdistance /2

                if pkt[yy][xx+1][0] == 0 and pkt[yy][xx+1][1] == 0 :
                    xx_ = xx_ - hdistance /2

                if pkt[yy+1][xx][0] == 0 and pkt[yy+1][xx][1] == 0 :
                    yy_ = yy_ - hdistance /2

                if pkt[yy-1][xx][0] == 0 and pkt[yy-1][xx][1] == 0 :
                    yy_ = yy_  + hdistance /2

        fullpath_info.wall = [False,False,False,False]
        if pkt[yy][xx - 1][0] == 0 and pkt[yy][xx - 1][1] == 0:
            fullpath_info.wall[1] = True

        if pkt[yy][xx + 1][0] == 0 and pkt[yy][xx + 1][1] == 0:
            fullpath_info.wall[2] = True

        if pkt[yy + 1][xx][0] == 0 and pkt[yy + 1][xx][1] == 0:
            fullpath_info.wall[3] = True

        if pkt[yy - 1][xx][0] == 0 and pkt[yy - 1][xx][1] == 0:
            fullpath_info.wall[0] = True

        drowline1(pkt[yy][xx][0], pkt[yy][xx][1], xx_ , yy_)    

        cv2.imshow('Original', original)
        cv2.waitKey(0)
    # idx=0
    # for i in range(len(coverage_list)-10):
    #     coverage_list.pop()
    #     idx = idx + 1
    execute_fulcoverage()
    p_file.close()
    cv2.imshow('Original', original)

    #cv2.waitKey(0)
    cv2.destroyAllWindows()

