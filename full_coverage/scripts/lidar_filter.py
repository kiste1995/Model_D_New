#!/usr/bin/env python

import rospy
# ROS Image message
from sensor_msgs.msg import LaserScan
from full_coverage.msg import lidar_filter 
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
# OpenCV2 for saving an image
import threading
import math


front_boundary = 0.50
width_boundary = 0.29



def boundary_send(msg) :
    global front_boundary
    global width_boundary
    front_boundary = msg.data[0]
    width_boundary = msg.data[1]
    

def filter(lidar_data):
    global front_boundary
    global width_boundary

    warning = lidar_filter()

    warning_range = []
    comp = [[],[],[]]
    scan_rectangle_R = Float64MultiArray()
    scan_rectangle_F = Float64MultiArray()
    scan_rectangle_L = Float64MultiArray()
    count_right = 0
    count_front = 0
    count_left = 0

    min_max = Float64MultiArray()

    warning_pub = rospy.Publisher('/lidar_warning', lidar_filter, queue_size=10)
    warning_range_min_max_pub = rospy.Publisher('/range_min_max',Float64MultiArray,queue_size=10)
    scan_rectangle_R_pub = rospy.Publisher('/scan_rectangle_R',Float64MultiArray,queue_size=10)
    scan_rectangle_F_pub = rospy.Publisher('/scan_rectangle_F',Float64MultiArray,queue_size=10)
    scan_rectangle_L_pub = rospy.Publisher('/scan_rectangle_L',Float64MultiArray,queue_size=10)

    ranges = lidar_data.ranges
    value = math.sqrt(0.2**2+0.7815**2)

    coner = round(math.degrees(math.atan(front_boundary/width_boundary)))
    degree = 0

    for x in ranges:
        
        if degree <= 45:  # Right
            y = abs(x * math.cos(math.radians(45-degree)))
            comp[0].append(y)
            if y<=width_boundary:
                count_right = count_right + 1
            # elif y>width_boundary:
            #     print(str(degree)+": safe :"+str(y))

        if 45 < degree < coner+45: # Right - Front
            y = abs(x * math.cos(math.radians(degree-45)))
            comp[0].append(y)
            if y<=width_boundary:
                warning_range.append(degree)
                warning_range.append(x)
                count_front = count_front + 1
                count_right = count_right + 1
            # elif y>width_boundary:
            #     print(str(degree)+": safe :"+str(y))

        if coner+45 < degree <= 135: # Front
            y = x * math.sin(math.radians(degree-45))
            comp[1].append(y)

            if degree == 135 :
                robot_center = degree
                robot_center_range = x
            if y<=front_boundary:
                # print(str(degree)+": warning :"+str(y))
                warning_range.append(degree)
                warning_range.append(x)
                count_front = count_front + 1
            elif y>front_boundary:
                # print(str(degree)+": safe :"+str(y))
                None

        if 135 < degree < 225-coner: # Front
            y = x * math.cos(math.radians(degree - 135))
            comp[1].append(y)

            if y<=front_boundary:
                # print(str(degree)+": warning :"+str(y))
                warning_range.append(degree)
                warning_range.append(x)
                count_front = count_front + 1
            elif y>front_boundary:
                None
                # print(str(degree)+": safe :"+str(y))

        if 225-coner < degree < 225: #Front - Left
            y = x * math.sin(math.radians(degree-135))
            comp[2].append(y)
            if y<=width_boundary:
                warning_range.append(degree)
                warning_range.append(x)
                count_front = count_front + 1
                count_left = count_left + 1

        if 225 < x: #Left
            y = x * math.cos(math.radians(degree-225))
            comp[2].append(y)
            if y<=width_boundary:
                count_left = count_left + 1

        degree = degree +1

    if count_front > 30 :
        warning.warning[1] = "front"
        min_max.data = [warning_range[0],warning_range[1],robot_center,robot_center_range,warning_range[-2],warning_range[-1]]
        if min_max.data[0] and min_max.data[1] and min_max.data[2] and min_max.data[3] and min_max.data[4] and min_max.data[5] :
            warning_range_min_max_pub.publish(min_max)
        print("warning front")
    else :
        warning.warning[1] = ""
        min_max.data = []

    if count_left > 10 :
        warning.warning[0] = "left"
        print("warning left")
    else :
        warning.warning[0] = "1234567890123456790"

    if count_right > 10 :
        warning.warning[2] = "right"
        print("warning right")

    else :
        warning.warning[2] = "12345678901234567890"





    scan_rectangle_R.data = comp[0]
    scan_rectangle_F.data = comp[1]
    scan_rectangle_L.data = comp[2]
    scan_rectangle_R_pub.publish(scan_rectangle_R)
    scan_rectangle_F_pub.publish(scan_rectangle_F)
    scan_rectangle_L_pub.publish(scan_rectangle_L)
    scan_rectangle_R.data = []
    scan_rectangle_F.data = []
    scan_rectangle_L.data = []




    warning_pub.publish(warning)
        

    

def main():

    rospy.init_node('lidar_filter')
    # Define your image topic
    lidar_scan = "/scan"
    # Set up your subscriber and define its callback
    lidar = rospy.Subscriber(lidar_scan, LaserScan,filter)

    boundary_topic = "/boundary_size"
    boundary_sub = rospy.Subscriber(boundary_topic,Float64MultiArray,boundary_send)
    # Spin until ctrl + c

    rospy.spin()

if __name__ == '__main__':
    main()