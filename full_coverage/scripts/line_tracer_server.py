#!/usr/bin/env python
import roslib

from full_coverage.srv import Fullpath
import rospy
import math
import numpy
import threading
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
#TEST
max_z = 0.3
x_val = 0.2
a_val = 0.2

pose = Pose()
warning_msg = ""
twist = Twist()
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

location_check_topic = '/location_check'
location_check_pub = rospy.Publisher(location_check_topic,Float64MultiArray,queue_size=1)

boundary_size_topic = "/boundary_size"
boundary_size_pub = rospy.Publisher(boundary_size_topic,Float64MultiArray,queue_size=10)
front_boundary = 0.15
width_boundary = 0.18
side_margin = 0.5
boundary_size = Float64MultiArray()
boundary_dodge = Float64MultiArray()
boundary_size.data = [front_boundary,width_boundary]
boundary_dodge.data = [front_boundary,width_boundary+side_margin]

z = 0
minimum = 0.0
minimum_range = 0.0
robot_center = 0
robot_center_range = 0
maximum = 0.0
maximum_range = 0.0
scan_rectangle_R = []
scan_rectangle_F = []
scan_rectangle_L = []

def quaternion_to_euler_angle(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

def g_rangle_range_tr(euler_z):

    angle = 0
    if euler_z > 0:
        angle = euler_z
    else:
        angle = 180 + euler_z + 179

    return angle

def get_direction(y_,x_):
    if y_ >= 0 :
        if x_ > 0 :
            return math.degrees(math.atan(y_/x_))
        elif x_ < 0 :
            return 180 + math.degrees(math.atan(y_/x_))
        else:
            return 90
    else :
        if x_ > 0 :
            return 360 + math.degrees(math.atan(y_/x_))
        elif x_ < 0 :
            return 180 + math.degrees(math.atan(y_/x_))
        else :
            return -90

def twist_init() :
    global twist

    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0

    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

def angle_scailing(z) :
    z = z-90
    
    if z<0 :
        return z + 360
    else :
        return z

def pose_send(val) :
    global pose
    global z

    pose = val
    X, Y, Z = quaternion_to_euler_angle(pose.orientation)
    z_comp = g_rangle_range_tr(Z)
    z = angle_scailing(z_comp)

def scan_rectangle_R_send(arr) :
    global scan_rectangle_R
    scan_rectangle_R = arr.data

def scan_rectangle_F_send(arr) :
    global scan_rectangle_F
    scan_rectangle_F = arr.data

def scan_rectangle_L_send(arr) :
    global scan_rectangle_L
    scan_rectangle_L = arr.data

def stop_warning_send(msg) :
    global warning_msg
    warning_msg = msg

def dodge_ready(direction):
    dodge_flag = False

    twist_init()
    cmd_vel_pub.publish(twist)
    start = time.time()
    while(warning_msg.data == "front" and dodge_flag == False) :
        if (time.time() - start >=3) :
            dodge_flag = True
    if dodge_flag == True :
        if dodge_maneuver() == 1 :
            print("dodge_fail")
            return 1
        dodge_flag = False
        print('dodge_done')

    while(abs(direction - z)>=2) :
        print("direction : ", direction)
        print("z : ", z)
        if(direction-z)/100 > max_z :
            twist.angular.z = max_z
        else :
            twist.angular.z = (direction - z)/50
        cmd_vel_pub.publish(twist)

    twist_init()
    cmd_vel_pub.publish(twist)

def vertical_go(rotation):
    obstacle_in_flag = False

    boundary_size_pub.publish(boundary_dodge)
    twist.linear.x = 0.2
    cmd_vel_pub.publish(twist)
    while True :
        obstacle_count = 0
        boundary_size_pub.publish(boundary_size)
        if warning_msg.data == 'front' :
            print("front_again")
            twist_init()
            cmd_vel_pub.publish(twist)
            return 1
        boundary_size_pub.publish(boundary_dodge)
        if rotation == -1 :
            for x in scan_rectangle_L[-55:-45] :
                if x < side_margin :
                    obstacle_count = obstacle_count +1
        elif rotation == 1 :
            for x in scan_rectangle_R[45:55] :
                if x < side_margin :
                    obstacle_count = obstacle_count +1
        # print(obstacle_count)
        if obstacle_count == 3:
            obstacle_in_flag = True
            print("Side In")
        if obstacle_in_flag :
            if obstacle_count == 0 :
                print("Side Out")
                boundary_size_pub.publish(boundary_size)
                return 0
    

def dodge_maneuver():
    global minimum
    global minimum_range
    global robot_center
    global robot_center_range
    global maximum
    global maximum_range
    global scan_rectangle_R
    global scan_rectangle_F
    global scan_rectangle_L
    global twist
    global cmd_vel_pub
    
    right_flag = True
    right_count = 0
    left_flag = True
    left_count = 0
    obstacle_edge_flag = True

    min_max_arr = Float64MultiArray()

    min_max_topic = "/range_min_max"
    min_max_arr = rospy.wait_for_message(min_max_topic,Float64MultiArray)

    minimum, minimum_range = min_max_arr.data[0], min_max_arr.data[1]
    robot_center, robot_center_range = min_max_arr.data[2], min_max_arr.data[3]
    maximum, maximum_range = min_max_arr.data[4], min_max_arr.data[5]
    

    print('minimum : ', minimum)
    print('minimum_range : ',minimum_range)
    print('maximum : ', maximum)
    print('maximum_range : ',maximum_range)

    scan_rectangle_R_topic = "/scan_rectangle_R"
    scan_rectangle_F_topic = "/scan_rectangle_F"
    scan_rectangle_L_topic = "/scan_rectangle_L"
    rospy.Subscriber(scan_rectangle_R_topic,Float64MultiArray,scan_rectangle_R_send)
    rospy.Subscriber(scan_rectangle_F_topic,Float64MultiArray,scan_rectangle_F_send)
    rospy.Subscriber(scan_rectangle_L_topic,Float64MultiArray,scan_rectangle_L_send)

    theta, center = maximum - minimum, (maximum + minimum)/2
    center_2_right = minimum_range*math.sin(math.radians(robot_center-minimum))
    center_2_left = maximum_range*math.sin(math.radians(maximum-robot_center))
    # if center_2_left < 0 :
    #     center_2_left = 0
    # if center_2_right < 0 :
    #     center_2_right = 0
    obstacle_range = math.sqrt(minimum_range**2 + maximum_range**2 - 2*minimum_range*maximum_range*math.cos(math.radians(theta)))

    print("center : ", center)
    print("obstacle_range : ", obstacle_range)
    print("center_2_right : ", center_2_right)
    print("center_2_left : ", center_2_left)

    for x in scan_rectangle_R:
        right_count = right_count+1 if x<=1 else right_count
        right_flag = False if right_count>15 else True

    for x in scan_rectangle_L:
        left_count = left_count+1 if x<=1 else left_count
        left_flag = False if left_count>15 else True

    
    rotation = -1 if right_flag == True and left_flag == False else +1 if left_flag == True and right_flag == False else 9999 if right_flag == False and left_flag == False else 0

    if rotation == 0 :
        if center_2_left >= center_2_right :
            horizontal_time = (center_2_right + 0.5)/0.2
            rotation = -1
        else :
            horizontal_time = (center_2_left + 0.5)/0.2
            rotation = 1
    elif rotation == 1 :
        horizontal_time = (center_2_left + 0.5)/0.2

    elif rotation == -1 :
        horizontal_time = (center_2_right + 0.5)/0.2

    elif rotation == 9999 :
        return 0

    vertical_time = 4

    original_goal_z = z



    goal_z = z + (rotation*90)
    if goal_z >= 360 :
        goal_z = goal_z - 360
    elif goal_z < 0 :
        goal_z = 360 + goal_z

    for i in range(1,8) :
        if i == 1 or i == 3 or i == 5 or i == 7:
            if i == 1 :
                x = 1
            elif i == 3 :
                x = -1
                goal_z = original_goal_z
            elif i == 5 :
                goal_z = z + (rotation*(-90))
                if goal_z >= 360 :
                    goal_z = goal_z - 360
                elif goal_z < 0 :
                    goal_z = 360 + goal_z
                x = -1
            elif i == 7 :
                x = 1
                goal_z = original_goal_z
            print("goal_z")
            while(abs(goal_z-z)>=2):   #1 rotate
                twist.angular.z = 0.2 * rotation * x
                cmd_vel_pub.publish(twist)
        
        elif i == 2 or i == 4 or i == 6:
            if i == 2 :
                go_time = horizontal_time
                print("horizontal_time : ", horizontal_time)
            elif i == 4 :
                go_time = vertical_time
                if vertical_go(rotation) == 1:
                    return 1
                print("vertical_time : ", vertical_time)
            elif i == 6 :
                go_time = horizontal_time
                print("horizontal_time : ", horizontal_time)

            start = time.time()
            while(time.time() - start <= go_time): #1 go
                if warning_msg.data == 'front' :
                    print("front_again")
                    twist_init()
                    cmd_vel_pub.publish(twist)
                    return 1
                twist.linear.x = 0.2
                cmd_vel_pub.publish(twist)

        twist_init()
        cmd_vel_pub.publish(twist)
        rospy.sleep(1)

    return 0


def go_line_tracer(direction,req,err):
    global pose
    global x_val
    global a_val
    global location_check_pub


    goal_location = req[-1]

    location_check = Float64MultiArray()

    for location in req :
        if 0.15>=math.sqrt((location.x - pose.position.x)**2 + (location.y - pose.position.y)**2) :
            location_check.data = [location.x, location.y]
            location_check_pub.publish(location_check)

    if err >= x_val :
        twist.linear.x = x_val
    else :
        twist.linear.x = err

    if abs(direction-z) >= 1 :
        x=1
        if abs(direction-z) > 200 :
            x = -1
        if (direction-z)/100 > max_z:
            twist.angular.z = max_z * ((direction-z)/abs(direction-z)) * x
        else :
            twist.angular.z = (direction - z)/100 * x
    else :
        twist.angular.z = 0

    direction = get_direction((goal_location.y-pose.position.y),(goal_location.x-pose.position.x))

    if warning_msg.data == "front" :
        if math.sqrt((goal_location.x-pose.position.x)**2+(goal_location.y-pose.position.y)**2)>=0.3:
            if dodge_ready(direction) == 1:
                return 1
        else :
            print("wall_close")
            return 100

    cmd_vel_pub.publish(twist)
    rospy.sleep(0.05)

def serviceproccess(req):
    global pose
    global start_flag
    global z
    global max_z
    global warning_msg
    global twist
    global cmd_vel_pub
    global x_val
    global a_val
    req = req.location
    goal_location = req[-1]
    
    

    boundary_size_pub.publish(boundary_size)

    print("go line")

    direction = get_direction((goal_location.y-pose.position.y),(goal_location.x-pose.position.x))

    print("goal_y: ",goal_location.y)
    print("goal_x: ",goal_location.x)
    print("count : ",req[0].c)
    print("direction:" + str(direction))
    while(3<=abs(direction - z)):
        direction = get_direction((goal_location.y-pose.position.y),(goal_location.x-pose.position.x))
        
        if req[0].c != 0:
            twist.angular.z = a_val*req[0].c
        else:
            twist.angular.z = a_val

        cmd_vel_pub.publish(twist)

        # print("direction:" + str(direction))
        # print("z:"+str(z))
        rospy.sleep(0.05)
    
    twist_init()
    cmd_vel_pub.publish(twist)
        
    rospy.sleep(1)
    if abs(goal_location.x-pose.position.x)>abs(goal_location.y-pose.position.y):
        err = abs(goal_location.x-pose.position.x)
        while (not(0.1>=err)):
            err = abs(goal_location.x-pose.position.x)
            if go_line_tracer(direction,req,err) == 1 :
                return 1
            elif go_line_tracer(direction,req,err) == 100 :
                twist_init()
                cmd_vel_pub.publish(twist)
                return 100
            
    else :
        err = abs(goal_location.y-pose.position.y)
        while (not(0.1>=err)):
            err = abs(goal_location.y-pose.position.y)
            if go_line_tracer(direction,req,err) == 1 :
                return 1
            elif go_line_tracer(direction,req,err) == 100 :
                twist_init()
                cmd_vel_pub.publish(twist)
                return 100

    print('goal reached')
    twist_init()
    cmd_vel_pub.publish(twist)
    
    return 100

def main():
    global timer
    rospy.init_node('line_tracer')

    robot_pose_topic = "/robot_pose"
    rospy.Subscriber(robot_pose_topic,Pose,pose_send)

    warning_topic = "/lidar_warning"
    rospy.Subscriber(warning_topic,String,stop_warning_send)


    s = rospy.Service('fullpathmove', Fullpath, serviceproccess)

    rospy.spin()

if __name__ == "__main__":
    main()