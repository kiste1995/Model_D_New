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
from full_coverage.msg import lidar_filter 

#TEST
max_a_val = 0.3
x_val = 0.2
a_val = 0.2

pose = Pose()
warning_msg = lidar_filter()
twist = Twist()
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

location_check_topic = '/location_check'
location_check_pub = rospy.Publisher(location_check_topic,Float64MultiArray,queue_size=1)

boundary_size_topic = "/boundary_size"
boundary_size_pub = rospy.Publisher(boundary_size_topic,Float64MultiArray,queue_size=10)
front_boundary = 0.20
width_boundary = 0.19
side_margin = 0.5
boundary_size = Float64MultiArray()
boundary_dodge = Float64MultiArray()
boundary_rotation = Float64MultiArray()
boundary_size.data = [front_boundary,width_boundary]
boundary_dodge.data = [front_boundary,width_boundary+side_margin]
boundary_rotation.data = [0,width_boundary + 0.35]

side_flag = ''

robot_z = 0
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

def get_rotation_flag(direction) :
    global robot_z

    relative_angle = direction - robot_z

    if relative_angle < 0 :
        relative_angle =  relative_angle + 360

    if relative_angle <= 3 or relative_angle >= 357 :
        return 0 , relative_angle
    elif relative_angle <= 180 :
        return 1 , relative_angle
    elif relative_angle > 180 :
        return -1 , relative_angle

def pose_send(val) :
    global pose
    global robot_z

    pose = val
    X, Y, Z = quaternion_to_euler_angle(pose.orientation)
    robot_z_comp = g_rangle_range_tr(Z)
    robot_z = angle_scailing(robot_z_comp)

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
    global side_flag
    warning_msg = msg
    # # if warning_msg.warning[0] == "left" :
    # #     side_flag = warning_msg.warning[0]
    # elif warning_msg.warning[2] == "right" :
    #     side_flag = warning_msg.warning[2]
    # else :
    #     side_flag = ""
    side_flag = warning_msg.warning[2]

def dodge_ready(direction):
    global twist
    dodge_flag = False

    twist_init()
    cmd_vel_pub.publish(twist)
    start = time.time()

    while(warning_msg.warning[1] == "front" and dodge_flag == False) :
        if (time.time() - start >=3) :
            dodge_flag = True

        if dodge_flag == True :
            print('dodge_mode')
            twist.linear.x = -x_val
            cmd_vel_pub.publish(twist)
            rospy.sleep(0.5)
            twist_init()
            cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
            return 1

def vertical_go(rotation):
    obstacle_in_flag = False

    boundary_size_pub.publish(boundary_dodge)
    twist.linear.x = x_val
    cmd_vel_pub.publish(twist)
    while True :
        obstacle_count = 0
        boundary_size_pub.publish(boundary_size)
        if warning_msg.warning[1] == 'front' :
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

    original_goal_z = robot_z

    goal_z = robot_z + (rotation*90)
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
            while(abs(goal_z - robot_z)>=2):   #1 rotate
                twist.angular.z = a_val * rotation * x
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
                if warning_msg.warning[1] == 'front' :
                    print("front_again")
                    twist_init()
                    cmd_vel_pub.publish(twist)
                    return 1
                twist.linear.x = x_val
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
    global max_a_val


    # location_check = Float64MultiArray()

    # for location in req :
    #     if 0.15>=math.sqrt((location.x - pose.position.x)**2 + (location.y - pose.position.y)**2) :
    #         location_check.data = [location.x, location.y]
    #         location_check_pub.publish(location_check)

    twist.linear.x = x_val

    rotation_flag , relative_angle = get_rotation_flag(direction)

    if relative_angle >= 1 and relative_angle <= 180 :
        x = 1
        val_z = relative_angle

        if val_z/30 > max_a_val:
            twist.angular.z = max_a_val * x
        else :
            twist.angular.z = val_z/30 * x

    elif relative_angle <= 359 and relative_angle >= 180 :
        x = -1
        val_z = 360 - relative_angle

        if val_z/30 > max_a_val:
            twist.angular.z = max_a_val * x
        else :
            twist.angular.z = val_z/30 * x

    else :
        twist.angular.z = 0

    direction = get_direction((req.y-pose.position.y),(req.x-pose.position.x))

    if warning_msg.warning[1] == "front" :
        if math.sqrt((req.x-pose.position.x)**2+(req.y-pose.position.y)**2)>=0.2 and req.wall[req.direction-1] == False:
            if dodge_ready(direction) == 1:
                return 1
        else :
            print("wall_close")
            return 100

    cmd_vel_pub.publish(twist)
    rospy.sleep(0.05)

def serviceproccess(req):
    global pose
    global robot_z
    global twist
    global cmd_vel_pub
    global a_val



    boundary_size_pub.publish(boundary_size)
    
    print("go line")
    location = req.location

    x_pre, y_pre = location.x_pre, location.y_pre

    direction = get_direction((location.y-pose.position.y),(location.x-pose.position.x))

    rotation_flag, relative_angle = get_rotation_flag(direction)

    # x_pre, y_pre = pose.position.x, pose.position.y

    print("goal_x: ",location.x)
    print("goal_y: ",location.y)
    print("count : ",rotation_flag)
    print("direction:" + str(direction))

    if abs(location.x-pose.position.x)>abs(location.y-pose.position.y):
        err = abs(location.x-pose.position.x)
        err_flag = 'x'
    else :
        err = abs(location.y-pose.position.y)
        err_flag = 'y'
    
    if err >= 0.05 :
        twist_init()
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.2)
        
        while((3<=abs(direction - robot_z)) and rotation_flag != 0):
            direction = get_direction((location.y-pose.position.y),(location.x-pose.position.x))

            twist.angular.z = a_val * rotation_flag
            cmd_vel_pub.publish(twist)

            # print("direction:" + str(direction))
            # print("robot_z:"+str(robot_z))
            rospy.sleep(0.05)

        twist_init()
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)

    if abs(location.x-pose.position.x)>abs(location.y-pose.position.y):
        err = abs(location.x-pose.position.x)
        while (0.05<=err):
            err = abs(location.x-pose.position.x)
            if go_line_tracer(direction,location,err) == 1 :
                return 1
            elif go_line_tracer(direction,location,err) == 100 :
                break
    else :
        err = abs(location.y-pose.position.y)
        while (0.05<=err):
            err = abs(location.y-pose.position.y)
            if go_line_tracer(direction,location,err) == 1 :
                return 1
            elif go_line_tracer(direction,location,err) == 100 :
                break

    # while (0.05<=err):
    #     if abs(location.x-pose.position.x)>abs(location.y-pose.position.y):
    #         err = abs(location.x-pose.position.x)
    #     else :
    #         err = abs(location.y-pose.position.y)
        
    #     if go_line_tracer(direction,location,err) == 1 :
    #         return 1
    #     elif go_line_tracer(direction,location,err) == 100 :
    #         break

    print('goal reached')

    twist_init()
    cmd_vel_pub.publish(twist)
    print("start_flag :",location.start_flag)
    ideal_direction = get_direction((location.y-y_pre),(location.x-x_pre))
    # if not(location.start_flag) :
    #     coner_coverage(location,ideal_direction,x_pre,y_pre)
    

    return 100

def wall_count(wall) :
    count = 0
    print('wall_count')
    for x in wall:
        if x == True :
            count = count + 1
    return count

def side_wall_check(wall,direction) :
    print('side_wall_check')
    print('wall',wall)
    print('direction', direction)
    if direction == 4:
        if wall[1] == True :
            return 'right'
        elif wall[2] == True :
            return 'left'
    elif direction == 3:
        if wall[3] == True :
            return 'right'
        elif wall[0] == True :
            return 'left'
    elif direction == 2:
        if wall[0] == True :
            return 'right'
        elif wall[3] == True :
            return 'left'
    elif direction == 1:
        if wall[2] == True :
            return 'right'
        elif wall[1] == True :
            return 'left'

def reverse_mode(rotation) :
    global twist
    global side_flag
    side_flag_count = 0

    reverse_flag = True

    print("reverse_mode")

    start = time.time()
    while reverse_flag :
        rospy.sleep(0.01)
        twist.linear.x = -(x_val)
        cmd_vel_pub.publish(twist)
        if side_flag != rotation :
            side_flag_count = side_flag_count + 1
            print("reverse_mode_side_flag : ", side_flag)
            print("reverse_mode_side_flag_count : ", side_flag_count)
        else :
            side_flag_count = 0

        if side_flag_count >= 5 :
            reverse_flag = False

    twist_init()
    cmd_vel_pub.publish(twist)
    rospy.sleep(0.1)
    
    return (time.time()- start - 0.1)*x_val
    
def trun(rotation,direction,degree = 0) :
    global pose
    global robot_z
    global twist
    global a_val
    global side_flag

    print('side_flag_starttttttttttttttttttttttttttttt',side_flag)
    side_flag_count = 0

    roll = 0
    reverse_flag = False
    reverse_distance = 0

    boundary_size_pub.publish(boundary_rotation)

    rospy.sleep(1)
    print('turn_90')
    print(rotation)

    for i in range(20):
        if side_flag == rotation :
            side_flag_count = side_flag_count + 1
            print("side_flag_count : ",side_flag_count)


    if side_flag_count > 5 :
        print("side_flag_count_sum : ",side_flag_count)
        reverse_flag = True
        print("reverse_flag : ", reverse_flag)

    if reverse_flag == True :
        reverse_distance = reverse_mode(rotation)
        reverse_flag = False

    robot_z_original = robot_z
    twist_init()
    cmd_vel_pub.publish(twist)
    if rotation == 'right' :
        roll = -1
    elif rotation == 'left' :
        roll = 1

    goal_direction = direction + (roll) * degree

    if goal_direction<0 :
        goal_direction = goal_direction + 360
    elif goal_direction >= 360 :
        goal_direction = goal_direction - 360

    start = time.time()
    while abs(goal_direction-robot_z)>=3 :
        rospy.sleep(0.01)
        twist.angular.z = a_val * roll
        cmd_vel_pub.publish(twist)
    
    twist_init()
    cmd_vel_pub.publish(twist)

    boundary_size_pub.publish(boundary_size)
    rospy.sleep(0.2)

    return reverse_distance

def go2wall () :
    global pose
    global twist
    global x_val
    global warning_msg

    print('go2wall')
    twist_init()
    cmd_vel_pub.publish(twist)

    start = time.time()

    while warning_msg.warning[1] != "front" :
        rospy.sleep(0.01)
        twist.linear.x = x_val
        cmd_vel_pub.publish(twist)
    
    go2wall_distance = x_val * (time.time() - start)

    twist_init()
    cmd_vel_pub.publish(twist)
    return go2wall_distance

def go2back (back_distance) :
    global pose
    global twist
    global x_val
    global warning_msg


    print('go2back')
    twist_init()
    cmd_vel_pub.publish(twist)
    print("back_distance :",back_distance)

    start = time.time()
    while time.time() - start <= back_distance/x_val :
        rospy.sleep(0.01)
        twist.linear.x = x_val
        cmd_vel_pub.publish(twist)
        if warning_msg.warning[1] == "front" :
            twist_init()
            cmd_vel_pub.publish(twist)
            return (time.time() - start) * x_val 

    twist_init()
    cmd_vel_pub.publish(twist)

    return 9999

def go2return (x_deffrence,y_deffrence) :
    global pose
    global twist
    global x_val
    global warning_msg

    print('go2return')
    twist_init()
    cmd_vel_pub.publish(twist)

    return_distance = math.sqrt((pose.position.x - x_deffrence - pose.position.x)**2 + (y_deffrence - pose.position.y)**2)

    start = time.time()
    while time.time() - start <= return_distance/x_val :
        twist.linear.x = x_val
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.01)

    twist_init()
    cmd_vel_pub.publish(twist)

def coner_coverage(location,direction,x_pre,y_pre):
    global pose
    global robot_z

    front_wall = False

    print('coner_coverage')

    pose_original_x , pose_original_y = pose.position.x , pose.position.y
    print("pose_original :", pose_original_x, pose_original_y)
    print("x_pre, y_pre :", x_pre, y_pre)
    back_distance = math.sqrt((pose_original_x - x_pre)**2 + (pose_original_y - y_pre)**2)

    wall_cnt = wall_count(location.wall)


    if wall_cnt == 2 :
        if location.wall[location.direction-1] == True :
            front_wall = True
            side_wall = side_wall_check(location.wall,location.direction)

            go2wall_distance = go2wall()
            reverse_distance = trun(side_wall,direction,90)
            go2wall_distance = go2wall_distance - reverse_distance
            back_distance_comp = back_distance + go2wall_distance

            side_wall_distance = go2wall()
            # x_deffrence , y_deffrence = (pose.position.x-pose_original_x), (pose.position.y-pose_original_y)
            reverse_distance = trun(side_wall,direction,180)

            go2back_distance = go2back(back_distance_comp)
            if go2back_distance <= back_distance :
                back_distance = go2back_distance
            back_distance = back_distance - reverse_distance
            trun(side_wall,direction,270)

            # go2return(y_deffrence,y_deffrence)
            go2back(side_wall_distance)
            trun(side_wall,direction)
            go2back(back_distance)

            
    elif wall_cnt == 1 :
        if location.wall[location.direction-1] == True :
            front_wall = True
        else :
            side_wall = side_wall_check(location.wall,location.direction)
            reverse_distance = trun(side_wall,direction,90)

            side_wall_distance = go2wall()
            # x_deffrence , y_deffrence = (pose.position.x-pose_original_x), (pose.position.y-pose_original_y)
            reverse_distance = trun(side_wall,direction,180)
            side_wall_distance = side_wall_distance - reverse_distance


            go2back_distance = go2back(back_distance)
            if go2back_distance <= back_distance :
                back_distance = go2back_distance
            trun(side_wall,direction,270)

            # go2return(x_deffrence,y_deffrence)
            go2back(side_wall_distance)
            trun(side_wall,direction)
            go2back(back_distance)

def main():
    global timer
    
    rospy.init_node('cell_tracer_server')

    robot_pose_topic = "/robot_pose"
    rospy.Subscriber(robot_pose_topic,Pose,pose_send)

    warning_topic = "/lidar_warning"
    rospy.Subscriber(warning_topic,lidar_filter,stop_warning_send)


    srv = rospy.Service('fullpathmove', Fullpath, serviceproccess)

    rospy.spin()

if __name__ == "__main__":
    main()