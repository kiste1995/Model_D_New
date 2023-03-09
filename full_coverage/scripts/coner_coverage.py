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

x_pre = 999
y_pre = 999
go2wall_distance = 0

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

wall_close_flag = False
wall_following_flag = False
robot_z = 0
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
    global robot_z

    pose = val
    X, Y, Z = quaternion_to_euler_angle(pose.orientation)
    robot_z_comp = g_rangle_range_tr(Z)
    robot_z = angle_scailing(robot_z_comp)

def stop_warning_send(msg) :
    global warning_msg
    warning_msg = msg

def serviceproccess(req):
    global pose
    global start_flag
    global robot_z
    global max_z
    global warning_msg
    global twist
    global cmd_vel_pub
    global x_val
    global a_val
    global wall_close_flag
    global wall_following_flag
    global x_pre
    global y_pre

    boundary_size_pub.publish(boundary_size)

    print("go line")
    location = req.location
    x_pre, y_pre = pose.position.x, pose.position.y 

    # if wall_close_flag == True and location.rotation_flag == 1 :
    #     wall_close_flag == False
    #     wall_following_flag = True

    direction = get_direction((location.y-pose.position.y),(location.x-pose.position.x))

    rotation_flag, relative_angle = get_rotation_flag(direction)

    print("goal_x: ",location.x)
    print("goal_y: ",location.y)
    print("count : ",rotation_flag)
    print("direction:" + str(direction))

    

    if rotation_flag != 0 :
        twist_init()
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)
        
        while(3<=abs(direction - robot_z)):
            direction = get_direction((location.y-pose.position.y),(location.x-pose.position.x))

            twist.angular.z = a_val * rotation_flag
            cmd_vel_pub.publish(twist)

            # print("direction:" + str(direction))
            # print("robot_z:"+str(robot_z))
            rospy.sleep(0.05)

    twist_init()
    cmd_vel_pub.publish(twist)
        
    rospy.sleep(1)

    if abs(location.x-pose.position.x)>abs(location.y-pose.position.y):
        err = abs(location.x-pose.position.x)
        while (0.1<=err):
            err = abs(location.x-pose.position.x)
            if go_line_tracer(direction,location,err) == 1 :
                return 1
            elif go_line_tracer(direction,location,err) == 100 :
                break
    else :
        err = abs(location.y-pose.position.y)
        while (0.1<=err):
            err = abs(location.y-pose.position.y)
            if go_line_tracer(direction,location,err) == 1 :
                return 1
            elif go_line_tracer(direction,location,err) == 100 :
                break

    print('goal reached')
    twist_init()
    cmd_vel_pub.publish(twist)


    return 100

def wall_count(wall) :
    count = 0
    for x in wall:
        if x == True :
            count = count + 1
    return count

def side_wall_check(wall,direction) :
    if direction == 4:
        if wall[2] == True :
            return 'right'
        elif wall[3] == True :
            return 'left'
    elif direction == 3:
        if wall[4] == True :
            return 'right'
        elif wall[1] == True :
            return 'left'
    if direction == 2:
        if wall[1] == True :
            return 'right'
        elif wall[4] == True :
            return 'left'
    if direction == 1:
        if wall[3] == True :
            return 'right'
        elif wall[2] == True :
            return 'left'

def trun_90 (rotation) :
    global pose
    global robot_z
    global twist
    global a_val
    

    robot_z_original = robot_z
    twist_init()
    cmd_vel_pub.publish(twist)
    if rotation == 'right' :
        roll = -1    
    elif rotation == 'left' :
        roll = 1

    for robot_z == robot_z_original + 90 * roll :
        twist.angular.z = a_val * roll
        cmd_vel_pub.publish(twist)
    
    twist_init()
    cmd_vel_pub.publish(twist)

def go2wall () :
    global pose
    global twist
    global x_val
    global warning_msg
    global go2wall_distance

    twist_init()
    cmd_vel_pub.publish(twist)

    start = time.time()
    for warning_msg.data != "front" :
        twist.linear.x = x_val
        cmd_vel_pub.publish(twist)
    go2wll_distance = x_val * (start-time.time())

    twist_init()
    cmd_vel_pub.publish(twist)

def go2back (back_distance) :
    global pose
    global twist
    global x_val
    global warning_msg
    global go2wall_distance

    twist_init()
    cmd_vel_pub.publish(twist)

    start = time.time()
    for time.time() - start <= back_distance/x_val :
        twist.linear.x = x_val
        cmd_vel_pub.publish(twist)

    twist_init()
    cmd_vel_pub.publish(twist)

def go2return (x_pre,y_pre) :
    global pose
    global twist
    global x_val
    global warning_msg
    global go2wall_distance

    twist_init()
    cmd_vel_pub.publish(twist)

    return_distance = math.sqrt((x_pre - pose.position.x)**2 + (y_pre - pose.position.y)**2))

    start = time.time()
    for time.time() - start <= return_distance/x_val :
        twist.linear.x = x_val
        cmd_vel_pub.publish(twist)

    twist_init()
    cmd_vel_pub.publish(twist)

class pose_orignal():
    def __init__(self,x,y):
        self.x = x
        self.y = y

def coner_coverage(msg):
    global pose
    global robot_z
    global x_pre
    global y_pre

    front_wall = False

    pose_orignal.x , pose_orignal.y = pose.position.x , pose.position.y
    back_distance = math.sqrt((pose_orgical.x - x_pre)**2 + (pose_orignal.x - x_pre)**2)


    wall_cnt = wall_count(msg.wall)
    if wall_cnt == 2 :
        if wall_cnt[msg.direction-1] == True :
            front_wall = True
        side_wall = side_wall_check(msg.wall,msg.direction)

    elif wall_cnt == 1 :
        if wall_cnt[msg.direction-1] == True :
            front_wall = True
        else :
            side_wall = side_wall_check(msg.wall,msg.direction)
            trun_90(side_wall)
            go2wall()
            trun_90(side_wall)
            go2back(back_distance)
            trun_90(side_wall)
            go2return(x_pre,y_pre)
            trun_90(side_wall)
            go2back(back_distance)



    elif wall_cnt ==0 :

    msg.rotation_flag


def main():
    global timer
    rospy.init_node('coner_coverage')

    robot_pose_topic = "/robot_pose"
    rospy.Subscriber(robot_pose_topic,Pose,pose_send)

    warning_topic = "/lidar_warning"
    rospy.Subscriber(warning_topic,String,stop_warning_send)


    s = rospy.Service('fullpathmove', Fullpath, serviceproccess)

    rospy.spin()

if __name__ == "__main__":
    main()