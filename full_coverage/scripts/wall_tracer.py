#!/usr/bin/env python
import rospy
import roslib
import actionlib

import sys
import os
import math
import numpy
import threading
import time

import full_coverage.msg
from std_msgs.msg import UInt8

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from full_coverage.msg import lidar_filter 

sonar_sensor = dict()


sonar_front1 = "sonar1"
sonar_front2 = "sonar2" 
sonar_right1 = "sonar3"
sonar_right2 = "sonar4"
sonar_rear1 = "sonar5"
sonar_rear2 = "sonar6"
sonar_left1 = "sonar7"
sonar_left2 = "sonar8"



wall_distance = 0.25
max_a_val = 0.2
x_val = 0.1

pose = Pose()
robot_z = 0
twist = Twist()
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)




rospy.init_node('wall_tracer_server')

wall_tracer_server = actionlib.SimpleActionServer('wall_tracer', full_coverage.msg.Wall_tracerAction)

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

def angle_scailing(z) :
    z = z-90
    
    if z<0 :
        return z + 360
    else :
        return z

def recv_pose(val) :
    global pose
    global robot_z

    pose = val
    X, Y, Z = quaternion_to_euler_angle(pose.orientation)
    robot_z_comp = g_rangle_range_tr(Z)
    robot_z = angle_scailing(robot_z_comp)

def recv_sonar(data):
    global sonar_sensor
    sonar_sensor[data.header.frame_id] = data.range


def sonar_control(command0="",command1="",command2="",command3=""):
    sonar_control_topic = "/sonar_control_NUC"
    sonar_control_pub = rospy.Publisher(sonar_control_topic,UInt8,queue_size=1)

    command_list = [command0,command1,command2,command3]
    id_list = []
    result = 0b00000000

    if "front" in command_list :
        id_list.append(6)
    if "right" in command_list :
        id_list.append(4)
    if "rear" in command_list :
        id_list.append(2)
    if "left" in command_list :
        id_list.append(0)
    if "all" in command_list :
        result = 0b11111111

    for i in id_list :
        result =  (3<<i) | result

    sonar_control_pub.publish(result)

    print("sonar_control : ", bin(result))


def twist_init() :
    global twist

    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0

    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    cmd_vel_pub.publish(twist)
    rospy.sleep(0.1)

def mode_selector() :
    if sonar_sensor[sonar_front1] <= 0.3 and sonar_sensor[sonar_front2] <= 0.3 :
        return "close_corner"
    if sonar_sensor[sonar_right1] >= 0.5 and sonar_sensor[sonar_right2] >= 0.5 :
        return "open_corner"

def straight_tracer() :
    global twist


    os.system('clear')
    print("straight_tracer_mode")
    print("sonar_right1 : " + str(sonar_sensor[sonar_right1]))
    print("sonar_right2 : " + str(sonar_sensor[sonar_right2]))


    twist.linear.x = x_val
    distance_err =  wall_distance - sonar_sensor[sonar_right1]
    
    if abs(distance_err) > 0.001 :
        a_val = distance_err
        if a_val > max_a_val :
            a_val = max_a_val
        twist.angular.z = a_val
    else:
        twist.angular.z = 0
    cmd_vel_pub.publish(twist)
    rospy.sleep(0.1)

    # err = sonar_sensor[sonar_right1] - sonar_sensor[sonar_right2]

    # cmd_vel_pub.publish(twist)

def close_corner_turn() :
    global twist

    twist_init()
    rospy.sleep(0.5)\

    while sonar_sensor[sonar_front1] < 0.45 or sonar_sensor[sonar_front2] < 0.45 :
        os.system('clear')
        print("parallel2wall")
        print("sonar_right1 : " + str(sonar_sensor[sonar_right1]))
        print("sonar_right2 : " + str(sonar_sensor[sonar_right2]))
        twist.angular.z = max_a_val
        cmd_vel_pub.publish(twist)
    parallel2wall()

def open_corner_turn() :
    global twist

    twist_init()
    rospy.sleep(0.1)
    print("open_corner_turn_mode")
    goal_z = robot_z + 90

    if goal_z<0 :
        goal_z = goal_z + 360
    elif goal_z >= 360 :
        goal_z = goal_z - 360

    while abs(goal_z - robot_z) >= 0.3 :
        twist.angular.z = -max_a_val
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.1)
    
    twist_init()
    rospy.sleep(0.1)

    while sonar_sensor[sonar_right1] <= 0.15 :
        twist.linear.x = x_val
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.1)


def parallel2wall() :
    global twist

    err =  sonar_sensor[sonar_right2] - sonar_sensor[sonar_right1]
    while abs(err) > 0.01 :
        os.system('clear')
        print("parallel2wall")
        print("sonar_right1 : " + str(sonar_sensor[sonar_right1]))
        print("sonar_right2 : " + str(sonar_sensor[sonar_right2]))
        a_val = err*2
        if a_val > max_a_val :
            a_val = max_a_val
        twist.angular.z = a_val
        cmd_vel_pub.publish(twist)
        err = sonar_sensor[sonar_right2] - sonar_sensor[sonar_right1]
        rospy.sleep(0.1)

    twist_init()
    rospy.sleep(0.1)

def wall_tracer() :
    # sonar_control("front","right")

    parallel2wall()

    while True :
        mode = mode_selector()
        if mode == "close_corner" :
            close_corner_turn()
        # elif mode == "open_corner" :
        #     open_corner_turn()
        straight_tracer()
    

def wall_tracer_cb (msg) :
    wall_tracer_feedback = full_coverage.msg.Wall_tracerFeedback()
    wall_tracer_result = full_coverage.msg.Wall_tracerResult()

    print("callback")

    print(msg)

    # wall_tracer_feedback.feedback = "running"
    # wall_tracer_server.publish_feedback(wall_tracer_feedback)



    wall_tracer()

    print("Done")
    wall_tracer_result.result = "Done"
    wall_tracer_server.set_succeeded(wall_tracer_result)


def main():
    global wall_tracer_server
    

    topic_name = "/sonar"
    rospy.Subscriber(topic_name,Range,recv_sonar)

    sonar_control("all")



    while not len(sonar_sensor) == 8 :
        
        pass

    print("ready")
    wall_tracer_server = actionlib.SimpleActionServer('wall_tracer', full_coverage.msg.Wall_tracerAction , execute_cb = wall_tracer_cb)
    wall_tracer_server.start()


    robot_pose_topic = "/robot_pose"
    rospy.Subscriber(robot_pose_topic,Pose,recv_pose)

    rospy.spin()

if __name__ == "__main__":
    main()