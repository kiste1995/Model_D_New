#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
last_stamp_secs = 0
last_stamp_nsecs = 0

def odom_send(msg):
    global last_stamp_secs
    global last_stamp_nsecs

    if last_stamp_secs == 0 :
        last_stamp_secs = msg.header.stamp.secs
        last_stamp_nsecs = msg.header.stamp.nsecs
        
    else :
        if last_stamp_secs >= msg.header.stamp.secs :
            if last_stamp_nsecs >= msg.header.stamp.nsecs:
                print(last_stamp_secs,last_stamp_nsecs)
                print(msg.header.stamp.secs,msg.header.stamp.nsecs)
                print("warrarar")
        last_stamp_secs = msg.header.stamp.secs
        last_stamp_nsecs = msg.header.stamp.nsecs

def main():
    rospy.init_node('odom_test')
    odom_sub = rospy.Subscriber("/odom",Odometry,odom_send)
    rospy.spin()

if __name__ == "__main__":
    main()
