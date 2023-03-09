#!/usr/bin/env python

import rospy
from PyQt5.QtCore import *
from geometry_msgs.msg import Twist

convkey = 0
prev_convkey = 0
convkey_val = 0
bRunMCtrl = False

moveBindings = {
        'w':(1,0),
        'a':(0,1),
        'd':(0,-1),
        'x':(-1,0),
        'r':(0,0),
        }

speedBindings={
        'u':(1.1,1.1),
        'm':(.9,.9),
        'i':(1.1,1),
        ',':(.9,1),
        'o':(1,1.1),
        '.':(1,.9),
        }


def vels(target_linear_vel, target_angular_vel):
    return "currently ==> linear vel:%s angular vel:%s" % (target_linear_vel,target_angular_vel)


class Thread_TeleOP(QThread):
    def __init__(self, parent = None):
        super(Thread_TeleOP, self).__init__(parent)
        #self.working = True
        #self.n = 0
        #self.main = parent
        #self.isRun = False

        self.count = 0
        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.speed = 0.1
        self.turn = 0.15
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

    def run(self):
        global convkey
        global prev_convkey
        global bRunMCtrl

        while bRunMCtrl == True:
            if convkey in moveBindings.keys():
                self.x = moveBindings[convkey][0]
                self.th = moveBindings[convkey][1]

                print vels(self.control_speed, self.control_turn)
 
            elif convkey in speedBindings.keys():
                self.speed = self.speed * speedBindings[convkey][0]              
                self.turn = self.turn * speedBindings[convkey][1]
                
                print vels(self.speed, self.turn)
 
            elif convkey == 's' :
                self.x = 0
                self.th = 0
                self.control_speed = 0
                self.control_turn = 0

            elif convkey == 'r':
                self.x = 0
                self.th = 0

            self.target_speed = self.speed * self.x

            if self.target_speed >= 1.0:
                self.target_speed = 1.0
            self.target_turn = self.turn * self.th
            if self.target_turn>= 1.0:
                self.target_turn = 1.0

            if self.target_speed > self.control_speed:
                self.control_speed = min( self.target_speed, self.control_speed + 0.02 )
            elif self.target_speed < self.control_speed:
                self.control_speed = max( self.target_speed, self.control_speed - 0.02 )
            else:
                self.control_speed = self.target_speed

            if self.target_turn > self.control_turn:
                self.control_turn = min( self.target_turn, self.control_turn + 0.1 )
            elif self.target_turn < self.control_turn:
                self.control_turn = max( self.target_turn, self.control_turn - 0.1 )
            else:
                self.control_turn = self.target_turn

            twist = Twist()
            twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
            self.pub_twist.publish(twist)

        print("Exit thread of TeleOP...")

        