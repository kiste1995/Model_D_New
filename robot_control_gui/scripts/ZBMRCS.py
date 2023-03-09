#!/usr/bin/env python

import os, sys
import yaml
import rospy
import re
import actionlib
import threading
import sys, select, termios, tty
import std_msgs
import math
from time import time
from time import sleep
from time import localtime
from time import strftime
# import pyrealsense2 as rs
import subprocess
from std_srvs.srv import Empty

from autocharge.msg import ChargingAction, ChargingActionGoal, ChargingGoal
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, String, UInt8
from std_msgs.msg import UInt16, UInt64, Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
from zetabot_main.msg import PowerControlMsgs
from zetabank_msgs.msg import NavigationControl, NavigationControlStatus
from zetabank_msgs.msg import SaveWaypoint
from zetabot_main.msg import EnvironmentMsgs
from zetabank_msgs.msg import BatteryInformationMsgs
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import schedule
import subprocess
from shlex import shlex
from os import kill
from signal import alarm, signal, SIGALRM, SIGKILL, SIGTERM, SIGINT
from subprocess import PIPE, Popen
import threading

# import argparse

# parser = argparse.ArgumentParser(description="Robot Control Program")
# parser.add_argument('--rootdir', type=str, help='root directory : robot control gui', default='/home/zetabank/catkin_ws/src/robot_control_gui')
# parser.add_argument('--wpdir', type=str, help='way point moving directory : robot control gui', default='/home/zetabank/catkin_ws/src/navi_waypoint')
# parser.add_argument('--robotid', type=str, help='id of robot : robot control gui', default='DI_1')
# parser.add_argument('--autostart', type=bool, help='auto run : robot control gui', default=False)
# args = parser.parse_args()
# print("args:", args)
# rootdir = args.rootdir
# print('rootdir:', rootdir)
# wpdir = args.wpdir
# robotid = args.robotid
# print('robotid:', robotid)
# autorun = args.autostart
# print('autorun:', autorun)


rospy.init_node('ZBMRCS_node')

wpdir = rospy.get_param('~wp_dir')
print('wpdir', wpdir)
rootdir = rospy.get_param('~root_dir')
print('rootdir:', rootdir)
print('wpdir', wpdir)
robotid = rospy.get_param('~robot_id')
print('robotid:', robotid)
autorun = rospy.get_param('~auto_start')
print('autorun:', autorun)

# yaml_fname = wpdir +"/config/waypoint6.yaml"
yaml_fname = wpdir +"/config/waypoint_221122.yaml"

# navirviz_fname = "/home/zetabank/catkin_ws/src/zetabank_robot_control/zetabank_navigation/rviz/zetabank_nav.rviz"
# slamrviz_fname = "/home/zetabank/catkin_ws/src/zetabank_robot_control/zetabank_slam/rviz/zetabank_slam.rviz"


bWPFileLoadOK = False       
air_result = ''
floor_result = ''

convkey = 0
prev_convkey = 0
convkey_val = 0
bRunMCtrl = False

TeleOP_Cont = True

moveBindings = {
        'w':(1,0),
        'a':(0,1),
        'd':(0,-1),
        'x':(-1,0),
        }

speedBindings={
        'u':(1.1,1.1),
        'm':(0.9,0.9),
        'i':(1.1,1.0),
        ',':(0.9,1.0),
        'o':(1.0,1.1),
        '.':(1.0,0.9),
        }

purifier_level = {
    "Off" : 0,
    "Level-1" : 100,
    "Level-2" : 250,
    "Level-3" : 400,
}

led_color = {
    "Black" : 0x000000,
    "Stay" : 0x000000,
    "White" : 0xffffff,
    "Red" : 0xff0000,
    "Lime" : 0x00ff00,
    "Blue" : 0x0000ff,
    "Yellow" : 0xffff00,
    "Cyan" : 0x00ffff,
    "Magenta" : 0xff00ff,
    "Silver" : 0xc0c0c0,
    "Gray" : 0x808080,
    "Maroon" : 0x800000,
    "Olive" : 0x808000,
    "Green" : 0x008000,
    "Purple" : 0x800080,
    "Teal" : 0x008080,
    "Navy" : 0x000080,
    "Orange" : 0xff7800,
    "DarkWhite" : 0xe1d9d1,
    "Steel" : 0x777b7e,
    "Iron" : 0x484948,
    "Shadow" : 0x363636,
    "Charcoal" : 0x222021,
}

led_mode = {
    "Off" : 0x00,
    "On" : 0x01,
    "Blink" : 0x02,
    "FBlink" : 0x03,
    "Fade" : 0x04,
    "Sweep" : 0x05,
    "FSweep" : 0x06,
    "Stay" : 0xff,
}

led_control = {
    "Charging-low-bat" : ("Fade", "Red", "Fade", "Red", "Charging"),
    "Charging-middle-bat" : ("Fade", "Orange", "Fade", "Orange", "Charging"),
    "Charging-high-bat" : ("Fade", "Lime", "Fade", "Green", "Charging"),
    "Charging-full-bat" : ("On", "Green", "On", "Green", "Charging"),
    "Low_bat" : ("Fade", "Red", "Fade", "Red", "Charging"),
    "Middle-bat" : ("On", "Orange", "On", "Orange", "Charging"),
    "High-bat" : ("On", "Lime", "On", "Green", "Charging"),
    "Full-bat" : ("On", "Green", "On", "Green", "Charging"),
    "E-Stop" : ("FBlink", "Red", "FBlink", "Red", "EStop"),
    "Warning" : ("FBlink", "Orange", "FBlink", "Orange", "Warning"),
    "Low_bat" : ("On", "Red", "On", "Red", "Low_battery"),
    "Service" : ("On", "Yellow", "On", "White", "Service"),
    "QR_code" : ("Sweep", "Blue", "Stay", "Stay", "Service"),
    "Face_detect" : ("Sweep", "Green", "Stay", "Stay", "Service"),
    "Normal" : ("On", "White", "On", "White", "Normal"),
    "Full-coverage" : ("On", "Blue", "On", "Blue", "Full-coverage"),
    "Air_Purifier" : ("On", "Cyan", "On", "Cyan", "Air_Purifier"),
    "UVC" : ("On", "Teal", "On", "Teal", "UVC"),
    "Pump" : ("On", "Steel", "On", "Steel", "Pump"),
    "Sol" : ("On", "Iron", "On", "Iron", "Sol"),
    "Navigation_Normal" : ("FSweep", "Navy", "FSweep", "Green", "Navigation_Normal"),
    "Navigation_AP" : ("FSweep", "Navy", "FSweep", "Navy", "Navigation_AP"),
    "SLAM" : ("FBlink", "White", "FBlink", "White", "SLAM"),
    "Auto_ParkingCS" : ("Sweep", "Magenta", "Sweep", "Magenta", "SLAM"),
    "Schedule" : ("FSweep", "Green", "Blink", "Blue", "Schedule"),
    "ReadyDone" : ("Blink", "Shadow", "On", "Charcoal", "ReadyDone"),
    "None" : ("Off", "Black", "Off", "Black", "None"),
}

initSpeed = 0.1
initTurn = 0.15

open_airinfo_dlg = False

bRunSchedule = False

MATH_RAD2DEG = 57.2957795

log_fname = []
log_now_time = []
safety_info = 0
working_region = 0
working_mode = 0
robot_mode = 1
robot_pos = []
logfp = []
blogFileReady = False


logdata = { "now_time" : "2202-08-01 12:00:00",
            "safety_info" : 0x00,
            "working_region" : 0x00,
            "working_mode" : 0x00,
            "robot_mode" : 0x00,
            "robot_status" : "none",
            "pos_x" : 0.0,
            "pos_y" : 0.0,
            "pos_z" : 0.0

}

def vels(target_linear_vel, target_angular_vel):
    return "currently ==> linear vel:%s angular vel:%s" % (target_linear_vel,target_angular_vel)

AutoParkNum = 1

bCSPRunOk = False
brunLCDCmd = False



class Thread_RunSchedule:

    def __init__(self):
        # super(Thread_RunSchedule, self).__init__(parent)

        self.runcnt = 1
        self.bCSParking = False
        self.bRunNaviWP = False
        self.bRunTrajNavi = True
        self.bAirPuriRun = False
        self.robot_mode = "None"
        self.bUVCRun = False
        self.bPumpRun = False
        self.bSolRun = False
        self.naviStatus = []
        self.bEStop = False
        self.bprevEStop = False
        self.bEStop_Status = 0
        self.schedule_run_cnt = 0
        self.trajmode = NavigationControl.LOOP       

        self.battery_cnt = 2
        self.battery_low = 11.0
        self.battery_middle = 80.0
        self.battery_full = 92.0
        self.hysteresis_val = 3.0
        self.hysteresis_low = 0.0
        self.hysteresis_middle = 0.0
        self.hysteresis_high = 0.0
        self.battery_mode = "None"
        self.Bat_AvgVoltage = 0
        self.Bat_AvgCurrent = 0
        self.Bat_AvgSOC = 0
        self.batSVCnt = 0
        self.bLowBat = False
        self.bViewSonarData = False
        self.prevbattery_mode = "None"
        self.bRunWaypoint = False

        self.bBatFirstRun = True
        self.bLowbatPark = False
        self.bBatteryFull = False

        self.csStatus = ""
        self.bRunAutoParking = False

        self.Bat1_Info = BatteryInformationMsgs()
        self.Bat2_Info = BatteryInformationMsgs()
        

        self.air_purifier_pub = rospy.Publisher("/purifier_control_command", UInt16, queue_size=10)
        self.ledcommand_pub = rospy.Publisher("/led_control_command", UInt64, queue_size=10)
        self.power_control_pub = rospy.Publisher("/power_control_command", PowerControlMsgs, queue_size=10)
        self.naviCtrl_pub = rospy.Publisher("/navi_ctrl", NavigationControl, queue_size = 10)

        self.navictrl_status_sub = rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)
        self.estop_sub = rospy.Subscriber("/estop", Bool, self.CallEStop)

        self.bat_sub = rospy.Subscriber("/battery", BatteryInformationMsgs, self.CallbackBatteryStatus)
        self.charge_state_sub = rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)
        self.charge_state_nuc_sub = rospy.Subscriber("/autocharge_state_NUC", UInt8, self.CallbackCSStatusNUC)

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        

    def run(self):
        global bRunSchedule

        schedule.every().day.at("09:00").do(self.RunTrajLoop)
        schedule.every().day.at("12:00").do(self.RunCSParking)

        schedule.every().day.at("13:00").do(self.RunTrajLoop)
        schedule.every().day.at("17:00").do(self.RunCSParking)

        # schedule.every().day.at("17:55").do(self.RunTrajLoopFinal)

        # schedule.every().day.at("18:30").do(self.RunTrajLoopPumpOn)
        # schedule.every().day.at("18:40").do(self.RunCSParkingPumpOff)

        # schedule.every().day.at("19:22").do(self.RunTrajLoopPumpOn)
        # schedule.every().day.at("19:25").do(self.RunCSParkingPumpOff)

        # schedule.every().day.at("10:55").do(self.RunTrajLoop)
        # schedule.every().day.at("11:05").do(self.RunCSParking)

        # schedule.every().day.at("11:10").do(self.RunTrajLoop)
        # schedule.every().day.at("11:20").do(self.RunCSParking)

        # schedule.every().day.at("11:25").do(self.RunTrajLoop)
        # schedule.every().day.at("11:35").do(self.RunCSParking)

        # schedule.every().day.at("11:40").do(self.RunTrajLoop)
        # schedule.every().day.at("11:50").do(self.RunCSParking)

        # schedule.every().day.at("11:55").do(self.RunTrajLoop)
        # schedule.every().day.at("12:05").do(self.RunCSParking)

        # schedule.every().day.at("13:10").do(self.RunTrajLoop)
        # schedule.every().day.at("13:15").do(self.RunCSParking)

        # schedule.every().day.at("13:20").do(self.RunTrajLoop)
        # schedule.every().day.at("13:25").do(self.RunCSParking)

        # schedule.every().day.at("13:30").do(self.RunTrajLoop)
        # schedule.every().day.at("13:35").do(self.RunCSParking)

        print("Run Schedule....")
        
        while bRunSchedule == True:
            schedule.run_pending()
            sleep(0.1)

        print("Exit schedule thread...")


    def StopSchedule(self):
        global bRunSchedule

        schedule.cancel_job(self.RunTrajLoop)
        schedule.cancel_job(self.RunCSParking)

        schedule.clear(tag=None)

        bRunSchedule = False

        print("Stopping schedule...")

    def CallbackBatteryStatus(self, msgs):

        # if self.bStartOK == False:
        #     return        

        if msgs.id == 0x60:
            self.Bat1_Info = msgs
        else:
            self.Bat2_Info = msgs

        self.Bat_AvgVoltage = (self.Bat1_Info.voltage + self.Bat2_Info.voltage) / self.battery_cnt*1.0
        self.Bat_AvgCurrent = (self.Bat1_Info.current + self.Bat2_Info.current) / self.battery_cnt*1.0
        self.Bat_AvgSOC = (self.Bat1_Info.SOC + self.Bat2_Info.SOC) / self.battery_cnt*1.0
 
        self.batSVCnt += 1
        if(self.batSVCnt > 4):
            self.batSVCnt = 0

            if self.robot_mode == "Charging":
                try:
                    if self.Bat_AvgSOC <= (self.battery_low - self.hysteresis_low):
                        self.hysteresis_low = 0.0
                        self.battery_mode = "Charging-low-bat"
                        self.bLowBat = True
                    elif (self.battery_low + self.hysteresis_low) <= self.Bat_AvgSOC < (self.battery_middle - self.hysteresis_middle):
                        self.hysteresis_low = 0.0
                        self.hysteresis_middle = 0.0
                        self.battery_mode = "Charging-middle-bat"
                        self.bLowBat = False
                    elif (self.battery_middle + self.hysteresis_middle) <= self.Bat_AvgSOC< (self.battery_full - self.hysteresis_high):
                        self.hysteresis_middle = 0.0
                        self.hysteresis_high = 0.0
                        self.battery_mode = "Charging-high-bat"
                        self.bLowBat = False
                    elif self.battery_full <= self.Bat_AvgSOC:          
                        self.battery_mode = "Charging-full-bat"
                        self.bLowBat = False
                    else:
                        self.battery_mode = "None"            

                    if  self.prevbattery_mode != self.battery_mode:
                        self.SendLedCmd(self.battery_mode)
                        self.prevbattery_mode = self.battery_mode  
                        print("Charging [3]: change battery mode ==> SendLedCmd")
                except:
                    print("[Error] : Led control.")

            else:
                try:
                    if self.Bat_AvgSOC <= (self.battery_low - self.hysteresis_low):
                        self.hysteresis_low = 0.0
                        self.bLowBat = True
                        self.battery_mode = "Charging-low-bat"
                    elif (self.battery_low + self.hysteresis_low) <= self.Bat_AvgSOC < (self.battery_middle - self.hysteresis_middle):
                        self.hysteresis_low = 0.0
                        self.hysteresis_middle = 0.0
                        self.bLowBat = False
                        self.battery_mode = "Charging-middle-bat"
                    elif (self.battery_middle + self.hysteresis_middle) <= self.Bat_AvgSOC< (self.battery_full - self.hysteresis_high):
                        self.hysteresis_middle = 0.0
                        self.hysteresis_high = 0.0
                        self.battery_mode = "Charging-high-bat"
                        self.bLowBat = False
                    elif self.battery_full <= self.Bat_AvgSOC:            
                        self.bLowBat = False
                        self.battery_mode = "Charging-full-bat"
                    else:
                        self.battery_mode = "None"            

                except:
                    pass

            if self.robot_mode == "FullCharging" and self.prevbattery_mode != self.battery_mode:
                self.bLowBat = False
                self.prevbattery_mode = self.battery_mode    
                self.battery_mode = "Charging-full-bat"
                self.SendLedCmd(self.battery_mode)

                print("Full charging mode...")

    def CallEStop(self, msgs):
        # global logdata

        self.bEStop = msgs.data
        self.bEStop_Status = self.bEStop

        if self.bEStop is True and self.bprevEStop == False:
            # logdata["safety_info"] = 0x01
            # logdata["robot_status"] = "EStop"
            # self.write_log()

            self.robot_mode = "E-Stop"
            self.SendLedCmd(self.robot_mode)

            print("!! ESTOP !!")

            self.bprevEStop = True
            
            if self.bRunTrajNavi is True:
                self.onTrajStop("Traj1")

            if self.bCSParking is True:
                self.bCSParking = False
   
                print("Cancel parking charge station-->ESTOP.")

            self.robot_mode = "E-Stop"
            self.SendLedCmd(self.robot_mode)

        else:
            if self.bprevEStop is True and self.bEStop is False:
                self.bprevEStop = False

                self.robot_mode = "Normal"
                self.SendLedCmd(self.robot_mode)

                print("ESTOP ==> SendLedCmd, robot_mode : Normal")

                if self.bRunTrajNavi is True:
                    self.onTrajStart("Traj1")

                # logdata["safety_info"] = 0x00

    def CallbackRunNaviCtrlStatus(self, request):
        # if self.bStartOK == False:
        #     return

        # if self.bWPCRun is True:

        self.naviStatus = request.status

        if self.bEStop is False:
            mstr = request.status_description

        if self.naviStatus is NavigationControlStatus.COMPLETED:
            if self.bRunNaviWP is True:
                self.bRunNaviWP = False
                
                self.robot_mode = "None"
                self.SendLedCmd(self.robot_mode)

        if self.naviStatus is NavigationControlStatus.TRAJCOMPLETED:
        
            if self.bRunTrajNavi is True:
                self.bRunTrajNavi = False

                self.robot_mode = "None"
                self.SendLedCmd(self.robot_mode)

    def CallbackCSStatus(self, msgs):
        # if self.bStartOK == False:
        #     return

        if self.bCSParking is True and (self.robot_mode != "Charging" or self.robot_mode != "FullCharging"):
            self.csStatus = msgs.data

            if self.csStatus == "contact" and self.robot_mode != "Charging":

                self.robot_mode = "Charging"
                # self.bViewCmd = False
                self.prevbattery_mode = "None"
                self.bRunAutoParking = False

                print("CallbackCSStatus ==> prevbattery_mode:" + self.prevbattery_mode + " battery_mode:" + self.battery_mode)

    def CallbackCSStatusNUC(self, msg):
        # if self.bStartOK == False:
        #     return

        csnuc_state = msg.data

        if self.bLowbatPark == True and csnuc_state == 5:
            self.bLowbatPark = False     

        if csnuc_state == 6:
            self.bBatteryFull = True

            if self.robot_mode == "Charging":
                self.robot_mode = "FullCharging"
                self.battery_mode = "Charging-full-bat"
                
                self.robot_mode = "Navigation_Normal"

                self.SendLedCmd(self.battery_mode)
                print("Change robot mode to FullCharging")

        else:
            self.bBatteryFull = False


    def onTrajStart(self, name):
        global logdata

        self.ClearCostMap()

        print("costmap : onTrajStart")

        sleep(2)

        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = name
        nc.mode = self.trajmode
        self.naviCtrl_pub.publish(nc)

        mstr = "Start trajector : " + name
        print(mstr)

        self.bRunTrajNavi = True        
        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        self.SendLedCmd(self.robot_mode)

        print("send led cmd  : onTrajStartBtn ==> robot_mode :" + self.robot_mode)

        logdata["robot_status"] = "Driving"

        
    def onTrajStop(self, name):
        global logdata

        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.mode = NavigationControl.NONE
        nc.goal_name = name
        self.naviCtrl_pub.publish(nc)
        mstr = "Stop trajector : " + name
        print(mstr)

        self.bRunTrajNavi = False
        if self.bAirPuriRun is True:
            self.robot_mode = "Air_Purifier"
        else:
            self.robot_mode = "None"

        self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "None"


    def RunTrajLoopNum(self):
        self.runtrajnum.emit(self.runcnt)
        self.runcnt += 1

        if self.runcnt >= 5:
            self.runcnt = 1

    def purifier_control(self, val):
        self.AirPurifierVal = val
        purifier_val = UInt16()
        purifier_val.data = self.AirPurifierVal
    
        self.air_purifier_pub.publish(purifier_val)

    def onPuriOn(self):
        global air_result

        if self.bAirPuriRun is False:
            self.bAirPuriRun = True

            self.purifier_control(400)

            if self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "Air_Purifier"

            self.SendLedCmd(self.robot_mode)
    
    def onPuriOff(self):
        global air_result

        if self.bAirPuriRun is True:
            self.bAirPuriRun = False

            self.purifier_control(0)

            if self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            self.SendLedCmd(self.robot_mode)

    def onUVCOn(self):
        if self.bUVCRun == False:
            self.bUVCRun = True
            uvc_control_msg = PowerControlMsgs()
            uvc_control_msg.port = 15
            uvc_control_msg.state = True
            self.power_control_pub.publish(uvc_control_msg)
            
            if self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "UVC"

            self.SendLedCmd(self.robot_mode)


    def onUVCOff(self):
        if self.bUVCRun == True:
            self.bUVCRun = False
            uvc_control_msg = PowerControlMsgs()
            uvc_control_msg.port = 15
            uvc_control_msg.state = False
            self.power_control_pub.publish(uvc_control_msg)

            if self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            self.SendLedCmd(self.robot_mode)

    def onPumpOn(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bPumpRun == False:
            self.bPumpRun = True
            pump_control_msg = PowerControlMsgs()
            pump_control_msg.port = 5
            pump_control_msg.state = True
            self.power_control_pub.publish(pump_control_msg)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "Pump"

            self.SendLedCmd(self.robot_mode)

    def onPumpOff(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bPumpRun == True:
            self.bPumpRun = False
            pump_control_msg = PowerControlMsgs()
            pump_control_msg.port = 5
            pump_control_msg.state = False
            self.power_control_pub.publish(pump_control_msg)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            self.SendLedCmd(self.robot_mode)

    def onSolValveOn(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bSolRun == False:
            self.bSolRun = True
            sol_control_msg = PowerControlMsgs()
            sol_control_msg.port = 14
            sol_control_msg.state = True
            self.power_control_pub.publish(sol_control_msg)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "Sol"
            
            self.SendLedCmd(self.robot_mode)


    def onSolValveOff(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bSolRun == True:
            self.bSolRun = False
            sol_control_msg = PowerControlMsgs()
            sol_control_msg.port = 14
            sol_control_msg.state = False
            self.power_control_pub.publish(sol_control_msg)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            self.SendLedCmd(self.robot_mode)

    def gotoWayPoint(self, wpname):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = wpname
        self.naviCtrl_pub.publish(nc)
        
        self.wpName = wpname
        
        mstr = "[WP] Start way point : " + nc.goal_name
        print(mstr)

        self.bRunNaviWP = True

    def SendLedCmd(self, bat_mode):
        global led_control
        global led_mode
        global led_color

        fm = led_control[bat_mode][0]
        fc = led_control[bat_mode][1]
        rm = led_control[bat_mode][2]
        rc = led_control[bat_mode][3]

        fm = led_mode[fm]
        fc = led_color[fc]
        rm = led_mode[rm]
        rc = led_color[rc]

        fcolor_cmd = UInt64()
        fcolor_cmd = ((((fm << 24) | fc) << 32) | (((rm << 24) | rc)))
        self.ledcommand_pub.publish(fcolor_cmd)
        sleep(1.0)
        self.ledcommand_pub.publish(fcolor_cmd)
        sleep(1.0)
        self.ledcommand_pub.publish(fcolor_cmd)

        print("Thread_Schedule : Run SendLedCmd ==> mode:" + bat_mode)


    def InitPosition(self):

        init_pose_param = rospy.get_param("robot_init_pose")

        publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.loginfo("This node sets the robot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
        start_pos = PoseWithCovarianceStamped()

        #filling header with relevant information
        start_pos.header.frame_id = "map"
        start_pos.header.stamp = rospy.Time.now()
        #filling payload with relevant information gathered from subscribing
        # to initialpose topic published by RVIZ via rostopic echo initialpose
        start_pos.pose.pose.position.x = init_pose_param['position_x']
        start_pos.pose.pose.position.y = init_pose_param['position_y']
        start_pos.pose.pose.position.z = 1.0

        start_pos.pose.pose.orientation.x = 0.0
        start_pos.pose.pose.orientation.y = 0.0
        start_pos.pose.pose.orientation.z = init_pose_param['orientation_z']
        start_pos.pose.pose.orientation.w = init_pose_param['orientation_w']

        start_pos.pose.covariance[0] = 0.25
        start_pos.pose.covariance[7] = 0.25
        start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[35] = 0.06853891945200942

        publisher.publish(start_pos)

        sleep(2)

        publisher.publish(start_pos)

    def ClearCostMap(self):
        # cmdstr = "rosservice call /move_base/clear_costmaps \"{}\"  -1 "
        cmdstr = "gnome-terminal -- rosservice call /move_base/clear_costmaps \"{}\"  -1 "
        os.system(cmdstr)

        # os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")


    def RunTrajLoop(self):
        global AutoParkNum
        # global bCSPRunOk

        self.schedule_run_cnt += 1

        print("start RunTrajLoop....")

        if self.bCSParking == True:
            AutoParkNum = 2
            self.RunCSAutoparking(False)

            # bCSPRunOk = False

            # while True:
            #     if bCSPRunOk == True:
            #         bCSPRunOk = False
            #         print("break while in RunTrajLoop")
            #         break

            #     sleep(0.1)
            
            self.bCSParking = False

        print("start onTrajStart ...")
        # print("start onTrajStart ==> bCSPRunOk :" + str(bCSPRunOk))

        self.onTrajStart("Traj1")

        print("Run trajectory loop : #" + str(self.schedule_run_cnt))

        sleep(2)
        self.onPuriOn()
        sleep(3)
        self.onUVCOn()

        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        self.SendLedCmd(self.robot_mode)

        self.bRunTrajNavi = True

        print("send led cmd  : RunTrajLoop ==> robot_mode :" + self.robot_mode)        
        

    def RunTrajLoopPumpOn(self):
        global AutoParkNum
        # global bCSPRunOk

        self.schedule_run_cnt += 1

        print("start RunTrajLoop....")

        if self.bCSParking == True:
            AutoParkNum = 2
            self.RunCSAutoparking(False)

            # bCSPRunOk = False

            # while True:
            #     if bCSPRunOk == True:
            #         bCSPRunOk = False
            #         print("break while in RunTrajLoop")
            #         break

            #     sleep(0.1)
            
            self.bCSParking = False

        print("start onTrajStart ...")
        # print("start onTrajStart ==> bCSPRunOk :" + str(bCSPRunOk))

        self.onTrajStart("Traj1")

        print("Run trajectory loop : #" + str(self.schedule_run_cnt))

        sleep(2)
        self.onPuriOn()
        sleep(3)
        self.onUVCOn()        

        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        self.SendLedCmd(self.robot_mode)

        self.bRunTrajNavi = True

        print("send led cmd  : RunTrajLoop ==> robot_mode :" + self.robot_mode)

        sleep(5)

        self.onPumpOn()
        print("On Pump...")

        sleep(2)

        self.onSolValveOn()
        print("On Solvalve...")

    def RunTrajLoopFinal(self):
        global AutoParkNum
        # global bCSPRunOk

        self.schedule_run_cnt += 1

        print("start RunTrajLoop....")

        if self.bCSParking == True:
            AutoParkNum = 2
            self.RunCSAutoparking(False)

            # bCSPRunOk = False

            # while True:
            #     if bCSPRunOk == True:
            #         bCSPRunOk = False
            #         print("break while in RunTrajLoop")
            #         break

            #     sleep(0.1)
            
            self.bCSParking = False


        sleep(2)

        self.onPuriOn()
        
        sleep(3)

        self.onUVCOn()        

        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        self.SendLedCmd(self.robot_mode)

        print("Stop Moving ... Done final robot mode...")


    def RunCSParking(self):
        global AutoParkNum

        if self.bCSParking == False:
            self.bCSParking = True

            AutoParkNum = 1

            self.RunCSAutoparking(True)

       
        print("Run auto parking : #" + str(self.schedule_run_cnt))

    def RunCSParkingPumpOff(self):
        global AutoParkNum

        if self.bCSParking == False:
            self.bCSParking = True

            AutoParkNum = 1

            self.RunCSAutoparkingPumpOff(True)

       
        print("Run auto parking : #" + str(self.schedule_run_cnt))


    def RunCSAutoparking(self, bRunWaypoint):
        global AutoParkNum
        # global bCSPRunOk

        # bCSPRunOk = False

        if AutoParkNum == 1:
            print("Run auto parking process [1]")
            # if self.bRunNaviWP is True:
            #     self.onWPStop()

            self.ClearCostMap()
            sleep(2)

            if self.bRunTrajNavi is True:
                print("Stop Traj1...")
                self.onTrajStop("Traj1")

            if bRunWaypoint == True:

                fpos = "point1"

                self.ClearCostMap()
                sleep(2)

                print("Goto waypoint : " + fpos)

                self.naviStatus = NavigationControlStatus.IDLING

                self.gotoWayPoint(fpos)

                self.SendLedCmd("Navigation_Normal")
               
                runcnt = 0
                
                while True:
                    if self.bEStop == True:
        
                        sleep(0.1)
                        
                        continue

                    if self.bEStop == False and self.bprevEStop == True:
                        
                        self.ClearCostMap()
                        # os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                        
                        sleep(3)
                        
                        self.bprevEStop = False

                        self.gotoWayPoint(fpos)

                        messStr = "Restart " + fpos

                        print(messStr)

                        self.naviStatus = NavigationControlStatus.IDLING

                    if self.naviStatus == NavigationControlStatus.COMPLETED:

                        messStr = "Arrived at " + fpos
                        print(messStr)
                        
                        break

                    if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                        runcnt = runcnt + 1

                        if runcnt <= self.RepCnt:
                            self.ClearCostMap()
                            # os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                            self.gotoWayPoint(fpos)

                            messStr = "Restart " + fpos + " : " + str(runcnt)
                            print(messStr)

                            sleep(2)

                        else:
                            runcnt = 0
                            messStr = "Error(WARNNRGTO) " + fpos
                            print(messStr)

                        break

                    if self.naviStatus == NavigationControlStatus.ERRGTGF:
                        messStr = "Error(ERRGTGF) " + fpos
                        print(messStr)          

                    sleep(0.1)

                self.ClearCostMap()
                sleep(3)

            self.onUVCOff()
            sleep(3)

            self.onUVCOff()
            sleep(3)

            self.onPuriOff()
            sleep(3)

            self.onPuriOff()
            
            self.client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            self.client.wait_for_server()

            goal = ChargingGoal()
            print("goal : ")
            print(goal)
            self.client.send_goal(goal)

            print("Send goal for parking charging station.")
            self.robot_mode = "Auto_ParkingCS"
            self.SendLedCmd("Auto_ParkingCS")
            
        elif AutoParkNum == 2:
            print("Run auto parking process [2]")
            
            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)

            sleep(5)

            print(" Run auto parking process [3] : go forawrd")

            twist = Twist()
            twist.linear.x = -0.06; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(1)

            twist = Twist()
            twist.linear.x = -0.06; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(9)

            print(" Run auto parking process [4] : go forawrd")

            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            print("Cancel parking charge station.")
            self.robot_mode = "ReadyDone"
            self.SendLedCmd("ReadyDone")

            sleep(1)

            self.InitPosition()

            sleep(3)

            self.onUVCOn()
            sleep(3)

            self.onPuriOn()
            sleep(3)

            print("Done Auto Parking Cancel process...")

        # bCSPRunOk = True
        print("Exit Thread_RunCSAutoparking")


    def RunCSAutoparkingPumpOff(self, bRunWaypoint):
        global AutoParkNum
        # global bCSPRunOk

        # bCSPRunOk = False

        if AutoParkNum == 1:
            print("Run auto parking process [1]")
            # if self.bRunNaviWP is True:
            #     self.onWPStop()

            self.ClearCostMap()
            sleep(2)

            if self.bRunTrajNavi is True:
                print("Stop Traj1...")
                self.onTrajStop("Traj1")

            if bRunWaypoint == True:

                fpos = "point1"

                self.ClearCostMap()
                sleep(2)

                print("Goto waypoint : " + fpos)

                self.naviStatus = NavigationControlStatus.IDLING

                self.gotoWayPoint(fpos)

                self.SendLedCmd("Navigation_Normal")
               
                runcnt = 0
                
                while True:
                    if self.bEStop == True:
        
                        sleep(0.1)
                        
                        continue

                    if self.bEStop == False and self.bprevEStop == True:
                        
                        self.ClearCostMap()
                        # os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                        
                        sleep(3)
                        
                        self.bprevEStop = False

                        self.gotoWayPoint(fpos)

                        messStr = "Restart " + fpos

                        print(messStr)

                        self.naviStatus = NavigationControlStatus.IDLING

                    if self.naviStatus == NavigationControlStatus.COMPLETED:

                        messStr = "Arrived at " + fpos
                        print(messStr)
                        
                        break

                    if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                        runcnt = runcnt + 1

                        if runcnt <= self.RepCnt:
                            self.ClearCostMap()
                            # os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                            self.gotoWayPoint(fpos)

                            messStr = "Restart " + fpos + " : " + str(runcnt)
                            print(messStr)

                            sleep(2)

                        else:
                            runcnt = 0
                            messStr = "Error(WARNNRGTO) " + fpos
                            print(messStr)

                        break

                    if self.naviStatus == NavigationControlStatus.ERRGTGF:
                        messStr = "Error(ERRGTGF) " + fpos
                        print(messStr)          

                    sleep(0.1)

                self.ClearCostMap()
                sleep(2)

            self.onSolValveOff()
            print("Off Solvalve...")
            sleep(2)

            self.onPumpOff()
            print("Off Pump...")
            sleep(2)

            self.onUVCOff()
            sleep(3)

            self.onPuriOff()
            sleep(3)
            
            self.client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            self.client.wait_for_server()

            goal = ChargingGoal()
            print("goal : ")
            print(goal)
            self.client.send_goal(goal)

            print("Send goal for parking charging station.")
            self.robot_mode = "Auto_ParkingCS"
            self.SendLedCmd("Auto_ParkingCS")
            
        elif AutoParkNum == 2:
            print("Run auto parking process [2]")
            
            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)

            sleep(5)

            print(" Run auto parking process [3] : go forawrd")

            twist = Twist()
            twist.linear.x = -0.06; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(1)

            twist = Twist()
            twist.linear.x = -0.06; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(9)

            print(" Run auto parking process [4] : go forawrd")

            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            print("Cancel parking charge station.")
            self.robot_mode = "ReadyDone"
            self.SendLedCmd("ReadyDone")

            sleep(1)

            self.InitPosition()

            sleep(3)

            self.onUVCOn()
            sleep(3)

            self.onPuriOn()
            sleep(3)

            print("Done Auto Parking Cancel process...")

        # bCSPRunOk = True
        print("Exit Thread_RunCSAutoparking")

class MainRun:
    def __init__(self):
        self.naviStatus = NavigationControlStatus.IDLING

        self.bAirPuriRun = False
        self.bUVCRun = False
        self.bPumpRun = False
        self.bSolRun = False
        self.bRoscoreRun = False
        self.bBringUpRun = False
        self.bGridmapSLAMRun = False
        self.bCartographerSLAMRun = False
        self.bTeleOPRun = False
        self.bmotionCtrl = False
        self.bNaviRVIZRun = False
        self.bSLAMRVIZRun = False
        self.bMultiLRFRun = False
        self.bWPCRun = False
        self.bNaviRun = False
        self.bMakeWPRun = False
        self.bCSParking = False
        self.bOPRun = False
        self.bReadyDone = False
        self.bLowbatPark = False

        self.AirPurifierVal = 0

        self.FrontColor = 0xffffff
        self.RearColor = 0xffffff

        self.Bat_AvgVoltage = 0
        self.Bat_AvgCurrent = 0
        self.Bat_AvgSOC = 0
        
        self.batSVCnt = 0
        self.bLowBat = False
        self.bViewSonarData = False

        self.VSDCnt = 0
        self.IMUVCnt = 0

        self.csStatus = ""

        self.trajmode = NavigationControl.NONE
        self.csmode = False

        self.bRunNaviWP = False
        self.bRunTrajNavi = False
        self.bEStop = False

        self.bprevEStop = False
        self.bEStop_Status = 0
        self.bReady = False
        self.bRunWebManager = False
        self.bGetCurPos = False
        self.bUseSonar = False
        self.battery_cnt = 2
        self.battery_low = 11.0
        self.battery_middle = 80.0
        self.battery_full = 92.0
        self.hysteresis_val = 3.0
        self.hysteresis_low = 0.0
        self.hysteresis_middle = 0.0
        self.hysteresis_high = 0.0
        self.battery_mode = "None"
        self.robot_mode = "None"

        self.pm10 = 40.0
        self.pm25 = 35.0

        self.bStartOK = False
        self.bLedMCtrl = False

        self.bAutoPF = False
        self.bBatFirstRun = True

        self.bCAMView = False

        self.bLauchCAMTopic = False

        self.bBatteryFull = False

        self.rosserial_proc = []
        self.roscore_proc = []
        self.rosbringup_proc = []
        self.teleop_proc = []
        self.gridmap_proc = []
        self.cartographer_proc = []
        self.slamcarto_proc = []
        self.wpctrl_proc = []
        self.navirviz_proc = []
        self.slam_proc = []
        self.lrfmultimerge_proc = []
        self.makewaypoint_proc = []
        self.navigation_proc = []
        self.oprobot_proc = []
        self.autocharge_proc = []

        self.webcam = []

        self.schedule_th = []

        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0

        self.schedule_run_cnt = 0

        self.bRunAutoParking = False

        self.autopark_th = []

        self.prevbattery_mode = "None"
        self.bRunWaypoint = False

        self.Bat1_Info = BatteryInformationMsgs()
        self.Bat2_Info = BatteryInformationMsgs()

        try :
            with open(yaml_fname) as f:
                wp_list = yaml.safe_load(f)
                
                wps = wp_list['waypoints']
                print("waypoints")
                for n in wps:
                    print("name : " + n["name"])
                    self.naviWPName_CB.addItem(n["name"])

                print("trajectories")
                trajs = wp_list['trajectories']
                for t in trajs:
                    print("name : " + t["name"])
                    self.navitrajName_CB.addItem(t["name"])

        except :
            bWPFileLoadOK = False
            
        finally:
            bWPFileLoadOK = True

        # os.system("python3 /home/zetabank/bin/target_term -set 18")
        # print("Run terminal windows.")

        sleep(2)

        # cmd = "gnome-terminal --tab --title=\"robot_serial\" -e roslaunch zetabank_bringup zetabank_robot_serial.launch"
        # # cmd = "python3 ~/bin/target_term -run 1 roslaunch zetabank_bringup zetabank_robot_serial.launch"
        # self.rosserial_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

        # print("Run roslaunch zetabank_robot_serial.launch")

        sleep(2)

        self.naviCtrl_pub = rospy.Publisher("/navi_ctrl", NavigationControl, queue_size = 10)
        self.power_control_pub = rospy.Publisher("/power_control_command", PowerControlMsgs, queue_size=10)
        self.air_purifier_pub = rospy.Publisher("/purifier_control_command", UInt16, queue_size=10)
        self.ledcommand_pub = rospy.Publisher("/led_control_command", UInt64, queue_size=10)
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.air_cancel_pub = rospy.Publisher("/air_condition_cancel", Bool, queue_size=10)
        self.floor_cancel_pub = rospy.Publisher("/floor_cleaning_cancel", Bool, queue_size=10)

        # self.navictrl_status_sub = rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)
        self.bat_sub = rospy.Subscriber("/battery", BatteryInformationMsgs, self.CallbackBatteryStatus)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.recv_IMU)
        self.sonar_sub = rospy.Subscriber("/sonar", Float32MultiArray, self.recv_Sonar)
        self.charge_state_sub = rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)
        # self.estop_sub = rospy.Subscriber("/estop", Bool, self.CallEStop)
        self.robotpos_sub = rospy.Subscriber("/robot_pose", Pose, self.CallCurPoseInfo)

        self.robotpos_sub = rospy.Subscriber("/set_camview", Bool, self.CallSetCAMView)

        # self.bat_sub = rospy.Subscriber("/battery", BatteryInformationMsgs, self.CallbackBatteryStatus)
        # self.charge_state_sub = rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)
        # self.charge_state_nuc_sub = rospy.Subscriber("/autocharge_state_NUC", UInt8, self.CallbackCSStatusNUC)
        
        print("Complete initialization...")

        sleep(2)

        self.bStartOK = True

        self.robot_mode = "None"
        self.SendLedCmd(self.robot_mode)

        if autorun == True:
            # sleep(2)

            # th_schedule = Thread_RunSchedule()

            # ready = threading.Thread(target= th_schedule.run, args=())
            # ready = threading.Thread(target= self.RunPackage, args=())
            # ready.daemon = True
            # ready.start()

            self.bReadyDone = True
            print("Done ready...")
            self.robot_mode = "ReadyDone"
            self.SendLedCmd(self.robot_mode)

            self.SetStartPosition()
            sleep(5)

            self.robot_mode = "Navigation_Normal"
            self.SendLedCmd(self.robot_mode)
          
            sleep(5)

            self.onRunProc()

            sleep(5)

            self.onPuriOn()

            sleep(3)

            self.onUVCOn()
            sleep(3)

            self.changeTrajMode(True)
            sleep(3)
            self.changeUseSonar(True)
            sleep(3)

            self.onTrajStart("Traj1")


    def runAutoParkThread(self, val):

        t = threading.Thread(target= self.RunCSAutoparking, args=(val, True))
        t.daemon = True
        t.start()

        self.bRunAutoParking = True


    def RunPackage(self):
        if self.bBringUpRun == False:
            self.onBringup()

            sleep(9)
        print("Run Percent : 27%")

        if self.bMultiLRFRun == False:
            self.onMultiLRF()
            sleep(4)

        print("Run Percent : 35%")

        if self.bOPRun == False:
            self.onOperation()
            sleep(5)

        print("Run Percent : 47%")

        if self.bNaviRun == False:
            self.onNavigation()
            sleep(8)

        print("Run Percent : 70%")

        # if self.bNaviRVIZRun == False:
        #     self.onNaviRVIZn()
        #     sleep(8)


        # if self.bWPCRun == False:
        #     self.onWPCtrl()
        #     if self.bRunTrajNavi == True:
        #         self.onTrajStart("Traj1")
        #         sleep(6)

        self.bReadyDone = True
        print("Done ready...")
        self.robot_mode = "ReadyDone"
        self.SendLedCmd(self.robot_mode)

        # print("Run Percent : 85%")

        self.SetStartPosition()
        sleep(5)

        
        # cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
        # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
        # os.system(cmdstr)
        # sleep(5)
        # cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
        # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
        # os.system(cmdstr)
        # sleep(5)

        print("Run Percent : 100%")
        print("Done ready...")

        sleep(10)

        self.onRunProc()

        sleep(5)

        self.onPuriOn()

        sleep(3)

        self.onUVCOn()
        sleep(3)

        self.changeTrajMode(True)
        sleep(3)
        self.changeUseSonar(True)
        sleep(3)


        self.onTrajStart("Traj1")



    def ExitPackage(self, num):
        if self.bRunTrajNavi == True:
            self.onTrajStart("Traj1")            
            sleep(4)
            
        print("Run Percent : 30%")

        self.bReadyDone = False        
        print("Exit all ready...")
        self.robot_mode = "None"
        self.SendLedCmd(self.robot_mode)

        self.onWPCtrl()
        sleep(4)
        print("Run Percent : 15%")

        # self.onNaviRVIZBtn()
        # sleep(4)
        # print("Run Percent : 30%")

        self.onNavigation()
        sleep(4)
        print("Run Percent : 45%")

        self.onOperation()
        sleep(5)
        print("Run Percent : 65%")

        self.onMultiLRF()
        sleep(3)
        print("Run Percent : 75%")

        self.onBringup()
        sleep(6)
        print("Run Percent : 100%")

        print("Done init...")
        

    def write_log(self):
        global logfp
        global logdata

        today_str = strftime('%Y_%m_%d', localtime(time()))

        logfile_dir = rootdir + "/log"
        logfile_name = logfile_dir + "/safety_func_log_" + robotid + "_"+ today_str + ".txt"

        logfp = open(logfile_name, 'a')
        logdata["now_time"] = strftime('%Y_%m_%d', localtime(time())) + " " + str(localtime(time()).tm_hour) + ":" + str(localtime(time()).tm_min) + ":" + str(localtime(time()).tm_sec)
        logfp.write('%s SFN=%03d WRegion=%03d WMode=%03d RMode=%03d RStatus=%s X=%f Y=%f Z=%f\n' % (logdata["now_time"], logdata["safety_info"], logdata["working_region"],
                    logdata["working_mode"], logdata["robot_mode"], logdata["robot_status"], logdata["pos_x"], logdata["pos_y"], logdata["pos_z"]))
        logfp.close()            
            
    def make_logfile(self):
        global blogFileReady
        global logfp
        
        today_str = strftime('%Y_%m_%d', localtime(time()))

        logfile_dir = rootdir + "/log"
        logfile_name = logfile_dir + "/safety_func_log_" + today_str + ".txt"

        if os.path.isfile(logfile_name):
            print("Ready log file...")
        else:
            if not os.path.exists(logfile_dir):
                os.makedirs(logfile_dir)

            logfp = open(logfile_name, 'w')
            data = "time\tSafetyInfo\tworkingRegion\tWorkingMode\tRobotMode\tRobotStatus\tRobotPosX\tRobotPosY\tRobotPosZ"
            logfp.write(data)
            logfp.close()    

    def CallbackCSStatusNUC(self, msg):
        if self.bStartOK == False:
            return

        csnuc_state = msg.data

        if self.bLowbatPark == True and csnuc_state == 5:
            self.bLowbatPark = False     

        if csnuc_state == 6:
            self.bBatteryFull = True

            if self.robot_mode == "Charging":
                self.robot_mode = "FullCharging"
                self.battery_mode = "Charging-full-bat"
                
                self.robot_mode = "Navigation_Normal"

                self.SendLedCmd(self.battery_mode)
                print("Change robot mode to FullCharging")

        else:
            self.bBatteryFull = False
            

    def CallSetCAMView(self, msg):
        setVal = msg.data

        if setVal == True:
            self.bLauchCAMTopic = True

            cmdstr = "gnome-terminal -- roslaunch usb_cam usb_cam.launch  -1 "
            os.system(cmdstr)
            print("Launch cam live topic...")
            
            # cmd = "python3 ~/bin/target_term -run 18 roslaunch usb_cam usb_cam.launch"
            # try:
            #     self.launchcamtopic_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

            #     print("Launch cam live topic...")
            # except:
            #     print("Failed to run navigation process...")

        else:
            self.bLauchCAMTopic = False

            cmdstr = "gnome-terminal -- rosnode kill /usb_cam  -1 "
            os.system(cmdstr)

            sleep(2.0)

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam")     
            
            # sleep(2.0)
            
            # try:            
            #     self.launchcamtopic_proc.kill()
            # except:
            #     print("usb cam nodes have died...") 

            print("Stop cam live topic...")


    def onInitRobotHeadingAngle(self):
        self.bGetCurPos = True

        heading_angle = self.headingangle_lineEdit.text()
        ha_val = float(heading_angle)
        print("robot heading angle : " + str(ha_val))

        sleep(1)

        publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        #Creating the message with the type PoseWithCovarianceStamped
        # rospy.loginfo("This node sets the robot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
        start_pos = PoseWithCovarianceStamped()
        #filling header with relevant information
        start_pos.header.frame_id = "map"
        start_pos.header.stamp = rospy.Time.now()
        #filling payload with relevant information gathered from subscribing
        # to initialpose topic published by RVIZ via rostopic echo initialpose
        start_pos.pose.pose.position.x = self.position_x
        start_pos.pose.pose.position.y = self.position_y
        start_pos.pose.pose.position.z = self.position_z

        quaternion = quaternion_from_euler(0.0, 0.0, ha_val)

        start_pos.pose.pose.orientation.x = quaternion[0]
        start_pos.pose.pose.orientation.y = quaternion[1]
        start_pos.pose.pose.orientation.z = quaternion[2]
        start_pos.pose.pose.orientation.w = quaternion[3]

        start_pos.pose.covariance[0] = 0.25
        start_pos.pose.covariance[7] = 0.25
        start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[35] = 0.06853891945200942

        rospy.loginfo(start_pos)
        rospy.sleep(1)
        publisher.publish(start_pos)
        rospy.sleep(1)

        viewstr = "Set robot heading angle : " + str(ha_val)
        print(viewstr)


    def CallCurPoseInfo(self, msgs):
        global logdata

        if self.bGetCurPos == True:
            self.position_x = msgs.position.x
            self.position_y = msgs.position.y
            self.position_z = msgs.position.z

        else:
            pass

        logdata["pos_x"] = msgs.position.x
        logdata["pos_y"] = msgs.position.y
        logdata["pos_z"] = msgs.position.z


    def onWebManagerLaunch(self):
        if self.bRunWebManager is False:

            cmdstr = "gnome-terminal -- roslaunch rosboard rosboard.launch  -1 "
            os.system(cmdstr)

            print("Run roslaunch rosboard.launch")

            sleep(5)

            cmdstr = "gnome-terminal -- rosrun rosboard rosboard_node  -1 "
            os.system(cmdstr)

            sleep(3)

            print("Run rosrun rosboard_node")

            self.bRunWebManager = True

            # cmd1 = "python3 ~/bin/target_term -run 16 roslaunch rosboard rosboard.launch"
            # try:
            #     self.rosboard_proc = subprocess.Popen(args=cmd1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

            #     print("Run roslaunch rosboard.launch")

            #     sleep(5)

            #     cmd2 = "python3 ~/bin/target_term -run 17 rosrun rosboard rosboard_node"
            #     try:
            #         self.rosboardnode_proc = subprocess.Popen(args=cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

            #         sleep(3)

            #         print("Run rosrun rosboard_node")
        
            #         self.bRunWebManager = True

            #     except:
            #         print("Failed to launch rosboard_node...")
            #         self.bRunWebManager = False
            # except:
            #     print("Failed to launch rosboard...")
            #     self.bRunWebManager = False

        else:
            self.bRunWebManager = False

            cmdstr = "gnome-terminal -- rosnode kill /rosbridge_websocket  -1 "
            os.system(cmdstr)
            sleep(3.0)        

            cmdstr = "gnome-terminal -- rosnode kill /web_video_server  -1 "
            os.system(cmdstr)
            sleep(3.0)        

            cmdstr = "gnome-terminal -- rosnode kill /ros_api  -1 "
            os.system(cmdstr)
            sleep(3.0) 

            cmdstr = "gnome-terminal -- rosnode kill /ros_pose_publisher  -1 "
            os.system(cmdstr)
            sleep(3.0) 

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /rosbridge_websocket")       
            # sleep(1.0)        

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /web_video_server")       
            # sleep(1.0)        

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /ros_api")       
            # sleep(1.0)        

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /ros_pose_publisher")       
            # sleep(2.0)        

            try:            
                self.rosboard_proc.kill()

                print("Exit roslaunch rosboard.launch")
            except:
                print("rosboard_proc process has died...") 

            cmdstr = "gnome-terminal -- rosnode kill /rosboard_node  -1 "
            os.system(cmdstr)
            sleep(2.0) 

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /rosboard_node")       

            # sleep(2.0)        

            try:            
                self.rosboardnode_proc.kill()
                print("Exit rosrun rosboard_node")

            except:
                print("rosboardnode_proc process has died...") 



    def InitPosition(self):

        start_pose_param = rospy.get_param("start_pose")

        publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.loginfo("This node sets the robot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
        start_pos = PoseWithCovarianceStamped()
        #filling header with relevant information
        start_pos.header.frame_id = "map"
        start_pos.header.stamp = rospy.Time.now()
        #filling payload with relevant information gathered from subscribing
        # to initialpose topic published by RVIZ via rostopic echo initialpose
        start_pos.pose.pose.position.x = start_pose_param['position_x']
        start_pos.pose.pose.position.y = start_pose_param['position_y']
        start_pos.pose.pose.position.z = 1.0

        start_pos.pose.pose.orientation.x = 0.0
        start_pos.pose.pose.orientation.y = 0.0
        start_pos.pose.pose.orientation.z = start_pose_param['orientation_z']
        start_pos.pose.pose.orientation.w = start_pose_param['orientation_w']

        start_pos.pose.covariance[0] = 0.25
        start_pos.pose.covariance[7] = 0.25
        start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[35] = 0.06853891945200942

        publisher.publish(start_pos)

        sleep(2)

        publisher.publish(start_pos)

        print("Set the start position...")


    def SetStartPosition(self):

        start_pose_param = rospy.get_param("start_pose")

        publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.loginfo("This node sets the robot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
        start_pos = PoseWithCovarianceStamped()
        #filling header with relevant information
        start_pos.header.frame_id = "map"
        start_pos.header.stamp = rospy.Time.now()
        #filling payload with relevant information gathered from subscribing
        # to initialpose topic published by RVIZ via rostopic echo initialpose
        start_pos.pose.pose.position.x = start_pose_param['position_x']
        start_pos.pose.pose.position.y = start_pose_param['position_y']
        start_pos.pose.pose.position.z = 1.0

        start_pos.pose.pose.orientation.x = 0.0
        start_pos.pose.pose.orientation.y = 0.0
        start_pos.pose.pose.orientation.z = start_pose_param['orientation_z']
        start_pos.pose.pose.orientation.w = start_pose_param['orientation_w']

        start_pos.pose.covariance[0] = 0.25
        start_pos.pose.covariance[7] = 0.25
        start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        start_pos.pose.covariance[35] = 0.06853891945200942

        publisher.publish(start_pos)

        sleep(2)

        publisher.publish(start_pos)

        print("Set the start position...")
            
        
    def onRunProc(self):
        global bRunSchedule

        if bRunSchedule == False:
            bRunSchedule = True

            if self.trajmode == NavigationControl.GOAL:
                self.trajmode = NavigationControl.LOOP

            th_schedule = Thread_RunSchedule()

            self.schedule_th = threading.Thread(target=th_schedule.run, args=())

            # self.schedule_th = threading.Thread(target= self.RunSchedule, args=())
            self.schedule_th.daemon = True
            self.schedule_th.start()


            # self.schedule_th = Thread_RunSchedule(self)
            # self.schedule_th.start()

            # print("Run schedule...")
            print("Run onRunProc...")


        else:

            self.schedule_th.StopSchedule()

            self.trajmode = NavigationControl.GOAL
            self.csmode = False

            nc = NavigationControl()
            nc.control = NavigationControl.STOP
            nc.goal_name = "Traj1"
            nc.mode = self.trajmode
            self.naviCtrl_pub.publish(nc)

            print("Stop onRunProc...")

            
    def RunTrajLoopNum(self, num):
        global AutoParkNum
        global bCSPRunOk

        self.schedule_run_cnt += 1

        if self.bCSParking == True:
            AutoParkNum = 2
            self.runAutoParkThread(False)

            bCSPRunOk = False

            while True:
                if bCSPRunOk == True:
                    bCSPRunOk = False
                    break

                sleep(0.1)
            
            self.bCSParking = False

            # self.onParkingChargingStationBtn()
            
        self.RunTrajectories(num)

        print("Run trajectory loop : #" + str(self.schedule_run_cnt) + " Traj. Num :" + str(num))

    def RunTrajLoop(self):
        global AutoParkNum
        global bCSPRunOk

        self.schedule_run_cnt += 1

        if self.bCSParking == True:
            AutoParkNum = 2
            self.runAutoParkThread(False)

            bCSPRunOk = False

            while True:
                if bCSPRunOk == True:
                    bCSPRunOk = False
                    print("break while in RunTrajLoop")
                    break

                sleep(0.1)
            
            self.bCSParking = False

        print("start onTrajStart ==> bCSPRunOk :" + str(bCSPRunOk))

        self.onTrajStart("Traj1")

        print("Run trajectory loop : #" + str(self.schedule_run_cnt))

        self.onPuriOn()
        sleep(3)
        self.onUVCOn()

        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        self.SendLedCmd(self.robot_mode)

        print("send led cmd  : RunTrajLoop ==> robot_mode :" + self.robot_mode)
        
    def RunCSParking(self):
        global AutoParkNum

        if self.bCSParking == False:
            self.bCSParking = True

            AutoParkNum = 1

            # self.runAutoParkThread(True)

       
        print("Run auto parking : #" + str(self.schedule_run_cnt))


    def RunCSAutoparking(self, AutoParkNum, bRunWaypoint):
        # global AutoParkNum
        global bCSPRunOk

        bCSPRunOk = False

        if AutoParkNum == 1:
            print("Run auto parking process [1]")
            # if self.bRunNaviWP is True:
            #     self.onWPStop()

            self.ClearCostMap()
            sleep(2)

            if self.bRunTrajNavi is True:
                self.onTrajStop("Traj1")

            if bRunWaypoint == True:

                fpos = "point1"

                self.ClearCostMap()
                sleep(2)

                print("Goto waypoint : " + fpos)

                self.gotoWayPoint(fpos)

                self.SendLedCmd("Navigation_Normal")
               
                runcnt = 0
                
                while True:
                    if self.bEStop == True:
        
                        sleep(0.1)
                        
                        continue

                    if self.bEStop == False and self.bprevEStop == True:
                        cmdstr = "gnome-terminal -- rosservice call /move_base/clear_costmaps \"{}\"  -1 "
                        os.system(cmdstr)

                        # os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                        
                        sleep(3)
                        
                        self.bprevEStop = False

                        self.gotoWayPoint(fpos)

                        messStr = "Restart " + fpos

                        print(messStr)

                        self.naviStatus = NavigationControlStatus.IDLING

                    if self.naviStatus == NavigationControlStatus.COMPLETED:

                        messStr = "Arrived at " + fpos
                        print(messStr)
                        
                        break

                    if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                        runcnt = runcnt + 1

                        if runcnt <= self.RepCnt:
                            cmdstr = "gnome-terminal -- rosservice call /move_base/clear_costmaps \"{}\"  -1 "
                            os.system(cmdstr)
                            # os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                            self.gotoWayPoint(fpos)

                            messStr = "Restart " + fpos + " : " + str(runcnt)
                            print(messStr)

                            sleep(2)

                        else:
                            runcnt = 0
                            messStr = "Error(WARNNRGTO) " + fpos
                            print(messStr)

                        break

                    if self.naviStatus == NavigationControlStatus.ERRGTGF:
                        messStr = "Error(ERRGTGF) " + fpos
                        print(messStr)          

                    sleep(0.1)

                self.ClearCostMap()
                sleep(2)

            self.setapuvc.emit(False)
            
            self.client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            self.client.wait_for_server()

            goal = ChargingGoal()
            print("goal : ")
            print(goal)
            self.client.send_goal(goal)

            print("Send goal for parking charging station.")
            self.robot_mode = "Auto_ParkingCS"
            self.SendLedCmd("Auto_ParkingCS")
            
        elif AutoParkNum == 2:
            print("Run auto parking process [2]")
            
            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)

            sleep(5)

            print(" Run auto parking process [3] : go forawrd")

            twist = Twist()
            twist.linear.x = -0.06; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            sleep(10)

            print(" Run auto parking process [4] : go forawrd")

            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            self.pub_twist.publish(twist)

            print("Cancel parking charge station.")
            self.robot_mode = "ReadyDone"
            self.SendLedCmd("ReadyDone")

            sleep(1)

            self.InitPosition()

            sleep(3)

            self.onUVCOn()
            sleep(3)

            self.onPuriOn()
            sleep(3)

            print("Done Auto Parking Cancel process...")

        bCSPRunOk = True
        print("Exit Thread_RunCSAutoparking")

        
    def CallEStop(self, msgs):
        global logdata

        if self.bStartOK == False:
            return

        self.bEStop = msgs.data
        self.bEStop_Status = self.bEStop

        if self.bEStop is True and self.bprevEStop == False:
            logdata["safety_info"] = 0x01
            logdata["robot_status"] = "EStop"
            self.write_log()

            self.robot_mode = "E-Stop"
            self.SendLedCmd(self.robot_mode)

            print("!! ESTOP !!")

            self.bprevEStop = True

            if self.bRunNaviWP is True:
                self.onWPStop()

            if self.bRunTrajNavi is True:
                self.onTrajStop("Traj1")

            if self.bCSParking is True:
                self.bCSParking = False
   
                print("Cancel parking charge station-->ESTOP.")

            self.robot_mode = "E-Stop"
            self.SendLedCmd(self.robot_mode)

        else:
            if self.bprevEStop is True and self.bEStop is False:
                self.bprevEStop = False

                self.robot_mode = "Normal"
                self.SendLedCmd(self.robot_mode)

                print("ESTOP ==> SendLedCmd, robot_mode : Normal")

                logdata["safety_info"] = 0x00
            

    def CallbackCSStatus(self, msgs):
        if self.bStartOK == False:
            return

        if self.bCSParking is True and (self.robot_mode != "Charging" or self.robot_mode != "FullCharging"):
            self.csStatus = msgs.data

            if self.csStatus == "contact" and self.robot_mode != "Charging":

                self.robot_mode = "Charging"
                self.bViewCmd = False
                self.prevbattery_mode = "None"
                self.bRunAutoParking = False

                print("CallbackCSStatus ==> prevbattery_mode:" + self.prevbattery_mode + " battery_mode:" + self.battery_mode)

    def RunRealsenView(self):
        cmdstr = "gnome-terminal -- realsense-viewer  -1 "
        os.system(cmdstr)
        # os.system("python3 ~/bin/target_term -run 10 realsense-viewer")

    def recv_HeadingAngle(self, msgs):
        self.angle_lcdNumber.display(msgs.data)


    def ClearCostMap(self):
        cmdstr = "gnome-terminal -- rosservice call /move_base/clear_costmaps \"{}\"  -1 "
        os.system(cmdstr)
        # os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")


    def onWPStart(self, name):
        global logdata

        self.ClearCostMap()

        sleep(2)

        nc = NavigationControl()
        nc.control = NavigationControl.START        
        nc.goal_name = name
        nc.mode = NavigationControl.GOAL
        self.naviCtrl_pub.publish(nc)
        mstr = "[WP] Start way point : " + self.WPNameCBStr
        print(mstr)

        self.bRunNaviWP = True
        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "Driving"

        
    def onWPStop(self, name):
        global logdata

        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = name
        nc.mode = NavigationControl.NONE
        self.naviCtrl_pub.publish(nc)
        mstr = "[WP] Stop way point : " + self.WPNameCBStr
        print(mstr)

        self.bRunNaviWP = False
        if self.bAirPuriRun is True:
            self.robot_mode = "Air_Purifier"
        else:
            self.robot_mode = "None"

        self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "None"


    def RunTrajectories(self, num):
        global logdata

        self.ClearCostMap()

        sleep(2)

        nc = NavigationControl()
        nc.control = NavigationControl.START

        if num == 1:
            nc.goal_name = "Traj1"
        elif num == 2:
            nc.goal_name = "Traj2"
        elif num == 3:
            nc.goal_name = "Traj3"
        elif num == 4:
            nc.goal_name = "Traj4"

        nc.mode = self.trajmode
        self.naviCtrl_pub.publish(nc)
        mstr = "Start trajector : " + nc.goal_name
        print(mstr)

        self.bRunTrajNavi = True        
        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "Driving"


    def onTrajStart(self, name):
        global logdata

        self.ClearCostMap()

        print("costmap : onTrajStart")

        sleep(2)

        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = name
        nc.mode = self.trajmode
        self.naviCtrl_pub.publish(nc)

        mstr = "Start trajector : " + name
        print(mstr)

        self.bRunTrajNavi = True        
        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        self.SendLedCmd(self.robot_mode)

        print("send led cmd  : onTrajStartBtn ==> robot_mode :" + self.robot_mode)

        logdata["robot_status"] = "Driving"

        
    def onTrajStop(self, name):
        global logdata

        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.mode = NavigationControl.NONE
        nc.goal_name = name
        self.naviCtrl_pub.publish(nc)
        mstr = "Stop trajector : " + name
        print(mstr)

        self.bRunTrajNavi = False
        if self.bAirPuriRun is True:
            self.robot_mode = "Air_Purifier"
        else:
            self.robot_mode = "None"

        self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "None"

    def onNavigation(self):
        if self.bNaviRun is False:
            self.bNaviRun = True

            if self.bReadyDone == True:
                self.bReadyDone = False

            cmdstr = "gnome-terminal -- source ~/catkin_ws/devel/setup.bash  -1 "
            os.system(cmdstr)
            # os.system("python3 ~/bin/target_term -run 10 source ~/catkin_ws/devel/setup.bash")

            sleep(2)

            cmd = "gnome-terminal --tab --title=\"navigation\" -c roslaunch zetabank_navigation normal_navigation.launch"
            # cmd = "python3 ~/bin/target_term -run 13 roslaunch zetabank_navigation normal_navigation.launch"
            try:
                self.navigation_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run navigation process...")

            print("Run roslaunch normal_navigation.launch")

        else:
            self.bNaviRun = False            

            cmdstr = "gnome-terminal -- rosnode kill /amcl -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /map_server -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /move_base -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /AMCL_particles -1 "
            os.system(cmdstr)
            sleep(3.0)

            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /amcl")       
            # sleep(0.2)      
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /map_server")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /move_base")            
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /AMCL_particles")            
            # sleep(2.0)
            
            try:            
                self.navigation_proc.kill()
            except:
                print("navigation process has died...")   

            sleep(2.0)

            print("Exit roslaunch normal_navigation.launch")

    def onMultiLRF(self):
        if self.bMultiLRFRun is False:
            self.bMultiLRFRun = True

            cmd = "gnome-terminal --tab --title=\"m_lrf\" -c roslaunch ira_laser_tools laserscan_multi_merger.launch"
            # cmd = "python3 ~/bin/target_term -run 11 roslaunch ira_laser_tools laserscan_multi_merger.launch"
            try:
                self.lrfmultimege_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run lrfmultimege process...")

            print("Run roslaunch laserscan_multi_merger.launch")
            
        else:
            self.bMultiLRFRun = False

            cmdstr = "gnome-terminal -- rosnode kill /laserscan_multi_merger -1 "
            os.system(cmdstr)
            
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /laserscan_multi_merger")       

            sleep(2.0)         

            try:            
                self.lrfmultimege_proc.kill()
            except:
                print("lrfmultimege process has died...")   

            print("Exit roslaunch laserscan_multi_merger.launch")


    def onWPCtrl(self):
        if self.bWPCRun is False:
            self.bWPCRun = True

            cmd = "gnome-terminal --tab --title=\"NaviWP\" -c roslaunch navi_waypoint navigationWayPoint.launch"
            # cmd = "python3 ~/bin/target_term -run 7 roslaunch navi_waypoint navigationWayPoint.launch"
            try:
                self.wpctrl_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run wpctrl process...")

            print("Run roslaunch navigationWayPoint.launch")
            
        else:
            self.bWPCRun = False

            cmdstr = "gnome-terminal -- rosnode kill /navigation_waypoints -1 "
            os.system(cmdstr)
                
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /navigation_waypoints")      

            sleep(2.0)         

            try:            
                self.wpctrl_proc.kill()
            except:
                print("wpctrl process has died...")          

            print("Stop roslaunch navigationWayPoint.launch")
            
    def changeUseSonar(self, state):
        if state == True:
            cmdstr = "gnome-terminal -- rostopic pub /set_sonar std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)

            self.bUseSonar = True
            print("Use Sonar Mode : True")
        else:
            # cmdstr = "gnome-terminal -- rostopic pub /set_sonar std_msgs/Bool \"data: false\" -1 "
            # os.system(cmdstr)

            self.bUseSonar = False
            print("Use Sonar Mode : False")

    def changeAutoPF(self, state):
        if state == True:
            self.bAutoPF = True
            print("Auto Purifier Mode : True")
        else:
            self.bAutoPF = False
            print("Auto Purifier Mode : False")
            
    def changeTrajMode(self, state):

        if state == True:
            self.trajmode = NavigationControl.LOOP
            print("Navi Waypoint Mode : Loop")
        else:
            self.trajmode = NavigationControl.GOAL
            print("Navi Waypoint Mode : Goal")

    def changeCSMode(self, state):
        if state == True:
            self.csmode = True
            print("C.S Mode : True")
        else:
            self.csmode = False
            print("C.S Mode : False")            


    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians

    def recv_Sonar(self, msgs):
        if self.bStartOK == False:
            return

        if self.bViewSonarData == True:

            self.VSDCnt += 1

            if self.VSDCnt >= 25:
                self.VSDCnt = 0
            
                rimu = Imu()
                rimu = msgs

    def recv_IMU(self, msgs):

        if self.bStartOK == False:
            return

        self.IMUVCnt += 1

        if self.IMUVCnt >= 40:
            self.IMUVCnt = 0

            rollx, pitchy, yawz = self.euler_from_quaternion(msgs.orientation.x, msgs.orientation.y, msgs.orientation.z, msgs.orientation.w)


    def onOperation(self):
        if self.bOPRun is False:  
            self.bOPRun = True    

            cmd = "gnome-terminal --tab --title=\"OP\" -c roslaunch zetabot_main operate_robot.launch"
            # cmd = "python3 ~/bin/target_term -run 14 roslaunch zetabot_main operate_robot.launch"
            try:
                self.oprobot_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run oprobot process...")

            print("Run roslaunch operater_robot.launch")

            sleep(0.5)

            cmd = "gnome-terminal --tab --title=\"AP\" -c roslaunch autocharge parkingcs.launch"
            # cmd = "python3 ~/bin/target_term -run 15 roslaunch autocharge parkingcs.launch"
            try:
                self.autocharge_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run parkingcs process...")

            print("Run roslaunch parkingcs.launch")

        else:
            self.bOPRun = False            

            cmdstr = "gnome-terminal -- rosnode kill /initial_pos_srv -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /battery_log -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /autocharge_act_srv -1 "
            os.system(cmdstr)
            sleep(3.0)

            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /initial_pos_srv")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /battery_log")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /autocharge_act_srv")
            # sleep(2.0)

            try:            
                self.oprobot_proc.kill()
            except:
                print("op robot process has died...")

            sleep(0.2)

            try:            
                self.autocharge_proc.kill()
            except:
                print("parkincs process has died...")

            sleep(1)

            print("Exit operater_robot.launch")
            

    def onBringup(self):
        if self.bBringUpRun is False:  
            self.bBringUpRun = True      

            cmd = "gnome-terminal --tab --title=\"bringup\" -c roslaunch zetabank_bringup zetabank_robot.launch"
            # cmd = "python3 ~/bin/target_term -run 3 roslaunch zetabank_bringup zetabank_robot.launch"
            try:
                self.rosbringup_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run rosbringup process...")

            print("Run roslaunch zetabank_robot.launch")
            
        else:
            self.bBringUpRun = False      

            cmdstr = "gnome-terminal -- rosnode kill /cam_1/realsense2_camera -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /cam_1/realsense2_camera_manager -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /cam_2/realsense2_camera -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /cam_2/realsense2_camera_manager -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /depthimage_to_laserscan_cam1 -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /depthimage_to_laserscan_cam2 -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /robot_state_publisher -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /sick_tim551_2050001 -1 "
            os.system(cmdstr)
            sleep(3.0)

            cmdstr = "gnome-terminal -- rosnode kill /zeta_mdrobot_BLDC_controller -1 "
            os.system(cmdstr)
            sleep(3.0)
           
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /cam_1/realsense2_camera")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /cam_1/realsense2_camera_manager")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /cam_2/realsense2_camera")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /cam_2/realsense2_camera_manager")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /depthimage_to_laserscan_cam1")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /depthimage_to_laserscan_cam2")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /robot_state_publisher")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /sick_tim551_2050001")
            # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /zeta_mdrobot_BLDC_controller")

            # sleep(2.0)

            try:            
                self.rosbringup_proc.kill()
            except:
                print("ros bringup process has died...")

            print("Exit zetabank_robot.launch")
            
    def setAirPurif(self, val):
        self.AirPurifierVal = val
        purifier_val = UInt16()
        purifier_val.data = self.AirPurifierVal
        self.air_purifier_pub.publish(purifier_val)

    
    def setAPVal(self):
        self.setAirPurif(self.airPurifier_dial.value())

    def purifier_control(self, val):
        self.AirPurifierVal = val
        purifier_val = UInt16()
        purifier_val.data = self.AirPurifierVal
    
        self.air_purifier_pub.publish(purifier_val)


    def onPuriOn(self):
        global air_result

        if self.bAirPuriRun is False:
            self.bAirPuriRun = True

            self.purifier_control(400)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "Air_Purifier"

            self.SendLedCmd(self.robot_mode)

    
    def onPuriOff(self):
        global air_result

        if self.bAirPuriRun is True:
            self.bAirPuriRun = False

            self.purifier_control(0)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            self.SendLedCmd(self.robot_mode)

    
    def onSolValveOn(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bSolRun == False:
            self.bSolRun = True
            sol_control_msg = PowerControlMsgs()
            sol_control_msg.port = 14
            sol_control_msg.state = True
            self.power_control_pub.publish(sol_control_msg)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "Sol"
            
            self.SendLedCmd(self.robot_mode)


    def onSolValveOff(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bSolRun == True:
            self.bSolRun = False
            sol_control_msg = PowerControlMsgs()
            sol_control_msg.port = 14
            sol_control_msg.state = False
            self.power_control_pub.publish(sol_control_msg)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            self.SendLedCmd(self.robot_mode)


    def onPumpOn(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bPumpRun == False:
            self.bPumpRun = True
            pump_control_msg = PowerControlMsgs()
            pump_control_msg.port = 5
            pump_control_msg.state = True
            self.power_control_pub.publish(pump_control_msg)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "Pump"

            self.SendLedCmd(self.robot_mode)

    def onPumpOff(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bPumpRun == True:
            self.bPumpRun = False
            pump_control_msg = PowerControlMsgs()
            pump_control_msg.port = 5
            pump_control_msg.state = False
            self.power_control_pub.publish(pump_control_msg)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            self.SendLedCmd(self.robot_mode)


    def onUVCOn(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bUVCRun == False:
            self.bUVCRun = True
            uvc_control_msg = PowerControlMsgs()
            uvc_control_msg.port = 15
            uvc_control_msg.state = True
            self.power_control_pub.publish(uvc_control_msg)
            
            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "UVC"

            self.SendLedCmd(self.robot_mode)


    def onUVCOff(self):
        # if self.bBringUpRun is False:  
        #     print("[Error] : The bringup node did not run.")

        # el
        if self.bUVCRun == True:
            self.bUVCRun = False
            uvc_control_msg = PowerControlMsgs()
            uvc_control_msg.port = 15
            uvc_control_msg.state = False
            self.power_control_pub.publish(uvc_control_msg)

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            self.SendLedCmd(self.robot_mode)


    def SendLedCmd(self, bat_mode):
        if self.bLedMCtrl is False:
            fm = led_control[bat_mode][0]
            fc = led_control[bat_mode][1]
            rm = led_control[bat_mode][2]
            rc = led_control[bat_mode][3]

            fm = led_mode[fm]
            fc = led_color[fc]
            rm = led_mode[rm]
            rc = led_color[rc]

            fcolor_cmd = UInt64()
            fcolor_cmd = ((((fm << 24) | fc) << 32) | (((rm << 24) | rc)))
            self.ledcommand_pub.publish(fcolor_cmd)
            sleep(1.0)
            self.ledcommand_pub.publish(fcolor_cmd)
            sleep(1.0)
            self.ledcommand_pub.publish(fcolor_cmd)

            print("Run SendLedCmd ==> mode:" + bat_mode)


    def CallbackBatteryStatus(self, msgs):

        if self.bStartOK == False:
            return        

        if msgs.id == 0x60:
            self.Bat1_Info = msgs
        else:
            self.Bat2_Info = msgs

        self.Bat_AvgVoltage = (self.Bat1_Info.voltage + self.Bat2_Info.voltage) / self.battery_cnt*1.0
        self.Bat_AvgCurrent = (self.Bat1_Info.current + self.Bat2_Info.current) / self.battery_cnt*1.0
        self.Bat_AvgSOC = (self.Bat1_Info.SOC + self.Bat2_Info.SOC) / self.battery_cnt*1.0
 
        self.batSVCnt += 1
        if(self.batSVCnt > 4):
            self.batSVCnt = 0

            if self.robot_mode == "Charging":
                try:
                    if self.Bat_AvgSOC <= (self.battery_low - self.hysteresis_low):
                        self.hysteresis_low = 0.0
                        self.battery_mode = "Charging-low-bat"
                        self.bLowBat = True
                    elif (self.battery_low + self.hysteresis_low) <= self.Bat_AvgSOC < (self.battery_middle - self.hysteresis_middle):
                        self.hysteresis_low = 0.0
                        self.hysteresis_middle = 0.0
                        self.battery_mode = "Charging-middle-bat"
                        self.bLowBat = False
                    elif (self.battery_middle + self.hysteresis_middle) <= self.Bat_AvgSOC< (self.battery_full - self.hysteresis_high):
                        self.hysteresis_middle = 0.0
                        self.hysteresis_high = 0.0
                        self.battery_mode = "Charging-high-bat"
                        self.bLowBat = False
                    elif self.battery_full <= self.Bat_AvgSOC:          
                        self.battery_mode = "Charging-full-bat"
                        self.bLowBat = False
                    else:
                        self.battery_mode = "None"            

                    if  self.prevbattery_mode != self.battery_mode:
                        self.SendLedCmd(self.battery_mode)
                        self.prevbattery_mode = self.battery_mode  
                        print("Charging [3]: change battery mode ==> SendLedCmd")
                except:
                    print("[Error] : Led control.")

            else:
                try:
                    if self.Bat_AvgSOC <= (self.battery_low - self.hysteresis_low):
                        self.hysteresis_low = 0.0
                        self.bLowBat = True
                        self.battery_mode = "Charging-low-bat"
                    elif (self.battery_low + self.hysteresis_low) <= self.Bat_AvgSOC < (self.battery_middle - self.hysteresis_middle):
                        self.hysteresis_low = 0.0
                        self.hysteresis_middle = 0.0
                        self.bLowBat = False
                        self.battery_mode = "Charging-middle-bat"
                    elif (self.battery_middle + self.hysteresis_middle) <= self.Bat_AvgSOC< (self.battery_full - self.hysteresis_high):
                        self.hysteresis_middle = 0.0
                        self.hysteresis_high = 0.0
                        self.battery_mode = "Charging-high-bat"
                        self.bLowBat = False
                    elif self.battery_full <= self.Bat_AvgSOC:            
                        self.bLowBat = False
                        self.battery_mode = "Charging-full-bat"
                    else:
                        self.battery_mode = "None"            

                except:
                    pass

            if self.robot_mode == "FullCharging" and self.prevbattery_mode != self.battery_mode:
                self.bLowBat = False
                self.prevbattery_mode = self.battery_mode    
                self.battery_mode = "Charging-full-bat"
                self.SendLedCmd(self.battery_mode)

                print("Full charging mode...")

    def CallbackRunNaviCtrlStatus(self, request):
        if self.bStartOK == False:
            return

        if self.bWPCRun is True:
            self.naviStatus = request.status

            if self.bEStop is False:
                mstr = request.status_description

            if self.naviStatus is NavigationControlStatus.COMPLETED:
                if self.bRunNaviWP is True:
                    self.bRunNaviWP = False
                    
                    self.robot_mode = "None"
                    self.SendLedCmd(self.robot_mode)

            elif self.naviStatus is NavigationControlStatus.TRAJCOMPLETED:
            
                if self.bRunTrajNavi is True:
                    self.bRunTrajNavi = False

                    self.robot_mode = "None"
                    self.SendLedCmd(self.robot_mode)

    def onExit(self):
        if self.bReady == True:
            print("Exit running ros packages...")
            self.ReadyBtn()

        print("Exit program...")

        self.SendLedCmd("None")

        cmdstr = "gnome-terminal -- rosnode kill /PWR -1 "
        os.system(cmdstr)
        sleep(3.0)

        cmdstr = "gnome-terminal -- rosnode kill /STM -1 "
        os.system(cmdstr)
        sleep(3.0)

        cmdstr = "gnome-terminal -- rosnode kill /stm_starter -1 "
        os.system(cmdstr)
        sleep(3.0)

        cmdstr = "gnome-terminal -- rosnode kill /AIR -1 "
        os.system(cmdstr)
        sleep(3.0)

        
        # os.system("python3 ~/bin/target_term -run 10 rosnode kill /PWR")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 10 rosnode kill /STM")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 10 rosnode kill /stm_starter")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 10 rosnode kill /AIR")
        
        sleep(2.0)

        try:            
            self.self.rosserial_proc.kill()
        except:
            print("ros serial process has died...")  

        cmdstr = "gnome-terminal -- rosnode kill /rosout -1 "
        os.system(cmdstr)
        sleep(3.0)

        # os.system("python3 ~/bin/target_term -run 10 rosnode kill /rosout")

        # sleep(1.0)        

        # os.system("python3 ~/bin/target_term -run 1 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 2 exit") 
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 3 exit") 
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 4 exit") 
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 5 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 6 exit") 
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 7 exit") 
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 8 exit") 
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 9 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 10 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 11 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 12 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 13 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 14 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 15 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 16 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 17 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 18 exit")

        self.navictrl_status_sub.unregister()
        self.bat_sub.unregister()
        self.imu_sub.unregister()
        self.sonar_sub.unregister()
        self.charge_state_sub.unregister()
        self.estop_sub.unregister()
        self.air_sub.unregister()
        self.robotpos_sub.unregister()

        self.close()


    def closeEvent(self, event):
        self.deleteLater()
        event.accept()
            

def main() :
    # rospy.init_node('ZBMRCS_node')

    run = MainRun()
    # run.start()

    while(True):
        rospy.spin()

        sleep(0.1)

    
    
        
if __name__ == "__main__":
    main()