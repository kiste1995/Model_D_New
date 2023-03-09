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

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt
from PyQt5 import QtCore
from PyQt5 import QtGui
from python_qt_binding import loadUi
from std_srvs.srv import Empty
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *

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
# from pynput.keyboard import Key, Controller

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

rospy.init_node('ZBMRGS_node')


rootdir = rospy.get_param('~root_dir')
mainui_fname = rootdir + "/scripts/ZBMRGS.ui"
amclui_fname = rootdir + "/scripts/GetAMCLPos.ui"
airinfoui_fname = rootdir + "/scripts/AirInfo.ui"
readyui_fname = rootdir + "/scripts/Ready.ui"

wpdir = rospy.get_param('~wp_dir')
# wpdir = args.wpdir
print('wpdir', wpdir)

robotid = rospy.get_param('~robot_id')

# robotid = args.robotid
print('robotid:', robotid)
# autorun = args.autostart
autorun = rospy.get_param('~auto_start')
print('autorun:', autorun)

# yaml_fname = wpdir +"/config/waypoint6.yaml"
yaml_fname = wpdir +"/config/waypoint_221122.yaml"

main_ui = uic.loadUiType(mainui_fname)[0]
amcl_ui = uic.loadUiType(amclui_fname)[0]
airinfo_ui = uic.loadUiType(airinfoui_fname)[0]
ready_ui = uic.loadUiType(readyui_fname)[0]

ICON_RED_LED = rootdir + "/scripts" + "/icons/red-led-on.png"
ICON_GRAY_LED = rootdir + "/scripts" + "/icons/gray-led-on.png"
ICON_BLUE_LED = rootdir + "/scripts" + "/icons/blue-led-on.png"
ICON_GREEN_LED = rootdir + "/scripts" + "/icons/green-led-on.png"
ICON_ORANGE_LED = rootdir + "/scripts" + "/icons/orange-led-on.png"
ICON_YELLOW_LED = rootdir + "/scripts" + "/icons/yellow-led-on.png"

navirviz_fname = "/home/zetabank/catkin_ws/src/zetabank_robot_control/zetabank_navigation/rviz/zetabank_nav.rviz"
slamrviz_fname = "/home/zetabank/catkin_ws/src/zetabank_robot_control/zetabank_slam/rviz/zetabank_slam.rviz"

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


class Thread_RunReady(QThread):
    runpackage = pyqtSignal(int)
    exitpackage = pyqtSignal(int)
    setrunstate = pyqtSignal(int)
    # readydone = pyqtSignal(int)    

    def __init__(self, parent = None):
        super(Thread_RunReady, self).__init__(parent)

        self.type = 1
        
    def run(self):
        if self.type == 1 :
            self.runpackage.emit(1)
            sleep(9)
            self.setrunstate.emit(27)

            self.runpackage.emit(2)
            sleep(4)
            self.setrunstate.emit(35)            

            self.runpackage.emit(3)
            sleep(5)
            self.setrunstate.emit(47)

            self.runpackage.emit(4)
            sleep(8)
            self.setrunstate.emit(70)

            self.runpackage.emit(5)
            sleep(8)
            self.setrunstate.emit(85)

            self.runpackage.emit(6)
            sleep(6)

            self.runpackage.emit(7)
            sleep(5)

            # self.runpackage.emit(8)
            # sleep(10)

            self.setrunstate.emit(100)

            # self.runpackage.emit(7)
            # sleep(2)

            print("Done ready...")

        else:
            self.exitpackage.emit(6)
            sleep(4)
            self.setrunstate.emit(15)

            self.exitpackage.emit(5)
            sleep(4)
            self.setrunstate.emit(30)
            
            self.exitpackage.emit(4)
            sleep(4)
            self.setrunstate.emit(45)
            
            self.exitpackage.emit(3)
            sleep(5)
            self.setrunstate.emit(65)
            
            self.exitpackage.emit(2)
            sleep(3)
            self.setrunstate.emit(75)
            
            self.exitpackage.emit(1)
            sleep(6)
            self.setrunstate.emit(100)
            
            print("Done init...")

    def setType(self, val):
        self.type = val

class AirInfoDialog(QDialog, airinfo_ui):

    def __init__(self, parent=None):
        QDialog.__init__(self, parent)
        self.setupUi(self)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

        self.aiexit_PB.clicked.connect(self.onExittBtn)
        
        rospy.Subscriber("/air", EnvironmentMsgs, self.CallAirInfo)

    def CallAirInfo(self, msgs):

        self.hour_lineEdit.setText(str(localtime(time()).tm_hour))
        self.minute_lineEdit.setText(str(localtime(time()).tm_min))
        self.second_lineEdit.setText(str(localtime(time()).tm_sec))
        self.finedust_lineEdit.setText(str(msgs.Dust_PM10_ugm3))
        self.ultrafinedust_lineEdit.setText(str(msgs.Dust_PM2_5_ugm3))
        self.co2_lineEdit.setText(str(msgs.CO2_ppm))
        self.formaldehyde_lineEdit.setText(str(msgs.HCHO_ugm3))
        self.co_lineEdit.setText(str(msgs.CO_ppm))
        self.no2_lineEdit.setText(str(msgs.NO2_ppm))
        self.radon_lineEdit.setText(str(msgs.Rn_Bqm3))
        self.organicompounds_lineEdit.setText(str(msgs.TVOCs_ugm3))
        self.temp_lineEdit.setText(str(msgs.temp_celcius))
        self.humidity_lineEdit.setText(str(msgs.hum_RHp))

    def onExittBtn(self):
        global open_airinfo_dlg

        open_airinfo_dlg = False

        self.close()


class GetAMCLPosDialog(QDialog, amcl_ui):
    set_item = pyqtSignal(int, str)

    def __init__(self, parent=None):
        QDialog.__init__(self, parent)
        self.setupUi(self)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

        self.amclpos_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.CallbackAMCLPos)

        self.getdata_PB.clicked.connect(self.onGetDataBtn)
        self.gap_exit_PB.clicked.connect(self.onExittBtn)

        self.set_item.connect(self.setTableItem)

    @pyqtSlot(int, str)
    def setTableItem(self, int, str):
        self.covval_tableWidget.setItem(int, 0, QTableWidgetItem(str))

    def CallbackAMCLPos(self, msg):
        px = msg.pose.pose.position.x
        self.posx_lineEdit.setText(str(px))
        py = msg.pose.pose.position.y
        self.posy_lineEdit.setText(str(py))
        pz = msg.pose.pose.position.z
        self.posz_lineEdit.setText(str(pz))

        print("px : %f py : %f pz: %f" %(px, py, pz))

        ox = msg.pose.pose.orientation.x
        self.orientx_lineEdit.setText(str(ox))
        oy = msg.pose.pose.orientation.y
        self.orienty_lineEdit.setText(str(oy))
        oz = msg.pose.pose.orientation.z
        self.orientz_lineEdit.setText(str(oz))
        ow = msg.pose.pose.orientation.w
        self.orientw_lineEdit.setText(str(ow))

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        ang = yaw*MATH_RAD2DEG
        self.orienta_lineEdit.setText(str(ang))

        cov_val = msg.pose.covariance
        print(cov_val)
        print(" cov len :" + str(len(cov_val)))

        for i in range(0, len(cov_val)):
            self.set_item.emit(i, str(cov_val[i]))

    def process_run(self, args, cwd = None, shell = False, kill_tree = True, timeout = -1, env = None):
        '''
        Run a command with a timeout after which it will be forcibly
        killed.
        '''
        class Alarm(Exception):
            pass

        def alarm_handler(signum, frame):
            raise Alarm

        out = ''
        err = ''

        p = Popen(args, shell = shell, cwd = cwd, stdout = PIPE, stderr = PIPE, env = env)
        if timeout != -1:
            signal(SIGALRM, alarm_handler)
            alarm(timeout)
        try:
            out, err = p.communicate()
            if timeout != -1:
                alarm(0)
        except Alarm:
            pids = [p.pid]
            if kill_tree:
                pids.extend(self.get_process_children(p.pid))
            for pid in pids:
                # process might have died before getting to this line
                # so wrap to avoid OSError: no such process
                try:
                    kill(pid, SIGKILL)
                except OSError:
                    pass
            return -9, out, err
            # return -9, '', ''
        return p.returncode, out, err

    def get_process_children(self, pid):
        p = Popen('ps --no-headers -o pid --ppid %d' % pid, shell = True, stdout = PIPE, stderr = PIPE)
        out, err = p.communicate()
        return [int(p) for p in out.split()]    

    class Alarm(Exception):
        pass

    def alarm_handler(self, signum, frame):
        raise self.Alarm

    def onGetDataBtn(self):

        cmd = ['timeout 5 rostopic echo amcl_pose']
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
            outs, errs = process.communicate()
        
            number = re.findall('\d+\.\d+', outs)
            print(number)

            self.posx_lineEdit.setText(str(number[0]))
            self.posy_lineEdit.setText(str(number[1]))
            self.posz_lineEdit.setText(str(number[2]))

            self.orientx_lineEdit.setText(str(number[3]))
            self.orienty_lineEdit.setText(str(number[4]))
            self.orientz_lineEdit.setText(str(number[5]))
            self.orientw_lineEdit.setText(str(number[6]))

            orientation_list = [number[3], number[4], number[5], number[6]]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            ang = yaw*MATH_RAD2DEG
            self.orienta_lineEdit.setText(str(ang))

            for i in range(7, 43):
                self.covval_tableWidget.setItem(i-7, 0, QTableWidgetItem(str(number[i])))

        except:
            print("error : amcl_pose")

    def onExittBtn(self):
        self.amclpos_sub.unregister()
        print("Exit get amcl point data proc...")
        self.close()


class CAMViewWindow(QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Camera Viewer Window')
 
        self.online_webcams = QCameraInfo.availableCameras()
        if self.online_webcams:   
            self.exist = QCameraViewfinder()
            self.exist.show()
            self.get_webcam(5)
            
        else:
            self.print_MessStr("The web camera is not installed.")

        self.btnOK = QPushButton("OK")
        self.btnOK.clicked.connect(self.onOKButtonClicked)

        self.Layout = QVBoxLayout()
        self.Layout.addWidget(self.exist)
        self.Layout.addWidget(self.btnOK)
        self.setLayout(self.Layout)

        self.setFixedWidth(640)
        self.setFixedHeight(480)
      

    def get_webcam(self, i):
        self.webcam = QCamera(self.online_webcams[i])
        self.webcam.setViewfinder(self.exist)
        self.webcam.setCaptureMode(QCamera.CaptureStillImage)
        self.webcam.error.connect(lambda: self.alert(self.my_webcam.errorString()))
        self.webcam.start()

    def alert(self, s):
        """
        This handle errors and displaying alerts.
        """
        err = QErrorMessage(self)
        err.showMessage(s)        

    def onOKButtonClicked(self):
        self.close()
    

class Thread_TeleOP(QThread):
    print_lspeed = pyqtSignal(str)
    print_aspeed = pyqtSignal(str)
    print_lsinc = pyqtSignal(str)
    print_asinc = pyqtSignal(str)

    def __init__(self, parent = None):
        super(Thread_TeleOP, self).__init__(parent)

        self.count = 0
        self.x = 0.0
        self.th = 0.0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.speed = initSpeed
        self.turn = initTurn
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

    def initVal(self):
        self.count = 0
        self.x = 0.0
        self.th = 0.0
        self.status = 0
        self.count = 0
        self.acc = 0.1
        self.speed = initSpeed
        self.turn = initTurn
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        self.print_lsinc.emit('%.2f' % (self.speed))
        self.print_asinc.emit('%.2f' % (self.turn))
        self.print_lspeed.emit('%.2f' % (self.control_speed))
        self.print_aspeed.emit('%.2f' % (self.control_turn))

    def run(self):
        global convkey
        global prev_convkey
        global bRunMCtrl

        while bRunMCtrl == True:
            if convkey in moveBindings.keys():
                try:                
                    self.x = moveBindings[convkey][0]
                    self.th = moveBindings[convkey][1]
                except:
                    pass
 
            elif convkey == 's' :
                self.x = 0
                self.th = 0
                self.control_speed = 0
                self.control_turn = 0

            elif convkey == 'r':

                if prev_convkey in speedBindings.keys():
                    try:
                        self.speed = self.speed * speedBindings[prev_convkey][0]              
                        self.turn = self.turn * speedBindings[prev_convkey][1]
                    except:
                        pass
                    else:    
                        self.print_lsinc.emit('%.2f' % (self.speed))
                        self.print_asinc.emit('%.2f' % (self.turn))
                        convkey = 0

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

            self.print_lspeed.emit('%.2f' % (self.control_speed))
            self.print_aspeed.emit('%.2f' % (self.control_turn))

            twist = Twist()
            twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
            self.pub_twist.publish(twist)

            sleep(0.1)

        print("Exit thread of TeleOP...")


class Thread_RunSchedule(QThread):
    runtrajloop = pyqtSignal()
    runcsparking = pyqtSignal()   
    runtrajnum = pyqtSignal(int)  

    def __init__(self, parent = None):
        super(Thread_RunSchedule, self).__init__(parent)

        self.runcnt = 1
        

    def run(self):
        global bRunSchedule
		
        # start_time = 0
        # end_time = 24

        # inc_time = 10

        # hour_iter = (end_time - start_time) + 1
        # minute_iter = int(60/inc_time) + 1
        # print("hour_iter:{}  minute_iter:{}".format(hour_iter, minute_iter))


        # cur_time = start_time
        # cur_minute = 0
        
        # for i in range(1, hour_iter):
        #     for j in range(1, minute_iter):
        #         time_str = str(cur_time).zfill(2) + ":" + str(cur_minute).zfill(2)

        #         if j % 2 == 1:
        #             print("Traj time :" + time_str)
        #             schedule.every().day.at(time_str).do(self.RunTrajLoop)
        #         else:
        #             print("Parking time :" + time_str)
        #             schedule.every().day.at(time_str).do(self.RunCSParking)

        #         cur_minute += inc_time

        #     cur_minute = 0
        #     cur_time += 1

        # if cur_time >= 24:
        #     cur_time = 0
        #     cur_minute = 1
        
        # time_str = str(cur_time).zfill(2) + ":" + str(cur_minute).zfill(2)

        # schedule.every().day.at(time_str).do(self.StopSchedule)



        # schedule.every().day.at("18:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("18:20").do(self.RunCSParking)

        # schedule.every().day.at("18:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("18:50").do(self.RunCSParking)

        # schedule.every().day.at("19:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("19:20").do(self.RunCSParking)

        # schedule.every().day.at("19:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("19:50").do(self.RunCSParking)

        # schedule.every().day.at("20:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("20:20").do(self.RunCSParking)

        # schedule.every().day.at("20:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("20:50").do(self.RunCSParking)

        # schedule.every().day.at("21:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("21:20").do(self.RunCSParking)

        # schedule.every().day.at("21:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("21:50").do(self.RunCSParking)
        
        # schedule.every().day.at("22:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("22:20").do(self.RunCSParking)

        # schedule.every().day.at("22:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("22:50").do(self.RunCSParking)

        # schedule.every().day.at("23:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("23:20").do(self.RunCSParking)

        # schedule.every().day.at("23:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("23:50").do(self.RunCSParking)

        # schedule.every().day.at("00:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("00:20").do(self.RunCSParking)

        # schedule.every().day.at("00:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("00:50").do(self.RunCSParking)

        # schedule.every().day.at("01:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("01:20").do(self.RunCSParking)

        # schedule.every().day.at("01:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("01:50").do(self.RunCSParking)

        # schedule.every().day.at("02:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("02:20").do(self.RunCSParking)

        # schedule.every().day.at("02:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("02:50").do(self.RunCSParking)

        # schedule.every().day.at("03:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("03:20").do(self.RunCSParking)

        # schedule.every().day.at("03:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("03:50").do(self.RunCSParking)

        # schedule.every().day.at("04:00").do(self.RunTrajLoopNum)
        # schedule.every().day.at("04:20").do(self.RunCSParking)

        # schedule.every().day.at("04:30").do(self.RunTrajLoopNum)
        # schedule.every().day.at("04:50").do(self.RunCSParking)


        # schedule.every().day.at("09:00").do(self.RunTrajLoop)
        # schedule.every().day.at("11:50").do(self.RunCSParking)

        # schedule.every().day.at("13:00").do(self.RunTrajLoop)
        # schedule.every().day.at("16:30").do(self.RunCSParking)

        schedule.every().day.at("09:30").do(self.RunTrajLoop)
        schedule.every().day.at("09:50").do(self.RunCSParking)

        schedule.every().day.at("10:00").do(self.RunTrajLoop)
        schedule.every().day.at("10:20").do(self.RunCSParking)

        schedule.every().day.at("10:30").do(self.RunTrajLoop)
        schedule.every().day.at("10:50").do(self.RunCSParking)

        schedule.every().day.at("11:00").do(self.RunTrajLoop)
        schedule.every().day.at("11:20").do(self.RunCSParking)

        schedule.every().day.at("11:30").do(self.RunTrajLoop)
        schedule.every().day.at("11:50").do(self.RunCSParking)

        schedule.every().day.at("13:10").do(self.RunTrajLoop)
        schedule.every().day.at("13:30").do(self.RunCSParking)

        schedule.every().day.at("11:40").do(self.RunTrajLoop)
        schedule.every().day.at("11:55").do(self.RunCSParking)

        schedule.every().day.at("12:00").do(self.RunTrajLoop)
        schedule.every().day.at("12:15").do(self.RunCSParking)

        schedule.every().day.at("12:20").do(self.RunTrajLoop)
        schedule.every().day.at("12:35").do(self.RunCSParking)

        schedule.every().day.at("12:40").do(self.RunTrajLoop)
        schedule.every().day.at("12:55").do(self.RunCSParking)

        schedule.every().day.at("13:00").do(self.RunTrajLoop)
        schedule.every().day.at("13:15").do(self.RunCSParking)

        schedule.every().day.at("13:20").do(self.RunTrajLoop)
        schedule.every().day.at("13:35").do(self.RunCSParking)

        schedule.every().day.at("13:40").do(self.RunTrajLoop)
        schedule.every().day.at("13:50").do(self.RunCSParking)

        schedule.every().day.at("14:00").do(self.RunTrajLoop)
        schedule.every().day.at("14:10").do(self.RunCSParking)

        schedule.every().day.at("14:20").do(self.RunTrajLoop)
        schedule.every().day.at("14:30").do(self.RunCSParking)

        schedule.every().day.at("14:40").do(self.RunTrajLoop)
        schedule.every().day.at("14:50").do(self.RunCSParking)

        schedule.every().day.at("15:00").do(self.RunTrajLoop)
        schedule.every().day.at("15:10").do(self.RunCSParking)

        schedule.every().day.at("15:20").do(self.RunTrajLoop)
        schedule.every().day.at("15:30").do(self.RunCSParking)

        schedule.every().day.at("15:40").do(self.RunTrajLoop)
        schedule.every().day.at("15:50").do(self.RunCSParking)

        schedule.every().day.at("16:00").do(self.RunTrajLoop)
        schedule.every().day.at("16:10").do(self.RunCSParking)

        schedule.every().day.at("16:20").do(self.RunTrajLoop)
        schedule.every().day.at("16:30").do(self.RunCSParking)

        schedule.every().day.at("16:40").do(self.RunTrajLoop)
        schedule.every().day.at("16:50").do(self.RunCSParking)

        schedule.every().day.at("17:00").do(self.RunTrajLoop)
        schedule.every().day.at("17:10").do(self.RunCSParking)

        schedule.every().day.at("17:20").do(self.RunTrajLoop)
        schedule.every().day.at("17:30").do(self.RunCSParking)

        schedule.every().day.at("17:40").do(self.RunTrajLoop)
        schedule.every().day.at("17:50").do(self.RunCSParking)

        schedule.every().day.at("18:00").do(self.RunTrajLoop)
        schedule.every().day.at("18:15").do(self.RunCSParking)

        schedule.every().day.at("18:20").do(self.RunTrajLoop)
        schedule.every().day.at("18:35").do(self.RunCSParking)

        schedule.every().day.at("18:40").do(self.RunTrajLoop)
        schedule.every().day.at("18:55").do(self.RunCSParking)

        schedule.every().day.at("19:00").do(self.RunTrajLoop)
        schedule.every().day.at("19:15").do(self.RunCSParking)

        schedule.every().day.at("19:20").do(self.RunTrajLoop)
        schedule.every().day.at("19:35").do(self.RunCSParking)

        schedule.every().day.at("19:40").do(self.RunTrajLoop)
        schedule.every().day.at("19:55").do(self.RunCSParking)

        schedule.every().day.at("20:00").do(self.RunTrajLoop)
        schedule.every().day.at("20:15").do(self.RunCSParking)

        schedule.every().day.at("20:20").do(self.RunTrajLoop)
        schedule.every().day.at("20:35").do(self.RunCSParking)

        schedule.every().day.at("20:40").do(self.RunTrajLoop)
        schedule.every().day.at("20:55").do(self.RunCSParking)

        schedule.every().day.at("21:00").do(self.RunTrajLoop)
        schedule.every().day.at("21:15").do(self.RunCSParking)

        schedule.every().day.at("21:20").do(self.RunTrajLoop)
        schedule.every().day.at("21:35").do(self.RunCSParking)

        schedule.every().day.at("21:40").do(self.RunTrajLoop)
        schedule.every().day.at("21:55").do(self.RunCSParking)

        schedule.every().day.at("22:00").do(self.RunTrajLoop)
        schedule.every().day.at("22:15").do(self.RunCSParking)

        schedule.every().day.at("22:20").do(self.RunTrajLoop)
        schedule.every().day.at("22:35").do(self.RunCSParking)

        schedule.every().day.at("22:40").do(self.RunTrajLoop)
        schedule.every().day.at("22:55").do(self.RunCSParking)

        schedule.every().day.at("23:00").do(self.RunTrajLoop)
        schedule.every().day.at("23:15").do(self.RunCSParking)

        schedule.every().day.at("23:20").do(self.RunTrajLoop)
        schedule.every().day.at("23:35").do(self.RunCSParking)

        schedule.every().day.at("23:40").do(self.RunTrajLoop)
        schedule.every().day.at("23:55").do(self.RunCSParking)

        schedule.every().day.at("00:00").do(self.RunTrajLoop)
        schedule.every().day.at("00:15").do(self.RunCSParking)

        schedule.every().day.at("00:20").do(self.RunTrajLoop)
        schedule.every().day.at("00:35").do(self.RunCSParking)

        schedule.every().day.at("00:40").do(self.RunTrajLoop)
        schedule.every().day.at("00:55").do(self.RunCSParking)
        
        # schedule.every().day.at("13:20").do(self.RunTrajLoop)
        # schedule.every().day.at("13:30").do(self.RunCSParking)

        # schedule.every().day.at("13:40").do(self.RunTrajLoop)
        # schedule.every().day.at("13:50").do(self.RunCSParking)

        # schedule.every().day.at("14:00").do(self.RunTrajLoop)
        # schedule.every().day.at("14:10").do(self.RunCSParking)

        # schedule.every().day.at("15:20").do(self.RunTrajLoop)
        # schedule.every().day.at("15:30").do(self.RunCSParking)

        # schedule.every().day.at("16:40").do(self.RunTrajLoop)
        # schedule.every().day.at("16:50").do(self.RunCSParking)

        # schedule.every().day.at("17:00").do(self.RunTrajLoop)
        # schedule.every().day.at("17:10").do(self.RunCSParking)

        # schedule.every().day.at("17:20").do(self.RunTrajLoop)
        # schedule.every().day.at("17:30").do(self.RunCSParking)

        # schedule.every().day.at("17:40").do(self.RunTrajLoop)
        # schedule.every().day.at("17:50").do(self.RunCSParking)

        print("Run Schedule....")
        
        while bRunSchedule == True:
            schedule.run_pending()
            sleep(1)

        print("Exit schedule thread...")


    def StopSchedule(self):
        global bRunSchedule

        schedule.cancel_job(self.RunTrajLoop)
        schedule.cancel_job(self.RunCSParking)

        schedule.clear(tag=None)

        bRunSchedule = False

        print("Stopping schedule...")

    def RunTrajLoopNum(self):
        self.runtrajnum.emit(self.runcnt)
        self.runcnt += 1

        if self.runcnt >= 5:
            self.runcnt = 1

    def RunTrajLoop(self):
        self.runtrajloop.emit()

    def RunCSParking(self):
        self.runcsparking.emit()


class Thread_RunCSAutoparking(QThread):
    printmess = pyqtSignal(str)
    # prevrun_procstop = pyqtSignal(int)
    wpstop = pyqtSignal()
    trajstop = pyqtSignal()
    setchargeled = pyqtSignal(str)
    setpcsbtn = pyqtSignal(str)
    setrobotmode = pyqtSignal(str)
    estopdata = pyqtSignal(int)
    # endautopark = pyqtSignal()
    navi_runstat = pyqtSignal(int, str)
    setapuvc = pyqtSignal(bool)

    def __init__(self, parent = None):
        super(Thread_RunCSAutoparking, self).__init__(parent)

        self.bEStop_Status = 0
        self.bEStop = False
        self.bprevEStop = False
        self.bRunWaypoint = False
        self.RepCnt = 5

        self.naviStatus = NavigationControlStatus.IDLING
        self.prevnaviStatus = NavigationControlStatus.IDLING

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.ledcommand_pub = rospy.Publisher("/led_control_command", UInt64, queue_size=10)
        self.navictrl_pub = rospy.Publisher("/navi_ctrl", NavigationControl, queue_size = 10)

        self.estopdata.connect(self.setEStop)
        self.navi_runstat.connect(self.setNaviStat)

    @pyqtSlot(int)
    def setEStop(self, data):
        self.bEStop_Status = data
        
        if self.bEStop_Status == True:
            if self.bEStop_Status == 1:
                print("[H]Detected Lidar Field\r")
            elif self.bEStop_Status == 2:
                print("[H]Pushed E-Stop Button\r")
            # elif self.bEStop_Status == 3:
            #     print("[H]Detected Lidar Field and Pushed E-Stop Button\r")

            self.bprevEStop = self.bEStop
            self.bEStop = True

        elif self.bEStop_Status == False:
            print("[H]Release E-stop/ Lidar detecting\r")
            
            self.bprevEStop = self.bEStop
            self.bEStop = False

    @pyqtSlot(int, str)
    def setNaviStat(self, status, stat_descript):
        self.naviStatus = status

        if(self.prevnaviStatus != self.naviStatus):
            mstr = stat_descript
            print(mstr)

            print("naviStatus : " + str(self.naviStatus))

            self.prevnaviStatus = self.naviStatus
      
    def setRunWaypoint(self, val):
        self.bRunWaypoint = val

    def run(self):
        global AutoParkNum
        global bCSPRunOk

        bCSPRunOk = False

        if AutoParkNum == 1:
            print("Run auto parking process [1]")
            self.wpstop.emit()

            self.ClearCostMap()
            sleep(2)

            self.trajstop.emit()

            if self.bRunWaypoint == True:

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
                        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                        
                        sleep(3)
                        
                        self.bprevEStop = False

                        self.gotoWayPoint(fpos)

                        messStr = "Restart " + fpos

                        self.printmess.emit(messStr)

                        print(messStr)

                        self.naviStatus = NavigationControlStatus.IDLING

                    if self.naviStatus == NavigationControlStatus.COMPLETED:

                        messStr = "Arrived at " + fpos
                        print(messStr)
                        self.printmess.emit(messStr)
                        
                        break

                    if self.naviStatus == NavigationControlStatus.WARNNRGTO:
                        runcnt = runcnt + 1

                        if runcnt <= self.RepCnt:
                            os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

                            self.gotoWayPoint(fpos)

                            messStr = "Restart " + fpos + " : " + str(runcnt)
                            self.printmess.emit(messStr)
                            print(messStr)

                            sleep(2)

                        else:
                            runcnt = 0
                            messStr = "Error(WARNNRGTO) " + fpos
                            self.printmess.emit(messStr)                    

                        break

                    if self.naviStatus == NavigationControlStatus.ERRGTGF:
                        messStr = "Error(ERRGTGF) " + fpos
                        self.printmess.emit(messStr)               

                    sleep(0.1)

                self.ClearCostMap()
                sleep(2)


            # sleep(1)

            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
            # os.system(cmdstr)
            # print("Realsense CAM #1 emitter off")
            # sleep(5)
            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
            # os.system(cmdstr)
            # print("Realsense CAM #2 emitter off")
            # sleep(5)

            self.setapuvc.emit(False)
            
            self.client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            self.client.wait_for_server()

            goal = ChargingGoal()
            print("goal : ")
            print(goal)
            self.client.send_goal(goal)

            self.printmess.emit("Send goal for parking charging station.")
            self.setpcsbtn.emit("Cancel")
            self.setrobotmode.emit("Auto_ParkingCS")
            self.SendLedCmd("Auto_ParkingCS")
            # self.setrobotmode.emit("Auto_ParkingCS")
            
        elif AutoParkNum == 2:
            print("Run auto parking process [2]")
            
            cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)

            self.setchargeled.emit(ICON_GRAY_LED)

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

            self.printmess.emit("Cancel parking charge station.")
            self.setpcsbtn.emit("Parking C.S.")
            # print("robotmode : Ready Done ==> sendlcdcmd")
            self.setrobotmode.emit("ReadyDone")
            self.SendLedCmd("ReadyDone")

            sleep(1)

            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 1\" "
            # os.system(cmdstr)
            # print("Realsense CAM #1 emitter on")
            # sleep(5)
            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 1\" "
            # os.system(cmdstr)
            # print("Realsense CAM #2 emitter on")
            # sleep(5)

            self.InitPosition()

            sleep(3)

            self.setapuvc.emit(True)

            print("Done Auto Parking Cancel process...")

        bCSPRunOk = True
        print("Exit Thread_RunCSAutoparking")

        # self.quit()
        # self.wait()

    def gotoWayPoint(self, wpname):
        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = wpname
        self.navictrl_pub.publish(nc)
        
        self.wpName = wpname
        
        mstr = "[WP] Start way point : " + nc.goal_name
        print(mstr)
        self.printmess.emit(mstr)

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

        print("Thread_RunCSAutoparking : Run SendLedCmd ==> mode:" + bat_mode)


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

        self.printmess.emit("Set the start position...")

    def ClearCostMap(self):
        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")


# class Thread_RunLCDCmd(QThread):
#     def __init__(self, parent = None):
#         super(Thread_RunLCDCmd, self).__init__(parent)

#         self.ledcommand_pub = rospy.Publisher("/led_control_command", UInt64, queue_size=10)

#         self.mode = "None"

      
#     def run(self):
#         global led_control
#         global brunLCDCmd

#         fm = led_control[self.mode][0]
#         fc = led_control[self.mode][1]
#         rm = led_control[self.mode][2]
#         rc = led_control[self.mode][3]

#         fm = led_mode[fm]
#         fc = led_color[fc]
#         rm = led_mode[rm]
#         rc = led_color[rc]

#         fcolor_cmd = UInt64()
#         fcolor_cmd = ((((fm << 24) | fc) << 32) | (((rm << 24) | rc)))
#         self.ledcommand_pub.publish(fcolor_cmd)
#         sleep(1.0)
#         self.ledcommand_pub.publish(fcolor_cmd)
#         sleep(1.0)
#         self.ledcommand_pub.publish(fcolor_cmd)

#         print("Thread_RunLCDCmd ==> mode:" + self.mode)

#         brunLCDCmd = False

    
#     def setMode(self, mode):
#         self.mode = mode


class MyWindow(QMainWindow, main_ui):
    def __init__(self):

        super(QMainWindow, self).__init__()
        self.setupUi(self)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

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
        # self.bViewCmd = False
        # self.bprevViewCmd = False
        try :
            with open(yaml_fname) as f:
                wp_list = yaml.safe_load(f)
                # print(wp_list)
                
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
            
        self.messLV = QStandardItemModel()

        # self.wpCBStr = self.naviWPName_CB.currentText()
        # self.trajCBStr = self.navitrajName_CB.currentText()        

        self.runRoscore_PB.clicked.connect(self.onROSCoreBtn)
        self.runBringup_PB.clicked.connect(self.onBringupBtn)
        self.runGMSLAM_PB.clicked.connect(self.onGridmapSLAMBtn)
        self.runCGSLAM_PB.clicked.connect(self.onCartographerSLAMBtn)
        self.runSLAMRVIZ_PB.clicked.connect(self.onSLAMRVIZBtn)
        self.runNaviRVIZ_PB.clicked.connect(self.onNaviRVIZBtn)      
        self.runSaveMap_PB.clicked.connect(self.onSaveMapBtn)        
        self.runWPCtrl_PB.clicked.connect(self.onWPCtrlBtn)        
        self.runTeleOP_PB.clicked.connect(self.onTeleopBtn)
        self.runMotionCtrl_PB.clicked.connect(self.onMotionCtrlBtn)
        self.runMLRF_PB.clicked.connect(self.onMultiLRFBtn)
        self.runNavi_PB.clicked.connect(self.onNavigationBtn)
        self.runMakeWP_PB.clicked.connect(self.onMakeWPBtn)
        self.runSaveWP_PB.clicked.connect(self.onSaveWPBtn)
        self.runPCS_PB.clicked.connect(self.onParkingChargingStationBtn)
        self.purion_PB.clicked.connect(self.onPuriOnBtn)
        self.purioff_PB.clicked.connect(self.onPuriOffBtn)

        self.airPurifier_dial.setRange(0, 400)
        self.airPurifier_dial.valueChanged.connect(self.setAPVal)
        self.airPurifier_dial.setNotchesVisible(True) 
        
        self.uvcon_PB.clicked.connect(self.onUVCOnBtn)
        self.uvcoff_PB.clicked.connect(self.onUVCOffBtn)
        self.pumpon_PB.clicked.connect(self.onPumpOnBtn)
        self.pumpoff_PB.clicked.connect(self.onPumpOffBtn)
        self.solvalon_PB.clicked.connect(self.onSolValveOnBtn)
        self.solvaloff_PB.clicked.connect(self.onSolValveOffBtn)
        

        # self.frontMode_CB.addItem("Off")
        # self.frontMode_CB.addItem("On")
        # self.frontMode_CB.addItem("Blink")
        # self.frontMode_CB.addItem("FBlink")
        # self.frontMode_CB.addItem("Fade")
        # self.frontMode_CB.addItem("Sweep")
        # self.frontMode_CB.addItem("FSweep")
        # self.frontMode_CB.addItem("Stay")

        self.rearMode_CB.addItem("Off")
        self.rearMode_CB.addItem("On")
        self.rearMode_CB.addItem("Blink")
        self.rearMode_CB.addItem("FBlink")
        self.rearMode_CB.addItem("Fade")
        self.rearMode_CB.addItem("Sweep")
        self.rearMode_CB.addItem("FSweep")
        self.rearMode_CB.addItem("Stay")

        self.airpurifval_CB.addItem("Off")
        self.airpurifval_CB.addItem("Level-1")
        self.airpurifval_CB.addItem("Level-2")
        self.airpurifval_CB.addItem("Level-3")
       
        # self.frontcolor_PB.clicked.connect(self.FrontColorBtn)
        self.rearcolor_PB.clicked.connect(self.RearColorBtn)
        self.ledcmdsend_PB.clicked.connect(self.LedCmdSendBtn)      
        self.mcinit_PB.clicked.connect(self.InitMCValueBtn)      
        self.ready_PB.clicked.connect(self.ReadyBtn)
        self.exit_PB.clicked.connect(self.onExit)

        self.Bat1_Info = BatteryInformationMsgs()
        self.Bat2_Info = BatteryInformationMsgs()

        self.viewsonar_checkBox.stateChanged.connect(self.changeViewSD)
        self.trajmode_checkBox.stateChanged.connect(self.changeTrajMode)
        self.chargingmode_checkBox.stateChanged.connect(self.changeCSMode)
        self.autopurif_checkBox.stateChanged.connect(self.changeAutoPF)
        self.usesonar_checkBox.stateChanged.connect(self.changeUseSonar)
        
        self.ur_progressBar.setFormat("%p mm")
        self.ub_progressBar.setFormat("%p mm")
        self.ul_progressBar.setFormat("%p mm")
        self.df_progressBar.setFormat("%p mm")
        self.dr1_progressBar.setFormat("%p mm")
        self.dr2_progressBar.setFormat("%p mm")
        self.db1_progressBar.setFormat("%p mm")
        self.db2_progressBar.setFormat("%p mm")
        self.dl1_progressBar.setFormat("%p mm")
        self.dl2_progressBar.setFormat("%p mm")

        self.mapfile_lineEdit.setText('Office')
        self.WPfile_lineEdit.setText('waypoint5')

        self.ready_progBar.setFormat("%p%")
        self.ready_progBar.setValue(0)

        self.WPNameCBStr = self.naviWPName_CB.currentText()
        self.naviTrajCBStr = self.navitrajName_CB.currentText()
        self.airPurifCBStr = self.airpurifval_CB.currentText()

        self.naviWPName_CB.activated[str].connect(self.onActivatedWPNameCB)
        self.navitrajName_CB.activated[str].connect(self.onActivatednaviTrajCB)
        self.airpurifval_CB.activated[str].connect(self.onActivatedAirPurifCB)
        self.wpStart_PB.clicked.connect(self.onWPStartBtn)
        self.wpStop_PB.clicked.connect(self.onWPStopBtn)
        self.naviTrajStart_PB.clicked.connect(self.onTrajStartBtn)
        self.naviTrajStop_PB.clicked.connect(self.onTrajStopBtn)
        self.runOP_PB.clicked.connect(self.onOperationBtn)
        self.camview_PB.clicked.connect(self.onCAMViewBtn)
        self.rsview_PB.clicked.connect(self.RunRealsenView)
        self.getamclpos_PB.clicked.connect(self.onGetAMCLPosBtn)
        self.AIR_PB.clicked.connect(self.onAirInfoBtn)
        self.runproc_PB.clicked.connect(self.onRunProc)
        self.initPos_PB.clicked.connect(self.SetStartPosition)      
        self.webmanager_PB.clicked.connect(self.onWebManagerLaunch)      
        self.initangle_PB.clicked.connect(self.onInitRobotHeadingAngle) 

        self.camtopic_PB.clicked.connect(self.onLaunchCAMTopic) 

        # run 18 terminal window
        os.system("python3 ~/bin/target_term -set 18")
        self.print_MessStr("Run terminal windows.")

        # for i in range(1, 19):
        #     cmd = "python3 ~/bin/target_term -run "+ str(i) + (" echo terminal num :" + str(i))
        #     # print(cmd)
        #     os.system(cmd)

        #     sleep(0.5)

        # data = os.environ["HOME"]+"/.term_list"
        # self.t_term = open(data).read().splitlines()
        # print(self.t_term)

        self.puriLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.uvcLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.chargingLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.estopLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

        self.lspeed_lineEdit.setText("0.0")
        self.aspeed_lineEdit.setText("0.0")
        self.lsinc_lineEdit.setText(str(initSpeed))
        self.asinc_lineEdit.setText(str(initTurn))
        self.headingangle_lineEdit.setText(str(0.0))

        sleep(2)

        # cmd = "roslaunch zetabank_bringup zetabank_robot_serial.launch"
        cmd = "python3 ~/bin/target_term -run 1 roslaunch zetabank_bringup zetabank_robot_serial.launch"
        self.rosserial_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
        # os.system(cmd)

        self.print_MessStr("Run roslaunch zetabank_robot_serial.launch")

        sleep(2)

        self.naviCtrl_pub = rospy.Publisher("/navi_ctrl", NavigationControl, queue_size = 10)
        self.power_control_pub = rospy.Publisher("/power_control_command", PowerControlMsgs, queue_size=10)
        self.air_purifier_pub = rospy.Publisher("/purifier_control_command", UInt16, queue_size=10)
        self.ledcommand_pub = rospy.Publisher("/led_control_command", UInt64, queue_size=10)
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.air_cancel_pub = rospy.Publisher("/air_condition_cancel", Bool, queue_size=10)
        self.floor_cancel_pub = rospy.Publisher("/floor_cleaning_cancel", Bool, queue_size=10)

        self.navictrl_status_sub = rospy.Subscriber("/navi_ctrl_status", NavigationControlStatus, self.CallbackRunNaviCtrlStatus)
        self.bat_sub = rospy.Subscriber("/battery", BatteryInformationMsgs, self.CallbackBatteryStatus)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.recv_IMU)
        self.sonar_sub = rospy.Subscriber("/sonar", Float32MultiArray, self.recv_Sonar)
        self.charge_state_sub = rospy.Subscriber("/autocharge_state_INO", String, self.CallbackCSStatus)
        self.estop_sub = rospy.Subscriber("/estop", Bool, self.CallEStop)
        self.air_sub = rospy.Subscriber("/air", EnvironmentMsgs, self.CallAirInfo)
        self.robotpos_sub = rospy.Subscriber("/robot_pose", Pose, self.CallCurPoseInfo)

        self.robotpos_sub = rospy.Subscriber("/set_camview", Bool, self.CallSetCAMView)

        self.charge_state_nuc_sub = rospy.Subscriber("/autocharge_state_NUC", UInt8, self.CallbackCSStatusNUC)

        # cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
        # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
        # os.system(cmdstr)
        # sleep(5)
        # cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
        # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
        # os.system(cmdstr)
        # sleep(5)

        
        self.print_MessStr("Complete initialization...")

        sleep(2)

        self.bStartOK = True

        self.robot_mode = "None"
        # self.runLCDCmd(self.robot_mode)
        self.SendLedCmd(self.robot_mode)

        if autorun == True:
            sleep(2)

            self.ReadyBtn()

        # start_time = 12
        # end_time = 16

        # inc_time = 10

        # hour_iter = (end_time - start_time) + 1
        # minute_iter = int(60/inc_time) + 1
        # print("hour_iter:{}  minute_iter:{}".format(hour_iter, minute_iter))


        # cur_time = start_time
        # cur_minute = 0
        
        # for i in range(1, hour_iter):
        #     for j in range(1, minute_iter):
        #         time_str = str(cur_time) + ":" + str(cur_minute)

        #         if j % 2 == 1:
        #             print("Traj time :" + time_str)
        #         else:
        #             print("Parking time :" + time_str)

        #         cur_minute += inc_time

        #     cur_minute = 0
        #     cur_time += 1

    # def runLCDCmd(self, mode):
    #     global brunLCDCmd

    #     if brunLCDCmd == False:
    #         brunLCDCmd = True
    #         self.lcdcmdth = Thread_RunLCDCmd(self)
    #         self.lcdcmdth.setMode(mode)
    #         self.lcdcmdth.start()

    # def runLCDCmd(self, mode):
    #     global brunLCDCmd

    #     if brunLCDCmd == False:
    #         brunLCDCmd = True
    #         self.lcdcmdth = Thread_RunLCDCmd(self)
    #         self.lcdcmdth.setMode(mode)
    #         self.lcdcmdth.start()

    def runAutoParkThread(self, val):

        self.autopark_th = Thread_RunCSAutoparking(self)

        self.autopark_th.printmess.connect(self.PrintMess)
        self.autopark_th.wpstop.connect(self.WPStop)
        self.autopark_th.trajstop.connect(self.TrajStop)
        self.autopark_th.setchargeled.connect(self.SetChargeLED)
        self.autopark_th.setpcsbtn.connect(self.SetPCSBtn)
        self.autopark_th.setrobotmode.connect(self.SetRobotMode)
        self.autopark_th.setapuvc.connect(self.SetAPUVC)

        self.autopark_th.setRunWaypoint(val)

        self.autopark_th.start()

        self.bRunAutoParking = True
         

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
                # 
                # 
                
                
                self.robot_mode = "Navigation_Normal"

                # self.runLCDCmd( self.battery_mode)
                self.SendLedCmd(self.battery_mode)
                print("Change robot mode to FullCharging")

                # self.bprevViewCmd = self.bViewCmd
                # self.bViewCmd = False

        else:
            self.bBatteryFull = False
            

    def CallSetCAMView(self, msg):
        setVal = msg.data

        if setVal == True:
            self.bLauchCAMTopic = True

            cmd = "python3 ~/bin/target_term -run 18 roslaunch usb_cam usb_cam.launch"
            try:
                self.launchcamtopic_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

                self.print_MessStr("Launch cam live topic...")
            except:
                print("Failed to run navigation process...")

            self.camtopic_PB.setText('Stop CAM Topic')

        else:
            self.bLauchCAMTopic = False

            # sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam")     
            
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/camera_info")       
            # sleep(0.5)      
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw")
            # sleep(0.5)
            # # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressed")            
            # # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressed/parameter_descriptions")            

            # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressed/parameter_updates")       
            # sleep(0.5)
            # # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressedDepth")       
            # # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressedDepth/parameter_descriptions")       
            # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressedDepth/parameter_updates")       
            # sleep(0.5)
            # # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/theora")       
            # # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/theora/parameter_descriptions")       
            # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/theora/parameter_updates")       
            sleep(2.0)
            
            try:            
                self.launchcamtopic_proc.kill()
            except:
                print("usb cam nodes have died...") 

            self.print_MessStr("Stop cam live topic...")
            self.camtopic_PB.setText('Launch CAM Topic')
        

    def onLaunchCAMTopic(self):

        if self.bLauchCAMTopic == False:
            self.bLauchCAMTopic = True

            cmd = "python3 ~/bin/target_term -run 18 roslaunch usb_cam usb_cam.launch"
            try:
                self.launchcamtopic_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

                self.print_MessStr("Launch cam live topic...")
            except:
                print("Failed to run navigation process...")

            self.camtopic_PB.setText('Stop CAM Topic')

        else:
            self.bLauchCAMTopic = False

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam")       

            # # sleep(0.2)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/camera_info")       
            # sleep(0.5)      
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw")
            # sleep(0.5)
            # # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressed")            
            # # sleep(0.5)
            # os.system("python3 ~/binexit
            # /target_term -run 10 rosnode kill /usb_cam/image_raw/compressed/parameter_descriptions")            

            # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressed/parameter_updates")       
            # sleep(0.5)
            # # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressedDepth")       
            # # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressedDepth/parameter_descriptions")       
            # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/compressedDepth/parameter_updates")       
            # sleep(0.5)
            # # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/theora")       
            # # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/theora/parameter_descriptions")       
            # sleep(0.5)
            # os.system("python3 ~/bin/target_term -run 10 rosnode kill /usb_cam/image_raw/theora/parameter_updates")       
            sleep(2.0)
            
            try:            
                self.launchcamtopic_proc.kill()
            except:
                print("usb cam nodes have died...") 

            self.print_MessStr("Stop cam live topic...")
            self.camtopic_PB.setText('Launch CAM Topic')


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
        self.print_MessStr(viewstr)


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

            cmd1 = "python3 ~/bin/target_term -run 16 roslaunch rosboard rosboard.launch"
            try:
                self.rosboard_proc = subprocess.Popen(args=cmd1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

                self.print_MessStr("Run roslaunch rosboard.launch")

                sleep(5)

                cmd2 = "python3 ~/bin/target_term -run 17 rosrun rosboard rosboard_node"
                try:
                    self.rosboardnode_proc = subprocess.Popen(args=cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)

                    sleep(3)

                    self.print_MessStr("Run rosrun rosboard_node")
                    self.webmanager_PB.setText('Stop Web Manager')
        
                    self.bRunWebManager = True

                except:
                    print("Failed to launch rosboard_node...")
                    self.bRunWebManager = False
            except:
                print("Failed to launch rosboard...")
                self.bRunWebManager = False

        else:
            self.bRunWebManager = False

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /rosbridge_websocket")       
            sleep(1.0)        

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /web_video_server")       
            sleep(1.0)        

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /ros_api")       
            sleep(1.0)        

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /ros_pose_publisher")       
            sleep(2.0)        

            try:            
                self.rosboard_proc.kill()

                self.print_MessStr("Exit roslaunch rosboard.launch")
            except:
                print("rosboard_proc process has died...") 


            os.system("python3 ~/bin/target_term -run 10 rosnode kill /rosboard_node")       

            sleep(2.0)        

            try:            
                self.rosboardnode_proc.kill()
                self.print_MessStr("Exit rosrun rosboard_node")

            except:
                print("rosboardnode_proc process has died...") 

            self.webmanager_PB.setText('Run Web Manager')

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

        self.print_MessStr("Set the start position...")


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

        self.print_MessStr("Set the start position...")

    @pyqtSlot(bool)
    def SetAPUVC(self, val):
        if val == True:
            self.onUVCOnBtn()
            sleep(3)
            self.onPuriOnBtn()

        else:
            self.onUVCOffBtn()
            sleep(3)
            self.onPuriOffBtn()

    @pyqtSlot(str)
    def PrintMess(self, mess):
        self.print_MessStr(mess)

    @pyqtSlot()
    def WPStop(self):
        if self.bRunNaviWP is True:
            self.onWPStopBtn()

    @pyqtSlot()
    def TrajStop(self):
        if self.bRunTrajNavi is True:
            self.onTrajStopBtn()

    @pyqtSlot(str)
    def SetChargeLED(self, led):
        self.chargingLED_label.setPixmap(QtGui.QPixmap(led))
        
    @pyqtSlot(str)
    def SetPCSBtn(self, btn_name):
        self.runPCS_PB.setText(btn_name)
        # if btn_name == "Cancel":
        #     self.robot_mode = "Auto_ParkingCS"
        #     # self.runLCDCmd(self.robot_mode)
        #     self.SendLedCmd(self.robot_mode)
        #     print("SetPCSBtn #1 ==> SendLedCmd, mode:" + self.robot_mode)
        # elif btn_name == "Parking C.S.":
        #     self.robot_mode = "ReadyDone"
        #     # self.runLCDCmd(self.robot_mode)
        #     self.SendLedCmd(self.robot_mode)
        #     print("SetPCSBtn #2 ==> SendLedCmd, mode:" + self.robot_mode)
                    

    @pyqtSlot(str)
    def SetRobotMode(self, mode):
        self.robot_mode = mode
        # self.runLCDCmd(self.robot_mode)
        # self.SendLedCmd(self.robot_mode)

        # print("SetRobotMode ==> SendLedCmd, mode:" + mode)

        # self.bLedMCtrl = False

    @pyqtSlot(int)
    def RunPackage(self, num):
        if num == 1:
            if self.bBringUpRun == False:
                self.onBringupBtn()
        elif num == 2:
            if self.bMultiLRFRun == False:
                self.onMultiLRFBtn()
        elif num == 3:
            if self.bOPRun == False:
                self.onOperationBtn()
        elif num == 4:
            if self.bNaviRun == False:
                self.onNavigationBtn()
        elif num == 5:
            if self.bNaviRVIZRun == False:
                self.onNaviRVIZBtn()
        elif num == 6:
            if self.bWPCRun == False:
                self.onWPCtrlBtn()
                if self.bRunTrajNavi == True:
                    self.onTrajStartBtn()

            self.bReadyDone = True
            self.ready_PB.setText('Init')
            self.print_MessStr("Done ready...")
            self.robot_mode = "ReadyDone"
            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)

        elif num == 7:
            self.SetStartPosition()

        elif num == 8:
            cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
            cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
            os.system(cmdstr)
            sleep(5)
            cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
            cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
            os.system(cmdstr)
            sleep(5)

    @pyqtSlot(int)
    def ExitPackage(self, num):
        if num == 1:
            self.onBringupBtn()
        elif num == 2:
            self.onMultiLRFBtn()
        elif num == 3:
            self.onOperationBtn()
        elif num == 4:
            self.onNavigationBtn()
        elif num == 5:
            self.onNaviRVIZBtn()
        elif num == 6:
            self.onWPCtrlBtn()
            if self.bRunTrajNavi == True:
                self.onTrajStartBtn()            

            self.bReadyDone = False
            self.ready_PB.setText('Ready')
            self.print_MessStr("Exit all ready...")
            self.robot_mode = "None"
            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)
        

    @pyqtSlot(int)
    def SetRunState(self, val):
        self.ready_progBar.setValue(val)

    def ReadyBtn(self):

        if self.bReady == False:
            self.bReady = True

            self.ready_progBar.setValue(0)

            ready_th = Thread_RunReady(self)
            ready_th.runpackage.connect(self.RunPackage)
            ready_th.exitpackage.connect(self.ExitPackage)
            ready_th.setrunstate.connect(self.SetRunState)
            ready_th.setType(1)
            ready_th.start()

            # self.camview_PB.setDisabled(True)            

        else:
            self.bReady = False
            self.ready_progBar.setValue(0)

            ready_th = Thread_RunReady(self)
            ready_th.runpackage.connect(self.RunPackage)
            ready_th.exitpackage.connect(self.ExitPackage)
            ready_th.setrunstate.connect(self.SetRunState)
            ready_th.setType(2)
            ready_th.start()

            self.camview_PB.setEnabled(True)

    def disableColorPicker(self):
        # self.frontMode_CB.setDisabled(True)
        # self.frontcolor_PB.setDisabled(True)
        self.rearMode_CB.setDisabled(True)
        self.rearcolor_PB.setDisabled(True)
        self.ledcmdsend_PB.setDisabled(True)            

    def enableColorPicker(self):
        # self.frontMode_CB.setEnabled(True)
        # self.frontcolor_PB.setEnabled(True)
        self.rearMode_CB.setEnabled(True)
        self.rearcolor_PB.setEnabled(True)
        self.ledcmdsend_PB.setEnabled(True)            

    def onRunProc(self):
        global bRunSchedule

        if bRunSchedule == False:
            bRunSchedule = True

            self.trajmode_checkBox.setChecked(True)
            # self.chargingmode_checkBox.setChecked(True)

            if self.trajmode == NavigationControl.GOAL:
                self.trajmode = NavigationControl.LOOP

            # if self.csmode == False:
            #     self.csmode = True

            self.schedule_th = Thread_RunSchedule(self)
            self.schedule_th.start()

            self.schedule_th.runtrajloop.connect(self.RunTrajLoop)
            self.schedule_th.runcsparking.connect(self.RunCSParking)
            self.schedule_th.runtrajnum.connect(self.RunTrajLoopNum)

            self.runproc_PB.setText('Stop Proc')       
            self.print_MessStr("Run schedule...")


        else:

            self.schedule_th.StopSchedule()

            # bRunSchedule = False

            self.trajmode_checkBox.setChecked(False)
            # self.chargingmode_checkBox.setChecked(False)

            self.trajmode = NavigationControl.GOAL
            self.csmode = False

            nc = NavigationControl()
            nc.control = NavigationControl.STOP
            nc.goal_name = self.naviTrajCBStr
            nc.mode = self.trajmode
            self.naviCtrl_pub.publish(nc)

            self.runproc_PB.setText('Run Proc')       
            self.print_MessStr("Stop schedule...")
        
    @pyqtSlot(int)
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
        self.print_MessStr("Run trajectory loop....")

    @pyqtSlot()
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

            # self.onParkingChargingStationBtn()

            # sleep(1)

            # self.onInitPosition()

            # sleep(2)

            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 1\" "
            # os.system(cmdstr)
            # sleep(1)
            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 1\" "
            # os.system(cmdstr)
            # sleep(1)
        
        print("start onTrajStartBtn ==> bCSPRunOk :" + str(bCSPRunOk))

        self.onTrajStartBtn()

        print("Run trajectory loop : #" + str(self.schedule_run_cnt))
        self.print_MessStr("Run trajectory loop....")

        self.onPuriOnBtn()
        sleep(3)
        self.onUVCOnBtn()

        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        # self.runLCDCmd(self.robot_mode)
        self.SendLedCmd(self.robot_mode)

        print("send led cmd  : RunTrajLoop ==> robot_mode :" + self.robot_mode)

    # def gotoWayPoint(self, wpname):
    #     nc = NavigationControl()
    #     nc.control = NavigationControl.START
    #     nc.goal_name = wpname
    #     self.naviCtrl_pub.publish(nc)
        
    #     self.wpName = wpname
        
    #     mstr = "[WP] Start way point : " + nc.goal_name
    #     print(mstr)
    #     self.print_MessStr(mstr)
        
        
    @pyqtSlot()
    def RunCSParking(self):
        global AutoParkNum

        # self.onTrajStartBtn()

        # self.ClearCostMap()

        # sleep(2)+

        # self.gotoWayPoint("finalpos")

        # runcnt = 0

        
        # while True:
        #     if self.bEStop == True:
  
        #         sleep(0.1)
                
        #         continue

        #     if self.bEStop == False and self.bprevEStop == True:
        #         os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")
                
        #         sleep(3)
                
        #         self.bprevEStop = False

        #         self.gotoWayPoint("finalpos")

        #         self.print_MessStr("Restart finalpos")

        #         print("Restart finalpos")

        #         self.naviStatus = NavigationControlStatus.IDLING

        #     if self.naviStatus == NavigationControlStatus.COMPLETED:

        #         print("Arrived at final pos...")
        #         self.print_MessStr("Arrived at final pos...")
                
        #         break

        #     if self.naviStatus == NavigationControlStatus.WARNNRGTO:
        #         runcnt = runcnt + 1

        #         if runcnt <= self.RepCnt:
        #             os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

        #             self.gotoWayPoint("finalpos")

        #             self.print_MessStr("Restart final pos")

        #             print("Restart final pos : " + str(runcnt))

        #             sleep(2)

        #         else:
        #             runcnt = 0

        #             self.print_MessStr("Error(WARNNRGTO) final pos")                    

        #         break

        #     if self.naviStatus == NavigationControlStatus.ERRGTGF:
        #         self.print_MessStr("Error(ERRGTGF) final pos")               

        #     sleep(0.1)
            

        # sleep(2)

        # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
        # os.system(cmdstr)
        # sleep(1)
        # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
        # os.system(cmdstr)
        # sleep(1)

        if self.bCSParking == False:
            self.bCSParking = True

            # self.bprevViewCmd = self.bViewCmd
            # self.bViewCmd = False
            
            # self.autopark_th.setRunWaypoint(True)

            AutoParkNum = 1
            self.runAutoParkThread(True)

            # self.onParkingChargingStationBtn()

            self.print_MessStr("Run auto parking....")
        
        print("Run auto parking : #" + str(self.schedule_run_cnt))


    def CallAirInfo(self, msgs):
        global open_airinfo_dlg

        if open_airinfo_dlg == False:
            air_info_dict = dict()  

            air_info_dict = {
                "hour" : str(localtime(time()).tm_hour),
                "minute" : str(localtime(time()).tm_min),
                "second" : str(localtime(time()).tm_sec),
                "Fine_dust" : msgs.Dust_PM10_ugm3,
                "Ultrafine_dust" : msgs.Dust_PM2_5_ugm3,
                "CO2" : msgs.CO2_ppm,
                "Formaldehyde" : msgs.HCHO_ugm3,
                "CO" : msgs.CO_ppm,
                "NO2" : msgs.NO2_ppm,
                "Radon" : msgs.Rn_Bqm3,
                "Organic_compounds" : msgs.TVOCs_ugm3,
                "Temperature" : msgs.temp_celcius,
                "Humidity" : msgs.hum_RHp
            }

            if self.bAutoPF == True:
                if ((msgs.Dust_PM10_ugm3 >= self.pm10) or (msgs.Dust_PM2_5_ugm3 >= self.pm25)) and (self.bAirPuriRun == False):

                    self.bAirPuriRun = True
                    self.purifier_control(400)                
                    
                    self.robotStatus_lineEdit.setText("Air-Purifier ON")
                    self.puriLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

                    self.print_MessStr("Run Air-Purifier...")
                else:

                    if self.bAirPuriRun == True:
                        self.bAirPuriRun = False
                        
                        self.purifier_control(0)                
                            
                        self.robotStatus_lineEdit.setText("Air-Purifier OFF")
                        self.puriLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))


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
            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)
            self.estopLED_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))

            self.robotStatus_lineEdit.setText("!! ESTOP !!")

            self.bprevEStop = True

            if self.bRunNaviWP is True:
                self.onWPStopBtn()

            if self.bRunTrajNavi is True:
                self.onTrajStopBtn()

            if self.bRunAutoParking == True:
                self.autopark_th.estopdata.emit(self.bEStop_Status)

            if self.bCSParking is True:
                self.bCSParking = False
                # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 1\" "
                # os.system(cmdstr)
                # sleep(5)

                # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 1\" "
                # os.system(cmdstr)
                # sleep(3)

                # cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
                # os.system(cmdstr)
                self.print_MessStr("Cancel parking charge station-->ESTOP.")
                self.runPCS_PB.setText('Parking C.S.')                

                self.chargingLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            self.robot_mode = "E-Stop"
            self.SendLedCmd(self.robot_mode)

        else:
            if self.bprevEStop is True and self.bEStop is False:
                self.bprevEStop = False

                self.robot_mode = "Normal"
                # self.runLCDCmd(self.robot_mode)
                self.SendLedCmd(self.robot_mode)

                print("ESTOP ==> SendLedCmd, robot_mode : Normal")

                logdata["safety_info"] = 0x00
            
                self.estopLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
                self.robotStatus_lineEdit.setText("Release ESTOP")

                if self.bRunAutoParking == True:
                    self.autopark_th.estopdata.emit(self.bEStop_Status)

                # self.robot_mode = "ReadyDone"
                # self.SendLedCmd(self.robot_mode)


    def CallbackCSStatus(self, msgs):
        if self.bStartOK == False:
            return

        if self.bCSParking is True and (self.robot_mode != "Charging" or self.robot_mode != "FullCharging"):
            self.csStatus = msgs.data

        # self.csStatus = msgs.data
        # print("robot status : %s" % (self.csStatus))

        # if self.bCSParking is True and self.robot_mode != "Charging":
        # if self.bCSParking is True and (self.robot_mode != "Charging" or self.robot_mode != "FullCharging"):
        # if self.bCSParking is True and self.robot_mode != "Charging":
            # self.csStatus = msgs.data
            mstr = "R<->C.S : " + self.csStatus
            self.robotStatus_lineEdit.setText(mstr)

            if self.csStatus == "contact" and self.robot_mode != "Charging":
                self.chargingLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

                self.robot_mode = "Charging"
                # print("robot mode : %s" % (self.robot_mode))
                self.bViewCmd = False
                self.prevbattery_mode = "None"
                self.bRunAutoParking = False

                print("CallbackCSStatus ==> prevbattery_mode:" + self.prevbattery_mode + " battery_mode:" + self.battery_mode)

        # elif self.bCSParking is True and self.robot_mode == "FullCharging":
        #     mstr = "R<->C.S : FullCharge"
        #     self.robotStatus_lineEdit.setText(mstr)


    def RunRealsenView(self):
        os.system("python3 ~/bin/target_term -run 10 realsense-viewer")

    def recv_HeadingAngle(self, msgs):
        self.angle_lcdNumber.display(msgs.data)

    @pyqtSlot(str)
    def printLinearSpeed(self, str):
        self.lspeed_lineEdit.setText(str)

    @pyqtSlot(str)
    def printAngleSpeed(self, str):
        self.aspeed_lineEdit.setText(str)

    @pyqtSlot(str)
    def printLSInc(self, str):
        self.lsinc_lineEdit.setText(str)

    @pyqtSlot(str)
    def printASInc(self, str):
        self.asinc_lineEdit.setText(str)

    def onAirInfoBtn(self):
        global open_airinfo_dlg

        if open_airinfo_dlg is False:
            open_airinfo_dlg = True
            
            dialog = AirInfoDialog()
            dialog.exec_()

    def onGetAMCLPosBtn(self):
        dialog = GetAMCLPosDialog()
        dialog.exec_()

    def ClearCostMap(self):
        os.system("python3 ~/bin/target_term -run 10 rosservice call /move_base/clear_costmaps \"{}\"  ")

    def onCAMViewBtn(self):
        if self.bCSParking == True:
            self.print_MessStr("Now the robot is doing auto parking.")
        else:
            if self.bCAMView == False:
                self.bCAMView = True
                
                self.online_webcams = QCameraInfo.availableCameras()
                # print(self.online_webcams)

                # if self.online_webcams:   
                #     self.exist = QCameraViewfinder()
                #     self.exist.show()

                for cam in self.online_webcams:
                    print(cam.deviceName())
                    if cam.deviceName() == "/dev/video6":
                        try:
                            self.exist = QCameraViewfinder()
                            self.exist.show()
                            self.webcam = QCamera(cam)
                            self.webcam.setViewfinder(self.exist)
                            self.webcam.setCaptureMode(QCamera.CaptureStillImage)
                            self.webcam.error.connect(lambda: self.alert(self.my_webcam.errorString()))
                            self.webcam.start()

                            self.print_MessStr("run camera viewwer...")
                        except:
                            print("CAM view running error...")

                self.camview_PB.setText('Stop View')

            else:
                self.bCAMView = False

                try:
                    self.webcam.stop()

                    self.exist.hide()

                    self.print_MessStr("stop camera viewwer...")

                    self.camview_PB.setText('CAM View')
                except:
                    print("CAM view stop error...")

                    self.camview_PB.setText('CAM View')

                
            # print(self.online_webcams)
            # if self.online_webcams:   
            #     self.exist = QCameraViewfinder()
            #     self.exist.show()
                
                # video_idx = None

                # for video_idx_file in os.listdir("/sys/class/video4linux"):
                #     video_file_path = os.path.realpath("/sys/class/video4linux/" + video_idx_file)

                #     for video_files in os.listdir(video_file_path):
                #         if 'name' in video_files:
                #             name = open(os.path.realpath(video_file_path + '/' + 'name'), 'r').readline()
                #             print("name:" + name)

                #             # if name.find("Video Capture 2") > -1:
                #             #     index = open(os.path.realpath(video_file_path + '/' + 'index'), 'r').readline()
                #             #     print("index" + index)

                #             #     video_idx = "/dev/" + video_idx_file
                #             #     print("video_idx:" + video_idx)

                #             #     return video_idx

                #             if re.match('KINGSEN', name) != None:
                #                 index = open(os.path.realpath(video_file_path + '/' + 'index'), 'r').readline()
                #                 if re.match('0', index) != None:
                #                     video_idx = "/dev/" + video_idx_file
                #                     print(video_idx)
                                    
                # set the default webcam.
                # self.get_webcam(video_idx)
                # self.get_webcam(6)
                # self.print_MessStr("run camera viewwer...")
                
            # else:
            #     self.print_MessStr("The web camera is not installed.")

    def get_webcam(self, i):
        self.webcam = QCamera(self.online_webcams[i])
        self.webcam.setViewfinder(self.exist)
        self.webcam.setCaptureMode(QCamera.CaptureStillImage)
        self.webcam.error.connect(lambda: self.alert(self.my_webcam.errorString()))
        self.webcam.start()

    def alert(self, s):
        """
        This handle errors and displaying alerts.
        """
        err = QErrorMessage(self)
        err.showMessage(s) 

    def InitMCValueBtn(self):
        global mctrl_th

        mctrl_th.initVal()

    def onWPStartBtn(self):
        global logdata

        self.ClearCostMap()

        sleep(2)

        nc = NavigationControl()
        nc.control = NavigationControl.START        
        nc.goal_name = self.WPNameCBStr
        nc.mode = NavigationControl.GOAL
        self.naviCtrl_pub.publish(nc)
        mstr = "[WP] Start way point : " + self.WPNameCBStr
        self.print_MessStr(mstr)

        self.wpStart_PB.setDisabled(True)
        self.wpStop_PB.setEnabled(True)

        self.bRunNaviWP = True
        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        # self.runLCDCmd(self.robot_mode)
        self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "Driving"

        self.disableColorPicker()
        
    def onWPStopBtn(self):
        global logdata

        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.goal_name = self.WPNameCBStr
        nc.mode = NavigationControl.NONE
        self.naviCtrl_pub.publish(nc)
        mstr = "[WP] Stop way point : " + self.WPNameCBStr
        self.print_MessStr(mstr)

        self.wpStart_PB.setEnabled(True)
        self.wpStop_PB.setDisabled(True)

        self.bRunNaviWP = False
        if self.bAirPuriRun is True:
            self.robot_mode = "Air_Purifier"
        else:
            self.robot_mode = "None"

        # self.runLCDCmd(self.robot_mode)
        self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "None"

        self.enableColorPicker()


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
        self.print_MessStr(mstr)

        self.naviTrajStart_PB.setDisabled(True)
        self.naviTrajStop_PB.setEnabled(True)

        self.bRunTrajNavi = True        
        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        # self.runLCDCmd(self.robot_mode)
        self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "Driving"

        self.enableColorPicker()

    def onTrajStartBtn(self):
        global logdata

        self.ClearCostMap()

        print("costmap : onTrajStartBtn")

        sleep(2)

        nc = NavigationControl()
        nc.control = NavigationControl.START
        nc.goal_name = self.naviTrajCBStr
        nc.mode = self.trajmode
        self.naviCtrl_pub.publish(nc)

        mstr = "Start trajector : " + self.naviTrajCBStr
        self.print_MessStr(mstr)
        print(mstr)

        self.naviTrajStart_PB.setDisabled(True)
        self.naviTrajStop_PB.setEnabled(True)

        self.bRunTrajNavi = True        
        if self.bAirPuriRun is True:
            self.robot_mode = "Navigation_AP"
        else:
            self.robot_mode = "Navigation_Normal"

        # self.runLCDCmd(self.robot_mode)
        self.SendLedCmd(self.robot_mode)

        print("send led cmd  : onTrajStartBtn ==> robot_mode :" + self.robot_mode)

        # sleep(1)

        # self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "Driving"

        self.disableColorPicker()
        
    def onTrajStopBtn(self):
        global logdata

        nc = NavigationControl()
        nc.control = NavigationControl.STOP
        nc.mode = NavigationControl.NONE
        nc.goal_name = self.naviTrajCBStr
        self.naviCtrl_pub.publish(nc)
        mstr = "Stop trajector : " + self.naviTrajCBStr
        self.print_MessStr(mstr)

        self.naviTrajStart_PB.setEnabled(True)
        self.naviTrajStop_PB.setDisabled(True)

        self.bRunTrajNavi = False
        if self.bAirPuriRun is True:
            self.robot_mode = "Air_Purifier"
        else:
            self.robot_mode = "None"

        # self.runLCDCmd(self.robot_mode)
        self.SendLedCmd(self.robot_mode)

        logdata["robot_status"] = "None"

        self.enableColorPicker()

    def onActivatedAirPurifCB(self, text):
        self.airPurifCBStr = text

        if self.airPurifCBStr is "Off":
            self.AirPurifierVal = purifier_level[self.airPurifCBStr]
            self.setAirPurif(self.AirPurifierVal)

            self.bAirPuriRun = False           
            self.airPurifier_dial.setValue(self.AirPurifierVal)
            self.apval_lcdNumber.display(self.AirPurifierVal)
            self.robotStatus_lineEdit.setText("Air-Purifier OFF")
            self.puriLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

        if self.bAirPuriRun is True:
            self.AirPurifierVal = purifier_level[self.airPurifCBStr]
            self.setAirPurif(self.AirPurifierVal)
            self.airPurifier_dial.setValue(self.AirPurifierVal)
            self.apval_lcdNumber.display(self.AirPurifierVal)
            apstr = "AP : " + self.airPurifCBStr
            self.robotStatus_lineEdit.setText(apstr)
            self.puriLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
        else:
            self.print_MessStr("Air-Purifier is Off.")
            self.puriLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))


    def onActivatedWPNameCB(self, text):
        self.WPNameCBStr = text

    def onActivatednaviTrajCB(self, text):
        self.naviTrajCBStr = text

    def onParkingChargingStationBtn(self):
        global AutoParkNum

        if self.bCSParking == False:
            self.bCSParking = True

            # self.autopark_th.setRunWaypoint(False)

            AutoParkNum = 1
            self.runAutoParkThread(False)

            print("onParkingChargingStationBtn ==> prevbattery_mode:" + self.prevbattery_mode + " battery_mode:" + self.battery_mode)

            # if self.bRunNaviWP is True:
            #     self.onWPStopBtn()

            #     self.ClearCostMap()
            #     sleep(2)

            # if self.bRunTrajNavi is True:
            #     self.onTrajStopBtn()

            #     self.ClearCostMap()
            #     sleep(2)
            # sleep(1)

            # pipeline_1 = rs.pipeline()
            # config_1 = rs.config()
            # config_1.enable_device('137322078739')
            # pipeline_profile = pipeline_1.start(config_1)
            # device = pipeline_profile.get_device()
            # depth_sensor = device.query_sensors()[0]
            # set_emitter = 0
            # depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)
            
            # pipeline_2 = rs.pipeline()
            # config_2 = rs.config()
            # config_2.enable_device('134322073389')
            # pipeline_profile = pipeline_2.start(config_2)
            # device = pipeline_profile.get_device()
            # depth_sensor = device.query_sensors()[0]
            # set_emitter = 0
            # depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)

            # cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 0\" "
            # os.system(cmdstr)
            # sleep(8)
            # cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 0\" "
            # os.system(cmdstr)
            # sleep(8)
                      
            # self.client = actionlib.SimpleActionClient('charging_act', ChargingAction)
            # self.client.wait_for_server()

            # goal = ChargingGoal()
            # print("goal : ")
            # print(goal)
            # self.client.send_goal(goal)

            # self.print_MessStr("Send goal for parking charging station.")
            # self.runPCS_PB.setText('Cancel')
            # self.robot_mode = "Auto_ParkingCS"
            # self.SendLedCmd(self.robot_mode)
            
            # self.bLedMCtrl = False

        else:
            AutoParkNum = 2
            self.runAutoParkThread(False)

            self.bCSParking = False

            # if self.robot_mode != "FullCharging":
            # cmdstr = "python3 ~/bin/target_term -run 10 rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            # cmdstr = "gnome-terminal -- rostopic pub /charging_cancel std_msgs/Bool \"data: true\" -1 "
            # os.system(cmdstr)


            # self.chargingLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            # sleep(5)

            # twist = Twist()
            # twist.linear.x = -0.04; twist.linear.y = 0; twist.linear.z = 0
            # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            # self.pub_twist.publish(twist)
            # print("backward moving[1]...")
            
            # # 2022. 09. 05
            # # sleep(1)

            # # self.pub_twist.publish(twist)
            # # print("backward moving[2]...")

            # sleep(10)


            # twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0
            # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
            # self.pub_twist.publish(twist)

            # print("stop moving...")

            # self.print_MessStr("Cancel parking charge station.")
            # self.runPCS_PB.setText('Parking C.S.')
            # self.robot_mode = "Navigation_Normal"
            # self.SendLedCmd(self.robot_mode)

            # sleep(1)

            # self.onInitPosition()

            # print("Initialize position")

            # sleep(2)

            # cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 1\" "
            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_1/stereo_module \"emitter_enabled: 1\" "
            # os.system(cmdstr)
            # sleep(8)
            # cmdstr = "python3 ~/bin/target_term -run 10 rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 1\" "
            # cmdstr = "gnome-terminal -- rosrun dynamic_reconfigure dynparam set /cam_2/stereo_module \"emitter_enabled: 1\" "
            # os.system(cmdstr)
            # sleep(8)


    def onSaveWPBtn(self):
        
        cmdstr = "gnome-terminal -- rostopic pub -1 save_wp zetabank_msgs/SaveWaypoint -- \"save\" \" + self.WPfile_lineEdit.text() + \" "
        # cmdstr = "python3 ~/bin/target_term -run 10 rostopic pub -1 save_wp zetabank_msgs/SaveWaypoint -- 'save' '" + self.WPfile_lineEdit.text() + "'"
        os.system(cmdstr)

        print(cmdstr)

        self.print_MessStr("Run rostopic pub save_wp")

    def onMakeWPBtn(self):
        if self.bMakeWPRun is False:
            self.bMakeWPRun = True

            cmd = "python3 ~/bin/target_term -run 12 roslaunch make_waypoint makewaypoint.launch"
            try:
                self.makewaypoint_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run make waypoint process...")

            self.print_MessStr("Run roslaunch navigationWayPoint.launch")
            self.runMakeWP_PB.setText('Stop makeWP')
            self.runSaveWP_PB.setEnabled(True)

        else:
            self.bMakeWPRun = False

            try:            
                self.makewaypoint_proc.kill()
            except:
                print("make waypoint process has died...")   

            self.print_MessStr("Stop roslaunch navigationWayPoint.launch")
            self.runMakeWP_PB.setText('Run makeWP')
            self.runSaveWP_PB.setDisabled(True)

    def onNavigationBtn(self):
        if self.bNaviRun is False:
            self.bNaviRun = True

            if self.bReadyDone == True:
                self.bReadyDone = False

            os.system("python3 ~/bin/target_term -run 10 source ~/catkin_ws/devel/setup.bash")

            sleep(2)

            cmd = "python3 ~/bin/target_term -run 13 roslaunch zetabank_navigation normal_navigation.launch"
            try:
                self.navigation_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run navigation process...")

            self.print_MessStr("Run roslaunch normal_navigation.launch")
            self.runNavi_PB.setText('Stop Navigation')
            self.runNaviRVIZ_PB.setEnabled(True)

        else:
            self.bNaviRun = False            

            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /amcl")       
            sleep(0.2)      
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /map_server")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /move_base")            
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /AMCL_particles")            
            sleep(2.0)
            
            try:            
                self.navigation_proc.kill()
            except:
                print("navigation process has died...")   

            sleep(2.0)

            self.print_MessStr("Exit roslaunch normal_navigation.launch")
            self.runNavi_PB.setText('Run Navigation')

            self.runNaviRVIZ_PB.setDisabled(True)

    def onSaveMapBtn(self):
        if self.bGridmapSLAMRun == True or self.bCartographerSLAMRun == True:
            cmdstr = "python3 ~/bin/target_term -run 10 rosrun map_server map_saver -f " + self.mapfile_lineEdit.text()
            os.system(cmdstr)

            print(cmdstr)
            
            self.print_MessStr("Save map data...")
        else:
            self.print_MessStr("Gridmap Node or Cartographer node is not running!")

    def onMultiLRFBtn(self):
        if self.bMultiLRFRun is False:
            self.bMultiLRFRun = True

            cmd = "python3 ~/bin/target_term -run 11 roslaunch ira_laser_tools laserscan_multi_merger.launch"
            try:
                self.lrfmultimege_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run lrfmultimege process...")

            self.print_MessStr("Run roslaunch laserscan_multi_merger.launch")
            self.runMLRF_PB.setText('Stop M-LRF')

            self.runGMSLAM_PB.setEnabled(True)
            self.runCGSLAM_PB.setEnabled(True)
            self.runNavi_PB.setEnabled(True)

        else:
            self.bMultiLRFRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /laserscan_multi_merger")       

            sleep(2.0)         

            try:            
                self.lrfmultimege_proc.kill()
            except:
                print("lrfmultimege process has died...")   

            self.print_MessStr("Exit roslaunch laserscan_multi_merger.launch")
            self.runMLRF_PB.setText('Run M-LRF')

            self.runGMSLAM_PB.setDisabled(True)
            self.runCGSLAM_PB.setDisabled(True)
            self.runNavi_PB.setDisabled(True)

    def onSLAMRVIZBtn(self):
        if self.bSLAMRVIZRun is False:
            self.bSLAMRVIZRun = True
            
            cmd = "python3 ~/bin/target_term -run 9 rosrun rviz rviz -d " + slamrviz_fname
            try:
                self.slamrviz_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run slamrviz process...")

            self.print_MessStr("Run rosrun SLAM rviz")
            self.runSLAMRVIZ_PB.setText('Stop SLAMRVIZ')

            self.runSaveMap_PB.setEnabled(True)
        else:
            self.bSLAMRVIZRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill $(rosnode list | grep rviz_*)")

            sleep(2.0)         

            try:            
                self.slamrviz_proc.kill()
            except:
                print("slamrviz process has died...")   

            self.print_MessStr("Stop rosrun SLAM rviz")
            self.runSLAMRVIZ_PB.setText('Run SLAMRVIZ')

            self.runSaveMap_PB.setDisabled(True)

    def onNaviRVIZBtn(self):
        if self.bNaviRVIZRun is False:
            self.bNaviRVIZRun = True

            if self.bNaviRun == True:
                self.bReadyDone = True
            
            cmd = "python3 ~/bin/target_term -run 8 rosrun rviz rviz -d " + navirviz_fname
            try:
                self.navirviz_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run navirviz process...")

            self.print_MessStr("Run rosrun navigation rviz")
            self.runNaviRVIZ_PB.setText('Stop NaviRVIZ')

            self.runWPCtrl_PB.setEnabled(True)
            self.runMakeWP_PB.setEnabled(True)
            self.runPCS_PB.setEnabled(True)
            self.angle_lcdNumber.setEnabled(True)
            self.getamclpos_PB.setEnabled(True)
            self.chargingmode_checkBox.setEnabled(True)
            self.initangle_PB.setEnabled(True)

            rospy.Subscriber("/heading_angle", Int16, self.recv_HeadingAngle)

        else:
            self.bNaviRVIZRun = False
            os.system("python3 ~/bin/target_term -run 10 rosnode kill $(rosnode list | grep rviz_*)")

            sleep(2.0)         

            try:            
                self.navirviz_proc.kill()
            except:
                print("navirviz process has died...")          

            self.print_MessStr("Stop rosrun navigation rviz")
            self.runNaviRVIZ_PB.setText('Run NaviRVIZ')

            self.runWPCtrl_PB.setDisabled(True)
            self.runMakeWP_PB.setDisabled(True)
            self.runPCS_PB.setDisabled(True)

            self.angle_lcdNumber.setDisabled(True)
            self.getamclpos_PB.setDisabled(True)
            self.chargingmode_checkBox.setDisabled(True)
            self.initangle_PB.setDisabled(True)


    def onWPCtrlBtn(self):
        if self.bWPCRun is False:
            self.bWPCRun = True
            cmd = "python3 ~/bin/target_term -run 7 roslaunch navi_waypoint navigationWayPoint.launch"
            try:
                self.wpctrl_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run wpctrl process...")

            self.print_MessStr("Run roslaunch navigationWayPoint.launch")
            self.runWPCtrl_PB.setText('Stop WPCtrl')       

            self.naviWPName_CB.setEnabled(True)
            self.navitrajName_CB.setEnabled(True)
            self.wpStart_PB.setEnabled(True)
            self.wpStop_PB.setDisabled(True)
            self.naviTrajStart_PB.setEnabled(True)
            self.naviTrajStop_PB.setDisabled(True)
            self.trajmode_checkBox.setEnabled(True)
            self.usesonar_checkBox.setEnabled(True)

        else:
            self.bWPCRun = False
                
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /navigation_waypoints")      

            sleep(2.0)         

            try:            
                self.wpctrl_proc.kill()
            except:
                print("wpctrl process has died...")          

            self.print_MessStr("Stop roslaunch navigationWayPoint.launch")
            self.runWPCtrl_PB.setText('Run WPCtrl')

            self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            self.naviWPName_CB.setDisabled(True)
            self.navitrajName_CB.setDisabled(True)
            self.wpStart_PB.setDisabled(True)
            self.wpStop_PB.setDisabled(True)
            self.naviTrajStart_PB.setDisabled(True)
            self.naviTrajStop_PB.setDisabled(True)
            self.trajmode_checkBox.setDisabled(True)
            self.usesonar_checkBox.setDisabled(True)

    def onCartographerSLAMBtn(self):
        if self.bCartographerSLAMRun is False:
            self.bCartographerSLAMRun = True

            cmd = "python3 ~/bin/target_term -run 6 source /home/zetabank/noncatkin_ws/install_isolated/setup.bash"
            try:
                self.cartographer_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run cartographer process...")

            sleep(2)

            cmd = "python3 ~/bin/target_term -run 6 roslaunch zetabank_slam zetabank_slam_cartographer.launch"
            try:
                self.slamcarto_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run slamcarto process...")

            self.print_MessStr("Run roslaunch zetabank_slam_cartographer.launch")
            self.runCGSLAM_PB.setText('Stop SLAM(CG)')
            self.runSLAMRVIZ_PB.setEnabled(True)
        else:
            self.bCartographerSLAMRun = False

            try:            
                self.cartographer_proc.kill()
            except:
                print("cartographer process has died...")

            sleep(0.5)

            try:            
                self.slamcarto_proc.kill()
            except:
                print("salmcarto process has died...")

            self.print_MessStr("Stop SLAM based on Cartographer.")
            self.runCGSLAM_PB.setText('Run SLAM(CG)')
            self.runSLAMRVIZ_PB.setDisabled(True)


    def onGridmapSLAMBtn(self):
        if self.bGridmapSLAMRun is False:
            self.bGridmapSLAMRun = True

            cmd = "python3 ~/bin/target_term -run 5 roslaunch zetabank_slam zetabank_slam.launch"
            try:
                self.gridmap_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run gridmap process...")

            self.print_MessStr("Run roslaunch zetabank_slam.launch")
            self.runGMSLAM_PB.setText('Stop SLAM(GM)')
            self.runSLAMRVIZ_PB.setEnabled(True)
        else:
            self.bGridmapSLAMRun = False

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zetabank_slam_gmapping")

            try:            
                self.gridmap_proc.kill()
            except:
                print("gridmap_proc process has died...")
            
            self.print_MessStr("Stop SLAM based on Gridmap.")
            self.runGMSLAM_PB.setText('Run SLAM(GM)')
            self.runSLAMRVIZ_PB.setDisabled(True)

    def changeUseSonar(self, state):
        if state == Qt.Checked:
            # cmdstr = "python3 ~/bin/target_term -run 10 rostopic pub /set_sonar std_msgs/Bool \"data: true\" -1 "
            cmdstr = "gnome-terminal -- rostopic pub /set_sonar std_msgs/Bool \"data: true\" -1 "
            os.system(cmdstr)

            self.bUseSonar = True
            self.print_MessStr("Use Sonar Mode : True")
        else:
            # cmdstr = "python3 ~/bin/target_term -run 10 rostopic pub /set_sonar std_msgs/Bool \"data: false\" -1 "
            cmdstr = "gnome-terminal -- rostopic pub /set_sonar std_msgs/Bool \"data: false\" -1 "
            os.system(cmdstr)

            self.bUseSonar = False
            self.print_MessStr("Use Sonar Mode : False")

    def changeAutoPF(self, state):
        if state == Qt.Checked:
            self.bAutoPF = True
            self.print_MessStr("Auto Purifier Mode : True")
        else:
            self.bAutoPF = False
            self.print_MessStr("Auto Purifier Mode : False")
            self.purioff_PB.setDisabled(True)
            self.purion_PB.setEnabled(True)

    def changeTrajMode(self, state):

        if state == Qt.Checked:
            self.trajmode = NavigationControl.LOOP
            self.print_MessStr("Navi Waypoint Mode : Loop")
        else:
            self.trajmode = NavigationControl.GOAL
            self.print_MessStr("Navi Waypoint Mode : Goal")

    def changeCSMode(self, state):
        if state == Qt.Checked:
            self.csmode = True
            self.print_MessStr("C.S Mode : True")
        else:
            self.csmode = False
            self.print_MessStr("C.S Mode : False")

    def changeViewSD(self, state):
        self.VSDCnt = 0

        if state == Qt.Checked:
            self.bViewSonarData = True
        else:
            self.bViewSonarData = False

            self.ur_progressBar.setValue(0.0)
            self.ub_progressBar.setValue(0.0)
            self.ul_progressBar.setValue(0.0)
            self.df_progressBar.setValue(0.0)
            self.dr1_progressBar.setValue(0.0)
            self.dr2_progressBar.setValue(0.0)
            self.db1_progressBar.setValue(0.0)
            self.db2_progressBar.setValue(0.0)
            self.dl1_progressBar.setValue(0.0)
            self.dl2_progressBar.setValue(0.0)

            
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

                self.ur_progressBar.setValue(rimu.data[7])
                self.ub_progressBar.setValue(rimu.data[8])
                self.ul_progressBar.setValue(rimu.data[9])
                self.df_progressBar.setValue(rimu.data[2])
                self.dr1_progressBar.setValue(rimu.data[3])
                self.dr2_progressBar.setValue(rimu.data[4])
                self.db1_progressBar.setValue(rimu.data[5])
                self.db2_progressBar.setValue(rimu.data[6])
                self.dl1_progressBar.setValue(rimu.data[1])
                self.dl2_progressBar.setValue(rimu.data[0])


    def recv_IMU(self, msgs):

        if self.bStartOK == False:
            return

        self.IMUVCnt += 1

        if self.IMUVCnt >= 40:
            self.IMUVCnt = 0

            rollx, pitchy, yawz = self.euler_from_quaternion(msgs.orientation.x, msgs.orientation.y, msgs.orientation.z, msgs.orientation.w)

            self.imu_lcdNumber.display(yawz*MATH_RAD2DEG)


    def LedCmdSendBtn(self):
        # fcolor_mode = self.frontMode_CB.currentText()
        rcolor_mode = self.rearMode_CB.currentText()

        fcolor_mode = "Off"

        fmode = led_mode[fcolor_mode]
        rmode = led_mode[rcolor_mode]

        print("fcolor_mode : %d  rcolor_mode: %d" % (fmode, rmode))

        if fcolor_mode is "Off" or rcolor_mode is "Off":
            self.bLedMCtrl = False
            self.print_MessStr("Change to LED auto control mode.")
        else:
            self.bLedMCtrl = True
            self.print_MessStr("Change to LED manual control mode.")

        fcolor_cmd = UInt64()
        fcolor_cmd = ((((fmode << 24) | self.FrontColor) << 32) | (((rmode << 24) | self.RearColor)))

        self.ledcommand_pub.publish(fcolor_cmd)


    @pyqtSlot()
    # def FrontColorBtn(self):
    #     front_color = "#000000"
    #     fcdlg = QColorDialog(self)
    #     fcdlg.setWindowFlags(Qt.WindowStaysOnTopHint)
    #     if fcdlg.exec_() == fcdlg.Accepted:
    #         front_color = fcdlg.currentColor()
    #     else:
    #         front_color = "#000000"

    #     if front_color:
    #         hexstr = front_color.name()
    #         hexstr = "0x" + hexstr[1:]

    #         self.FrontColor = int(hexstr, 16)

    #         print("front color : %d" % self.FrontColor)

    #         self.frontcolor_PB.setStyleSheet("QPushButton{font-size: 16px;font-family: Arial; background-color: %s;}" % front_color.name())


    @pyqtSlot()
    def RearColorBtn(self):
        rcdlg = QColorDialog(self)
        rcdlg.setWindowFlags(Qt.WindowStaysOnTopHint)

        rear_color = "#000000"

        if rcdlg.exec_() == rcdlg.Accepted:
            rear_color = rcdlg.currentColor()
        else:
            rear_color = "#000000"         

        if rear_color:
            hexstr = rear_color.name()
            hexstr = "0x" + hexstr[1:]
            self.RearColor = int(hexstr, 16)

            print("rear color : %d" % self.RearColor)

            self.rearcolor_PB.setStyleSheet("QPushButton{font-size: 16px;font-family: Arial; background-color: %s;}" % rear_color.name())
            

    def onROSCoreBtn(self):
        if self.bRoscoreRun == False:
            self.bRoscoreRun = True
            cmd = "python3 ~/bin/target_term -run 2 roscore"

            try:
                self.roscore_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run roscore process...")

            self.print_MessStr("Run roscore")
            self.runRoscore_PB.setText('Stop Roscore')

        else:
            self.bRoscoreRun = False

            try:            
                self.roscore_proc.kill()
            except:
                print("roscore process has died...")
            
            self.runRoscore_PB.setText('Run Roscore')
            self.print_MessStr("Stop roscore")

    def onOperationBtn(self):
        if self.bOPRun is False:  
            self.bOPRun = True    

            cmd = "python3 ~/bin/target_term -run 14 roslaunch zetabot_main operate_robot.launch"
            try:
                self.oprobot_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run oprobot process...")

            self.print_MessStr("Run roslaunch operater_robot.launch")

            sleep(0.5)

            cmd = "python3 ~/bin/target_term -run 15 roslaunch autocharge parkingcs.launch"
            try:
                self.autocharge_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run parkingcs process...")

            self.print_MessStr("Run roslaunch parkingcs.launch")

            self.runOP_PB.setText('Stop OP')
            self.runGMSLAM_PB.setEnabled(True)
            self.runCGSLAM_PB.setEnabled(True)
            self.runNavi_PB.setEnabled(True)

        else:
            self.bOPRun = False            
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /initial_pos_srv")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /battery_log")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /autocharge_act_srv")
            sleep(2.0)

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

            self.runGMSLAM_PB.setDisabled(True)
            self.runCGSLAM_PB.setDisabled(True)
            self.runNavi_PB.setDisabled(True)

            self.print_MessStr("Exit operater_robot.launch")
            self.runOP_PB.setText('Run OP')
        

    def onBringupBtn(self):
        if self.bBringUpRun is False:  
            self.bBringUpRun = True      

            cmd = "python3 ~/bin/target_term -run 3 roslaunch zetabank_bringup zetabank_robot.launch"
            try:
                self.rosbringup_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run rosbringup process...")

            self.print_MessStr("Run roslaunch zetabank_robot.launch")
            self.runBringup_PB.setText('Stop Bringup')
            self.runMLRF_PB.setEnabled(True)
            self.runTeleOP_PB.setEnabled(True)
            self.runMotionCtrl_PB.setEnabled(True)
            self.uvcon_PB.setEnabled(True)
            self.pumpon_PB.setEnabled(True)
            self.solvalon_PB.setEnabled(True)
            self.ledcmdsend_PB.setEnabled(True)
            self.viewsonar_checkBox.setEnabled(True)
            self.runOP_PB.setEnabled(True)
            self.webmanager_PB.setEnabled(True)
            self.autopurif_checkBox.setEnabled(True)
            
        else:
            self.bBringUpRun = False      
           
            self.runMLRF_PB.setDisabled(True)
            self.runTeleOP_PB.setDisabled(True)
            self.runMotionCtrl_PB.setDisabled(True)
            self.uvcon_PB.setDisabled(True)
            self.uvcoff_PB.setDisabled(True)
            self.pumpon_PB.setDisabled(True)
            self.pumpoff_PB.setDisabled(True)
            self.solvalon_PB.setDisabled(True)
            self.solvaloff_PB.setDisabled(True)
            self.ledcmdsend_PB.setDisabled(True)
            self.viewsonar_checkBox.setDisabled(True)
            self.runOP_PB.setDisabled(True)
            self.webmanager_PB.setDisabled(True)
            self.autopurif_checkBox.setDisabled(True)

            os.system("python3 ~/bin/target_term -run 10 rosnode kill /cam_1/realsense2_camera")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /cam_1/realsense2_camera_manager")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /cam_2/realsense2_camera")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /cam_2/realsense2_camera_manager")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /depthimage_to_laserscan_cam1")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /depthimage_to_laserscan_cam2")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /robot_state_publisher")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /sick_tim551_2050001")
            sleep(0.2)
            os.system("python3 ~/bin/target_term -run 10 rosnode kill /zeta_mdrobot_BLDC_controller")

            sleep(2.0)

            try:            
                self.rosbringup_proc.kill()
            except:
                print("ros bringup process has died...")

            self.print_MessStr("Exit zetabank_robot.launch")
            self.runBringup_PB.setText('Run Bringup')

    def onMotionCtrlBtn(self):
        global bRunMCtrl
        global mctrl_th

        if self.bmotionCtrl == False:
            self.bmotionCtrl= True
            bRunMCtrl = True

            mctrl_th = Thread_TeleOP(self)

            mctrl_th.print_lspeed.connect(self.printLinearSpeed)
            mctrl_th.print_aspeed.connect(self.printAngleSpeed)
            mctrl_th.print_lsinc.connect(self.printLSInc)
            mctrl_th.print_asinc.connect(self.printASInc)
            mctrl_th.start()
            
            self.runMotionCtrl_PB.setText('Stop MCtrl')
            self.mcinit_PB.setEnabled(True)

            self.print_MessStr("Run motion control thread.")
            
        else:
            self.bmotionCtrl= False
            bRunMCtrl = False
            self.runMotionCtrl_PB.setText('Motion Ctrl')
            self.mcinit_PB.setDisabled(True)
            self.print_MessStr("Stop motion control thread.")


    def onTeleopBtn(self):
        if self.bTeleOPRun is False:
            self.bTeleOPRun = True
            
            cmd = "python3 ~/bin/target_term -run 4 roslaunch teleop_keyandjoy zetabank_teleop_key.launch"
            try:
                self.teleop_proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, close_fds=False)
            except:
                print("Failed to run teleop process...")

            self.print_MessStr("Run roslaunch zetabank_teleop_key.launch")
            self.runTeleOP_PB.setText('Stop teleop')            
        else:
            self.bTeleOPRun = False
            
            os.system("python3 ~/bin/target_term -run 4 xdotool key b")
            print("teleop : send ctrl+c")

            self.print_MessStr("Exit roslaunch zetabank_teleop_key.launch")
            self.runTeleOP_PB.setText('Run teleop')


    def keyReleaseEvent(self, event):
        global convkey
        global prev_convkey
        global convkey_val

        if convkey_val == event.key() and not event.isAutoRepeat():
            prev_convkey = convkey
            convkey = 'r'

            print("release key:" + str(event.key()) + " convkey:" + str(convkey) + " prev_convkey :" + str(prev_convkey))

    
    def keyPressEvent(self, event):
        global bRunMCtrl
        global convkey
        global convkey_val

        if(bRunMCtrl == True):

            key = event.key()
            convkey_val = key
            
            
            if TeleOP_Cont == True:
            
                if key == Qt.Key_W :
                    convkey = 'w'                
                elif key == Qt.Key_X :
                    convkey = 'x'                
                elif key == Qt.Key_A :
                    convkey = 'a'                
                elif key == Qt.Key_D :
                    convkey = 'd'
                elif key == Qt.Key_U :
                    convkey = 'u'
                elif key == Qt.Key_M :
                    convkey = 'm'
                elif key == Qt.Key_I :
                    convkey = 'i'
                elif key == Qt.Key_O :
                    convkey = 'o'                    
                elif key == Qt.Key_Comma :
                    convkey = ','
                elif key == Qt.Key_Period :
                    convkey = '.'
                elif key == Qt.Key_Space or key == Qt.Key_S :
                    convkey = 's'
                else:
                    convkey = 0

            else:

                if key == Qt.Key_W :
                    self.target_linear_vel = self.target_linear_vel + self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                elif key == Qt.Key_X :
                    self.target_linear_vel = self.target_linear_vel - self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                elif key == Qt.Key_A :
                    self.target_angular_vel = self.target_angular_vel + self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                elif key == Qt.Key_D :
                    self.target_angular_vel = self.target_angular_vel - self.incdec_val
                    self.status = self.status + 1
                    mstr = self.vels(self.target_linear_vel,self.target_angular_vel)
                    self.print_MessStr(mstr)
                elif key == Qt.Key_Space or key == Qt.Key_S :
                    self.target_linear_vel   = 0
                    self.control_linear_vel  = 0
                    self.target_angular_vel  = 0
                    self.control_angular_vel = 0
                    mstr = self.vels(0, 0)
                    self.print_MessStr(mstr)

                if self.target_linear_vel > self.control_linear_vel:
                    self.control_linear_vel = min( self.target_linear_vel, self.control_linear_vel + (0.1/4.0) )
                else:
                    self.control_linear_vel = self.target_linear_vel

                if self.target_angular_vel > self.control_angular_vel:
                    self.control_angular_vel = min( self.target_angular_vel, self.control_angular_vel + (0.1/4.0) )
                else:
                    self.control_angular_vel = self.target_angular_vel

                twist = Twist()
                twist.linear.x = self.control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_angular_vel
                self.pub_twist.publish(twist)

    def setAirPurif(self, val):
        self.AirPurifierVal = val
        purifier_val = UInt16()
        purifier_val.data = self.AirPurifierVal
        self.air_purifier_pub.publish(purifier_val)

        self.apval_lcdNumber.display(self.AirPurifierVal)


    def setAPVal(self):
        self.setAirPurif(self.airPurifier_dial.value())

    def purifier_control(self, val):
        self.AirPurifierVal = val
        self.airPurifier_dial.setValue(self.AirPurifierVal)
        purifier_val = UInt16()
        purifier_val.data = self.AirPurifierVal
        self.apval_lcdNumber.display(self.AirPurifierVal)

        self.air_purifier_pub.publish(purifier_val)


    def onPuriOnBtn(self):
        global air_result

        if self.bAirPuriRun is False:
            self.bAirPuriRun = True

            self.purifier_control(400)

            self.robotStatus_lineEdit.setText("Air-Purifier ON")

            self.puriLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "Air_Purifier"

            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)

            self.purioff_PB.setEnabled(True)
            self.airpurifval_CB.setEnabled(True)            
            self.purion_PB.setDisabled(True)


    def onPuriOffBtn(self):
        global air_result

        if self.bAirPuriRun is True:
            self.bAirPuriRun = False

            self.purifier_control(0)

            self.robotStatus_lineEdit.setText("Air-Purifier OFF")

            self.puriLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)

            self.purioff_PB.setDisabled(True)
            self.airpurifval_CB.setDisabled(True)            
            self.purion_PB.setEnabled(True)

    def onSolValveOnBtn(self):
        if self.bBringUpRun is False:  
            self.print_MessStr("[Error] : The bringup node did not run.")

        elif self.bSolRun == False:
            self.bSolRun = True
            sol_control_msg = PowerControlMsgs()
            sol_control_msg.port = 14
            sol_control_msg.state = True
            self.power_control_pub.publish(sol_control_msg)

            self.robotStatus_lineEdit.setText("Pump ON")

            self.pumpLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "Sol"

            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)

            self.solvalon_PB.setDisabled(True)
            self.solvaloff_PB.setEnabled(True)

    def onSolValveOffBtn(self):
        if self.bBringUpRun is False:  
            self.print_MessStr("[Error] : The bringup node did not run.")

        elif self.bSolRun == True:
            self.bSolRun = False
            sol_control_msg = PowerControlMsgs()
            sol_control_msg.port = 14
            sol_control_msg.state = False
            self.power_control_pub.publish(sol_control_msg)

            self.robotStatus_lineEdit.setText("Sol OFF")

            self.solLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)

            self.solvalon_PB.setEnabled(True)
            self.solvaloff_PB.setDisabled(True)


    def onPumpOnBtn(self):
        if self.bBringUpRun is False:  
            self.print_MessStr("[Error] : The bringup node did not run.")

        elif self.bPumpRun == False:
            self.bPumpRun = True
            pump_control_msg = PowerControlMsgs()
            pump_control_msg.port = 5
            pump_control_msg.state = True
            self.power_control_pub.publish(pump_control_msg)

            self.robotStatus_lineEdit.setText("Pump ON")

            self.pumpLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "Pump"

            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)

            self.pumpon_PB.setDisabled(True)
            self.pumpoff_PB.setEnabled(True)

    def onPumpOffBtn(self):
        if self.bBringUpRun is False:  
            self.print_MessStr("[Error] : The bringup node did not run.")

        elif self.bPumpRun == True:
            self.bPumpRun = False
            pump_control_msg = PowerControlMsgs()
            pump_control_msg.port = 5
            pump_control_msg.state = False
            self.power_control_pub.publish(pump_control_msg)

            self.robotStatus_lineEdit.setText("Pump OFF")

            self.pumpLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)

            self.pumpon_PB.setEnabled(True)
            self.pumpoff_PB.setDisabled(True)


    def onUVCOnBtn(self):
        if self.bBringUpRun is False:  
            self.print_MessStr("[Error] : The bringup node did not run.")

        elif self.bUVCRun == False:
            self.bUVCRun = True
            uvc_control_msg = PowerControlMsgs()
            uvc_control_msg.port = 15
            uvc_control_msg.state = True
            self.power_control_pub.publish(uvc_control_msg)

            self.robotStatus_lineEdit.setText("UVC ON")

            self.uvcLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_AP"
            else:
                self.robot_mode = "UVC"

            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)

            self.uvcon_PB.setDisabled(True)
            self.uvcoff_PB.setEnabled(True)


    def onUVCOffBtn(self):
        if self.bBringUpRun is False:  
            self.print_MessStr("[Error] : The bringup node did not run.")

        elif self.bUVCRun == True:
            self.bUVCRun = False
            uvc_control_msg = PowerControlMsgs()
            uvc_control_msg.port = 15
            uvc_control_msg.state = False
            self.power_control_pub.publish(uvc_control_msg)

            self.robotStatus_lineEdit.setText("UVC OFF")

            self.uvcLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))

            if self.bRunNaviWP is True or self.bRunTrajNavi is True:
                self.robot_mode = "Navigation_Normal"
            else:
                self.robot_mode = "None"

            # self.runLCDCmd(self.robot_mode)
            self.SendLedCmd(self.robot_mode)

            self.uvcon_PB.setEnabled(True)
            self.uvcoff_PB.setDisabled(True)

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

            self.batvol_lcdNumber.display(self.Bat_AvgVoltage)
            self.batcur_lcdNumber.display(self.Bat_AvgCurrent)
            self.batsoc_lcdNumber.display(self.Bat_AvgSOC)     

            if self.robot_mode == "Charging":
                # print("battery : charging")
                # print("charging [1] ==> prevbattery_mode:" + self.prevbattery_mode + " battery_mode:" + self.battery_mode)                
                try:
                    if self.Bat_AvgSOC <= (self.battery_low - self.hysteresis_low):
                        self.hysteresis_low = 0.0
                        # self.hysteresis_low = self.hysteresis_val
                        # self.prevbattery_mode = self.battery_mode
                        self.battery_mode = "Charging-low-bat"
                        self.bLowBat = True
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))
                        # self.bprevViewCmd = self.bViewCmd
                        # self.bViewCmd = False
                    elif (self.battery_low + self.hysteresis_low) <= self.Bat_AvgSOC < (self.battery_middle - self.hysteresis_middle):
                        self.hysteresis_low = 0.0
                        self.hysteresis_middle = 0.0
                        # self.hysteresis_middle = self.hysteresis_val
                        # self.prevbattery_mode = self.battery_mode
                        self.battery_mode = "Charging-middle-bat"
                        self.bLowBat = False
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_ORANGE_LED))
                        # self.bprevViewCmd = self.bViewCmd
                        # self.bViewCmd = False
                    elif (self.battery_middle + self.hysteresis_middle) <= self.Bat_AvgSOC< (self.battery_full - self.hysteresis_high):
                        self.hysteresis_middle = 0.0
                        self.hysteresis_high = 0.0
                        # self.hysteresis_high = self.hysteresis_val
                        # self.prevbattery_mode = self.battery_mode
                        self.battery_mode = "Charging-high-bat"
                        self.bLowBat = False
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
                        # self.bprevViewCmd = self.bViewCmd
                        # self.bViewCmd = False
                    elif self.battery_full <= self.Bat_AvgSOC:          
                        # self.prevbattery_mode = self.battery_mode                          
                        self.battery_mode = "Charging-full-bat"
                        self.bLowBat = False
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
                        # self.bprevViewCmd = self.bViewCmd
                        # self.bViewCmd = False
                    else:
                        # self.prevbattery_mode = self.battery_mode
                        self.battery_mode = "None"            
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
                    
                    # print("charging [2] ==> prevbattery_mode:" + self.prevbattery_mode + " battery_mode:" + self.battery_mode)
                    if  self.prevbattery_mode != self.battery_mode:
                    # if  self.bViewCmd == False and self.bprevViewCmd != self.bViewCmd:
                        # self.runLCDCmd(self.battery_mode)
                        self.SendLedCmd(self.battery_mode)
                        self.prevbattery_mode = self.battery_mode  
                        print("Charging [3]: change battery mode ==> SendLedCmd")
                        # self.bprevViewCmd = self.bViewCmd
                        # self.bViewCmd = True
                except:
                    self.print_MessStr("[Error] : Led control.")

            else:
                # print("battery : no charging" + "bat soc:" + str(self.Bat_AvgSOC) + "bat_full:" +str(self.battery_full))
                try:
                    if self.Bat_AvgSOC <= (self.battery_low - self.hysteresis_low):
                        self.hysteresis_low = 0.0
                        # self.hysteresis_low = self.hysteresis_val
                        self.bLowBat = True
                         # self.prevbattery_mode = self.battery_mode
                        self.battery_mode = "Charging-low-bat"
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))
                        self.robotStatus_lineEdit.setText("Low Battery!!!")
                    elif (self.battery_low + self.hysteresis_low) <= self.Bat_AvgSOC < (self.battery_middle - self.hysteresis_middle):
                        self.hysteresis_low = 0.0
                        self.hysteresis_middle = 0.0
                        # self.hysteresis_middle = self.hysteresis_val
                        self.bLowBat = False
                        # self.prevbattery_mode = self.battery_mode
                        self.battery_mode = "Charging-middle-bat"
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_ORANGE_LED))
                        self.robotStatus_lineEdit.setText("Middle Battery!!!")
                    elif (self.battery_middle + self.hysteresis_middle) <= self.Bat_AvgSOC< (self.battery_full - self.hysteresis_high):
                        self.hysteresis_middle = 0.0
                        self.hysteresis_high = 0.0
                        # self.hysteresis_high = self.hysteresis_val
                        # self.prevbattery_mode = self.battery_mode                        
                        self.battery_mode = "Charging-high-bat"
                        self.bLowBat = False
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
                        self.robotStatus_lineEdit.setText("Normal Battery!!!")
                        # print("inside normal bat")
                    elif self.battery_full <= self.Bat_AvgSOC:            
                        self.bLowBat = False
                        # self.prevbattery_mode = self.battery_mode                        
                        self.battery_mode = "Charging-full-bat"
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
                        self.robotStatus_lineEdit.setText("Full Battery!!!")
                        # print("inside full bat")
                    else:
                        self.battery_mode = "None"            
                        self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
                        # print("inside none")

                    # print("charging [3]")
                    # if  self.prevbattery_mode != self.battery_mode:
                    #     self.SendLedCmd(self.battery_mode)
                    #     print("Non-Charging [4]: change battery mode ==> SendLedCmd")                                
        
                except:
                    pass

            if self.robot_mode == "FullCharging" and self.prevbattery_mode != self.battery_mode:
                self.bLowBat = False
                self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
                self.robotStatus_lineEdit.setText("Full Battery!!!")
                # print("charging [4] ==> prevbattery_mode:" + self.prevbattery_mode + " battery_mode:" + self.battery_mode)
                self.prevbattery_mode = self.battery_mode    
                self.battery_mode = "Charging-full-bat"

                # self.bprevViewCmd = self.bViewCmd
                # self.bViewCmd = False
                # self.runLCDCmd(self.battery_mode)
                self.SendLedCmd(self.battery_mode)
                # self.bprevViewCmd = self.bViewCmd
                # self.bViewCmd = True

                print("Full charging mode...")
                # print("inside full bat --2")

            # 2022. 09 .07
            # if self.bLowBat == True and self.bReadyDone == True and self.csmode == True:
            #     self.robotStatus_lineEdit.setText("Low Battery!!!")

            #     if self.bRunNaviWP is True:
            #         self.onWPStopBtn()
            #     elif self.bRunTrajNavi is True:
            #         self.onTrajStopBtn()
                
            #     if self.bCSParking == False and self.bLowbatPark == False:
            #         self.bLowbatPark = True
            #         self.print_MessStr("The battery voltage is low, so it will start auto-charging now.")
            #         self.onParkingChargingStationBtn()

            #     self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))

            # else:
            #    self.robotStatus_lineEdit.setText("Normal Battery!!!")

            #    self.lowbatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

    def CallbackRunNaviCtrlStatus(self, request):
        if self.bStartOK == False:
            return

        if self.bWPCRun is True:
            self.naviStatus = request.status

            if self.bRunAutoParking == True:
                self.autopark_th.navi_runstat.emit(request.status, request.status_description)


            if self.bEStop is False:
                mstr = request.status_description
                self.robotStatus_lineEdit.setText(mstr)

            if self.naviStatus is NavigationControlStatus.ABORTED or self.naviStatus is NavigationControlStatus.ERROR or self.naviStatus is NavigationControlStatus.ERRGTGF:
              self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_RED_LED))
            elif self.naviStatus is NavigationControlStatus.RUNNING:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_BLUE_LED))
            elif self.naviStatus is NavigationControlStatus.COMPLETED:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))
                if self.bRunNaviWP is True:
                    self.bRunNaviWP = False
                    
                    self.wpStart_PB.setEnabled(True)
                    self.wpStop_PB.setDisabled(True)

                    self.robot_mode = "None"
                    # self.runLCDCmd(self.robot_mode)
                    self.SendLedCmd(self.robot_mode)

            elif self.naviStatus is NavigationControlStatus.TRAJCOMPLETED:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GREEN_LED))

                if self.bRunTrajNavi is True:
                    self.bRunTrajNavi = False

                    self.naviTrajStart_PB.setEnabled(True)
                    self.naviTrajStop_PB.setDisabled(True)

                    self.robot_mode = "None"
                    # self.runLCDCmd(self.robot_mode)
                    self.SendLedCmd(self.robot_mode)

            elif self.naviStatus is NavigationControlStatus.CANCELLED or self.naviStatus is NavigationControlStatus.WARNNRGTO:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_YELLOW_LED))
            elif self.naviStatus is NavigationControlStatus.IDLING:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
            elif self.naviStatus is NavigationControlStatus.REAROBSTACLE:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_ORANGE_LED))
            else:
                self.navistatLED_label.setPixmap(QtGui.QPixmap(ICON_GRAY_LED))
            

    def print_MessStr(self, str):
        self.messLV.appendRow(QStandardItem(str))
        self.messbox_listView.setModel(self.messLV)
        self.messbox_listView.scrollToBottom()

    def onExit(self):
        if self.bReady == True:
            self.print_MessStr("Exit running ros packages...")
            self.ReadyBtn()

        self.print_MessStr("Exit program...")

        # self.runLCDCmd("None")
        self.SendLedCmd("None")
        
        os.system("python3 ~/bin/target_term -run 10 rosnode kill /PWR")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 10 rosnode kill /STM")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 10 rosnode kill /stm_starter")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 10 rosnode kill /AIR")
        
        sleep(2.0)

        try:            
            self.self.rosserial_proc.kill()
        except:
            print("ros serial process has died...")  

        # self.rosserial_proc.send_signal(SIGINT)
        # print("send SIGINT signal")

        os.system("python3 ~/bin/target_term -run 10 rosnode kill /rosout")

        sleep(1.0)        

        os.system("python3 ~/bin/target_term -run 1 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 2 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 3 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 4 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 5 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 6 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 7 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 8 exit") 
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 9 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 10 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 11 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 12 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 13 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 14 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 15 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 16 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 17 exit")
        sleep(0.5)
        os.system("python3 ~/bin/target_term -run 18 exit")
        # sleep(0.5)
        # os.system("python3 ~/bin/target_term -run 19 exit")

        self.navictrl_status_sub.unregister()
        self.bat_sub.unregister()
        self.imu_sub.unregister()
        self.sonar_sub.unregister()
        self.charge_state_sub.unregister()
        self.estop_sub.unregister()
        self.air_sub.unregister()
        self.robotpos_sub.unregister()

        # os.system("python3 ~/bin/target_term -run 10 rosnode kill /ZBMRGS")
        # sleep(0.5)

        self.close()


    def closeEvent(self, event):
        self.deleteLater()
        event.accept()

# app = QtCore.QCoreApplication(sys.argv)
app = QApplication(sys.argv)
window = MyWindow()
window.show()
sys.exit(app.exec_())

