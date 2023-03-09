#! /usr/bin/env python

import rospy

import pygame
import time
import sys, tty, select, termios, os

from std_msgs.msg import Bool
from darknet_ros_msgs.msg import BoundingBoxesCenter

from indy_utils import indydcp_client as client

import json
import threading
import numpy as np

robot_ip = "192.168.0.6"  # Robot (Indy) IP
robot_name = "NRMK-Indy7"  # Robot name (Indy7)

#indy = client.IndyDCPClient(robot_ip, robot_name)
file_dir = os.path.dirname(__file__)

rospy.init_node("say_something")

# detect_class = rospy.get_param("~detect_class")
detect_dis = 1000
detect_class = 'person'
# detect_class = rospy.get_param("~detect_class")

class RosMaker(object):
    def __init__(self,type,topic_name,message_type,callback = None):
        self._topic_name = topic_name
        self._message_type = message_type
        if type == "pub" :
            self._pub = rospy.Publisher(self._topic_name,self._message_type,queue_size=10)

        elif type == "sub" :
            if callback == None :
                callback = self.callback
            self.msg = message_type()
            self._sub = rospy.Subscriber(self._topic_name,self._message_type,callback)


    def publish(self,msgs):
        self._pub.publish(msgs)


class SaySomething(object) :
    def __init__(self) :
        self.hello = file_dir + "/sound/welcome.mp3"   # mp3 or mid file
        self.panpare = file_dir + "/sound/panpare.mp3"
        self.clap = file_dir + "/sound/clap.wav"
        self.nice2meetU = file_dir + '/sound/niceToMeetU.mp3'
        self.bgm1 = file_dir + '/sound/bgm.mp3'
        self.bgm2 = file_dir + '/sound/bgm2.mp3'

        self.sound_1 = file_dir + '/sound/prevention.mp3'
        self.sound_2 = file_dir + '/sound/mask.mp3'


        self.hello_flag = False
        self.hello_done = False
        self.face_get_cnt = 0
        self.face_loss_cnt = 0

        self.person_5_flag = False
        self.person_5_done = False
        self.person_5_get_cnt = 0
        self.person_5_loss_cnt = 0


        self.switch_flag = 0

        self.settings = termios.tcgetattr(sys.stdin)

        self.depth_sub = RosMaker("sub","/darknet_depth",BoundingBoxesCenter,self.depthSubCallback)
        self.initMixer()
	self.say_pub = RosMaker("pub","/speak",Bool)

        self.say_hello = None

        # pygame.init()
        # pygame.mixer.init()hello_done
        # print("init_done")


    def say(self) :
        key = 0
        try : 
            print("say_start")
            while (rospy.is_shutdown) :
                time.sleep(0.2)
                # while key == 0 :
                #     key__ = getKey()
                #     if key__ and int(key__) != 0 :
                #         break

                # if key == 1 :
                #     print("key111111111")
                #     filename = 'speech_20210210015806285.mp3'
                # elif key == 2 :
                #     print("key22222222222")
                #     filename = 'speech_20210218053305942.mp3'
                #if self.person_5_flag and self.person_5_done == False :
                #    os.system("mplayer "+ self.sijang_5)

                #    self.person_5_done = True
                #    print("person_5_done")


                if self.hello_flag and self.hello_done == False :
                    # print("play_hello")
                    # self.playmusic(self.hello)
                    # time.sleep(1)
                    
                    if self.switch_flag == 0 :
			self.speak_pub(True)
                        os.system("mplayer "+ self.sound_1)
                        self.switch_flag = 1
			self.speak_pub(False)
                    elif self.switch_flag == 1 :
			self.speak_pub(True)
                        os.system("mplayer "+ self.sound_2)
                        self.switch_flag = 0
			self.speak_pub(False)

                    self.hello_done = True
                    print("hello_done")

                # key = playmusic(filename)
                # print("\nPlay Stopped by user")
        except KeyboardInterrupt :
            print("exit")
            indy.disconnect()
            sys.exit()

    def play_say(self,say_pygame):
        self.hello_sound.play()
        while pygame.mixer.get_busy():
            print("Playing... - func => playingmusic")
            self.clock.tick(1000)

        print("playmusic")
        return 'done'

    def depthSubCallback(self,msg) :
        global detect_class
        global detect_dis
        # print(msg)
        minimum = 9999999
        person_cnt = 0
        # print(msg.bounding_boxes_center)
        for i in msg.bounding_boxes_center :
            if i.minimum <= detect_dis :
                person_cnt += 1
            if i.Class == detect_class and minimum > i.minimum :
                minimum = i.minimum
            #    print(minimum)
        if (msg.bounding_boxes_center ==[] or minimum >= detect_dis):
            self.face_loss_cnt += 1
            self.person_5_loss_cnt += 1


        elif person_cnt >= 2 :
            self.person_5_get_cnt += 1
            self.person_5_loss_cnt = 0
            self.face_get_cnt = 0

        elif minimum <= detect_dis :
            self.face_get_cnt += 1
            self.face_loss_cnt = 0
            self.person_5_loss_cnt += 1



        if self.person_5_get_cnt >= 5 and self.person_5_flag == False and self.hello_flag == False :
            self.person_5_flag = True


        elif self.person_5_loss_cnt >= 5 :
            self.person_5_get_cnt = 0

            self.person_5_flag = False
            self.person_5_done = False



        if self.face_get_cnt >= 5 and self.hello_flag == False and self.person_5_flag == False :
            self.hello_flag = True


        elif self.face_loss_cnt >= 5 :
            self.face_get_cnt = 0

            self.hello_flag = False
            self.hello_done = False



        if self.face_get_cnt%10 == 0 and self.face_get_cnt != 0 :
            print("face_get_cnt",self.face_get_cnt)
        if self.person_5_get_cnt%10 == 0 and self.person_5_get_cnt != 0 :
            print("person_5_get_cnt",self.person_5_get_cnt)
        # print("loss_cnt",self.face_loss_cnt)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            _key = sys.stdin.read(1)
        else:
            _key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return _key
    
    def playsound(self,soundfile):
        """Play sound through default mixer channel in blocking manner.
        This will load the whole sound into memory before playback
        """    
        pygame.init()
        pygame.mixer.init()
        sound = pygame.mixer.Sound(soundfile)
        clock = pygame.time.Clock()
        sound.play()
        while pygame.mixer.get_busy():
            print("Playing... - func => playsound")
            key = self.getKey()
            print(key)
            if key:
                self.stopmusic()
                return int(key)
            clock.tick(1000)
    
    def playmusic(self,soundfile):
        """Stream music with mixer.music module in blocking manner.
        This will stream the sound from disk while playing.
        """
        
        pygame.init()
        pygame.mixer.init()
        clock = pygame.time.Clock()
        pygame.mixer.music.load(soundfile)
        time.sleep(2)
        pygame.mixer.music.play()
        print("playmusic")
        print(pygame.mixer.music.get_busy())
        while pygame.mixer.music.get_busy():
            
            print("Playing... - func => playingmusic")
            # key = self.getKey()
            # print("key 1 :",key)
            # if key:
            #     self.stopmusic()
            #     return int(key)
            clock.tick(1000)
        print("playmusic")
        return 'done'

    def stopmusic(self):
        """stop currently playing music"""
        pygame.mixer.music.stop()
    
    def getmixerargs(self):
        pygame.mixer.init()
        freq, size, chan = pygame.mixer.get_init()
        return freq, size, chan
    
    def initMixer(self):
        BUFFER = 3072  # audio buffer size, number of samples since pygame 1.8.
        FREQ, SIZE, CHAN = self.getmixerargs()
        pygame.mixer.init(FREQ, SIZE, CHAN, BUFFER)

say = SaySomething()

def goto_zero_pos():
    indy.go_zero()
    indy.wait_for_move_finish()

def moveto(j_pos):
    #indy.connect()
    goto_joint_pos(j_pos)
    indy.wait_for_move_finish()
    #indy.disconnect()
# !!! be careful that indy.connect()/disconnect() pair is not nested !!!

def goto_joint_pos(j_pos):
    indy.joint_move_to(j_pos)
    indy.wait_for_move_finish()

def dance():
    global say
    #goto_zero_pos()
    #print_cur_pos()
    #for i in range(-4,5):
        #print(i)
    #moveto([90,60,0,0,0,0])
    status = indy.get_robot_status()
    if status['collision'] :
        indy.reset_robot()
    moveto([120, 65.07874039579457, -65.0, 0, 0, 0])
    print("dance_ready")
    while True:
        status = indy.get_robot_status()
        j_cur  = indy.get_joint_pos()
        if status['collision'] :
            indy.reset_robot()
        if say.hello_flag and say.hello_done == False:
            moveto([j_cur[0],j_cur[1],-45,j_cur[3],j_cur[4],j_cur[5]])
            moveto([j_cur[0],j_cur[1],-85,j_cur[3],j_cur[4],j_cur[5]])

        elif not(-66 <= j_cur[2] <= -64)  :
            print("not_center")
            moveto([120, 65.07874039579457, -65.0, 0, 0, 0])
            
        time.sleep(0.1)
        #sleep(0.1)
    #check_max_range()
    #goto_home_pos()


    
    
    '''You definitely need test mp3 file (a.mp3 in example) in a directory, say under 'C:\\Temp'
    * To play wav format file instead of mp3, 
        1) replace a.mp3 file with it, say 'a.wav'
        2) In try except clause below replace "playmusic()" with "playsound()"
        
    '''

def main():
    
    print(type(detect_dis))
    print("ready")
    # indy.connect()
    # moveto([120, 65.07874039579457, -65.0, 0, 0, 0])
    
    # dance_thread = threading.Thread(target=dance)
    # dance_thread.daemon = True
    # dance_thread.start()

    say.say()

    rospy.spin()



        # except Exception:
        #     print("unknown error")
        
    print("Done")

if __name__ == "__main__" :
    main()
