#!/usr/bin/env python
import rospy
from socket import *
from time import sleep

rospy.init_node("jetson_starter")

clientSock = socket(AF_INET, SOCK_STREAM)

connected = False

while not connected :
    try :
        clientSock.connect(('192.168.112.21',8080))
        connected = True
        print("connected!!")
    except :
        sleep(1)


recv_msg = None

msg = 'ready'
clientSock.send(msg.encode('utf-8'))

while not(recv_msg == 'start') :
    if recv_msg == 'ready' :
        print(recv_msg)
    recv_msg = clientSock.recv(1024)

    sleep(0.1)
    
print(recv_msg)
