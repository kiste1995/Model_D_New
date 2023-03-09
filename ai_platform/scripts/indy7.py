#!/usr/bin/env python
# coding: utf-8

# ## khsim 20201008 - check maximum joint move and make it dance!


from indy_utils import indydcp_client as client

import json
from time import sleep
import threading
import numpy as np

robot_ip = "192.168.0.6"  # Robot (Indy) IP
robot_name = "NRMK-Indy7"  # Robot name (Indy7)

indy = client.IndyDCPClient(robot_ip, robot_name)


# all the j_pos become 0.0
def goto_zero_pos():
    indy.go_zero()
    indy.wait_for_move_finish()


def goto_home_pos():
    indy.go_home()
    indy.wait_for_move_finish()


def goto_joint_pos(j_pos):
    indy.joint_move_to(j_pos)
    indy.wait_for_move_finish()


def print_cur_pos():
    j_pos = indy.get_joint_pos()
    t_pos = indy.get_task_pos()
    print("- Joint position: ", j_pos)
    print("- Task position :", t_pos)


def get_cur_j_pos():
    j_pos = indy.get_joint_pos()
    print("- Joint position: ", j_pos)
    return j_pos


def get_cur_t_pos():
    t_pos = indy.get_task_pos()
    print("- Task position :", t_pos)
    return t_pos


def dance_joint(i, j_pos, min_f, max_f):
    if i < 0 or i > 5:
       print("* range out")
       return
    j_pos[i] = min_f
    goto_joint_pos(j_pos)
    j_pos[i] = max_f
    goto_joint_pos(j_pos)
    j_pos[i] = 0.0
    goto_joint_pos(j_pos)


def dance_joints(j0_pos, j1_pos, j2_pos, j3_pos, j4_pos, j5_pos):
    j_pos = [j0_pos, j1_pos, j2_pos, j3_pos, j4_pos, j5_pos]
    goto_joint_pos(j_pos)


def moveto(j_pos):
    #indy.connect()
    goto_joint_pos(j_pos)
    indy.wait_for_move_finish()
    #indy.disconnect()
# !!! be careful that indy.connect()/disconnect() pair is not nested !!!
def dance():
    #goto_zero_pos()
    #print_cur_pos()
    #for i in range(-4,5):
        #print(i)
    #moveto([90,60,0,0,0,0])
    moveto([0, 65.07874039579457, -65.0, 0, 0, 0])
    j_cur  = indy.get_joint_pos()
    moveto([j_cur[0],j_cur[1],-35,j_cur[3],j_cur[4],j_cur[5]])
    moveto([j_cur[0],j_cur[1],-95,j_cur[3],j_cur[4],j_cur[5]])
    moveto([j_cur[0],j_cur[1],-35,j_cur[3],j_cur[4],j_cur[5]])
    moveto([j_cur[0],j_cur[1],-95,j_cur[3],j_cur[4],j_cur[5]])
    moveto([0, 65.07874039579457, -65.0, 0, 0, 0])
        #sleep(0.1)
    #check_max_range()
    #goto_home_pos()
    



def main():
    indy.connect()
    dance()
    indy.disconnect()


if __name__ == "__main__":
    main()


# #### EOF #####
