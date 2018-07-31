# -*- coding: utf-8 -*-
#!/usr/bin/env python
"""
@author: Bogdan Duchevski.
"""
#Import Libraries:
import vrep
import sys,getch
import time
import numpy as np
import math
import matplotlib as mpl
from pathlib import Path

from kb_h_cl import KBHit
import os,time
os = 'LINUX'
import sys
import termios
import atexit
from select import select
import rospy, os
from pathlib import Path
from std_msgs.msg import Float32

# - INIT
dir=0
ESC = 27
acc_dec = False
sel = None
desiredWheelRotSpeed = 0
desiredWheelRotSpeed2 = 0
d_ar=[]
spd = 1 # new speed mode
save_spd = None

def nav_prem(dt,dir_x):
    global desiredWheelRotSpeed
    global desiredWheelRotSpeed2
    global d_ar
    global save_spd
    wheelRotSpeedDx=20*math.pi/180
    if dt[0]*dt[1]<0:
        # TURN
        if save_spd == None:
            save_spd = desiredWheelRotSpeed
        desiredWheelRotSpeed=desiredWheelRotSpeed+wheelRotSpeedDx*dt[0]
        desiredWheelRotSpeed2=desiredWheelRotSpeed2+wheelRotSpeedDx*dt[1]
        d_ar=[desiredWheelRotSpeed,desiredWheelRotSpeed2]
    else:
        # DRIVE
        if save_spd is not None:
            desiredWheelRotSpeed=save_spd+wheelRotSpeedDx*dt[0]
            desiredWheelRotSpeed2=save_spd+wheelRotSpeedDx*dt[1]
            save_spd = None
        else:
            desiredWheelRotSpeed=desiredWheelRotSpeed+wheelRotSpeedDx*dt[0]
            desiredWheelRotSpeed2=desiredWheelRotSpeed2+wheelRotSpeedDx*dt[1]
        # FORWARD
        # ENC RESET !!!!!!
        d_ar=[desiredWheelRotSpeed,desiredWheelRotSpeed]
    return d_ar

def process(key):
    global dir
    cmnd = []
    nav_prem_res = []
    dir_k = None
    if key == 'x':
        exit('exitting')
    elif key == 'w':
        dir_k = 1
        cmnd = [1,1,1]
    elif key == 's':
        dir_k = -1
        cmnd = [-1,-1,-1]
    elif key == 'd':
        cmnd = [1,-1,0]
    elif key == 'a':
        cmnd = [-1,1,0]
    if len(cmnd)>0:
        nav_prem_res = nav_prem(cmnd,dir_k)
        #if dir_k is not None and dir*dir_k>0:
        #    dir = dir_k
    return nav_prem_res

def int_chk(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def talker():
    global desiredWheelRotSpeed
    global desiredWheelRotSpeed2
    global sel
    global acc_dec
    pub = rospy.Publisher('leftMotorSpeed', Float32, queue_size=10)
    pub2 = rospy.Publisher('rightMotorSpeed', Float32, queue_size=10)
    rospy.init_node('rosBubbleRob', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if kbd.kbhit():
            key = kbd.getch()
            termios.tcflush(sys.stdin, termios.TCIOFLUSH)
            time.sleep(.2)
            if ord(key) == ESC:
                print "ESC - ending test"
                break
            print key#, ord(key)
            res=process(key)
            aa = res[0]
            bb = res[1]
        else:
            #desiredWheelRotSpeed = desiredWheelRotSpeed2 # - Turn only on KBHIT
            if acc_dec == True:
                if desiredWheelRotSpeed > 0:
                    desiredWheelRotSpeed -=.1
                if desiredWheelRotSpeed2 > 0:
                    desiredWheelRotSpeed2 -=.1
                if desiredWheelRotSpeed < 0:
                    desiredWheelRotSpeed +=.1
                if desiredWheelRotSpeed2 < 0:
                    desiredWheelRotSpeed2 +=.1
            aa = desiredWheelRotSpeed
            bb = desiredWheelRotSpeed2
        #rospy.loginfo(float(aa))
        print 'A: ',aa
        print 'B: ',bb
        pub.publish(float(aa))
        pub2.publish(float(bb))
        rate.sleep()
    kbd.set_normal_term()

if __name__ == '__main__':
    kbd = KBHit()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

'''
#Pre-Allocation

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print 'Connected to remote API server'

else:
    print 'Connection not successful'
    sys.exit('Could not connect')
'''
