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
acc_dec = True
sel = None
desiredWheelRotSpeed = 0
desiredWheelRotSpeed2 = 0
d_ar=[]
spd = 1 # new speed mode
save_spd = None
pub_sp = rospy.Publisher('speed', Float32, queue_size=10)
nav_mode = False
sim_in = False

def nav_prem(dt,dir_x):
    global desiredWheelRotSpeed
    global desiredWheelRotSpeed2
    global d_ar
    global save_spd
    global pub_sp
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
        #pub_sp.publish(desiredWheelRotSpeed)
    return d_ar

def nav_prem2(dt,dir_x):
    global desiredWheelRotSpeed
    global desiredWheelRotSpeed2
    global d_ar
    desiredWheelRotSpeed=dt[0]
    desiredWheelRotSpeed2=dt[1]
    d_ar=[dt[0],dt[1]]
    return d_ar

def process(key,nav_mode_x):
    global nav_mode
    global dir
    cmnd = []
    nav_prem_res = []
    dir_k = None
    if key == '1':
        nav_mode = True
    if key == '2':
        nav_mode = False
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
        if nav_mode_x == False:
            nav_prem_res = nav_prem2(cmnd,dir_k)
        else:
            nav_prem_res = nav_prem(cmnd,dir_k)
        #if dir_k is not None and dir*dir_k>0:
        #    dir = dir_k
    return nav_prem_res

def accel_f(desiredWheelRotSpeed_x,acc_dec_x):
    if acc_dec_x == True:
        if desiredWheelRotSpeed_x[0] > 0:
            desiredWheelRotSpeed_x[0] -=.1
        if desiredWheelRotSpeed_x[0] < 0:
            desiredWheelRotSpeed_x[0] +=.1

        if desiredWheelRotSpeed_x[1] > 0:
            desiredWheelRotSpeed_x[1] -=.1
        if desiredWheelRotSpeed_x[1] < 0:
            desiredWheelRotSpeed_x[1] +=.1
    return desiredWheelRotSpeed_x

def int_chk(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def callback_spd(data):
    global nav_mode
    global save_spd
    if nav_mode == False and data > 0:
    #if data > 0:
        print 'true',data.data
        save_spd = data.data
        #pub_sp.publish(save_spd)

def callback_sim(data):
    global desiredWheelRotSpeed
    global desiredWheelRotSpeed2
    global acc_dec
    global save_spd
    global nav_mode
    pub = rospy.Publisher('leftMotorSpeed', Float32, queue_size=10)
    pub2 = rospy.Publisher('rightMotorSpeed', Float32, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if kbd.kbhit():
            key = kbd.getch()
            termios.tcflush(sys.stdin, termios.TCIOFLUSH)
            if ord(key) == ESC:
                pub.publish(0.0)
                pub2.publish(0.0)
                kbd.set_normal_term()
                #os.system("rosnode kill "+ rosBubbleRob)
                rospy.signal_shutdown('Quit')
                break
            print key#, ord(key)
            res=process(key,nav_mode)
            print 'RES: ', res
            if len(res)>0:
                if nav_mode == False:
                    save_spd = 0 if save_spd == None else save_spd
                    aa = res[0]*save_spd
                    bb = res[1]*save_spd
                else:
                    aa = res[0]
                    bb = res[1]
        else:
            if nav_mode == False:
                save_spd = 0 if save_spd == None else save_spd
                aa = desiredWheelRotSpeed*save_spd
                bb = desiredWheelRotSpeed2*save_spd
            else:
                desiredWheelRotSpeed,desiredWheelRotSpeed2=accel_f([desiredWheelRotSpeed,desiredWheelRotSpeed2],acc_dec)
                aa = desiredWheelRotSpeed
                bb = desiredWheelRotSpeed2
        #rospy.loginfo(float(aa))
        #print 'A: ',aa
        #print 'B: ',bb
        pub.publish(float(aa))
        pub2.publish(float(bb))
        rate.sleep()

def listener():
    rospy.init_node('rosBubbleRob', anonymous=True)
    rospy.Subscriber("speed", Float32, callback_spd)
    rospy.Subscriber("simTime", Float32, callback_sim)
    rospy.spin()

if __name__ == '__main__':
    kbd = KBHit()
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
