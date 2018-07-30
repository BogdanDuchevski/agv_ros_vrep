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
import rospy, os
from pathlib import Path
from std_msgs.msg import Float32

# - INIT FOLDERS
sel = None
wheelRotSpeedDx=20*math.pi/180
desiredWheelRotSpeed = 0
desiredWheelRotSpeed2 = 0
d_ar=[]

def process(key):
    global desiredWheelRotSpeed
    global desiredWheelRotSpeed2
    global wheelRotSpeedDx
    global d_ar
    if key == 'x':
        exit('exitting')
    if key == 'A':
        if desiredWheelRotSpeed<10:
            desiredWheelRotSpeed=desiredWheelRotSpeed+wheelRotSpeedDx
        else:
            desiredWheelRotSpeed=10
        d_ar=[desiredWheelRotSpeed,desiredWheelRotSpeed]
    elif key == 'B':
        if desiredWheelRotSpeed>-10:
            desiredWheelRotSpeed=desiredWheelRotSpeed-wheelRotSpeedDx
        else:
            desiredWheelRotSpeed=-10
        #desiredWheelRotSpeed2=desiredWheelRotSpeed
        d_ar=[desiredWheelRotSpeed,desiredWheelRotSpeed]
    elif key == 'C':
        desiredWheelRotSpeed=desiredWheelRotSpeed
        desiredWheelRotSpeed2=desiredWheelRotSpeed2+wheelRotSpeedDx
        d_ar=[desiredWheelRotSpeed,desiredWheelRotSpeed2]
    elif key == 'D':
        if desiredWheelRotSpeed<10 and desiredWheelRotSpeed2<10:
            desiredWheelRotSpeed=desiredWheelRotSpeed+wheelRotSpeedDx
            desiredWheelRotSpeed2=(desiredWheelRotSpeed2-wheelRotSpeedDx)-1
        else:
            desiredWheelRotSpeed=10
            desiredWheelRotSpeed2=desiredWheelRotSpeed-1
        d_ar=[desiredWheelRotSpeed2,desiredWheelRotSpeed]
    return d_ar


def int_chk(s):
    try:
        int(s)
        return True
    except ValueError:
        return False
def talker():
    global sel

    pub = rospy.Publisher('leftMotorSpeed', Float32, queue_size=10)
    pub2 = rospy.Publisher('rightMotorSpeed', Float32, queue_size=10)
    rospy.init_node('rosBubbleRob', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        key = getch.getch()
        res=process(key)
        print 'brr',res
        #if sel == True:
        if len(res)>0:# is not None:
            rospy.loginfo(float(res[0]))
            pub.publish(float(res[0]))
            pub2.publish(float(res[1]))
            rate.sleep()
        #A = up; B = down; C = right; D = left
        '''if sel == None:
            #hello_str = raw_input('Select: ')
            if int_chk(res)==True:
                if 0 <= int(res) <= 3:
                    sel = True
                else:
                    sel = False
            else:
                sel = False
        '''

if __name__ == '__main__':
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
