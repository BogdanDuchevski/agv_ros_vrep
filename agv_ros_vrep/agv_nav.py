# -*- coding: utf-8 -*-
#!/usr/bin/env python
"""
@author: Bogdan Duchevski.
"""
#Import Libraries:
import vrep
import numpy as np
import math
import os,time
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool

# - INIT
ck =-False
save_spd = None
#pub_sp = rospy.Publisher('sensorTrigger', Float32, queue_size=10)
nav_mode = False
sim_in = False
sensor = None
start_time = None
prob_time = None
def callback_sens(data):
    global sensor
    if data > 0:
    #if data > 0:
        #print 'true',data.data
        sensor = data.data
        #pub_sp.publish(save_spd)

def callback_spd(data):
    global save_spd
    if data > 0:
    #if data > 0:
        #print 'true',data.data
        save_spd = data.data
        #pub_sp.publish(save_spd)

def callback_sim(data):
    global save_spd
    global sensor
    global ck
    global start_time
    global prob_time
    pub = rospy.Publisher('leftMotorSpeed', Float32, queue_size=10)
    pub2 = rospy.Publisher('rightMotorSpeed', Float32, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if sensor==True:
            if start_time == None:
                start_time = time.time()
                prob_time = start_time
            #else:
        if start_time is not None:
            if prob_time<start_time+3:
                ck = True
            else:
                ck = False
                start_time = None
            prob_time = time.time()

        if sensor == False and ck == False:
            steer=0
        else:
            steer=-1#/sensor_loc[min_ind]'
        kp=1.5	#steering gain
        vl=save_spd+kp*steer
        vr=save_spd-kp*steer
        pub.publish(float(vl))
        pub2.publish(float(vr))
        rate.sleep()

def listener():
    rospy.init_node('rosBubbleRob2', anonymous=True)
    rospy.Subscriber("speed", Float32, callback_spd)
    rospy.Subscriber("sensorTrigger", Bool, callback_sens)
    rospy.Subscriber("simTime", Float32, callback_sim)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
