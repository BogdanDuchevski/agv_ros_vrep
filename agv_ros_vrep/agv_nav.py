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
from std_msgs.msg import String

# - INIT
ck =-False
save_spd = None
#pub_sp = rospy.Publisher('sensorTrigger', Float32, queue_size=10)
nav_mode = False
sim_in = False
sensor = None
start_time = None
prob_time = None
sensor_tr = False
def callback_sens(data):
    global sensor
    global sensor_tr
    if data > 0:
    #if data > 0:
        #print 'true',data.data
        sensor_x = data.data
        sensor_x = sensor_x.split(',')
        sensor_x = [float(x) for x in sensor_x]
        x = 0
        while x<3:
        #for x in range(len(sensor_x)):
            if sensor_x[x] == 100:
                sensor_tr = False
            else:
                sensor_tr = True
                break
            x+=1
                #if sensor_x[x]==min(sensor_x)
            #    sensor_x[x] = True
        #sens_min = min(sensor_x)
        sensor = sensor_x#[1]
        #sensor_tr =
        #print '3',sensor#[1]
        #sensor = True
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
    global sensor_tr
    global ck
    global start_time
    global prob_time
    pub = rospy.Publisher('leftMotorSpeed', Float32, queue_size=10)
    pub2 = rospy.Publisher('rightMotorSpeed', Float32, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    sensor_loc = [-30/180.0*math.pi,-math.pi/2,30/180.0*math.pi]
    while not rospy.is_shutdown():
        if sensor_tr==True:
            if start_time == None:
                start_time = time.time()
                prob_time = start_time
            #else:
        if start_time is not None:
            if prob_time<start_time+1:
                ck = True
            else:
                ck = False
                start_time = None
            prob_time = time.time()

        if sensor_tr == False and ck == False:
            steer=0
        else:
            min_ind = sensor.index(min(sensor))#min(sensor)
            print 'min_ind : ',min_ind
            steer=-1/sensor_loc[min_ind]
        kp=1.5	#steering gain
        vl=save_spd+kp*steer
        vr=save_spd-kp*steer
        pub.publish(float(vl))
        pub2.publish(float(vr))
        rate.sleep()

def listener():
    rospy.init_node('rosBubbleRob2', anonymous=True)
    rospy.Subscriber("speed", Float32, callback_spd)
    rospy.Subscriber("sensorTrigger", String, callback_sens)
    rospy.Subscriber("simTime", Float32, callback_sim)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
