#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import math

#-- SENSORS
enc = 0
previous_angle=0
cumulative_angles=0
tick_per_rev = 1000
deg_per_tick = tick_per_rev / 360# -- DEGREES PER TICK F
enc_pass=True
##ENCODER : enc_f(int current position, int previous angle)
def enc_f(current_p_x,previous_angle_x):
    global angle_result
    global cumulative_angles
    global previous_angle
    if previous_angle_x is not None:
        angle_result=current_p_x-previous_angle_x#-- GET ANGLE RESULT
    if (abs_b(angle_result)>math.pi):
        angle_result=angle_result - (2.0 * math.pi * abs_b(angle_result)/angle_result)#-- ADD MAGIC
    cumulative_angles = cumulative_angles + angle_result#-- ADD ANGLE RESULT
    previous_angle = current_p_x#-- DEFINE PREVIOUS ANGLE
    encoder=(tick_per_rev*cumulative_angles)/(2.0*math.pi)# -- DEFINE TICKS
    round_enc=round(encoder, 1)
    return round_enc

def abs_b(x):
    if x < 0 :
        x = x*(-1)
    return(x)

def callback(data):
    global previous_angle
    rospy.loginfo(data.data)
    current_p = data.data#-- GET CURRENT POSITION
    enc = enc_f(current_p,previous_angle)
    print enc

def listener():
    rospy.init_node('encoder_node', anonymous=True)
    rospy.Subscriber("leftMotorEnc", Float32, callback)0
    rospy.spin()

if __name__ == '__main__':
    listener()
