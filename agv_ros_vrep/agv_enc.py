#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import math

#-- SENSORS
enc = 0
previous_angle=0
previous_angle2=0
cumulative_angles=0
cumulative_angles2=0
tick_per_rev = 1000
deg_per_tick = tick_per_rev / 360# -- DEGREES PER TICK F
enc_pass=True
angle_result=0
angle_result2=0
##ENCODER : enc_f(int current position, int previous angle)
def enc_f(current_p_x,previous_angle_x,angle_result,cumulative_angles):
    if previous_angle_x is not None:
        angle_result=current_p_x-previous_angle_x#-- GET ANGLE RESULT
    if (abs_b(angle_result)>math.pi):
        angle_result=angle_result - (2.0 * math.pi * abs_b(angle_result)/angle_result)#-- ADD MAGIC
    cumulative_angles = cumulative_angles + angle_result#-- ADD ANGLE RESULT
    encoder=(tick_per_rev*cumulative_angles)/(2.0*math.pi)# -- DEFINE TICKS
    round_enc=round(encoder, 1)
    return round_enc,angle_result,cumulative_angles

def abs_b(x):
    if x < 0 :
        x = x*(-1)
    return(x)

def callback1(data):
    global previous_angle
    global angle_result
    global cumulative_angles
    rospy.loginfo(data.data)
    current_p = data.data#-- GET CURRENT POSITION
    enc,angle_result,cumulative_angles = enc_f(current_p,previous_angle,angle_result,cumulative_angles)
    previous_angle = current_p
    print '111111111: ',enc

def callback2(data):
    global previous_angle2
    global angle_result2
    global cumulative_angles2
    rospy.loginfo(data.data)
    current_p = data.data#-- GET CURRENT POSITION
    enc,angle_result2,cumulative_angles2 = enc_f(current_p,previous_angle2,angle_result2,cumulative_angles2)
    previous_angle2 = current_p
    print '222222222: ',enc

def listener():
    rospy.init_node('encoder_node', anonymous=True)
    rospy.Subscriber("leftMotorEnc", Float32, callback1)
    rospy.Subscriber("rightMotorEnc", Float32, callback2)
    rospy.spin()

if __name__ == '__main__':
    listener()
