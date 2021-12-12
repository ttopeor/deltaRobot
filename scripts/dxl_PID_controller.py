#!/usr/bin/env python3

import sys
import os

import rospy
from deltarobot.srv import *
from deltarobot.msg import *

import time
import math
import numpy

theta_1_goal = 0
theta_2_goal = 0
theta_3_goal = 0

pre_D1 = 0
pre_D2 = 0
pre_D3 = 0

I1 = 0
I2 = 0
I3 = 0

def get_dxl_position_client():

    rospy.wait_for_service('get_dxl_position')
    
    try:
        Get_dxl_position = rospy.ServiceProxy('get_dxl_position', GetDxlPosition)
        
        resp1 = Get_dxl_position(1)

        return resp1

    except rospy.ServiceException:
        print("get_dxl_position failed")

def update_goal(data):
    global theta_1_goal
    global theta_2_goal
    global theta_3_goal

    theta_1_goal = data.encoder_pos1
    theta_2_goal = data.encoder_pos2
    theta_3_goal = data.encoder_pos3

def callback(data):
    global theta_1_goal
    global theta_2_goal
    global theta_3_goal

    global pre_D1
    global pre_D2
    global pre_D3

    global I1
    global I2
    global I3

    theta_1_actural = data.encoder_pos1
    theta_2_actural = data.encoder_pos2
    theta_3_actural = data.encoder_pos3

    #P controller
    Kp1 = 12
    Kp2 = 14
    Kp3 = 10

    Kd1 = 0.5 * Kp1
    Kd2 = 0.2 * Kp2
    Kd3 = 0.2 * Kp3

    Ki1 = 0 * Kp1
    Ki2 = 0 * Kp2
    Ki3 = 0 * Kp3

    THRESHOLD = 3
    t = 0.02 #50HZ
    D1 = theta_1_goal - theta_1_actural
    D2 = theta_2_goal - theta_2_actural
    D3 = theta_3_goal - theta_3_actural
    pre_D1 = D1
    pre_D2 = D2
    pre_D3 = D3

    if ((abs(D1) > THRESHOLD) or (abs(D2) > THRESHOLD) or (abs(D3) > THRESHOLD)):

        I1 = I1 + Ki1 * D1 * t
        I2 = I2 + Ki2 * D2 * t
        I3 = I3 + Ki3 * D3 * t

        MV1 = Kp1 * D1 + Kd1 * (D1 - pre_D1)/t + I1
        MV2 = Kp2 * D2 + Kd2 * (D2 - pre_D2)/t + I2
        MV3 = Kp3 * D3 + Kd3 * (D3 - pre_D3)/t + I3

        pre_D1 = D1
        pre_D2 = D2
        pre_D3 = D3

        dxl_spd1 =  int(round(MV1))
        dxl_spd2 =  int(round(MV2))
        dxl_spd3 =  int(round(MV3))
        
        died_speed1 = 0
        died_speed2 = 0
        died_speed3 = 0
        if dxl_spd1>0:
            dxl_spd1 = dxl_spd1+died_speed1
        if dxl_spd1<0:
            dxl_spd1 = dxl_spd1-died_speed1
        
        if dxl_spd2>0:
            dxl_spd2 = dxl_spd2+died_speed2
        if dxl_spd2<0:
            dxl_spd2 = dxl_spd2-died_speed2
        
        if dxl_spd3>0:
            dxl_spd3 = dxl_spd3+died_speed3
        if dxl_spd3<0:
            dxl_spd3 = dxl_spd3-died_speed3


        dxl_pos = get_dxl_position_client()

        if ((dxl_pos.dxl_pos1<-210)and(dxl_spd1 < 0)):
            dxl_spd1 = 0
        if ((dxl_pos.dxl_pos2<-210)and(dxl_spd2 < 0)):
            dxl_spd2 = 0
        if ((dxl_pos.dxl_pos3<-210)and(dxl_spd3 < 0)):    
            dxl_spd3 = 0

        if ((dxl_pos.dxl_pos1>50)and(dxl_spd1 > 0)):
            dxl_spd1 = 0
        if ((dxl_pos.dxl_pos2>50)and(dxl_spd2 > 0)):
            dxl_spd2 = 0
        if ((dxl_pos.dxl_pos3>50)and(dxl_spd3 > 0)):
            dxl_spd3 = 0 

        SetDxlSpeed_msg = DxlSpeed()
        SetDxlSpeed_msg.dxl_spd1 = dxl_spd1
        SetDxlSpeed_msg.dxl_spd2 = dxl_spd2
        SetDxlSpeed_msg.dxl_spd3 = dxl_spd3

        print("dxl_spd:",dxl_spd1," ",dxl_spd2," ", dxl_spd3)

        

        set_dxl_speed.publish(SetDxlSpeed_msg)
        

def ROS_listener_publisher():
    rospy.init_node('offset', anonymous=True)

    rospy.Subscriber("encoder_position_goal", EncoderPosition, update_goal)
    
    
    rospy.spin()

if __name__ == '__main__':
    try:

        set_dxl_speed = rospy.Publisher('set_dxl_speed', DxlSpeed, queue_size=1)
        rospy.Subscriber("encoder_position", EncoderPosition, callback)

        ROS_listener_publisher()  #z -> -100 10 

    except rospy.ROSInterruptException:
        print("exit")