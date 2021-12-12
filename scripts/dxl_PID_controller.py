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

    theta_1_actural = data.encoder_pos1
    theta_2_actural = data.encoder_pos2
    theta_3_actural = data.encoder_pos3

    #P controller
    Kp = 15
    THRESHOLD = 1
    D1 = theta_1_goal - theta_1_actural
    D2 = theta_2_goal - theta_2_actural
    D3 = theta_3_goal - theta_3_actural

    if ((abs(D1) > THRESHOLD) or (abs(D2) > THRESHOLD) or (abs(D3) > THRESHOLD)):
        MV1 = Kp * D1
        MV2 = Kp * D2
        MV3 = Kp * D3

        dxl_spd_goal1 =  int(round(MV1))
        dxl_spd_goal2 =  int(round(MV2))
        dxl_spd_goal3 =  int(round(MV3))

        SetDxlSpeed_msg = DxlSpeed()
        SetDxlSpeed_msg.dxl_spd1 = dxl_spd_goal1
        SetDxlSpeed_msg.dxl_spd2 = dxl_spd_goal2
        SetDxlSpeed_msg.dxl_spd3 = dxl_spd_goal3

        print("dxl_spd_goal:",dxl_spd_goal1," ",dxl_spd_goal2," ", dxl_spd_goal3)


        set_dxl_speed.publish(SetDxlSpeed_msg)
    

def ROS_listener_publisher():
    rospy.init_node('offset', anonymous=True)

    rospy.Subscriber("encoder_position_goal", EncoderPosition, update_goal)
    rospy.Subscriber("encoder_position", EncoderPosition, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:

        set_dxl_speed = rospy.Publisher('set_dxl_speed', DxlSpeed, queue_size=1)


        ROS_listener_publisher()  #z -> -100 10 

    except rospy.ROSInterruptException:
        print("exit")