#!/usr/bin/env python3

import sys
import os

import rospy
from deltarobot.srv import *

import time

def get_encoder_position_client():

    rospy.wait_for_service('get_encoder_position')
    
    try:
        get_encoder_position = rospy.ServiceProxy('get_encoder_position', GetEncoderPosition)
        #print ("Requesting encoder position")
        resp = get_encoder_position(1)
             
    except rospy.ServiceException:
        print ("Service call failed")
        return
    return resp 

for i in range(10):
    encoder_pos=get_encoder_position_client()

    print(encoder_pos.encoder_pos1," ",encoder_pos.encoder_pos2," ",encoder_pos.encoder_pos3)

    time.sleep(0.01)
