#!/usr/bin/env python3

import sys
import os

import rospy
from deltarobot.srv import *
from deltarobot.msg import *

import time
import math
import numpy

a = 50
b = 30
l_a = 33.5
l_b = 108

k = a-b
m = (math.sqrt(3)/2)*b - (math.sqrt(3)/2)*a
n = (1/2)*b-(1/2)*a


def callback(data):
    

    p = [0,data.set_end_pos1,data.set_end_pos2,-data.set_end_pos3-93.82]

    G = [0,2*l_a*(p[2]+k), -l_a*(math.sqrt(3)*(p[1]+m)+p[2]+n), l_a*(math.sqrt(3)*(p[1]-m)-p[2]-n)]
    F = [0,2*p[3]*l_a,2*p[3]*l_a,2*p[3]*l_a]
    E = [0,p[1]**2+p[2]**2+p[3]**2+k**2+l_a**2+2*p[2]*k-l_b**2,
         p[1]**2+p[2]**2+p[3]**2+m**2+n**2+l_a**2+2*p[1]*m+ 2*p[2]*n-l_b**2,
         p[1]**2+p[2]**2+p[3]**2+m**2+n**2+l_a**2-2*p[1]*m+ 2*p[2]*n-l_b**2]
    
    t_1 = (-F[1] - math.sqrt(E[1]**2 +F[1]**2 - G[1]**2))/(G[1]-E[1])
    t_2 = (-F[2] - math.sqrt(E[2]**2 +F[2]**2 - G[2]**2))/(G[2]-E[2])
    t_3 = (-F[3] - math.sqrt(E[3]**2 +F[3]**2 - G[3]**2))/(G[3]-E[3])

    theta_1_goal = round(math.degrees(2* math.atan(t_1)),2)
    theta_2_goal = round(math.degrees(2* math.atan(t_2)),2)
    theta_3_goal = round(math.degrees(2* math.atan(t_3)),2)

    
    print("goal theata:",theta_1_goal," ",theta_2_goal," ", theta_3_goal)

    EncoderPositionGoal_msg = EncoderPosition()
    EncoderPositionGoal_msg.encoder_pos1 = theta_1_goal
    EncoderPositionGoal_msg.encoder_pos2 = theta_2_goal
    EncoderPositionGoal_msg.encoder_pos3 = theta_3_goal
    pub_encoder_position_goal.publish(EncoderPositionGoal_msg)

    

def ROS_listener_publisher():
    rospy.init_node('endpoint', anonymous=True)
    rospy.Subscriber("set_end_position", SetEndPosition, callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        pub_encoder_position_goal = rospy.Publisher('encoder_position_goal', EncoderPosition, queue_size=5)
        
        ROS_listener_publisher()  #z -> -100 10 

    except rospy.ROSInterruptException:
        print("exit")