#!/usr/bin/env python3

import sys
import os

import rospy
from deltarobot.srv import *
from deltarobot.msg import *

import time
import math
from numpy import *

theta_1_goal = 0
theta_2_goal = 0
theta_3_goal = 0

pre_D1 = 0
pre_D2 = 0
pre_D3 = 0

I1 = 0
I2 = 0
I3 = 0

def get_force_unit_vectors(theta1, theta2, theta3):
    a = 50
    b = 30
    l_a = 33.5
    l_b = 108

    
    e  =  a
    f  =  b
    re = l_b
    rf =  l_a

    # Trigonometric constants
    s      = 165*2
    sqrt3  = math.sqrt(3.0)
    pi     = 3.141592653
    sin120 = sqrt3 / 2.0
    cos120 = -0.5
    tan60  = sqrt3
    sin30  = 0.5
    tan30  = 1.0 / sqrt3

    x0 = 0.0
    y0 = 0.0
    z0 = 0.0
    
    t = (f-e) * tan30 / 2.0
    dtr = pi / 180.0
    
    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr
    
    x1 = 0
    y1 = -(t + rf*math.cos(theta1) )
    z1 = -rf * math.sin(theta1)
    
    y2 = (t + rf*math.cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * math.sin(theta2)
    
    y3 = (t + rf*math.cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * math.sin(theta3)
    
    dnm = (y2-y1)*x3 - (y3-y1)*x2
    
    w1 = y1*y1 + z1*z1
    w2 = x2*x2 + y2*y2 + z2*z2
    w3 = x3*x3 + y3*y3 + z3*z3
    
    # x = (a1*z + b1)/dnm
    a1 = (z2-z1)*(y3-y1) - (z3-z1)*(y2-y1)
    b1= -( (w2-w1)*(y3-y1) - (w3-w1)*(y2-y1) ) / 2.0
    
    # y = (a2*z + b2)/dnm
    a2 = -(z2-z1)*x3 + (z3-z1)*x2
    b2 = ( (w2-w1)*x3 - (w3-w1)*x2) / 2.0
    
    # a*z^2 + b*z + c = 0
    a = a1*a1 + a2*a2 + dnm*dnm
    b = 2.0 * (a1*b1 + a2*(b2 - y1*dnm) - z1*dnm*dnm)
    c = (b2 - y1*dnm)*(b2 - y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re)
    
    # discriminant
    d = b*b - 4.0*a*c
    if d < 0.0:
        return [1,0,0,0] # non-existing povar. return error,x,y,z
    
    z0 = -0.5*(b + math.sqrt(d)) / a
    x0 = (a1*z0 + b1) / dnm
    y0 = (a2*z0 + b2) / dnm

    z0 = z0+10
    EndPosition_msg = EndPosition()
    EndPosition_msg.end_pos1 = round(x0,2)
    EndPosition_msg.end_pos2 = round(y0,2)
    EndPosition_msg.end_pos3 = round(z0,2)
    pub_end_position.publish(EndPosition_msg)

    v1 = mat([(x0-x1)/re,(y0-y1)/re,(z0-z1)/re]).T
    v2 = mat([(x0-x2)/re,(y0-y2)/re,(z0-z2)/re]).T
    v3 = mat([(x0-x3)/re,(y0-y3)/re,(z0-z3)/re]).T
   
    return [0,v1,v2,v3]

def pub_force_info(encoder_pos,dxl_pos):
    l_a = 33.5
    a=50
    ks = 1 
    N = 2
    torque1 = ks*(dxl_pos.dxl_pos1 - N * encoder_pos.encoder_pos1) 
    torque2 = ks*(dxl_pos.dxl_pos2 - N * encoder_pos.encoder_pos2) 
    torque3 = ks*(dxl_pos.dxl_pos3 - N * encoder_pos.encoder_pos3) 
    
    
    unit_vectors = get_force_unit_vectors(encoder_pos.encoder_pos1, encoder_pos.encoder_pos2, encoder_pos.encoder_pos3)

    s      = 165*2
    sqrt3  = math.sqrt(3.0)
    pi     = 3.141592653
    sin120 = sqrt3 / 2.0
    cos120 = -0.5
    tan60  = sqrt3
    sin30  = 0.5
    tan30  = 1.0 / sqrt3
    

    Trans1 = mat([[1,0,0],[0,1,0],[0,0,1]])
    Trans2 = mat([[cos120,sin120,0],[-sin120,cos120,0],[0,0,1]])
    Trans3 = mat([[cos120,-sin120,0],[sin120,cos120,0],[0,0,1]])
   
    unit_vector_trans1 = Trans1*unit_vectors[1]
    unit_vector_trans2 = Trans2*unit_vectors[2]
    unit_vector_trans3 = Trans3*unit_vectors[3]
    
   
    unit_vector_trans1=(asarray(unit_vector_trans1)).flatten().tolist()
    unit_vector_trans2=(asarray(unit_vector_trans2)).flatten().tolist()
    unit_vector_trans3=(asarray(unit_vector_trans3)).flatten().tolist()
    
    force1 = torque1/(unit_vector_trans1[2] * l_a * math.cos((180+encoder_pos.encoder_pos1)*pi/180) - unit_vector_trans1[1] * l_a * math.sin((180+encoder_pos.encoder_pos1)*pi/180))
    force2 = torque2/(unit_vector_trans2[2] * l_a * math.cos((180+encoder_pos.encoder_pos2)*pi/180) - unit_vector_trans2[1] * l_a * math.sin((180+encoder_pos.encoder_pos2)*pi/180))
    force3 = torque3/(unit_vector_trans3[2] * l_a * math.cos((180+encoder_pos.encoder_pos3)*pi/180) - unit_vector_trans3[1] * l_a * math.sin((180+encoder_pos.encoder_pos3)*pi/180))
    
    force_vector1 = [force1 * unit_vector_trans1[0], force1* unit_vector_trans1[1],force1*unit_vector_trans1[2]]
    force_vector2 = mat([force2 * unit_vector_trans2[0], force2* unit_vector_trans2[1],force2*unit_vector_trans2[2]]).T
    force_vector3 = mat([force3 * unit_vector_trans3[0], force3* unit_vector_trans3[1],force3*unit_vector_trans3[2]]).T

    force_vector2 = (asarray(Trans3 * force_vector2)).flatten().tolist()
    force_vector3 = (asarray(Trans2 * force_vector3)).flatten().tolist()

    #print("1:",force_vector1)
    #print("2:",force_vector2)
    #print("3:",force_vector3)

    force_vector = [force_vector1[0]+force_vector2[0]+force_vector3[0],force_vector1[1]+force_vector2[1]+force_vector3[1],force_vector1[2]+force_vector2[2]+force_vector3[2]]
    
    EndForce_msg = EndForce()
    EndForce_msg.end_force_x = round(force_vector[0],2)
    EndForce_msg.end_force_y = round(force_vector[1]-0.47,2)
    EndForce_msg.end_force_z = round(force_vector[2]+1.67,2)

    pub_end_force.publish(EndForce_msg)

    DxlPosition_msg = DxlPosition()
    DxlPosition_msg = dxl_pos
    pub_dxl_position.publish(DxlPosition_msg)

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
    Kp1 = 5
    Kp2 = 5
    Kp3 = 5

    Kd1 = 0.2 * Kp1
    Kd2 = 0.2 * Kp2
    Kd3 = 0.2 * Kp3

    Ki1 = 0 * Kp1
    Ki2 = 0 * Kp2
    Ki3 = 0 * Kp3

    THRESHOLD = 0
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
        
        died_speed1 = 36
        died_speed2 = 28
        died_speed3 = 50
        
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

        if ((dxl_pos.dxl_pos1<-50)and(dxl_spd1 < 0)):
            dxl_spd1 = 0
        if ((dxl_pos.dxl_pos2<-50)and(dxl_spd2 < 0)):
            dxl_spd2 = 0
        if ((dxl_pos.dxl_pos3<-50)and(dxl_spd3 < 0)):    
            dxl_spd3 = 0

        if ((dxl_pos.dxl_pos1>200)and(dxl_spd1 > 0)):
            dxl_spd1 = 0
        if ((dxl_pos.dxl_pos2>200)and(dxl_spd2 > 0)):
            dxl_spd2 = 0
        if ((dxl_pos.dxl_pos3>200)and(dxl_spd3 > 0)):
            dxl_spd3 = 0 

        SetDxlSpeed_msg = DxlSpeed()
        SetDxlSpeed_msg.dxl_spd1 = dxl_spd1
        SetDxlSpeed_msg.dxl_spd2 = dxl_spd2
        SetDxlSpeed_msg.dxl_spd3 = dxl_spd3

        #print("dxl_spd:",dxl_spd1," ",dxl_spd2," ", dxl_spd3)
        

        set_dxl_speed.publish(SetDxlSpeed_msg)

        pub_force_info(data,dxl_pos)
        

def ROS_listener_publisher():
    rospy.init_node('offset', anonymous=True)

    rospy.Subscriber("encoder_position_goal", EncoderPosition, update_goal)
    
    rospy.Subscriber("encoder_position", EncoderPosition, callback)

    rospy.spin()

if __name__ == '__main__':
    try:

        set_dxl_speed = rospy.Publisher('set_dxl_speed', DxlSpeed, queue_size=1)

        pub_end_position = rospy.Publisher('end_position', EndPosition, queue_size=1)
        
        pub_dxl_position = rospy.Publisher('dxl_position', DxlPosition, queue_size=1)

        pub_end_force = rospy.Publisher('end_force', EndForce, queue_size=1)

        ROS_listener_publisher()  #z -> -100 10 

    except rospy.ROSInterruptException:
        print("exit")