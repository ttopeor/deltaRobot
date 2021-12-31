#!/usr/bin/env python3
# -*- coding: utf-8 -*-
NAME = 'get_end_force_server'
from deltarobot.msg import *
from deltarobot.srv import *
import rospy

force_x = 0
force_y = 0
force_z = 0

def update_end_force(data):
    global force_x
    global force_y
    global force_z

    force_x = data.end_force_x
    force_y = data.end_force_y
    force_z = data.end_force_z
   

def get_end_force(req):
    global force_x
    global force_y
    global force_z

    end_force_x = force_x
    end_force_y = force_y
    end_force_z = force_z

    return GetEndForceResponse(end_force_x,end_force_y,end_force_z)


def get_end_force_server():
    rospy.init_node(NAME)
    s = rospy.Service('get_end_force', GetEndForce, get_end_force)
    print ("Ready to get end force")
    # spin() keeps Python from exiting until node is shutdown
    

def ROS_listener():
    

    #rospy.Subscriber("encoder_position_goal", EncoderPosition, update_goal)
    
    #rospy.Subscriber("encoder_position", EncoderPosition, callback)

    rospy.Subscriber("end_force", EndForce, update_end_force)

    rospy.spin()

if __name__ == '__main__':
    try:
        get_end_force_server()
        ROS_listener()
    except rospy.ROSInterruptException:

        pass