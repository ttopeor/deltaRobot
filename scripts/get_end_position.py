#!/usr/bin/env python3
# -*- coding: utf-8 -*-
NAME = 'get_end_position_server'
from deltarobot.msg import *
from deltarobot.srv import *
import rospy


end_pos1 = 0
end_pos2 = 0
end_pos3 = 0

def update_end_position(data):
    global end_pos1
    global end_pos2
    global end_pos3

    end_pos1 = data.end_pos1
    end_pos2 = data.end_pos2
    end_pos3 = data.end_pos3
   

def get_end_position(req):
    global end_pos1
    global end_pos2
    global end_pos3

    return GetEndPositionResponse(end_pos1,end_pos2,end_pos3)


def get_end_position_server():
    rospy.init_node(NAME)
    s = rospy.Service('get_end_position', GetEndPosition, get_end_position)
    print ("Ready to get end position")
    # spin() keeps Python from exiting until node is shutdown
    

def ROS_listener():
    

    #rospy.Subscriber("encoder_position_goal", EncoderPosition, update_goal)
    
    #rospy.Subscriber("encoder_position", EncoderPosition, callback)

    rospy.Subscriber("end_position", EndPosition, update_end_position)

    rospy.spin()

if __name__ == '__main__':
    try:
        get_end_position_server()
        ROS_listener()
    except rospy.ROSInterruptException:

        pass