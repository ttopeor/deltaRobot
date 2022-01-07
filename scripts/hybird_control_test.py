#!/usr/bin/env python3
import rospy
from deltarobot.msg import *
from deltarobot.srv import *
import time
import pandas as pd
import math 

action = pd.read_excel('massage_tasks/hybird_test_path.xlsx', sheet_name='Sheet1') 


action = action.values.tolist()


t = 0

force_x = 0
force_y = 0
force_z = 0
force_data = {'force_x':[],
              'force_y':[],
              'force_z':[]
              }

end_pos_1 = 0
end_pos_2 = 0
end_pos_3 = 0
pos_data = {'end_pos_1':[],
              'end_pos_2':[],
              'end_pos_3':[]
              }

set_end_pos_1 = 0
set_end_pos_2 = 0
set_end_pos_3 = 0
set_pos_data = {'set_end_pos_1':[],
                'set_end_pos_2':[],
                'set_end_pos_3':[]
                }

def get_end_force_client():

    rospy.wait_for_service('get_end_force')
    
    try:
        get_end_force = rospy.ServiceProxy('get_end_force', GetEndForce)
        
        resp1 = get_end_force(1)

        return resp1

    except rospy.ServiceException:
        print("get_end_force failed")

def get_end_position_client():

    rospy.wait_for_service('get_end_position')
    
    try:
        get_end_position = rospy.ServiceProxy('get_end_position', GetEndPosition)
        
        resp1 = get_end_position(1)

        return resp1

    except rospy.ServiceException:
        print("get_end_position failed")

def dxlPosition_pubulisher():
    global t
    global force_data
    global force_x
    global force_y
    global force_z
    global end_pos_1
    global end_pos_2
    global end_pos_3

    freq = action[7][10]
    
    pub = rospy.Publisher('set_end_position', SetEndPosition, queue_size=1)
    rospy.init_node('on_going_test', anonymous=True)
    r = rospy.Rate(freq) #10hz

    f_ref = 5
    I_e = 0
    pre_e = 0
    time = (1/freq)
    while not rospy.is_shutdown():
        
        force = get_end_force_client()

        force_data["force_x"].append(force.end_force_x)
        force_data["force_y"].append(force.end_force_y)
        force_data["force_z"].append(force.end_force_z)
        
        end_pos = get_end_position_client()

        pos_data["end_pos_1"].append(end_pos.end_pos1)
        pos_data["end_pos_2"].append(end_pos.end_pos2)
        pos_data["end_pos_3"].append(end_pos.end_pos3)

        index = round(t*freq)
        
        x = action[0][index+1]
        y = action[1][index+1]
        z = action[2][index+1]    
        
        #force_abs = (force.end_force_x)**2 + (force.end_force_y)**2 + (force.end_force_z)**2
        #force_abs = math.sqrt(force_abs)
        force_abs = abs(force.end_force_z)

        if (force_abs > 0.6):

            e = -(f_ref - force_abs)

            kp = 12
            ki = 0
            kd = 1.5
            
            I_e = I_e + ki * e * time

            delta_pz = kp * e + kd * (e - pre_e)/time + I_e

            pre_e = e
        else: 
            delta_pz = 0

        SetEndPosition_msg = SetEndPosition()
        SetEndPosition_msg.set_end_pos1 = x
        SetEndPosition_msg.set_end_pos2 = y
        SetEndPosition_msg.set_end_pos3 = z + delta_pz
        print("abs_force:", force_abs)
        #rospy.loginfo(SetEndPosition_msg)
        pub.publish(SetEndPosition_msg)

        set_pos_data["set_end_pos_1"].append(x)
        set_pos_data["set_end_pos_2"].append(y)
        set_pos_data["set_end_pos_3"].append(z + delta_pz)

        if (index+2) < len(action[0]):
            t = t + 1/freq
        else:
            t =0
            df_force = pd.DataFrame(force_data, columns = ['force_x', 'force_y','force_z'])
            df_force.to_excel (r'test_results/export_force_data.xlsx', index = False, header=True)
            df_pos = pd.DataFrame(pos_data, columns = ['end_pos_1', 'end_pos_2','end_pos_3'])
            df_pos.to_excel (r'test_results/export_pos_data.xlsx', index = False, header=True)
            df_set_pos = pd.DataFrame(set_pos_data, columns = ['set_end_pos_1', 'set_end_pos_2','set_end_pos_3'])
            df_set_pos.to_excel (r'test_results/export_set_pos_data.xlsx', index = False, header=True)
            exit()
        r.sleep()


if __name__ == '__main__':
    try:
        dxlPosition_pubulisher()
   
        
    except rospy.ROSInterruptException: pass