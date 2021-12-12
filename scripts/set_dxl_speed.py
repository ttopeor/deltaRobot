#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from deltarobot.msg import *

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_SPEED         = 32
ADDR_MX_PRESENT_POSITION   = 36

# Data Byte Length
LEN_MX_GOAL_POSITION       = 2
LEN_MX_PRESENT_POSITION    = 2
LEN_MX_MOVING_SPEED        = 2
# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 100           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 900            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1                # Dynamixel moving status threshold

index = 0

dxl_pos1=0
dxl_pos2=0
dxl_pos3=0

spd1 = 0
spd2 = 0
spd3 = 0

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_SPEED, LEN_MX_MOVING_SPEED)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# Enable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)

def update_dxl_position(data):
    global dxl_pos1
    global dxl_pos2
    global dxl_pos3

    global spd1
    global spd2
    global spd3

    dxl_pos1 = data.dxl_pos1
    dxl_pos2 = data.dxl_pos2
    dxl_pos3 = data.dxl_pos3

    if (((dxl_pos1<-200)and(spd1>1024)) or ((dxl_pos1>50)and(spd1<1024))):
        spd1 = 1024
        param_goal_speed1 = [DXL_LOBYTE(DXL_LOWORD(spd1)), DXL_HIBYTE(DXL_LOWORD(spd1))]
        dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_speed1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
            quit()
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        groupSyncWrite.clearParam()
    if (((dxl_pos2<-200)and (spd2>1024)) or ((dxl_pos2>50)and(spd2<1024))):
        spd2 = 1024
        param_goal_speed2 = [DXL_LOBYTE(DXL_LOWORD(spd2)), DXL_HIBYTE(DXL_LOWORD(spd2))]
        dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_speed2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        groupSyncWrite.clearParam()
    if (((dxl_pos3<-200)and (spd3>1024)) or ((dxl_pos3>50)and(spd3<1024))):
        spd3 = 1024
        param_goal_speed3 = [DXL_LOBYTE(DXL_LOWORD(spd3)), DXL_HIBYTE(DXL_LOWORD(spd3))]
        dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_speed3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
            quit()
            # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

def callback(data):
    global dxl_pos1
    global dxl_pos2
    global dxl_pos3

    global spd1
    global spd2
    global spd3

    if (data.dxl_spd1 < 0):
        spd1 = -data.dxl_spd1 + 1024
        if (spd1 > 2047):
            spd1 = 2047
    else:
        spd1 = data.dxl_spd1 
        if(spd1 > 1023):
            spd1=1023
    
    if (data.dxl_spd2 < 0):
        spd2 = -data.dxl_spd2 + 1024
        if (spd2 > 2047):
            spd2 = 2047
    else:
        spd2 = data.dxl_spd2
        if(spd2 > 1023):
            spd2=1023

    if (data.dxl_spd3 < 0):
        spd3 = -data.dxl_spd3 + 1024
        if (spd3 > 2047):
            spd3 = 2047
    else:
        spd3 = data.dxl_spd3   
        if(spd3 > 1023):
            spd3=1023

    param_goal_speed1 = [DXL_LOBYTE(DXL_LOWORD(spd1)), DXL_HIBYTE(DXL_LOWORD(spd1))]
    param_goal_speed2 = [DXL_LOBYTE(DXL_LOWORD(spd2)), DXL_HIBYTE(DXL_LOWORD(spd2))]
    param_goal_speed3 = [DXL_LOBYTE(DXL_LOWORD(spd3)), DXL_HIBYTE(DXL_LOWORD(spd3))]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_speed1)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_speed2)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_speed3)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

def ROS_listener():
    rospy.init_node('dxl_A12', anonymous=True)
    rospy.Subscriber("set_dxl_speed", DxlSpeed, callback)
    #rospy.Subscriber("dxl_position", DxlPosition, update_dxl_position)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        ROS_listener()

    except rospy.ROSInterruptException:
        # Disable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        portHandler.closePort()
        print('port closed')