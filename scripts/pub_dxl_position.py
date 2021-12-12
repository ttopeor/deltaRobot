#!/usr/bin/env python3

import rospy
from deltarobot.msg import *

import os

import serial
import sys
#import binascii
import time

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
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Data Byte Length
LEN_MX_GOAL_POSITION       = 2
LEN_MX_PRESENT_POSITION    = 2

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
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 3                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

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

def get_dxl_position():
    # Read Dynamixel#1 present position
    dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read Dynamixel#2 present position
    dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    # Read Dynamixel#3 present position
    dxl3_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    #print("[ID:%03d]  Pos:%03d\t [ID:%03d]  Pos:%03d\t [ID:%03d]  Pos:%03d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position,DXL3_ID, dxl3_present_position))
    
    #dxl_pos1=-dxl1_present_position*90/700+753*90/700
    #dxl_pos2=-dxl1_present_position*90/700+753*90/700
    #dxl_pos3=-dxl1_present_position*90/700+753*90/700
    dxl_pos1 = round((dxl1_present_position*300/1023)-221,2)
    dxl_pos2 = round((dxl2_present_position*300/1023)-230,2)
    dxl_pos3 = round((dxl3_present_position*300/1023)-235,2)

    dxl_pos = DxlPosition()
    dxl_pos.dxl_pos1 = dxl_pos1
    dxl_pos.dxl_pos2 = dxl_pos2
    dxl_pos.dxl_pos3 = dxl_pos3

    return dxl_pos

def dxlPosition_pubulisher():
    pub = rospy.Publisher('dxl_position', DxlPosition, queue_size=1)
    rospy.init_node('dxl_position_pub', anonymous=True)
    r = rospy.Rate(10) #10hz

    
    while not rospy.is_shutdown():

        dxl_pos = get_dxl_position()
        dxlPosition = DxlPosition()
        dxlPosition.dxl_pos1 = dxl_pos.dxl_pos1
        dxlPosition.dxl_pos2 = dxl_pos.dxl_pos2
        dxlPosition.dxl_pos3 = dxl_pos.dxl_pos3

        rospy.loginfo(dxlPosition)
        pub.publish(dxlPosition)
        r.sleep()

if __name__ == '__main__':
    try:
        dxlPosition_pubulisher()
    except rospy.ROSInterruptException: pass