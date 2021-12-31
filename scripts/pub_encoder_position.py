#!/usr/bin/env python3

import rospy
from deltarobot.msg import EncoderPosition



import os

import serial
import sys
#import binascii
import time

ser = serial.Serial('/dev/ttyUSB0',9600,timeout=0.05)  # open serial port
print(ser.name)         # check which port was really used

def write_req(ser,encoder_num):
    req1 = b'\x9A\x03\xB6\xC1\x00\x02\xAF\x94'
    req2 = b'\x01\x03\xA3\x48\x00\x02\x66\x59'
    req3 = b'\x76\x03\xA4\x48\x00\x02\x6D\xAA'
    req = [req1,req2,req3]
    ser.write(req[encoder_num-1])
    res = ser.read(9)

    res = res[5:7]
    #print(binascii.hexlify(bytearray(res)))
    res = int.from_bytes(res, "big")
    #print(res)
    return res


def encoderPosition_pubulisher():
    pub = rospy.Publisher('encoder_position', EncoderPosition, queue_size=1)
    rospy.init_node('encoder_position_pub', anonymous=True)
    r = rospy.Rate(50) 


    while not rospy.is_shutdown():
        res1 = write_req(ser,1)
        time.sleep(0.01)
        res2 = write_req(ser,2)
        time.sleep(0.01)
        res3 = write_req(ser,3)

        if (res1 > 500):
            res1 = -(1023-res1)
        if (res2 > 500):
            res2 =-(1023-res2)
        if (res3 > 500):
            res3 =-(1023-res3)
            
        pos1 = -round(((res1-275)*360/1023),2)
        pos2 = -round(((res2-275)*360/1023),2)
        pos3 = -round(((res3-275)*360/1023),2)


        encoderPosition = EncoderPosition()
        encoderPosition.encoder_pos1 = pos1
        encoderPosition.encoder_pos2 = pos2
        encoderPosition.encoder_pos3 = pos3
        rospy.loginfo(encoderPosition)
        pub.publish(encoderPosition)
        r.sleep()

if __name__ == '__main__':
    try:
        encoderPosition_pubulisher()
    except rospy.ROSInterruptException: pass