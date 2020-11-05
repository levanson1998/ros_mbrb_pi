#!/usr/bin/env python3

import rospy
import numpy as np
import serial
import os
import struct
import time
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
# from your_package.msg import Foo


def DEG2RAD(deg):
    return deg*math.pi/180

def RAD2DEG(rad):
    return rad*180/math.pi

#-----------------------BEGIN SERIAL---------------------------
time_t = 0.1
def serialInit():
    global ser, sio

    ser = serial.Serial(
    port = '/dev/ttyAMA0', # /dev/ttyAMA0 /dev/ttyUSB0
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
    )
    # sio=io.TextIOWrapper(io.BufferedRWPair(ser, ser), encoding="utf-16", errors='ignore')
    # sio.flush()


def transmitSerial():
    global next_call, time_t, ser
    a=12.34
    isStart = 0
    SttSpeed = 0
# ---> van toc banh trai, phai (12,34)
# ---< goc. gia toc tu cam bien mpu6050 ()
    a1 = int(a)
    a2 = int(round((a-a1)*10000))
    
    dataa1=a1.to_bytes(2, byteorder = "little", signed = True)
    a1L=dataa1[1]
    a1H=dataa1[0]
    dataa2=a2.to_bytes(2, byteorder = "little", signed = True)
    a2L=dataa2[1]
    a2H=dataa2[0]

    packet = [a1L, a1H, a2L, a2H, isStart,SttSpeed]
    # print(packet)
    # print("{} * {} * {} * {} * {}".format(a, a1, a2, isStart,SttSpeed))
    #print("time: ",time.time())
    # packet = [chr(dataIsTracking), chr(dataX1), chr(dataX2), chr(dataY1), chr(dataY2)]
    # print("{} - - {}".format(packet, type(packet)))
    # ser.write(packet)
    try:
        receiveData = ser.read(11)
        rospy.loginfo("receive serial: {}".format(receiveData))
        return receiveData
    except:
        rospy.loginfo("serial no data !!!")
        return None
    
    # if(len(receiveData)!= 0):
    #     speedCurrent=receiveData[0]+receiveData[1]/100
    #     ReStop = receiveData[2]

#-----------------------END SERIAL---------------------------

def main():
    rospy.init_node('serial_pub', anonymous=True)
    imu_pub = rospy.Publisher('imu_in', Imu, queue_size=5)
    rospy.loginfo("Publishing Imu at: " + imu_pub.resolved_name)
    serialInit()
    while(True):
        data_ser=transmitSerial()
        imu_=Imu()
        imu_.header.stamp=rospy.Time.now()
        imu_.header.frame_id='imu'


        # imu_.linear_acceleration.x =int.from_bytes(data_ser[4:7], "big")
        # imu_.linear_acceleration.y=int.from_bytes(data_ser[7:10], "big")
        # imu_.linear_acceleration.z=int.from_bytes(data_ser[10:13], "big")
        # imu_.angular_velocity.x=int.from_bytes(data_ser[13:16], "big")
        # imu_.angular_velocity.y=int.from_bytes(data_ser[16:19], "big")
        # imu_.angular_velocity.z=int.from_bytes(data_ser[19:22], "big")

        imu_.linear_acceleration.x=0.2 
        imu_.linear_acceleration.y=3.2
        imu_.linear_acceleration.z=5.6
        imu_.angular_velocity.x=1.023
        imu_.angular_velocity.y=2.36
        imu_.angular_velocity.z=3.37

        imu_pub.publish(imu_)
        if rospy.is_shutdown():
            rospy.loginfo("stop serial publisher")
            break


if __name__=="__main__":
    main()