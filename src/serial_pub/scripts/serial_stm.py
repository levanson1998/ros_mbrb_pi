#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import serial
import time
import numpy as np


#-----------------------BEGIN SERIAL---------------------------
time_t = 0.1
data_ser = 0
def serialInit():
    global ser
    ser = serial.Serial(
    port = '/dev/ttyAMA0', # /dev/ttyAMA0 /dev/ttyUSB0
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
    )
    print("Connect to {}".format(ser.port))
    # sio=io.TextIOWrapper(io.BufferedRWPair(ser, ser), encoding="utf-16", errors='ignore')
    # sio.flush()


def sendSerial(velo_l, velo_r, dir_):
    global ser
    velo_l1 = int(velo_l)
    velo_l2 = int(round((velo_l-velo_l1)*10000))

    velo_r1 = int(velo_r)
    velo_r2 = int(round((velo_r-velo_r1)*10000))
    
    data1=velo_l1.to_bytes(1, byteorder = "little", signed = True)
    a1H=data1[0]
    data2=velo_l2.to_bytes(2, byteorder = "little", signed = True)
    a2L=data2[1]
    a2H=data2[0]

    data3=velo_r1.to_bytes(1, byteorder = "little", signed = True)
    a3H=data3[0]
    data4=velo_r2.to_bytes(2, byteorder = "little", signed = True)
    a4L=data4[1]
    a4H=data4[0]

    packet = [a1H, a2L, a2H, a3H, a4L, a4H, dir_]
    ser.flush()
    ser.write(packet)

def receiveSerial():
    global ser
    # try:
    ser.flush()
    data=ser.read(23)
    receiveData=processDataSer(data)
    return receiveData

    # except:
    #     rospy.loginfo("serial no data !!!")
    #     return []

def processDataSer(data):
    # print(data)
    # print(len(data))
    dataRecv=[]
    dataRecv.append(int.from_bytes(data[0:2], 'big', signed=True))
    dataRecv.append(int.from_bytes(data[2:4], 'big', signed=True))
    for i in range(4, 13, 3):
        dataRecv.append(int.from_bytes(data[i:i+3], 'big', signed=True)/1000)
    for i in range(13, 22, 3):
        dataRecv.append(int.from_bytes(data[i:i+3], 'big', signed=True)/1000)
    dataRecv.append(data[22])
    # print(dataRecv)
    dataRecv = np.array(dataRecv)
    return dataRecv

#-----------------------END SERIAL---------------------------

def getDataSerial():
    global data_ser
    return data_ser

def main():
    global data_ser
    rospy.init_node('serial_stm', anonymous=True)
    serial_pub = rospy.Publisher('serial', Float32MultiArray, queue_size=10)
    floatarray = Float32MultiArray()
    serialInit()
    t1 = 0
    while(True):
        print("\n")
        data_ser = receiveSerial()
        prt=""
        for i in range(len(data_ser)):
            prt += "\n{}".format(data_ser[i])
        print(time.time()-t1)
        t1 = time.time()
        print(prt)  
        # floatarray.layout="ser"
        floatarray.data=data_ser
        serial_pub.publish(floatarray)
        # print("serial_pub !!")
        if rospy.is_shutdown():
            rospy.loginfo("stop serial stm")
            break    


if __name__=="__main__":
    main()