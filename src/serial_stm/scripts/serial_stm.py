#!/usr/bin/env python3

import rospy
import serial
import time


#-----------------------BEGIN SERIAL---------------------------
time_t = 0.1
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
    data1=ser.read(23)
    # data1=str(data1)
    # data1=data1.encode('utf-8')
    print(data1)
    print(type(data1))
    # receiveData=processDataSer(data1)
    return data1
    
        # data1=str.encode(unicode(data1))
        # data1=ser.readline()
        # print(data1)
        # print("data [1]: {}".format(data1[0:2]))
        # print(type(data1))
        # print(len(data1))
        # print(type(data1[0]))
        # print(time.time()-t1)
    # receiveData=processDataSer(data1)
    # return receiveData
    # except:
    #     rospy.loginfo("serial no data !!!")
    #     return []

def processDataSer(data):

    dataRecv=[]
    dataRecv[0]=int.from_bytes(data[0:2], 'big')
    dataRecv[1]=int.from_bytes(data[2:4], 'big')
    for i in range(4, 13, 3):
        dataRecv[(i+2)/3]=int.from_bytes(data[i:i+3], 'big')
    for i in range(13, 22, 3):
        dataRecv[(i+2)/3]=int.from_bytes(data[i:i+3], 'big', signed=True)
    print(dataRecv)
    return dataRecv

#-----------------------END SERIAL---------------------------

def main():
    # serial_pub = rospy.Publisher('serial', String, queue_size=10)
    serialInit()
    t1 = 0
    while(True):
        print(time.time()-t1)
        t1 = time.time()
        # data_ser=transmitSerial()
        # processDataSer(data_ser)
        data_ser = receiveSerial()
        print(data_ser)
        # serial_pub.publish(data_ser)
        # print("serial_pub !!")
    


if __name__=="__main__":
    main()