#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import serial
import time
import numpy as np
from math import fabs, cos, sin, pi


#-----------------------BEGIN SERIAL---------------------------
time_t = 0.1
get_ser = 0
put_ser = 0
# file = open("/home/ubuntu/ros_mbrb/src/serial_pub/scripts/datastm.txt", "a")
# file.writelines("New data !\n\n\n")
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

def sendSerial(velo_r, velo_l, dir_):
    global ser
    print("\nvelo_r: {} \nvelo_l: {} \ndir_: {}".format(round(velo_r, 6), round(velo_l, 6), round(dir_, 6)))
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

    byte_start = 200
    byte_end = 201

    packet = [byte_start, a3H, a4L, a4H, a1H, a2L, a2H, dir_, byte_end]
    ser.flushOutput()
    ser.write(packet)
    # print(ser.out_waiting())
    # print("send serial !")

def receiveSerial():
    global ser
    try:
        data=ser.read(25)
        print("len: {}, inWaiting: {}".format(len(data), ser.inWaiting()))
        # print("data: {}".format(data))
        # print("data[-1]: {}, data[-2]: {}".format(data[-1], data[-2]))
        if not ((data[-1]==27) and (data[0]==127)):
            print(create_error)
        # data = ser.readline(-1)
        # ser.flush()
        ser.flushInput()
        # print(data)
        
        receiveData=processDataSer(data)
        return receiveData
    except:
        rospy.loginfo("serial no data !!!")
        a = []
        for i in range(15):
            a.append(0)
        return a

def processDataSer(data):
    # print(data)
    # print(len(data))
    dataRecv=[]
    dataRecv.append(int.from_bytes(data[1:3], 'big', signed=True))
    dataRecv.append(int.from_bytes(data[3:5], 'big', signed=True))
    for i in range(5, 14, 3):
        dataRecv.append(int.from_bytes(data[i:i+3], 'big', signed=True)/1000)
    for i in range(14, 23, 3):
        dataRecv.append(int.from_bytes(data[i:i+3], 'big', signed=True)/1000)
    dataRecv.append(data[23])
    dataRecv.append(0.0) # th
    dataRecv.append(0.0) # x
    dataRecv.append(0.0) # y
    dataRecv.append(0.0) # vx
    dataRecv.append(0.0) # vy
    dataRecv.append(0.0) # vth
    # print(dataRecv)
    dataRecv = np.array(dataRecv)
    return dataRecv

#-----------------------END SERIAL---------------------------

def putDataSerial(_put_ser):
    global put_ser
    # put_ser=_put_ser
    # put_ser = 123
    # print("putDataSerial: {}".format(put_ser))
    return put_ser

# vl: velo linear (m/s)
# va: velo angular (rad/s)
def updateVelo(vl, va):
    L = 0.2275 #(m) (khoang cach 2 banh xe)
    # if (abs(va)>=0.3):
    #     va = va*va/(abs(va))
    # file.writelines("{}:  {}  {}\n".format(time.time(), va, vl))
    va = -va
    # vl = -vl
    # va = va/2
    print("vl: {}\nva: {}".format(vl, va))
    velo_l = (vl+L/2*va)
    velo_r = (vl-L/2*va)

    dir_ = 0
    if velo_l >= 0:
        dir_ = dir_|2
    else:
        pass

    if velo_r >= 0:
        dir_ = dir_|1
    else: 
        pass
    return fabs(velo_r), fabs(velo_l), dir_

def teleop_key_Callback(teleop):
    global put_ser
    velo_r, velo_l, dir_ = updateVelo(teleop.linear.x, teleop.angular.z)
    put_ser = [velo_r, velo_l, dir_]

def main():
    global get_ser
    global put_ser
    serialInit()
    rospy.init_node('serial_stm', anonymous=True)
    serial_pub = rospy.Publisher('serial', Float32MultiArray, queue_size=10)
    rospy.Subscriber('cmd_vel', Twist, teleop_key_Callback, queue_size=10)
    floatarray = Float32MultiArray()
    
    t1 = time.time()
    x,y,th=0,0,0
    while(True):
        # print("\n")
        dt=time.time()-t1
        t1 = time.time()
        get_ser = receiveSerial()
        
        # 0.00015708: encoder -> quang duong(m)
        v_right = get_ser[0]*0.00015708 # (m/s)
        v_left = get_ser[1]*0.00015708

        # get_ser[0] = get_ser[0]*0.00015708
        # get_ser[1] = get_ser[1]*0.00015708

        vx = ((v_left+v_right)/2)*(10) # *10 # (m/s)
        vy = 0
        vth = ((v_left-v_right)/0.2781)*(-10)# *10 # (rad/s)

        delta_x = (vx*cos(th))*dt
        delta_y = (vx*sin(th))*dt
        delta_th = vth*dt

        x += delta_x
        y += delta_y
        th += delta_th

        # print(len(get_ser))

        get_ser[-1] = vth
        get_ser[-2] = vy
        get_ser[-3] = vx
        get_ser[-4] = y
        get_ser[-5] = x
        get_ser[-6] = th

        # print("putser: {}".format(put_ser))
        print("\033c")
        rospy.loginfo("\nenc[0]: {}\nenc[1]:; {}\nv_right: {}\nv_left: {}\nvx: {}\nvy: {}\nvth: {}\ndelta_x: {}\ndelta_y: {}\ndelta_th: {}\nx: {}\ny: {}\nth: {} (rad)\
            \nimu_ax: {}\nimu_ay:{}\nimu_az: {}\nimu_gx: {}\nimu_gy: {}\nimu_gz: {}"\
            .format(get_ser[0], get_ser[1], v_right, v_left, vx, vy, vth, delta_x, delta_y, delta_th, round(x, 4), round(y, 4), round(th, 4), \
                round(get_ser[2], 4), round(get_ser[3], 4), round(get_ser[4], 4), round(get_ser[5], 4), round(get_ser[6], 4), round(get_ser[7], 4)))
        try:
            sendSerial(put_ser[0],put_ser[1], put_ser[2])
        except:
            pass
        floatarray.data=get_ser
        serial_pub.publish(floatarray)
        # print("serial_pub !!")
        if rospy.is_shutdown():
            # file.close()
            rospy.loginfo("stop serial stm")
            break

if __name__=="__main__":
    main()