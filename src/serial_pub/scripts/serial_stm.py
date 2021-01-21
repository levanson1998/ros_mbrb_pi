#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import serial
import time
import numpy as np
from math import fabs, cos, sin, pi
import traceback

#-----------------------BEGIN SERIAL---------------------------
time_t = 0.1
get_ser = 0
put_ser = 0
va_pre=[0, 0, 0]
vl_pre=[0, 0, 0]
ser_error=False
data_temp=[]
file = open("/home/ubuntu/ros_mbrb/src/serial_pub/scripts/datastm.txt", "a")
file.writelines("New data !\n\n\n")
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
    time.sleep(0.1)
    sendSerial(0, 0, 0)
    # ser.close()
    # ser.open()
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
    # ser.flushOutput()
    ser.write(packet)
    # print(ser.out_waiting())
    # print("send serial !")

def receiveSerial():
    global ser, ser_error, data_temp
    try:
        data_watiting = ser.inWaiting()
        data_temp+=ser.read(data_watiting)
        for i in range(len(data_temp)):
            if ((data_temp[i]==127) and (data_temp[i+28]==27)):
                data = data_temp[i:i+29]
                del data_temp[:i+29]
                receiveData=processDataSer(data)
                ser_error = False
                return receiveData
        raise NameError('NO DATA!')
    except Exception:
        ser_error = True
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
    dataRecv.append(int.from_bytes(data[23:27], 'big', signed=True)/1000000)
    dataRecv.append(data[27])
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
    global vl_pre, va_pre
    L = 0.2275 #(m) (khoang cach 2 banh xe)
    # if (abs(va)>=0.3):
    #     va = va*va/(abs(va))
    file.writelines("{}:  {}  {}\n".format(time.time(), va, vl))
    # if (va-va_pre[0] >= 1.0):
    #     va=va_pre[0]
    # if (vl-vl_pre[0] >= 1.0):
    #     va=va_pre[0]
    va_pre[1] = va_pre[0]
    vl_pre[1] = vl_pre[0]
    va_pre[0] = va
    vl_pre[0] = vl

    
    va = -va
    # vl = -vl
    # va = va/2
    # print("vl: {}\nva: {}".format(vl, va))
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
    sendSerial(put_ser[0],put_ser[1], put_ser[2])

def main():
    global get_ser
    global put_ser, ser_error
    t1 = time.time()
    x,y,th=0,0,0
    show=0
    th_origign1, th_origign = 0, 0
    th_onetime = 0
    yaw_pre = 0
    yaw = 0
    # vth = 10

    serialInit()
    rospy.init_node('serial_stm', anonymous=True)
    serial_pub = rospy.Publisher('serial', Float32MultiArray, queue_size=10)
    rospy.Subscriber('cmd_vel', Twist, teleop_key_Callback, queue_size=10)
    floatarray = Float32MultiArray()
    while(True):
        # print("\n")
        dt1=time.time()-t1
        t1 = time.time()
        dt = 0.1
        get_ser = receiveSerial()
        
        if not get_ser[8] == 0: 
            vth = (-get_ser[8] + yaw)/dt
            yaw = get_ser[8]
        if th_onetime <= 3:
            th_onetime+=1
            th_origign1 += yaw
            th_origign = th_origign1/th_onetime
            # print("23123")
        # print(th_origign)
        # 0.00015708: encoder -> quang duong(m)
        v_right = get_ser[0]*0.00014 # (m/s)
        v_left = get_ser[1]*0.00014

        # get_ser[0] = get_ser[0]*0.00015708
        # get_ser[1] = get_ser[1]*0.00015708

        vx = ((v_left+v_right)/2)*(1000) # *10 # (m/s)
        vy = 0
        
        # vth = ((v_left-v_right)/0.2781)*(-1000)# *10 # (rad/s)
        th = -yaw + th_origign

        delta_x = (vx*cos(th))*dt/1000
        delta_y = (vx*sin(th))*dt/1000
        # delta_th = th - th_pre

        x += delta_x*10
        y += delta_y*10
        # th += delta_th
        
        # print(len(get_ser))

        get_ser[-1] = vth
        get_ser[-2] = vy
        get_ser[-3] = vx
        get_ser[-4] = y
        get_ser[-5] = x
        get_ser[-6] = th

        # if ((ser_error==False) and (show == 10)):
        if show == 10:
            show = 0
            print("\033c")
            rospy.loginfo("Connect to {}".format(ser.name))
            # rospy.loginfo("\nyaw: {}\nyaw_pre: {}\n".format(yaw, yaw_pre))
            rospy.loginfo("\ndt1: {}\nenc[0]: {}\nenc[1]:; {}\nv_right: {}\nv_left: {}\nvx: {}\nvy: {}\nvth: {}\ndelta_x: {}\ndelta_y: {}\nx: {}\ny: {}\nth: {} (rad)\
                \nimu_ax: {}\nimu_ay:{}\nimu_az: {}\nimu_gx: {}\nimu_gy: {}\nimu_gz: {}\nimu_yaw: {}"\
                .format(dt1, get_ser[0], get_ser[1], v_right, v_left, vx, vy, vth, delta_x, delta_y, round(x, 4), round(y, 4), round(th, 4), \
                    round(get_ser[2], 4), round(get_ser[3], 4), round(get_ser[4], 4), round(get_ser[5], 4), round(get_ser[6], 4), round(get_ser[7], 4), round(get_ser[8], 8)))
            rospy.loginfo("\nyaw: {}\nyaw_pre: {}".format(yaw, yaw_pre))
        show+=1
        
        # try:
        #     # sendSerial(put_ser[0],put_ser[1], put_ser[2])
        # except:
        #     pass
        floatarray.data=get_ser
        serial_pub.publish(floatarray)
        # print("serial_pub !!")
        # print("\033c")
        # rospy.loginfo("dt1: {}".format(dt1))
        if rospy.is_shutdown():
            file.close()
            ser.close()
            time.sleep(0.5)
            rospy.loginfo("stop serial stm")
            break

if __name__=="__main__":
    main()