#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
# import serial
import os
import struct
import time
from serial_stm import getDataSerial
from math import cos, sin, pi, fabs
from std_msgs.msg import Float32, String, Float32MultiArray
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from turtlebot3_msgs.msg import SensorState
from geometry_msgs.msg import TransformStamped, Quaternion, Twist, Vector3
# import tf
import tf2_ros
import tf_conversions
# from your_package.msg import Foo

# tf.transformations.quaternion_from_euler

def DEG2RAD(deg):
    return deg*pi/180

def RAD2DEG(rad):
    return rad*180/pi

'''
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
    data1=data1.encode('utf-8')
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
'''
# vl: velo linear (m/s)
# va: velo angular (rad/s)
def updateVelo(vl, va):
    L = 0.2275 #(m) (khoang cach 2 banh xe)
    velo_l = vl-L/2*va
    velo_r = vl+L/2*va
    dir_ = 0
    if velo_l >= 0:
        dir_ = dir_|2
    else:
        pass

    if velo_r >= 0:
        dir_ = dir_|1
    else: 
        pass
    # return velo left, velo right, dir
    return fabs(velo_l), fabs(velo_r), dir_

def teleop_key_Callback(teleop):
    velo_l, velo_r, dir_ = updateVelo(teleop.linear.x, teleop.angular.z)
    sendSerial(velo_l, velo_r, dir_)

def serial_stm_Callback(data):
    global data_ser
    data_ser = data.data

def main():
    global data_ser
    data_ser=np.array([])
    rospy.init_node('serial_pub', anonymous=True)
    Imu_pub = rospy.Publisher('imu', Imu, queue_size=5)
    Odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
    SensorState_pub = rospy.Publisher('sensor_state', SensorState, queue_size=5)
    JointState_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
    # serial_pub = rospy.Publisher('serial', String, queue_size=10)

    rospy.loginfo("Publishing Imu at: " + Imu_pub.resolved_name)
    # serialInit()
    imu_=Imu()
    odom_=Odometry()
    ss_State_=SensorState()
    joint_state_=JointState()
    transform_ = TransformStamped()
    # tf_=tf.TransformBroadcaster()
    # transformer_=tf.Transformer(True, rospy.Duration(10.0))
    transformer_=tf2_ros.TransformBroadcaster()
    x = 0.0
    y = 0.0
    th = 0.0
    vx = 0.1
    vy = 0.0
    vth = 0.0
    last_time = rospy.Time.now()
    rospy.Subscriber('cmd_vel', Twist, teleop_key_Callback, queue_size=10)
    rospy.Subscriber('serial', Float32MultiArray, serial_stm_Callback, queue_size=10)
    while(True):
        # print(data_ser)
        stamp=rospy.Time.now()
        dt = (last_time-stamp).to_sec()
        delta_x = (vx*cos(th)-vy*sin(th))*dt
        delta_y = (vx*sin(th)+vy*cos(th))*dt
        delta_th = vth*dt

        x += delta_x
        y += delta_y
        th += delta_th

        odom_.header.stamp=stamp
        odom_.header.frame_id='odom'
        odom_.child_frame_id='base_link'
        odom_.pose.pose.position.x=x
        odom_.pose.pose.position.y=y
        odom_.pose.pose.position.z=0
        odom_.twist.twist.linear.x=3.6
        odom_.twist.twist.angular.z=4.9

        ss_State_.header.stamp=stamp
        ss_State_.left_encoder=10
        ss_State_.right_encoder=10

        joint_state_.header.stamp=stamp
        joint_state_.header.frame_id='base_link'
        joint_state_.name=["wheel_left_joint", "wheel_right_joint"]
        joint_state_.position=[0,0]
        joint_state_.velocity=[0.1,0.1]
        joint_state_.effort=[0.1,0.1]

        odom_squat = Quaternion(*(tf_conversions.transformations.quaternion_from_euler(0, 0, th)))
        transform_.header.stamp=stamp
        transform_.header.frame_id='odom'
        transform_.child_frame_id='base_footprint'
        transform_.transform.translation.x=0.0
        transform_.transform.translation.y=0.0
        transform_.transform.translation.z=0.0
        transform_.transform.rotation = odom_squat

        
        transformer_.sendTransform(transform_)

        imu_.header.stamp=rospy.Time.now()
        imu_.header.frame_id='imu_link'
        try:
            if (len(data_ser)):
                imu_.linear_acceleration.x =int.from_bytes(data_ser[2], "big")
                imu_.linear_acceleration.y=int.from_bytes(data_ser[3], "big")
                imu_.linear_acceleration.z=int.from_bytes(data_ser[4], "big")
                imu_.angular_velocity.x=int.from_bytes(data_ser[5], "big")
                imu_.angular_velocity.y=int.from_bytes(data_ser[6], "big")
                imu_.angular_velocity.z=int.from_bytes(data_ser[7], "big")

                # imu_.linear_acceleration.x=0.2 
                # imu_.linear_acceleration.y=3.2
                # imu_.linear_acceleration.z=5.6
                # imu_.angular_velocity.x=1.023
                # imu_.angular_velocity.y=2.36
                # imu_.angular_velocity.z=3.37
        except:
            pass

        Imu_pub.publish(imu_)
        Odom_pub.publish(odom_)
        SensorState_pub.publish(ss_State_)
        JointState_pub.publish(joint_state_)
        if rospy.is_shutdown():
            rospy.loginfo("stop serial publisher")
            break


if __name__=="__main__":
    main()