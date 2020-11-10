#!/usr/bin/env python3
import rospy
import numpy as np
import serial
import os
import struct
import time
from math import cos, sin, pi
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from turtlebot3_msgs.msg import SensorState
from geometry_msgs.msg import TransformStamped, Quaternion
import tf
import tf2_ros
import tf_conversions
# from your_package.msg import Foo

# tf.transformations.quaternion_from_euler

def DEG2RAD(deg):
    return deg*pi/180

def RAD2DEG(rad):
    return rad*180/pi

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
        # rospy.loginfo("receive serial: {}".format(receiveData))
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
    Imu_pub = rospy.Publisher('imu', Imu, queue_size=5)
    Odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
    SensorState_pub = rospy.Publisher('sensor_state', SensorState, queue_size=5)
    JointState_pub = rospy.Publisher('joint_states', JointState, queue_size=5)

    rospy.loginfo("Publishing Imu at: " + Imu_pub.resolved_name)
    serialInit()
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
    vx = 0.0
    vy = 0.0
    vth = 0.0
    last_time = rospy.Time.now()
    while(True):
        # data_ser=transmitSerial()

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
        transform_.transform.translation.x=x
        transform_.transform.translation.y=y
        transform_.transform.translation.z=0.0
        transform_.transform.rotation = odom_squat

        
        transformer_.sendTransform(transform_)

        imu_.header.stamp=rospy.Time.now()
        imu_.header.frame_id='imu_link'


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

        Imu_pub.publish(imu_)
        Odom_pub.publish(odom_)
        SensorState_pub.publish(ss_State_)
        JointState_pub.publish(joint_state_)
        if rospy.is_shutdown():
            rospy.loginfo("stop serial publisher")
            break


if __name__=="__main__":
    main()