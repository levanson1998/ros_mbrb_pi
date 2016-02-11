#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os
import struct
import time
from math import cos, sin, pi, fabs
from std_msgs.msg import Float32, String, Float32MultiArray
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from turtlebot3_msgs.msg import SensorState
from geometry_msgs.msg import TransformStamped, Quaternion, Twist, Vector3
from tf import TransformBroadcaster, transformations
import tf2_ros
import tf_conversions

def DEG2RAD(deg):
    return deg*pi/180

def RAD2DEG(rad):
    return rad*180/pi

def serial_stm_Callback(data):
    global data_ser
    data_ser = data.data
    # print(data_ser)

def main():
    global data_ser
    data_ser=np.array([0,0,0,0,0,0,0,0])
    rospy.init_node('serial_pub', anonymous=True)
    Imu_pub = rospy.Publisher('imu', Imu, queue_size=5)
    Odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
    SensorState_pub = rospy.Publisher('sensor_state', SensorState, queue_size=5)
    JointState_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
    # serial_pub = rospy.Publisher('serial', String, queue_size=10)

    rospy.loginfo("Publishing Imu at: " + Imu_pub.resolved_name)
    # serialInit()
    imu_=Imu()
    
    ss_State_=SensorState()
    joint_state_=JointState()
    
    # tf_=tf.TransformBroadcaster()
    # transformer_=tf.Transformer(True, rospy.Duration(10.0))
    odom_broadcaster=tf2_ros.TransformBroadcaster()
    # odom_broadcaster = TransformBroadcaster()
    
    x = 0.0
    y = 0.0
    th = 0.0
    vx = 0.1
    vy = 0.0
    vth = 0.0
    last_time = rospy.Time.now()
    tt1 = time.time()
    rospy.Subscriber('serial', Float32MultiArray, serial_stm_Callback, queue_size=10)
    while(True):
        # rospy.loginfo("")
        # print(time.time()-tt1)
        tt1 = time.time()
        prt=""
        for i in range(len(data_ser)):
            prt += "\n{}".format(data_ser[i])
        # print(prt)
        stamp=rospy.Time.now()
        # dt = data_ser[-1]
        # v_left = data_ser[0]
        # v_right = data_ser[1]
        # vx = ((v_left+v_right)/2)
        # vy = 0
        # vth = ((v_right-v_left)/0.2275)

        # delta_x = (vx*cos(th)-vy*sin(th))*dt
        # delta_y = (vx*sin(th)+vy*cos(th))*dt
        # delta_th = vth*dt

        # x += delta_x
        # y += delta_y
        # th += delta_th

        vth = data_ser[-1]
        vy = data_ser[-2]
        vx = data_ser[-3]
        y = data_ser[-4]
        x = data_ser[-5]
        th = data_ser[-6]

        # print("\033c")
        rospy.loginfo("\nvx: {}\nvy: {}\nvth: {}\nx: {}\ny: {}\nth: {}".format(vx, vy, vth, x, y, th))
        for i in range(2, 8, 1):
            prt+="\ndata: {}".format(data_ser[i])
        # print(prt)

        ss_State_.header.stamp=stamp
        ss_State_.left_encoder=10
        ss_State_.right_encoder=10

        joint_state_.header.stamp=stamp
        joint_state_.header.frame_id='base_link'
        joint_state_.name=["wheel_left_joint", "wheel_right_joint"]
        joint_state_.position=[0,0]
        joint_state_.velocity=[0.1,0.1]
        joint_state_.effort=[0.1,0.1]

        # odom_squat = Quaternion(*(tf_conversions.transformations.quaternion_from_euler(0, 0, th)))
        odom_squat = Quaternion(*(transformations.quaternion_from_euler(0,0,th)))
        print("\nodom_squat.x: {}\nodom_squat.y: {}\nodom_squat.z: {}\nodom_squat.w: {}".format(odom_squat.x, odom_squat.y, odom_squat.z, odom_squat.w))

        transform_ = TransformStamped()
        transform_.header.stamp=stamp
        transform_.header.frame_id='odom'
        transform_.child_frame_id='base_link'
        # transform_.child_frame_id='base_link'
        transform_.transform.translation.x=x
        transform_.transform.translation.y=y
        transform_.transform.translation.z=0.0
        transform_.transform.rotation.x = odom_squat.x
        transform_.transform.rotation.y = odom_squat.y
        transform_.transform.rotation.z = odom_squat.z
        transform_.transform.rotation.w = odom_squat.w

        # print(transform_)

        odom_broadcaster.sendTransform([transform_])

        odom_=Odometry()
        
        odom_.header.stamp=stamp
        odom_.header.frame_id='odom'
        
        odom_.pose.pose.position.x=x
        odom_.pose.pose.position.y=y
        odom_.pose.pose.position.z=0
        odom_.pose.pose.orientation=odom_squat

        odom_.child_frame_id='base_link'
        odom_.twist.twist.linear.x=vx
        odom_.twist.twist.linear.y=vy
        odom_.twist.twist.linear.z=0
        odom_.twist.twist.angular.x=0
        odom_.twist.twist.angular.y=0
        odom_.twist.twist.angular.z=vth

        # transformer_.sendTransform(transform_)

        imu_.header.stamp=rospy.Time.now()
        imu_.header.frame_id='imu_link'
        imu_.linear_acceleration.x =data_ser[2]
        imu_.linear_acceleration.y=data_ser[3]
        imu_.linear_acceleration.z=data_ser[4]
        imu_.angular_velocity.x=data_ser[5]
        imu_.angular_velocity.y=data_ser[6]
        imu_.angular_velocity.z=data_ser[7]

        # Imu_pub.publish(imu_)
        Odom_pub.publish(odom_)
        SensorState_pub.publish(ss_State_)
        JointState_pub.publish(joint_state_)
        if rospy.is_shutdown():
            rospy.loginfo("stop serial publisher")
            break


if __name__=="__main__":
    main()