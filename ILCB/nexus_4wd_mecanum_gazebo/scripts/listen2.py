#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import cv2
import numpy as np
import math
import nav_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

alist=[]
rlist=[]
ylist=[]
xlist=[]
scann = LaserScan()
x = 0
y = 0
w_o = 0
x_o = 0alist=[]
rlist=[]
ylist=[]
xlist=[]
y_o = 0
z_o = 0

def callback(msg1):
    global rlist
    global alist
    rlist=[]
    alist=[]	
    #print(len(msg.ranges)) len is 2019 from 0-360
    current_time = rospy.Time.now()
    '''scann.header.stamp = current_time
    scann.header.frame_id = 'laser'
    scann.angle_min = -3.1415
    scann.angle_max = 3.1415
    scann.angle_increment = msg.angle_increment
    scann.time_increment = 4.99999987369e-05
    scann.range_min = 0.00999999977648
    scann.range_max = 32.0
    scann.ranges = msg.ranges[0:72]
    scann.intensities = msg.intensities[0:72]'''
    angle = msg1.angle_min

    for r in msg1.ranges:
	
        #change infinite values to 0
        #如果r的值是正负无穷大，归零
        if math.isinf(r) == True:
            r = 0
        #convert angle and radius to cartesian coordinates
	angle= angle + msg1.angle_increment
        rlist.append(r)
        alist.append(angle)
    return rlist,alist

    #pub.publish(scann)


def Trans_robot_pose(msg):  # 回调函数
    # 位置坐标声明
    global x
    global y
    global w_o    # 当前小车位姿的四元数信息.8590735439489559
    global x_o
    global y_o
    global z_o
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w_o = msg.pose.pose.orientation.w
    x_o = msg.pose.pose.orientation.x
    y_o = msg.pose.pose.orientation.y
    z_o = msg.pose.pose.orientation.z
   
    return w_o, y_o, z_o, x_o, x, y

if __name__ == '__main__':
    rospy.init_node('revised_scan', anonymous=True)
    pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 1)
    turtle_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        
	msg = geometry_msgs.msg.Twist()
	(roll, pitch, yaw) = euler_from_quaternion([x_o,y_o,z_o,w_o])
	print('yaw',yaw)
	print(x)
	msg1 = LaserScan()
	global xlist
	global ylist
	xlist=[]
	ylist=[]
	
	for i in range(len(rlist)):
		
		x = (rlist[i])*math.cos((alist[i])*3.1416/180.0)
		y = (rlist[i])*math.sin((alist[i])*3.1416/180.0)
		ylist.append(y)
		xlist.append(x)
    
	print('jin')
	print(xlist)
	print(ylist)
	turtle_vel.publish(msg)    # 发布运动指令
        rospy.Subscriber('/odom', nav_msgs.msg.Odometry,  Trans_robot_pose)
	pub.publish(msg1)    # pub.publish(scann)
        rospy.Subscriber('/scan', LaserScan, callback)
        rate.sleep()
    rospy.spin()
