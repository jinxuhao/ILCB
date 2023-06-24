#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import cv2
import numpy as np
import math
x=0

scann = LaserScan()

def callback(msg):
    global x
    #print(len(msg.ranges)) len is 2019 from 0-360
    current_time = rospy.Time.now()
    scann.header.stamp = current_time
    scann.header.frame_id = 'laser'
    scann.angle_min = -3.1415
    scann.angle_max = 3.1415
    scann.angle_increment = 0.00311202858575
    scann.time_increment = 4.99999987369e-05
    scann.range_min = 0.00999999977648
    scann.range_max = 32.0
    scann.ranges = msg.ranges[0:72]
    scann.intensities = msg.intensities[0:72]
    angle = msg.angle_min
    for r in msg.ranges:
        #change infinite values to 0
        #如果r的值是正负无穷大，归零
        if math.isinf(r) == True:
            r = 0
        #convert angle and radius to cartesian coordinates
        #这里就是将极坐标的信息转为直角坐标信息，只是像素的转化，不对应具体值
        #如果x中的90是正的，则顺时针显示，如果是负的，则逆时针显示。
        x = math.trunc((r * 50.0)*math.cos(angle + (-90.0*3.1416/180.0)))
        y = math.trunc((r * 50.0)*math.sin(angle + (-90.0*3.1416/180.0)))
     
    return x

    #pub.publish(scann)


    

if __name__ == '__main__':
    rospy.init_node('revised_scan', anonymous=True)
    pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 1)
    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        msg = LaserScan()
	
	print('jin')
	print(scann.ranges)
	pub.publish(msg)    # pub.publish(scann)
        rospy.Subscriber('/scan', LaserScan, callback)
        rate.sleep()
    rospy.spin()
