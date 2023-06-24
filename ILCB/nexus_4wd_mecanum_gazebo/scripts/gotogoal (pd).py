#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import rospy
import math
import nav_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
	
from cvxopt import solvers,matrix
import numpy as np

# 定义全局变量
x = 0
y = 0
w_o = 0
x_o = 0
y_o = 0
z_o = 0
v=0
w=0
yaw_t = 0
liner_speed = 0
angular_speed = 0
liner_speed_old = 0
angular_speed_old = 0
nominalController = 0

X_t = 0
Y_t = 0
X_sim = 10       # 目标点x坐标
Y_sim = 10       # 目标点y坐标

kp=0.6
kd=0.5
dt=0.1


##a=5
##b=5
##r=1
a1=2
b1=1.5
r1=0.6

a2=4.85
b2=5.25
r2=1

c1=5
c2=5


def Trans_robot_pose(msg):  # 回调函数
    # 位置坐标声明
    global x
    global y
    global w_o    # 当前小车位姿的四元数信息
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
    rospy.init_node('item1')

    turtle_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        msg = geometry_msgs.msg.Twist()
        (roll, pitch, yaw) = euler_from_quaternion([x_o,y_o,z_o,w_o])  # 将四元数转化为roll, pitch, yaw
        if yaw < 0:
            yaw = yaw + 2 * math.pi
	X_t = X_sim
        Y_t = Y_sim
	 ##math.pow((X_t - x), 2) + math.pow((Y_t - y), 2)

	D_err =math.sqrt(math.pow((X_t - x), 2) + math.pow((Y_t - y), 2))

	##while D_err >=0.1:


 	
	## Upd
	##nominalController 
	
	Updlin= kp*(D_err)+kd*liner_speed_old
##Ulin= kp*(D_err)-kd*liner_speed_old



	##h =(math.pow((x-a), 2) + math.pow((y-b), 2) - math.pow((r), 2))
   	##dh =(2*(x-a)*math.cos(yaw)+2*(y-b)*math.sin(yaw))*Ulin	
	##g = (2*(x-a)*math.cos(yaw*180/3.14)+2*(y-b)*math.sin(yaw*180/3.14))
	if y < 3 and   x <3 :
        	a=a1
		b=b1
		r=r1
		kp=0.2
		kd=0.7
		can1=0.3
		can2=0.7
        if y > 3 or   x >3 :
        	a=a2
		b=b2
		r=r2
		kp=0.6
		kd=0.5
		can1=0.35
		can2=0.7

	g1 =(2*(x-a)*math.cos(yaw*180/3.14)+2*(y-b)*math.sin(yaw*180/3.14))
	g2 = (-2*(x-a)*math.sin(yaw*180/3.14)+2*(y-b)*math.cos(yaw*180/3.14))*v
	h1 = 2*v*v+c1*(2*(x-a)*math.cos(yaw*180/3.14)+2*(y-b)*math.sin(yaw*180/3.14))+c2*((math.pow((x-a), 2) + math.pow((y-b), 2) - math.pow((r), 2)))
        # 判断坐标象限
        if (Y_t - y) == 0 and (X_t - x) > 0:
            yaw_t = 0
        if (Y_t - y) > 0 and (X_t - x) > 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x))
        if (Y_t - y) > 0 and (X_t - x) == 0:
            yaw_t = 0.5 * math.pi
        if (Y_t - y) > 0 and (X_t - x) < 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + math.pi
        if (Y_t - y) == 0 and (X_t - x) < 0:
            yaw_t = math.pi
        if (Y_t - y) < 0 and (X_t - x) < 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + math.pi
        if (Y_t - y) < 0 and (X_t - x) == 0:
            yaw_t = 1.5 * math.pi
        if (Y_t - y) < 0 and (X_t - x) > 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) + 2 * math.pi

        Theta_err = yaw_t - yaw
        if Theta_err < -math.pi:
            Theta_err = Theta_err + 2 * math.pi
        if Theta_err > math.pi:
            Theta_err = Theta_err - 2 * math.pi
	Updang= kp*(Theta_err)+kd*angular_speed_old
##Uang= kp*(Theta_err)-kd*angular_speed_old

	P = 2*matrix([[1.0,0.0],[0.0,1.0]])   # matrix里区分int和double，所以数字后面都需要加小数点
	q = matrix([-2.0*Updlin,-2.0*Updang])
	G = matrix([[-1.0* g1],[-1.0*g2]])
##G = matrix([[-1.0* g],[0.0]])
	h = matrix([h1])

	sol = solvers.qp(P,q,G,h)
	
	Ulin = sol['x'][0,0]##Updlin
	Uang = sol['x'][1,0]
	print('yaw',yaw)
	print('x',x)
	print('y',y)
	print('kp',kp)
	print('ulin',Ulin )
	##print ##sol['x'][0,0]
	print ('updl',Updlin )

	dh =(2*(x-a)*math.cos(yaw)+2*(y-b)*math.sin(yaw))*Ulin
        # 目前先只使用最简单的比例控制
        ##liner_speed = 0.1 * (Updlin*dt+liner_speed_old)
        ##angular_speed = 0.9 * (Updang)
 	liner_speed = can1*(Ulin*dt+liner_speed_old)
	angular_speed = can2 * (Uang)

        msg.linear.x = liner_speed
        msg.angular.z = -angular_speed

	v = liner_speed
	w = angular_speed
        liner_speed_old = liner_speed
        angular_speed_old = angular_speed
	
	
	##print msg.linear.x
	##print msg.angular.z
	
	print('uang' ,Uang)
	print ('upda',Updang )
	print('sudu',msg.linear.x)
	
	

	
	
	##P = matrix([[2.0,0.0],[0.0,2.0]])   # matrix里区分int和double，所以数字后面都需要加小数点
	##q = matrix([-2*Upblin,-2*Upbang])
	##G = matrix([[- g],[0.0]])
	##h = matrix([0.0])

	##sol = solvers.qp(P,q,G,h)   # 调用优化函数solvers.qp求解
	##print sol['x']  # 打印结果，sol里面还有很多其他属性，读者可以自行了解


        turtle_vel.publish(msg)    # 发布运动指令
        rospy.Subscriber('/odom', nav_msgs.msg.Odometry,  Trans_robot_pose) # 获取位姿信息

        rate.sleep()

	 # 如果到达最后一个点，让小车停下来
        ##msg.linear.x = 0.0
       ## msg.angular.z = 0.0
       ## print msg.linear.x

        ##turtle_vel.publish(msg)                 # 向/cmd_vel话题发布数据
       ## rospy.Subscriber('/tb3_0/odom', nav_msgs.msg.Odometry, Trans_robot_pose)

        ##rate.sleep()  # 以固定频率执行


    rospy.spin()
