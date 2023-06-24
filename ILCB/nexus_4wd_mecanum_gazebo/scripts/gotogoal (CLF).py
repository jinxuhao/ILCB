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
v=0.00
w=0
yaw_t = 0
liner_speed = 0
angular_speed = 0
liner_speed_old = 0.00
angular_speed_old = 0
nominalController = 0

X_t = 10
Y_t = 10
X_sim = 10       # 目标点x坐标
Y_sim = 10      # 目标点y坐标

kp=0.6
kd=0.5
dt=0.01


##a=5
##b=5
##r=1
##shezhiyuan
a1=2.5
b1=1.0
r1=1.4

a2=3##-0.5
b2=4.5
r2=1.5

c1=100
c2=250
can1=0.3
can2=0.7

##canshu
M=0.57
p11= 1.0
p12= 0.5
p22= 1.0
p21=0.5

can =15
canshu1 =1.0
canshu2 =1.5

p111 =can*p11
p112 =can*p12
p121 =can*p21
p122 =can*p22

p211 =p11*1	
p212 =p12*1
p221 =p21*1
p222 =p22*1

p311 =2*p11*58##58
p312 =2*p12
p321 =2*p21
p322 =2*p22*15##15



##thd = yaw_t##-math.pi/4 
vd  = 0
Ulin= 0

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
        ##if yaw < 0:
          ##  yaw = yaw + 2 * math.pi

	X_t = X_sim
        Y_t = Y_sim
	 ##math.pow((X_t - x), 2) + math.pow((Y_t - y), 2)

##D_err =math.sqrt(math.pow((X_t - x), 2) + math.pow((Y_t - y), 2))

	##dv = p1*(x-X_t)*v*math.cos(yaw*180/math.pi) + p2*(y-Y_t)*v*math.sin(yaw*180/math.pi)+p3*(yaw - thd)*w+p4*(v - vd)*Ulin




 	
	## Upd
	##nominalController 
	
	##Updlin= kp*(D_err)+kd*liner_speed_old
##Ulin= kp*(D_err)-kd*liner_speed_old



	##h =(math.pow((x-a), 2) + math.pow((y-b), 2) - math.pow((r), 2))
   	##dh =(2*(x-a)*math.cos(yaw)+2*(y-b)*math.sin(yaw))*Ulin	
	##g = (2*(x-a)*math.cos(yaw*180/3.14)+2*(y-b)*math.sin(yaw*180/3.14))
	if y < 2.5 and   x <2.5 :
        	a=a1
		b=b1
		r=r1
	##	kp=0.2
	##	kd=0.7
	##	can1=0.3
	##	can2=0.7
        if y >= 2.5 or   x >= 2.5 :
        	a=a2
		b=b2
		r=r2
		
	##	kp=0.6
	##	kd=0.5
	##	can1=0.35
	##	can2=0.7

	G22 =(2*(x-a)*math.cos(yaw*180/3.14)+2*(y-b)*math.sin(yaw*180/3.14))
	G21 = (-2*(x-a)*math.sin(yaw*180/3.14)+2*(y-b)*math.cos(yaw*180/3.14))*v##*0.55
	h2 = 2*v*v+c1*(2*(x-a)*math.cos(yaw*180/3.14)*v+2*(y-b)*math.sin(yaw*180/3.14)*v)+c2*((math.pow((x-a), 2) + math.pow((y-b), 2) - math.pow((r), 2)))
	##g1 =
	##g2 =

	
	
	x1=x-X_t
	y1=y-Y_t
	##th1=yaw-thd
	v1=v-vd

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
            yaw_t = math.atan((Y_t - y) / (X_t - x)) - math.pi
        if (Y_t - y) < 0 and (X_t - x) == 0:
            yaw_t = -0.5 * math.pi
        if (Y_t - y) < 0 and (X_t - x) > 0:
            yaw_t = math.atan((Y_t - y) / (X_t - x)) ##+ 2 * math.pi

        Theta_err = - yaw_t + yaw
        if Theta_err < -math.pi:
            Theta_err = Theta_err + 2 * math.pi
        if Theta_err > math.pi:
            Theta_err = Theta_err - 2 * math.pi

	th1=Theta_err



	G11=(p211*x1+p221*y1+2.0*p311*th1+p211*x1+p221*y1+p312*v1+p321*v1)/2.0
	G12=(p212*x1+p222*y1+2.0*p322*v1+p212*x1+p222*y1+p312*th1+p321*th1)/(2*M)
	V=p111*x1*x1+p112*x1*y1+p211*x1*th1+p212*x1*v1+p122*y1*y1+p121*x1*y1+p221*y1*th1+p222*y1*v1+p311*th1*th1+p211*th1*x1+p221*y1*th1+p312*th1*v1+p322*v1*v1+p212*x1*v1+p222*v1*y1+p321*v1*th1
	h1= -1.0*canshu2*V/2.0
	##h1 =-1.0*(p1*(x-X_t)*v*math.cos(yaw*180/math.pi) + p2*(y-Y_t)*v*math.sin(yaw*180/math.pi))
       
	##Updang= kp*(Theta_err)+kd*angular_speed_old
##Uang= kp*(Theta_err)-kd*angular_speed_old
	



	##G21=G21*250

	if y >= 2.5 or x >= 2.5 :
		h1=h1*0.2
		h2=h2*2
		G21=G21*250


	P = 2*matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,canshu1]])   # matrix里区分int和double，所以数字后面都需要加小数点
	q = matrix([0.0,0.0,0.0])
	G = matrix([[25*G11,25*G21],[13*G12,13*G22],[-1.0,0.0]])
##G = matrix([[-1.0* g],[0.0]])
	h = matrix([h1,h2])

	sol = solvers.qp(P,q,G,h)
	
	Uang = sol['x'][0,0]##Updlin
	Ulin = sol['x'][1,0]##
	##if Uang >   2 * math.pi:
	##	Uang = Uang-2 * math.pi
	##if Uang <   -2 * math.pi:
	##	Uang = Uang+2 * math.pi
		
	
	print('v',v)
	print('w',w)	
	print('yaw',yaw)
	print('x',x)
	print('y',y)
	print('h1',h1)
	print('h2',h2)
	print('a',a)
	print('ulin',Ulin )
	##print ##sol['x'][0,0]
	##print ('updl',Updlin )

	##dh =(2*(x-a)*math.cos(yaw)+2*(y-b)*math.sin(yaw))*Ulin
        # 目前先只使用最简单的比例控制
        ##liner_speed = 0.1 * (Updlin*dt+liner_speed_old)
        ##angular_speed = 0.9 * (Updang)
 	
	##Uang=0.5*Uang

	if Ulin >4.1:
      		Ulin = 4.1	
	if Ulin <-4.1:
      		Ulin = -4.1

	##Uang = Uang *0.28


	if Uang >0.785:
      		Uang = 0.785	
	if Uang <-0.785:
      		Uang= -0.785


	liner_speed = (Ulin*dt+liner_speed_old)
	angular_speed =  (Uang)##*0.5##*dt*50##*100##*19000##*0.1*dt##*100
	
	if liner_speed >0.1:
      		liner_speed = 0.1	
	if liner_speed <-0.1:
      		liner_speed = -0.1

        msg.linear.x = liner_speed
        msg.angular.z = -angular_speed

	v = liner_speed
	w = angular_speed
        liner_speed_old = liner_speed
        angular_speed_old = angular_speed
	
	
	##print msg.linear.x
	##print msg.angular.z
	
	print('uang' ,Uang)
	##print ('upda',Updang )
	print('sudu',msg.linear.x)
	print('jiaosudu',msg.angular.z)
	##print('dv',dv)
	

	
	
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
