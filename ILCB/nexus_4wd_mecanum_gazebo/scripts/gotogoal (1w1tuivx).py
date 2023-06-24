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
vx=0
vy=0
vxold = 0.00
vyold = 0.00
w_old=0

angular_speed_old = 0
nominalController = 0
yaw_old=0
yaw_wold=0

X_t = 3
Y_t = 3
X_sim = 3       # 目标点x坐标
Y_sim = 3      # 目标点y坐标

r=0.05
vx=0
vy=0
kp=0.2
kd=0.5
##a=5
##b=5
##r=1
##shezhiyuan
a1=5#1.75
b1=5#1.0
r1=0.6
c=0.075



a2=3.1#3.25##-0.5
b2=3.9
r2=0.65

a3=8#3.25##-0.5
b3=7.5
r3=0.5

u1_old=0
u2_old=0
u3_old=0
u4_old=0



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
	c1 = math.cos(yaw*180/3.14)+math.sin(yaw*180/3.14)
	c2 = math.cos(yaw*180/3.14)-math.sin(yaw*180/3.14)
	c3=1/(0.2+0.14)
	X_t = X_sim
        Y_t = Y_sim
	 ##math.pow((X_t - x), 2) + math.pow((Y_t - y), 2)

	D_err =math.sqrt(math.pow((X_t - x), 2) + math.pow((Y_t - y), 2))
	D_err1 =math.sqrt(math.pow((X_t - x-0.2), 2) + math.pow((Y_t - y-0.14), 2))
	D_err2 =math.sqrt(math.pow((X_t - x+0.2), 2) + math.pow((Y_t - y-0.14), 2))
	D_err3 =math.sqrt(math.pow((X_t - x-0.2), 2) + math.pow((Y_t - y+0.14), 2))
	D_err4 =math.sqrt(math.pow((X_t - x+0.2), 2) + math.pow((Y_t - y+0.14), 2))
	u1d= kp*(D_err1)+kd*u1_old
	u2d= kp*(D_err2)+kd*u2_old
	u3d= kp*(D_err3)+kd*u3_old
	u4d= kp*(D_err4)+kd*u4_old
	##dv = p1*(x-X_t)*v*math.cos(yaw*180/math.pi) + p2*(y-Y_t)*v*math.sin(yaw*180/math.pi)+p3*(yaw - thd)*w+p4*(v - vd)*Ulin




 	
	## Upd
	##nominalController 
	
	##Updlin= kp*(D_err)+kd*liner_speed_old
##Ulin= kp*(D_err)-kd*liner_speed_old



	##g = (2*(x-a)*math.cos(yaw*180/3.14)+2*(y-b)*math.sin(yaw*180/3.14))
	


	#if  y < 2 and   x <2 :
        a=a1
	b=b1
	rd=1
	
      
	h1 =(math.pow((x-a), 2) + math.pow((y-b), 2) - math.pow((rd), 2))
   	#dh =(2*(x-a)*vx+2*(y-b)*vy)	

	##th1=yaw-thd
	#v1=v-vd
	G1=2*(x-a)*r*c1/4 -2*(y-b)*r*c2/4  
	G2=2*(x-a)*r*c2/4 +2*(y-b)*r*c1/4
	G3=G2
	G4=G1

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
	#Updang= kp*(Theta_err)+kd*angular_speed_old



	P = 2*matrix([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])   # matrix里区分int和double，所以数字后面都需要加小数点
	q = matrix([-2.*u1d, -2.*u2d, -2.*u3d, -2.*u4d])
	G = matrix([[-G1],[-G2],[-G3],[-G4]])
	h = matrix([c*h1])

	sol = solvers.qp(P,q,G,h)
	
	u1 = sol['x'][0,0]##Updlin
	u2 = sol['x'][1,0]
	u3 = sol['x'][2,0]
	u4 = sol['x'][3,0]
##
	##if Uang >   2 * math.pi:
	##	Uang = Uang-2 * math.pi
	##if Uang <   -2 * math.pi:
	##	Uang = UanTheta_erg+2 * math.pi
		
	
	print('v',v)
	print('u1_old',u1_old)	
	
	print('x',x)
	print('y',y)
	#print('h1',h1)
	#print('h2',h2)
	#print('a',a)
	print('u1',u1 )
	print('u2',u2 )
	print('u3',u3 )
	print('u4',u4 )
	print('yaw',yaw)
	print('cos',math.cos(yaw*180/3.14))
	##print ##sol['x'][0,0]
	##print ('updl',Updlin )
	
	#vx=vxold+ux*dt
	#vy=vyold+uy*dtTheta_er
	#v=math.sqrt(math.pow((vx), 2) + math.pow((vy), 2))
	#if (vy) < 0 and (vx) <  0:
	#	v=-v
	# 判断坐标象限
        """if (vy) == 0 and (vx) > 0:
            yaw_v = 0
        if (vy) > 0 and (vx) > 0:
            yaw_v = math.atan((vy) / (vx))
        if (vy) > 0 and (vx) == 0:
            yaw_v = 0.5 * math.pi
        if (vy) > 0 and (vx) < 0:
            yaw_v = math.atan((vy) / (vx)) + math.pi
        if (vy) == 0 and (vx) < 0:
            yaw_v = math.pi
        if (vy) < 0 and (vx) < 0:
            yaw_v = math.atan((vy) / (vx)) - math.pi
        if (vy) < 0 and (vx) == 0:
            yaw_v = -0.5 * math.pi
        if (vy) < 0 and (vx) > 0:
            yaw_v = math.atan((vy) / (vx)) ##+ 2 * math.pi

	# 判断坐标象限
        if (uy) == 0 and (ux) > 0:
            yaw_w = 0
        if (uy) > 0 and (ux) > 0:
            yaw_w = math.atan((uy) / (ux))
        if (uy) > 0 and (ux) == 0:
            yaw_w = 0.5 * math.pi
        if (uy) > 0 and (ux) < 0:
            yaw_w = math.atan((uy) / (ux)) + math.pi
        if (uy) == 0 and (ux) < 0:
            yaw_w = math.pi
        if (uy) < 0 and (ux) < 0:
            yaw_w = math.atan((uy) / (ux)) - math.pi
        if (uy) < 0 and (ux) == 0:
            yaw_w = -0.5 * math.pi
        if (uy) < 0 and (ux) > 0:
            yaw_w = math.atan((uy) / (ux)) ##+ 2 * math.pi"""




	##dh =(2*(x-a)*math.cos(yaw)+2*(y-b)*math.sin(yaw))*Ulin
        # 目前先只使用最简单的比例控制
        ##liner_speed = 0.1 * (Updlin*dt+liner_speed_old)
        ##angular_speed = 0.9 * (Updang)
 	
	##Uang=0.5*Uang

	#if Ulin >4.1:
      	#	Ulin = 4.1	
	#if Ulin <-4.1:
      	#	Ulin = -4.1

	##Uang = Uang *0.28


	#if Uang >0.785:
      	#	Uang = 0.785	
	#if Uang <-0.785:
      	#	Uang= -0.785

#jiaojiasudu   (vx*uy-vy*ux)*dt/(vx*vx+vy*vy)
	u1_old=u1
	u2_old=u2
	u3_old=u3
	u4_old=u4
	vx = r/4*(c1*u1+c2*u2+c2*u3+c1*u4)	#-4*u1
 	vy = r/4*(-c2*u1+c1*u2+c1*u3-c2*u4)
	wz = (-u1+u2-u3+u4)
	#angular_speed = (yaw_v-yaw)#*0.5#/dt ##*0.5##*dt*50##*100##*19000##*0.1*dt##*100
	
	#if liner_speed >0.1:
      	#	liner_speed = 0.1	
	#if liner_speed <-0.1:
      	#	liner_speed = -0.1

        msg.linear.x = vx#*math.cos(Theta_err)
	msg.linear.y = vy#*math.sin(Theta_err)
        msg.angular.z = wz

	
    
	
	##print msg.linear.x
	##print msg.angular.z
	
	#print('uang' ,Uang)
	##print ('upda',Updang )
	
	print('vx',msg.linear.x)
	print('vy',msg.linear.y)
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



    rospy.spin()
