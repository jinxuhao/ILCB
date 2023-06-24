#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import rospy
import math
import nav_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
	
from cvxopt import solvers,matrix
import numpy as np
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import cv2

import matplotlib.pyplot as plt
import random
import numpy as np
import math
#from sklearn import dataset

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
vx_old = 0.00
vy_old = 0.00
w_old=0

angular_speed_old = 0
nominalController = 0
yaw_old=0
yaw_wold=0
yaw_v=0
yaw_vold=0
dtheta=0
import numpy as np

X_t = 10
Y_t = 10
X_sim = 10       # 目标点x坐标
Y_sim = 10     # 目标点y坐标

r=0.05
vx=0

vy=0

##a=5
##b=5
##r=1
##shezhiyuan
a1= 3.0#1.75
b1= 3.25#1.0
r1=0.7

c=6.0
kp=0.2
kd=0.9


a2=6.5
b2=5.0
r2=0.7

a3=4.3#3.25##-0.5
b3=3.8
r3=0.7

a4=4.8
b4=2.0
r4=0.7

a5=6.0
b5=3.8
r5=0.7

u1_old=0
u2_old=0
u3_old=0
u4_old=0

alist=[]
rlist=[]
ylist=[]
xlist=[]

##thd = yaw_t##-math.pi/4 
vd  = 0
Ulin= 0

def callback(msg1):
    global rlist
    global alist
    rlist=[]
    alist=[]	
    #print(len(msg.ranges)) len is 2019 from 0-360
    current_time = rospy.Time.now()
    '''scann.header.stamp = current_time'''
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


def dist(t1, t2):
    dis = math.sqrt((np.power((t1[0]-t2[0]),2) + np.power((t1[1]-t2[1]),2)))
    # print("两点之间的距离为："+str(dis))
    return dis


def dbscan(Data, Eps, MinPts):
    num = len(Data)  # 点的个数
    # print("点的个数："+str(num))
    unvisited = [i for i in range(num)]  # 没有访问到的点的列表
    # print(unvisited)
    visited = []  # 已经访问的点的列表
    C = [-1 for i in range(num)]
    # C为输出结果，默认是一个长度为num的值全为-1的列表
    # 用k来标记不同的簇，k = -1表示噪声点
    k = -1
    # 如果还有没访问的点
    while len(unvisited) > 0:
        # 随机选择一个unvisited对象
        p = random.choice(unvisited)
        unvisited.remove(p)
        visited.append(p)
        # N为p的epsilon邻域中的对象的集合
        N = []
        for i in range(num):
            if (dist(Data[i], Data[p]) <= Eps):# and (i!=p):
                N.append(i)
        # 如果p的epsilon邻域中的对象数大于指定阈值，说明p是一个核心对象
        if len(N) >= MinPts:
            k = k+1
            # print(k)
            C[p] = k
            # 对于p的epsilon邻域中的每个对象pi
            for pi in N:
                if pi in unvisited:                  
		    unvisited.remove(pi)
                    visited.append(pi)
                    # 找到pi的邻域中的核心对象，将这些对象放入N中
                    # M是位于pi的邻域中的点的列表
                    M = []
                    for j in range(num):
                        if (dist(Data[j], Data[pi])<=Eps): #and (j!=pi):
                            M.append(j)
                    if len(M)>=MinPts:
                        for t in M:
                            if t not in N:
                                N.append(t)
                # 若pi不属于任何簇，C[pi] == -1说明C中第pi个值没有改动
                if C[pi] == -1:
                    C[pi] = k
        # 如果p的epsilon邻域中的对象数小于指定阈值，说明p是一个噪声点
        else:
            C[p] = -1
 
    return C



if __name__ == '__main__':
    rospy.init_node('item1')
    pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 1)
    turtle_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    rate = rospy.Rate(5.0)

    while not rospy.is_shutdown():
        msg = geometry_msgs.msg.Twist()
        (roll, pitch, yaw) = euler_from_quaternion([x_o,y_o,z_o,w_o])
	print('cos1',math.cos(1))
	print('li',x)

  
	msg1 = LaserScan()
	global xlist
	global ylist
	xlist=[]
	ylist=[]
	
	for i in range(len(rlist)):
	        if rlist[i] == 0:
			p_x=0
			p_y=0
		else:
			p_x = x+(rlist[i])*math.cos((alist[i]+yaw))
			p_y = y+(rlist[i])*math.sin((alist[i]+yaw))
		ylist.append(p_y)
		xlist.append(p_x)
		
	dataSet = [list(t) for t in zip(xlist,ylist)]                  
	    
	print(dataSet)                # 输出列表
	
	C = dbscan(dataSet, 0.25, 1)
	print(C)
	
	X_t = X_sim
    	Y_t = Y_sim
	 ##math.pow((X_t - x), 2) + math.pow((Y_t - y), 2)

	##dv = p1*(x-X_t)*v*math.cos(yaw*180/math.pi) + p2*(y-Y_t)*v*math.sin(yaw*180/math.pi)+p3*(yaw - thd)*w+p4*(v - vd)*Ulin

	v1d = kp*(X_t - x) #+kd*(0-vx_old)
	v2d = kp*(Y_t - y) #+kd*(0-vy_old)


 	
	## Upd
	##nominalController 
	
	##Updlin= kp*(D_err)+kd*liner_speed_old
##Ulin= kp*(D_err)-kd*liner_speed_old



	##g = (2*(x-a)*math.cos(yaw*180/3.14)+2*(y-b)*math.sin(yaw*180/3.14))
	


	#if  y < 2 and   x <2 :

	
      	#h11 = 2.*(x-a1)*vx+2.*(y-b1)*vy
	h12 =(math.pow((x-a1), 2.) + math.pow((y-b1), 2.) - math.pow((r1), 2.))

	#h21 = 2.*(x-a2)*vx+2.*(y-b2)*vy
	h22 =(math.pow((x-a2), 2.) + math.pow((y-b2), 2.) - math.pow((r2), 2.))

	h3 =(math.pow((x-a3), 2.) + math.pow((y-b3), 2.) - math.pow((r3), 2.))
	h4 =(math.pow((x-a4), 2.) + math.pow((y-b4), 2.) - math.pow((r4), 2.))
	h5 =(math.pow((x-a5), 2.) + math.pow((y-b5), 2.) - math.pow((r5), 2.))
   	#dh =(2*(x-a)*vx+2*(y-b)*vy)	

	##th1=yaw-thd
	#v1=v-vd
	

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



	P = 2*matrix([[1.0,0.0],[0.0,1.0]])   # matrix里区分int和double，所以数字后面都需要加小数点
	q = matrix([-2.*v1d, -2.*v2d])
	G = matrix([[-2.*(x-a1),-2.*(x-a2),-2.*(x-a3),-2.*(x-a4),-2.*(x-a5)],[-2.*(y-b1),-2.*(y-b2),-2.*(y-b3),-2.*(y-b4),-2.*(y-b5)]])
#,1.0,0.0,-1.0,0.0
#,0.0,1.0,0.0,-1.0
	h = matrix([c*h12,c*h22,c*h3,c*h4,c*h5])#,0.3,0.3,0.3,0.3

	sol = solvers.qp(P,q,G,h)
	
	v1 = sol['x'][0,0]##Updlin
	v2 = sol['x'][1,0]
	
##
	##if Uang >   2 * math.pi:
	##	Uang = Uang-2 * math.pi
	##if Uang <   -2 * math.pi:
	##	Uang = UanTheta_erg+2 * math.pi
		
	
	print('x',x)
	print('y',y)
	#print('h1',h1)
	#print('h2',h2)
	#print('a',a)
	print('v1',v1 )
	print('v2',v2 )
	
	print('cos',math.cos(yaw*180/3.14))
	##print ##sol['x'][0,0]
	##print ('updl',Updlin )
	
	#vx=vxold+ux*dt
	#vy=vyold+uy*dtTheta_er
	#v=math.sqrt(math.pow((vx), 2) + math.pow((vy), 2))
	#if (vy) < 0 and (vx) <  0:
	#	v=-v
	# 判断坐标象限
       
	"""# 判断坐标象限
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


	 


	vy=v2
	vx=v1

	if (vy) == 0 and (vx) > 0:
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

	print('yaw_v',yaw_v)
	
	msg.angular.z =yaw_v-yaw

	vx_old =vx
	vy_old =vy
	#angular_speed = (yaw_v-yaw)#*0.5#/dt ##*0.5##*dt*50##*100##*19000##*0.1*dt##*100
	
	#if liner_speed >0.1:
      	#	liner_speed = 0.1	
	#if liner_speed <-0.1:
      	#	liner_speed = -0.1

	vx = math.cos(yaw)*vx+math.sin(yaw)*vy
	vy = math.cos(yaw)*vy-math.sin(yaw)*vx


        msg.linear.x = vx*0.05#*math.cos(Theta_err)
	msg.linear.y = vy*0.05#*math.sin(Theta_err)
        
	
    
	
	##print msg.linear.x
	##print msg.angular.z
	
	#print('uang' ,Uang)
	##print ('upda',Updang )
	#print('vx_old',vx_old)#
	#print('vy_old',vy_old)#*
	print('msg.linear.x',msg.linear.x)
	print('msg.linear.y',msg.linear.y)
	print('msg.angular.z',msg.angular.z)
	
	print('yaw',yaw)
	print('yaw_t',yaw_t)
	
	
	##P = matrix([[2.0,0.0],[0.0,2.0]])   # matrix里区分int和double，所以数字后面都需要加小数点
	##q = matrix([-2*Upblin,-2*Upbang])
	##G = matrix([[- g],[0.0]])
	##h = matrix([0.0])

	##sol = solvers.qp(P,q,G,h)   # 调用优化函数solvers.qp求解
	##print sol['x']  # 打印结果，sol里面还有很多其他属性，读者可以自行了解


        turtle_vel.publish(msg)    # 发布运动指令
        rospy.Subscriber('/odom', nav_msgs.msg.Odometry,  Trans_robot_pose) # 获取位姿信息
	pub.publish(msg1)    # pub.publish(scann)
        rospy.Subscriber('/scan', LaserScan, callback)
        rate.sleep()



    rospy.spin()
