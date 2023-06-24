#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
 
from pyexpat.errors import XML_ERROR_DUPLICATE_ATTRIBUTE
import re
import rospy
import math
import nav_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from std_msgs.msg import Header
	
from cvxopt import solvers,matrix
import numpy as np
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

import matplotlib.pyplot as plt
import random

from least_square import fit_circle
from fit_polynom import fit_polynom

#import pandas as pd

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

X_t = (6.7-2.5)*5
Y_t = (-1.1-4.6)*5
X_sim= (6.7-2.5   )*5   # 目标点x坐标
Y_sim = (-1.1-4.6)*5

r=0.05
vx=0

vy=0
n02=0.0
n01=0.0
n0=0.0
n0y=0.0
n12=0.0
n11=0.0
n1=0.0
n1y=0.0

n22=0.0
n21=0.0
n2=0.0
n2y=0.0

n32=0.0
n31=0.0
n3=0.0
n3y=0.0

n42=0.0
n41=0.0
n4=0.0
n4y=0.0


##a=5
##b=5
##r=1
##shezhiyuan
a1= 3.0#1.75
b1= 3.25#1.0
r1=0.7

C_shinengcanshu = 0.25
para_1= 25.0
para_2= 12.50
c=1.8#2.8#6.0
kp=0.14#0.2
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

vd  = 0
Ulin= 0

x_center_0=0.0
y_center_0=0.0
R_0_list=0.0
p_0_flag =0.0

p_0_2=0.0
p_0_1=0.0
p_0_0=0.0

q_0_2=0.0
q_0_1=0.0
q_0_0=0.0


x_center_1=0.0
y_center_1=0.0
R_1_list=0.0
p_1_flag =0.0

p_1_2=0.0
p_1_1=0.0
p_1_0=0.0

q_1_2=0.0
q_1_1=0.0
q_1_0=0.0

x_center_2=0.0
y_center_2=0.0
R_2_list=0.0
p_2_flag =0.0

p_2_2=0.0
p_2_1=0.0
p_2_0=0.0

q_2_2=0.0
q_2_1=0.0
q_2_0=0.0


def callback(msg1):
	global rlist
	global alist
	rlist=[]
	alist=[]	
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
	global C
	C = []
	num = len(Data)  # 点的个数
    # print("点的个数："+str(num))
	unvisited = [i for i in range(num)]  # 没有访问到的点的列表
    # print(unvisited)
	visited = []  # 已经访问的点的列表
	DK = [index for index in range(num)]
	for index in range(num):
		DK[index]=[]
	C = [-1 for i in range(num)]
    # C为输出结果，默
    # 用k来标记不同的簇，k = -1表示噪声点np.sort(X,axis=0)
	k = -1
    # 如果还有没访问的点 print('neibu',num)
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
			DK[k].append(Data[p])
            # 对于_p的epsilon邻域中的每个对象pi
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
					DK[k].append(Data[pi])
        # 如果p的epsilon邻域中的对象数小于指定阈值，说明p是一个噪声点
		else:
			C[p] = -1
	# print ('DK',DK)
	return C ,DK


def set_data_radar(x,y,rlist_def,alist_def,yaw_def):
	global xlist
	global ylist
	xlist=[]
	ylist=[]

	for i in range(len(rlist)):
		if rlist[i] == 0:
			p_x=0
			p_y=0
		else:
			p_x = x+(rlist_def[i])*math.cos((alist_def[i]+yaw_def))
			p_y = y+(rlist_def[i])*math.sin((alist_def[i]+yaw_def))
		ylist.append(p_y)
		xlist.append(p_x)


	return xlist ,ylist

def set_circle_data(DK_x,DK_y):
	global x_center_list
	global y_center_list
	global R_list
	x_center_list=[]
	y_center_list=[]
	R_list=[]
	if (len(DK_x) != 0 )and(len(DK_y) != 0 ):
		
		x_center_ ,y_center_,r_= fit_circle(DK_x,DK_y)
		print('yuan',x_center_ ,y_center_,r_)
		x_center_list.append(x_center_)
		y_center_list.append(y_center_)
		R_list.append(r_)

	return x_center_list ,y_center_list ,R_list

if __name__ == '__main__':
	
	rospy.init_node('item1')
	pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 1)
	turtle_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
	rate = rospy.Rate(10.0)
	time_list = []
	x_center_0_list_final=[]
	y_center_0_list_final=[]
	t_0_list_final=[]

	x_center_1_list_final=[]
	y_center_1_list_final=[]
	t_1_list_final=[]

	x_center_2_list_final=[]
	y_center_2_list_final=[]
	t_2_list_final=[]
	
	while not rospy.is_shutdown():
		time_now = rospy.get_time()
		# print('time',time_now)
		time_list.append(time_now) 

		msg = geometry_msgs.msg.Twist()
		(roll, pitch, yaw) = euler_from_quaternion([x_o,y_o,z_o,w_o])
		msg1 = LaserScan()

		try:
			(xlist,ylist) =  set_data_radar(x,y,rlist,alist,yaw)
			# print('xxxxxxx',x)
		except:
			continue
		# print('xlist',xlist)
		dataSet = [list(t) for t in zip(xlist,ylist)]                  
	    
		# print('dataset',dataSet)                # 输出列表
	
		Cr,DK_data = dbscan(dataSet, 0.2,20)# 0.1 20x_center_list_fina
		k=-1
		for i in range(len(rlist)):
			try:
				if k<Cr[i]:k=Cr[i]
			except:
				continue
		DK0x=[]
		DK0y=[]	
		DK1x=[]
		DK1y=[]
		DK2y=[]
		DK2x=[]
		DK3x=[]
		DK3y=[]
		DK4x=[]
		DK4y=[]
		# print(DK_data)
		for i in range(k+1):
			
			for j in range(len(rlist)):
				
				try:
					if i ==0:
						DK0x.append(DK_data[i][j][0])
						DK0y.append(DK_data[i][j][1])
					if i ==1:
						DK1x.append(DK_data[i][j][0])
						DK1y.append(DK_data[i][j][1])
					if i ==2:
						DK2x.append(DK_data[i][j][0])
						DK2y.append(DK_data[i][j][1])
						
					if i ==3:
						DK3x.append(DK_data[i][j][0])
						DK3y.append(DK_data[i][j][1])
					if i ==4:
						DK4x.append(DK_data[i][j][0])
						DK4y.append(DK_data[i][j][1])

				except:
					continue
		

		# print('DK0x',DK0x)
		
		#n02, n01 ,n0, n0y =fit_polynom(DK0x,DK0y)
		
		#n12, n11 ,n1, n1y =fit_polynom(DK1x,DK1y)

		#n22, n21 ,n2, n2y =fit_polynom(DK2x,DK2y)

		#n32, n31 ,n3, n3y =fit_polynom(DK3x,DK3y)
		
		#n42, n41 ,n4, n4y =fit_polynom(DK4x,DK4y)

		'''
		pf2 = PolynomialFeatures(degree = 2,include_bias = False)
		DK_2x =np.array(DK2x)
		DK_2y =np.array(DK2y)
		if DK_2x.size != 0:

			X_new2 = pf2.fit_transform(DK_2x.reshape(-1, 1))
			reg1 = LinearRegression()
			reg1.fit(X_new2,DK_2y.reshape(-1, 1))
			print('reg1.inter',reg2.intercept_)
			print('reg1.coef',reg2.coef_)
			n22 = reg2.coef_[0][1]
			n21 = reg2.coef_[0][0]
			n2  = reg2.intercept_[0]
			n2y=0
			if not (np.array_equal(reg2.coef_,array) and (n2==0.0)) :
				n2y = 1
		'''
	
		"""
		
			if np.linalg.det(eTe) == 0.0:
				print('xTx不可逆')
			else:
				q = np.ravel(eTe.I * (Z_train.T * y_train))
			coef_ = q[:-1]
			intercept_ = q[-1]
				
			print('Coefficients: ', coef_)
			print('Intercept:', intercept_)
			print('the model is: y = ', coef_[0], '* X + ', coef_[1], '* X^2 + ', intercept_)
		except:
			continue
		
		"""

		p_0_flag =0.0
		if len(DK0x) !=0:
			x_center_0_list,y_center_0_list,R_0_list = set_circle_data(DK0x,DK0y)
			x_center_0 = x_center_0_list[0]
			y_center_0 = y_center_0_list[0]
			R_0_list = R_0_list[0]

			print('x-center',x_center_0_list[0])
			if len(x_center_0_list_final) >= 3:
				if x_center_0_list[0] !=0.0:
					x_center_0_list_final.append(x_center_0_list[0])
					x_center_0_list_final.remove(x_center_0_list_final[0])
					y_center_0_list_final.append(y_center_0_list[0])
					y_center_0_list_final.remove(y_center_0_list_final[0])
					t_0_list_final.append(time_now)
					t_0_list_final.remove(t_0_list_final[0])
					#p_0_2,
					p_0_1,p_0_0,p_0_flag =fit_polynom(t_0_list_final,x_center_0_list_final)
					#q_0_2,
					q_0_1,q_0_0,q_0_flag =fit_polynom(t_0_list_final,y_center_0_list_final)
					# print('p_0_2,p_0_1,p_0_0,p_0_flag',p_0_1,p_0_0,p_0_flag)#,p_0_2
			else:
				if x_center_0_list[0] !=0.0:
					x_center_0_list_final.append(x_center_0_list[0])
					y_center_0_list_final.append(y_center_0_list[0])
					t_0_list_final.append(time_now)
			
			
			print('x_dian',p_0_1*time_now+p_0_0)#p_0_2*time_now*time_now+
			
			print('p_0_1',p_0_1)
			print('p_0_0',p_0_0)
			print('time',time_now)

			# print('t_final',t_0_list_final)
		

		p_1_flag = 0.0
		if len(DK1x) !=0:
			x_center_1_list,y_center_1_list,R_1_list = set_circle_data(DK1x,DK1y)
			x_center_1 = x_center_1_list[0]
			y_center_1 = y_center_1_list[0]
			R_1_list = R_1_list[0]

			print('x-center',x_center_1_list[0])
			if len(x_center_1_list_final) >= 3:
				if x_center_1_list[0] !=0.0:
					x_center_1_list_final.append(x_center_1_list[0])
					x_center_1_list_final.remove(x_center_1_list_final[0])
					y_center_1_list_final.append(y_center_1_list[0])
					y_center_1_list_final.remove(y_center_1_list_final[0])
					t_1_list_final.append(time_now)
					t_1_list_final.remove(t_1_list_final[0])
					# p_0_2,
					p_1_1,p_1_0,p_1_flag =fit_polynom(t_1_list_final,x_center_1_list_final)
					# q_0_2,
					q_1_1,q_1_0,q_1_flag =fit_polynom(t_1_list_final,y_center_1_list_final)
					# print('p_0_2,p_0_1,p_0_0,p_0_flag',p_0_1,p_0_0,p_0_flag)#,p_0_2
			else:
				if x_center_1_list[0] !=0.0:
					x_center_1_list_final.append(x_center_1_list[0])
					y_center_1_list_final.append(y_center_1_list[0])
					t_1_list_final.append(time_now)


		p_2_flag =0.0
		if len(DK2x) !=0:
			x_center_2_list,y_center_2_list,R_2_list = set_circle_data(DK2x,DK2y)
			x_center_2 = x_center_2_list[0]
			y_center_2 = y_center_2_list[0]
			R_2_list = R_2_list[0]

		
			if len(x_center_2_list_final) >= 3:
				if x_center_2_list[0] !=0.0:
					x_center_2_list_final.append(x_center_2_list[0])
					x_center_2_list_final.remove(x_center_2_list_final[0])
					y_center_2_list_final.append(y_center_2_list[0])
					y_center_2_list_final.remove(y_center_2_list_final[0])
					t_2_list_final.append(time_now)
					t_2_list_final.remove(t_2_list_final[0])
					# p_0_2,
					p_2_1,p_2_0,p_2_flag =fit_polynom(t_2_list_final,x_center_2_list_final)
					# q_0_2,
					q_2_1,q_2_0,q_2_flag =fit_polynom(t_2_list_final,y_center_2_list_final)
					# print('p_0_2,p_0_1,p_0_0,p_0_flag',p_0_1,p_0_0,p_0_flag)#,p_0_2
			else:
				if x_center_2_list[0] !=0.0:
					x_center_2_list_final.append(x_center_2_list[0])
					y_center_2_list_final.append(y_center_2_list[0])
					t_2_list_final.append(time_now)


		# print('dk1x',DK1x)
		
		# print('Cr',Cr)
		X_t = X_sim
		Y_t = Y_sim
		##math.pow((X_t - x), 2) + math.pow((Y_t - y), 2)

		##dv = p1*(x-X_t)*v*math.cos(yaw*180/math.pi) + p2*(y-Y_t)*v*math.sin(yaw*180/math.pi)+p3*(yaw - thd)*w+p4*(v - vd)*Ulin

		v1d = kp*(X_t - x) #+kd*(0-vx_old)
		v2d = kp*(Y_t - y) #+kd*(0-vy_old)


		h1 = (math.pow((x-x_center_0), 2.) + math.pow((y-y_center_0), 2.) - math.pow((R_0_list), 2.))
		h2 = (math.pow((x-x_center_1), 2.) + math.pow((y-y_center_1), 2.) - math.pow((R_1_list), 2.))
		h3 = (math.pow((x-x_center_2), 2.) + math.pow((y-y_center_2), 2.) - math.pow((R_2_list), 2.))
		

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


		'''
		
		P = 2*matrix([[1.0,0.0],[0.0,1.0]])   # matrix里区分int和double，所以数字后面都需要加小数点
		q = matrix([-2.*v1d, -2.*v2d])
		G = matrix([[-2.*(x-x_center_0)*p_0_flag],[-2.*(y-y_center_0)*p_0_flag]])
		#,1.0,0.0,-1.0,0.0,-2.*n22*x-1.*n21,-2.*n32*x-1.*n31,-2.*n42*x-1.*n41
		#,0.0,1.0,0.0,-1.0,+1.*n2y,+1.*n3y,+1.*n4y
		h = matrix([(c*h1-2.*(x-x_center_0)*(p_0_0)-2.*(y-y_center_0)*(q_0_0))*p_0_flag])
		print('GX',[-2.*(x-x_center_0)*p_0_flag])
		print('hhhhhhh',(c*h1-2.*(x-x_center_0)*(p_0_0)-2.*(y-y_center_0)*(q_0_0))*p_0_flag)
		# matrix([(c*h1-2.*(x-x_center_0)*(p_0_1*time_now+p_0_0)-2.*(y-y_center_0)*(q_0_1*time_now+q_0_0))*p_0_flag])#,0.3,0.3,0.3,0.3,c*h3*n2y,c*h4*n3y,c*h5*n4y
        # -2.*(x-x_center_0)*(p_0_1*time_now+p_0_0)-2.*(y-y_center_0)*(q_0_1*time_now+q_0_0)
		sol = solvers.qp(P,q,G,h)
		'''
		x_d = X_t
		y_d = Y_t
		V = (x-x_d)**2*para_1+(y-y_d)**2*para_1+2*para_2*(x-x_d)*(y-y_d)

		P = 2*matrix([[1.0,0.0],[0.0,1.0]])   # matrix里区分int和double，所以数字后面都需要加小数点
		q = matrix([0.0, 0.0])
		G = matrix([[2*para_1*(x-x_d)+2*para_2*(y-y_d),-2.*(x-x_center_0)*p_0_flag,-2.*(x-x_center_1)*p_1_flag,-2.*(x-x_center_2)*p_2_flag],[2*para_1*(y-y_d)+2*para_2*(x-x_d),-2.*(y-y_center_0)*p_0_flag,-2.*(y-y_center_1)*p_1_flag,-2.*(y-y_center_2)*p_2_flag]])
		#,1.0,0.0,-1.0,0.0,-2.*n22*x-1.*n21,-2.*n32*x-1.*n31,-2.*n42*x-1.*n41
		#,0.0,1.0,0.0,-1.0,+1.*n2y,+1.*n3y,+1.*n4y
		h = matrix([-C_shinengcanshu*V,(c*h1)*p_0_flag,(c*h2)*p_1_flag,(c*h3)*p_2_flag])
		print('p_0_flag',p_0_flag)
		print('p_1_flag',p_1_flag)
		print('p_2_flag',p_2_flag)
		sol = solvers.qp(P,q,G,h)


		v1 = sol['x'][0,0]##Updlin
		v2 = sol['x'][1,0]
		
		vy=v2#*0.1
		vx=v1#*0.1

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

		#print('yaw_v',yaw_v)	
		
		msg.angular.z = (yaw_v-yaw)#*0.5

		vx_old =vx
		vy_old =vy

		vx = math.cos(yaw)*vx_old+math.sin(yaw)*vy_old
		vy = math.cos(yaw)*vy_old-math.sin(yaw)*vx_old	


		msg.linear.x = vx*0.05#0.4#0.23#0.03#0.05*math.cos(Theta_err)
		msg.linear.y = vy*0.05#0.4#0.23#0.03#0.05*math.sin(Theta_err)
			

		print('msg.linear.x',msg.linear.x)
		print('msg.linear.y',msg.linear.y)
		#print('msg.angular.z',msg.angular.z)
		
		#print('yaw',yaw)
		#print('yaw_t',yaw_t)
		


		turtle_vel.publish(msg)    # 发布运动指令
		rospy.Subscriber('/odom', nav_msgs.msg.Odometry,  Trans_robot_pose) # 获取位姿信息
		pub.publish(msg1)    # pub.publish(scann)
		rospy.Subscriber('/scan', LaserScan, callback)
		rate.sleep()
	rospy.spin()
