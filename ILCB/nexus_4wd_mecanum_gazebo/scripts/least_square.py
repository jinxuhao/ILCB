#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
This program is debugged by Harden Qiu
"""


from statistics import mean

from numpy import sqrt
from scipy import optimize
from scipy.optimize import leastsq
import functools
#from matplotlib import pyplot as p, cm, colors

method_2  = "Fitting Circle"

# Coordinates of the 2D points
#x = r_[3.688086  , 3.89499714, 3.82375639, 3.7544322 , 4.23283412,4.08311165, 3.98422161]
#y = r_[3.74430945, 3.50502529, 3.56644291, 3.64036308, 3.43359167,3.43464117, 3.46850284]
basename = 'arc'


#修饰器：用于输出反馈
def countcalls(fn):
    "decorator function count function calls "

    @functools.wraps(fn)
    def wrapped(*args):
        wrapped.ncalls +=1
        return fn(*args)

    wrapped.ncalls = 0
    return wrapped

def calc_R(xc, yc):
    global x,y
    return sqrt((x - xc) ** 2 + (y - yc) ** 2)

@countcalls
def f_2(c):
    Ri = calc_R(*c)
    return Ri - Ri.mean()

def fit_circle(x,y):
    global x_1
    global y_1
    # 质心坐标
    x_1 =x
    y_1 = y
    x_m = mean(x)
    y_m = mean(y)

    #圆心估计
    center_estimate = x_m, y_m
    center_2, _ = optimize.leastsq(f_2, center_estimate)
    xc_2, yc_2 = center_2
    print('li',xc_2)
    Ri_2       = calc_R(xc_2, yc_2)
    #拟合圆的半径
    R_2        = Ri_2.mean()
    residu_2   = sum((Ri_2 - R_2)**2)
    residu2_2  = sum((Ri_2**2-R_2**2)**2)
    ncalls_2   = f_2.ncalls

    #输出列表
    # fmt = '%-22s %10.5f %10.5f %10.5f %10d %10.6f %10.6f %10.2f'
    # print (('\n%-22s' +' %10s'*7) % tuple('方法 Xc Yc Rc nb_calls std(Ri) residu residu2'.split()))
    # print('-'*(22 +7*(10+1)))
    # print(fmt % (method_2 , xc_2 , yc_2 , R_2 , ncalls_2 , Ri_2.std() , residu_2 , residu2_2 ))
    return  xc_2 , yc_2 , R_2

def calc_R(xc, yc):

    return sqrt((x_1 - xc) ** 2 + (y_1 - yc) ** 2)

#xc_3 , yc_3 , R_3 = fit_circle(x,y)
#print('jin' ,(method_2 , xc_3 , yc_3 , R_3))
'''
#输出图
p.close('all')

def plot_all(residu2=False):

    p.figure(facecolor='white')  # figsize=(7, 5.4), dpi=72,
    p.axis('equal')

    theta_fit = linspace(-pi, pi, 180)

    x_fit2 = xc_2 + R_2 * cos(theta_fit)
    y_fit2 = yc_2 + R_2 * sin(theta_fit)
    p.plot(x_fit2, y_fit2, 'k--', label=method_2, lw=2)

    p.plot([xc_2], [yc_2], 'gD', mec='r', mew=1)

    # draw
    p.xlabel('x')
    p.ylabel('y')


    # 数据
    p.plot(x, y, 'ro', label='data', ms=8, mec='b', mew=1)
    p.legend(loc='best', labelspacing=0.1)

    #标题
    # p.grid()
    p.title('Least Squares Circle')

    p.savefig('%s_residu%d.png' % (basename, 2 if residu2 else 1))

# plot_all(residu2=False)
plot_all(residu2=True )

p.show()
'''