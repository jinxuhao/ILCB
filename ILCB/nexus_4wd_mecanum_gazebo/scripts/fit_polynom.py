#! /usr/bin/env python
# -*- coding: utf-8 -*-



import numpy as np
import matplotlib.pyplot as plt



def fit_polynom(dataset_1,dataset_2):
    array = np.array([[0., 0.]])
    data_set_1 =np.array(dataset_1)
    data_set_2 =np.array(dataset_2)
    global n_2
    n_2 = 0
    global n_1
    n_1 = 0
    global n_0
    n_0 = 0
    global n_flag
    n_flag = 0

    if data_set_1.size != 0:
        
        x = dataset_1
        num = dataset_2
        y = np.array(num)
        # print('y is :\n',y)

        #  f1 为各项的系数，3 表示想要拟合的最高次项是多少。
        f1 = np.polyfit(x, y,1)
        # p1 为拟合的多项式表达式
        p1 = np.poly1d(f1)
        # print('p1 is :\n',p1)

        plt.plot(x, y, 's',label='original values')
        yvals = p1(x) #拟合y值
        plt.plot(x, yvals, 'r',label='polyfit values')
        #plt.show()
        # n_2 = p1[2]
        n_1 = p1[1]
        n_0 = p1[0]
        n_flag =1
        #print('p1',p1[0])
        #if (np.array_equal(p1,array) ) :# and (n_0==0.0)
          #  n_flag = 0

    return  n_1, n_0, n_flag


if __name__ == '__main__':
    y =  [0.0,0.0,0.0,0.0]
    x = [0.0,1.0,2.0,3.0]
    b,c,d = fit_polynom(x,y)
    print('jin',b,c,d)
    