#! /usr/bin/env python
# -*- coding: utf-8 -*-



from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures


import numpy as np
import matplotlib.pyplot as plt
import random

def fit_polynom(dataset_1,dataset_2):

    array = np.array([[0., 0.]])
    pf = PolynomialFeatures(degree = 1,include_bias = False)
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
        
        X_new0 = pf.fit_transform(data_set_2.reshape(-1, 1))
        reg0 = LinearRegression()
        reg0.fit(X_new0,data_set_2.reshape(-1, 1))
        print('reg.inter',reg0.intercept_[0])
        print('reg.coef',reg0.coef_)
        #print('rif __name__ == '__main__':eg.coef1111111',reg0.coef_[0])
        #priif DKt('reg.coef22222',reg0.coef_[0][1])
        plt.scatter(data_set_1,data_set_2,c='green', alpha=0.6)
        plt.plot(data_set_1, reg0.predict(X_new0), color='r')
        plt.show()
        #n_2 = reg0.coef_[0][0]
        n_1 = reg0.coef_[0][0]
        n_0 = reg0.intercept_[0]
        n_flag =0
        if not (np.array_equal(reg0.coef_,array) and (n_0==0.0)) :
            n_flag = 1

    return  n_1, n_0, n_flag
#n_2,
if __name__ == '__main__':
    y =  [5,15.0,25.0,35,45]
    x = [0.0,0.5,1.0,1.5,2.0]
    b,c,d = fit_polynom(x,y)
    print('jin',b,c,d)