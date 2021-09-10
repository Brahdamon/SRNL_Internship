#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 27 16:06:22 2021

@author: brahdamon







"""

#Create two numpy arrays

import numpy as np


x_array = np.array([-1,0,1,2,3,4], dtype = int)
print(type(x_array))
# y_array = np.array([-3,-1,1,3,5,7], dtype = int)
y_array = 2 * x_array - -1
m_naught = np.random.randn()
b_naught = np.random.randn()
m = m_naught
b = b_naught
epochs = 200




learning_rate = 0.08


# Define Useful functions for this exercise
# Function to predict the relationship between x and y
def LinReg (x, m, b):
    p = m * x + b
    return p
    


# Create a function to represent loss function (cost function, in this case MSE)

def MeanSquareError(prediction, output):
    return np.square(prediction - output).mean()
# can use np.subtract() as another option

# OPTIONAL EXERCISE: Find the mean manually by iterating




# Optimization Algorithm
# Take partial derivatives of the error function (Mean square error)
# x and y are from given arrays
def GradientDescent (prediction, x, y):
    # store value of partial derivative of Loss Function L wrt slope(weight) m
    dldm = 2 * (prediction - y) * x
    # store value of partial derivative of Loss Function L wrt intercept (bias) b
    dldb = 2 * (prediction - y)
   
    
    return dldm, dldb



print("Values before adjustment: \nm = {} , b = {}".format(m, b))

for e in range(epochs):
    prediction = LinReg(x_array, m, b)
    #save outputs of GradientDescent function into variables
    mgrad, bgrad = GradientDescent(prediction, x_array, y_array)
    
    mgrad *= learning_rate
    bgrad *= learning_rate
    
    
    m -= mgrad.mean()
    b -= bgrad.mean()
    
    print("Iteration {} , m = {} , b = {}".format(e, m, b))
    
    













