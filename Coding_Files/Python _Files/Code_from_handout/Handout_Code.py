#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 21 11:45:55 2021

@author: brahdamon
"""

# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import pandas as pd
import seaborn as sns
import sklearn
import statsmodels.api as sm
import matplotlib.pyplot as plt


plt.rcParams['figure.figsize'] = (12.0, 9.0)


# preprocessing input data

data = pd.read_csv("NN-test.csv") # Read in the csv file
print(data)
data.dropna(how = 'all', axis = 1, inplace = True) # Clean up the csv file
print(data)

print("Text after this is a new print")
X = data.iloc[:,0] # Store inputs in a variable as an array
Y = data.iloc[:,1] # Store outputs in a variable as an array

fig1 = plt.scatter(X,Y) # Create the scatter plot

X_mean = np.mean(X) # Calculate the mean of the inputs
Y_mean = np.mean(Y) # Calculate the mean of the outputs


"""
num = 0
den = 0
for i in range(len(X)):
    num += (X[i] - X_mean)*(Y[i] - Y_mean)
    den += (X[i] - X_mean)**2
m = num / den
c = Y_mean - m*X_mean

print (m, c)

"""




num = 0 # Initialize variable for numerator
den = 0 # Initialize denominator variable

# Following is the programming of the Least Square Error Statistic
# i.e. the template for how to code a sum

for i in range(len(X)):
    num += (X[i] - X_mean) * (Y[i] - Y_mean)
    print("Iteration ", i, ", numerator = ", num)
    den += (X[i] - X_mean) ** 2
    print("Iteration ", i, ", denominator = ", den)
    
m = num / den
m


# y = mx + b
b = Y_mean - m * X_mean
print(m, b)


#making predictions

Y_pred = m * X + b
plt.scatter(X,Y) #actual
plt.plot([min(X), max(X)], [min(Y_pred), max(Y_pred)], color = "red")
plt.show
