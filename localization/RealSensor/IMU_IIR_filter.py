# -*- coding: utf-8 -*-
"""
Created on Sat Jan 15 11:55:32 2022

@author: Dell
"""
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

#Filter coefficients (poles at 3Hz and 60Hz)
a = 900
b = 1
c = 190
d = 900

#Array to store data
not_filtered_arr = []
filtered_arr = []
time = []

#IIR filter
class IIR_IMU():
    
    def __init__(self, a, b, c):
        self.y = [0, 0, 0] # [y[n], y[n-1], y[n-2]]
        self.x = [0, 0, 0] # [x[n], x[n-1], x[n-2]]
        self.coeff = [a, b, c, d] #filter coefficients 
        
        
    def filter_IMU(self, x):
        
        # H[n] = a/(bs^2 + cs + d)
        # y[n] = 1/b * ( -cy[n-1] -dy[n-2] + ax[n-2])
        self.y[0] = 1/self.coeff[1] * (-self.coeff[2]*self.y[1] -self.coeff[3]*self.y[2] 
                                       /+self.coeff[0]*self.x[2] )
        #push everything by 1 
        self.y = [self.y[0], self.y[0], self.y[1]]
        self.x = [x, self.x[0], self.x[1]]
        return self.y[0]
    
    def create_data(self, i):
        #really bad dataset, could be improved
        return ( 3*math.sin(200*i) + 2*math.sin(100*i) +9* math.sin((20*i)))


#loop 
IIR_instance = IIR_IMU(a, b, c)

i = 0
while (i < 100 ):    
    not_filtered = IIR_instance.create_data(i) #create data point
    #print(not_filtered)
    filtered = IIR_instance.filter_IMU(not_filtered) #filter it
    print(filtered)
    #store filtered and not filtered in array 
    not_filtered_arr.append(not_filtered)
    filtered_arr.append(filtered)
    time.append(i)
    i += 1 

def PlotKF(time, position, ylabel, col): #Plots KF
    plt.scatter(time, position, color=col, linestyle='--', linewidths=0.1, 
                marker='o', label=ylabel)
    plt.xlabel('time')
    plt.ylabel(ylabel)
    plt.yscale("linear")
    plt.legend(loc='upper right')  
    
    
#Plot results 
print("done calculating")
PlotKF(time, filtered_arr , "filtered", "green")
PlotKF(time, not_filtered_arr, "not filtered", "orange")
plt.show()
