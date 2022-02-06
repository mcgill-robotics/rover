# -*- coding: utf-8 -*-
"""
Created on Sat Jan 15 11:55:32 2022

@author: Dell
"""
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
import random 

#Filter coefficients 
a = 0.98

#Array to store data
not_filtered_arr = []
filtered_arr = []
time = []

#IIR filter
class IIR_IMU():
    
    def __init__(self, a):
        self.y = 0
        self.a = 0.9
        
        #https://fiiir.com/ 
    def filter_IMU(self, x):
        # y[n] = (1-a)*x[n] + a*y[n-1]
        self.y = (1-self.a)*x + self.a*self.y
        return self.y
    
    def create_data(self, i):
        #really bad dataset, could be improved
        return ( 3*math.sin(200*i) + 2*math.sin(100*i) + 10* math.sin(math.radians(20*i) ))


#loop 
IIR_instance = IIR_IMU(a)

i = 0
while (i < 400 ):    
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