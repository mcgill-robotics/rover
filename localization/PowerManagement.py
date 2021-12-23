# -*- coding: utf-8 -*-
"""
Created on Wed Dec 22 17:37:01 2021

@author: 
"""

""" READ ME, no seriously please do so

Here are my assumptions:
    -The current sensor output is filtered through a lowpass filter
    -The total current consumption can be calculated with a linear model 
    -dt is small enough for discretization errors to not matter much

How to use:
    1.create PowerManagement object with initial charge of battery
    2.call getTotalChargeDrawn and getPredictBatteryLife whenever
        you want 

"""
#imports 
import time

#constants or ROS dependant variables 
I_amp = 0.2 #current measured by sensor (A)

battery_1 = 22.0 * 3600.0 # Ahr/3600 = Q, charge of battery_1

#objects    
class PowerManagement():
    
    #initialization
    def __init__(self, total_charge):
        #variables related to getTotalChargeDrawn
        self.time_prev = time.time() #time at initialization (s)
        self.time_curr = 0 #current time (s)
        self.dt = 0 #time since the last measurement (s)
        self.charge_drawn = 0 #total charge drawn since init (Q)
        
        #variables related to getPredictedBatteryLife
        self.total_charge = total_charge #Charge in given battery (Q)
        self.charge_remain = 0 #electric charge remaining in battery (Q)
        self.seconds_remain = 0 #seconds remaining (s)
    
    #get the total charge drawn since initialization
    def getTotalChargeDrawn(self):
        self.time_curr = time.time() #update the current time
        self.dt = self.time_curr - self.time_prev #update dt 
        self.charge_drawn += I_amp * self.dt #update current drawn
        self.time_prev = time.time() #update the prev time
        
        #if more charge drawn than total capacity, raise error
        if (self.total_charge <= self.charge_drawn):
            raise ValueError('More charge drawn than total battery capacity')
        
        return self.charge_drawn
        
    #get the predicted remaining battery life at a current consumption I_amp
    def getPredictedBatteryLife(self):
        #Find how much electric charge remains
        self.charge_remain = self.total_charge - self.charge_drawn
        #Find nbr seconds remaining at this rate
        self.seconds_remain = self.charge_remain/I_amp 
        return self.seconds_remain
        
#Unit Testing 

M1 = PowerManagement(battery_1) #create a PowerManagement object
while(1):
    M1.getTotalChargeDrawn()
    print(M1.getPredictedBatteryLife())
    time.sleep(1) #pause for 1s

        



