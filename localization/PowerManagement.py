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
""" Info about the battery
    https://www.genstattu.com/ta-p2-25c-22000-6s1p-as150.html
    configuration : 6S1P 
    discharge : 25C
    peak discharge : 50C
    
"""

#imports 
import time
import matplotlib.pyplot as plt

#constants or ROS dependant variables 
I_amp = 20000 #current measured by sensor (A)
V_V = 22.2 #Initial voltage 

battery_1_Q = 22.0 * 3600.0 # Ahr/3600 = Q, charge of battery_1

predicted_time_remaining = []
percentage_left_arr_V = []
percentage_left_arr_C = []
time_arr = []

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
            print('More charge drawn than total battery capacity')
            
        #if Discharge rate > 25C, raise an error 
        if (I_amp > 25):
            print('Discharge rate too high! Limit is 25C/s')
        
        return self.charge_drawn
        
    #get the predicted remaining battery life at a current consumption I_amp
    def getPredictedBatteryLife(self):
        #Find how much electric charge remains
        self.charge_remain = self.total_charge - self.charge_drawn
        #Find nbr seconds remaining at this rate
        self.seconds_remain = self.charge_remain/I_amp 
        return self.seconds_remain
    
    #get the percentage of battery life based only on remaining charge 
    def getPercentageCurrent(self):
        per = ((self.total_charge - self.charge_drawn)/self.total_charge) * 100
        if (per < 15):
            print('Voltage Critically Low (<15%). Recharge battery!')
        if (per > 15):
            return per
    
    """The voltage discharge curve IS NOT PROVIDED IN THE DATASHEET.
        Lipo batteries have a fairly flat voltage curve.
        But, this value varies with load, temperature, internal impedance. 
        Readings on the subject:
        https://www.mpoweruk.com/performance.htm
        https://www.dnkpower.com/lithium-polymer-battery-guide/ 
        https://blog.ampow.com/lipo-voltage-chart/ #Votlage curve here 
        State of Charge estimation (SoC):
            https://www.mpoweruk.com/soc.htm
        """
        
    """The voltage curve was calculated by assuming:
        1.The voltage discharge curve is linear over the 3.75-4.11V range of the cell
        2.I ignore the effect of temperature 
    """
    #get the percentage of battery life based only on voltage readings 
    def getPercentageVoltage(self, V_V):
        #if the voltage is really high, say it is >90%
        if (V_V > 24.66):
            return 90
        #if the voltage is critically low, raise an error 
        if (V_V <= 22.26):
            print('Voltage Critically Low (<15%). Recharge battery!')
        #if the voltage is in the right range, calculate the battery %
        if (V_V <= 24.66 and V_V >= 22.5):
            return (V_V-21.6693)/0.03323
        
        
def PlotKF(time, position, ylabel, col): #Plots KF
    plt.scatter(time, position, color=col, linestyle='--', linewidths=0.1, 
                marker='o', label=ylabel)
    plt.xlabel('time')
    plt.ylabel(ylabel)
    plt.yscale("linear")
    plt.legend(loc='upper right')    
    
"For the sake of the test, I will generate voltage data to feed into getPercentageVoltage()"
    
#Unit Testing 
M1 = PowerManagement(battery_1_Q) #create a PowerManagement object
i = 0 #variable only used for plotting 
while(i < 100):
    time_arr.append(i) 
    M1.getTotalChargeDrawn() #update the charge_drawn value 
    
    #generate fake data to feed into getPercentVoltage
    fake_percentage = M1.getPercentageCurrent()
    percentage_left_arr_C.append(fake_percentage)
    fake_voltage = 0.03323*fake_percentage + 21.6693
    #feed said data into function 
    percentage_left_arr_V.append(M1.getPercentageVoltage(fake_voltage))
    
    predicted_time_remaining.append(M1.getPredictedBatteryLife())
    time.sleep(0.01) #pause for 1s
    i += 1

PlotKF(time_arr, predicted_time_remaining, "predicted battery life (s)", "blue")  
plt.show() 
PlotKF(time_arr, percentage_left_arr_C, "percentage battery left (%) - Current", "blue")
PlotKF(time_arr, percentage_left_arr_V, "percentage battery left (%) - Voltage", "green")     
print("done calculating")


