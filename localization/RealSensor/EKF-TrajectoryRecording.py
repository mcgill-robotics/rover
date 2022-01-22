



import math 
import matplotlib.pyplot as plt
import random

def gen_data(i):
    return math.sin( i/4) + random.randint(0, 10)/25

def PlotKF(time, position, ylabel, col): #Plots KF
    plt.scatter(time, position, color=col, linestyle='--', linewidths=0.1, 
                marker='o', label=ylabel)
    plt.xlabel('time')
    plt.ylabel(ylabel)
    #plt.yscale("linear")
    plt.legend(loc='upper right') 

time_stamps = []
positions = []
i = 1


while( i < 100  ):
    position = gen_data(i)
    positions.append(position)
    time_stamps.append(i)
    i += 1
    
#INSERT PERSISTENCE CODE HERE 

PlotKF(time_stamps, positions, "position", "blue")





