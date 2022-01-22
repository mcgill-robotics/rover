import csv
from time import ctime
import time
import os
import sys
import math 
import matplotlib.pyplot as plt
import random
from pathlib import Path

#generate data
def gen_data(i):
    return math.sin( i/4) + random.randint(0, 10)/25

#plotting
def PlotKF(time, position, ylabel, col): #Plots KF
    plt.scatter(time, position, color=col, linestyle='--', linewidths=0.1, 
                marker='o', label=ylabel)
    plt.xlabel('time')
    plt.ylabel(ylabel)
    #plt.yscale("linear")
    plt.legend(loc='upper right') 
    plt.show()

#create file with the header
def create_file():
    file_name = "trajectory-recording.csv"

    cwd = Path.cwd()
    completeName = os.path.join(cwd, file_name)
    f = open(completeName, 'w')
    writer = csv.writer(f)

    #write the header
    header = ['x coord.','y coord','time']
    writer.writerow(header)

#add an extra data row to the file
def recording(x_coord, y_coord):

    #current file path
    cwd = Path.cwd()
    file_name = "trajectory-recording.csv"

    #open existing file
    completeName = os.path.join(cwd, file_name)
    f = open(completeName, 'a')
    writer = csv.writer(f)

    c = time.time()
    current_time=ctime(c)
    #write x and y coord and time

    row = [[str(x_coord)]+[str(y_coord)]+[str(current_time)]]

    writer.writerows(row)

    
create_file()


time_stamps = []
positions = []
i = 1


while( i < 100  ):
    position = gen_data(i)
    positions.append(position)
    time_stamps.append(i)
    i += 1
    
    recording(position, position)

PlotKF(time_stamps, positions, "position", "blue")






