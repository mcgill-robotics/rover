import csv
from time import ctime
import time
import os
import sys
import math 
import matplotlib.pyplot as plt
import random
from pathlib import Path

def create_file():
    file_name = "trajectory-recording.csv"

    cwd = Path.cwd()
    completeName = os.path.join(cwd, file_name)
    f = open(completeName, 'w')
    writer = csv.writer(f)

    #write the header
    header = ['x coord.','y coord','time']
    writer.writerow(header)

def recording(x_coord, y_coord):

    cwd = Path.cwd()

    file_path = "/trajectory-recording-test/trajectory"
    file_name = "trajectory-recording.csv"

    completeName = os.path.join(cwd, file_name)
    f = open(completeName, 'a')
    writer = csv.writer(f)

    c = time.time()
    current_time=ctime(c)
    #write x and y coord and time

    row = [[str(x_coord)]+[str(y_coord)]+[str(current_time)]]

    writer.writerows(row)
def read_csv_file():
    return 0
    
create_file()
recording(34, 34)
recording(69, 420)
recording(390, 454)


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






