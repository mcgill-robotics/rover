#!/usr/bin/env python
# Imports
import numpy as np
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import time
import datetime

class GPS_Backend():

    def __init__(self, gps_tab):
        self.ui = gps_tab
        self.control_station = [45.5056037902832, -73.57576751708984]
        self.fixed_points = [
            [45.50570297241211, -73.57591247558594],
            [45.50539779663086, -73.5755615234375],
            [45.50568771362305, -73.5759506225586],
            [45.50548771362305, -73.5756506225586],
        ]

        self.robot_marmathker = None
        self.first_gps_data = []

         # Logger for sensor data
        self.gps_log_path = os.path.dirname(__file__) + f"/../../rover_out/{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
        os.makedirs(self.gps_log_path, exist_ok=True)
        self.gps_log_filename = self.gps_log_path + "/gps_data.log"

        

    def plot_gps_figure(self,gps_data):

        # Log new data feedback
        log_file = open(self.gps_log_filename, "a")
        log_file.write(f"{time.time()},{gps_data.data[0]},{gps_data.data[1]}\n")
        log_file.close()


        self.first_gps_data.append([gps_data.data[0], gps_data.data[1]])
        

        self.ui.gps_data.setText("latitude: " + str(gps_data.data[0])+ "\nlongitude: " + str(gps_data.data[1]))

        self.ui.gps_fig.clear()
        
        ax = self.ui.gps_fig.add_subplot(111)
        ax.scatter(self.control_station[1], self.control_station[0], c='black', label='Control Station')

        lats, lons = zip(*self.fixed_points)

        ax.scatter(lons, lats, c='red', label='Goal')

        if len(self.first_gps_data) == 2:
            dx = self.first_gps_data[1][0] - self.first_gps_data[0][0]
            dy = self.first_gps_data[1][1] - self.first_gps_data[0][1]
            norm = np.sqrt(dx**2 + dy**2)
            dx = dx/norm if not np.isclose(norm, 0) else 0
            dy = dy/norm if not np.isclose(norm, 0) else 0
            ax.arrow(self.first_gps_data[1][1], self.first_gps_data[1][0], dy*0.00001, dx*0.00001, width =0.00001,head_width=0.00005, head_length=0.00002,ec='blue')
            self.first_gps_data.pop(0)

        ax.scatter(gps_data.data[1], gps_data.data[0], c='blue', label='Robot')

        # compute and set limits on graph
        longitude_max = max(self.fixed_points, key=lambda x: x[1])[1]
        longitude_min = min(self.fixed_points, key=lambda x: x[1])[1]
        latitude_max = max(self.fixed_points, key=lambda x: x[0])[0]
        latitude_min = min(self.fixed_points, key=lambda x: x[0])[0]

        ax.set_xlim(longitude_min - 0.00025, longitude_max + 0.00025)
        ax.set_ylim(latitude_min - 0.00025, latitude_max + 0.00025)

        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.legend()
        self.ui.gps_plotter.draw()

