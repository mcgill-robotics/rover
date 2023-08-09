#!/usr/bin/env python
# Imports
import matplotlib.pyplot as plt
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from matplotlib.animation import FuncAnimation
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)

class GPS_Backend():

    def __init__(self, gps_tab):
        self.ui = gps_tab
        self.control_station = [45.5056037902832, -73.57576751708984]
        self.fixed_points = [
            [45.50570297241211, -73.57591247558594], 
            [45.50539779663086, -73.5755615234375],
            [45.50568771362305, -73.5759506225586]
        ]

        self.robot_marker = None
        self.animation = None
        self.plot_gps_figure()

    def plot_gps_figure(self):

        self.ui.gps_fig.clear()
        fig = plt.figure()
        ax = self.ui.gps_fig.add_subplot(111)
        ax.scatter(self.control_station[1], self.control_station[0], c='black', label='Control Station')

        lats, lons = zip(*self.fixed_points)

        ax.scatter(lons, lats, c='red', label='Goal')

        self.robot_marker = ax.scatter([], [], c='blue', label='Robot')
        self.animation = FuncAnimation(fig,
                                       lambda _: self.robot_marker.set_offsets(rospy.wait_for_message(
                                           '/roverGPSData', Float32MultiArray, timeout=3).data[::-1]),
                                       frames=range(200))

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

