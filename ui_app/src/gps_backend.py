#!/usr/bin/env python
# Imports
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32MultiArray
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
sys.path.append(currentdir + "/../..")


class GPS_Backend():

    def __init__(self, gps_tab):
        self.ui = gps_tab
        # self.ui.gps_plotter.clear()
        # self.ui.gps_plotter.plot(34,56,symbol='o', pen=None)
        #ax = self.ui.moisture_data_fig.add_subplot(111)
