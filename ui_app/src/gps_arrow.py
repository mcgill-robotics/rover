import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import os
import time
import datetime


class GPS_Map:

    def __init__(self):
        rospy.init_node('gps_node')

        self.control_station = [51.4706844, -112.7525343]
        self.fixed_points = [
            [51.47087, -112.75299]
            # [51.47109,-112.75209],
            # [51.47014,-11275346]
        ]

        self.robot_marker = None
        self.animation = None
        self.first_gps_data = []
        
 # Logger for sensor data
        # self.gps_log_path = os.path.dirname(__file__) + f"/../../rover_out/{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
        # os.makedirs(self.gps_log_path, exist_ok=True)
        # self.gps_log_filename = self.gps_log_path + "/gps_data.log"
        # self.gps_subscriber = rospy.Subscriber("roverGPSData", Float32MultiArray, self.plot_gps_figure)

        

    def plot_gps_figure(self,gps_data):

        # Log new data feedback
        # log_file = open(self.gps_log_filename, "a")
        # log_file.write(f"{time.time()},{gps_data.data[0]},{gps_data.data[1]}\n")
        # log_file.close()


        self.first_gps_data.append([gps_data.data[0], gps_data.data[1]])
        fig, ax = plt.subplots()
    
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
        plt.show()

    

if __name__ == "__main__":
    gps = GPS_Map()
