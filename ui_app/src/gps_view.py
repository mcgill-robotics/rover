import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np


class GPS_Map:

    def __init__(self):
        rospy.init_node('gps_node')

        self.control_station = [51.4706844, -112.7525343]
        self.fixed_points = [
            [51.453239441, -112.716644287]
        ]

        self.debris = [
            [51.453140259, -112.716438293],
            [51.453266144, -112.716186523],
            [51.453479767, -112.716339111],
            [51.453620911, -112.716606140]
        ]

        self.robot_marker = None
        self.animation = None
        self.first_gps_data = []
        self.run()


    def every_frame(self, _):
        self.robot_marker.set_offsets(rospy.wait_for_message(
                                           '/roverGPSData', Float32MultiArray).data[::-1])


    def run(self):
        fig, ax = plt.subplots(figsize=(8, 8))  # Adjust the dimensions as needed
        ax.scatter(self.control_station[1], self.control_station[0], c='black', label='Control Station')

        lats, lons = zip(*self.fixed_points)  # Unpack latitudes and longitudes

        ax.scatter(lons, lats, c='green', label='Goal')

        lats, lons = zip(*self.debris)  # Unpack latitudes and longitudes

        ax.scatter(lons, lats, c='red', label='Debris')

        for d in self.debris:
            circle = plt.Circle((d[1], d[0]), 0.00004497, color='red', fill=False)
            ax.add_artist(circle)

        self.robot_marker = ax.scatter([], [], c='blue', label='Robot')
        #self.first_gps_data.append([rospy.wait_for_message('/roverGPSData', Float32MultiArray).data[0], rospy.wait_for_message('/roverGPSData', Float32MultiArray).data[1]])
        self.animation = FuncAnimation(fig,
                                       self.every_frame,
                                       frames=range(200))
        
        if len(self.first_gps_data) == 2:
            dx = self.first_gps_data[1][0] - self.first_gps_data[0][0]
            dy = self.first_gps_data[1][1] - self.first_gps_data[0][1]
            norm = np.sqrt(dx**2 + dy**2)
            dx = dx/norm if not np.isclose(norm, 0) else 0
            dy = dy/norm if not np.isclose(norm, 0) else 0
            FuncAnimation(fig,ax.arrow(self.first_gps_data[1][1], self.first_gps_data[1][0], dy*0.00001, dx*0.00001, width =0.00001,head_width=0.00005, head_length=0.00002,ec='blue'),frames=range(200))
            self.first_gps_data.pop(0)
        


        # compute and set limits on graph
        longitude_max = max(self.fixed_points, key=lambda x: x[1])[1]
        longitude_min = min(self.fixed_points, key=lambda x: x[1])[1]
        latitude_max = max(self.fixed_points, key=lambda x: x[0])[0]
        latitude_min = min(self.fixed_points, key=lambda x: x[0])[0]

        ax.set_aspect('equal')


        ax.set_xlim(longitude_min - 0.00075, longitude_max + 0.00075)
        ax.set_ylim(latitude_min - 0.00075, latitude_max + 0.00075)

        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.legend()

        plt.savefig('gps_plot.png')
        plt.show()


if __name__ == "__main__":
    gps = GPS_Map()
