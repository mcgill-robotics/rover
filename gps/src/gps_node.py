import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from std_msgs.msg import Float32MultiArray


class GPS_Map:

    def __init__(self):
        rospy.init_node('gps_node')

        # Coordinates : (latitude, longitude)
        # location_input = input("Enter control station coordinates (separated by commas): ")
        # values_list = location_input.split(',')
        # self.control_station = tuple(float(value.strip()) for value in values_list)

        # self.fixed_points = []
        # self.fixed_points.append(self.control_station)
        # while True:
        #     location_input = input("Enter checkpoint coordinates (separated by commas): ")
        #     if location_input == "":
        #         break
        #     values_list = location_input.split(',')
        #     location = tuple(float(value.strip()) for value in values_list)   
        #     self.fixed_points.append(location) 

        # for testing
        self.control_station = [45.5056037902832, -73.57576751708984]
        self.fixed_points = [
            [45.50570297241211, -73.57591247558594], 
            [45.50539779663086, -73.5755615234375],
            [45.50568771362305, -73.5759506225586]
        ]

        self.robot_marker = None
        self.animation = None
        self.run()



    def run(self):
        fig, ax = plt.subplots()
        ax.scatter(self.control_station[1], self.control_station[0], c='black', label='Control Station')

        lats, lons = zip(*self.fixed_points)  # Unpack latitudes and longitudes

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

        plt.savefig('gps_plot.png')
        plt.show()


if __name__ == "__main__":
    gps = GPS_Map()