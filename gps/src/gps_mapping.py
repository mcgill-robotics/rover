import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import psutil

class GPS_Map:

    def __init__(self):
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

        self.control_station = [45.5056037902832, -73.57576751708984]
        self.fixed_points = [[45.50570297241211, -73.57591247558594], [45.50539779663086, -73.5755615234375], [45.50568771362305, -73.5759506225586]]
        self.robot_marker = None
        self.animation = None

        print(self.fixed_points)
        self.robot_location_x = 51.5074
        self.robot_location_y = -0.1278

        self.run()


    def update_robot_location(self, frame):
        location_x, location_y = self.get_robot_location()  
        self.robot_marker.set_offsets([location_y, location_x])


    def get_robot_location(self):
        with open('data.txt', 'r', encoding='utf-8') as f:
            data = f.read()
            if data == "":
                return self.robot_location_x, self.robot_location_y
            data = data.split(',')
            location_x = float(data[0])
            location_y = float(data[1])
            print(location_x, location_y)
            return location_x, location_y


    def run(self):
        # Create a scatter plot
        fig, ax = plt.subplots()
        control_marker = ax.scatter(self.control_station[1], self.control_station[0], c='black', label='Control Station')

        for i in range(1, len(self.fixed_points)):
            if i == 1:
                goal_marker = ax.scatter(self.fixed_points[i][1], self.fixed_points[i][0], c='red', label='Goal')
            else:
                goal_marker = ax.scatter(self.fixed_points[i][1], self.fixed_points[i][0], c='red')

        self.robot_marker = ax.scatter([], [], c='blue', label='Robot')
        self.animation = FuncAnimation(fig, self.update_robot_location, frames=range(200), interval=1)

        longitude_max = max(self.fixed_points, key=lambda x: x[1])[1]
        longitude_min = min(self.fixed_points, key=lambda x: x[1])[1]
        latitude_max = max(self.fixed_points, key=lambda x: x[0])[0]
        latitude_min = min(self.fixed_points, key=lambda x: x[0])[0]

        ax.set_xlim(longitude_min - 0.05, longitude_max + 0.05)
        ax.set_ylim(latitude_min - 0.05, latitude_max + 0.05)

        # Add labels and legend
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.legend()

        # Show the scatter plot
        plt.show()



if __name__ == "__main__":
    gps = GPS_Map()