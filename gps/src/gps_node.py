import os, sys
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import psutil
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
from std_msgs.msg import Float32MultiArray
from gps_mapping import GPS_Map

class Node_GPS():


    def __init__(self):
        rospy.init_node('gps_node')
        self.robot_twist_subscriber = rospy.Subscriber("/roverGPSData", Float32MultiArray, self.write_data)
        self.rate = rospy.Rate(100)
    
    def write_data(self, msg: Float32MultiArray):
        with open('data.txt', 'w', encoding='utf-8') as f:
            f.write(f"{msg.data[0]},{msg.data[1]}")
        

if __name__ == "__main__":
    driver = Node_GPS()
    rospy.spin()
