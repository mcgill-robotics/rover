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
        self.gps_map = GPS_Map()
        rospy.init_node('gps_node')
        self.robot_twist_subscriber = rospy.Subscriber("/rover_gps_coordinate", Float32MultiArray, self.gps_map.update_robot_location)
        self.rate = rospy.Rate(100)
        
        

if __name__ == "__main__":
    driver = Node_GPS()
    rospy.spin()
