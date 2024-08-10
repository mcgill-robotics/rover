#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import datetime

output_file = "src/app/components/gps/GPSWaypoints.txt"


def gps_callback(data):
    
    latitude = data.data[0]
    longitude = data.data[1]

    now = datetime.datetime.now()

    with open(output_file, "a") as f:
        f.write(f"Latitude: {latitude}, Longitude: {longitude}, Time: {now.time()}\n")

    rospy.loginfo(f"Coordinates on Route: Latitude: {latitude}, Longitude: {longitude}, Time: {now.time()}")

def gps_listener():
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber("/roverGPSData", Float32MultiArray, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_listener()
    except rospy.ROSInterruptException:
        pass

