#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String
from geodesy.utm import UTMPoint # imports latitude, longitude to utm converter (interface)

rospy.init_node("BU353")
ser = serial.Serial('/dev/ttyUSB0', 4800, timeout = 5)

gpsPublisher = rospy.Publisher('bu353_data', String)
while not rospy.is_shutdown():
    line = ser.readline()
    line = line.decode("utf-8")
    splitline = line.split(",")

    if splitline[0] == "$GPGGA":
      gpsPublisher.publish(line)
	
