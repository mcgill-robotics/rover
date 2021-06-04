#!/usr/bin/env python

import rospy
import serial
import pynmea2 # package for parsing NMEA sentences given by BU353-S4
from std_msgs.msg import String
from geodesy.utm import UTMPoint # imports latitude, longitude to utm converter (interface)

rospy.init_node("BU353")
ser = serial.Serial('/dev/ttyUSB0', 4800, timeout = 5)

gpsPublisher = rospy.Publisher('bu353_data', String)
while not rospy.is_shutdown():
    line = ser.readline()
    splitline = line.split(",")

    if splitline[0] == "$GPGGA":
		gpsPublisher.publish(str(line))
	
