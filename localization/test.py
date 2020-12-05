#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker)

rospy.init_node('register')


while not rospy.is_shutdown():
   marker = Marker()
   marker.header.frame_id = "/neck"
   marker.type = marker.SPHERE
   marker.action = marker.ADD
   marker.scale.x = 0.2
   marker.scale.y = 0.2
   marker.scale.z = 0.2
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 1.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = 0
   marker.pose.position.y = 0
   marker.pose.position.z = 0

   publisher.publish(marker)
