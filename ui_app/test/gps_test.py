import rospy
from std_msgs.msg import Float32MultiArray
import random

robot_twist_subscriber = rospy.Publisher("/roverGPSData", Float32MultiArray, queue_size=10)
rospy.init_node('gps_publisher')

while not rospy.is_shutdown():
    robot_twist_subscriber.publish(Float32MultiArray(data=[45.50539779663086+ random.random()/1000, -73.5755615234375 + random.random()/1000]))