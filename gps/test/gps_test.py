import rospy
from std_msgs.msg import Float32MultiArray
import random

robot_twist_subscriber = rospy.Publisher("/roverGPSData", Float32MultiArray, queue_size=10)
rospy.init_node('gps_publisher')

while True:
    robot_twist_subscriber.publish(Float32MultiArray(data=[45.50+ random.random()/100, -73.57 + random.random()/100]))