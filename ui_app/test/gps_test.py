import rospy
from std_msgs.msg import Float32MultiArray
import random
from time import sleep

robot_twist_subscriber = rospy.Publisher("/roverGPSData", Float32MultiArray, queue_size=0)
rospy.init_node('gps_publisher')
# 51.4537192,-112.7160973
while not rospy.is_shutdown():
    robot_twist_subscriber.publish(Float32MultiArray(data=[51.4537192+ random.random()/10000, -112.7160973 + random.random()/10000]))
    sleep(1)