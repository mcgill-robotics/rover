import rospy
import Steering
from std_msgs.msg import String

class Node_DriveControl():

    def __init__(self):
        self.wheel_radius = 10

        self.angular_velocity = 

        pub = rospy.Publisher('angular_velocity', String, queue_size=10)
        rospy.init_node('angular_velocity_publisher')
        rate = rospy.Rate(10)

    def publisher():
        



if __name__ == "__main__":
    driver = Node_DriveControl()
    rospy.spin()