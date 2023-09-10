import rospy
import time
from drive_control.msg import WheelSpeed

def sub():
    rclpy.init()
    node = rclpy.create_node('test_sub', anonymous=True)
    # sub = node.create_subscription(WheelSpeed, '/wheel_velocity_cmd', printValues)
    sub = node.create_subscription(WheelSpeed, '/feedback_velocity', printValues)
    rospy.spin()

def printValues(values):
    time.sleep(0.1)
    print(values)

if __name__ == "__main__":
    sub()
    