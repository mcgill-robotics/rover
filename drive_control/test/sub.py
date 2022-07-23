import rospy
import time
from drive_control.msg import WheelSpeed

def sub():
    rospy.init_node('test_sub', anonymous=True)
    sub = rospy.Subscriber('/wheel_velocity_cmd', WheelSpeed, printValues)
    rospy.spin()

def printValues(values):
    time.sleep(0.1)
    print(values)

if __name__ == "__main__":
    sub()
    