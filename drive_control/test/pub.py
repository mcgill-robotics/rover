import time
import rospy
from geometry_msgs.msg import Twist
from drive_control.msg import WheelSpeed
    

def pub():
    robot_twist = Twist()
    robot_twist.linear.x = 10
    robot_twist.angular.z = 20
    twist_pub = rospy.Publisher("rover_velocity_controller/cmd_vel", Twist, queue_size=10)
    
    feedback = WheelSpeed()
    feedback.left[0] = 20
    feedback.left[1] = 22
    feedback.right[0] = 18
    feedback.right[1] = 20
    feedback_pub = rospy.Publisher('/feedback_velocity', WheelSpeed, queue_size=10)

    rospy.init_node('test_pub', anonymous=True)
    i = 0
    while not rospy.is_shutdown():
        twist_pub.publish(robot_twist)
        feedback_pub.publish(feedback)
        time.sleep(1)
        




if __name__ == "__main__":
    pub()

    