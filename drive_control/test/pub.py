import time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from drive_control.msg import WheelSpeed
    

def pub():
    robot_twist = Twist()
    robot_twist.angular.z = 1
    robot_twist.linear.x = 1
    # robot_twist.angular.z = 1500
    twist_pub = node.create_publisher(Twist, queue_size=10, "rover_velocity_controller/cmd_vel")
    
    feedback = WheelSpeed()
    feedback.left[0] = 1
    feedback.left[1] = 1
    feedback.right[0] = 1
    feedback.right[1] = 1
    feedback_pub = node.create_publisher(WheelSpeed, queue_size=10, '/feedback_velocity')

    position_pub = rospy.Publisher('/position_pose',Pose,queue_size=10)
    rover_position = Pose()
    rover_position.position.x = 2.0
    rover_position.position.y = 3.0
    rover_position.position.z = 1.0
    rover_position.orientation.x = 0.778
    rover_position.orientation.y = 0.34
    rover_position.orientation.z = 0.35
    rover_position.orientation.w = 0.395



    rclpy.init()



    node = rclpy.create_node('test_pub', anonymous=True)
    i = 0
    while not rospy.is_shutdown():
        robot_twist.linear.x = 30 + i
        twist_pub.publish(robot_twist)
        position_pub.publish(rover_position)
        feedback_pub.publish(feedback)
        rospy.loginfo(rover_position)
        time.sleep(0.35)
        i += 10
        

if __name__ == "__main__":
    pub()

# import time
# import rospy
# from geometry_msgs.msg import Twist
# from drive_control.msg import WheelSpeed
    

# def pub():
#     robot_twist = Twist()
#     robot_twist.linear.x = 10
#     robot_twist.angular.z = 20
#     twist_pub = node.create_publisher(Twist, queue_size=10, "rover_velocity_controller/cmd_vel")
    
#     feedback = WheelSpeed()
#     feedback.left[0] = 20
#     feedback.left[1] = 22
#     feedback.right[0] = 18
#     feedback.right[1] = 20
#     feedback_pub = node.create_publisher(WheelSpeed, queue_size=10, '/feedback_velocity')

#     rospy.init_node('test_pub', anonymous=True)
#     i = 0
#     while not rospy.is_shutdown():
#         twist_pub.publish(robot_twist)
#         feedback_pub.publish(feedback)
#         time.sleep(1)
        




# if __name__ == "__main__":
#     pub()

    