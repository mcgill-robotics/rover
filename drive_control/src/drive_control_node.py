import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
from steering import Steering
from simple_pid import PID
from std_msgs.msg import String
from drive_control.msg import WheelSpeed
from geometry_msgs.msg import Twist

class Node_DriveControl():

    def __init__(self):
        self.wheel_radius = 10
        self.wheel_base_length = 20
        self.wheel_speed = None
        self.correction_wheel_speed = WheelSpeed()
        self.steering = Steering(self.wheel_radius, self.wheel_base_length)
        self.right_pid_controller = PID(1, 0.1, 0.05, 0)
        self.left_pid_controller = PID(1, 0.1, 0.05, 0)
        
        rospy.init_node('drive_controller')
        self.angular_velocity_publisher = rospy.Publisher('/wheel_velocity_cmd', WheelSpeed, queue_size=1)
        self.robot_twist_subscriber = rospy.Subscriber('/robot_twist_cmd', Twist, self.twist_to_velocity)
        self.feedback_velocity_subscriber = rospy.Subscriber('/feedback_velocity', WheelSpeed, self.set_correction_velocity)

        self.run()
    
    def twist_to_velocity(self, robot_twist):
        vR = robot_twist.linear
        wR = robot_twist.angular
        self.wheel_speed = self.steering.steering_control(vR, wR)



    def run(self):
        while not rospy.is_shutdown():
            cmd = WheelSpeed()
        
            if(self.correction_wheel_speed == None):
                cmd.left = self.wheel_speed.left
                cmd.right = self.wheel_speed.right
            else:
                cmd.left = self.correction_wheel_speed.left
                cmd.right = self.correction_wheel_speed.right

            self.angular_velocity_publisher.publish(cmd)
            


    
    def set_correction_velocity(self, velocity_feedback):
        self.right_pid_controller.setpoint = self.wheel_speed.right
        self.left_pid_controller = self.wheel_speed.left
        
        vLeft = self.left_pid_controller(velocity_feedback.left)
        vRight = self.right_pid_controller(velocity_feedback.right)

        self.correction_wheel_speed.left = vLeft
        self.correction_wheel_speed.right = vRight




if __name__ == "__main__":
    driver = Node_DriveControl()
    rospy.spin()