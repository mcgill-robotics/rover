import os, sys
import time
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
        self.same_speed = True
        self.correction_wheel_speed = None
        self.steering = Steering(self.wheel_radius, self.wheel_base_length)
        self.right_front_pid_controller = PID(1, 0.1, 0.05, 0)
        self.left_front_pid_controller = PID(1, 0.1, 0.05, 0)
        self.right_rear_pid_controller = PID(1, 0.1, 0.05, 0)
        self.left_rear_pid_controller = PID(1, 0.1, 0.05, 0)
        
        rospy.init_node('drive_controller')
        self.angular_velocity_publisher = rospy.Publisher('/wheel_velocity_cmd', WheelSpeed, queue_size=1)
        self.robot_twist_subscriber = rospy.Subscriber("rover_velocity_controller/cmd_vel", Twist, self.twist_to_velocity)
        self.feedback_velocity_subscriber = rospy.Subscriber('/feedback_velocity', WheelSpeed, self.set_correction_velocity)

        self.run()
    
    def twist_to_velocity(self, robot_twist):
        vR = robot_twist.linear.x
        wR = robot_twist.angular.z
        self.same_speed = False
        self.wheel_speed = self.steering.steering_control(vR, wR)



    def run(self):
        while not rospy.is_shutdown():
            if not self.same_speed:
                cmd = WheelSpeed()
        
                print(f"Desired speed: {self.wheel_speed}")
            
                if(self.correction_wheel_speed is not None):
                    cmd.left[0] = self.correction_wheel_speed.left[0]
                    cmd.left[1] = self.correction_wheel_speed.left[1]
                    cmd.right[0] = self.correction_wheel_speed.right[0]
                    cmd.right[1] = self.correction_wheel_speed.right[1] 

                elif(self.wheel_speed is not None):
                    cmd.left[0] = self.wheel_speed[0]
                    cmd.right[0] = self.wheel_speed[1]
                    cmd.left[1] = self.wheel_speed[0]
                    cmd.right[1] = self.wheel_speed[1]
                else:
                    continue

                self.angular_velocity_publisher.publish(cmd)
                self.same_speed = True
                time.sleep(0.1)
                print(cmd)

    
    def set_correction_velocity(self, velocity_feedback):
        self.correction_wheel_speed = WheelSpeed()

        #PID removed
        # self.right_front_pid_controller.setpoint = self.wheel_speed[1]
        # self.left_front_pid_controller.setpoint = self.wheel_speed[0]
        # self.right_rear_pid_controller.setpoint = self.wheel_speed[1]
        # self.left_rear_pid_controller.setpoint = self.wheel_speed[0]
        
        # vLeftFront = self.left_front_pid_controller(velocity_feedback.left[0])
        # vRightFront = self.right_front_pid_controller(velocity_feedback.right[0])
        # vLeftRear = self.left_rear_pid_controller(velocity_feedback.left[1])
        # vRightRear = self.right_rear_pid_controller(velocity_feedback.right[1])

        # self.correction_wheel_speed.left[0] = vLeftFront
        # self.correction_wheel_speed.right[0] = vRightFront
        # self.correction_wheel_speed.left[1] = vLeftRear
        # self.correction_wheel_speed.right[1] = vRightRear

        self.correction_wheel_speed.left[0] = self.wheel_speed[0]
        self.correction_wheel_speed.right[0] = self.wheel_speed[0]
        self.correction_wheel_speed.left[1] = self.wheel_speed[1]
        self.correction_wheel_speed.right[1] = self.wheel_speed[1]


if __name__ == "__main__":
    driver = Node_DriveControl()
    rospy.spin()
