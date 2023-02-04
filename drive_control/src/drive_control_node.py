import os, sys
import time
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
import numpy as np
from steering import Steering
from simple_pid import PID
from std_msgs.msg import String
from drive_control.msg import WheelSpeed
from geometry_msgs.msg import Twist
from scipy.ndimage import gaussian_filter1d
import scipy.stats as st

class Node_DriveControl():

    def __init__(self):
        self.wheel_radius = 0.04688
        self.wheel_base_length = 0.28
        self.wheel_speed = None
        self.correction_wheel_speed = None
        self.steering = Steering(self.wheel_radius, self.wheel_base_length)
        self.right_front_pid_controller = PID(1, 0.1, 0.05, 0)
        self.left_front_pid_controller = PID(1, 0.1, 0.05, 0)
        self.right_rear_pid_controller = PID(1, 0.1, 0.05, 0)
        self.left_rear_pid_controller = PID(1, 0.1, 0.05, 0)

        # Gaussian kernel
        self.basis = self.gkern(kernlen=50)

        # Filter samples for four wheels
        self.front_left = np.zeros(50).tolist()
        self.back_left = np.zeros(50).tolist()
        self.front_right = np.zeros(50).tolist()
        self.back_right = np.zeros(50).tolist()
        
        rospy.init_node('drive_controller')
        self.angular_velocity_publisher = rospy.Publisher('/wheel_velocity_cmd', WheelSpeed, queue_size=1)
        self.robot_twist_subscriber = rospy.Subscriber("rover_velocity_controller/cmd_vel", Twist, self.twist_to_velocity)
        self.feedback_velocity_subscriber = rospy.Subscriber('/feedback_velocity', WheelSpeed, self.set_correction_velocity)

        # The controller publisher is publishing straight to the twist_to_velocity values
        # which then results in what we have now. Must I add a new publisher that creates a sample first
        # and then feeds it to steering? yes

        self.run()
    
    def twist_to_velocity(self, robot_twist):
        vR = robot_twist.linear.x
        wR = robot_twist.angular.z
        self.wheel_speed = self.steering.steering_control(vR, wR)

    # Function that yields a 1D gaussian basis
    # Source: https://stackoverflow.com/questions/29731726/how-to-calculate-a-gaussian-kernel-matrix-efficiently-in-numpy
    def gkern(self, kernlen=10, sigma=3):
        x = np.linspace(-sigma, sigma, kernlen+1)
        kern1d = np.diff(st.norm.cdf(x))
        return kern1d/kern1d.sum()


    def run(self):
        while not rospy.is_shutdown():
            cmd = WheelSpeed()
        
            print(f"Desired speed: {self.wheel_speed}")
            
            if(self.correction_wheel_speed is not None):
                if(self.correction_wheel_speed.left[0] < 1):
                    cmd.left[0] = self.wheel_speed[0]
                else:
                    cmd.left[0] = self.correction_wheel_speed.left[0]

                if(self.correction_wheel_speed.left[1] < 1):
                    cmd.left[1] = self.wheel_speed[0]
                else:
                    cmd.left[1] = self.correction_wheel_speed.left[1]

                if(self.correction_wheel_speed.left[0] < 1):
                    cmd.right[0] = self.wheel_speed[1]
                else:
                    cmd.right[0] = self.correction_wheel_speed.right[0]

                if(self.correction_wheel_speed.left[0] < 1):
                    cmd.right[1] = self.wheel_speed[1]
                else:
                    cmd.right[1] = self.correction_wheel_speed.right[1]    
            elif(self.wheel_speed is not None):
                cmd.left[0] = self.wheel_speed[0]
                cmd.right[0] = self.wheel_speed[1]
                cmd.left[1] = self.wheel_speed[0]
                cmd.right[1] = self.wheel_speed[1]
            else:
                continue

            # Velocity filtering:
            self.front_left.append(self.wheel_speed[0])
            self.back_left.append(self.wheel_speed[0])
            self.front_right.append(self.wheel_speed[1])
            self.back_right.append(self.wheel_speed[1])

            self.front_left.pop(0)
            self.back_left.pop(0)
            self.front_right.pop(0)
            self.back_right.pop(0)
            
            front_left = np.asarray(self.front_left)
            back_left = np.asarray(self.back_left)
            front_right = np.asarray(self.front_right)
            back_right = np.asarray(self.back_right)

            correct_lf = np.sum(front_left * self.basis)
            correct_lb = np.sum(back_left * self.basis)
            correct_rf = np.sum(front_right * self.basis)
            correct_rb = np.sum(back_right * self.basis)

            cmd.left[0], cmd.left[1] = correct_lf, correct_lb
            cmd.right[0], cmd.right[1] = correct_rf, correct_rb

            self.angular_velocity_publisher.publish(cmd)
            time.sleep(0.1)
            print(cmd)

    
    def set_correction_velocity(self, velocity_feedback):
        self.correction_wheel_speed = WheelSpeed()

        self.right_front_pid_controller.setpoint = self.wheel_speed[1]
        self.left_front_pid_controller.setpoint = self.wheel_speed[0]
        self.right_rear_pid_controller.setpoint = self.wheel_speed[1]
        self.left_rear_pid_controller.setpoint = self.wheel_speed[0]
        
        vLeftFront = self.left_front_pid_controller(velocity_feedback.left[0])
        vRightFront = self.right_front_pid_controller(velocity_feedback.right[0])
        vLeftRear = self.left_rear_pid_controller(velocity_feedback.left[1])
        vRightRear = self.right_rear_pid_controller(velocity_feedback.right[1])

        self.correction_wheel_speed.left[0] = vLeftFront
        self.correction_wheel_speed.right[0] = vRightFront
        self.correction_wheel_speed.left[1] = vLeftRear
        self.correction_wheel_speed.right[1] = vRightRear


if __name__ == "__main__":
    driver = Node_DriveControl()
    rospy.spin()
