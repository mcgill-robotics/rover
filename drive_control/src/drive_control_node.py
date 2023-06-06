import os, sys
import time
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
import numpy as np
from steering import Steering
from std_msgs.msg import String
from drive_control.msg import WheelSpeed
from geometry_msgs.msg import Twist
from scipy.ndimage import gaussian_filter1d
import scipy.stats as st

class Node_DriveControl():

    MAX_NO_STEER = 21.333
    MAX_WITH_STEER = 95.99
    MIN_NO_STEER = -MAX_NO_STEER
    MIN_WITH_STEER_1 = -53.33
    MIN_WITH_STEER_2 = -MAX_WITH_STEER


    def __init__(self):
        """
        Hard-coded the simulation rover measurement, will need to adapt it towards the real model.
        Drive control takes care interpolating velocities and publishing them.
        """
        self.wheel_radius = 0.04688
        self.wheel_base_length = 0.28
        self.wheel_speed = [0, 0]
        self.steering = Steering(self.wheel_radius, self.wheel_base_length)
        self.sample_size = 50
        # Gaussian kernel
        self.basis = self.gkern(kernlen=50)

        # Filter samples for four wheels
        self.front_left = np.zeros(self.sample_size).tolist()
        self.back_left = np.zeros(self.sample_size).tolist()
        self.front_right = np.zeros(self.sample_size).tolist()
        self.back_right = np.zeros(self.sample_size).tolist()
        
        rospy.init_node('drive_controller')
        self.angular_velocity_publisher = rospy.Publisher('/wheel_velocity_cmd', WheelSpeed, queue_size=1)
        self.robot_twist_subscriber = rospy.Subscriber("rover_velocity_controller/cmd_vel", Twist, self.twist_to_velocity)
        # The controller publisher is publishing straight to the twist_to_velocity values
        # which then results in what we have now. Must I add a new publisher that creates a sample first
        # and then feeds it to steering? yes

        # Control Frequency of the drive controller
        self.rate = rospy.Rate(100)

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
            
            # Interpolating the veloctities according to gaussian kernel.
            correct_lf = np.sum(front_left * self.basis)
            correct_lb = np.sum(back_left * self.basis)
            correct_rf = np.sum(front_right * self.basis)
            correct_rb = np.sum(back_right * self.basis)

            # Send out the correct values.
            cmd.left[0], cmd.left[1] = correct_lf, correct_lb
            cmd.right[0], cmd.right[1] = correct_rf, correct_rb

            motor_lf = self.motor_speed(correct_lf)
            motor_lb = self.motor_speed(correct_lb)
            motor_rf = self.motor_speed(correct_rf)
            motor_rb = self.motor_speed(correct_rb)

            
            print(f"Motor values: {[motor_lf, motor_lb, motor_rf, motor_rb]}")

            self.angular_velocity_publisher.publish(cmd)

            self.rate.sleep()
            print(cmd)
        
    # def motor_speed(self, cur_val):
                
    # TODO: CHECK MOTOR VALUES AND VERIFY CALCULATIONS.
    def motor_speed(self, cur_val):
        motor_val = cur_val / Node_DriveControl.MAX_NO_STEER
        if motor_val < 1 and motor_val > 0:
            return motor_val

        if motor_val > 1:
            motor_val = cur_val / Node_DriveControl.MAX_WITH_STEER
            return motor_val
        
        if motor_val < 0:
            motor_val = -(cur_val / Node_DriveControl.MIN_NO_STEER)
            if motor_val < -1:
                motor_val = -(cur_val / Node_DriveControl.MIN_WITH_STEER_1)
                if motor_val < -1:
                    motor_val = -(cur_val / Node_DriveControl.MIN_WITH_STEER_2)
            return motor_val
        
        return motor_val


        

if __name__ == "__main__":
    driver = Node_DriveControl()
    rospy.spin()
