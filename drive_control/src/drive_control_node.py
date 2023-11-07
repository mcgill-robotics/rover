import os, sys
import time
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
import numpy as np
from steering import Steering
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from drive_control.msg import WheelSpeed
from geometry_msgs.msg import Twist
from scipy.ndimage import gaussian_filter1d
import scipy.stats as st

class Node_DriveControl():

    def __init__(self):
        """
        Hard-coded the simulation rover measurement, need to adapt it towards the real model.
        Script takes care of converting twist velocities from controller into rover wheel velocities.
        Specifically, velocities are published to create an accelerating filter (local average filter for math geeks).
        
        """
        self.wheel_radius = 0.04688 # In meters.
        self.wheel_base_length = 0.28 # In meters.
        self.wheel_speed = [0, 0] # Array elements: Left wheel speed, right wheel speed.
        self.steering = Steering(self.wheel_radius, self.wheel_base_length)
        self.sample_size = 50 # The greater the value, the more "smoothing" of the wheel speeds.
        self.basis = self.gkern(kernlen=50) # Gaussian kernel array. "kernlen" must be the same as "self.sample_size".

        # Velocity samples for four wheels
        self.front_left = np.zeros(self.sample_size).tolist()
        self.back_left = np.zeros(self.sample_size).tolist()
        self.front_right = np.zeros(self.sample_size).tolist()
        self.back_right = np.zeros(self.sample_size).tolist()
        
        rospy.init_node('drive_controller')
        self.angular_velocity_publisher = rospy.Publisher('/wheel_velocity_cmd', WheelSpeed, queue_size=1)
        self.robot_twist_subscriber = rospy.Subscriber("rover_velocity_controller/cmd_vel", Twist, self.twist_to_velocity)

        # Control Frequency of the drive controller
        self.rate = rospy.Rate(50)

        self.run()
    
    # Function to receive twist velocities and call the steering function to get the rover wheel velocities.
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
            cmd = WheelSpeed() # Create the wheel speed message.        
            print(f"Desired speed: {self.wheel_speed}")
            
            # Velocity filtering:

            # Append the most recent steering values.
            self.front_left.append(self.wheel_speed[0])
            self.back_left.append(self.wheel_speed[0])
            self.front_right.append(self.wheel_speed[1])
            self.back_right.append(self.wheel_speed[1])

            # Remove the oldest values to keep the array a constant length.
            self.front_left.pop(0)
            self.back_left.pop(0)
            self.front_right.pop(0)
            self.back_right.pop(0)
            
            front_left = np.asarray(self.front_left)
            back_left = np.asarray(self.back_left)
            front_right = np.asarray(self.front_right)
            back_right = np.asarray(self.back_right)
            
            # Local average filter calculation.
            correct_lf = np.sum(front_left * self.basis)
            correct_lb = np.sum(back_left * self.basis)
            correct_rf = np.sum(front_right * self.basis)
            correct_rb = np.sum(back_right * self.basis)

            # Populate the message with the averaged values.
            cmd.left[0], cmd.left[1] = correct_lf, correct_lb
            cmd.right[0], cmd.right[1] = correct_rf, correct_rb
            # print(f"Corrected speed: {correct_lf}, {correct_lb}, {correct_rf}, {correct_rb}")

            # motor_lf = self.motor_speed(correct_lf)
            # motor_lb = self.motor_speed(correct_lb)
            # motor_rf = self.motor_speed(correct_rf)
            # motor_rb = self.motor_speed(correct_rb)

            # Final if statement to make sure that in place steering works.
            # if correct_lf == correct_lb and correct_rf == correct_rb and motor_rb == - motor_lb:
            #    motor_lf = correct_lf/Node_DriveControl.MAX_PURE_STEER
            #    motor_lb = correct_lb/Node_DriveControl.MAX_PURE_STEER
            #    motor_rf = correct_rf/Node_DriveControl.MAX_PURE_STEER
            #    motor_rb = correct_rb/Node_DriveControl.MAX_PURE_STEER
            

            # motor_val.data.append(motor_rb * 100)  # rb
            # motor_val.data.append(motor_lf * 58)  # lf, too fast
            # motor_val.data.append(motor_lb * -100)  # lb, tpo slow
            # motor_val.data.append(motor_rf * 88)  # rf

            print(cmd)

            self.angular_velocity_publisher.publish(cmd) # Send the angular speeds.
            # self.motor_pubisher.publish(motor_val)

            self.rate.sleep()
            print(cmd)     

# ROS runtime main entry point
if __name__ == "__main__":
    driver = Node_DriveControl()
    rospy.spin()
