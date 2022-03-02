#!/usr/bin/env python3

import rospy
from human_control_interface.msg import Gamepad_input
from geometry_msgs.msg import Twist
import numpy as np

class Node_GamepadProcessing:
    def __init__(self, v_max, w_max):
        # initialize ROS node
        rospy.init_node("gamepad_process_node")

        # initialize variables for velocity
        self.roverLinearVelocity = 0
        self.roverAngularVelocity = 0

        # initialize variables for rover's max linear and angular velocities
        self.maxLinearVelocity = v_max
        self.maxAngularVelocity = w_max

        # initialize a subscriber for grabbing data from gamepad
        self.processSub = rospy.Subscriber("gamepad_data", Gamepad_input, self.gamepadProcessCall)
        self.processPub = rospy.Publisher("rover_velocity_controller/cmd_vel", Twist, queue_size=1)

    def gamepadProcessCall(self, msg):
        # assign axis values
        steering = msg.A1
        lt = msg.A3
        rt = msg.A6

        # normalize to [0, 1] range
        backward_vel = (lt + 1)/2
        forward_vel = (rt + 1)/2        

        # calc. for linear velocity
        self.roverLinearVelocity = self.maxLinearVelocity * (forward_vel - backward_vel)

        # calc. for angular velocity
        self.roverAngularVelocity = -self.maxAngularVelocity * np.tanh(steering)

        # assigns values to a Twist msg, then publish it to ROS
        roverTwist = Twist()
        roverTwist.linear.x = self.roverLinearVelocity
        roverTwist.angular.z = self.roverAngularVelocity

        self.processPub.publish(roverTwist)

if __name__ == "__main__":
    gamepadProcess = Node_GamepadProcessing(1, 5)
    rospy.spin()