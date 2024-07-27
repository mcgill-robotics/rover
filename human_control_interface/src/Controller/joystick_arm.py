#!/usr/bin/env python3

import pygame
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import sys


class Joint:
    def __init__(self, name, button, direction, axis=1):
        self.name = name
        self.button = button
        self.direction = direction
        self.axis = axis
        self.position = 0.0


class JoystickController:
    def __init__(self):
        self.base_angle_increment = 1e-2  # Base increment/decrement step size
        self.min_speed_multiplier = 0.2
        self.max_speed_multiplier = 5
        self.angle_increment = self.base_angle_increment
        self.axis_threshold = 0.2  # Threshold for joystick axis movement

        # Initialize joints
        self.joints = [
            Joint("joint_elbow", button=4, direction=-1),
            Joint("joint_shoulder", button=0, direction=-1),
            Joint("joint_waist", button=1, direction=-1, axis=2),
            Joint("joint_end_effector", button=3, direction=-1),
            Joint("joint_wrist_roll", button=5, direction=1, axis=0),
            Joint("joint_wrist_pitch", button=2, direction=1),
            Joint("joint_7", button=6, direction=1),
        ]

        # Initialize ROS node
        rospy.init_node("joystick_controller", anonymous=True)
        self.brushed_pub = rospy.Publisher(
            "/armBrushedCmd", Float32MultiArray, queue_size=10
        )
        self.brushless_pub = rospy.Publisher(
            "/armBrushlessCmd", Float32MultiArray, queue_size=10
        )

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()

        self.joystick = self.init_joystick()
        if self.joystick is None:
            rospy.logerr("No joystick found. Exiting.")
            sys.exit(1)

        rospy.on_shutdown(self.shutdown_hook)

    def map_range(self, value, in_min, in_max, out_min, out_max):
        # Map a value from one range to another
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def init_joystick(self):
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("No joystick found.")
            return None
        else:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Joystick initialized: {joystick.get_name()}")
            return joystick

    def get_joystick_axis(self, axis_num):
        pygame.event.pump()  # Process event queue
        return self.joystick.get_axis(axis_num)

    def get_joint_by_name(self, name):
        for joint in self.joints:
            if joint.name == name:
                return joint
        return None

    def publish_joint_states(self):
        brushed_msg = Float32MultiArray()
        brushless_msg = Float32MultiArray()

        brushed_msg.data = [
            np.rad2deg(
                self.get_joint_by_name("joint_end_effector").position
            ),  # Joint 5
            np.rad2deg(self.get_joint_by_name("joint_wrist_roll").position),  # Joint 4
            np.rad2deg(self.get_joint_by_name("joint_wrist_pitch").position),  # Joint 6
        ]

        brushless_msg.data = [
            np.rad2deg(self.get_joint_by_name("joint_elbow").position),  # Joint 3
            np.rad2deg(self.get_joint_by_name("joint_shoulder").position),  # Joint 2
            np.rad2deg(self.get_joint_by_name("joint_waist").position),  # Joint 1
        ]

        self.brushed_pub.publish(brushed_msg)
        self.brushless_pub.publish(brushless_msg)

        # Combine all joint states into a single line for print
        joint_states = [
            f"{joint.name}: {np.rad2deg(joint.position):.2f} degrees"
            for joint in self.joints
        ]
        print("\r" + " | ".join(joint_states), end="")

    def run(self):
        rate = rospy.Rate(20)  # 20 Hz

        while not rospy.is_shutdown():
            # Check for speed adjustment using axis 3
            speed_control_value = self.get_joystick_axis(3)
            # Map speed_control_value from [-1, 1] to [min_speed_multiplier, max_speed_multiplier]
            speed_multiplier = self.map_range(
                speed_control_value,
                -1,
                1,
                self.max_speed_multiplier,
                self.min_speed_multiplier,
            )
            self.angle_increment = self.base_angle_increment * speed_multiplier

            for joint in self.joints:
                axis_value = self.get_joystick_axis(joint.axis)
                if abs(axis_value) > self.axis_threshold:
                    if self.joystick.get_button(joint.button):
                        # Adjust the joint angle based on the joystick axis value and direction
                        joint.position += (
                            self.angle_increment * axis_value * joint.direction
                        )
            # Check if button 11 is pressed to reset all brushed joints
            if self.joystick.get_button(11):
                self.get_joint_by_name("joint_end_effector").position = 0.0
                self.get_joint_by_name("joint_wrist_roll").position = 0.0
                self.get_joint_by_name("joint_wrist_pitch").position = 0.0

            # Check if button 10 is pressed to reset all brushless joints
            if self.joystick.get_button(10):
                self.get_joint_by_name("joint_elbow").position = 0.0
                self.get_joint_by_name("joint_shoulder").position = 0.0
                self.get_joint_by_name("joint_waist").position = 0.0

            self.publish_joint_states()
            rate.sleep()

        self.shutdown_hook()

    def shutdown_hook(self):
        pygame.quit()
        print("Shutting down joystick controller.")


if __name__ == "__main__":
    try:
        joystick_controller = JoystickController()
        joystick_controller.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down due to KeyboardInterrupt")
        rospy.signal_shutdown("KeyboardInterrupt")
