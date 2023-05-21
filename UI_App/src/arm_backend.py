#!/usr/bin/env python
# Imports
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
sys.path.append(currentdir + "/../..")
from arm_layout import Ui_Arm
from arm_control.msg import ArmStatusFeedback
from arm_control.src import arm_kinematics


class Arm_Backend():

    def __init__(self, arm_tab):
        self.ui = arm_tab
        self.ui.error_label.clear()

    def update_joints(self, motor_state):
        self.ui.joint_one_value.display(motor_state.MotorPos[0])
        self.ui.joint_two_value.display(motor_state.MotorPos[1])
        self.ui.joint_three_value.display(motor_state.MotorPos[2])
        self.ui.joint_four_value.display(motor_state.MotorPos[3])
        self.ui.joint_five_value.display(motor_state.MotorPos[4])

        hand = arm_kinematics.forwardKinematics(motor_state.MotorPos)
        pose = arm_kinematics.Mat2Pose(hand)

        self.ui.hand_x_val.display(pose[0])
        self.ui.hand_y_val.display(pose[1])
        self.ui.hand_z_val.display(pose[2])
        self.ui.orientation_x_val.display(pose[3])
        self.ui.orientation_y_val.display(pose[4])
        self.ui.orientation_z_val.display(pose[5])

