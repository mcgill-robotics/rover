#!/usr/bin/env python
# Imports
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
sys.path.append(currentdir + "/../..")
from arm_control.src import arm_kinematics



class Arm_Backend():

    def __init__(self, arm_tab):
        self.ui = arm_tab
        self.ui.error_label.clear()
        self.state_brushed = [0,0,0]
        self.state_brushless = [0,0,0]
        self.stateAll =[]

    def update_joints_brushed(self, state_brushed):
        self.ui.joint_four_value.display(state_brushed.data[2]) #Wrist
        self.ui.joint_five_value.display(state_brushed.data[1]) #Dist
        self.ui.joint_EOAT_value.display(state_brushed.data[0]) #EOAT
        self.state_brushed = [state_brushed.data[2],state_brushed.data[1],state_brushed.data[0]]
        self.handPosition()

    def update_joints_brushless(self, state_brushless):
        self.ui.joint_one_value.display(state_brushless.data[2]) #Waist
        self.ui.joint_two_value.display(state_brushless.data[1]) #Shoulder
        self.ui.joint_three_value.display(state_brushless.data[0]) #Elbow
        self.state_brushless = [state_brushless.data[2], state_brushless.data[1],state_brushless.data[0]]
        self.handPosition()

    def handPosition(self):
        self.stateAll = self.state_brushless+self.state_brushed
        stateAll_r = tuple(data_i * (3.1415/180) for data_i in self.stateAll)
        hand = arm_kinematics.forwardKinematics(stateAll_r)
        pose = arm_kinematics.Mat2Pose(hand)
        self.ui.hand_x_val.display(pose[0])
        self.ui.hand_y_val.display(pose[1])
        self.ui.hand_z_val.display(pose[2])
        self.ui.orientation_x_val.display(pose[3])
        self.ui.orientation_y_val.display(pose[4])
        self.ui.orientation_z_val.display(pose[5])
        
    def update_control(self,ctrl_input):
        self.ui.joint_mode_value.display (ctrl_input.Mode)
        self.ui.joint_velocity_value.display (ctrl_input.MaxVelPercentage/2*100)

    def set_error(self, error):
        self.ui.error_label.setText(error.data)
        




