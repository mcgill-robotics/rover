#!/usr/bin/env python3

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import arm_kinematics 
import numpy as np
import time
import rospy
from arm_control.msg import ProcessedControllerInput, ArmMotorCommand, ArmStatusFeedback

class Node_ArmControl():

    def __init__(self):
        
        self.nbJoints = 5
        self.nbCart   = 3
        
        # Actual Arm State
        self.q    = [0] * self.nbJoints
        self.dq   = [0] * self.nbJoints
        self.torq = [0] * self.nbJoints
        self.x    = [0] * self.nbCart
        self.dx   = [0] * self.nbCart
        self.ee   = False                   # End-effector (EE) : active/inactive

        # Desired Arm State
        self.q_d  = [0] * self.nbJoints
        self.dq_d = [0] * self.nbJoints
        self.x_d  = [0] * self.nbCart
        self.dx_d = [0] * self.nbCart
        self.ee_d = False

        # Control mode
        self.mode = 0     # default cartesian mode
        # Options: [0, nbJoints), In joint control mode, control is
        # one joint at a time to keep it intuitive to the user

        # Physical Constraints
        self.jointUpperLimits = [175*np.pi/180, 90*np.pi/180, 75*np.pi/180, 75*np.pi/180, np.pi]      # rad
        self.jointLowerLimits = [-175*np.pi/180, -60*np.pi/180, -70*np.pi/180, -75*np.pi/180, -np.pi] # rad

        self.jointVelLimits = [np.pi, np.pi, np.pi, np.pi, np.pi]   # rad/s
        self.cartVelLimits = [0.5, 0.5, 0.5]   # m/s 

        # Initialize ROS
        rospy.init_node("arm_control", anonymous=False)
        self.armControlPublisher = rospy.Publisher("arm_control_data", ArmMotorCommand, queue_size=1)
        self.uiSubscriber        = rospy.Subscriber("arm_controller_input", ProcessedControllerInput, self.controlLoop)
        self.armStateSubscriber  = rospy.Subscriber("arm_state_data", ArmStatusFeedback, self.updateArmState)

        self.run()


    def run(self):
        cmds = ArmMotorCommand()
        while not rospy.is_shutdown():
            cmds.MotorVel = self.dq_d
            cmds.ClawState = self.ee_d

            self.armControlPublisher.publish(cmds)

            time.sleep(0.01)


    def controlLoop(self, ctrlInput):
        
        if ctrlInput.ModeChange:
            self.changeControlMode()

        # Cartesian Velocity Control
        if self.mode == 0:
            xyz_ctrl = [
                ctrlInput.X_dir,
                ctrlInput.Y_dir,
                ctrlInput.Z_dir
            ]
            self.dq_d, self.dx_d = self.computePoseJointVel(xyz_ctrl)

        elif self.mode == 1:
            xyz_ctrl = [
                0,
                ctrlInput.Y_dir,
                0
            ]

            self.dq_d, self.dx_d = self.computeOrientationJointVel(xyz_ctrl)

        else:
            # Joint Velocity Control
            i = self.mode - 2
            self.dq_d[i] = ctrlInput.Y_dir * self.jointVelLimits[i]
        
        self.ee_d = ctrlInput.ClawOpen

        # Check Joint Limits
        for i in range(len(self.q)):
            if(
                self.q[i] > self.jointUpperLimits[i] and self.dq_d[i] > 0 
                or
                self.q[i] < self.jointLowerLimits[i] and self.dq_d[i] < 0
            ):
                if self.mode == 0:
                    self.dq_d = [0] * self.nbJoints

                elif self.mode == 1:
                    self.dq_d = [0] * self.nbJoints

                else:
                    self.dq_d[i] = 0
                

    def updateArmState(self, state):
        self.q      = state.MotorPos
        # self.dq     = state.MotorVel
        # self.torq   = state.MotorTorq
        self.ee     = state.ClawMode

        self.x = arm_kinematics.forwardKinematics(self.q)

        # J, Jv, Jw = arm_kinematics.Jacobian(self.q)
        # self.dx = arm_kinematics.forwardVelocity(self.q, self.dq)


    def changeControlMode(self):
        self.mode += 1
        self.mode = self.mode % (self.nbJoints + 2)
        
        return self.mode


    def computePoseJointVel(self, ctrlVel):
        v_x = ctrlVel[0] * self.cartVelLimits[0]
        v_y = ctrlVel[1] * self.cartVelLimits[1]
        v_z = ctrlVel[2] * self.cartVelLimits[2]

        self.dx_d = [v_x, v_y, v_z]
        try:
            self.dq_d = arm_kinematics.inverseVelocity(self.q, self.dx_d)
        except ValueError:
            # Leave JointVels as is
            pass

        max_ratio = 1
        for i in range(len(self.dq_d)):
            ratio = abs(self.dq_d[i] / self.jointVelLimits[i])
            if ratio > max_ratio:
                max_ratio = ratio
            
        if max_ratio != 1:
            self.dq_d[i] = self.dq_d[i] / max_ratio

        return self.dq_d, self.dx_d

    def computeOrientationJointVel(self, ctrlVel):
        w_x = ctrlVel[0] * self.cartVelLimits[0]
        w_y = ctrlVel[1] * self.cartVelLimits[1]
        w_z = ctrlVel[2] * self.cartVelLimits[2]

        self.dx_d = [0, 0, 0, w_x, w_y, w_z]
        try:
            self.dq_d = arm_kinematics.inverseVelocity(self.q, self.dx_d)
        except ValueError:
            # Leave JointVels as is
            pass

        max_ratio = 1
        for i in range(len(self.dq_d)):
            ratio = abs(self.dq_d[i] / self.jointVelLimits[i])
            if ratio > max_ratio:
                max_ratio = ratio
            
        if max_ratio != 1:
            self.dq_d[i] = self.dq_d[i] / max_ratio

        return self.dq_d, self.dx_d


if __name__ == "__main__":
    driver = Node_ArmControl()
    rospy.spin()
