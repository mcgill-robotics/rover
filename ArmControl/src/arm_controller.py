#!/usr/bin/env python3

import arm_kinematics
import numpy as np
import rospy
from arm_control.msg import ProcessedControllerInput, ArmMotorCommand
from arm_control.msg import ArmStatusFeedback

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
        self.ee   = 0                   # End-effector (EE) : active/inactive

        # Desired Arm State
        self.q_d  = [0] * self.nbJoints
        self.dq_d = [0] * self.nbJoints
        self.x_d  = [0] * self.nbCart
        self.dx_d = [0] * self.nbCart
        self.ee_d = 0

        # Control mode
        self.mode = 'Cartesian'     # default cartesian mode
        # Options: Cartesian, Joint_N where N is the controlled joint
        # (one joint at a time to keep it intuitive to the user)

        # Physical Constraints
        self.jointUpperLimits = [np.pi, np.pi, np.pi, np.pi, np.pi]      # rad
        self.jointLowerLimits = [-np.pi, -np.pi, -np.pi, -np.pi, -np.pi] # rad

        self.jointVelLimits = [np.pi, np.pi, np.pi, np.pi, np.pi]   # rad/s
        self.cartVelLimits = [1, 1, 1]   # m/s 

        # Initialize ROS
        rospy.init_node("arm_control", anonymous=False)
        self.armControlPublisher = rospy.Publisher("arm_control_data", ArmMotorCommand, queue_size=1)
        self.uiSubscriber        = rospy.Subscriber("processed_arm_controller_input", ProcessedControllerInput, self.controlLoop)
        self.armStateSubscriber  = rospy.Subscriber("arm_state_data", ArmStatusFeedback, self.updateArmState)

        self.run()


    def run(self):
        cmds = ArmMotorCommand()
        while not rospy.is_shutdown():
            cmds.MotorVel = self.dq_d
            cmds.ClawState = int(self.ee_d)

            self.armControlPublisher.publish(cmds)


    def controlLoop(self, ctrlInput):
        
        if ctrlInput.ModeChange:
            self.changeControlMode()

        if self.mode == 'Cartesian':
            xyz_ctrl = [
                ctrlInput.X_dir,
                ctrlInput.Y_dir,
                ctrlInput.Z_dir
            ]
            self.dq_d, self.dx_d = self.computeTargetJointVel(xyz_ctrl)

        else:
            try:
                i = int(self.mode[-1]) - 1
            except ValueError as error:
                rospy.logerr(str(error) + "- Changing to Cartesian Mode")
                self.mode == 'Cartesian'
            
            self.dq_d[i] = ctrlInput.Y_dir * self.jointVelLimits[i]
        
        try:
            self.ee_d = int(ctrlInput.ClawOpen)
        except ValueError as error:
                rospy.logerr(str(error))


    def updateArmState(self, state):
        self.q      = state.MotorPos
        self.dq     = state.MotorVel
        self.torq   = state.MotorTorq
        self.ee     = int(state.ClawMode)

        self.x = arm_kinematics.forwardKinematics(self.q)

        J, Jv, Jw = arm_kinematics.Jacobian(self.q)
        self.dx = arm_kinematics.forwardVelocity(self.q, self.dq)


    def changeControlMode(self):
        if self.mode == 'Cartesian':
            self.mode = 'Joint_1'
        elif self.mode == 'Joint_5':
            self.mode = 'Cartesian'
        else:
            try:
                i = int(self.mode[-1])
                self.mode = f'Joint_{i+1}'
            except ValueError:
                self.mode = 'Joint_1'
        
        return self.mode


    def computeTargetJointVel(self, ctrlVel):
        v_x = ctrlVel[0] * self.cartVelLimits[0]
        v_y = ctrlVel[1] * self.cartVelLimits[1]
        v_z = ctrlVel[2] * self.cartVelLimits[2]

        self.dx_d = [v_x, v_y, v_z]
        try:
            self.dq_d = arm_kinematics.inverseVelocity(self.q, self.dx_d)
        except ValueError:
            # Leave JointVels as is
            pass

        return self.dq_d, self.dx_d


if __name__ == "__main__":
    driver = Node_ArmControl()
    rospy.spin()
