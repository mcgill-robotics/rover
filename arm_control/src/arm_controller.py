import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import arm_kinematics 
import numpy as np
import time
import rospy
import math
from arm_control.msg import ArmControllerInput, ArmMotorCommand, ArmStatusFeedback

class Node_ArmControl():

    def __init__(self):
        
        self.nbJoints = 7
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

        # print length instead
        # print(f"{len(self.q)=} {len(self.dq)=} {len(self.dq_d)=} {len(self.torq)=} {len(self.x)=} {len(self.dx)=}")

        # Control mode
        self.mode = 0     # default cartesian mode
        # Options: [0, nbJoints), In joint control mode, control is
        # one joint at a time to keep it intuitive to the user

        # Physical Constraints
        self.jointUpperLimits = [175*np.pi/180, 960*np.pi/180, 75*np.pi/180, 75*np.pi/180, np.pi, np.pi, np.pi]      # rad  (3.05, 1.57, 1.309, 1.309)
        self.jointLowerLimits = [-175*np.pi/180, -0*np.pi/180, -70*np.pi/180, -75*np.pi/180, -np.pi, np.pi, np.pi] # rad  (3.05, 1.047, 1.22, 1.309)

        self.jointVelLimits = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]   # rad/s
        self.cartVelLimits = [0.5, 0.5, 0.5]   # m/s 

        # Initialize ROS
        rospy.init_node("arm_control", anonymous=False)
        self.armControlPublisher = rospy.Publisher("arm_control_data", ArmMotorCommand, queue_size=1)
        self.uiSubscriber        = rospy.Subscriber("arm_controller_input", ArmControllerInput, self.controlLoop)
        self.armStateSubscriber  = rospy.Subscriber("arm_state_data", ArmStatusFeedback, self.updateArmState)

        # Control Frequency of the arm controller
        self.rate = rospy.Rate(100)

        self.run()


    def run(self):
        cmds = ArmMotorCommand()
        while not rospy.is_shutdown():
            cmds.MotorPos = self.q_d 
            cmds.ClawState = self.ee_d

            self.armControlPublisher.publish(cmds)

            self.rate.sleep()


    def controlLoop(self, ctrlInput):
        
        # if ctrlInput.ModeChange:
        #     self.changeControlMode()
        #     print(self.mode)
        if self.mode != ctrlInput.Mode:
            self.mode = ctrlInput.Mode
            print(self.mode)

        #Cartesian Velocity Control

        if self.mode == 0:
            xyz_ctrl = [
                ctrlInput.X_dir,
                0,
                ctrlInput.Y_dir
            ]
            self.dq_d, self.dx_d = self.computePoseJointVel(xyz_ctrl)
            self.dq_d[0] = ctrlInput.Z_dir *-1* self.jointVelLimits[0]
        
        # if self.mode == 0:
        #     xyz_ctrl = [
        #         ctrlInput.X_dir,
        #         ctrlInput.Y_dir,
        #         ctrlInput.Z_dir
        #     ]
        #     self.dq_d, self.dx_d = self.computePoseJointVel(xyz_ctrl)

        # if self.mode == 0:
        #     xyz_ctrl = [
        #         ctrlInput.X_dir,
        #         ctrlInput.Y_dir,
        #         ctrlInput.Z_dir
        #     ]
        #     self.dq_d, self.dx_d = self.computeIntuitiveJointVel(xyz_ctrl)

        # if self.mode == 0:
        #     xyz_ctrl = [
        #         ctrlInput.X_dir,
        #         ctrlInput.Y_dir,
        #         ctrlInput.Z_dir,
        #     ]

        #     self.dq_d, self.dx_d = self.computeOrientationJointVel(xyz_ctrl)

        elif (self.mode ==1):
            self.dq_d[0] = ctrlInput.Z_dir *-1* self.jointVelLimits[0]

        elif(self.mode == 3):
            self.dq_d[2] = ctrlInput.X_dir *-1* self.jointVelLimits[2]

        else:
            # Joint Velocity Control
            i = self.mode - 1
            self.dq_d[i] = ctrlInput.X_dir * self.jointVelLimits[i]

        self.ee_d = ctrlInput.ClawOpen

        # Check Joint Limits
        for i in range(5):
            if(
                (self.q[i] > self.jointUpperLimits[i] and self.dq_d[i] > 0 )
                or
                (self.q[i] < self.jointLowerLimits[i] and self.dq_d[i] < 0 )
            ): 
                print("reach the limit")
                if self.mode == 0:
                    self.dq_d = [0] * self.nbJoints

                elif self.mode == 1:
                    self.dq_d = [0] * self.nbJoints

                else:
                    self.dq_d[i] = 0

            if ((self.q_d[i] > self.jointUpperLimits[i] and self.dq_d[i] > self.jointUpperLimits[i]) 
                or
                (self.q_d[i] < self.jointLowerLimits[i] and self.dq_d[i] < self.jointLowerLimits[i])
            ):
                print("reach the limit 2")
                if self.mode == 0:
                    self.dq_d = [0] * self.nbJoints

                elif self.mode == 1:
                    self.dq_d = [0] * self.nbJoints

                else:
                    self.dq_d[i] = 0
        
            self.q_d[i] += self.dq_d[i] * 0.01* ctrlInput.velocity


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
            print("inverse velocity error")
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

    def computeIntuitiveJointVel(self, ctrlVel):
        v_z = ctrlVel[2] * self.cartVelLimits[2]
        theta = self.q[0]
        v_x = ctrlVel[0] * math.cos(theta) * self.cartVelLimits[0]
        v_y = ctrlVel[0] * -math.sin(theta) * self.cartVelLimits[0]

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
        
        self.dq_d[0] = ctrlVel[1] * self.jointVelLimits[0]

        J, Jv, Jw = arm_kinematics.Jacobian(self.q)
        self.dx_d = Jv.dot(self.dq_d)

        return self.dq_d, self.dx_d 
    


if __name__ == "__main__":
    driver = Node_ArmControl()
    rospy.spin()

