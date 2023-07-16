import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import arm_kinematics 
import numpy as np
import time
import rospy
import copy
import math
from arm_control.msg import ArmControllerInput, ArmMotorCommand, ArmStatusFeedback
from std_msgs.msg import Float32MultiArray

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
        

        # Desired Arm State
        self.q_d  = [0] * self.nbJoints
        self.dq_d = [0] * self.nbJoints
        self.x_d  = [0] * self.nbCart
        self.dx_d = [0] * self.nbCart
        

        # print length instead
        # print(f"{len(self.q)=} {len(self.dq)=} {len(self.dq_d)=} {len(self.torq)=} {len(self.x)=} {len(self.dx)=}")

        # Control mode
        self.mode = 0     # default cartesian mode
        # Options: [0, nbJoints), In joint control mode, control is
        # one joint at a time to keep it intuitive to the user

        # Physical Constraints
        self.jointUpperLimits = [175*np.pi/180, 960*np.pi/180, 75*np.pi/180, 75*np.pi/180, np.pi, 0.125, 0.125]      # rad  (3.05, 1.57, 1.309, 1.309)
        self.jointLowerLimits = [-175*np.pi/180, -0*np.pi/180, -70*np.pi/180, -75*np.pi/180, -np.pi, -0.55, -0.55] # rad  (3.05, 1.047, 1.22, 1.309)

        self.jointVelLimits = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]   # rad/s
        self.cartVelLimits = [0.5, 0.5, 0.5]   # m/s 

        # Initialize ROS
        rospy.init_node("arm_control", anonymous=False)
        self.armControlPublisher = rospy.Publisher("arm_control_data", ArmMotorCommand, queue_size=1)
        self.uiSubscriber        = rospy.Subscriber("arm_controller_input", ArmControllerInput, self.controlLoop)
        self.armStateSubscriber  = rospy.Subscriber("arm_state_data", ArmStatusFeedback, self.updateArmState)

        # Arduino message 
        self.arm12Subscriber = rospy.Subscriber("arm12FB", Float32MultiArray, self.updateArm12State)
        self.arm24Subscriber = rospy.Subscriber("arm24FB", Float32MultiArray, self.updateArm24State)
        self.arm12Publisher = rospy.Publisher("arm12Cmd", Float32MultiArray, queue_size=10)
        self.arm24Publisher = rospy.Publisher("arm24Cmd", Float32MultiArray, queue_size=10)

        # Control Frequency of the arm controller
        self.rate = rospy.Rate(100)

        self.run()


    def run(self):
        cmds = ArmMotorCommand()
        cmd12 = Float32MultiArray()
        cmd24 = Float32MultiArray()
        while not rospy.is_shutdown():
            cmds.MotorPos = self.q_d 
            q_dDeg=[]
            for i in range(len(self.q_d)):
                q_dDeg.append(self.q_d[i]/np.pi*180)
            cmd12.data.append(q_dDeg[5]*2)
            cmd12.data.append(q_dDeg[4])
            cmd12.data.append(q_dDeg[3])

            cmd24.data.append(q_dDeg[2])
            cmd24.data.append(q_dDeg[1])
            cmd24.data.append(q_dDeg[0])

            self.arm12Publisher.publish(cmd12)
            self.arm24Publisher.publish(cmd24)
            self.armControlPublisher.publish(cmds)


            self.rate.sleep()


    def controlLoop(self, ctrlInput):
        
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
            self.dq_d[0] = ctrlInput.Z_dir * -1 *self.jointVelLimits[0]
        

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

        elif(self.mode == 4):
            self.dq_d[3] = ctrlInput.X_dir * self.jointVelLimits[3]
            self.dq_d[4] = ctrlInput.Z_dir * self.jointVelLimits[4]
        
        # Control of EOAT
        elif (self.mode == 5):
            self.q_d[5] += ctrlInput.X_dir * self.jointVelLimits[5] * 0.01



            if (self.q_d[5] > self.jointUpperLimits[5]) or (self.q_d[5] < self.jointLowerLimits[5]):
                self.q_d[5] = self.jointUpperLimits[5] if self.q_d[5] > self.jointUpperLimits[5] else self.jointLowerLimits[5]

            self.q_d[6] += ctrlInput.X_dir * self.jointVelLimits[6] * 0.01

            # print(self.q_d[5], self.q_d[6])

            if (self.q_d[6] > self.jointUpperLimits[6]) or (self.q_d[6] < self.jointLowerLimits[6]):
                self.q_d[6] = self.jointUpperLimits[6] if self.q_d[6] > self.jointUpperLimits[6] else self.jointLowerLimits[6]

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

                else:
                    self.dq_d[i] = 0

            if ((self.q_d[i] > self.jointUpperLimits[i] and self.dq_d[i] > self.jointUpperLimits[i]) 
                or
                (self.q_d[i] < self.jointLowerLimits[i] and self.dq_d[i] < self.jointLowerLimits[i])
            ):
                print("reach the limit 2")
                if self.mode == 0:
                    self.dq_d = [0] * self.nbJoints

                else:
                    self.dq_d[i] = 0
        
            self.q_d[i] += self.dq_d[i] * 0.01* ctrlInput.velocity

    # simulation 
    def updateArmState(self, state):
        self.q      = state.MotorPos
        # self.dq     = state.MotorVel
        # self.torq   = state.MotorTorq
        # print(round(state.MotorPos[5], 3), round(state.MotorPos[6], 3))
        self.x = arm_kinematics.forwardKinematics(self.q)

        # J, Jv, Jw = arm_kinematics.Jacobian(self.q)
        # self.dx = arm_kinematics.forwardVelocity(self.q, self.dq)
    
    def updateArm12State(self, state12):
        state12_r=[]
        for i in range(len(state12.data)):
            state12_r.append(state12.data[i]/180* np.pi)
        self.q[5] = state12_r[0]/2              #EOAT
        self.q[6] = state12_r[0]/2              #EOAT
        self.q[4] = state12_r[1]                #Wrist Roll, disk
        self.q[3] = state12_r[2]                #Wrist Pitch
        self.x = arm_kinematics.forwardKinematics(self.q)


    def updateArm24State(self, state24):
        state24_r=[]
        for i in range(len(state24.data)):
           state24_r.append(state24.data[i]/180* np.pi)
        self.q[2] = state24_r[0]                #Elbow
        self.q[1] = state24_r[1]                #Shoulder
        self.q[0] = state24_r[2]                #Tumor
        self.x = arm_kinematics.forwardKinematics(self.q)


    def computePoseJointVel(self, ctrlVel):
        v_x = ctrlVel[0] * self.cartVelLimits[0]
        v_y = ctrlVel[1] * self.cartVelLimits[1]
        v_z = ctrlVel[2] * self.cartVelLimits[2]

        self.dx_d = [v_x, v_y, v_z]
        q_t = list(self.q)
        q_t[0] =0
        try:
            self.dq_d = arm_kinematics.inverseVelocity(q_t, self.dx_d)
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

