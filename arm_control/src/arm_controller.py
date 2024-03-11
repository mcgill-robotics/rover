import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import arm_kinematics
import numpy as np
import rospy
import math
from arm_control.msg import ArmControllerInput
from std_msgs.msg import Float32MultiArray
from arm_kinematics import jointLowerLimits, jointUpperLimits
from std_msgs.msg import String


class Node_ArmControl:

    def __init__(self):
        self.nbJoints = 5
        self.nbJointsArm = 5
        self.nbCart = 3

        # Actual Arm State
        self.q = [0] * self.nbJoints
        self.dq = [0] * self.nbJoints
        self.torq = [0] * self.nbJoints
        self.x = [0] * self.nbCart
        self.dx = [0] * self.nbCart

        # Desired Arm State
        self.q_d = [0] * self.nbJoints
        self.dq_d = [0] * self.nbJoints
        self.x_d = [0] * self.nbCart
        self.dx_d = [0] * self.nbCart

        # Control mode
        self.mode = 0  # default cartesian mode
        # Options: [0, nbJoints), In joint control mode, control is
        # one joint at a time to keep it intuitive to the user

        self.claw_state = 0

        self.jointVelLimits = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]  # rad/s
        self.cartVelLimits = [0.5, 0.5, 0.5]  # m/s

        # Initialize ROS
        rospy.init_node("arm_control", anonymous=False)
        self.controllerSubscriber = rospy.Subscriber(
            "arm_controller_input", ArmControllerInput, self.controlLoop
        )

        # Arduino message
        self.armBrushedSubscriber = rospy.Subscriber(
            "armBrushedFb", Float32MultiArray, self.updateArmBrushedState
        )
        self.armBrushlessSubscriber = rospy.Subscriber(
            "armBrushlessFb", Float32MultiArray, self.updateArmBrushlessState
        )
        self.armBrushedPublisher = rospy.Publisher(
            "armBrushedCmd", Float32MultiArray, queue_size=10
        )
        self.armBrushlessPublisher = rospy.Publisher(
            "armBrushlessCmd", Float32MultiArray, queue_size=10
        )

        self.arm_error_publisher = rospy.Publisher("armError", String, queue_size=10)

        # Control Frequency of the arm controller
        self.rate = rospy.Rate(100)

        self.run()

    def run(self):
        cmd_brushed = Float32MultiArray()
        cmd_brushless = Float32MultiArray()
        while not rospy.is_shutdown():
            q_dDeg = tuple(q_d_i * (180 / np.pi) for q_d_i in self.q_d)

            cmd_brushed.data = [self.claw_state, q_dDeg[4], q_dDeg[3]]
            cmd_brushless.data = [q_dDeg[2], q_dDeg[1], q_dDeg[0]]

            self.armBrushedPublisher.publish(cmd_brushed)
            self.armBrushlessPublisher.publish(cmd_brushless)

            self.rate.sleep()

    def controlLoop(self, ctrlInput: ArmControllerInput):

        if self.mode != ctrlInput.Mode:
            self.mode = ctrlInput.Mode
            print(f"Changed to mode {self.mode}")

        # Cartesian Velocity Control
        if self.mode == 0:
            xyz_ctrl = [ctrlInput.X_dir, 0, ctrlInput.Y_dir]
            self.dq_d, self.dx_d = self.computePoseJointVel(xyz_ctrl)
            self.dq_d[0] = ctrlInput.Z_dir * -1 * self.jointVelLimits[0]

        elif self.mode == 1:
            self.dq_d[0] = ctrlInput.Z_dir * -1 * self.jointVelLimits[0]

        elif self.mode == 3:
            self.dq_d[2] = ctrlInput.X_dir * 1 * self.jointVelLimits[2]

        elif self.mode == 4:

            if abs(ctrlInput.X_dir) > 0.35:
                X_dir_filt = ctrlInput.X_dir / 0.8
            else:
                X_dir_filt = 0

            if abs(ctrlInput.Z_dir) > 0.35:
                Z_dir_filt = ctrlInput.Z_dir / 0.8
            else:
                Z_dir_filt = 0

            if X_dir_filt >= 0:
                X_dir_filt = min(1, X_dir_filt)
            else:
                X_dir_filt = max(-1, X_dir_filt)

            if Z_dir_filt >= 0:
                Z_dir_filt = min(1, Z_dir_filt)
            else:
                Z_dir_filt = max(-1, Z_dir_filt)

            self.dq_d[3] = X_dir_filt * self.jointVelLimits[3]
            self.dq_d[4] = Z_dir_filt * self.jointVelLimits[4]

        # Control of EOAT
        elif self.mode == 5:
            self.claw_state = ctrlInput.X_dir * 99.9999999

        else:
            # Joint Velocity Control
            i = self.mode - 1
            self.dq_d[i] = ctrlInput.X_dir * self.jointVelLimits[i]

        # Check Joint Limits of arm joints
        for i in range(self.nbJointsArm):
            if (self.q[i] > jointUpperLimits[i] and self.dq_d[i] > 0) or (
                self.q[i] < jointLowerLimits[i] and self.dq_d[i] < 0
            ):
                self.displayError(
                    f"Joint {i} reached the limit: {self.q[i]} {self.dq_d[i]}"
                )
                if self.mode == 0:
                    self.dq_d = [0] * self.nbJoints

                else:
                    self.dq_d[i] = 0

            if (self.q_d[i] > jointUpperLimits[i] and self.dq_d[i] > 0) or (
                self.q_d[i] < jointLowerLimits[i] and self.dq_d[i] < 0
            ):
                self.displayError(
                    f"Joint {i} reached the second limit: {self.q_d[i]} {self.dq_d[i]}"
                )
                if self.mode == 0:
                    self.dq_d = [0] * self.nbJoints

                else:
                    self.dq_d[i] = 0

            self.q_d[i] += self.dq_d[i] * 0.01 * ctrlInput.MaxVelPercentage

    def updateArmBrushedState(self, state_brushed: Float32MultiArray):
        state12_r = tuple(data_i * (np.pi / 180) for data_i in state_brushed.data)
        self.q[4] = state12_r[0]  # Wrist Roll, disk
        self.q[3] = state12_r[1]  # Wrist Pitch
        self.x = arm_kinematics.forwardKinematics(self.q)

    def updateArmBrushlessState(self, state_brushless: Float32MultiArray):
        state_brushless_r = tuple(
            data_i * (np.pi / 180) for data_i in state_brushless.data
        )
        self.q[2] = state_brushless_r[0]  # Elbow
        self.q[1] = state_brushless_r[1]  # Shoulder
        self.q[0] = state_brushless_r[2]  # Tumor
        self.x = arm_kinematics.forwardKinematics(self.q)

    def computePoseJointVel(self, ctrlVel):
        v_x = ctrlVel[0] * self.cartVelLimits[0]
        v_y = ctrlVel[1] * self.cartVelLimits[1]
        v_z = ctrlVel[2] * self.cartVelLimits[2]

        self.dx_d = [v_x, v_y, v_z]
        q_t = list(self.q)
        q_t[0] = 0
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

    def displayError(self, msg: str):
        print(msg)
        self.arm_error_publisher.publish(msg)


if __name__ == "__main__":
    driver = Node_ArmControl()
    rospy.spin()
