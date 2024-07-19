#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from numpy import pi
import rospy
import numpy as np
import pybullet as p
import pybullet_data
import sys
import os

sys.path.append("..")


class Node_ArmVisualizer:
    def __init__(self):
        # Initialize PyBullet and load the environment
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", [0, 0, -0.1])

        urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../model/MR_arm.urdf"

        # Load the robot URDF twice
        self.robotIdCmd = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=1)
        self.robotIdFb = p.loadURDF(urdf_path, [0, 0, 0.5], useFixedBase=1)

        p.resetBasePositionAndOrientation(self.robotIdCmd, [0, 0, 0], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(self.robotIdFb, [0, 0, 0], [0, 0, 0, 1])

        self.numJoints = p.getNumJoints(self.robotIdCmd)
        self.endEffectorIndex = self.numJoints - 1

        # Change the color of the commanded robot to green
        for i in range(self.numJoints):
            p.changeVisualShape(
                self.robotIdCmd, i, rgbaColor=[0, 1, 0, 1]
            )  # Green color

        # Change the color of the feedback robot to red
        # for i in range(self.numJoints):
        #     p.changeVisualShape(self.robotIdFb, i, rgbaColor=[1, 0, 0, 1])  # Red color

        for i in range(self.numJoints):
            p.resetJointState(self.robotIdCmd, i, 0)
            p.resetJointState(self.robotIdFb, i, 0)

        # Disable collisions between all links of the commanded and feedback robots
        for j in range(self.numJoints):
            p.setCollisionFilterPair(self.robotIdCmd, self.robotIdFb, -1, j, 0)
            p.setCollisionFilterPair(self.robotIdCmd, self.robotIdFb, j, -1, 0)
            for k in range(self.numJoints):
                p.setCollisionFilterPair(self.robotIdCmd, self.robotIdFb, j, k, 0)

        p.setGravity(0, 0, -9.8)
        self.prevPoseCmd = [0, 0, 0]
        self.prevPoseFb = [0, 0, 0]
        self.hasPrevPoseCmd = 0
        self.hasPrevPoseFb = 0
        self.t = 0.0

        p.setRealTimeSimulation(0)
        self.trailDuration = 5

        self.jointPosesCmd = [0] * self.numJoints
        self.jointPosesFb = [0] * self.numJoints

        rospy.init_node("arm_visualizer", anonymous=False)

        # Subscribers for command and feedback positions
        self.armBrushedCmdSubscriber = rospy.Subscriber(
            "armBrushedCmd", Float32MultiArray, self.updateArmBrushedCmd
        )
        self.armBrushlessCmdSubscriber = rospy.Subscriber(
            "armBrushlessCmd", Float32MultiArray, self.updateArmBrushlessCmd
        )
        self.armBrushedFbSubscriber = rospy.Subscriber(
            "armBrushedFb", Float32MultiArray, self.updateArmBrushedFb
        )
        self.armBrushlessFbSubscriber = rospy.Subscriber(
            "armBrushlessFb", Float32MultiArray, self.updateArmBrushlessFb
        )

        self.run()

    def updateArmBrushedCmd(self, data: Float32MultiArray):
        # Process the commanded positions similar to Node_ArmSim
        print("Received brushed command data:", data.data)
        self.jointPosesCmd[4], self.jointPosesCmd[3] = tuple(
            x * (pi / 180) for x in data.data[1:]
        )
        self.jointPosesCmd[5] = np.clip(
            self.jointPosesCmd[5] + data.data[0], -0.3, 0.11
        )
        self.jointPosesCmd[6] = self.jointPosesCmd[5]
        print("Updated jointPosesCmd for brushed motors:", self.jointPosesCmd)

    def updateArmBrushlessCmd(self, data: Float32MultiArray):
        # Process the commanded positions similar to Node_ArmSim
        print("Received brushless command data:", data.data)
        self.jointPosesCmd[2], self.jointPosesCmd[1], self.jointPosesCmd[0] = tuple(
            x * (pi / 180) for x in data.data
        )
        print("Updated jointPosesCmd for brushless motors:", self.jointPosesCmd)

    def updateArmBrushedFb(self, data: Float32MultiArray):
        # Process the feedback positions similar to Node_ArmSim
        print("Received brushed feedback data:", data.data)
        self.jointPosesFb[4], self.jointPosesFb[3] = tuple(
            x * (pi / 180) for x in data.data[1:]
        )
        self.jointPosesFb[5] = data.data[0] * (pi / 180)
        self.jointPosesFb[6] = self.jointPosesFb[5]
        print("Updated jointPosesFb for brushed motors:", self.jointPosesFb)

    def updateArmBrushlessFb(self, data: Float32MultiArray):
        # Process the feedback positions similar to Node_ArmSim
        print("Received brushless feedback data:", data.data)
        self.jointPosesFb[2], self.jointPosesFb[1], self.jointPosesFb[0] = tuple(
            x * (pi / 180) for x in data.data
        )
        print("Updated jointPosesFb for brushless motors:", self.jointPosesFb)

    def run(self):
        while not rospy.is_shutdown():
            self.t = self.t + 0.01
            p.stepSimulation()

            # Apply commanded positions to the joints of the first robot
            for i in range(self.numJoints):
                p.setJointMotorControl2(
                    bodyIndex=self.robotIdCmd,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=self.jointPosesCmd[i],
                    force=500,
                    positionGain=0.2,  # Higher gain for faster response
                    velocityGain=1,
                )

            # Apply feedback positions to the joints of the second robot
            for i in range(self.numJoints):
                p.setJointMotorControl2(
                    bodyIndex=self.robotIdFb,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=self.jointPosesFb[i],
                    force=500,
                    positionGain=0.03,  # Normal gain for real movement
                    velocityGain=1,
                )

            # Get the end effector's link state and add a debug line to show the trajectory
            lsCmd = p.getLinkState(self.robotIdCmd, self.endEffectorIndex)
            if self.hasPrevPoseCmd:
                p.addUserDebugLine(
                    self.prevPoseCmd, lsCmd[4], [0, 1, 0], 1, self.trailDuration
                )
            self.prevPoseCmd = lsCmd[4]
            self.hasPrevPoseCmd = 1

            lsFb = p.getLinkState(self.robotIdFb, self.endEffectorIndex)
            if self.hasPrevPoseFb:
                p.addUserDebugLine(
                    self.prevPoseFb, lsFb[4], [1, 0, 0], 1, self.trailDuration
                )
            self.prevPoseFb = lsFb[4]
            self.hasPrevPoseFb = 1

        p.disconnect()


if __name__ == "__main__":
    visualizer = Node_ArmVisualizer()
    rospy.spin()
