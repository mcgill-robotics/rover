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


class Node_ArmSim():

    def __init__(self):
        p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", [0, 0, -0.1])

        urdf_path = os.path.dirname(
            os.path.abspath(__file__)) + "/../model/MR_arm.urdf"
        self.robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=1)

        p.resetBasePositionAndOrientation(self.robotId, [0, 0, 0], [0, 0, 0, 1])
        self.numJoints = p.getNumJoints(self.robotId)
        self.endEffectorIndex = self.numJoints - 1

        for i in range(self.numJoints):
            p.resetJointState(self.robotId, i, 0)

        p.setGravity(0, 0, -9.8)
        self.prevPose = [0, 0, 0]
        self.hasPrevPose = 0
        self.t = 0.

        p.setRealTimeSimulation(0)
        self.trailDuration = 5

        self.jointPoses = [0]*self.numJoints
        self.jointVels = [0]*self.numJoints
        self.jointTorq = [0]*self.numJoints

        self.desiredJointPos = [0]*self.numJoints

        rospy.init_node("arm_sim", anonymous=False)

        self.arm12Subscriber = rospy.Subscriber(
            "arm12Cmd", Float32MultiArray, self.updateArm12Sim)
        self.arm24Subscriber = rospy.Subscriber(
            "arm24Cmd", Float32MultiArray, self.updateArm24Sim)
        self.arm12Publisher = rospy.Publisher(
            "arm12FB", Float32MultiArray, queue_size=10)
        self.arm24Publisher = rospy.Publisher(
            "arm24FB", Float32MultiArray, queue_size=10)

        self.run()


    def updateArm12Sim(self, cmds: Float32MultiArray):
        self.desiredJointPos[4], self.desiredJointPos[3] = tuple(x * (pi/180) for x in cmds.data[1:])
        self.desiredJointPos[5] = np.clip(self.desiredJointPos[5] + cmds.data[0], -0.3, 0.11)
        self.desiredJointPos[6] = self.desiredJointPos[5]


    def updateArm24Sim(self, cmds: Float32MultiArray):
        self.desiredJointPos[2], self.desiredJointPos[1], self.desiredJointPos[0] = tuple(x * (pi/180) for x in cmds.data)


    def run(self):
        while not rospy.is_shutdown():
            self.t = self.t + 0.01
            p.stepSimulation()

            for i in range(self.numJoints):

                p.setJointMotorControl2(bodyIndex=self.robotId,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=self.desiredJointPos[i],
                                        force=500,
                                        positionGain=0.03,
                                        velocityGain=1)

            ls = p.getLinkState(self.robotId, self.endEffectorIndex)

            if (self.hasPrevPose):
                p.addUserDebugLine(self.prevPose, ls[4], [
                                1, 0, 0], 1, self.trailDuration)
            self.prevPose = ls[4]
            self.hasPrevPose = 1

            states = p.getJointStates(self.robotId, range(self.numJoints))
            for i in range(len(states)):
                self.jointPoses[i] = states[i][0] * (180/pi)
                self.jointVels[i] = states[i][1]
                self.jointTorq[i] = states[i][3]

            state12_msg = Float32MultiArray()
            state12_msg.data = self.jointPoses[4], self.jointPoses[3]

            state24_msg = Float32MultiArray()
            state24_msg.data = self.jointPoses[2], self.jointPoses[1], self.jointPoses[0]

            self.arm12Publisher.publish(state12_msg)
            self.arm24Publisher.publish(state24_msg)

        p.disconnect()


if __name__ == "__main__":
  driver = Node_ArmSim()
  rospy.spin()
