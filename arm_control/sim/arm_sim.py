#!/usr/bin/env python3

import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import sys
import os
sys.path.append("..") 
from arm_control.msg import ArmMotorCommand, ArmStatusFeedback
import numpy as np
import rospy

class Node_ArmSim():

  def __init__(self):
    p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf", [0, 0, -0.1])

    urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../model/MR_arm.urdf"
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
    self.jointVels  = [0]*self.numJoints
    self.jointTorq  = [0]*self.numJoints

    self.desiredJointPos = [0]*self.numJoints

    rospy.init_node("arm_sim", anonymous=False)
    self.armCmdSubscriber = rospy.Subscriber("arm_control_data", ArmMotorCommand, self.updateSim)
    self.armStatePublisher = rospy.Publisher("arm_state_data", ArmStatusFeedback, queue_size=1)

    self.run()

  def updateSim(self, cmds):
    self.desiredJointPos = cmds.MotorPos

  
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

      if(self.hasPrevPose):
        p.addUserDebugLine(self.prevPose, ls[4], [1, 0, 0], 1, self.trailDuration)
      self.prevPose = ls[4]
      self.hasPrevPose = 1

      states = p.getJointStates(self.robotId, range(self.numJoints))
      for i in range(len(states)):
        self.jointPoses[i] = states[i][0]
        self.jointVels[i]  = states[i][1]
        self.jointTorq[i]  = states[i][3]

      state_msg = ArmStatusFeedback()
      state_msg.MotorPos = self.jointPoses
      state_msg.MotorVel = self.jointVels
      state_msg.MotorTorq = self.jointTorq
      state_msg.ClawMode = False

      self.armStatePublisher.publish(state_msg)

    p.disconnect()


if __name__ == "__main__":
  driver = Node_ArmSim()
  rospy.spin()