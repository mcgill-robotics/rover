import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import kinematics
import numpy as np

class InverseKinematicsController():

    def __init__(self, dt, home_state, max_vel) -> None:
        self.goal = None
        self.dt = dt 
        self.home_state = home_state
        self.max_vel = max_vel

    def set_goal(self, goal):
        self.goal = goal

    def get_control(self, state):
        if self.goal is None:
            dq = np.zeros(state.shape[0])
        else:
            robot_pose = kinematics.Mat2Pose(kinematics.forwardKinematics(state))
            dx = (self.goal - robot_pose)/self.dt
            J, Jv, Jw = kinematics.Jacobian(state)
            invJ = np.linalg.pinv(J)

            N = np.eye(state.shape[0]) - invJ @ J
            dq = invJ @ dx + N @ (self.home_state - state)

        return np.clip(dq, -self.max_vel, self.max_vel)
