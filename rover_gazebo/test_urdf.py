
import pybullet as p
import math
import pybullet_data
import numpy as np

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.1])

robotId = p.loadURDF("rover_gazebo/urdf/rover.urdf", [0, 0, 0], useFixedBase=1)

p.resetBasePositionAndOrientation(robotId, [0, 0, 0], [0, 0, 0, 1])
while 1:
    p.stepSimulation()
p.disconnect()
