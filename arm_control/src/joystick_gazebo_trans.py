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


