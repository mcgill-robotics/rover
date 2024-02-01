import os, sys
from enum import Enum

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)

# TODO: Figure out why catkin on the Jetson isn't playing nice with this import. It worked on my PC -Eren
# import init_functions
import rospy
from odrive.enums import AxisState, ProcedureResult
from odrive.utils import dump_errors
from std_msgs.msg import Float32MultiArray
from ODrive_Joint import *
from odrive_interface_arm.msg import MotorState, MotorError


# TODO
class Node_outshaft:
    def __init__(self):
        pass


if __name__ == "__main__":
    driver = Node_outshaft()
    rospy.spin()
