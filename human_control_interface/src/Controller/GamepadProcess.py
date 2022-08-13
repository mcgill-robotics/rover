#!/usr/bin/env python3
import time
import rospy
from human_control_interface.msg import Gamepad_input
from arm_control.msg import ArmControllerInput
from science_module.msg import SciencePilot
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from drive_control.msg import WheelSpeed
import numpy as np

class Node_GamepadProcessing:
    def __init__(self, v_max, w_max):
        # initialize ROS node
        rospy.init_node("gamepad_process_node")

        # initialize variables for velocity
        self.roverLinearVelocity = 0
        self.roverAngularVelocity = 0

        # initialize variables for rover's max linear and angular velocities
        self.maxLinearVelocity = v_max
        self.maxAngularVelocity = w_max

        # Initialize variables for the rover arm
        self.prevB1 = 0
        self.prevB2 = 0
        self.modeState = False
        self.clawState = False

        # System Selection variables
        self.active_system = 0

        # initialize a subscriber for grabbing data from gamepad
        self.joystick_sub = rospy.Subscriber("gamepad_data", Gamepad_input, self.gamepadProcessCall)
        self.mode_sub = rospy.Subscriber("system_selection", Int16, self.systemSelection)
        # self.drive_publisher = rospy.Publisher("rover_velocity_controller/cmd_vel", Twist, queue_size=1)
        self.drive_publisher = rospy.Publisher("/wheel_velocity_cmd", WheelSpeed, queue_size=1)
        self.arm_publisher = rospy.Publisher("arm_controller_input", ArmControllerInput, queue_size=1)
        self.sci_publisher = rospy.Publisher("science_controller_input", SciencePilot, queue_size=1)

    def gamepadProcessCall(self, msg):
        if self.active_system == 1:
            self.driveProcessCall(msg)
        elif self.active_system == 0:
            self.armProcessCall(msg)
        elif self.active_system == 2:
            self.scienceProcessCall(msg)
        else:
            pass

    def systemSelection(self, msg):
        self.active_system = msg.data

def driveProcessCall(self, msg):
    # assign axis values
    # steering = msg.A1
    # lt = msg.A3
    # rt = msg.A6
    left_speed = msg.A2
    right_speed = msg.A5

    # # normalize to [0, 1] range
    # backward_vel = (lt + 1)/2
    # forward_vel = (rt + 1)/2        

    # # calc. for linear velocity
    # self.roverLinearVelocity = self.maxLinearVelocity * (forward_vel - backward_vel)

    # # calc. for angular velocity
    # self.roverAngularVelocity = -self.maxAngularVelocity * np.sign(steering) * steering**2

    # # assigns values to a Twist msg, then publish it to ROS
    # roverTwist = Twist()
    # roverTwist.linear.x = self.roverLinearVelocity
    # roverTwist.angular.z = self.roverAngularVelocity

    wheel_speed = WheelSpeed()
    wheel_speed.left[0] = left_speed
    wheel_speed.left[1] = left_speed
    wheel_speed.right[0] = right_speed
    wheel_speed.right[1] = right_speed
    

    time.sleep(0.1)
    self.drive_publisher.publish(wheel_speed)

    def armProcessCall(self, msg):
        arm_ctrl = ArmControllerInput()
        # Convert joysticks to X, Y, Z motion
        arm_ctrl.X_dir = msg.A2**2
        if msg.A2 < 0:
            arm_ctrl.X_dir = -1 * arm_ctrl.X_dir
        arm_ctrl.Y_dir = msg.A1**2
        if msg.A1 > 0:
            arm_ctrl.Y_dir = -1 * arm_ctrl.Y_dir
        arm_ctrl.Z_dir = msg.A5**2
        if msg.A5 < 0:
            arm_ctrl.Z_dir = -1 * arm_ctrl.Z_dir

        # Rising edge filter on buttons
        if self.risingEdge(msg.B1, self.prevB1):
            self.modeState = True
        else:
            self.modeState = False

        if self.risingEdge(msg.B2, self.prevB2):
            self.clawState = True
        else:
            self.clawState = False

        # Publish msg
        arm_ctrl.ModeChange = self.modeState
        arm_ctrl.ClawOpen   = self.clawState

        self.prevB1 = msg.B1
        self.prevB2 = msg.B2

        self.arm_publisher.publish(arm_ctrl)

    def scienceProcessCall(self, msg):
        sci_ctrl = SciencePilot()

        sci_ctrl.LedState = msg.B1
        sci_ctrl.LaserState = msg.B2
        sci_ctrl.GripperState = msg.B3
        sci_ctrl.PeltierState = msg.B4
        sci_ctrl.CcdSensorSnap = msg.B5
        sci_ctrl.Shutdown = msg.B6

        sci_ctrl.Stepper1ControlMode = 1
        sci_ctrl.StepperMotor1Speed = msg.A1 * abs(msg.A1)
        sci_ctrl.Stepper2ControlMode = 1
        sci_ctrl.StepperMotor2Speed = msg.A4 * abs(msg.A4)

        self.sci_publisher.publish(sci_ctrl)

    def risingEdge(self, prevSignal, nextSignal):
        if prevSignal < nextSignal:
            return True
        else: 
            return False

if __name__ == "__main__":
    gamepadProcess = Node_GamepadProcessing(1, 25)
    rospy.spin()