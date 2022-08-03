#!/usr/bin/python3

# will need to publish a ScienceCmd msg to ROS
import rospy
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
from stepper_motor import StepperMotor
from science_module.msg import ScienceCmd, ScienceFeedback, SciencePilot

class ScienceSystem:
    def __init__(self):
        # Initialize ROS
        rospy.init_node("science_module_node")

        # default system attributes to true (i.e. on); based off ScienceFeedback.msg
        self.gripper_state = False
        self.led_state = False
        self.cooler_state = False
        self.laser_state = False
        self.ccd_snap = False
        self.shutdown = False

        # science feedback
        self.stepper1_fault = False
        self.stepper2_fault = False

        # stepper motor positions, speeds and modes
        dt = 0.001
        self.stepper1 = StepperMotor(dt)
        self.stepper2 = StepperMotor(dt)

        self.stepper1_pos = 0
        self.stepper2_pos = 0
        self.stepper1_vel = 0
        self.stepper2_vel = 0
        
        self.stepper1_control_mode = 1
        self.stepper2_control_mode = 1

        # cont. motor speed
        self.excavator_vel= 0

        # ROS publisher and subscribers from serial_node.py
        self.science_control_publisher = rospy.Publisher("science_control_data", ScienceCmd, queue_size=1)

        # two subscribers (Science Pilot, science feedback)
        self.science_state_subscriber = rospy.Subscriber("science_state_data", ScienceFeedback, self.feedback_callback)
        self.science_ui_subscriber = rospy.Subscriber("science_controller_input", SciencePilot, self.ui_callback)

    # Callback functions for ScienceFeedback and SciencePilot messages ^^^
    def feedback_callback(self, msg):
        self.stepper1_fault = msg.Stepper1Fault
        self.stepper2_fault = msg.Stepper2Fault

    def ui_callback(self, msg):
        # bool states
        self.gripper_state = msg.GripperState
        self.led_state = msg.LedState
        self.cooler_state = msg.PeltierState
        self.laser_state = msg.LaserState
        self.ccd_snap = msg.CcdSensorSnap
        self.shutdown = msg.Shutdown

        # stepper motor and cont. motor 
        self.stepper1_pos = msg.StepperMotor1Pos
        self.stepper2_pos = msg.StepperMotor2Pos
        self.stepper1_vel = msg.StepperMotor1Speed
        self.stepper2_vel = msg.StepperMotor2Speed

        # check for state changes
        if self.stepper1_control_mode != msg.Stepper1ControlMode:
            self.stepper1.set_control_mode(msg.Stepper1ControlMode)

            # check for rising edge and that requested modes are agitation
            if msg.Stepper1ControlMode == 3:
                self.stepper1.init_agitation()

        # check for state changes
        if self.stepper2_control_mode != msg.Stepper2ControlMode:
            self.stepper2.set_control_mode(msg.Stepper2ControlMode)

            # check for rising edge and that requested modes are agitation
            if msg.Stepper2ControlMode == 3:
                self.stepper2.init_agitation()

        self.stepper1_control_mode = msg.Stepper1ControlMode
        self.stepper2_control_mode = msg.Stepper2ControlMode

        if msg.ContMotorSpeed == 0:     #stop
            self.excavator_vel = 0
        elif msg.ContMotorSpeed == 1:   #up
            self.excavator_vel = 1
        elif msg.ContMotorSpeed == 2:   #down
            self.excavator_vel = -1
        else:
            print("Unknown excavator state")
            self.excavator_vel = 0

    # run the system; publish control data + subscribe to feedback data
    def run(self):
        # loop continuously
        # initialize angles of stepper motors
        angle_stepper1 = 0
        angle_stepper2 = 0

        while not rospy.is_shutdown():
            # Stepper 1
            # continuous mode
            if self.stepper1.mode == 1:
                angle_stepper1 = self.stepper1.continous_mode_control(self.stepper1_vel)

            # position mode
            elif self.stepper1.mode == 2:
                angle_stepper1 = self.stepper1.position_mode_control(self.stepper1_pos)
            
            # agitation mode
            elif self.stepper1.mode == 3:
                angle_stepper1 = self.stepper1.agitation_mode_control()
            
            # unsupported
            else:
                angle_stepper1 = 0

            # Stepper 2
            # continuous mode
            if self.stepper2.mode == 1:
                angle_stepper2 = self.stepper2.continous_mode_control(self.stepper2_vel)

            # position mode
            elif self.stepper2.mode == 2:
                angle_stepper2 = self.stepper2.position_mode_control(self.stepper2_pos)
            
            # agitation mode
            elif self.stepper2.mode == 3:
                angle_stepper2 = self.stepper2.agitation_mode_control()
            
            # unsupported
            else:
                angle_stepper2 = 0

            # create a new ScienceCmd msg and map with instance values
            newCmd = ScienceCmd()
            newCmd.GripperState = self.gripper_state
            newCmd.LedState = self.led_state 
            newCmd.PeltierState = self.cooler_state 
            newCmd.LaserState = self.laser_state 
            newCmd.CcdSensorSnap = self.ccd_snap 
            newCmd.Shutdown = self.shutdown
            newCmd.MotorSpeed = self.excavator_vel
            newCmd.Stepper1IncAng = angle_stepper1
            newCmd.Stepper2IncAng = angle_stepper2

            self.science_control_publisher.publish(newCmd)
            print(newCmd)
            print("========================================")
            rospy.sleep(0.1)

if __name__ == "__main__":
    scienceSystem = ScienceSystem()
    scienceSystem.run()
    rospy.spin() 