#!/usr/bin/python3

# will need to publish a ScienceCmd msg to ROS
import rospy
from stepper_motor import StepperMotor
from science_module.msg import ScienceCmd, ScienceFeedback, SciencePilot

class ScienceSystem:
    def __init__(self):
        # Initialize ROS
        rospy.init_node("science_module_node")

        # default system attributes to true (i.e. on); based off ScienceFeedback.msg
        self.GripperState = False
        self.LedState = False
        self.PeltierState = False
        self.LaserState = False
        self.CcdSensorSnap = False
        self.Shutdown = False

        # science feedback
        self.Stepper1Fault = False
        self.Stepper2Fault = False

        # stepper motor positions, speeds and modes
        dt = 0.01
        self.Stepper1 = StepperMotor(dt)
        self.Stepper2 = StepperMotor(dt)

        self.StepperMotor1Pos = 0
        self.StepperMotor2Pos = 0
        self.StepperMotor1Speed = 0
        self.StepperMotor2Speed = 0
        
        self.Stepper1ControlMode = 1
        self.Stepper2ControlMode = 1

        # cont. motor speed
        self.ContMotorSpeed = 0

        # ROS publisher and subscribers from serial_node.py
        self.science_control_publisher = rospy.Publisher("science_control_data", ScienceCmd)

        # two subscribers (Science Pilot, science feedback)
        self.science_state_subscriber = rospy.Subscriber("science_state_data", ScienceFeedback, self.feedbackCall)
        self.science_ui_subscriber = rospy.Subscriber("science_ui_data", SciencePilot, self.uiCall)

    # Callback functions for ScienceFeedback and SciencePilot messages ^^^
    def feedbackCall(self, msg):
        self.Stepper1Fault = msg.Stepper1Fault
        self.Stepper2Fault = msg.Stepper2Fault

    def uiCall(self, msg):
        # bool states
        self.GripperState = msg.GripperState
        self.LedState = msg.LedState
        self.PeltierState = msg.PeltierState
        self.LaserState = msg.LaserState
        self.CcdSensorSnap = msg.CcdSensorSnap
        self.Shutdown = msg.Shutdown

        # stepper motor and cont. motor 
        self.StepperMotor1Pos = msg.StepperMotor1Pos
        self.StepperMotor2Pos = msg.StepperMotor2Pos
        self.StepperMotor1Speed = msg.StepperMotor1Speed
        self.StepperMotor2Speed = msg.StepperMotor2Speed

        # check for state changes
        if self.Stepper1ControlMode != msg.Stepper1ControlMode:
            self.Stepper1.SetControlMode(msg.Stepper1ControlMode)

            # check for rising edge and that requested modes are agitation
            if msg.Stepper1ControlMode == 3:
                self.Stepper1.InitAgitation()

        # check for state changes
        if self.Stepper2ControlMode != msg.Stepper2ControlMode:
            self.Stepper2.SetControlMode(msg.Stepper2ControlMode)

            # check for rising edge and that requested modes are agitation
            if msg.Stepper2ControlMode == 3:
                self.Stepper2.InitAgitation()

        self.Stepper1ControlMode = msg.Stepper1ControlMode
        self.Stepper2ControlMode = msg.Stepper2ControlMode

        self.ContMotorSpeed = msg.ContMotorSpeed

    # run the system; publish control data + subscribe to feedback data
    def run(self):
        # loop continuously
        # initialize angles of stepper motors
        angle_stepper1 = 0
        angle_stepper2 = 0

        while not rospy.is_shutdown():
            # Stepper 1
            # continuous mode
            if self.Stepper1.mode == 1:
                angle_stepper1 = self.Stepper1.ContinousModeControl(self.StepperMotor1Speed)

            # position mode
            elif self.Stepper1.mode == 2:
                angle_stepper1 = self.Stepper1.PositionModeControl(self.StepperMotor1Pos)
            
            # agitation mode
            elif self.Stepper1.mode == 3:
                angle_stepper1 = self.Stepper1.AgitationModeControl(self.StepperMotor1Pos, self.StepperMotor1Speed)
            
            # unsupported
            else:
                angle_stepper1 = 0

            # Stepper 2
            # continuous mode
            if self.Stepper2.mode == 1:
                angle_stepper2 = self.Stepper2.ContinousModeControl(self.StepperMotor2Speed)

            # position mode
            elif self.Stepper2.mode == 2:
                angle_stepper2 = self.Stepper2.PositionModeControl(self.StepperMotor2Pos)
            
            # agitation mode
            elif self.Stepper2.mode == 3:
                angle_stepper2 = self.Stepper2.AgitationModeControl(self.StepperMotor2Pos, self.StepperMotor2Speed)
            
            # unsupported
            else:
                angle_stepper2 = 0

            # create a new ScienceCmd msg and map with instance values
            newCmd = ScienceCmd()
            newCmd.GripperState = self.GripperState
            newCmd.LedState = self.LedState 
            newCmd.PeltierState = self.PeltierState 
            newCmd.LaserState = self.LaserState 
            newCmd.CcdSensorSnap = self.CcdSensorSnap 
            newCmd.Shutdown = self.Shutdown
            newCmd.MotorSpeed = self.ContMotorSpeed
            newCmd.Stepper1IncAng = angle_stepper1
            newCmd.Stepper2IncAng = angle_stepper2

            self.science_control_publisher.publish(newCmd)
            rospy.sleep(0.001)

if __name__ == "__main__":
    scienceSystem = ScienceSystem()
    scienceSystem.run()
    rospy.spin() 