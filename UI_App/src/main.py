#!/usr/bin/env python
# Imports
import sys

import rospy

from ui_layout import Ui_MainWindow


from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc

# TODO: Directory change
#from embedded_bridge.msg import PowerFeedback
#from science_module.msg import SciencePilot, ScienceFeedback

class UI(qtw.QMainWindow, Ui_MainWindow):
    '''
    Main application interface. It inherits from Ui_MainWindow which is the base layout for the
    app present in ui_layout.py. Most of the app is controlled from this class.
    '''

    def __init__(self, *args, **kwargs):
        # Setup the UI from Ui_MainWindow
        super().__init__(*args, **kwargs)
        self.setupUi(self)

        # Listeners
        # power
        self.Autonomy.kill_power_button.clicked.connect(self.on_kill_power)

    def rospy_init(self):
        #rospy.init_node("UINode", anonymous=True)

        # Rospy subscriber
        #self.power_state_subscriber = rospy.Subscriber("power_state_data", PowerFeedback, self.on_power_feedback)
        #self.science_module_subscriber = rospy.Subscriber("science_state_data", ScienceFeedback, self.science_feedback)
        #TODO: ML, CCD Camera, Microcamera

        # Rospy publisher
        #self.science_module_publisher = rospy.Publisher("science_state_data", SciencePilot)
        #TODO: KillSwitch Publisher
        pass

    ## SCIENCE SECTION
    # Subscribers
    def science_feedback(self, msg):
        self.Science.stepper1Fault_bool.setText("Enabled" if msg.Stepper1Fault else "Disabled")
        self.Science.stepper2Fault_bool.setText("Enabled" if msg.Stepper2Fault else "Disabled")
        self.Science.coolerState_bool.setText("Enabled" if msg.PeltierState else "Disabled")
        self.Science.ledState_state_label.setText("Enabled" if msg.LedState else "Disabled")
        self.Science.laserState_state_label.setText("Enabled" if msg.LaserState else "Disabled")
        self.Science.gripperState_state_label.setText("Enabled" if msg.GripperState else "Disabled")

    # Helper
    def send_science_pilot(self):
        msg = None # SciencePilot()

        # Booleans
        msg.LedState = self.ledState
        msg.LaserState = self.laserState
        msg.GripperState = self.gripperState
        msg.PeltierState = self.peltierState
        msg.CcdSensorSnap = self.CcdSensorSnap
        msg.Shutdown = self.shutdown

        # Float 32
        msg.ContMotorSpeed = float(self.contMotorSpeed)
        msg.StepperMotor1Pos = float(self.stepperMotor1Pos)
        msg.StepperMotor2Pos = float(self.stepperMotor2Pos)
        msg.StepperMotor1Speed = float(self.stepperMotor1Speed)
        msg.StepperMotor2Speed = float(self.stepperMotor2Speed)

        # UInt 32
        msg.StepperMotor1ControlMode = self.stepperMotor1ControlMode    # TODO: uint32 conversion?
        msg.StepperMotor2ControlMode = self.stepperMotor2ControlMode

        #self.science_module_publisher.publish(msg)

    ## POWER SECTION
    # Subscribers
    def on_power_feedback(self, msg):
        self.Autonomy.voltage_value.display("%.2f" % float(msg.VoltageBattery1))
        self.Autonomy.current_value.display("%.2f" % float(msg.CurrentBattery1))
        self.Autonomy.power_value.display("%.2f" % (float(msg.CurrentBattery1) * float(msg.VoltageBattery1)))

        self.Autonomy.voltage_value_2.display("%.2f" % float(msg.VoltageBattery2))
        self.Autonomy.current_value_2.display("%.2f" % float(msg.CurrentBattery2))
        self.Autonomy.power_value_2.display("%.2f" % (float(msg.CurrentBattery2) * float(msg.VoltageBattery2)))
        # TODO: Battery lifetime, System enables, kill switch enabled local var?

    # Listeners
    def on_kill_power(self):
        self.power_kill_toggle(True)
        # TODO: send kill power

    # Helpers
    def power_kill_toggle(self, signal):
        self.power_killed = signal
        self.Autonomy.kill_switch_bool.setText("System Killed" if signal else "System Normal")
        # TODO: Change system enabled?



    def arm_error_toggle(self, signal):
        '''
        Takes in a boolean value for signal. If the signal is true, it changes error to red
        otherwise it makes it green.
        '''

        if signal == True:
            self.Arm.error_label.setStyleSheet("QLabel {background:red}\n""")
        else:
            self.Arm.error_label.setStyleSheet("QLabel {background:green}\n""")


    def on_control_changed(self, value):
        '''
        Method takes in the UI and the value of the control_selector combo box. It gets 
        called whenever the ComboBox value gets changed. 
        TODO: Waiting for system controls to be implemented so that this selector can select the control system.
        '''

        if value == "Arm-Cartesian Control":
            pass
            # return arm file
        elif value == "Arm-Joint Control":
            pass
            # Return arm file
        elif value == "Science":
            pass
            # Return science file
        elif value == "Drive":
            pass
            # Return drive file
        else:
            pass
            # Return self for autonomy


def main():
    app = qtw.QApplication([])

    window = UI()
    window.arm_error_toggle(False)      # No errors in arm system at the start
    window.show()

    app.exec()
    #rospy.spin()


if __name__ == '__main__':
    main()
