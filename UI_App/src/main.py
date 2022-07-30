#!/usr/bin/env python
# Imports
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)

from ui_layout import Ui_MainWindow
import rospy
from drive import Drive_Backend
from UI_App.msg import WheelSpeed
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc


from embedded_bridge.msg import PowerFeedback
from science_module.msg import SciencePilot, ScienceFeedback

class UI(qtw.QMainWindow, Ui_MainWindow):
    '''
    Main application interface. It inherits from Ui_MainWindow which is the base layout for the
    app present in ui_layout.py. Most of the app is controlled from this class.
    '''

    

    def __init__(self, *args, **kwargs):
        # Setup the UI from Ui_MainWindow
        super().__init__(*args, **kwargs)
        self.setupUi(self)
        rospy.init_node("UINode", anonymous=True)


        # Listeners
        self.control_selector.currentTextChanged.connect(self.on_control_changed)
        
        # power
        self.Autonomy.kill_power_button.clicked.connect(self.on_kill_power)

        # science
        self.Science.send_button.clicked.connect(self.send_science_pilot)
        self.Science.io_shutdown_button.clicked.connect(self.send_science_shutdown)

        #drive setup
        self.drive_backend = Drive_Backend(self.Drive)
        self.drive_wheel_velocity_subscriber = rospy.Subscriber('/wheel_velocity_cmd', WheelSpeed, self.drive_backend.update_wheel_velocities)
        self.drive_twist_subscriber = rospy.Subscriber("rover_velocity_controller/cmd_vel", Twist, self.drive_backend.update_twist_data)
        self.drive_location_subscriber = rospy.Subscriber('/visualization_marker_array', MarkerArray, self.drive_backend.update_robot_location)



        # Rospy subscriber
        self.power_state_subscriber = rospy.Subscriber("power_state_data", PowerFeedback, self.on_power_feedback)
        self.science_module_subscriber = rospy.Subscriber("science_state_data", ScienceFeedback, self.on_science_feedback)
        # TODO: ML, CCD Camera, Microcamera

        # Rospy publisher
        self.science_module_publisher = rospy.Publisher("science_controller_feedback", SciencePilot, queue_size=10)
        # TODO: KillSwitch Publisher

    
    ## POWER SECTION
    # Subscribers
    def on_power_feedback(self, msg):
        self.update_float_value(msg.VoltageBattery1, self.Autonomy.voltage_value_1)
        self.update_float_value(msg.CurrentBattery1, self.Autonomy.current_value_1)
        self.update_float_value((float(msg.CurrentBattery1) * float(msg.VoltageBattery1)), self.Autonomy.current_value_1)

        self.update_float_value(msg.VoltageBattery2, self.Autonomy.voltage_value_2)
        self.update_float_value(msg.CurrentBattery2, self.Autonomy.current_value_2)
        self.update_float_value((float(msg.CurrentBattery2) * float(msg.VoltageBattery2)), self.Autonomy.current_value_2)
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
        #TODO: Waiting for system controls to be implemented so that this selector can 
        select the control system.
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


if __name__ == '__main__':
    main()
    rospy.spin()