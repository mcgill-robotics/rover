# Imports
from ui_layout import Ui_MainWindow
import rospy
from drive import Drive_Backend
from UI_App.msg import WheelSpeed
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc


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
        

        #drive setup
        self.drive_backend = Drive_Backend(self.Drive)
        self.drive_wheel_velocity_subscriber = rospy.Subscriber('/wheel_velocity_cmd', WheelSpeed, self.drive_backend.update_wheel_velocities)
        self.drive_twist_subscriber = rospy.Subscriber('/robot_twist_cmd', Twist, self.drive_backend.update_twist_data)
        self.drive_location_subscriber = rospy.Subscriber('/visualization_marker_array', MarkerArray, self.drive_backend.update_robot_location) 

    
    
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