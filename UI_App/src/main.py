#!/usr/bin/env python
# Imports
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
from PyQt5 import QtGui
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge

from ui_layout import Ui_MainWindow
import rospy
from drive_backend import Drive_Backend
from arm_backend import Arm_Backend
from drive_control.msg import WheelSpeed
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray
from arm_control.msg import ArmStatusFeedback

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc


from embedded_bridge.msg import PowerFeedback
from science_module.msg import SciencePilot, ScienceFeedback

class UI(qtw.QMainWindow, Ui_MainWindow):
    '''
    Main application interface. It inherits from Ui_MainWindow which is the base layout for the
    app present in ui_layout.py. Most of the app is controlled from this class.
    '''

    ## READER FUNCTIONS
    @staticmethod
    def get_boolean_value(button):
        return button.isChecked()

    @staticmethod
    def get_double_spin_box_value(double_spin_box):
        return float(double_spin_box.value())

    @staticmethod
    def get_uint_spin_box_value(spin_box):
        return int(spin_box.value())

    ## UPDATER FUNCTIONS
    @staticmethod
    def update_boolean_value(condition, label):
        label.setText("Enabled" if condition else "Disabled")

    @staticmethod
    def update_float_value(value, label):
        label.display("%.2f" % float(value))


    def __init__(self, *args, **kwargs):
        # Setup the UI from Ui_MainWindow
        super().__init__(*args, **kwargs)
        self.setupUi(self)
        rospy.init_node("UINode", anonymous=True)


        self.timer_camera = qtc.QTimer()  # set up a timer, this is used to control frame rate
        # self.cap1 = cv2.VideoCapture()  # video float
        self.__camera_opened = False
        self.cam_image = None

        # Listeners
        self.control_selector.currentTextChanged.connect(self.on_control_changed)
        
        # power
        self.Autonomy.kill_power_button.clicked.connect(self.on_kill_power)

        # science
        self.Science.send_button.clicked.connect(self.send_science_pilot)
        self.Science.io_shutdown_button.clicked.connect(self.send_science_shutdown)

        #drive setup
        self.drive_backend = Drive_Backend(self.Drive)
        self.arm_backend = Arm_Backend(self.Arm)
        self.drive_wheel_velocity_subscriber = rospy.Subscriber('/wheel_velocity_cmd', WheelSpeed, self.drive_backend.update_wheel_velocities)
        self.drive_twist_subscriber = rospy.Subscriber("rover_velocity_controller/cmd_vel", Twist, self.drive_backend.update_twist_data)
        self.drive_location_subscriber = rospy.Subscriber('/position_pose', Pose, self.drive_backend.update_robot_location)
        self.arm_hand_subscriber= rospy.Subscriber("arm_state_data", ArmStatusFeedback, self.arm_backend.update_joints) 

        # Rospy subscriber
        self.power_state_subscriber = rospy.Subscriber("power_state_data", PowerFeedback, self.on_power_feedback)
        self.science_module_subscriber = rospy.Subscriber("science_state_data", ScienceFeedback, self.on_science_feedback)
        # TODO: ML, CCD Camera, Microcamera

        # Rospy publisher
        self.science_module_publisher = rospy.Publisher("science_controller_feedback", SciencePilot, queue_size=10)
        # TODO: KillSwitch Publisher


        # camera selection
        self.camera_index_publisher = rospy.Publisher("camera_selection", Int16, queue_size=1)
        self.camera_selector.currentTextChanged.connect(self.on_camera_changed)

        #camera float
        self.timer_camera.timeout.connect(self.show_image)
        self.camera_frame_subscriber = rospy.Subscriber('/camera_frames', Image, self.set_image)
        self.timer_camera.start(33)
        self.count = 0

    ## SCIENCE SECTION
    # Subscribers
    def on_science_feedback(self, msg):
        """
        Callback function when receiving a science message from messaging broker

        :param msg: ScienceFeedback
        """
        self.update_boolean_value(msg.Stepper1Fault, self.Science.stepper1Fault_bool)
        self.update_boolean_value(msg.Stepper2Fault, self.Science.stepper2Fault_bool)
        self.update_boolean_value(msg.PeltierState, self.Science.coolerState_bool)
        self.update_boolean_value(msg.LedState, self.Science.ledState_state_label)
        self.update_boolean_value(msg.LaserState, self.Science.laserState_state_label)
        self.update_boolean_value(msg.GripperState, self.Science.gripperState_state_label)

    # Helpers
    def send_science_shutdown(self):
        """
        Function creating a SciencePilot message for shutdown, setting it to true and sending it through messaging broker
        """
        msg = SciencePilot()
        msg.Shutdown = True
        self.science_module_publisher.publish(msg)

    def send_science_pilot(self):
        """
        Function creating a SciencePilot message and sending request through messaging broker
        """
        msg = SciencePilot()

        self.Science.laserState_toggle.isChecked()

        # Booleans
        msg.LedState = self.get_boolean_value(self.Science.ledState_toggle)
        msg.LaserState = self.get_boolean_value(self.Science.laserState_toggle)
        msg.GripperState = self.get_boolean_value(self.Science.gripperState_toggle)
        msg.PeltierState = self.get_boolean_value(self.Science.peltierState_toggle)
        msg.CcdSensorSnap = self.get_boolean_value(self.Science.ccdSensorSnap_toggle)

        # Float 32
        msg.ContMotorSpeed = self.get_double_spin_box_value(self.Science.contMotorSpeed_doubleSpinBox)
        msg.StepperMotor1Pos = self.get_double_spin_box_value(self.Science.stepper1Pos_doubleSpinBox)
        msg.StepperMotor2Pos = self.get_double_spin_box_value(self.Science.stepper1Pos_doubleSpinBox)
        msg.StepperMotor1Speed = self.get_double_spin_box_value(self.Science.stepper1Speed_doubleSpinBox)
        msg.StepperMotor2Speed = self.get_double_spin_box_value(self.Science.stepper2Speed_doubleSpinBox)

        # UInt 32
        msg.Stepper1ControlMode = self.get_uint_spin_box_value(self.Science.stepper1ControlMode_spinBox)
        msg.Stepper2ControlMode = self.get_uint_spin_box_value(self.Science.stepper2ControlMode_spinBox)

        self.science_module_publisher.publish(msg)

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
        if value == "Drive":
            msg = Int16(1)
            self.system_select_publisher.publish(msg)
        elif value == "Arm":
            msg = Int16(2)
            self.system_select_publisher.publish(msg)
        elif value == "Science":
            msg = Int16(3)
            self.system_select_publisher.publish(msg)
        else:
            pass


    def on_camera_changed(self, value):
        self.timer_camera.stop()
        index = self.camera_selector.currentIndex()
        msg = Int16(index)
        self.camera_index_publisher.publish(msg)
        self.timer_camera.start(16)  # start timer to request for video


    def show_image(self):
        try:
            height, width, channel = self.cam_image.shape
            bytesPerLine = 3 * width
            self.cam_image = cv2.cvtColor(self.cam_image, cv2.COLOR_BGR2RGB)
            self.cam_image = cv2.resize(self.cam_image, (410, 270))  #TODO: this causes image flicker
            showImage = QtGui.QImage(self.cam_image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.Camera.setPixmap(QtGui.QPixmap.fromImage(showImage))
        except:
            print("no image %d' "% self.count,end = "\r")
            self.count+=1


    def set_image(self,msg):
        try:
            frames = self.ros_to_openCV_image(msg)
            frames= cv2.resize(frames, (410, 270)) #todo test which one is better
            self.cam_image = cv2.imdecode(frames, 1)
            print("image set")
        except:
            print("cannot set image correctly" + msg)


    def ros_to_openCV_image(self, ros_image):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        return cv_image
    


def main():
    app = qtw.QApplication([])
    window = UI()
    window.arm_error_toggle(False)      # No errors in arm system at the start
    window.show()
    app.exec()


if __name__ == '__main__':
    main()
    rospy.spin()