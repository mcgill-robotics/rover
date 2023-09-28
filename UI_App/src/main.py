#!/usr/bin/env python
# Imports
import os
import sys

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
from science_backend import Science_Backend
from power_backend import Power_Backend
from gps_backend import GPS_Backend
from drive_control.msg import WheelSpeed
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from arm_control.msg import ArmControllerInput
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc

from embedded_bridge.msg import PowerFeedback
from science_module.msg import SciencePilot, ScienceFeedback

msg = "test"
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

        self.drive_backend = Drive_Backend(self.Drive)
        self.arm_backend = Arm_Backend(self.Arm)
        self.science_backend = Science_Backend(self.Science)
        self.power_backend = Power_Backend(self.Power)
        self.gps_backend = GPS_Backend(self.GPS)

        # power
        self.Power.drive_enabled.toggled.connect(self.power_backend.get_drive_enabled)
        self.Power.science_enabled.toggled.connect(self.power_backend.get_science_enabled)
        self.Power.lower_arm_enabled.toggled.connect(self.power_backend.get_lower_arm_enabled)
        self.Power.upper_arm_enabled.toggled.connect(self.power_backend.get_upper_arm_enabled)


        # self.power_state_subscriber = rospy.Subscriber("power_state_data", PowerFeedback,self.power_backend.on_power_feedback)
        self.killswitch_subscriber = rospy.Subscriber("killswitchFB", Float32MultiArray, self.power_backend.power_feedback)
        self.power_state_subscriber = rospy.Subscriber("powerFB", Float32MultiArray, self.power_backend.system_feedback)
        self.power_state_publisher = rospy.Publisher("powerCmd", Float32MultiArray, queue_size=1)


        # science
        self.Science.send_button.clicked.connect(self.send_science_cmd)
        self.science_module_subscriber = rospy.Subscriber("scienceFB", Float32MultiArray, self.science_backend.on_science_feedback)
        self.science_module_publisher = rospy.Publisher("scienceCmd", Float32MultiArray, queue_size=10)

        # drive
        self.Drive.send_antenna.clicked.connect(self.drive_backend.on_send)
        self.drive_twist_subscriber = rospy.Subscriber("driveFB", Float32MultiArray, self.drive_backend.update_wheel_velocities)
        self.drive_location_subscriber = rospy.Subscriber('/position_pose', Pose,self.drive_backend.update_robot_location)


        # arm
        self.arm_brushed_subscriber = rospy.Subscriber("armBrushedFB", Float32MultiArray, self.arm_backend.update_joints_brushed)
        self.arm_brushless_subscriber = rospy.Subscriber("armBrushlessFB", Float32MultiArray, self.arm_backend.update_joints_brushless)
        self.arm_control_subscriber = rospy.Subscriber("arm_controller_input", ArmControllerInput, self.arm_backend.update_control)
        self.arm_error_subscriber = rospy.Subscriber("armError", String, self.arm_backend.set_error)

        #gps
        self.gps_subscriber = rospy.Subscriber("roverGPSData", Float32MultiArray, self.set_gps_data)
        self.pan_tilt_angle_subscriber = rospy.Subscriber("panTiltAngles", Float32MultiArray, self.set_pan_tilt_angle)
        self.gps_data = []
        self.pan_tilt_angle = []

        self.save_button.clicked.connect(self.save_image)

        # camera selection
        # this timer seemes not in use now
        self.timer_camera = qtc.QTimer()  # set up a timer, this is used to control frame rate
        self.cam_image = None
        self.camera_index_publisher = rospy.Publisher("camera_selection", Int16, queue_size=1)
        self.camera_selector.currentTextChanged.connect(self.on_camera_changed)

        self.camera_frame_subscriber = rospy.Subscriber('/camera_frames', Image, self.set_image)
        self.timer_camera.start(33)
        self.count = 0

    def set_gps_data(self, gps_data):
        self.gps_data = gps_data.data
        self.gps_backend.plot_gps_figure(gps_data)

    def set_pan_tilt_angle(self, angle_data):
        self.pan_tilt_angle = angle_data.data

    def save_image(self):
        save_image = self.cam_image
        image_name = f"lat: {self.gps_data[0]}, long: {self.gps_data[1]} v-angle: {self.pan_tilt_angle[0]}, h-angle: {self.pan_tilt_angle[1]}"
        image_path = self.save_path.text() + image_name + ".jpg"
        print(image_path)
        cv2.imwrite(image_path, save_image)

    def send_science_cmd(self):
        msg = self.science_backend.set_science_cmd()
        self.science_module_publisher.publish(msg)

    def arm_error_toggle(self, signal):
        # Takes in a boolean value for signal. If the signal is true, it changes error to red otherwise it makes it green.
        if signal == True:
            self.Arm.error_label.setStyleSheet("QLabel {background:red}\n""")
        else:
            self.Arm.error_label.setStyleSheet("QLabel {background:green}\n""")

    def on_control_changed(self, value):
        # Method takes in the UI and the value of the control_selector combo box. It gets called whenever the ComboBox value gets changed.
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
        self.timer_camera.start(33)  # start timer to request for video
        self.count = 0

    def show_image(self):
        try:
            height, width, channel = self.cam_image.shape
            bytesPerLine = 3 * width
            showImage = cv2.cvtColor(self.cam_image, cv2.COLOR_BGR2RGB)
            showImage = QtGui.QImage(showImage.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.Camera.setPixmap(QtGui.QPixmap.fromImage(showImage))
        except:
            # if no image is published, it will show words on camera showbox
            msg = f"no image {self.count // 30}"
            # print(msg, end="\r")
            self.Camera.setText(msg)  # if you don't feel comfortable displaying error message on camera screen, comment out this line and enable the line above
            self.count += 1

    # when rospy send the image, it will call this function
    def set_image(self, msg):
        try:
            frames = self.ros_to_openCV_image(msg)
            frames = cv2.imdecode(frames, 1)
            # this will reset the image size to fit the camera shobox
            self.cam_image = cv2.resize(frames, (1000, 880))
            print("image set")
            self.show_image()
        except:
            print("cannot set image correctly")

    def ros_to_openCV_image(self, ros_image):
        # the ros publisher gives us pictures in ros format, we have to translate it back to CV
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        return cv_image

    def append_overall_feedback_text(self, msg: str):
        # for append messages on overall_feedback box
        if msg:
            self.overall_feedback_messagebox.append(msg)
        else:
            self.overall_feedback_messagebox.append("empty message was sent (append)")

    def set_overall_feedback_text(self, msg: str):
        # for set messages on overall_feedback box(clear and write)
        if msg:
            self.overall_feedback_messagebox.setText(msg)
        else:
            self.overall_feedback_messagebox.setText("empty message was sent (set)")


def main():
    app = qtw.QApplication([])
    window = UI()
    window.arm_error_toggle(False)  # No errors in arm system at the start
    window.show()
    app.exec()


if __name__ == '__main__':
    main()
    rospy.spin()
