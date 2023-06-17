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
from science_backend import Science_Backend
from power_backend import Power_Backend
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


    def __init__(self, *args, **kwargs):

        # Setup the UI from Ui_MainWindow
        super().__init__(*args, **kwargs)
        self.setupUi(self)
        rospy.init_node("UINode", anonymous=True)

        self.drive_backend = Drive_Backend(self.Drive)
        self.arm_backend = Arm_Backend(self.Arm)
        self.science_backend = Science_Backend(self.Science)
        self.power_backend = Power_Backend(self.Power)
        
        # system selection
        self.system_select_publisher = rospy.Publisher("system_selection", Int16, queue_size=1)
        
        # power
        self.control_selector.currentTextChanged.connect(self.on_control_changed)
        self.Power.kill_power_button.clicked.connect(self.power_backend.on_kill_power)
        self.power_state_subscriber = rospy.Subscriber("power_state_data", PowerFeedback, self.power_backend.on_power_feedback)

        # science
        self.Science.send_button.clicked.connect(self.send_science_pilot)
        self.Science.io_shutdown_button.clicked.connect(self.send_science_shutdown)
        self.science_module_subscriber = rospy.Subscriber("science_state_data", ScienceFeedback, self.science_backend.on_science_feedback)
        self.science_module_publisher = rospy.Publisher("science_controller_feedback", SciencePilot, queue_size=10)

        #drive
        self.drive_wheel_velocity_subscriber = rospy.Subscriber('/wheel_velocity_cmd', WheelSpeed, self.drive_backend.update_wheel_velocities)
        self.drive_twist_subscriber = rospy.Subscriber("rover_velocity_controller/cmd_vel", Twist, self.drive_backend.update_twist_data)
        self.drive_location_subscriber = rospy.Subscriber('/position_pose', Pose, self.drive_backend.update_robot_location)

        #arm
        self.arm_hand_subscriber= rospy.Subscriber("arm_state_data", ArmStatusFeedback, self.arm_backend.update_joints) 

        # TODO: KillSwitch Publisher

        # camera selection
        self.timer_camera = qtc.QTimer()  # set up a timer, this is used to control frame rate
        self.cam_image = None
        self.camera_index_publisher = rospy.Publisher("camera_selection", Int16, queue_size=1)
        self.camera_selector.currentTextChanged.connect(self.on_camera_changed)
        self.timer_camera.timeout.connect(self.show_image)
        self.camera_frame_subscriber = rospy.Subscriber('/camera_frames', Image, self.set_image)
        self.timer_camera.start(1)
        self.count = 0



    def send_science_pilot(self):
        msg = self.science_backend.set_science_pilot()
        self.science_module_publisher.publish(msg)


    def send_science_shutdown(self):
        # Function creating a SciencePilot message for shutdown, setting it to true and sending it through messaging broker
        msg = SciencePilot()
        msg.Shutdown = True
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
        self.timer_camera.start(1)  # start timer to request for video


    def show_image(self):
        try:
            height, width, channel = self.cam_image.shape
            bytesPerLine = 3 * width
            showImage = cv2.cvtColor(self.cam_image, cv2.COLOR_BGR2RGB)
            showImage = QtGui.QImage(showImage.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.Camera.setPixmap(QtGui.QPixmap.fromImage(showImage))
        except:
            print("no image %d' "% self.count,end = "\r")
            self.count+=1


    def set_image(self,msg):
        try:
            frames = self.ros_to_openCV_image(msg)
            frames = cv2.imdecode(frames, 1)
            self.cam_image = cv2.resize(frames, (410, 270)) #todo test which one is better
            print("image set")
        except:
            print("cannot set image correctly")


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