from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import QTransform
from PyQt5.QtWidgets import QComboBox, QPushButton
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QRadioButton
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget
import std_msgs
import window_manager
import time
from functools import partial
import cv2
import numpy as np
import rospy as rp
from sensor_msgs.msg._Image import Image
from cv_bridge import CvBridge

class AngleSelection(QWidget):
    """!@brief Widget offers a series of angles to chose from.
    Signal is emitted with the selected angle when the selection changes.
    """
    ## Dictates the desired angle
    turnAngle = pyqtSignal(int, name="turnAngle")

    def __init__(self, angle=0, parent=None):
        """!@brief Constructor lays out and initialises data
        @param self Python object pointer
        @param angle Starting angle for the selector
        @param parent Qt parent object
        """
        super(AngleSelection, self).__init__(parent)

        layout = QHBoxLayout()
        self.setLayout(layout)

    def _emit_signal(self):
        """!@brief Handle a new angle. Emit the proper signal
        @param self Python object pointer
        """
        angle = 0

        self.turnAngle.emit(angle)

class SingleVideoScreen(QWidget):
    """!@brief Display widget for a single video element with controls
    """
    ## Requests a new topic to be played through this screen
    playTopic = pyqtSignal(str)

    def handle_image(self, _, image: Image):
        qimage = QImage(image.data, image.width, image.height, QImage.Format_RGB888)
        self.new_sample(qimage)

    def __init__(self, angle=0, parent=None):
        """!@brief Constructor
        @param self Python object pointer
        @param parent The Qt parent object
        """
        super(SingleVideoScreen, self).__init__(parent)

        self.counter = 1

        self.current_image: QImage = None

        self._image_display = QLabel(self)
        self._image_display.setMinimumSize(400, 300)
        #self._topic_selector = QComboBox(self)
        self._angle_selector = AngleSelection(angle, self)
        self._angle = angle

        size_policy = QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        #self._topic_selector.setSizePolicy(size_policy)
        self._angle_selector.setSizePolicy(size_policy)

        hbox = QVBoxLayout()
        self.setLayout(hbox)
        self._button_row = QWidget()
        self._button_row_layout = QHBoxLayout()
        self._button_row.setLayout(self._button_row_layout)
        self._cycle_forward = QPushButton(">>")
        self._cycle_backward = QPushButton("<<")
        self._capture = QPushButton("Capture")
        self._button_row_layout.addWidget(self._cycle_backward)
        self._button_row_layout.addWidget(self._cycle_forward)
        self._button_row_layout.addWidget(self._capture)

        hbox.addWidget(self._image_display)
        #hbox.addWidget(self._topic_selector)
        hbox.addWidget(self._angle_selector)
        hbox.addWidget(self._button_row)

        self._cycle_forward.clicked.connect(self._cycle_video_stream_forward)
        self._cycle_backward.clicked.connect(self._cycle_video_stream_backward)
        self._capture.clicked.connect(self._capture_image)

        self._angle_selector.turnAngle.connect(self.set_angle)
        #self._topic_selector.activated.connect(self._topic_sel_callback)
        self._active_topic = ""

        self.setWindowTitle("Video Feeds")

        # subscribe to image stream
        self.cameraSub = rp.Subscriber("/webcam/image_raw", Image, partial(self.handle_image, self))
        self.forwardPub = rp.Publisher("/cam_control/forward", std_msgs.msg.Int32, queue_size=1)
        self.backwardPub = rp.Publisher("/cam_control/backward", std_msgs.msg.Int32, queue_size=1)

        self.cv_bridge = CvBridge()

        # self.cam = cv2.VideoCapture(0)

        self.cam_sub = rp.Subscriber("/cam_feed", Image, callback=partial(self._on_frame_received, self), queue_size=1)

        # class DefaultCameraFeedThread(QtCore.QThread):
        #     def __init__(self, parent, new_sample_fn):
        #         super().__init__()
        #         self.parent = parent
        #         self.new_sample_fn = new_sample_fn

        #     def run(self):
        #         while True:
        #             time.sleep(1.0 / 60.0)
        #             rval, frame = self.cam.read()
        #             frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #             qimage = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        #             self.new_sample_fn(qimage)

        # self.cam_feed_thread = DefaultCameraFeedThread(self, partial(self.new_sample))
        # self.cam_feed_thread.start()

    def _capture_image(self):
        if self.current_image is not None:
            self.current_image.save(f"~/captures/image.png{self.counter}", "png")
            self.counter += 1
        else:
            print("No image")

    def _on_frame_received(self, _, image):
        frame = self.cv_bridge.imgmsg_to_cv2(image)
        #print(type(frame))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimage = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        self.current_image = qimage
        self.new_sample(qimage)

    def _cycle_video_stream_forward(self):
        self.forwardPub.publish(std_msgs.msg.Int32(0))

    def _cycle_video_stream_backward(self):
        self.backwardPub.publish(std_msgs.msg.Int32(0))

    def get_active_topic(self):
        """!@brief Get the currently selected topic
        @param self Python object pointer
        """
        return self._active_topic

    @pyqtSlot(int)
    def set_angle(self, angle):
        """!@brief Set the angle the feed should be rotated by
        @param self Python object pointer
        @param angle Orientation of the image
        """
        self._angle = angle

    @pyqtSlot(QImage)
    def new_sample(self, image):
        """!@brief Display a new image
        Create new pixmap from the QImage, transform to rotation then display in label
        @param image QImage to display
        @param self Python object pointer
        """
        if image is None or image.isNull():
            self._image_display.setText("No Image")
            return

        #if self._angle != 0:
        #    image_rotated = image.transformed(QTransform().rotate(self._angle), Qt.SmoothTransformation)
        #else:
        image_rotated = image
        image_rotated = image_rotated.scaled(self.width(), self.height(), Qt.KeepAspectRatio)
        pixmap = QPixmap.fromImage(image_rotated)
        self._image_display.setPixmap(pixmap)

    @pyqtSlot(bool)
    def set_controls_visible(self, visible):
        """!@brief Show or hide the feed selection and orientaton controls
        @param self Python object pointer
        @param visible True to show the control elements
        """
        self._topic_selector.setVisible(visible)
        self._angle_selector.setVisible(visible)

    @pyqtSlot(str)
    def add_feed_entry(self, string):
        """!@brief Add a single topic element to the selection box
        @param self Python object pointer
        @param string Element to add to the combo box
        """
        if string is not None:
            self._topic_selector.addItem(string)

    def set_feed_list(self, feed_list):
        """!@brief Replace the list of topics in the selection box by a new list
        @param self Python object pointer
        @param feed_list The list to use as replacement
        """
        self._topic_selector.clear()
        for i in feed_list:
            self.add_feed_entry(i)

    def _topic_sel_callback(self, index):
        """!@brief Change topic callback
        emits the signal to request a new topic
        @param self Python object pointer
        @param index The list index of the selected topic
        """
        if index < self._topic_selector.count():
            topic = self._topic_selector.itemText(index)
            self._active_topic = topic
            self.playTopic.emit(topic)
