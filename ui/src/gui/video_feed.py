from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import QTransform
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QRadioButton
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget

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
        self._deg0 = QRadioButton(self)
        self._deg90 = QRadioButton(self)
        self._deg180 = QRadioButton(self)
        self._deg270 = QRadioButton(self)

        layout.addWidget(self._deg0)
        layout.addWidget(self._deg90)
        layout.addWidget(self._deg180)
        layout.addWidget(self._deg270)

        self._deg0.setText("0" + chr(176))
        self._deg90.setText("90" + chr(176))
        self._deg180.setText("180" + chr(176))
        self._deg270.setText("270" + chr(176))

        self.setLayout(layout)
        self._deg0.toggled.connect(self._emit_signal)
        self._deg90.toggled.connect(self._emit_signal)
        self._deg180.toggled.connect(self._emit_signal)
        self._deg270.toggled.connect(self._emit_signal)

        self._set_default_angle(angle)

    def _set_default_angle(self, angle):
        """!@brief Initialise the widget and click the proper button.
        @param self Python object pointer
        @param angle Current angle for the selector
        """
        if angle == 90:
            self._deg90.click()
        elif angle == 180:
            self._deg180.click()
        elif angle == 270:
            self._deg270.click()
        else:
            self._deg0.click()

    def _emit_signal(self):
        """!@brief Handle a new angle. Emit the proper signal
        @param self Python object pointer
        """
        if self._deg90.isChecked():
            angle = 90

        elif self._deg180.isChecked():
            angle = 180

        elif self._deg270.isChecked():
            angle = 270

        else:
            angle = 0

        self.turnAngle.emit(angle)

class SingleVideoScreen(QWidget):
    """!@brief Display widget for a single video element with controls
    """
    ## Requests a new topic to be played through this screen
    playTopic = pyqtSignal(str)

    def __init__(self, angle=0, parent=None):
        """!@brief Constructor
        @param self Python object pointer
        @param parent The Qt parent object
        """
        super(SingleVideoScreen, self).__init__(parent)

        self._image_display = QLabel(self)
        self._image_display.setMinimumSize(400, 300)
        self._topic_selector = QComboBox(self)
        self._angle_selector = AngleSelection(angle, self)
        self._angle = angle

        size_policy = QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        self._topic_selector.setSizePolicy(size_policy)
        self._angle_selector.setSizePolicy(size_policy)

        hbox = QVBoxLayout()
        self.setLayout(hbox)

        hbox.addWidget(self._image_display)
        hbox.addWidget(self._topic_selector)
        hbox.addWidget(self._angle_selector)

        self._angle_selector.turnAngle.connect(self.set_angle)
        self._topic_selector.activated.connect(self._topic_sel_callback)
        self._active_topic = ""

        self.setWindowTitle("Video Feeds")

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

        # TODO : remove
        if self._angle != 0:
            image_rotated = image.transformed(QTransform().rotate(self._angle), Qt.SmoothTransformation)
        else:
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