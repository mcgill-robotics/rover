#!/usr/bin/env python3

import cv2
import sys

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

# PyQt UI code
# thread to get camera data in parallel; emits a signal to main program loop
# https://stackoverflow.com/questions/44404349/pyqt-showing-video-stream-from-opencv/44404713
class Thread(QThread):
    # defines signal that will emit QImage object (after conversion from OpenCV array)
    pixmapSignal = pyqtSignal(QImage)

    def run(self):
        cam = cv2.VideoCapture(0)
        while True:
            ret_val, cv_image = cam.read()
            # if return value received, convert the image obtained from Numpy array to pixmap for Qt
            # conversion code found here: https://github.com/docPhil99/opencvQtdemo/blob/master/staticLabel2.py
            if ret_val:
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                convert_to_Qt = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                p = convert_to_Qt.scaled(640, 480, Qt.KeepAspectRatio)
                self.pixmapSignal.emit(p)

class CamFeed(QWidget):
    @pyqtSlot(QImage) # defines a slot
    def updateImage(self, img):
        self.img_label.setPixmap(QPixmap.fromImage(img))

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera Feed")

        # label that will hold the OpenCv video data
        self.img_label = QLabel(self)
        self.txt_label = QLabel("Displaying camera feed...")

        # vertical layout
        layout = QVBoxLayout()
        layout.addWidget(self.img_label)
        layout.addWidget(self.txt_label)

        self.setLayout(layout)

        # intialize thread + connect signal received from thread (video data) to update image function
        newThread = Thread(self)
        newThread.pixmapSignal.connect(self.updateImage)
        newThread.start()
        
        # display widget
        self.show()

app = QApplication(sys.argv)
feed = CamFeed()
sys.exit(app.exec_())
