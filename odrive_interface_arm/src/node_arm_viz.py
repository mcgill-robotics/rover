#!/usr/bin/env python
from std_msgs.msg import Float32MultiArray
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                             QSlider, QLabel, QPushButton, QApplication)
from PyQt5.QtCore import Qt
import rospy
import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)


class ArmControlGUI(QWidget):
    def __init__(self):
        super(ArmControlGUI, self).__init__()
        self.initUI()
        self.ros_setup()

    def initUI(self):
        self.setWindowTitle('Arm Control GUI')
        vbox = QVBoxLayout()

        self.cmd_slider = QSlider(Qt.Horizontal)
        self.cmd_slider.setMinimum(0)
        self.cmd_slider.setMaximum(100)
        self.cmd_slider.valueChanged[int].connect(self.changeValue)

        self.cmd_up_btn = QPushButton("Up")
        self.cmd_up_btn.clicked.connect(lambda: self.adjust_cmd(1))
        self.cmd_down_btn = QPushButton("Down")
        self.cmd_down_btn.clicked.connect(lambda: self.adjust_cmd(-1))

        self.fb_label = QLabel('Feedback: 0')
        self.fb_label.setAlignment(Qt.AlignCenter)

        cmd_hbox = QHBoxLayout()
        cmd_hbox.addWidget(self.cmd_down_btn)
        cmd_hbox.addWidget(self.cmd_slider)
        cmd_hbox.addWidget(self.cmd_up_btn)

        vbox.addLayout(cmd_hbox)
        vbox.addWidget(self.fb_label)

        self.setLayout(vbox)

    def adjust_cmd(self, delta):
        self.cmd_slider.setValue(self.cmd_slider.value() + delta)

    def changeValue(self, value):
        cmd_msg = Float32MultiArray(data=[float(value)])
        self.cmd_publisher.publish(cmd_msg)

    def update_feedback(self, data):
        self.fb_label.setText('Feedback: {:.2f}'.format(data.data[0]))

    def ros_setup(self):
        rospy.init_node('arm_control_gui', anonymous=True)
        self.cmd_publisher = rospy.Publisher(
            '/armBrushlessCmd', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/armBrushlessFb', Float32MultiArray,
                         self.update_feedback)


def main():
    app = QApplication(sys.argv)
    gui = ArmControlGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
