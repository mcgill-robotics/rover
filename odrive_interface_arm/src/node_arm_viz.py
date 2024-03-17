from std_msgs.msg import Float32MultiArray
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QPushButton, QApplication
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
        self.rosSetup()

    def initUI(self):
        self.setWindowTitle('Arm Control GUI')
        vbox = QVBoxLayout()

        # Define ranges for each joint: (min, max, initial)
        self.jointRanges = {
            'Elbow': (-30, 30, 0),
            'Shoulder': (-30, 30, 0),
            'Waist': (-90, 90, 0)
        }

        self.sliders = []
        self.labels = []
        self.downButtons = []
        self.upButtons = []

        for joint, (minVal, maxVal, initVal) in self.jointRanges.items():
            hbox = QHBoxLayout()

            downBtn = QPushButton("-")
            self.downButtons.append(downBtn)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(minVal)
            slider.setMaximum(maxVal)
            slider.setValue(initVal)  # Set the initial value
            slider.valueChanged[int].connect(
                lambda value, j=joint: self.onChangeCmd(j))
            self.sliders.append(slider)

            upBtn = QPushButton("+")
            self.upButtons.append(upBtn)

            label = QLabel(f'{joint} Cmd: {initVal}')
            self.labels.append(label)

            downBtn.clicked.connect(
                lambda checked, j=joint: self.adjustCmd(j, -1))
            upBtn.clicked.connect(
                lambda checked, j=joint: self.adjustCmd(j, 1))

            hbox.addWidget(downBtn)
            hbox.addWidget(slider)
            hbox.addWidget(upBtn)

            vbox.addLayout(hbox)
            vbox.addWidget(label)

        self.fbLabel = QLabel('Fb - Elbow: 0, Shoulder: 0, Waist: 0')
        self.fbLabel.setAlignment(Qt.AlignCenter)
        vbox.addWidget(self.fbLabel)

        self.setLayout(vbox)

    def adjustCmd(self, joint, delta):
        index = ['Elbow', 'Shoulder', 'Waist'].index(joint)
        slider = self.sliders[index]
        newValue = max(self.jointRanges[joint][0], min(
            self.jointRanges[joint][1], slider.value() + delta))
        slider.setValue(newValue)
        self.onChangeCmd(joint)

    def onChangeCmd(self, joint):
        cmdMsg = Float32MultiArray()
        cmdMsg.data = [float(slider.value()) for slider in self.sliders]
        self.brushlessCmdPublisher.publish(cmdMsg)
        self.brushedCmdPublisher.publish(cmdMsg)
        # Update the labels to reflect the current values using triple double quotes for multi-line or complex expressions.
        for i, label in enumerate(self.labels):
            label.setText(f"""{["Elbow", "Shoulder", "Waist"][i]} Cmd: {
                          self.sliders[i].value()}""")

    def onUpdateBrushlessFb(self, data):
        self.updateFb('Brushless', data)

    def onUpdateBrushedFb(self, data):
        self.updateFb('Brushed', data)

    def updateFb(self, motorType, data):
        fbText = f"""Fb {motorType} - Elbow: {data.data[0]:.2f}, Shoulder: {
            data.data[1]:.2f}, Waist: {data.data[2]:.2f}"""
        self.fbLabel.setText(fbText)

    def rosSetup(self):
        rospy.init_node('arm_control_gui', anonymous=True)
        self.brushlessCmdPublisher = rospy.Publisher(
            '/armBrushlessCmd', Float32MultiArray, queue_size=10)
        self.brushedCmdPublisher = rospy.Publisher(
            '/armBrushedCmd', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/armBrushlessFb', Float32MultiArray,
                         self.onUpdateBrushlessFb)
        rospy.Subscriber('/armBrushedFb', Float32MultiArray,
                         self.onUpdateBrushedFb)


def main():
    app = QApplication(sys.argv)
    gui = ArmControlGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
