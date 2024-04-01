from std_msgs.msg import Float32MultiArray
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QSlider,
    QLabel,
    QPushButton,
    QApplication,
)
from PyQt5.QtCore import Qt
import rospy
import os
import sys
from drive_control.msg import WheelSpeed

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)


class ArmControlGUI(QWidget):
    def __init__(self):
        # Define ranges for each joint: (min, max, initial)
        self.sliders = {}
        self.labels = []
        self.downButtons = []
        self.upButtons = []
        self.joint_brushless_lst = {
            "Elbow": (-30, 30, 0),
            "Shoulder": (-30, 30, 0),
            "Waist": (-90, 90, 0),
        }
        self.joint_brushed_lst = {
            "EndEffector": (-400, 400, 0),
            "WristRoll": (0, 360, 0),
            "WristPitch": (-30, 30, 0),
        }
        self.joint_drive_lst = {
            "LB": (-200, 200, 0),
            "LF": (-200, 200, 0),
            "RB": (-200, 200, 0),
            "RF": (-200, 200, 0),
        }
        super(ArmControlGUI, self).__init__()
        self.initUI()
        self.rosSetup()

    def initUI(self):
        mainLayout = QVBoxLayout()

        # Brushless and Brushed motor control
        brushlessGroupLayout = self.createMotorGroup(
            "Brushless", self.joint_brushless_lst
        )
        self.fbLabelBrushless = QLabel(
            "Fb Brushless - Elbow: 0, Shoulder: 0, Waist: 0")
        self.fbLabelBrushless.setAlignment(Qt.AlignCenter)
        brushlessGroupLayout.addWidget(self.fbLabelBrushless)
        mainLayout.addLayout(brushlessGroupLayout)

        brushedGroupLayout = self.createMotorGroup(
            "Brushed", self.joint_brushed_lst)
        self.fbLabelBrushed = QLabel(
            "Fb Brushed - Elbow: 0, Shoulder: 0, Waist: 0")
        self.fbLabelBrushed.setAlignment(Qt.AlignCenter)
        brushedGroupLayout.addWidget(self.fbLabelBrushed)
        mainLayout.addLayout(brushedGroupLayout)

        # Drive control
        driveGroupLayout = self.createMotorGroup(
            "Drive", self.joint_drive_lst, 5)
        self.fbLabelDrive = QLabel("LB: 0, LF: 0, RB: 0, RF: 0")
        self.fbLabelDrive.setAlignment(Qt.AlignCenter)
        driveGroupLayout.addWidget(self.fbLabelDrive)
        mainLayout.addLayout(driveGroupLayout)

        # Reset Button
        self.resetButton = QPushButton("Reset")
        self.resetButton.clicked.connect(self.resetSliders)
        mainLayout.addWidget(self.resetButton)

        self.setLayout(mainLayout)

    def createMotorGroup(self, motorType, jointRanges, increment=1):
        vbox = QVBoxLayout()
        vbox.addWidget(QLabel(f"{motorType} Motor Control"))

        for joint, (minVal, maxVal, initVal) in jointRanges.items():
            hbox = QHBoxLayout()

            downBtn = QPushButton("-")
            self.downButtons.append(downBtn)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(minVal)
            slider.setMaximum(maxVal)
            # Set the initial value
            slider.setValue(initVal)
            slider.valueChanged[int].connect(
                lambda value, j=joint: self.onChangeCmd(j))
            self.sliders[joint] = slider

            upBtn = QPushButton("+")
            self.upButtons.append(upBtn)

            label = QLabel(f"{joint} Cmd: {initVal}")
            self.labels.append(label)

            downBtn.clicked.connect(
                lambda checked, j=joint: self.adjustCmd(j, -increment))
            upBtn.clicked.connect(
                lambda checked, j=joint: self.adjustCmd(j, increment))

            hbox.addWidget(downBtn)
            hbox.addWidget(slider)
            hbox.addWidget(upBtn)

            # labels on top of the sliders
            vbox.addWidget(label)
            vbox.addLayout(hbox)

        return vbox

    def resetSliders(self):
        # Reset brushless sliders
        for joint, (minVal, maxVal, initVal) in self.joint_brushless_lst.items():
            self.sliders[joint].setValue(initVal)
            # Optionally, update the label as well
            index = self.labels.index(
                next(filter(lambda x: x.text().startswith(joint), self.labels))
            )
            self.labels[index].setText(f"{joint} Cmd: {initVal}")

        # Reset brushed sliders
        for joint, (minVal, maxVal, initVal) in self.joint_brushed_lst.items():
            self.sliders[joint].setValue(initVal)
            # Optionally, update the label as well
            index = self.labels.index(
                next(filter(lambda x: x.text().startswith(joint), self.labels))
            )
            self.labels[index].setText(f"{joint} Cmd: {initVal}")

        # Reset drive sliders
        for joint, (minVal, maxVal, initVal) in self.joint_drive_lst.items():
            self.sliders[joint].setValue(initVal)
            # Optionally, update the label as well
            index = self.labels.index(
                next(filter(lambda x: x.text().startswith(joint), self.labels))
            )
            self.labels[index].setText(f"{joint} Cmd: {initVal}")

    def adjustCmd(self, joint, delta):
        # Determine the joint's range based on whether it's in the brushless or brushed list.
        if joint in self.joint_brushless_lst:
            joint_range = self.joint_brushless_lst[joint]
        elif joint in self.joint_brushed_lst:
            joint_range = self.joint_brushed_lst[joint]
        elif joint in self.joint_drive_lst:
            joint_range = self.joint_drive_lst[joint]
        else:
            print(f"Joint '{joint}' not found in either motor list.")
            return

        # Get the slider associated with the joint.
        slider = self.sliders[joint]

        # Calculate the new value, ensuring it stays within the specified range.
        newValue = max(joint_range[0], min(
            joint_range[1], slider.value() + delta))

        # Set the new value to the slider.
        slider.setValue(newValue)

        # No need for manual call to onChangeCmd, as the valueChanged signal will trigger it.
        # self.onChangeCmd(joint)

    def onChangeCmd(self, joint):
        # Convert items to a list to be able to slice
        items = list(self.sliders.items())

        # For brushless motors
        cmdMsgBrushless = Float32MultiArray()
        cmdMsgBrushless.data = [float(slider.value())
                                for key, slider in items[0:3]]
        self.brushlessCmdPublisher.publish(cmdMsgBrushless)

        # For brushed motors
        cmdMsgBrushed = Float32MultiArray()
        cmdMsgBrushed.data = [float(slider.value())
                              for key, slider in items[3:6]]
        self.brushedCmdPublisher.publish(cmdMsgBrushed)

        # For drive motors
        cmdMsgDrive = WheelSpeed()
        cmdMsgDrive.left[0] = items[6][1].value()
        cmdMsgDrive.left[1] = items[7][1].value()
        cmdMsgDrive.right[0] = items[8][1].value()
        cmdMsgDrive.right[1] = items[9][1].value()
        self.driveCmdPublisher.publish(cmdMsgDrive)

        # Update labels
        for i, (joint, slider) in enumerate(items):
            self.labels[i].setText(f"{joint} Cmd: {slider.value()}")

    def on_update_brushless_fb(self, data):
        self.update_arm_fb("Brushless", data)

    def on_update_brushed_fb(self, data):
        self.update_arm_fb("Brushed", data)

    # TODO
    def update_drive_fb(self, data):
        self.fbLabelDrive.setText(
            f"Fb Drive - Left: {data.data[0]:.2f}, Right: {data.data[1]:.2f}")

    def update_arm_fb(self, motorType, data):
        if motorType == "Brushless":
            names = list(self.joint_brushless_lst.keys())
            fbText = f"""Fb {motorType} - {names[0]}: {data.data[0]:.2f}, {names[1]}: {
                data.data[1]:.2f}, {names[2]}: {data.data[2]:.2f}"""
            self.fbLabelBrushless.setText(fbText)
        elif motorType == "Brushed":
            names = list(self.joint_brushed_lst.keys())
            fbText = f"""Fb {motorType} - {names[0]}: {data.data[0]:.2f}, {names[1]}: {
                data.data[1]:.2f}, {names[2]}: {data.data[2]:.2f}"""
            self.fbLabelBrushed.setText(
                fbText)  # Corrected to update the brushed label

    def rosSetup(self):
        rospy.init_node("arm_control_gui", anonymous=True)
        self.brushlessCmdPublisher = rospy.Publisher(
            "/armBrushlessCmd", Float32MultiArray, queue_size=10
        )
        self.brushedCmdPublisher = rospy.Publisher(
            "/armBrushedCmd", Float32MultiArray, queue_size=10
        )
        self.driveCmdPublisher = rospy.Publisher(
            "/wheel_velocity_cmd", WheelSpeed, queue_size=10
        )
        rospy.Subscriber("/armBrushlessFb", Float32MultiArray,
                         self.on_update_brushless_fb)
        rospy.Subscriber("/armBrushedFb", Float32MultiArray,
                         self.on_update_brushed_fb)
        rospy.Subscriber(
            "/wheel_velocity_feedback", WheelSpeed, self.update_drive_fb
        )


def main():
    app = QApplication(sys.argv)
    gui = ArmControlGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
