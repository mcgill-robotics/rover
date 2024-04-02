

import sys
import os

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
sys.path.append(currentdir + "/../..")

try:
    import scipy.stats as st
    from scipy.ndimage import gaussian_filter1d
    import numpy as np
    import math
    from PyQt5.QtCore import Qt, QThread, QTimer, QEventLoop, pyqtSignal
    from PyQt5.QtWidgets import (
        QWidget,
        QVBoxLayout,
        QHBoxLayout,
        QSlider,
        QLabel,
        QPushButton,
        QApplication,
    )
    from PyQt5.QtGui import QKeyEvent, QPainter, QPen, QColor

    import rospy
    from std_msgs.msg import Float32MultiArray
    from drive_control.msg import WheelSpeed
    from geometry_msgs.msg import Twist

    from pynput import keyboard

    from drive_control.src.steering import Steering
except ImportError as e:
    print(f"Error: {e}")
    print("Please install the required packages using the command:")
    print("pip install -r requirements.txt")
    sys.exit(1)


class ArrowWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.linear_velocity = 0
        self.angular_velocity = 0
        # Should be same as the maxLinearVelocity and maxAngularVelocity in the main class
        self.maxLinearVelocity = 10
        self.maxAngularVelocity = 10
        self.setMinimumSize(200, 200)  # Ensure widget is large enough

    def set_velocities(self, linear, angular):
        self.linear_velocity = linear
        self.angular_velocity = angular
        self.update()  # Trigger a repaint to show updated velocities

    # def paintEvent(self, event):
    #     qp = QPainter(self)
    #     qp.setPen(QPen(QColor(0, 0, 0), 2))
    #     qp.setRenderHint(QPainter.Antialiasing)

    #     # Calculate arrow parameters based on velocities
    #     center = self.rect().center()
    #     arrow_length = max(min(abs(self.linear_velocity) *
    #                        10, self.rect().height() / 2), 10)
    #     arrow_angle = self.angular_velocity * 10

    #     # Calculate arrow end point
    #     # Explicitly converted to int
    #     end_point_x = int(center.x() + arrow_length)
    #     # Explicitly converted to int
    #     end_point_y = int(center.y() + arrow_angle)

    #     # Ensure end point is within widget bounds
    #     end_point_y = max(
    #         min(end_point_y, self.rect().bottom()), self.rect().top())

    #     # Draw the arrow
    #     qp.drawLine(center.x(), center.y(), end_point_x, end_point_y)
    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setPen(QPen(QColor(0, 0, 0), 2))
        qp.setRenderHint(QPainter.Antialiasing)

        center = self.rect().center()
        velocity_threshold = 0.01  # Threshold for considering velocity as zero
        # Use smaller dimension for max length
        max_pixel_length = min(self.rect().width(), self.rect().height()) / 2
        max_angle_degrees = 60  # Max rotation angle from the vertical "north"

        # Normalize linear velocity to [-1, 1] range
        normalized_linear_vel = self.linear_velocity / self.maxLinearVelocity
        # Normalize angular velocity and calculate rotation angle
        normalized_angular_vel = self.angular_velocity / self.maxAngularVelocity
        angle_degrees = normalized_angular_vel * max_angle_degrees

        if abs(self.linear_velocity) < velocity_threshold:
            dot_radius = 3  # Draw a dot for zero/near-zero velocities
            qp.drawEllipse(center, dot_radius, dot_radius)
        else:
            # Calculate arrow length from normalized linear velocity, ensuring minimum visibility
            arrow_length = max(abs(normalized_linear_vel)
                               * max_pixel_length, 3)

            # Adjust angle based on direction; flip arrow for negative velocities
            angle_radians = math.radians(angle_degrees)
            if self.linear_velocity < 0:
                # For negative linear velocities, adjust the arrow to point "south"
                # This involves adding 180 degrees (pi radians) to flip the arrow direction
                angle_radians += math.pi

            # Calculate arrow end point using polar coordinates conversion
            end_point_x = center.x() + arrow_length * math.sin(angle_radians)
            end_point_y = center.y() - arrow_length * math.cos(angle_radians)

            # Draw the arrow
            qp.drawLine(center.x(), center.y(), int(
                end_point_x), int(end_point_y))


class ArmControlGUI(QWidget):
    def __init__(self):
        # Define ranges for each joint: (min, max, initial)
        self.sliders = {}
        self.labels = []
        self.downButtons = []
        self.upButtons = []
        self.joint_brushless_lst = {
            "Elbow": (-30, 30, 0, 1),
            "Shoulder": (-30, 30, 0, 1),
            "Waist": (-90, 90, 0, 1),
        }
        self.joint_brushed_lst = {
            "EndEffector": (-400, 400, 0, 1),
            "WristRoll": (0, 360, 0, 1),
            "WristPitch": (-30, 30, 0, 1),
        }
        self.joint_drive_lst = {
            "LB": (-200, 200, 0, -1),
            "LF": (-200, 200, 0, -1),
            "RB": (-200, 200, 0, 1),
            "RF": (-200, 200, 0, 1),
        }
        super(ArmControlGUI, self).__init__()
        self.initUI()
        self.rosSetup()

        # Set up the keyboard listener thread

        # self.keyboardListenerThread = KeyboardListenerThread()
        # self.keyboardListenerThread.start()
        # print("Keyboard listener started")
        self.roverLinearVelocity = 0.0
        self.roverAngularVelocity = 0.0
        self.keyboard_accumulator_linear = 0.0
        self.keyboard_accumulator_twist = 0.0
        self.keyboard_sensitivity = 0.025
        self.maxLinearVelocity = 10
        self.maxAngularVelocity = 10

        # Setup movement timer
        self.movement_timer = QTimer(self)
        self.movement_timer.timeout.connect(self.updateMovement)
        self.movement_timer.start(10)  # Update every 10ms

        # Setup the decay timer
        self.decay_timer = QTimer(self)
        self.decay_timer.timeout.connect(self.decay_velocity)
        self.decay_timer.start(10)  # Decay every 10ms

        self.pressedKeys = set()  # Set to keep track of pressed keys
        self.keyStates = {}  # Tracks the press state of each key

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

        # Drive Fb and Cmd
        driveGroupLayout = self.createMotorGroup(
            "Drive", self.joint_drive_lst, 5)
        self.fbLabelDrive = QLabel("Drive Fb - LB: 0, LF: 0, RB: 0, RF: 0")
        self.fbLabelDrive.setAlignment(Qt.AlignCenter)
        self.drive_cmd_label = QLabel("Drive Cmd - LB: 0, LF: 0, RB: 0, RF: 0")
        self.drive_cmd_label.setAlignment(Qt.AlignCenter)
        driveGroupLayout.addWidget(self.fbLabelDrive)
        driveGroupLayout.addWidget(self.drive_cmd_label)
        mainLayout.addLayout(driveGroupLayout)

        # Setup a label to display key presses (for demonstration)
        self.drive_twist_label = QLabel(
            "Press arrow keys, WASD, or space to interact", self)
        self.drive_twist_label.setAlignment(Qt.AlignCenter)
        mainLayout.addWidget(self.drive_twist_label)

        # Reset Button
        self.resetButton = QPushButton("Reset")
        self.resetButton.clicked.connect(self.resetSliders)
        mainLayout.addWidget(self.resetButton)

        # Arrow Widget
        self.arrowWidget = ArrowWidget()
        mainLayout.addWidget(self.arrowWidget)

        self.setLayout(mainLayout)

    # def keyPressEvent(self, event):
    #     # if event.isAutoRepeat():  # Ignore auto-repeated events
    #     #     return
    #     self.pressedKeys.add(event.key())
    #     print(f"Key pressed: {event.key()}")
    #     self.updateMovement()

    # def keyReleaseEvent(self, event):
    #     if event.isAutoRepeat():  # Ignore auto-repeated events
    #         return
    #     if event.key() in self.pressedKeys:
    #         self.pressedKeys.remove(event.key())
    #     self.updateMovement()

    def keyPressEvent(self, event):
        print(f"Key pressed: {event.key()}")
        self.keyStates[event.key()] = True  # Mark as pressed
        # self.updateMovement()
        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            event.ignore()
            return
        if event.key() in self.keyStates:
            self.keyStates[event.key()] = False  # Mark as not pressed
        # self.updateMovement()
        event.accept()

    def updateMovement(self):
        # Check combinations and adjust accordingly
        # if Qt.Key_Up in self.pressedKeys or Qt.Key_W in self.pressedKeys:
        #     self.keyboard_accumulator_linear += self.keyboard_sensitivity
        # if Qt.Key_Down in self.pressedKeys or Qt.Key_S in self.pressedKeys:
        #     self.keyboard_accumulator_linear -= self.keyboard_sensitivity
        # if Qt.Key_Left in self.pressedKeys or Qt.Key_A in self.pressedKeys:
        #     self.keyboard_accumulator_twist -= self.keyboard_sensitivity
        # if Qt.Key_Right in self.pressedKeys or Qt.Key_D in self.pressedKeys:
        #     self.keyboard_accumulator_twist += self.keyboard_sensitivity
        # if Qt.Key_Space in self.pressedKeys:
        #     self.keyboard_accumulator_linear = 0.0
        #     self.keyboard_accumulator_twist = 0.0
        print(self.keyStates)
        if self.keyStates.get(Qt.Key_Up, False) or self.keyStates.get(Qt.Key_W, False):
            print("Up")
            self.keyboard_accumulator_linear += self.keyboard_sensitivity
        if self.keyStates.get(Qt.Key_Down, False) or self.keyStates.get(Qt.Key_S, False):
            print("Down")
            self.keyboard_accumulator_linear -= self.keyboard_sensitivity
        if self.keyStates.get(Qt.Key_Left, False) or self.keyStates.get(Qt.Key_A, False):
            print("Left")
            self.keyboard_accumulator_twist -= self.keyboard_sensitivity
        if self.keyStates.get(Qt.Key_Right, False) or self.keyStates.get(Qt.Key_D, False):
            print("Right")
            self.keyboard_accumulator_twist += self.keyboard_sensitivity
        if self.keyStates.get(Qt.Key_Space, False) or self.keyStates.get(Qt.Key_0, False):
            self.keyboard_accumulator_linear = 0.0
            self.keyboard_accumulator_twist = 0.0

        # Clamp and apply the velocities
        self.keyboard_accumulator_linear = max(
            min(self.keyboard_accumulator_linear, 1.0), -1.0)
        self.keyboard_accumulator_twist = max(
            min(self.keyboard_accumulator_twist, 1.0), -1.0)
        self.roverLinearVelocity = self.maxLinearVelocity * \
            self.keyboard_accumulator_linear
        self.roverAngularVelocity = self.maxAngularVelocity * \
            self.keyboard_accumulator_twist

        self.apply_velocities()

    def apply_velocities(self):
        # Publish the velocities
        roverTwist = Twist()
        roverTwist.linear.x = self.roverLinearVelocity
        roverTwist.angular.z = self.roverAngularVelocity
        self.drive_twist_publisher.publish(roverTwist)

        # Update GUI
        self.drive_twist_label.setText(
            f"""Linear: {self.roverLinearVelocity:.2f}, Angular: {self.roverAngularVelocity:.2f}""")
        self.arrowWidget.set_velocities(
            self.roverLinearVelocity, self.roverAngularVelocity)

    def decay_velocity(self):
        # Decay the velocity over time
        # print("Decaying velocity")
        self.keyboard_accumulator_linear *= 0.99
        self.keyboard_accumulator_twist *= 0.99
        self.roverLinearVelocity *= 0.99
        self.roverAngularVelocity *= 0.99

        self.apply_velocities()

    def createMotorGroup(self, motorType, jointRanges, increment=1):
        vbox = QVBoxLayout()
        vbox.addWidget(QLabel(f"{motorType} Motor Control"))

        for joint, (minVal, maxVal, initVal, direction) in jointRanges.items():
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
        for joint, (minVal, maxVal, initVal, direction) in self.joint_brushless_lst.items():
            self.sliders[joint].setValue(initVal)
            # Optionally, update the label as well
            index = self.labels.index(
                next(filter(lambda x: x.text().startswith(joint), self.labels))
            )
            self.labels[index].setText(f"{joint} Cmd: {initVal}")

        # Reset brushed sliders
        for joint, (minVal, maxVal, initVal, direction) in self.joint_brushed_lst.items():
            self.sliders[joint].setValue(initVal)
            # Optionally, update the label as well
            index = self.labels.index(
                next(filter(lambda x: x.text().startswith(joint), self.labels))
            )
            self.labels[index].setText(f"{joint} Cmd: {initVal}")

        # Reset drive sliders
        for joint, (minVal, maxVal, initVal, direction) in self.joint_drive_lst.items():
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
        self.arm_brushless_cmd_publisher.publish(cmdMsgBrushless)

        # For brushed motors
        cmdMsgBrushed = Float32MultiArray()
        cmdMsgBrushed.data = [float(slider.value())
                              for key, slider in items[3:6]]
        self.arm_brushed_cmd_publisher.publish(cmdMsgBrushed)

        # For drive motors
        cmdMsgDrive = WheelSpeed()
        # flip is done in odrive_interface
        # cmdMsgDrive.left[0] = items[6][1].value() * \
        #     self.joint_drive_lst["LB"][3]
        # cmdMsgDrive.left[1] = items[7][1].value() * \
        #     self.joint_drive_lst["LF"][3]
        # cmdMsgDrive.right[0] = items[8][1].value() * \
        #     self.joint_drive_lst["RB"][3]
        # cmdMsgDrive.right[1] = items[9][1].value() * \
        #     self.joint_drive_lst["RF"][3]
        cmdMsgDrive.left[0] = items[6][1].value()
        cmdMsgDrive.left[1] = items[7][1].value()
        cmdMsgDrive.right[0] = items[8][1].value()
        cmdMsgDrive.right[1] = items[9][1].value()
        # print(type(items[6][1]))
        self.drive_cmd_publisher.publish(cmdMsgDrive)

        # Update labels
        for i, (joint, slider) in enumerate(items):
            self.labels[i].setText(f"{joint} Cmd: {slider.value()}")

    def on_update_brushless_fb(self, data):
        self.update_arm_fb("Brushless", data)

    def on_update_brushed_fb(self, data):
        self.update_arm_fb("Brushed", data)

    def update_drive_fb(self, data):
        self.fbLabelDrive.setText(
            f"""Drive Fb - LB: {data.left[0] * self.joint_drive_lst["LB"][3]:.2f}, LF: {data.left[1] * self.joint_drive_lst["LF"][3]:.2f}, RB: {data.right[0] * self.joint_drive_lst["RB"][3]:.2f}, RF: {data.right[1] * self.joint_drive_lst["RF"][3]:.2f}""")
        # self.fbLabelDrive.setText(
        #     f"""Drive Fb - LB: {data.left[0]:.2f}, LF: {data.left[1]:.2f}, RB: {data.right[0]:.2f}, RF: {data.right[1]:.2f}""")

    def on_update_drive_cmd(self, data):
        self.drive_cmd_label.setText(
            f"Drive Cmd - LB: {data.left[0]:.2f}, LF: {data.left[1]:.2f}, RB: {data.right[0]:.2f}, RF: {data.right[1]:.2f}")

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
        self.arm_brushless_cmd_publisher = rospy.Publisher(
            "/armBrushlessCmd", Float32MultiArray, queue_size=10
        )
        self.arm_brushed_cmd_publisher = rospy.Publisher(
            "/armBrushedCmd", Float32MultiArray, queue_size=10
        )
        self.drive_cmd_publisher = rospy.Publisher(
            "/wheel_velocity_cmd", WheelSpeed, queue_size=10
        )
        self.drive_twist_publisher = rospy.Publisher(
            "rover_velocity_controller/cmd_vel", Twist, queue_size=1
        )

        rospy.Subscriber(
            "/wheel_velocity_cmd", WheelSpeed, self.on_update_drive_cmd)
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
