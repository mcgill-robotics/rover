# Author: mn297
import sys
import os

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
sys.path.append(currentdir + "/../..")

# try statement to force it to stay below the directory settings throughout autoformatting
try:
    from collections import namedtuple
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
        self.max_linear_vel = 3
        self.max_angular_vel = 10
        self.setMinimumSize(200, 200)  # Ensure widget is large enough

    def set_velocities(self, linear, angular):
        self.linear_velocity = linear
        self.angular_velocity = angular
        self.update()  # Trigger a repaint to show updated velocities

    # OVERRIDE, don't change name
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
        normalized_linear_vel = self.linear_velocity / self.max_linear_vel
        # Normalize angular velocity and calculate rotation angle
        normalized_angular_vel = self.angular_velocity / self.max_angular_vel
        angle_degrees = normalized_angular_vel * max_angle_degrees

        if abs(self.linear_velocity) < velocity_threshold:
            dot_radius = 3  # Draw a dot for zero/near-zero velocities
            qp.drawEllipse(center, dot_radius, dot_radius)
        else:
            # Calculate arrow length from normalized linear velocity, ensuring minimum visibility
            arrow_length = max(abs(normalized_linear_vel) * max_pixel_length, 3)

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
            qp.drawLine(center.x(), center.y(), int(end_point_x), int(end_point_y))


# Define a namedtuple for joint properties
Joint = namedtuple("Joint", ["min_val", "max_val", "init_val", "direction"])


class ArmControlGUI(QWidget):
    def __init__(self):
        self.sliders = {}
        self.labels = {}
        self.downButtons = []
        self.upButtons = []
        self.joints = {
            "brushless": {
                "arm_brushless_elbow": Joint(-30, 30, 0, 1),
                "arm_brushless_shoulder": Joint(-30, 30, 0, 1),
                "arm_brushless_waist": Joint(-50, 50, 0, 1),
            },
            "brushed": {
                "arm_brushed_end_effector": Joint(-100, 100, 0, 1),
                "arm_brushed_wrist_roll": Joint(-100, 100, 0, 1),
                "arm_brushed_wrist_pitch": Joint(-30, 30, 0, 1),
            },
            "drive": {
                "drive_lb": Joint(-200, 200, 0, -1),
                "drive_lf": Joint(-200, 200, 0, -1),
                "drive_rb": Joint(-200, 200, 0, 1),
                "drive_rf": Joint(-200, 200, 0, 1),
            },
        }

        super(ArmControlGUI, self).__init__()
        self.initUI()
        self.ros_setup()

        # Set up the keyboard listener thread
        # self.keyboardListenerThread = KeyboardListenerThread()
        # self.keyboardListenerThread.start()
        # print("Keyboard listener started")

        self.rover_linear_vel = 0.0
        self.rover_angular_vel = 0.0
        self.keyboard_accumulator_linear = 0.0
        self.keyboard_accumulator_twist = 0.0
        self.keyboard_sensitivity = 0.01
        self.max_linear_vel = 10
        self.max_angular_vel = 10

        # Setup movement timer
        self.movement_timer = QTimer(self)
        self.movement_timer.timeout.connect(self.update_movement)
        self.movement_timer.start(10)  # Update every 10ms

        # Setup the decay timer
        self.decay_timer = QTimer(self)
        self.decay_timer.timeout.connect(self.decay_velocity)
        self.decay_timer.start(10)  # Decay every 10ms

        self.pressedKeys = set()  # Set to keep track of pressed keys
        self.keyStates = {}  # Tracks the press state of each key

    def initUI(self):
        main_layout = QHBoxLayout()

        # Brushless group
        brushless_group_layout = self.create_motor_group(
            "Brushless", self.joints["brushless"]
        )
        self.brushless_outshaft_fb_label = QLabel(
            "Fb Brushless - Elbow: 0, Shoulder: 0, Waist: 0"
        )
        self.brushless_outshaft_fb_label.setAlignment(Qt.AlignCenter)
        self.brushless_odrive_fb_label = QLabel(
            "Odrive Pos Fb - Elbow: 0, Shoulder: 0, Waist: 0"
        )
        self.brushless_odrive_fb_label.setAlignment(Qt.AlignCenter)
        brushless_group_layout.addWidget(self.brushless_outshaft_fb_label)
        brushless_group_layout.addWidget(self.brushless_odrive_fb_label)

        # Add reset button for Brushless group
        self.resetBrushlessButton = QPushButton("Reset Brushless")
        self.resetBrushlessButton.clicked.connect(
            lambda: self.resetSliders("brushless")
        )
        brushless_group_layout.addWidget(self.resetBrushlessButton)

        main_layout.addLayout(brushless_group_layout)

        # Brushed group
        brushed_group_layout = self.create_motor_group(
            "Brushed", self.joints["brushed"]
        )
        self.fbLabelBrushed = QLabel("Fb Brushed - Elbow: 0, Shoulder: 0, Waist: 0")
        self.fbLabelBrushed.setAlignment(Qt.AlignCenter)
        brushed_group_layout.addWidget(self.fbLabelBrushed)

        # Add reset button for Brushed group
        self.resetBrushedButton = QPushButton("Reset Brushed")
        self.resetBrushedButton.clicked.connect(lambda: self.resetSliders("brushed"))
        brushed_group_layout.addWidget(self.resetBrushedButton)

        main_layout.addLayout(brushed_group_layout)

        # Drive group
        drive_group_layout = self.create_motor_group("Drive", self.joints["drive"])
        self.fbLabelDrive = QLabel("Drive Fb - LB: 0, LF: 0, RB: 0, RF: 0")
        self.fbLabelDrive.setAlignment(Qt.AlignCenter)
        self.drive_cmd_label = QLabel("Drive Cmd - LB: 0, LF: 0, RB: 0, RF: 0")
        self.drive_cmd_label.setAlignment(Qt.AlignCenter)
        self.drive_twist_label = QLabel(
            "Press arrow keys, WASD, or space to interact", self
        )
        self.drive_twist_label.setAlignment(Qt.AlignCenter)
        drive_group_layout.addWidget(self.fbLabelDrive)
        drive_group_layout.addWidget(self.drive_cmd_label)
        drive_group_layout.addWidget(self.drive_twist_label)

        # Add reset button for Drive group
        self.resetDriveButton = QPushButton("Reset Drive")
        self.resetDriveButton.clicked.connect(lambda: self.resetSliders("drive"))
        drive_group_layout.addWidget(self.resetDriveButton)

        main_layout.addLayout(drive_group_layout)

        # Add total reset button
        self.totalResetButton = QPushButton("Total Reset")
        self.totalResetButton.clicked.connect(lambda: self.resetSliders("all"))
        main_layout.addWidget(self.totalResetButton)

        # Arrow widget
        self.arrow_widget = ArrowWidget()
        main_layout.addWidget(self.arrow_widget)

        self.setLayout(main_layout)

    # OVERRIDE, don't change name
    def keyPressEvent(self, event):
        print(f"Key pressed: {event.key()}")
        self.keyStates[event.key()] = True  # Mark as pressed
        event.accept()

    # OVERRIDE, don't change name
    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            event.ignore()
            return
        if event.key() in self.keyStates:
            self.keyStates[event.key()] = False  # Mark as not pressed
        event.accept()

    def update_movement(self):
        # print(self.keyStates)

        self.keyboard_accumulator_linear = 0.0
        self.keyboard_accumulator_twist = 0.0

        if self.keyStates.get(Qt.Key_Up, False) or self.keyStates.get(Qt.Key_W, False):
            print("Up")
            self.keyboard_accumulator_linear += self.keyboard_sensitivity
        if self.keyStates.get(Qt.Key_Down, False) or self.keyStates.get(
            Qt.Key_S, False
        ):
            print("Down")
            self.keyboard_accumulator_linear -= self.keyboard_sensitivity
        if self.keyStates.get(Qt.Key_Left, False) or self.keyStates.get(
            Qt.Key_A, False
        ):
            print("Left")
            self.keyboard_accumulator_twist -= self.keyboard_sensitivity
        if self.keyStates.get(Qt.Key_Right, False) or self.keyStates.get(
            Qt.Key_D, False
        ):
            print("Right")
            self.keyboard_accumulator_twist += self.keyboard_sensitivity
        if self.keyStates.get(Qt.Key_Space, False) or self.keyStates.get(
            Qt.Key_0, False
        ):
            print("Stop")
            self.rover_linear_vel = 0.0
            self.rover_angular_vel = 0.0

        # Clamp and apply the velocities
        self.keyboard_accumulator_linear = max(
            min(self.keyboard_accumulator_linear, 1.0), -1.0
        )
        self.keyboard_accumulator_twist = max(
            min(self.keyboard_accumulator_twist, 1.0), -1.0
        )
        self.rover_linear_vel += self.max_linear_vel * self.keyboard_accumulator_linear
        self.rover_angular_vel += self.max_angular_vel * self.keyboard_accumulator_twist

        self.publish_drive_twist()

    def publish_drive_twist(self):
        # Publish the velocities
        roverTwist = Twist()
        roverTwist.linear.x = self.rover_linear_vel
        roverTwist.angular.z = self.rover_angular_vel

        # Flip the angular velocity if the linear velocity is negative, similar to WASD controls
        # if self.rover_linear_vel < 0:
        #     roverTwist.angular.z = self.rover_angular_vel * -1

        self.drive_twist_publisher.publish(roverTwist)

        # Update GUI
        self.drive_twist_label.setText(
            f"""Twist - Linear: {self.rover_linear_vel:.2f}, Angular: {self.rover_angular_vel:.2f}"""
        )
        self.arrow_widget.set_velocities(self.rover_linear_vel, self.rover_angular_vel)

    def decay_velocity(self):
        # Decay the velocity over time
        # print("Decaying velocity")
        # self.keyboard_accumulator_linear *= 0.96
        # self.keyboard_accumulator_twist *= 0.96
        is_accelerating_linear = False
        is_accelerating_twist = False
        if self.keyStates.get(Qt.Key_Up, False) or self.keyStates.get(Qt.Key_W, False):
            is_accelerating_linear = True
        if self.keyStates.get(Qt.Key_Down, False) or self.keyStates.get(
            Qt.Key_S, False
        ):
            is_accelerating_linear = True
        if self.keyStates.get(Qt.Key_Left, False) or self.keyStates.get(
            Qt.Key_A, False
        ):
            is_accelerating_twist = True
        if self.keyStates.get(Qt.Key_Right, False) or self.keyStates.get(
            Qt.Key_D, False
        ):
            is_accelerating_twist = True

        if is_accelerating_linear:
            self.rover_linear_vel *= 0.96
        else:
            self.rover_linear_vel *= 0.9

        if is_accelerating_twist:
            self.rover_angular_vel *= 0.99
        else:
            self.rover_angular_vel *= 0.9

        self.publish_drive_twist()

    def create_joint_control(self, joint_name, joint_properties, group_layout):
        hbox = QHBoxLayout()

        down_btn = QPushButton("-")
        self.downButtons.append(down_btn)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(joint_properties.min_val)
        slider.setMaximum(joint_properties.max_val)
        slider.setValue(joint_properties.init_val)
        slider.valueChanged[int].connect(
            lambda value, j=joint_name: self.onChangeCmd(j)
        )
        self.sliders[joint_name] = slider

        up_btn = QPushButton("+")
        self.upButtons.append(up_btn)

        label = QLabel(f"{joint_name} Cmd: {joint_properties.init_val}")
        self.labels[joint_name] = label

        down_btn.clicked.connect(lambda checked, j=joint_name: self.adjustCmd(j, -1))
        up_btn.clicked.connect(lambda checked, j=joint_name: self.adjustCmd(j, 1))

        hbox.addWidget(down_btn)
        hbox.addWidget(slider)
        hbox.addWidget(up_btn)

        group_layout.addWidget(label)
        group_layout.addLayout(hbox)

    # VBox so the joints are stacked vertically
    def create_motor_group(self, motor_type, joint_dict):
        vbox = QVBoxLayout()
        vbox.addWidget(QLabel(f"{motor_type} Motor Control"))

        for joint_name, joint_properties in joint_dict.items():
            self.create_joint_control(joint_name, joint_properties, vbox)

        return vbox

    def resetSliders(self, group):
        if group == "all":
            for joint_category in self.joints.values():
                for joint, props in joint_category.items():
                    self.sliders[joint].setValue(props.init_val)
                    self.labels[joint].setText(f"{joint} Cmd: {props.init_val}")
        else:
            for joint, props in self.joints[group].items():
                self.sliders[joint].setValue(props.init_val)
                self.labels[joint].setText(f"{joint} Cmd: {props.init_val}")

    def adjustCmd(self, joint, delta):
        # Determine the joint's range based on whether it's in the brushless, brushed, or drive list.
        joint_props = None
        for joint_category in self.joints.values():
            if joint in joint_category:
                joint_props = joint_category[joint]
                break

        if not joint_props:
            print(f"Joint '{joint}' not found in any motor list.")
            return

        # Get the slider associated with the joint.
        slider = self.sliders[joint]

        # Calculate the new value, ensuring it stays within the specified range.
        newValue = max(
            joint_props.min_val, min(joint_props.max_val, slider.value() + delta)
        )

        # Set the new value to the slider.
        slider.setValue(newValue)

        # No need for manual call to onChangeCmd, as the valueChanged signal will trigger it.
        # self.onChangeCmd(joint)

    def onChangeCmd(self, joint):
        print(f"Cmd changed for {joint}, new value: {self.sliders[joint].value()}")
        items = list(self.sliders.items())

        if joint in self.joints["brushless"]:
            cmdMsgBrushless = Float32MultiArray()
            cmdMsgBrushless.data = [float(slider.value()) for key, slider in items[0:3]]
            self.arm_brushless_cmd_publisher.publish(cmdMsgBrushless)

        elif joint in self.joints["brushed"]:
            cmdMsgBrushed = Float32MultiArray()
            cmdMsgBrushed.data = [float(slider.value()) for key, slider in items[3:6]]
            self.arm_brushed_cmd_publisher.publish(cmdMsgBrushed)

        elif joint in self.joints["drive"]:
            cmdMsgDrive = WheelSpeed()
            cmdMsgDrive.left[0] = items[6][1].value()
            cmdMsgDrive.left[1] = items[7][1].value()
            cmdMsgDrive.right[0] = items[8][1].value()
            cmdMsgDrive.right[1] = items[9][1].value()
            self.drive_cmd_publisher.publish(cmdMsgDrive)

        for key, slider in self.sliders.items():
            self.labels[key].setText(f"{key} Cmd: {slider.value()}")

    def on_update_brushless_outshaft_fb(self, data):
        names = list(self.joints["brushless"].keys())
        fbText = f"""Fb Brushless - {names[0]}: {data.data[0]:.2f}, {names[1]}: {data.data[1]:.2f}, {names[2]}: {data.data[2]:.2f}"""
        # print(fbText)
        self.brushless_outshaft_fb_label.setText(fbText)

    def on_update_brushless_odrive_fb_odrive(self, data):
        fbText = f"""Odrive Pos Fb - Elbow: {data.data[0]:.2f}, Shoulder: {data.data[1]:.2f}, Waist: {data.data[2]:.2f}"""
        # print(fbText)
        self.brushless_odrive_fb_label.setText(fbText)

    def on_update_brushed_fb(self, data):
        names = list(self.joints["brushed"].keys())
        fbText = f"""Fb Brushed - {names[0]}: {data.data[0]:.2f}, {names[1]}: {data.data[1]:.2f}, {names[2]}: {data.data[2]:.2f}"""
        # print(fbText)
        self.fbLabelBrushed.setText(fbText)  # Corrected to update the brushed label

    def update_drive_fb(self, data):
        fbText = (
            f"""Drive Fb - LB: {data.left[0] * self.joints['drive']['drive_lb'].direction:.2f}, """
            f"""LF: {data.left[1] * self.joints['drive']['drive_lf'].direction:.2f}, """
            f"""RB: {data.right[0] * self.joints['drive']['drive_rb'].direction:.2f}, """
            f"""RF: {data.right[1] * self.joints['drive']['drive_rf'].direction:.2f}"""
        )
        # print(fbText)
        self.fbLabelDrive.setText(fbText)

    def on_update_drive_cmd(self, data):
        cmdText = (
            f"""Drive Cmd - LB: {data.left[0]:.2f}, """
            f"""LF: {data.left[1]:.2f}, """
            f"""RB: {data.right[0]:.2f}, """
            f"""RF: {data.right[1]:.2f}"""
        )
        print(cmdText)
        self.drive_cmd_label.setText(cmdText)

    def ros_setup(self):
        rospy.init_node("arm_control_gui", anonymous=True)

        # Publishers
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

        # Subscribers
        rospy.Subscriber("/wheel_velocity_cmd", WheelSpeed, self.on_update_drive_cmd)
        rospy.Subscriber(
            "/armBrushlessFb", Float32MultiArray, self.on_update_brushless_outshaft_fb
        )
        rospy.Subscriber(
            "/odrive_armBrushlessFb",
            Float32MultiArray,
            self.on_update_brushless_odrive_fb_odrive,
        )
        rospy.Subscriber("/armBrushedFb", Float32MultiArray, self.on_update_brushed_fb)
        rospy.Subscriber("/wheel_velocity_feedback", WheelSpeed, self.update_drive_fb)


def main():
    app = QApplication(sys.argv)
    gui = ArmControlGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
