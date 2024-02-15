#!/usr/bin/env python3
from rospy import Subscriber
from std_msgs.msg import Float32MultiArray

class AngularGamepad():
    """Driver to get the event inputs from the Logitech Extreme 3D Pro Gamepad
    """
    data = None

    def __init__(self, topic_name = "/angular_ui_app/drive") -> None:
        self.data = GamepadData()
        Subscriber(topic_name, Float32MultiArray, self.gamepad_callback)


    def gamepad_callback(self, msg: Float32MultiArray):
        """Update the gamepad data from the callback"""
        self.data.a2 = msg.data[0]
        self.data.a4 = msg.data[1]

    def update(self):
        """Do nothing"""
        pass

class GamepadData():
    """Object containing mapping of the Logitech Extreme 3D Pro
    """

    def __init__(self):
        # Buttons
        self.b1 = 0
        self.b2 = 0
        self.b3 = 0
        self.b4 = 0
        self.b5 = 0
        self.b6 = 0
        self.b7 = 0
        self.b8 = 0
        self.b9 = 0
        self.b10 = 0
        self.b11 = 0
        self.b12 = 0
        self.b13 = 0

        # Axis
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0
        self.a5 = 0
        self.a6 = 0
