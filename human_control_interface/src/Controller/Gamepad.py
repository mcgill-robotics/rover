#!/usr/bin/env python3

import pygame
import time
import rospy
from human_control_interface.msg import Gamepad_input

class Node_Gamepad():
    """Gets gamepad data and publishes the data to the gamepad_data topic

    Topic data : Float32MultiArray
    Data:
            Input   | Possible Value    | Mapping
        --------------------------------------------
        Button 1    :   0 or 1          : X / A
        Button 2    :   0 or 1          : O / B
        Button 3    :   0 or 1          : triangle / Y
        Button 4    :   0 or 1          : square / X
        Button 5    :   0 or 1          : LB
        Button 6    :   0 or 1          : RB
        Button 7    :   0 or 1          : LT
        Button 8    :   0 or 1          : RT
        Button 9    :   0 or 1          : Select
        Button 10   :   0 or 1          : Start
        Button 11   :   0 or 1          : Home
        Button 12   :   0 or 1          : L-stick press
        Button 13   :   0 or 1          : R-stick press
        Axis 1      : Range [-1, 1]     : L-stick L/R
        Axis 2      : Range [-1, 1]     : L-stick U/D
        Axis 3      : Range [-1, 1]     : LT - analog
        Axis 4      : Range [-1, 1]     : R-stick L/R
        Axis 5      : Range [-1, 1]     : R-stick U/D
        Axis 6      : Range [-1, 1]     : RT - analog

    """
    def __init__(self):
        # Initialize Gamepad
        try:
            self.gamepad = Gamepad()
        except AssertionError as error:
            rospy.logerr(str(error))

        # Initialize ROS
        rospy.init_node("gamepad", anonymous=False)
        self.gamepad_publisher = rospy.Publisher("gamepad_data", Gamepad_input, queue_size=1)

        # Start Node
        self.run()

    def run(self):
        """Runs the node loop for getting and updating the Gamepad information for user control
        """
        while not rospy.is_shutdown():
            if rospy.is_shutdown():
                exit()
            try:
                self.gamepad.update()
                msg = Gamepad_input()

                # Transfer Data into msg
                msg.B1 = self.gamepad.data.b1
                msg.B2 = self.gamepad.data.b2
                msg.B3 = self.gamepad.data.b3
                msg.B4 = self.gamepad.data.b4
                msg.B5 = self.gamepad.data.b5
                msg.B6 = self.gamepad.data.b6
                msg.B7 = self.gamepad.data.b7
                msg.B8 = self.gamepad.data.b8
                msg.B9 = self.gamepad.data.b9
                msg.B10 = self.gamepad.data.b10
                msg.B11 = self.gamepad.data.b11
                msg.B12 = self.gamepad.data.b12
                msg.B13 = self.gamepad.data.b13
                msg.A1 = self.gamepad.data.a1
                msg.A2 = self.gamepad.data.a2
                msg.A3 = self.gamepad.data.a3
                msg.A4 = self.gamepad.data.a4
                msg.A5 = self.gamepad.data.a5
                msg.A6 = self.gamepad.data.a6


                self.gamepad_publisher.publish(msg)

                time.sleep(0.1)
            except Exception as error:
                rospy.logerr(str(error))
        exit()


class Gamepad():
    """Driver to get the event inputs from the Logitech Extreme 3D Pro Gamepad
    """
    def __init__(self):
        """Constructor for the Gamepad Driver

        Raises
        --------
            AssertionError
                Gamepad wasn't able to be initialized
        """
        # Data container for the Gamepad input data
        self.data = GamepadData()

        # Initialize Gamepad
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 1:
            # Take the only Gamepad available
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
        else:
            # Either no Gamepad found or multiple detected (currently unsupported)
            self.controller = None

        if self.controller is None:
            raise AssertionError("Gamepad not initialized properly, make sure you have one connected")

    def update(self):
        """Gets the latest data from the Gamepad from event information received from the Gamepad
        """
        for an_event in pygame.event.get():
            try:
                # Get event information
                if an_event.type == pygame.JOYBUTTONDOWN or an_event.type == pygame.JOYBUTTONUP:
                    self.data.b1 = self.controller.get_button(0)
                    self.data.b2 = self.controller.get_button(1)
                    self.data.b3 = self.controller.get_button(2)
                    self.data.b4 = self.controller.get_button(3)
                    self.data.b5 = self.controller.get_button(4)
                    self.data.b6 = self.controller.get_button(5)
                    self.data.b7 = self.controller.get_button(6)
                    self.data.b8 = self.controller.get_button(7)
                    self.data.b9 = self.controller.get_button(8)
                    self.data.b10 = self.controller.get_button(9)
                    self.data.b11 = self.controller.get_button(10)
                    self.data.b12 = self.controller.get_button(11)
                    self.data.b13 = self.controller.get_button(12)

                elif an_event.type == pygame.JOYAXISMOTION:
                    self.data.a1 = self.controller.get_axis(0)
                    self.data.a2 = -1 * self.controller.get_axis(1)
                    self.data.a3 = self.controller.get_axis(2)
                    self.data.a4 = self.controller.get_axis(3)
                    self.data.a5 = -1 * self.controller.get_axis(4)
                    self.data.a6 = self.controller.get_axis(5)

            except pygame.error:
                pass
            finally:
                pass

    def printData(self):
        """Sends copy of Gamepad data to the standard output (sys.out)
        """
        data = ''
        # Buttons
        data += f"B1 {self.data.b1}| "
        data += f"B2 {self.data.b2}| "
        data += f"B3 {self.data.b3}| "
        data += f"B4 {self.data.b4}| "
        data += f"B5 {self.data.b5}| "
        data += f"B6 {self.data.b6}| "
        data += f"B7 {self.data.b7}| "
        data += f"B8 {self.data.b8}| "
        data += f"B9 {self.data.b9}| "
        data += f"B10 {self.data.b10}| "
        data += f"B11 {self.data.b11}| "
        data += f"B12 {self.data.b12}| "
        data += f"B13 {self.data.b13}| "

        # Axis
        data += f"A1 {self.data.a1}| "
        data += f"A2 {self.data.a2}| "
        data += f"A3 {self.data.a3}| "
        data += f"A4 {self.data.a4}| "
        data += f"A5 {self.data.a5}| "
        data += f"A6 {self.data.a6}| "
        
        print(data)

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

if __name__ == "__main__":
    driver = Node_Gamepad()
    rospy.spin()