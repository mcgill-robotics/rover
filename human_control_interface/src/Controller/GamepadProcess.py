#!/usr/bin/env python3
import rospy
from human_control_interface.msg import Gamepad_input
from camera_data.msg import Camera_Orientation
from geometry_msgs.msg import Twist
from Gamepad import *
from std_msgs.msg import Float32MultiArray



class Node_GamepadProcessing:
    def __init__(self, v_max, w_max):
        """
        The member variables of the GamepadProcess object

        gamepad: Reference to a Gamepad object.

        roverLinearVelocity: The rover's linear velocity
        roverAngularVelocity: The rover's angular velocity
        maxLinearVelocity: The upper limit set for linear velocity.
        maxAngularVelocity: The lower limit set for angular velocity

        NOTE: Twist measurements are in SI units. (m/s, m, ...)

        """
        # initialize ROS node
        rospy.init_node("gamepad_process_node")
        
        # Initialize a Gamepad object
        self.gamepad = Gamepad()


        # initialize variables for velocity
        self.roverLinearVelocity = 0
        self.roverAngularVelocity = 0

        # initialize variables for rover's max linear and angular velocities
        self.maxLinearVelocity = v_max
        self.maxAngularVelocity = w_max

        # Initialize variables for camera
        self.cam_ctrl = Float32MultiArray()
        self.cam_ctrl.data = [0, 0] # Elements: [X-axis, Y-axis]

        self.drive_publisher = rospy.Publisher("rover_velocity_controller/cmd_vel", Twist, queue_size=1) # Publisher for twist values.
        self.camera_publisher = rospy.Publisher("panTiltAngles", Float32MultiArray, queue_size=1) # Publisher for pan tilt camera angles.

        # Control frequency of the node
        self.rate = rospy.Rate(100)

        self.run()



    # The run loop that updates a controller's value.
    def run(self):
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
                #Passes msg (gamepad data) to gamepadProcessCall
                self.gamepadProcessCall(msg)
            except Exception as error:
                    rospy.logerr(str(error))

            self.rate.sleep()

        exit()
    
    # Poll the gamepad data and then call the respective process call.
    def gamepadProcessCall(self, msg):
        self.driveProcessCall(msg)
        self.cameraProcessCall(msg)

    def driveProcessCall(self, msg):
        
        # A2 is the left stick moving up and down. Drives forwards or backwards.
        # A4 is the right stick that moves left and right. Steers left or right.

        if abs(msg.A4) < 0.1:
            msg.A4 = 0
        if abs(msg.A2) < 0.1:
            msg.A2 = 0

        drive = msg.A2
        steer = msg.A4

        # calc. for linear velocity
        self.roverLinearVelocity = self.maxLinearVelocity * drive

        # calc. for angular velocity
        self.roverAngularVelocity = self.maxAngularVelocity * steer

        # Assigns values to a Twist msg, then publish it to ROS
        roverTwist = Twist()
        roverTwist.linear.x = self.roverLinearVelocity
        roverTwist.angular.z = self.roverAngularVelocity

        self.drive_publisher.publish(roverTwist)


    def cameraProcessCall(self, msg):
        # up: triangle
        # down: X
        # right: O
        # left: square

        if (msg.B3 == 1 or msg.B1 == 1) and (self.cam_ctrl.data[0] + msg.B3 <= 180) and (self.cam_ctrl.data[0] - msg.B1 >= 0):
            self.cam_ctrl.data[0] = self.cam_ctrl.data[0] + msg.B3 - msg.B1
            print(f"vertical: {self.cam_ctrl.data[0]}")

        if (msg.B4 == 1 or msg.B2 == 1) and (self.cam_ctrl.data[1] + msg.B2 <= 180) and (self.cam_ctrl.data[1] - msg.B4 >= 0):
            self.cam_ctrl.data[1] = self.cam_ctrl.data[1] + msg.B2 - msg.B4
            print(f"horizontal: {self.cam_ctrl.data[1]}")
        
        self.camera_publisher.publish(self.cam_ctrl)
        

    def risingEdge(self, prevSignal, nextSignal):
        if prevSignal < nextSignal:
            return True
        else: 
            return False
        


if __name__ == "__main__":
    gamepadProcess = Node_GamepadProcessing(1, 3)
    #rospy.spin()
