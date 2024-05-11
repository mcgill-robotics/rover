# Author: mn297
from drive_control.msg import WheelSpeed
from odrive_interface.msg import MotorState, MotorError, ODriveStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from enum import Enum
from odrive.enums import AxisState, ODriveError, ProcedureResult
from odrive.utils import dump_errors
from ODriveJoint import *
import threading
from threading import Lock

import rospy
import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)


class NodeODriveInterfaceDrive:
    def __init__(self):
        self.is_homed = False
        self.is_calibrated = False

        # CONFIGURATION ---------------------------------------------------------------
        # Serial number of the ODrive controlling the joint
        self.joint_serial_numbers = {
            "rover_drive_rf": "384F34683539",
            "rover_drive_lf": "386134503539",
            "rover_drive_rb": "387134683539",
            "rover_drive_lb": "385C347A3539"
        }

        # VARIABLES -------------------------------------------------------------------
        # Dictionary of ODriveJoint objects, key is the joint name in string format, value is the ODriveJoint object
        self.joint_dict = {
            "rover_drive_rf": None,
            "rover_drive_lf": None,
            "rover_drive_rb": None,
            "rover_drive_lb": None
        }
        self.locks = {joint_name: Lock()
                      for joint_name in self.joint_dict.keys()}

        # Subscriptions
        rospy.init_node("odrive_interface_drive")
        # Cmd comes from the external control node, we convert it to setpoint and apply it to the ODrive
        self.drive_cmd_subscriber = rospy.Subscriber("/wheel_velocity_cmd", WheelSpeed, self.handle_drive_cmd)

        # Publishers
        self.drive_fb_publisher = rospy.Publisher("/wheel_velocity_feedback", WheelSpeed, queue_size=1)

        self.odrive_publisher = rospy.Publisher("/odrive_state", ODriveStatus, queue_size=1)

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    # Receive setpoint from external control node
    def handle_drive_cmd(self, msg):
        if not self.is_calibrated:
            print("ODrive not calibrated. Ignoring command.")
            return
        self.joint_dict["rover_drive_lb"].vel_cmd = msg.left[0]
        self.joint_dict["rover_drive_lf"].vel_cmd = msg.left[1]
        self.joint_dict["rover_drive_rb"].vel_cmd = msg.right[0]
        self.joint_dict["rover_drive_rf"].vel_cmd = msg.right[1]

    def reconnect_joint(self, joint_name, joint_obj):
        # Attempt to reconnect...
        # Update joint_obj.odrv as necessary...
        joint_obj.reconnect()

        # Once done, reset the flag
        with self.locks[joint_name]:
            joint_obj.is_reconnecting = False

    def calibrate_and_enter_closed_loop(self, joint_obj):
        if joint_obj.odrv is not None:
            print(f"CALIBRATING joint {joint_obj.name}...")
            joint_obj.calibrate()

            print(
                f"ENTERING CLOSED LOOP CONTROL for joint {joint_obj.name}...")
            joint_obj.enter_closed_loop_control()

    def run(self):
        threads = []
        calibrate_threads = []
        enter_closed_loop_threads = []

        # SETUP ODRIVE CONNECTIONS -----------------------------------------------------
        for key, value in self.joint_serial_numbers.items():
            # Instantiate ODriveJoint class whether or not the connection attempt was made/successful
            self.joint_dict[key] = ODriveJoint(name=key, serial_number=value)

            if value == 0:
                print(
                    f"""Skipping connection for joint: {
                        key} due to serial_number being 0"""
                )
                continue

            self.joint_dict[key].reconnect()

        print("Connection step completed.")

        for joint_name, joint_obj in self.joint_dict.items():
            if joint_obj.odrv is None:
                continue
            print(f"Creating thread for joint {joint_obj.name}")
            t = threading.Thread(
                target=joint_obj.calibrate
            )
            calibrate_threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in calibrate_threads:
            t.join()

        print("Calibration step completed.")

        for joint_name, joint_obj in self.joint_dict.items():
            print(f"Creating thread for joint {joint_obj.name}")
            t = threading.Thread(
                target=joint_obj.enter_closed_loop_control
            )
            enter_closed_loop_threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in enter_closed_loop_threads:
            t.join()

        self.is_calibrated = True
        print("Enter closed loop control step completed.")

        # Set the direction of the motors
        self.joint_dict["rover_drive_lb"].direction = -1
        self.joint_dict["rover_drive_lf"].direction = -1
        self.joint_dict["rover_drive_rb"].direction = 1
        self.joint_dict["rover_drive_rf"].direction = 1

        print("Entering closed loop control step completed.")

        # MAIN LOOP -----------------------------------------------------
        while not rospy.is_shutdown():
            # PRINT TIMESTAMP
            print(f"""Time: {rospy.get_time()}""")

            # PUBLISH Odrive velocity FB
            feedback = WheelSpeed()
            for joint_name, joint_obj in self.joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    self.joint_dict[joint_name].vel_fb = (
                        joint_obj.odrv.encoder_estimator0.vel_estimate
                    )
                except fibre.libfibre.ObjectLostError:
                    joint_obj.odrv = None
                except:
                    print(f"""Cannot get feedback from joint: {joint_name}""")

            # Publish
            feedback.left[0] = self.joint_dict["rover_drive_lb"].vel_fb
            feedback.left[1] = self.joint_dict["rover_drive_lf"].vel_fb
            feedback.right[0] = self.joint_dict["rover_drive_rb"].vel_fb
            feedback.right[1] = self.joint_dict["rover_drive_rf"].vel_fb
            self.drive_fb_publisher.publish(feedback)

            # APPLY Velocity CMD
            for joint_name, joint_obj in self.joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    # setpoint in rev/s
                    joint_obj.odrv.axis0.controller.input_vel = (
                        joint_obj.vel_cmd * joint_obj.direction
                    )
                except fibre.libfibre.ObjectLostError:
                    joint_obj.odrv = None
                except:
                    print(
                        f"""Cannot apply vel_cmd {
                            joint_obj.vel_cmd} to joint: {joint_name}"""
                    )

            # PRINT POSITIONS TO CONSOLE
            print_joint_state_from_dict(self.joint_dict)

            # SEND ODRIVE STATUS AND HANDLE ERRORS
            # TODO test reconnecting
            for joint_name, joint_obj in self.joint_dict.items():
                with self.locks[joint_name]:  # Use a lock specific to each joint
                    if joint_obj.odrv is not None:
                        # Check if 'odrv' object has 'axis0' attribute before accessing it
                        if hasattr(joint_obj.odrv, 'axis0') and joint_obj.odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
                            if not joint_obj.is_reconnecting:
                                print(
                                    f"{joint_name} is not in closed loop control, recalibrating...")
                                joint_obj.is_reconnecting = True
                                t = threading.Thread(
                                    target=self.calibrate_and_enter_closed_loop, args=(joint_obj,))
                                t.start()
                        else:
                            # Handle case where 'axis0' is not available
                            print(
                                f"Cannot check current state for {joint_name}, 'axis0' attribute missing.")
                    else:
                        # ODrive interface is None, indicating a disconnection or uninitialized state
                        if not joint_obj.is_reconnecting:
                            print(f"RECONNECTING {joint_name}...")
                            joint_obj.is_reconnecting = True
                            t = threading.Thread(
                                target=self.reconnect_joint, args=(joint_name, joint_obj))
                            t.start()

            self.rate.sleep()

        # On shutdown, bring motors to idle state
        print("Shutdown prompt received. Setting all motors to idle state.")
        for joint_name, joint_obj in self.joint_dict.items():
            if not joint_obj.odrv:
                continue
            joint_obj.odrv.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    driver = NodeODriveInterfaceDrive()
    rospy.spin()
