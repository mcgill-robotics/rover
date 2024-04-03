import scipy.stats as st
from scipy.ndimage import gaussian_filter1d
from drive_control.msg import WheelSpeed
from odrive_interface.msg import MotorState, MotorError, ODriveStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from enum import Enum
from odrive.enums import AxisState, ODriveError, ProcedureResult
from odrive.utils import dump_errors
from ODriveJoint import *
import threading
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
            "rover_drive_lb": "385C347A3539",
        }

        # VARIABLES -------------------------------------------------------------------
        # Dictionary of ODriveJoint objects, key is the joint name in string format, value is the ODriveJoint object
        self.joint_dict = {
            "rover_drive_rf": None,
            "rover_drive_lf": None,
            "rover_drive_rb": None,
            "rover_drive_lb": None,
        }

        # Subscriptions
        rospy.init_node("odrive_interface_drive")
        # Cmd comes from the external control node, we convert it to setpoint and apply it to the ODrive
        self.drive_cmd_subscriber = rospy.Subscriber(
            "/wheel_velocity_cmd", WheelSpeed, self.handle_drive_cmd
        )

        # Publishers
        self.drive_fb_publisher = rospy.Publisher(
            "/wheel_velocity_feedback", WheelSpeed, queue_size=1
        )

        self.odrive_publisher = rospy.Publisher(
            "/odrive_state", ODriveStatus, queue_size=1
        )

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    # Receive setpoint from external control node
    def handle_drive_cmd(self, msg):
        self.joint_dict["rover_drive_lb"].vel_cmd = msg.left[0]
        self.joint_dict["rover_drive_lf"].vel_cmd = msg.left[1]
        self.joint_dict["rover_drive_rb"].vel_cmd = msg.right[0]
        self.joint_dict["rover_drive_rf"].vel_cmd = msg.right[1]

    def run(self):
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

            self.joint_dict[key].attach_odrive()

        print("Connection step completed.")

        # Create a thread for calibration and entering closed loop control
        for joint_name, joint_obj in self.joint_dict.items():
            t = threading.Thread(
                target=self.joint_obj.calibrate(),
                # args=(key, self.joint_dict[key]),
            )
            calibrate_threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in calibrate_threads:
            t.join()

        self.is_calibrated = True
        print("Calibration step completed.")

        for joint_name, joint_obj in self.joint_dict.items():
            t = threading.Thread(
                target=self.joint_obj.enter_closed_loop_control(),
            )
            enter_closed_loop_threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in enter_closed_loop_threads:
            t.join()

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
                # Rescue disconnected joints
                if not joint_obj.odrv:
                    # Create a thread for reconnection
                    t = threading.Thread(
                        target=joint_obj.attach_odrive(),
                        # args=(self.joint_dict[joint_name]),
                    )
                    # threads.append(t)
                    t.start()
                    continue
                try:
                    # TODO feedback for disconnected joint
                    odrive_status_msg = ODriveStatus()
                    odrive_status_msg.id = joint_obj.serial_number
                    odrive_status_msg.state = joint_obj.odrv.axis0.current_state
                    odrive_status_msg.dump_errors = dump_errors(joint_obj.odrv)
                    odrive_status_msg.error = joint_obj.odrv.axis0.active_errors
                    odrive_status_msg.process_result = (
                        joint_obj.odrv.axis0.procedure_result
                    )
                    odrive_status_msg.input_pos = (
                        joint_obj.odrv.axis0.controller.input_pos
                    )
                    odrive_status_msg.pos_abs = (
                        joint_obj.odrv.axis0.pos_vel_mapper.pos_abs
                    )
                    odrive_status_msg.pos_rel = (
                        joint_obj.odrv.axis0.pos_vel_mapper.pos_rel
                    )
                    odrive_status_msg.input_vel = (
                        joint_obj.odrv.axis0.controller.input_vel
                    )
                    odrive_status_msg.vel_estimate = (
                        joint_obj.odrv.encoder_estimator0.vel_estimate
                    )
                    self.odrive_publisher.publish(odrive_status_msg)

                    # ERROR HANDLING
                    if joint_obj.odrv.axis0.active_errors != 0:
                        # Tell the rover to stop
                        for joint_name, joint_obj in self.joint_dict.items():
                            if not joint_obj.odrv:
                                continue
                            joint_obj.odrv.axis0.controller.input_vel = 0

                        # Wait until it actually stops
                        motor_stopped = False
                        while not motor_stopped:
                            motor_stopped = True
                            for joint_name, joint_obj in self.joint_dict.items():
                                if not joint_obj.odrv:
                                    continue
                                if (
                                    abs(joint_obj.odrv.encoder_estimator0.vel_estimate)
                                    >= 0.01
                                ):
                                    motor_stopped = False
                            if rospy.is_shutdown():
                                print(
                                    "Shutdown prompt received. Setting all motors to idle state."
                                )
                                for (
                                    joint_name,
                                    joint_obj,
                                ) in self.joint_dict.items():
                                    if not joint_obj.odrv:
                                        continue
                                    joint_obj.odrv.axis0.requested_state = (
                                        AxisState.IDLE
                                    )
                                break

                        # Wait for two seconds while all the transient currents and voltages calm down
                        rospy.sleep(5)

                        # Now try to recover from the error. This will always succeed the first time, but if
                        # the error persists, the ODrive will not allow the transition to closed loop control, and
                        # re-throw the error.
                        joint_obj.odrv.clear_errors()
                        rospy.sleep(0.5)
                        joint_obj.odrv.axis0.requested_state = (
                            AxisState.CLOSED_LOOP_CONTROL
                        )
                        rospy.sleep(0.5)

                        # If the motor cannot recover successfully publish a message about the error, then print to console
                        if joint_obj.odrv.axis0.active_errors != 0:
                            error_fb = MotorError()
                            error_fb.id = joint_obj.serial_number
                            error_fb.error = ODriveError(
                                joint_obj.odrv.axis0.active_errors
                            ).name
                            self.error_publisher.publish(error_fb)
                            print(
                                f"""\nError(s) occurred. Motor ID: {
                                    error_fb.id}, Error(s): {error_fb.error}"""
                            )

                            # Finally, hang the node and keep trying to recover until the error is gone or the shutdown signal is received
                            print(
                                f"""\nMotor {error_fb.id} Cannot recover from error(s) {
                                    error_fb.error}. R to retry, keyboard interrupt to shut down node."""
                            )
                            while not rospy.is_shutdown():
                                prompt = input(">").upper()
                                if prompt == "R":
                                    joint_obj.odrv.clear_errors()
                                    rospy.sleep(0.5)
                                    joint_obj.odrv.axis0.requested_state = (
                                        AxisState.CLOSED_LOOP_CONTROL
                                    )
                                    rospy.sleep(0.5)
                                    if joint_obj.odrv.axis0.active_errors == 0:
                                        break
                                    else:
                                        print("Recovery failed. Try again?")
                except AttributeError:
                    print(f"""{joint_name} is not connected""")
            print()
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
