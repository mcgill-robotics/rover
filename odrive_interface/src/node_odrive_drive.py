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
        self.drive_serial_numbers = {
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

    def odrive_reconnect_watchdog(self):
        for key, value in self.drive_serial_numbers.items():
            try:
                odrv = odrive.find_any(serial_number=value, timeout=5)
                self.joint_dict[key].attach_odrive(odrv)
                print(f"""Connected joint: {key}, serial_number: {value}""")
            except:
                odrv = None
                print(
                    f"""Cannot connect joint: {
                      key}, serial_number: {value}"""
                )

    def calibrate_and_enter_closed_loop(self, joint_name, joint_obj):
        if joint_obj.odrv is not None:
            print(f"CALIBRATING joint {joint_name}...")
            joint_obj.calibrate()

            print(f"ENTERING CLOSED LOOP CONTROL for joint {joint_name}...")
            joint_obj.enter_closed_loop_control()

            self.is_calibrated = True

    def run(self):
        threads = []

        # SETUP ODRIVE CONNECTIONS -----------------------------------------------------
        for key, value in self.drive_serial_numbers.items():
            if value == 0:
                print(
                    f"""Skipping connection for joint: {
                        key} due to serial_number being 0"""
                )
                # Instantiate class with odrv as None because we're skipping connection
                self.joint_dict[key] = ODriveJoint(
                    odrv=None,
                )
                continue

            try:
                # Attempt to CONNECT TO ODRIVE only if serial_number is not 0
                odrv = odrive.find_any(serial_number=value, timeout=5)
                print(f"""Connected joint: {key}, serial_number: {value}""")
            except Exception as e:
                odrv = None
                print(
                    f"""Cannot connect joint: {
                        key}, serial_number: {value}. Error: {e}"""
                )

            # Instantiate ODriveJoint class whether or not the connection attempt was made/successful
            self.joint_dict[key] = ODriveJoint(
                odrv=odrv,
            )

            # if odrv is not None:
            #     # CALIBRATE -----------------------------------------------------
            #     print("CALIBRATING...")
            #     self.joint_dict[key].calibrate()

            #     # ENTER CLOSED LOOP CONTROL ------------------------------------
            #     print("ENTERING CLOSED LOOP CONTROL...")
            #     self.joint_dict[key].enter_closed_loop_control()

            #     self.is_calibrated = True

            # Create a thread for calibration and entering closed loop control
            t = threading.Thread(
                target=self.calibrate_and_enter_closed_loop, args=(key, self.joint_dict[key]))
            threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in threads:
            t.join()

        # Set the direction of the motors
        self.joint_dict["rover_drive_lb"].direction = -1
        self.joint_dict["rover_drive_lf"].direction = -1
        self.joint_dict["rover_drive_rb"].direction = 1
        self.joint_dict["rover_drive_rf"].direction = 1

        print("All operations completed.")
        # print(self.joint_dict)
        # calibrate_non_blocking(self.joint_dict)
        # rospy.sleep(1)
        # enter_closed_loop_control_non_blocking(self.joint_dict)

        # MAIN LOOP -----------------------------------------------------
        while not rospy.is_shutdown():
            # PRINT TIMESTAMP
            print(f"""Time: {rospy.get_time()}""")

            # ODRIVE Velocity FB
            feedback = WheelSpeed()
            for joint_name, joint_obj in self.joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    self.joint_dict[joint_name].vel_fb = (
                        joint_obj.odrv.encoder_estimator0.vel_estimate
                    )
                except:
                    print(f"""Cannot get feedback from joint: {joint_name}""")

            # Publish
            feedback.left[0] = self.joint_dict["rover_drive_lb"].vel_fb * \
                self.joint_dict["rover_drive_lb"].direction
            feedback.left[1] = self.joint_dict["rover_drive_lf"].vel_fb * \
                self.joint_dict["rover_drive_lf"].direction
            feedback.right[0] = self.joint_dict["rover_drive_rb"].vel_fb * \
                self.joint_dict["rover_drive_rb"].direction
            feedback.right[1] = self.joint_dict["rover_drive_rf"].vel_fb * \
                self.joint_dict["rover_drive_rf"].direction
            self.drive_fb_publisher.publish(feedback)

            # APPLY Velocity CMD
            for joint_name, joint_obj in self.joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    # setpoint in rev/s
                    joint_obj.odrv.axis0.controller.input_vel = joint_obj.vel_cmd * joint_obj.direction
                except:
                    print(
                        f"""Cannot apply vel_cmd {
                            joint_obj.vel_cmd} to joint: {joint_name}"""
                    )

            # PRINT POSITIONS TO CONSOLE
            print_joint_state_from_lst(self.joint_dict)

            # SEND ODRIVE INFO AND HANDLE ERRORS
            # TODO trim error handling down
            for joint_name, joint_obj in self.joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    # TODO full feedback
                    odrive_state = ODriveStatus()
                    odrive_state.id = joint_obj.serial_number
                    odrive_state.state = joint_obj.odrv.axis0.current_state
                    odrive_state.error = joint_obj.odrv.axis0.active_errors
                    odrive_state.input_pos = joint_obj.odrv.axis0.controller.input_pos
                    odrive_state.pos_abs = joint_obj.odrv.axis0.pos_vel_mapper.pos_abs
                    odrive_state.pos_rel = joint_obj.odrv.axis0.pos_vel_mapper.pos_rel
                    odrive_state.input_vel = joint_obj.odrv.axis0.controller.input_vel
                    odrive_state.vel_estimate = (
                        joint_obj.odrv.encoder_estimator0.vel_estimate
                    )
                    self.odrive_publisher.publish(odrive_state)

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
