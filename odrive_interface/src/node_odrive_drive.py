import scipy.stats as st
from scipy.ndimage import gaussian_filter1d
from drive_control.msg import WheelSpeed
from odrive_interface.msg import MotorState, MotorError
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

# CONFIGURATION ---------------------------------------------------------------
# Serial number of the ODrive controlling the joint
drive_serial_numbers = {
    "rover_drive_rf": "384F34683539",
    "rover_drive_lf": "386134503539",
    "rover_drive_rb": "387134683539",
    "rover_drive_lb": "385C347A3539",
}

# VARIABLES -------------------------------------------------------------------
# Dictionary of ODriveJoint objects, key is the joint name in string format, value is the ODriveJoint object
drive_joint_dict = {
    "rover_drive_rf": None,
    "rover_drive_lf": None,
    "rover_drive_rb": None,
    "rover_drive_lb": None,
}


class Node_odrive_interface_arm:
    def __init__(self):
        self.is_homed = False
        self.is_calibrated = False

        # FEEDBACK VARIABLES

        # SETPOINT VARIABLES
        self.joint_setpoint_dict = {
            "rover_drive_rf": 0,
            "rover_drive_lf": 0,
            "rover_drive_rb": 0,
            "rover_drive_lb": 0,
        }

        # Subscriptions
        rospy.init_node("odrive_interface_drive")
        self.outshaft_fb_subscriber = rospy.Subscriber(
            "/armBrushlessFb",
            Float32MultiArray,
            self.handle_outshaft_fb,
        )
        # Cmd comes from the external control node, we convert it to setpoint and apply it to the ODrive
        self.arm_joint_cmd_subscriber = rospy.Subscriber(
            "/armBrushlessCmd", Float32MultiArray, self.handle_arm_cmd
        )

        # Publishers
        self.odrive_state_publisher = rospy.Publisher(
            "/odrive_arm_state", MotorState, queue_size=1
        )
        self.odrive_fb_publisher = rospy.Publisher(
            "/odrive_armBrushlessFb", Float32MultiArray, queue_size=1
        )

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    # Receive setpoint from external control node

    def handle_arm_cmd(self, msg):
        self.joint_setpoint_dict["rover_drive_lb"] = msg.data[0]
        self.joint_setpoint_dict["rover_drive_lf"] = msg.data[1]
        self.joint_setpoint_dict["rover_drive_rb"] = msg.data[2]
        self.joint_setpoint_dict["rover_drive_rf"] = msg.data[3]

    def odrive_reconnect_watchdog(self):
        for key, value in drive_serial_numbers.items():
            try:
                odrv = odrive.find_any(serial_number=value, timeout=5)
                drive_joint_dict[key].attach_odrive(odrv)
                print(f"""Connected joint: {key}, serial_number: {value}""")
            except:
                odrv = None
                print(
                    f"""Cannot connect joint: {
                      key}, serial_number: {value}"""
                )

    def run(self):
        for key, value in drive_serial_numbers.items():
            if value == 0:
                print(
                    f"""Skipping connection for joint: {
                        key} due to serial_number being 0"""
                )
                # Instantiate class with odrv as None because we're skipping connection
                drive_joint_dict[key] = ODriveJoint(
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
            drive_joint_dict[key] = ODriveJoint(
                odrv=odrv,
            )

            if odrv is not None:
                # CALIBRATE -----------------------------------------------------
                print("CALIBRATING...")
                drive_joint_dict[key].calibrate()

                # ENTER CLOSED LOOP CONTROL ------------------------------------
                print("ENTERING CLOSED LOOP CONTROL...")
                drive_joint_dict[key].enter_closed_loop_control()

                self.is_calibrated = True

        # self.watchdog_stop_event = threading.Event()
        # self.watchdog_thread = threading.Thread(
        #     target=self.odrive_reconnect_watchdog)
        # self.watchdog_thread.start()

        # START WATCHDOG THREAD FOR DEBUG INFO ---------------------------------------------------------
        # watchdog_stop_event = threading.Event()
        # watchdog_thread = threading.Thread(
        #     target=print_joint_state_from_lst,
        #     args=(drive_joint_dict, watchdog_stop_event),
        # )
        # watchdog_thread.start()

        # MAIN LOOP
        while not rospy.is_shutdown():
            # PRINT TIMESTAMP
            print(f"""Time: {rospy.get_time()}""")

            # ODRIVE POSITION FEEDBACK, different from the outshaft feedback
            feedback = Float32MultiArray()
            for joint_name, joint_obj in drive_joint_dict.items():
                try:
                    # Assuming .odrv.axis0.pos_vel_mapper.pos_abs and .gear_ratio are correct
                    feedback.data.append(
                        360
                        * (
                            joint_obj.odrv.axis0.pos_vel_mapper.pos_abs
                            / joint_obj.gear_ratio
                        )
                    )
                # Default value in case lose connection to ODrive
                except:
                    feedback.data.append(0.0)
            # Publish
            self.odrive_fb_publisher.publish(feedback)

            # APPLY SETPOINT
            for joint_name, joint_obj in drive_joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    # setpoint in radians
                    setpoint = joint_obj.setpoint_deg * joint_obj.gear_ratio / 360
                    joint_obj.odrv.axis0.controller.input_pos = setpoint
                except:
                    print(
                        f"""Cannot apply setpoint {
                            setpoint} to joint: {joint_name}"""
                    )

            # PRINT POSITIONS TO CONSOLE
            print_joint_state_from_lst(drive_joint_dict)

            # SEND ODRIVE INFO AND HANDLE ERRORS
            # TODO trim error handling down
            for joint_name, joint_obj in drive_joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    # TODO full feedback
                    # state_fb = MotorState()
                    # state_fb.id = joint_obj.serial_number
                    # state_fb.state = joint_obj.odrv.axis0.current_state
                    # state_fb.pos_rel = joint_obj.odrv.axis0.pos_vel_mapper.pos_rel
                    # state_fb.pos_abs = joint_obj.odrv.axis0.pos_vel_mapper.pos_abs
                    # state_fb.input_pos = joint_obj.odrv.axis0.controller.input_pos
                    # self.state_publisher.publish(state_fb)

                    # ERROR HANDLING
                    if joint_obj.odrv.axis0.active_errors != 0:
                        # Tell the rover to stop
                        for joint_name, joint_obj in drive_joint_dict.items():
                            if not joint_obj.odrv:
                                continue
                            joint_obj.odrv.axis0.controller.input_vel = 0

                        # Wait until it actually stops
                        motor_stopped = False
                        while not motor_stopped:
                            motor_stopped = True
                            for joint_name, joint_obj in drive_joint_dict.items():
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
                                for joint_name, joint_obj in drive_joint_dict.items():
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
        for joint_name, joint_obj in drive_joint_dict.items():
            if not joint_obj.odrv:
                continue
            joint_obj.odrv.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    driver = Node_odrive_interface_arm()
    rospy.spin()
