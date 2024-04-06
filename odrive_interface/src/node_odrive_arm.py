# Author: mn297
import threading
from threading import Lock
from odrive_interface.msg import MotorState, MotorError
from ODriveJoint import *
from std_msgs.msg import Float32MultiArray
from odrive.utils import dump_errors
from odrive.enums import AxisState, ODriveError, ProcedureResult
from enum import Enum
import rospy
import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)


# unused at the moment
class FeedbackMode(Enum):
    FROM_ODRIVE = "fb_from_odrive"
    FROM_OUTSHAFT = "fb_from_outshaft"


# Unused at the moment
arm_zero_offsets = {
    "rover_arm_elbow": 0,
    "rover_arm_shoulder": 45.0,
    "rover_arm_waist": 0,
}

current_mode = FeedbackMode.FROM_OUTSHAFT

# VARIABLES -------------------------------------------------------------------
# Dictionary of ODriveJoint objects, key is the joint name in string format, value is the ODriveJoint object


# Predefine the order of joints for publishing feedback
joint_order = ["rover_arm_elbow", "rover_arm_shoulder", "rover_arm_waist"]


class NodeODriveInterfaceArm:
    def __init__(self):
        self.is_homed = False
        self.is_calibrated = False

        # CONFIGURATION ---------------------------------------------------------------
        # Serial number of the ODrive controlling the joint
        self.joint_serial_numbers = {
            # 0x383834583539 = 61814047520057 in decimal
            "rover_arm_elbow": "383834583539",
            # 0x386434413539 = 62003024573753 in decimal
            "rover_arm_shoulder": "386434413539",
            "rover_arm_waist": "0",
        }
        self.joint_dict = {
            "rover_arm_elbow": None,
            "rover_arm_shoulder": None,
            "rover_arm_waist": None,
        }

        # FEEDBACK VARIABLES
        self.joint_pos_outshaft_dict = {
            "rover_arm_elbow": 0,
            "rover_arm_shoulder": 0,
            "rover_arm_waist": 0,
        }

        self.locks = {joint_name: Lock()
                      for joint_name in self.joint_dict.keys()}

        # Subscriptions
        rospy.init_node("odrive_interface_arm")
        self.outshaft_pos_fb_subscriber = rospy.Subscriber(
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
            "/odrive_status", MotorState, queue_size=1
        )
        self.odrive_pos_fb_publisher = rospy.Publisher(
            "/odrive_armBrushlessFb", Float32MultiArray, queue_size=1
        )

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    # Update encoder angle from external encoders
    def handle_outshaft_fb(self, msg):
        if not self.is_calibrated:
            return
        if not self.is_homed:
            self.joint_pos_outshaft_dict["rover_arm_elbow"] = msg.data[0]
            self.joint_pos_outshaft_dict["rover_arm_shoulder"] = msg.data[1]
            self.joint_pos_outshaft_dict["rover_arm_waist"] = msg.data[2]

            # Home the joints to 0 position
            for joint_name, joint_obj in self.joint_dict.items():
                try:
                    if not joint_obj.odrv:
                        continue
                    temp = (
                        self.joint_pos_outshaft_dict[joint_name]
                        * joint_obj.gear_ratio
                        / 360
                    )
                    joint_obj.odrv.axis0.set_abs_pos(temp)
                except:
                    print(
                        f"""Cannot home joint: {
                            joint_name} to position: {
                            self.joint_pos_outshaft_dict[joint_name]}"""
                    )
            self.is_homed = True

    # Receive setpoint from external control node
    def handle_arm_cmd(self, msg):
        if not self.is_calibrated:
            print("ODrive not calibrated. Ignoring command.")
            return
        self.joint_dict["rover_arm_elbow"].pos_cmd = msg.data[0]
        self.joint_dict["rover_arm_shoulder"].pos_cmd = msg.data[1]
        self.joint_dict["rover_arm_waist"].pos_cmd = msg.data[2]

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
                target=joint_obj.calibrate,
                # args=(joint_obj,),
            )
            calibrate_threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in calibrate_threads:
            t.join()

        print("Calibration step completed.")

        for joint_name, joint_obj in self.joint_dict.items():
            if joint_obj.odrv is None:
                continue
            print(f"Creating thread for joint {joint_obj.name}")
            t = threading.Thread(
                target=joint_obj.enter_closed_loop_control,
                # args=(joint_obj,),
            )
            enter_closed_loop_threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in enter_closed_loop_threads:
            t.join()

        self.is_calibrated = True

        # Set the direction of the motors
        self.joint_dict["rover_arm_elbow"].gear_ratio = 100
        self.joint_dict["rover_arm_shoulder"].gear_ratio = 100
        self.joint_dict["rover_arm_waist"].gear_ratio = 1

        print("Entering closed loop control step completed.")

        # MAIN LOOP
        while not rospy.is_shutdown():
            # PRINT TIMESTAMP
            print(f"""Time: {rospy.get_time()}""")

            # ODRIVE POSITION FEEDBACK, different from the outshaft feedback
            feedback = Float32MultiArray()
            for joint_name, joint_obj in self.joint_dict.items():
                if not joint_obj.odrv:
                    continue
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
                except fibre.libfibre.ObjectLostError:
                    joint_obj.odrv = None
                    feedback.data.append(0.0)
                except:
                    print(f"""Cannot get feedback from joint: {joint_name}""")
                    feedback.data.append(0.0)

            # Publish
            self.odrive_pos_fb_publisher.publish(feedback)

            # APPLY Position Cmd
            for joint_name, joint_obj in self.joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    # setpoint in radians
                    setpoint = joint_obj.pos_cmd * joint_obj.gear_ratio / 360
                    joint_obj.odrv.axis0.controller.input_pos = setpoint
                except fibre.libfibre.ObjectLostError:
                    joint_obj.odrv = None
                except:
                    print(
                        f"""Cannot apply setpoint {
                            setpoint} to joint: {joint_name}"""
                    )

            # PRINT POSITIONS TO CONSOLE
            print_joint_state_from_dict(self.joint_dict)

            # SEND ODRIVE INFO AND HANDLE ERRORS
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

                # try:
                #     # TODO ODriveStatus.msg

                #     # ERROR HANDLING
                #     if joint_obj.odrv.axis0.active_errors != 0:
                #         # Tell the rover to stop
                #         for joint_name, joint_obj in self.joint_dict.items():
                #             if not joint_obj.odrv:
                #                 continue
                #             joint_obj.odrv.axis0.controller.input_vel = 0

                #         # Wait until it actually stops
                #         motor_stopped = False
                #         while not motor_stopped:
                #             motor_stopped = True
                #             for joint_name, joint_obj in self.joint_dict.items():
                #                 if not joint_obj.odrv:
                #                     continue
                #                 if (
                #                     abs(joint_obj.odrv.encoder_estimator0.vel_estimate)
                #                     >= 0.01
                #                 ):
                #                     motor_stopped = False
                #             if rospy.is_shutdown():
                #                 print(
                #                     "Shutdown prompt received. Setting all motors to idle state."
                #                 )
                #                 for joint_name, joint_obj in self.joint_dict.items():
                #                     if not joint_obj.odrv:
                #                         continue
                #                     joint_obj.odrv.axis0.requested_state = (
                #                         AxisState.IDLE
                #                     )
                #                 break

                #         # Wait for two seconds while all the transient currents and voltages calm down
                #         rospy.sleep(5)

                #         # Now try to recover from the error. This will always succeed the first time, but if
                #         # the error persists, the ODrive will not allow the transition to closed loop control, and
                #         # re-throw the error.
                #         joint_obj.odrv.clear_errors()
                #         rospy.sleep(0.5)
                #         joint_obj.odrv.axis0.requested_state = (
                #             AxisState.CLOSED_LOOP_CONTROL
                #         )
                #         rospy.sleep(0.5)

                #         # If the motor cannot recover successfully publish a message about the error, then print to console
                #         if joint_obj.odrv.axis0.active_errors != 0:
                #             error_fb = MotorError()
                #             error_fb.id = joint_obj.serial_number
                #             error_fb.error = ODriveError(
                #                 joint_obj.odrv.axis0.active_errors
                #             ).name
                #             self.error_publisher.publish(error_fb)
                #             print(
                #                 f"""\nError(s) occurred. Motor ID: {
                #                     error_fb.id}, Error(s): {error_fb.error}"""
                #             )

                #             # Finally, hang the node and keep trying to recover until the error is gone or the shutdown signal is received
                #             print(
                #                 f"""\nMotor {error_fb.id} Cannot recover from error(s) {
                #                     error_fb.error}. R to retry, keyboard interrupt to shut down node."""
                #             )
                #             while not rospy.is_shutdown():
                #                 prompt = input(">").upper()
                #                 if prompt == "R":
                #                     joint_obj.odrv.clear_errors()
                #                     rospy.sleep(0.5)
                #                     joint_obj.odrv.axis0.requested_state = (
                #                         AxisState.CLOSED_LOOP_CONTROL
                #                     )
                #                     rospy.sleep(0.5)
                #                     if joint_obj.odrv.axis0.active_errors == 0:
                #                         break
                #                     else:
                #                         print("Recovery failed. Try again?")
                # except AttributeError:
                #     print(f"""{joint_name} is not connected""")
            print()
            self.rate.sleep()

        # On shutdown, bring motors to idle state
        print("Shutdown prompt received. Setting all motors to idle state.")
        for joint_name, joint_obj in self.joint_dict.items():
            if not joint_obj.odrv:
                continue
            joint_obj.odrv.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    driver = NodeODriveInterfaceArm()
    rospy.spin()
