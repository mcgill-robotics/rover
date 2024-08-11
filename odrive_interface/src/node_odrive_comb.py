# Author: mn297
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
from threading import Lock
from queue import Queue

import rospy
import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)


class NodeODriveInterfaceComb:
    def __init__(self):
        self.is_homed = False
        self.is_calibrated = False
        self.threads = []
        self.shutdown_flag = False

        # CONFIGURATION ---------------------------------------------------------------
        # Serial number of the ODrive controlling the joint
        self.joint_serial_numbers = {
            # Drive
            # "rover_drive_rf": "384F34683539",
            "rover_drive_rf": "385C347A3539",
            # "rover_drive_lf": "386134503539",
            "rover_drive_lf": "387134683539",
            # "rover_drive_rb": "387134683539",
            "rover_drive_rb": "386134503539",
            # "rover_drive_lb": "385C347A3539",
            "rover_drive_lb": "384F34683539",
            # Arm
            # 0x383834583539 = 61814047520057 in decimal
            "rover_arm_elbow": "383834583539",
            # 0x386434413539 = 62003024573753 in decimal
            "rover_arm_shoulder": "386434413539",
            # Not installed yet
            # "rover_arm_waist": "395935333231",
            "rover_arm_waist": "0",
        }

        # VARIABLES -------------------------------------------------------------------
        # Dictionary of ODriveJoint objects, key is the joint name in string format, value is the ODriveJoint object
        self.joint_dict = {
            # Drive
            "rover_drive_rf": None,
            "rover_drive_lf": None,
            "rover_drive_rb": None,
            "rover_drive_lb": None,
            # Arm
            "rover_arm_elbow": None,
            "rover_arm_shoulder": None,
            "rover_arm_waist": None,
        }

        self.joint_pos_outshaft_dict = {
            "rover_arm_elbow": 0,
            "rover_arm_shoulder": 0,
            "rover_arm_waist": 0,
        }

        # Joint limits in degrees
        self.joint_pos_lim_dict = {
            "rover_arm_elbow": [-30, 30],
            "rover_arm_shoulder": [-30, 30],
            "rover_arm_waist": [-30, 30],
        }

        self.locks = {joint_name: Lock() for joint_name in self.joint_dict.keys()}

        rospy.init_node("odrive_interface_comb")

        # Subscriptions
        # Cmd comes from the external control node, we convert it to setpoint and apply it to the ODrive
        # Drive
        self.drive_cmd_subscriber = rospy.Subscriber(
            "/wheel_velocity_cmd", WheelSpeed, self.handle_drive_cmd
        )

        # Arm
        self.outshaft_pos_fb_subscriber = rospy.Subscriber(
            "/armBrushlessFb",
            Float32MultiArray,
            self.handle_outshaft_fb,
        )

        self.arm_joint_cmd_subscriber = rospy.Subscriber(
            "/armBrushlessCmd", Float32MultiArray, self.handle_arm_cmd
        )

        # Publishers
        # drive
        self.drive_fb_publisher = rospy.Publisher(
            "/wheel_velocity_feedback", WheelSpeed, queue_size=1
        )

        self.odrive_publisher = rospy.Publisher(
            "/odrive_state", ODriveStatus, queue_size=1
        )

        # Arm
        self.odrive_state_publisher = rospy.Publisher(
            "/odrive_status", MotorState, queue_size=1
        )
        self.odrive_pos_fb_publisher = rospy.Publisher(
            "/odrive_armBrushlessFb", Float32MultiArray, queue_size=1
        )

        rospy.on_shutdown(self.shutdown_hook)

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)

        self.run()

    # Receive setpoint from external control node
    def handle_drive_cmd(self, msg):
        if not self.is_calibrated:
            print("ODrive not calibrated. Ignoring command.")
            return
        try:
            self.joint_dict["rover_drive_lb"].vel_cmd = msg.left[0]
        except KeyError:
            # print("KeyError: 'rover_drive_lb' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_lf"].vel_cmd = msg.left[1]
        except KeyError:
            # print("KeyError: 'rover_drive_lf' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_rb"].vel_cmd = msg.right[0]
        except KeyError:
            # print("KeyError: 'rover_drive_rb' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_rf"].vel_cmd = msg.right[1]
        except KeyError:
            # print("KeyError: 'rover_drive_rf' not found in joint_dict")
            pass

        # APPLY Velocity CMD
        for joint_name, joint_obj in self.joint_dict.items():
            if "drive" in joint_name:
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

    def reconnect_joint(self, joint_name, joint_obj):
        # Attempt to reconnect...
        # Update joint_obj.odrv as necessary...
        joint_obj.reconnect()

        # Once done, reset the flag
        with self.locks[joint_name]:
            joint_obj.is_reconnecting = False

    # Deprecated because enter_closed_loop_control() will call calibrate() if necessary
    def calibrate_and_enter_closed_loop_control(self, joint_obj):
        if joint_obj.odrv is not None:
            print(f"CALIBRATING joint {joint_obj.name}...")
            joint_obj.calibrate()

            print(f"ENTERING CLOSED LOOP CONTROL for joint {joint_obj.name}...")
            joint_obj.enter_closed_loop_control()

    def enter_closed_loop_control(self, joint_obj):
        if joint_obj.odrv is not None:
            print(f"ENTERING CLOSED LOOP CONTROL for joint {joint_obj.name}...")
            joint_obj.enter_closed_loop_control()

    def publish_joints_feedback_drive(self):
        # PUBLISH Odrive velocity FB
        feedback = WheelSpeed()
        for joint_name, joint_obj in self.joint_dict.items():
            if "drive" in joint_name:
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
            try:
                feedback.left[0] = self.joint_dict["rover_drive_lb"].vel_fb
            except KeyError:
                # print("KeyError: 'rover_drive_lb' not found in joint_dict")
                feedback.left[0] = 0.0  # Default value if key is missing
            try:
                feedback.left[1] = self.joint_dict["rover_drive_lf"].vel_fb
            except KeyError:
                # print("KeyError: 'rover_drive_lf' not found in joint_dict")
                feedback.left[1] = 0.0  # Default value if key is missing
            try:
                feedback.right[0] = self.joint_dict["rover_drive_rb"].vel_fb
            except KeyError:
                # print("KeyError: 'rover_drive_rb' not found in joint_dict")
                feedback.right[0] = 0.0  # Default value if key is missing
            try:
                feedback.right[1] = self.joint_dict["rover_drive_rf"].vel_fb
            except KeyError:
                # print("KeyError: 'rover_drive_rf' not found in joint_dict")
                feedback.right[1] = 0.0  # Default value if key is missing
            self.drive_fb_publisher.publish(feedback)

    # Send joints angle feedback to ROS
    def publish_joints_feedback_arm(self):
        feedback = Float32MultiArray()
        for joint_name, joint_obj in self.joint_dict.items():
            if "arm" in joint_name:
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
                except Exception as e:
                    print(f"Cannot get feedback from joint: {joint_name} - {str(e)}")
                    feedback.data.append(0.0)

        # Publish
        self.odrive_pos_fb_publisher.publish(feedback)

    def setup_odrive(self):
        # CONNECT -----------------------------------------------------
        # for key, value in self.joint_serial_numbers.items():
        for key, value in self.joint_dict.items():
            # Instantiate ODriveJoint class whether or not the connection attempt was made/successful
            # self.joint_dict[key] = ODriveJoint(name=key, serial_number=value)
            self.joint_dict[key] = ODriveJoint(
                name=key, serial_number=self.joint_serial_numbers[key]
            )

            # Set limits for arm
            if "arm" in key:
                self.joint_dict[key].pos_min_deg = self.joint_pos_lim_dict[key][0]
                self.joint_dict[key].pos_max_deg = self.joint_pos_lim_dict[key][1]

            if value == 0:
                print(
                    f"""Skipping connection for joint: {
                            key} due to serial_number being 0"""
                )
                continue

            self.joint_dict[key].reconnect()

        # # CONNECT -----------------------------------------------------
        # for joint_name, joint_obj in self.joint_dict.items():
        #     if joint_obj.odrv is None:
        #         continue
        #     print(f"Creating reconnect() thread for joint {joint_obj.name}")
        #     t = threading.Thread(
        #         target=self.reconnect_joint,
        #         args=(joint_name, joint_obj),
        #     )
        #     self.threads.append(t)
        #     t.start()

        # # Wait for all threads to complete
        # for t in self.threads:
        #     t.join()
        # self.threads = []

        # print("Connection step completed.")

        # CALIBRATE AND ENTER CLOSED LOOP CONTROL -----------------------------------------------------
        for joint_name, joint_obj in self.joint_dict.items():
            if joint_obj.odrv is None:
                continue
            print(f"Creating calibrate() thread for joint {joint_obj.name}")
            t = threading.Thread(
                target=joint_obj.calibrate,
            )
            self.threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in self.threads:
            t.join()
        self.threads = []

        print("Calibration step completed.")

        for joint_name, joint_obj in self.joint_dict.items():
            if joint_obj.odrv is None:
                continue
            print(
                f"Creating enter_closed_loop_control() thread for joint {joint_obj.name}"
            )
            t = threading.Thread(
                target=joint_obj.enter_closed_loop_control,
            )
            self.threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in self.threads:
            t.join()
        self.threads = []

        self.is_calibrated = True

        # Set the direction of the motors
        self.joint_dict["rover_arm_elbow"].gear_ratio = 100
        self.joint_dict["rover_arm_shoulder"].gear_ratio = 100
        self.joint_dict["rover_arm_waist"].gear_ratio = 1

        # Set the direction of the motors
        try:
            self.joint_dict["rover_drive_lb"].direction = -1
        except KeyError:
            # print("KeyError: 'rover_drive_lb' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_lf"].direction = -1
        except KeyError:
            # print("KeyError: 'rover_drive_lf' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_rb"].direction = 1
        except KeyError:
            # print("KeyError: 'rover_drive_rb' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_rf"].direction = 1
        except KeyError:
            # print("KeyError: 'rover_drive_rf' not found in joint_dict")
            pass
        print("Entering closed loop control step completed.")

    # SEND ODRIVE INFO AND HANDLE ERRORS
    def handle_joints_error(self):
        for joint_name, joint_obj in self.joint_dict.items():
            with self.locks[joint_name]:  # Use a lock specific to each joint
                if joint_obj.odrv is not None:
                    if (
                        joint_obj.odrv.axis0.current_state
                        != AxisState.CLOSED_LOOP_CONTROL
                    ):
                        if not joint_obj.is_reconnecting:
                            print(
                                f"{joint_name} is not in closed loop control, recalibrating..."
                            )
                            joint_obj.is_reconnecting = True
                            t = threading.Thread(
                                target=self.enter_closed_loop_control,
                                # target=self.calibrate_and_enter_closed_loop,
                                args=(joint_obj,),
                                # target=joint_obj.enter_closed_loop_control
                            )
                            self.threads.append(t)
                            t.start()
                    else:
                        # Handle case where 'axis0' is not available
                        # print(
                        #     f"Cannot check current state for {joint_name}, 'axis0' attribute missing."
                        # )
                        pass
                else:
                    # ODrive interface is None, indicating a disconnection or uninitialized state
                    if not joint_obj.is_reconnecting:
                        print(f"RECONNECTING {joint_name}...")
                        joint_obj.is_reconnecting = True
                        t = threading.Thread(
                            target=self.reconnect_joint,
                            args=(joint_name, joint_obj),
                        )
                        t.start()

    # Arm func
    def handle_outshaft_fb(self, msg):
        if not self.is_calibrated:
            return
        if not self.is_homed:
            self.joint_pos_outshaft_dict["rover_arm_elbow"] = msg.data[0]
            self.joint_pos_outshaft_dict["rover_arm_shoulder"] = msg.data[1]
            self.joint_pos_outshaft_dict["rover_arm_waist"] = msg.data[2]

            # Home the joints to 0 position
            for joint_name, joint_obj in self.joint_dict.items():
                if "arm" in joint_name:
                    try:
                        if not joint_obj.odrv:
                            continue
                        temp = (
                            self.joint_pos_outshaft_dict[joint_name]
                            * joint_obj.gear_ratio
                            / 360
                        )
                        joint_obj.odrv.axis0.set_abs_pos(temp)
                        print("Homing joint: ", joint_name)
                    except Exception as e:
                        print(
                            f"Cannot home joint: {joint_name} to position: {self.joint_pos_outshaft_dict[joint_name]} - {str(e)}"
                        )

            # Set all pos_cmd to 0
            for joint_name, joint_obj in self.joint_dict.items():
                if "arm" in joint_name:
                    joint_obj.pos_cmd = 0
                self.apply_arm_cmd()
                self.is_homed = True

    # Receive setpoint from external control node
    def handle_arm_cmd(self, msg):
        if not self.is_calibrated:
            print("ODrive not calibrated. Ignoring command.")
            return
        self.joint_dict["rover_arm_elbow"].pos_cmd = msg.data[0]
        self.joint_dict["rover_arm_shoulder"].pos_cmd = msg.data[1]
        self.joint_dict["rover_arm_waist"].pos_cmd = msg.data[2]
        self.apply_arm_cmd()

    def apply_arm_cmd(self):
        # APPLY Position Cmd
        for joint_name, joint_obj in self.joint_dict.items():
            if "arm" in joint_name:
                if not joint_obj.odrv:
                    continue
                try:
                    # setpoint in radians
                    setpoint = joint_obj.pos_cmd * joint_obj.gear_ratio / 360
                    joint_obj.odrv.axis0.controller.input_pos = setpoint
                    print(f"Setpoint applied to {joint_name}: {setpoint}")
                except fibre.libfibre.ObjectLostError:
                    joint_obj.odrv = None
                except Exception as e:
                    print(
                        f"Cannot apply setpoint {setpoint} to joint: {joint_name} - {str(e)}"
                    )

    def print_loop(self):
        while not self.shutdown_flag:
            print_joint_state_from_dict(self.joint_dict, sync_print=True)
            time.sleep(0.2)

    def loop_odrive(self):
        # MAIN LOOP -----------------------------------------------------
        while not rospy.is_shutdown() and not self.shutdown_flag:
            # PRINT TIMESTAMP
            # print(f"""Time: {rospy.get_time()}""")

            self.publish_joints_feedback_drive()

            self.publish_joints_feedback_arm()

            # HANDLE ERRORS
            self.handle_joints_error()

            self.rate.sleep()

    def run(self):
        self.setup_odrive()
        thread = threading.Thread(target=self.loop_odrive)
        thread.start()
        print_thread = threading.Thread(target=self.print_loop)
        print_thread.start()
        rospy.spin()
        self.shutdown_hook()

    def shutdown_hook(self):
        self.shutdown_flag = True
        print("Shutdown initiated. Setting all motors to idle state.")
        for joint_name, joint_obj in self.joint_dict.items():
            if not joint_obj.odrv:
                continue
            joint_obj.odrv.axis0.requested_state = AxisState.IDLE
        for t in self.threads:
            t.join()
        print("All threads joined. Shutdown complete.")


if __name__ == "__main__":
    driver = NodeODriveInterfaceComb()
    rospy.spin()
