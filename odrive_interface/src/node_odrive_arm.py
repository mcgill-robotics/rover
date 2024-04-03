import threading
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


arm_gear_ratios = {
    "rover_arm_elbow": 100,
    "rover_arm_shoulder": 100,
    "rover_arm_waist": 1,
}

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
        self.arm_serial_numbers = {
            "rover_arm_elbow": "383834583539",  # 0x383834583539 = 61814047520057 in decimal
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

        # SETPOINT VARIABLES
        # self.joint_setpoint_dict = {
        #     "rover_arm_elbow": 0,
        #     "rover_arm_shoulder": 0,
        #     "rover_arm_waist": 0,
        # }

        # Subscriptions
        rospy.init_node("odrive_interface_arm")
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
            "/odrive_status", MotorState, queue_size=1
        )
        self.odrive_fb_publisher = rospy.Publisher(
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
        # self.joint_setpoint_dict["rover_arm_elbow"] = msg.data[0]
        # self.joint_setpoint_dict["rover_arm_shoulder"] = msg.data[1]
        # self.joint_setpoint_dict["rover_arm_waist"] = msg.data[2]

        # for joint_name, joint_obj in self.joint_dict.items():
        #     if not joint_obj.odrv:
        #         return
        #     joint_obj.pos_cmd = self.joint_setpoint_dict[joint_name]

        self.joint_dict["rover_arm_elbow"].pos_cmd = msg.data[0]
        self.joint_dict["rover_arm_shoulder"].pos_cmd = msg.data[1]
        self.joint_dict["rover_arm_waist"].pos_cmd = msg.data[2]

    def odrive_reconnect_watchdog(self):
        for key, value in self.arm_serial_numbers.items():
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

    def run(self):
        for key, value in self.arm_serial_numbers.items():
            if value == 0:
                print(
                    f"""Skipping connection for joint: {
                        key} due to serial_number being 0"""
                )
                # Instantiate class with odrv as None because we're skipping connection
                self.joint_dict[key] = ODriveJoint(
                    gear_ratio=arm_gear_ratios[key],
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
                gear_ratio=arm_gear_ratios[key],
                odrv=odrv,
            )

            if odrv is not None:
                # CALIBRATE -----------------------------------------------------
                print("CALIBRATING...")
                self.joint_dict[key].calibrate()

                # ENTER CLOSED LOOP CONTROL ------------------------------------
                print("ENTERING CLOSED LOOP CONTROL...")
                self.joint_dict[key].enter_closed_loop_control()

                self.is_calibrated = True

        # self.watchdog_stop_event = threading.Event()
        # self.watchdog_thread = threading.Thread(
        #     target=self.odrive_reconnect_watchdog)
        # self.watchdog_thread.start()

        # START WATCHDOG THREAD FOR DEBUG INFO ---------------------------------------------------------
        # watchdog_stop_event = threading.Event()
        # watchdog_thread = threading.Thread(
        #     target=print_joint_state_from_dict,
        #     args=(arm_self.joint_dict, watchdog_stop_event),
        # )
        # watchdog_thread.start()

        # MAIN LOOP
        while not rospy.is_shutdown():
            # PRINT TIMESTAMP
            print(f"""Time: {rospy.get_time()}""")

            # ODRIVE POSITION FEEDBACK, different from the outshaft feedback
            feedback = Float32MultiArray()
            for joint_name, joint_obj in self.joint_dict.items():
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
            for joint_name, joint_obj in self.joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    # setpoint in radians
                    setpoint = joint_obj.pos_cmd * joint_obj.gear_ratio / 360
                    joint_obj.odrv.axis0.controller.input_pos = setpoint
                except:
                    print(
                        f"""Cannot apply setpoint {
                            setpoint} to joint: {joint_name}"""
                    )

            # PRINT POSITIONS TO CONSOLE
            print_joint_state_from_dict(self.joint_dict)

            # SEND ODRIVE INFO AND HANDLE ERRORS
            # TODO trim error handling down
            for joint_name, joint_obj in self.joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    # TODO ODriveStatus.msg

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
                                for joint_name, joint_obj in self.joint_dict.items():
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
    driver = NodeODriveInterfaceArm()
    rospy.spin()
