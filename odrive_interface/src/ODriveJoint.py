# Author: mn297
from __future__ import print_function

import odrive
from odrive.config import Axis
import odrive.enums
import odrive.utils
import odrive.config

from odrive.enums import *
from odrive.utils import dump_errors
from queue import Queue

import time
import threading
import fibre
import re
import math
import sys

try:
    import rospy

    ROS_ENABLED = True
except ImportError:
    ROS_ENABLED = False


def custom_sleep(duration):
    if ROS_ENABLED:
        rospy.sleep(duration)
    else:
        time.sleep(duration)


def watchdog(joint_dict, watchdog_stop_event):
    while not watchdog_stop_event.is_set():
        try:
            for joint_name, joint_obj in joint_dict.items():
                print(
                    "current_state="
                    + str(AxisState(joint_obj.odrv.axis0.current_state).name)
                    + ", "
                    + "Raw angle="
                    + str(joint_obj.odrv.rs485_encoder_group0.raw)
                    + ", "
                    + "pos_rel="
                    + str(joint_obj.odrv.axis0.pos_vel_mapper.pos_rel)
                    + ", "
                    + "pos_abs="
                    + str(joint_obj.odrv.axis0.pos_vel_mapper.pos_abs)
                    + ", "
                    + "input_pos="
                    + str(joint_obj.odrv.axis0.controller.input_pos)
                    + ", "
                    + "vel_estimate="
                    + str(joint_obj.odrv.encoder_estimator0.vel_estimate)
                    + ", "
                    + "lower_limit_switch_pin_state="
                    + f"""{bin(joint_obj.odrv.get_gpio_states())[2:]:0>12}"""[0]
                    + ", "
                    + "upper_limit_switch_pin_state="
                    + f"""{bin(joint_obj.odrv.get_gpio_states())[2:]:0>12}"""[2]
                )
                dump_errors(joint_obj.odrv)

            custom_sleep(1)
        except NameError:
            pass
        except fibre.libfibre.ObjectLostError:
            print(" lost connection in watchdog() ...")
            for joint_name, joint_obj in joint_dict.items():
                joint_obj.odrv = odrive.find_any(
                    serial_number=joint_obj.serial_number,
                    timeout=joint_obj.timeout,
                )
                if joint_obj.odrv:
                    print("  re-connected in watchdog()")
            pass


def print_joint_state_from_dict(joint_dict, sync_print=False):
    status_lines = []
    for joint_name, joint_obj in joint_dict.items():
        # Check if the odrive is connected
        status = "connected" if joint_obj.odrv else "disconnected, odrv=None"
        line = f"""{joint_name} {joint_obj.serial_number} ({status}) is_reconnecting={joint_obj.is_reconnecting}"""
        status_lines.append(line)
        if joint_obj.odrv:
            status = "connected"
            # Assuming dump_errors prints errors to stdout or logs them
            # dump_errors(joint_obj.odrv)
            current_state = f"""-current_state={AxisState(joint_obj.odrv.axis0.current_state).name}"""
            pos_rel = f"""-pos_rel={joint_obj.odrv.axis0.pos_vel_mapper.pos_rel}"""
            pos_abs = f"""-pos_abs={joint_obj.odrv.axis0.pos_vel_mapper.pos_abs}"""
            input_pos = f"""-input_pos={joint_obj.odrv.axis0.controller.input_pos}"""
            vel_estimate = (
                f"""-vel_estimate={joint_obj.odrv.encoder_estimator0.vel_estimate}"""
            )
        else:
            current_state = "-current_state=None"
            pos_rel = "-pos_rel=None"
            pos_abs = "-pos_abs=None"
            input_pos = "-input_pos=None"
            vel_estimate = "-vel_estimate=None"

        status_lines.append(current_state)
        status_lines.append(pos_rel)
        status_lines.append(pos_abs)
        status_lines.append(input_pos)
        status_lines.append(vel_estimate)
        pos_cmd = f"""-pos_cmd={joint_obj.pos_cmd}"""
        vel_cmd = f"""-vel_cmd={joint_obj.vel_cmd}"""
        status_lines.append(pos_cmd)
        status_lines.append(vel_cmd)
        status_lines.append("\n")  # Empty line for separation

    status = "\n".join(status_lines)

    if sync_print:
        sys.stdout.write("\r" + status)
        sys.stdout.flush()
    else:
        print(status)


class ODriveJoint:
    def __init__(
        self, name=None, odrv=None, gear_ratio=1, zero_offset_deg=0, serial_number=None
    ):
        self.name = name
        self.odrv = odrv
        # odrv.serial_number is int, serial_number should be hex version in string
        if self.odrv:
            self.serial_number = str(hex(self.odrv.serial_number)[2:])
        else:
            self.serial_number = serial_number
        self.timeout = 5
        self.is_reconnecting = False

        # Position control
        # gear_ratio is input revolutions / output revolutions
        self.gear_ratio = gear_ratio
        self.zero_offset_deg = zero_offset_deg
        self.pos_cmd = 0
        self.pos_outshaft_deg = 0

        # Velocity control
        self.vel_cmd = 0
        self.vel_fb = 0
        self.direction = 1

        # Configs
        self.pos_max_deg = 360
        self.pos_min_deg = -360

    def config_lower_limit_switch(self):
        print("starting lower limit switch config...")
        self.odrv.config.gpio12_mode = (
            GpioMode.DIGITAL_PULL_UP
        )  # the G12 pin is a configurable GPIO (see Odrive documentation)
        self.odrv.axis0.min_endstop.config.gpio_num = 12
        self.odrv.axis0.min_endstop.config.is_active_high = False
        self.odrv.axis0.min_endstop.config.enabled = True
        # TODO untested, min angle is -3, only for testing
        self.odrv.axis0.min_endstop.config.offset = -3
        # while (not test_odrv_joint.odrv.axis0.min_endstop.endstop_state):
        #     custom_sleep(0.1)

        print("lower limit pin is enabled")

        # verify it is on mode high
        print(
            "initial state of limit switch pin 12 is : ",
            f"""{bin(self.odrv.get_gpio_states())[2:]:0>12}"""[0],
        )

    def config_upper_limit_switch(self):
        print("starting upper limit switch config...")
        self.odrv.config.gpio10_mode = (
            GpioMode.DIGITAL
        )  # the G10 pin is a configurable GPIO (see Odrive documentation)
        self.odrv.axis0.max_endstop.config.gpio_num = 10
        # TODO untested, does max_endstop have offset?
        # self.odrv.axis0.max_endstop.config.offset = 3
        self.odrv.axis0.max_endstop.config.enabled = True
        self.odrv.axis0.max_endstop.config.is_active_high = False
        # while (not test_odrv_joint.odrv.axis0.min_endstop.endstop_state):
        #     custom_sleep(0.1)

        print("upper limit pin is enabled")

        # verify it is on mode high
        print(
            "initial state of limit switch pin 10 is : ",
            f"""{bin(self.odrv.get_gpio_states())[2:]:0>12}"""[2],
        )

    # TODO untested, call after entered closed loop control
    def enter_homing(self):
        self.odrv.axis0.requested_state = AxisState.HOMING
        # TODO check if necessary
        # <odrv>.<axis>.controller.config.vel_ramp_rate = ?
        # <odrv>.<axis>.trap_traj.config.vel_limit = ?
        # <odrv>.<axis>.trap_traj.config.accel_limit = ?
        # <odrv>.<axis>.trap_traj.config.decel_limit = ?

        while (
            self.odrv.axis0.current_state != AxisState.HOMING
            or self.odrv.axis0.current_state == AxisState.IDLE
        ):
            custom_sleep(0.5)
            print(
                "Motor {} {} is still entering homing. Current state: {}".format(
                    self.odrv.name,
                    self.odrv.serial_number,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
            dump_errors(self.odrv)
        print(
            "SUCCESS: Motor {} {} is in state: {}.".format(
                self.odrv.name,
                self.odrv.serial_number,
                AxisState(self.odrv.axis0.current_state).name,
            )
        )
        # block until homing is done
        while self.odrv.axis0.current_state == AxisState.HOMING:
            custom_sleep(1)
            print(
                "Motor {} {} is still homing. Current state: {}".format(
                    self.odrv.name,
                    self.odrv.serial_number,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
            dump_errors(self.odrv)

    def save_config(self):
        try:
            self.odrv.save_configuration()
            while self.odrv.axis0.procedure_result != ProcedureResult.SUCCESS:
                custom_sleep(0.5)
        # Saving configuration makes the ODrive reboot
        except fibre.libfibre.ObjectLostError:
            print(" lost connection in save_config() ...")
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
            if self.odrv:
                print("  re-connected in save_config()")
            pass

    def erase_config(self):
        try:
            self.odrv.erase_configuration()
        # Erasing configuration makes the ODrive reboot
        except fibre.libfibre.ObjectLostError:
            print(" lost connection in erase_config() ...")
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
            if self.odrv:
                print("  re-connected in erase_config()")
            pass

    def reconnect(self):
        self.is_reconnecting
        try:
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
            print("Successfully reconnected to motor {}".format(self.name))
            self.is_reconnecting = False  # Set to False when reconnection is successful
        except (TimeoutError, fibre.libfibre.ObjectLostError) as e:
            print(f"Failed to reconnect to motor {self.name}. Error: {e}")
            self.odrv = None
        finally:
            if (
                self.is_reconnecting
            ):  # If still reconnecting, it means reconnection failed
                self.is_reconnecting = False
                # Handle failed reconnection if needed

    def calibrate(self):
        self.odrv.clear_errors()
        if self.odrv.axis0.current_state != AxisState.FULL_CALIBRATION_SEQUENCE:
            self.odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
        custom_sleep(0.2)

        # Wait for calibration to end
        while (
            # self.odrv.axis0.current_state == AxisState.MOTOR_CALIBRATION
            # or self.odrv.axis0.current_state == AxisState.FULL_CALIBRATION_SEQUENCE
            not self.odrv.axis0.current_state
            == AxisState.IDLE
        ):
            custom_sleep(1)
            print(
                "Motor {} is still calibrating. Current state: {}".format(
                    self.name,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
        results_available = False

        # Wait for calibration results
        while not results_available:
            results_available = True
            if self.odrv.axis0.procedure_result == ProcedureResult.BUSY:
                results_available = False
            custom_sleep(0.1)

        # ERROR CHECKING
        calibration_failed = False
        result = self.odrv.axis0.procedure_result
        errors = self.odrv.axis0.active_errors
        if result != ProcedureResult.SUCCESS:
            calibration_failed = True
            if errors != 0:
                print(
                    "Motor error(s) in motor {}: {}".format(
                        self.odrv.serial_number,
                        AxisError(self.odrv.axis0.disarm_reason).name,
                    )
                )
            print(
                "Calibration procedure failed in motor {}. Reason: {}".format(
                    self.name, ProcedureResult(result).name
                )
            )
            if result == ProcedureResult.DISARMED:
                print(
                    "Motor {} disarmed. Reason: {}".format(
                        self.odrv.serial_number,
                        AxisError(self.odrv.axis0.disarm_reason).name,
                    )
                )
            self.odrv.axis0.requested_state = AxisState.IDLE
        else:
            print("Calibration successful!")

    def enter_closed_loop_control(self):
        max_retries = 5  # Maximum number of retries
        retries = 0

        self.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

        while (
            self.odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL
            or self.odrv.axis0.current_state == AxisState.IDLE
        ) and retries < max_retries:
            custom_sleep(0.5)
            print(
                "Motor {} is still entering closed loop control. Current state: {}".format(
                    self.name,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
            self.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
            dump_errors(self.odrv)
            retries += 1

        if retries == max_retries:
            print(
                "ERROR: Motor {} failed to enter closed loop control after {} retries.".format(
                    self.name, max_retries
                )
            )
            self.calibrate()
            retries = 0
        if self.odrv.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
            print(
                "SUCCESS: Motor {} is in state: {}.".format(
                    self.odrv.serial_number,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
            self.is_reconnecting = False

    def calibrate_and_enter_closed_loop(self):
        if self.odrv is not None:
            print(f"CALIBRATING joint {self.name}...")
            self.calibrate()

            print(f"ENTERING CLOSED LOOP CONTROL for joint {self.name}...")
            self.enter_closed_loop_control()

            self.is_calibrated = True

    # Print the voltage on the GPIO pins
    def print_gpio_voltages(self):
        for i in [1, 2, 3, 4]:
            print(
                "voltage on GPIO{} is {} Volt".format(i, self.odrv.get_adc_voltage(i))
            )


# Maybe useful helper function for future use
def calibrate_non_blocking(joint_dict):
    # Start calibration for all motors
    for joint_name, joint_obj in joint_dict.items():
        joint_obj.odrv.clear_errors()
        if joint_obj.odrv.axis0.current_state != AxisState.FULL_CALIBRATION_SEQUENCE:
            joint_obj.odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
        custom_sleep(0.2)  # Short delay to ensure command is sent

    # Continuously check if calibration is done for all motors
    all_motors_idle = False
    while not all_motors_idle:
        all_motors_idle = True  # Assume all motors are idle until proven otherwise
        for joint_name, joint_obj in joint_dict.items():
            if joint_obj.odrv.axis0.current_state != AxisState.IDLE:
                all_motors_idle = False  # Found a motor still calibrating
                print(
                    f"Motor {joint_obj.odrv.serial_number} is still calibrating. Current state: {AxisState(joint_obj.odrv.axis0.current_state).name}"
                )
                break  # Exit early since we found a motor still calibrating
        if not all_motors_idle:
            custom_sleep(1)  # Wait a bit before checking again

    # Once all motors are idle, check calibration results
    for joint_name, joint_obj in joint_dict.items():
        results_available = False
        while not results_available:
            if joint_obj.odrv.axis0.procedure_result == ProcedureResult.BUSY:
                custom_sleep(0.1)  # Wait if result is still busy
            else:
                results_available = True  # Result is available, move on to next motor
                # Handle the calibration result for each motor
                if joint_obj.odrv.axis0.procedure_result != ProcedureResult.SUCCESS:
                    print(
                        f"Calibration failed for motor {joint_obj.odrv.serial_number}."
                    )
                else:
                    print(
                        f"Calibration successful for motor {joint_obj.odrv.serial_number}."
                    )


# Maybe useful helper function for future use
def enter_closed_loop_control_non_blocking(joint_dict):
    # Request CLOSED_LOOP_CONTROL state for all motors
    for joint_name, joint_obj in joint_dict.items():
        joint_obj.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        # Short delay to ensure the state request is processed
        custom_sleep(0.2)

    # Continuously check if all motors have entered CLOSED_LOOP_CONTROL
    all_motors_in_closed_loop = False
    while not all_motors_in_closed_loop:
        # Assume all motors are in closed loop until proven otherwise
        all_motors_in_closed_loop = True
        for joint_name, joint_obj in joint_dict.items():
            current_state = joint_obj.odrv.axis0.current_state
            if current_state != AxisState.CLOSED_LOOP_CONTROL:
                all_motors_in_closed_loop = (
                    False  # Found a motor not in CLOSED_LOOP_CONTROL
                )
                print(
                    f"Motor {joint_obj.odrv.serial_number} is still entering closed loop control. Current state: {AxisState(current_state).name}"
                )
                # Assuming dump_errors() is a function that prints out the current errors
                dump_errors(joint_obj.odrv)
                break  # Exit early since we found a motor not in CLOSED_LOOP_CONTROL
        if not all_motors_in_closed_loop:
            custom_sleep(0.5)  # Wait a bit before checking again

    # Once all motors are confirmed to be in CLOSED_LOOP_CONTROL, print success message for each
    for joint_name, joint_obj in joint_dict.items():
        print(
            f"Enter CLOSED_LOOP SUCCESS: Motor {joint_obj.odrv.serial_number} is in state: {AxisState(joint_obj.odrv.axis0.current_state).name}."
        )


# SAMPLE USAGE FOR REFERENCE
def main():
    # TODO find more serial, it is a string of hex of the serial number
    joint_serial_numbers = {
        # 0x386434413539 = 62003024573753 in decimal
        "rover_arm_elbow": "383834583539",  # change as needed
        "rover_arm_shoulder": "386434413539",
        "rover_arm_waist": "395935333231",
        "rover_drive_rf": "385C347A3539",
        "rover_drive_lf": "387134683539",
        "rover_drive_rb": "386134503539",
        "rover_drive_lb": "384F34683539",
    }

    # Define the menu options
    menu_options = {
        "1": "rover_arm_elbow",
        "2": "rover_arm_shoulder",
        "3": "rover_arm_waist",
        "4": "rover_drive_rf",
        "5": "rover_drive_lf",
        "6": "rover_drive_rb",
        "7": "rover_drive_lb",
    }

    # Display the menu to the user
    print("Please select a motor to configure:")
    for option, joint in menu_options.items():
        print(f"{option}: {joint}")

    # Get the user's choice
    choice = input("Enter your choice (1/2/3): ")
    test_joint_name = None
    # Validate and process the choice
    if choice in menu_options:
        test_joint_name = menu_options[choice]
        print(
            f"You selected: {test_joint_name}, serial number: {joint_serial_numbers[test_joint_name]}"
        )
    else:
        print("Invalid choice. Exiting.")
        return

    # Set to True if you want to reapply the config, False if you want to skip it
    reapply_config = True

    # True by default, set to False if you don't want to calibrate
    do_calibration = True

    # if there is a limit switch
    setup_lower_lim_switch = False
    setup_upper_lim_switch = False

    odrv = odrive.find_any(
        serial_number=joint_serial_numbers[test_joint_name], timeout=5
    )

    test_odrv_joint = ODriveJoint(
        name=test_joint_name,
        odrv=odrv,
        gear_ratio=1,
        serial_number=joint_serial_numbers[test_joint_name],
    )

    # ERASE CONFIG -----------------------------------------------------------------------
    if reapply_config:
        print("ERASING CONFIG...")
        test_odrv_joint.erase_config()

    # APPLY CONFIG -----------------------------------------------------------------------
    if test_joint_name == "rover_arm_elbow":
        print("APPLYING CONFIG for rover_arm_elbow...")
        test_odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
        test_odrv_joint.odrv.config.dc_max_positive_current = 6
        test_odrv_joint.odrv.config.dc_max_negative_current = -0.1
        test_odrv_joint.odrv.config.brake_resistor0.enable = True
        test_odrv_joint.odrv.config.brake_resistor0.resistance = 2
        test_odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        test_odrv_joint.odrv.axis0.config.motor.torque_constant = 0.04543956043956044
        test_odrv_joint.odrv.axis0.config.motor.pole_pairs = 7
        test_odrv_joint.odrv.axis0.config.motor.current_soft_max = 6
        test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 17.8
        # test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 12
        test_odrv_joint.odrv.axis0.config.motor.calibration_current = 2.5
        test_odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        test_odrv_joint.odrv.axis0.config.calibration_lockin.current = 2.5
        test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        test_odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.POSITION_CONTROL
        )
        test_odrv_joint.odrv.axis0.controller.config.vel_limit = 4
        test_odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 100
        test_odrv_joint.odrv.axis0.config.torque_soft_min = -2
        test_odrv_joint.odrv.axis0.config.torque_soft_max = 2
        test_odrv_joint.odrv.can.config.protocol = Protocol.NONE
        test_odrv_joint.odrv.config.enable_uart_a = False
        test_odrv_joint.odrv.rs485_encoder_group0.config.mode = (
            Rs485EncoderMode.AMT21_POLLING
        )
        test_odrv_joint.odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
        test_odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0

    if test_joint_name == "rover_arm_shoulder":
        print("APPLYING CONFIG for rover_arm_shoulder...")
        test_odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
        test_odrv_joint.odrv.config.dc_bus_undervoltage_trip_level = 10.5
        test_odrv_joint.odrv.config.dc_max_positive_current = 10
        test_odrv_joint.odrv.config.dc_max_negative_current = -0.1
        test_odrv_joint.odrv.config.brake_resistor0.enable = True
        test_odrv_joint.odrv.config.brake_resistor0.resistance = 2
        test_odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        test_odrv_joint.odrv.axis0.config.motor.torque_constant = 0.06080882352941176
        test_odrv_joint.odrv.axis0.config.motor.pole_pairs = 11
        test_odrv_joint.odrv.axis0.config.motor.current_soft_max = 12
        test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 25.6
        test_odrv_joint.odrv.axis0.config.motor.calibration_current = 2.5
        test_odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        test_odrv_joint.odrv.axis0.config.calibration_lockin.current = 2.5
        test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        test_odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.POSITION_CONTROL
        )
        # vel_limit determines how fast, vel_limit_tolerance determines how much it can go over, so we avoid vel_limit violations
        test_odrv_joint.odrv.axis0.controller.config.vel_limit = 4
        test_odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 100
        test_odrv_joint.odrv.axis0.config.torque_soft_min = -5
        test_odrv_joint.odrv.axis0.config.torque_soft_max = 5
        test_odrv_joint.odrv.can.config.protocol = Protocol.NONE
        test_odrv_joint.odrv.config.enable_uart_a = False
        test_odrv_joint.odrv.rs485_encoder_group0.config.mode = (
            Rs485EncoderMode.AMT21_POLLING
        )
        test_odrv_joint.odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
        test_odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0
    if test_joint_name == "rover_arm_waist":
        print("APPLYING CONFIG for rover_arm_waist...")
        test_odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
        test_odrv_joint.odrv.config.dc_max_positive_current = 6
        test_odrv_joint.odrv.config.dc_max_negative_current = -0.1
        test_odrv_joint.odrv.config.brake_resistor0.enable = True
        test_odrv_joint.odrv.config.brake_resistor0.resistance = 2
        test_odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        test_odrv_joint.odrv.axis0.config.motor.torque_constant = 0.04543956043956044
        test_odrv_joint.odrv.axis0.config.motor.pole_pairs = 7
        test_odrv_joint.odrv.axis0.config.motor.current_soft_max = 6
        test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 17.8
        # test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 12
        test_odrv_joint.odrv.axis0.config.motor.calibration_current = 2.5
        test_odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        test_odrv_joint.odrv.axis0.config.calibration_lockin.current = 2.5
        test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        test_odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.POSITION_CONTROL
        )
        test_odrv_joint.odrv.axis0.controller.config.vel_limit = 4
        test_odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 100
        test_odrv_joint.odrv.axis0.config.torque_soft_min = -2
        test_odrv_joint.odrv.axis0.config.torque_soft_max = 2
        test_odrv_joint.odrv.can.config.protocol = Protocol.NONE
        test_odrv_joint.odrv.config.enable_uart_a = False
        test_odrv_joint.odrv.rs485_encoder_group0.config.mode = (
            Rs485EncoderMode.AMT21_POLLING
        )
        test_odrv_joint.odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
        test_odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0
    if test_joint_name == "rover_drive_rf":
        test_odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
        test_odrv_joint.odrv.config.dc_bus_undervoltage_trip_level = 10.5
        test_odrv_joint.odrv.config.dc_max_positive_current = 12
        test_odrv_joint.odrv.config.dc_max_negative_current = -0.1
        test_odrv_joint.odrv.config.brake_resistor0.enable = True
        test_odrv_joint.odrv.config.brake_resistor0.resistance = 2
        test_odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        test_odrv_joint.odrv.axis0.config.motor.pole_pairs = 7
        test_odrv_joint.odrv.axis0.config.motor.torque_constant = 0.04543956043956044
        test_odrv_joint.odrv.axis0.config.motor.current_soft_max = 10
        test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 23
        test_odrv_joint.odrv.axis0.config.motor.calibration_current = 10
        test_odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        test_odrv_joint.odrv.axis0.config.calibration_lockin.current = 10
        test_odrv_joint.odrv.axis0.motor.motor_thermistor.config.enabled = False
        test_odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.VELOCITY_CONTROL
        )
        # test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
        test_odrv_joint.odrv.axis0.controller.config.vel_limit = 200
        test_odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 2
        test_odrv_joint.odrv.axis0.config.torque_soft_min = -math.inf
        test_odrv_joint.odrv.axis0.config.torque_soft_max = math.inf
        test_odrv_joint.odrv.axis0.trap_traj.config.accel_limit = 200
        test_odrv_joint.odrv.axis0.controller.config.vel_ramp_rate = 200
        test_odrv_joint.odrv.can.config.protocol = Protocol.NONE
        test_odrv_joint.odrv.axis0.config.enable_watchdog = False
        test_odrv_joint.odrv.inc_encoder0.config.enabled = True
        test_odrv_joint.odrv.axis0.config.load_encoder = EncoderId.INC_ENCODER0
        test_odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.INC_ENCODER0
        test_odrv_joint.odrv.inc_encoder0.config.cpr = 2400
        test_odrv_joint.odrv.axis0.commutation_mapper.config.use_index_gpio = False
        test_odrv_joint.odrv.axis0.pos_vel_mapper.config.use_index_gpio = False
        test_odrv_joint.odrv.config.enable_uart_a = False

    if test_joint_name == "rover_drive_lf":
        test_odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
        test_odrv_joint.odrv.config.dc_bus_undervoltage_trip_level = 10.5
        test_odrv_joint.odrv.config.dc_max_positive_current = 12
        test_odrv_joint.odrv.config.dc_max_negative_current = -0.1
        test_odrv_joint.odrv.config.brake_resistor0.enable = True
        test_odrv_joint.odrv.config.brake_resistor0.resistance = 2
        test_odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        test_odrv_joint.odrv.axis0.config.motor.pole_pairs = 7
        test_odrv_joint.odrv.axis0.config.motor.torque_constant = 0.04543956043956044
        test_odrv_joint.odrv.axis0.config.motor.current_soft_max = 10
        test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 23
        test_odrv_joint.odrv.axis0.config.motor.calibration_current = 10
        test_odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        test_odrv_joint.odrv.axis0.config.calibration_lockin.current = 10
        test_odrv_joint.odrv.axis0.motor.motor_thermistor.config.enabled = False
        test_odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.VELOCITY_CONTROL
        )
        # test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
        test_odrv_joint.odrv.axis0.controller.config.vel_limit = 200
        test_odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 2
        test_odrv_joint.odrv.axis0.config.torque_soft_min = -math.inf
        test_odrv_joint.odrv.axis0.config.torque_soft_max = math.inf
        test_odrv_joint.odrv.axis0.trap_traj.config.accel_limit = 200
        test_odrv_joint.odrv.axis0.controller.config.vel_ramp_rate = 200
        test_odrv_joint.odrv.can.config.protocol = Protocol.NONE
        test_odrv_joint.odrv.axis0.config.enable_watchdog = False
        test_odrv_joint.odrv.inc_encoder0.config.enabled = True
        test_odrv_joint.odrv.axis0.config.load_encoder = EncoderId.INC_ENCODER0
        test_odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.INC_ENCODER0
        test_odrv_joint.odrv.inc_encoder0.config.cpr = 2400
        test_odrv_joint.odrv.axis0.commutation_mapper.config.use_index_gpio = False
        test_odrv_joint.odrv.axis0.pos_vel_mapper.config.use_index_gpio = False
        test_odrv_joint.odrv.config.enable_uart_a = False

    if test_joint_name == "rover_drive_rb":
        test_odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
        test_odrv_joint.odrv.config.dc_bus_undervoltage_trip_level = 10.5
        test_odrv_joint.odrv.config.dc_max_positive_current = 12
        test_odrv_joint.odrv.config.dc_max_negative_current = -0.1
        test_odrv_joint.odrv.config.brake_resistor0.enable = True
        test_odrv_joint.odrv.config.brake_resistor0.resistance = 2
        test_odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        test_odrv_joint.odrv.axis0.config.motor.pole_pairs = 7
        test_odrv_joint.odrv.axis0.config.motor.torque_constant = 0.04543956043956044
        test_odrv_joint.odrv.axis0.config.motor.current_soft_max = 10
        test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 23
        test_odrv_joint.odrv.axis0.config.motor.calibration_current = 10
        test_odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        test_odrv_joint.odrv.axis0.config.calibration_lockin.current = 10
        test_odrv_joint.odrv.axis0.motor.motor_thermistor.config.enabled = False
        test_odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.VELOCITY_CONTROL
        )
        # test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
        test_odrv_joint.odrv.axis0.controller.config.vel_limit = 200
        test_odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 2
        test_odrv_joint.odrv.axis0.config.torque_soft_min = -math.inf
        test_odrv_joint.odrv.axis0.config.torque_soft_max = math.inf
        test_odrv_joint.odrv.axis0.trap_traj.config.accel_limit = 200
        test_odrv_joint.odrv.axis0.controller.config.vel_ramp_rate = 200
        test_odrv_joint.odrv.can.config.protocol = Protocol.NONE
        test_odrv_joint.odrv.axis0.config.enable_watchdog = False
        test_odrv_joint.odrv.inc_encoder0.config.enabled = True
        test_odrv_joint.odrv.axis0.config.load_encoder = EncoderId.INC_ENCODER0
        test_odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.INC_ENCODER0
        test_odrv_joint.odrv.inc_encoder0.config.cpr = 2400
        test_odrv_joint.odrv.axis0.commutation_mapper.config.use_index_gpio = False
        test_odrv_joint.odrv.axis0.pos_vel_mapper.config.use_index_gpio = False
        test_odrv_joint.odrv.config.enable_uart_a = False

    if test_joint_name == "rover_drive_lb":
        test_odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
        test_odrv_joint.odrv.config.dc_bus_undervoltage_trip_level = 10.5
        test_odrv_joint.odrv.config.dc_max_positive_current = 12
        test_odrv_joint.odrv.config.dc_max_negative_current = -0.1
        test_odrv_joint.odrv.config.brake_resistor0.enable = True
        test_odrv_joint.odrv.config.brake_resistor0.resistance = 2
        test_odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        test_odrv_joint.odrv.axis0.config.motor.pole_pairs = 7
        test_odrv_joint.odrv.axis0.config.motor.torque_constant = 0.026006289308176098
        test_odrv_joint.odrv.axis0.config.motor.current_soft_max = 10
        test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 23
        test_odrv_joint.odrv.axis0.config.motor.calibration_current = 10
        test_odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        test_odrv_joint.odrv.axis0.config.calibration_lockin.current = 10
        test_odrv_joint.odrv.axis0.motor.motor_thermistor.config.enabled = False
        test_odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.VELOCITY_CONTROL
        )
        # test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
        test_odrv_joint.odrv.axis0.controller.config.vel_limit = 200
        test_odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 2
        test_odrv_joint.odrv.axis0.config.torque_soft_min = -math.inf
        test_odrv_joint.odrv.axis0.config.torque_soft_max = math.inf
        test_odrv_joint.odrv.axis0.trap_traj.config.accel_limit = 200
        test_odrv_joint.odrv.axis0.controller.config.vel_ramp_rate = 200
        test_odrv_joint.odrv.can.config.protocol = Protocol.NONE
        test_odrv_joint.odrv.axis0.config.enable_watchdog = False
        test_odrv_joint.odrv.inc_encoder0.config.enabled = True
        test_odrv_joint.odrv.axis0.config.load_encoder = EncoderId.INC_ENCODER0
        test_odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.INC_ENCODER0
        test_odrv_joint.odrv.inc_encoder0.config.cpr = 2400
        test_odrv_joint.odrv.axis0.commutation_mapper.config.use_index_gpio = False
        test_odrv_joint.odrv.axis0.pos_vel_mapper.config.use_index_gpio = False
        test_odrv_joint.odrv.config.enable_uart_a = False

    # SAVE CONFIG -----------------------------------------------------------------------
    if reapply_config:
        print("SAVING CONFIG...")
        test_odrv_joint.save_config()

    # CONFIG LIMIT SWITCH ---------------------------------------------------------------
    if setup_lower_lim_switch:
        test_odrv_joint.config_lower_limit_switch()

    if setup_upper_lim_switch:
        test_odrv_joint.config_upper_limit_switch()

    # CALIBRATE -------------------------------------------------------------------------
    if do_calibration:
        print("CALIBRATING...")
        test_odrv_joint.calibrate()

    # ENTER CLOSED LOOP CONTROL ---------------------------------------------------------
    print("ENTERING CLOSED LOOP CONTROL...")
    test_odrv_joint.enter_closed_loop_control()

    # TODO untested
    # ENTER HOMING -----------------------------------------------------------------------
    # use absolute position to home
    # test_odrv_joint.odrv.axis0.config.startup_motor_calibration = True
    # test_odrv_joint.odrv.axis0.config.startup_encoder_offset_calibration = True
    # test_odrv_joint.odrv.axis0.config.startup_closed_loop_control = True
    # test_odrv_joint.odrv.axis0.controller.config.absolute_setpoints = True

    # test_odrv_joint.odrv.axis0.pos_vel_mapper.config.offset_valid = True
    # test_odrv_joint.odrv.axis0.pos_vel_mapper.config.approx_init_pos = 0
    # test_odrv_joint.odrv.axis0.pos_vel_mapper.config.approx_init_pos_valid = True

    # test_odrv_joint.odrv.axis0.set_abs_pos(0)

    # test_odrv_joint.enter_homing()

    # SAVE CALIBRATION -----------------------------------------------------------------
    # if reapply_config:
    #     print("SAVING CONFIG again...")
    #     # test_odrv_joint.odrv.axis0.motor.config.pre_calibrated = True
    #     test_odrv_joint.odrv.axis0.config.startup_motor_calibration = True
    #     test_odrv_joint.odrv.axis0.config.startup_encoder_offset_calibration = True
    #     test_odrv_joint.odrv.axis0.config.startup_closed_loop_control = True
    #     test_odrv_joint.save_config()

    # SET ABSOLUTE POSITION ----------------------------------------------------------------
    # abs_pos = 6.9
    # print(f"""SETTING ABSOLUTE POSITION to {abs_pos}")
    # test_odrv_joint.odrv.axis0.set_abs_pos(abs_pos)

    # TODO investigate more, this is not working, NOT A PRIORITY
    # test_odrv_joint.odrv.axis0.pos_vel_mapper.config.offset = 5.5
    # test_odrv_joint.odrv.axis0.pos_vel_mapper.config.offset_valid = True
    # test_odrv_joint.odrv.axis0.pos_vel_mapper.config.approx_init_pos = 0
    # test_odrv_joint.odrv.axis0.pos_vel_mapper.config.approx_init_pos_valid = True
    # test_odrv_joint.odrv.axis0.controller.config.absolute_setpoints = True

    # START WATCHDOG THREAD FOR DEBUG INFO ---------------------------------------------------------
    joint_dict = {test_joint_name: test_odrv_joint}
    watchdog_stop_event = threading.Event()
    watchdog_thread = threading.Thread(
        target=watchdog, args=(joint_dict, watchdog_stop_event)
    )
    watchdog_thread.start()

    # PROMPT FOR SETPOINT (INCREMENTAL AND ABSOLUTE) -----------------------------------------------------
    while True:
        try:
            user_input = input("Enter command (increment 'i X' or absolute 'a X'): ")
            # Using regular expression to parse the input
            match = re.match(r"([ia])\s*(-?\d+(\.\d+)?)", user_input)
            if not match:
                raise ValueError(
                    "Invalid format. Use 'i X' for increment or 'a X' for absolute."
                )

            command, value_str = match.groups()[:2]
            value = float(value_str)

            # Increment command
            if command == "i":  # Incremental command
                setpoint_increment = value
                if math.isnan(test_odrv_joint.odrv.axis0.pos_vel_mapper.pos_abs):
                    current = test_odrv_joint.odrv.axis0.pos_vel_mapper.pos_rel
                else:
                    current = test_odrv_joint.odrv.axis0.pos_vel_mapper.pos_abs
                setpoint = current + (setpoint_increment * test_odrv_joint.gear_ratio)
                print(
                    f"""INCREMENTING {setpoint_increment}, setpoint={setpoint}, pos_rel={
                      test_odrv_joint.odrv.axis0.pos_vel_mapper.pos_rel}, current_state={test_odrv_joint.odrv.axis0.current_state}"""
                )
                test_odrv_joint.odrv.axis0.controller.input_pos = setpoint

            # Setpoint command
            elif command == "s":
                setpoint = value
                print(
                    f"""SETTING SETPOINT to {setpoint}, current_state={
                        test_odrv_joint.odrv.axis0.current_state}"""
                )
                if (
                    test_odrv_joint.odrv.axis0.controller.config.control_mode
                    == ControlMode.VELOCITY_CONTROL
                ):
                    test_odrv_joint.odrv.axis0.controller.input_vel = setpoint
                elif (
                    test_odrv_joint.odrv.axis0.controller.config.control_mode
                    == ControlMode.POSITION_CONTROL
                ):
                    test_odrv_joint.odrv.axis0.controller.input_pos = setpoint

            # Absolute command
            elif command == "a":
                print(
                    f"""SETTING ABSOLUTE POSITION to {value}, current_state={
                        test_odrv_joint.odrv.axis0.current_state}"""
                )
                test_odrv_joint.odrv.axis0.set_abs_pos(value)

            # Apply the setpoint
            dump_errors(test_odrv_joint.odrv)

        except ValueError as e:
            print(e)
            # Skip the rest of the loop and prompt for input again
            continue

        custom_sleep(0.01)

    # Stop watchdog thread, when closing the script -------------------------------------------------
    watchdog_stop_event.set()
    watchdog_thread.join()


if __name__ == "__main__":
    main()
