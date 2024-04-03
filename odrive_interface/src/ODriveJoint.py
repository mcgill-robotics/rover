from __future__ import print_function

import odrive
from odrive.config import Axis
import odrive.enums
import odrive.utils
import odrive.config

from odrive.enums import *
from odrive.utils import dump_errors

import time
import threading
import fibre
import re
import math

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


# def calibrate_and_enter_closed_loop(joint_obj):
#     if joint_obj.odrv is not None:
#         print(f"CALIBRATING joint {joint_obj.name}...")
#         joint_obj.calibrate()

#         print(f"ENTERING CLOSED LOOP CONTROL for joint {joint_obj.name}...")
#         joint_obj.enter_closed_loop_control()

#         self.is_calibrated = True


# def detect_odrive_hardware(joint_obj):
#     try:
#         # Attempt to CONNECT TO ODRIVE only if serial_number is not 0
#         joint_obj.attach_odrive()
#         print(
#             f"""Connected joint: {joint_obj.name}, serial_number: {joint_obj.serial_number}"""
#         )
#     except Exception as e:
#         print(
#             f"""Cannot connect joint: {joint_obj.name}, serial_number: {joint_obj.serial_number}. Error: {e}"""
#         )


def print_joint_state_from_dict(joint_dict):
    for joint_name, joint_obj in joint_dict.items():
        # Check if the odrive is connected
        status = "connected" if joint_obj.odrv else "disconnected"
        print(f"""{joint_name} {joint_obj.serial_number} ({status})""")
        if joint_obj.odrv:
            try:
                print(
                    f"""-current_state={
                        AxisState(joint_obj.odrv.axis0.current_state).name}"""
                )
                print(f"""-pos_rel={joint_obj.odrv.axis0.pos_vel_mapper.pos_rel}""")
                print(f"""-pos_abs={joint_obj.odrv.axis0.pos_vel_mapper.pos_abs}""")
                print(f"""-input_pos={joint_obj.odrv.axis0.controller.input_pos}""")
                print(
                    f"""-vel_estimate={joint_obj.odrv.encoder_estimator0.vel_estimate}"""
                )
            except:
                print(f"""-pos_rel=None""")
                print(f"""-pos_abs=None""")
        print(f"""-pos_cmd={joint_obj.pos_cmd}""")
        print(f"""-vel_cmd={joint_obj.vel_cmd}""")


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

    def attach_odrive(self, serial_number=None):
        self.serial_number = serial_number
        try:
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
            if self.odrv:
                self.serial_number = str(hex(self.odrv.serial_number)[2:])
        except Exception as e:
            print(
                f"""Cannot connect joint: {self.name}, serial_number: {self.serial_number}. Error: {e}"""
            )

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
                "Motor {} is still entering homing. Current state: {}".format(
                    self.odrv.serial_number,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
            dump_errors(self.odrv)
        print(
            "SUCCESS: Motor {} is in state: {}.".format(
                self.odrv.serial_number,
                AxisState(self.odrv.axis0.current_state).name,
            )
        )
        # block until homing is done
        while self.odrv.axis0.current_state == AxisState.HOMING:
            custom_sleep(1)
            print(
                "Motor {} is still homing. Current state: {}".format(
                    self.odrv.serial_number,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
            dump_errors(self.odrv)

    def attach_odrive(self, odrv):
        self.odrv = odrv
        if self.odrv:
            self.serial_number = str(hex(self.odrv.serial_number)[2:])

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
        try:
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
        except fibre.libfibre.ObjectLostError:
            print(" lost connection in reconnect() ...")
            self.odrv = odrive.find_any(
                serial_number=self.serial_number, timeout=self.timeout
            )
            if self.odrv:
                print("  re-connected in reconnect()")
            pass

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
                    self.odrv.serial_number,
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
                    self.odrv.serial_number, ProcedureResult(result).name
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
        self.odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        while (
            self.odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL
            or self.odrv.axis0.current_state == AxisState.IDLE
        ):
            custom_sleep(0.5)
            print(
                "Motor {} is still entering closed loop control. Current state: {}".format(
                    self.odrv.serial_number,
                    AxisState(self.odrv.axis0.current_state).name,
                )
            )
            dump_errors(self.odrv)
        print(
            "SUCCESS: Motor {} is in state: {}.".format(
                self.odrv.serial_number,
                AxisState(self.odrv.axis0.current_state).name,
            )
        )

    # Print the voltage on the GPIO pins
    def print_gpio_voltages(self):
        for i in [1, 2, 3, 4]:
            print(
                "voltage on GPIO{} is {} Volt".format(i, self.odrv.get_adc_voltage(i))
            )


# def calibrate_joint(joint_name, joint_obj):
#     joint_obj.odrv.clear_errors()
#     if joint_obj.odrv.axis0.current_state != AxisState.FULL_CALIBRATION_SEQUENCE:
#         joint_obj.odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
#     time.sleep(0.2)

#     # Wait for calibration to end
#     while not joint_obj.odrv.axis0.current_state == AxisState.IDLE:
#         time.sleep(1)
#         print(f"Motor {joint_obj.serial_number} is still calibrating. Current state: {AxisState(joint_obj.odrv.axis0.current_state).name}")

#     results_available = False
#     # Wait for calibration results
#     while not results_available:
#         if joint_obj.odrv.axis0.procedure_result == ProcedureResult.BUSY:
#             results_available = False
#         else:
#             results_available = True
#         time.sleep(0.1)

#     # ERROR CHECKING
#     if joint_obj.odrv.axis0.procedure_result != ProcedureResult.SUCCESS:
#         errors = joint_obj.odrv.axis0.active_errors
#         error_message = f"Calibration procedure failed in motor {joint_obj.serial_number}. Reason: {ProcedureResult(joint_obj.odrv.axis0.procedure_result).name}"
#         if errors != 0:
#             error_message += f", Motor error(s): {AxisError(joint_obj.odrv.axis0.disarm_reason).name}"
#         print(error_message)
#         joint_obj.odrv.axis0.requested_state = AxisState.IDLE
#     else:
#         print(f"Calibration successful for motor {joint_obj.serial_number}!")


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
    arm_serial_numbers = {
        # 0x386434413539 = 62003024573753 in decimal
        "rover_arm_elbow": "383834583539",  # change as needed
        "rover_arm_shoulder": "386434413539",
        "rover_arm_waist": "0",
    }

    test_joint_name = "rover_arm_shoulder"
    # test_joint_name = "rover_arm_elbow"

    # Set to True if you want to reapply the config, False if you want to skip it
    reapply_config = True

    # True by default, set to False if you don't want to calibrate
    do_calibration = True

    # if there is a limit switch
    setup_lower_lim_switch = False
    setup_upper_lim_switch = False

    test_odrv_joint = ODriveJoint(
        odrive.find_any(serial_number=arm_serial_numbers[test_joint_name], timeout=5)
    )

    # ERASE CONFIG -----------------------------------------------------------------------
    if reapply_config:
        print("ERASING CONFIG...")
        test_odrv_joint.erase_config()

    # APPLY CONFIG -----------------------------------------------------------------------
    if test_joint_name == "rover_arm_shoulder":
        test_odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
        test_odrv_joint.odrv.config.dc_bus_undervoltage_trip_level = 10.5
        test_odrv_joint.odrv.config.dc_max_positive_current = 10
        test_odrv_joint.odrv.config.brake_resistor0.enable = True
        test_odrv_joint.odrv.config.brake_resistor0.resistance = 2
        test_odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        test_odrv_joint.odrv.axis0.config.motor.torque_constant = 0.06080882352941176
        test_odrv_joint.odrv.axis0.config.motor.pole_pairs = 11
        test_odrv_joint.odrv.axis0.config.motor.current_soft_max = 10
        test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 23
        test_odrv_joint.odrv.axis0.config.motor.calibration_current = 2.5
        test_odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        test_odrv_joint.odrv.axis0.config.calibration_lockin.current = 2.5
        test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        test_odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.POSITION_CONTROL
        )
        # vel_limit determines how fast, vel_limit_tolerance determines how much it can go over, so we avoid vel_limit violations
        test_odrv_joint.odrv.axis0.controller.config.vel_limit = 3
        test_odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 2
        test_odrv_joint.odrv.axis0.config.torque_soft_min = -0.7
        test_odrv_joint.odrv.axis0.config.torque_soft_max = 0.7
        test_odrv_joint.odrv.can.config.protocol = Protocol.NONE
        test_odrv_joint.odrv.config.enable_uart_a = False
        test_odrv_joint.odrv.rs485_encoder_group0.config.mode = (
            Rs485EncoderMode.AMT21_POLLING
        )
        test_odrv_joint.odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
        test_odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0

    if test_joint_name == "rover_arm_elbow":
        test_odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
        test_odrv_joint.odrv.config.dc_max_positive_current = 5
        test_odrv_joint.odrv.config.brake_resistor0.enable = True
        test_odrv_joint.odrv.config.brake_resistor0.resistance = 2
        test_odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        test_odrv_joint.odrv.axis0.config.motor.torque_constant = 0.04543956043956044
        test_odrv_joint.odrv.axis0.config.motor.pole_pairs = 7
        test_odrv_joint.odrv.axis0.config.motor.current_soft_max = 5
        test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 16.5
        # test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 12
        test_odrv_joint.odrv.axis0.config.motor.calibration_current = 2.5
        test_odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        test_odrv_joint.odrv.axis0.config.calibration_lockin.current = 2.5
        test_odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        test_odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.POSITION_CONTROL
        )
        test_odrv_joint.odrv.axis0.controller.config.vel_limit = 6
        test_odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 1.2
        test_odrv_joint.odrv.axis0.config.torque_soft_min = -0.1
        test_odrv_joint.odrv.axis0.config.torque_soft_max = 0.1
        test_odrv_joint.odrv.can.config.protocol = Protocol.NONE
        test_odrv_joint.odrv.config.enable_uart_a = False
        test_odrv_joint.odrv.rs485_encoder_group0.config.mode = (
            Rs485EncoderMode.AMT21_POLLING
        )
        test_odrv_joint.odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
        test_odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0

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
