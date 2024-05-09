# Author: mn297
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

def print_joint_state_from_dict(joint_dict):
    for joint_name, joint_obj in joint_dict.items():
        # Check if the odrive is connected
        status = "connected" if joint_obj.odrv else "disconnected, odrv=None"
        print(
            f"""{joint_name} {joint_obj.serial_number} ({status}) is_reconnecting={joint_obj.is_reconnecting}""")
        if joint_obj.odrv and not joint_obj.is_reconnecting:
            try:
                status = "connected"
                dump_errors(joint_obj.odrv)
                print(f"""-current_state={AxisState(joint_obj.odrv.axis0.current_state).name}""")
                print(f"""-pos_rel={joint_obj.odrv.axis0.pos_vel_mapper.pos_rel}""")
                print(f"""-pos_abs={joint_obj.odrv.axis0.pos_vel_mapper.pos_abs}""")
                print(f"""-input_pos={joint_obj.odrv.axis0.controller.input_pos}""")
                print(f"""-vel_estimate={joint_obj.odrv.encoder_estimator0.vel_estimate}""")
            except:
                print(f"""-pos_rel=None""")
                print(f"""-pos_abs=None""")
        print(f"""-pos_cmd={joint_obj.pos_cmd}""")
        print(f"""-vel_cmd={joint_obj.vel_cmd}""")
        print()


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
        self.odrv.axis0.max_endstop.config.enabled = True
        self.odrv.axis0.max_endstop.config.is_active_high = False

        print("upper limit pin is enabled")

        # verify it is on mode high
        print(
            "initial state of limit switch pin 10 is : ",
            f"""{bin(self.odrv.get_gpio_states())[2:]:0>12}"""[2],
        )

    def enter_homing(self):
        self.odrv.axis0.requested_state = AxisState.HOMING


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
        except TimeoutError:
            print("TimeoutError in reconnect() ...")
            pass
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
        while (not self.odrv.axis0.current_state == AxisState.IDLE):
            custom_sleep(1)
            print("Motor {} is still calibrating. Current state: {}".format(self.name, AxisState(self.odrv.axis0.current_state).name))
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
                print("Motor error(s) in motor {}: {}".format(self.odrv.serial_number, AxisError(self.odrv.axis0.disarm_reason).name))
            
            print("Calibration procedure failed in motor {}. Reason: {}".format(self.name,ProcedureResult(result).name))

            if result == ProcedureResult.DISARMED:
                print("Motor {} disarmed. Reason: {}".format(self.odrv.serial_number,AxisError(self.odrv.axis0.disarm_reason).name))

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
            print("Motor {} is still entering closed loop control. Current state: {}".format(self.name, AxisState(self.odrv.axis0.current_state).name))
            dump_errors(self.odrv)
        
        print("SUCCESS: Motor {} is in state: {}.".format(self.odrv.serial_number, AxisState(self.odrv.axis0.current_state).name))

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
            print("voltage on GPIO{} is {} Volt".format(i, self.odrv.get_adc_voltage(i))
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
                print(f"Motor {joint_obj.odrv.serial_number} is still calibrating. Current state: {AxisState(joint_obj.odrv.axis0.current_state).name}")
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
                print(f"Motor {joint_obj.odrv.serial_number} is still entering closed loop control. Current state: {AxisState(current_state).name}")
                # Assuming dump_errors() is a function that prints out the current errors
                dump_errors(joint_obj.odrv)
                break  # Exit early since we found a motor not in CLOSED_LOOP_CONTROL
        if not all_motors_in_closed_loop:
            custom_sleep(0.5)  # Wait a bit before checking again

    # Once all motors are confirmed to be in CLOSED_LOOP_CONTROL, print success message for each
    for joint_name, joint_obj in joint_dict.items():
        print(f"Enter CLOSED_LOOP SUCCESS: Motor {joint_obj.odrv.serial_number} is in state: {AxisState(joint_obj.odrv.axis0.current_state).name}.")
