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


def watchdog(ODrive_Joint_lst, watchdog_stop_event):
    while not watchdog_stop_event.is_set():
        try:
            for joint in ODrive_Joint_lst:
                print(
                    "current_state="
                    + str(AxisState(joint.odrv.axis0.current_state).name)
                    + ", "
                    + "Raw angle="
                    + str(joint.odrv.rs485_encoder_group0.raw)
                    + ", "
                    + "pos_rel="
                    + str(joint.odrv.axis0.pos_vel_mapper.pos_rel)
                    + ", "
                    + "pos_abs="
                    + str(joint.odrv.axis0.pos_vel_mapper.pos_abs)
                    + ", "
                    + "input_pos="
                    + str(joint.odrv.axis0.controller.input_pos)
                    + ", "
                    + "vel_estimate="
                    + str(joint.odrv.encoder_estimator0.vel_estimate)
                    + ", "
                    + "lower_limit_switch_pin_state="
                    + f'{bin(joint.odrv.get_gpio_states())[2:]:0>12}'[0]
                    + ", "
                    + "upper_limit_switch_pin_state="
                    + f'{bin(joint.odrv.get_gpio_states())[2:]:0>12}'[2]
                )
            custom_sleep(1)
        except NameError:
            pass
        except fibre.libfibre.ObjectLostError:
            print(" lost connection in watchdog() ...")
            for odrv_shoulder in ODrive_Joint_lst:
                odrv_shoulder.odrv = odrive.find_any(
                    serial_number=odrv_shoulder.serial_number,
                    timeout=odrv_shoulder.timeout,
                )
                if odrv_shoulder.odrv:
                    print("  re-connected in watchdog()")
            pass


class ODrive_Joint:
    def __init__(self, odrv=None, gear_ratio=1):
        self.odrv = odrv
        # odrv.serial_number is int, serial_number should be hex version in string
        if self.odrv:
            self.serial_number = str(hex(self.odrv.serial_number)[2:])
        else:
            self.serial_number = None
        self.timeout = 5
        # gear_ratio is input revolutions / output revolutions
        self.gear_ratio = gear_ratio
    
    def lower_limit_switch(self):
        print("starting lower limit switch config...")
        self.odrv.config.gpio12_mode = GpioMode.DIGITAL #the G12 pin is a configurable GPIO (see Odrive documentation)
        self.odrv.axis0.min_endstop.config.gpio_num = 12
        self.odrv.axis0.min_endstop.config.enabled = True
        self.odrv.axis0.min_endstop.config.is_active_high = False
        # while (not odrv_shoulder.odrv.axis0.min_endstop.endstop_state):
        #     custom_sleep(0.1)
            
        print("lim switch pin is enabled")

        # verify it is on mode high
        print("initial state of limit switch pin 12 is : ", f'{bin(self.odrv.get_gpio_states())[2:]:0>12}'[0])

    def upper_limit_switch(self):
        print("starting upper limit switch config...")
        self.odrv.config.gpio10_mode = GpioMode.DIGITAL #the G10 pin is a configurable GPIO (see Odrive documentation)
        self.odrv.axis0.max_endstop.config.gpio_num = 10
        self.odrv.axis0.max_endstop.config.enabled = True
        self.odrv.axis0.max_endstop.config.is_active_high = False
        # while (not odrv_shoulder.odrv.axis0.min_endstop.endstop_state):
        #     custom_sleep(0.1)
            
        print("lim switch pin is enabled")

        # verify it is on mode high
        print("initial state of limit switch pin 12 is : ", f'{bin(self.odrv.get_gpio_states())[2:]:0>12}'[2])

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


# SAMPLE USAGE FOR REFERENCE
def main():
    # TODO find more serial, it is a string of hex of the serial number
    arm_serial_numbers = {
        "rover_arm_shoulder": "386434413539",  # 0x386434413539 = 62003024573753 in decimal
        "rover_arm_elbow": "0",
        "rover_arm_waist": "0",
    }

    # Set to True if you want to reapply the config, False if you want to skip it
    reapply_config = True

    # True by default, set to False if you don't want to calibrate
    do_calibration = True

    # if there is a limit switch
    set_up_lower_lim_switch = True
    set_up_upper_lim_switch = False

    odrv_shoulder = ODrive_Joint(
        odrive.find_any(
            serial_number=arm_serial_numbers["rover_arm_shoulder"], timeout=5
        )
    )

    # ERASE CONFIG -----------------------------------------------------------------------
    if reapply_config:
        print("ERASING CONFIG...")
        odrv_shoulder.erase_config()

    # APPLY CONFIG -----------------------------------------------------------------------
    odrv_shoulder.odrv.config.dc_bus_overvoltage_trip_level = 30
    odrv_shoulder.odrv.config.dc_bus_undervoltage_trip_level = 10.5
    odrv_shoulder.odrv.config.dc_max_positive_current = 10
    odrv_shoulder.odrv.config.brake_resistor0.enable = True
    odrv_shoulder.odrv.config.brake_resistor0.resistance = 2
    odrv_shoulder.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv_shoulder.odrv.axis0.config.motor.torque_constant = 0.06080882352941176
    odrv_shoulder.odrv.axis0.config.motor.pole_pairs = 11
    odrv_shoulder.odrv.axis0.config.motor.current_soft_max = 10
    odrv_shoulder.odrv.axis0.config.motor.current_hard_max = 23
    odrv_shoulder.odrv.axis0.config.motor.calibration_current = 2.5
    odrv_shoulder.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
    odrv_shoulder.odrv.axis0.config.calibration_lockin.current = 2.5
    odrv_shoulder.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
    odrv_shoulder.odrv.axis0.controller.config.control_mode = (
        ControlMode.POSITION_CONTROL
    )
    odrv_shoulder.odrv.axis0.config.torque_soft_min = -0.12161764705882352
    odrv_shoulder.odrv.axis0.config.torque_soft_max = 0.12161764705882352
    odrv_shoulder.odrv.can.config.protocol = Protocol.NONE
    odrv_shoulder.odrv.config.enable_uart_a = False
    odrv_shoulder.odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_POLLING
    odrv_shoulder.odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
    odrv_shoulder.odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0

    # SAVE CONFIG -----------------------------------------------------------------------
    if reapply_config:
        print("SAVING CONFIG...")
        odrv_shoulder.save_config()

    # CONFIG LIMIT SWITCH ---------------------------------------------------------------
    if set_up_lower_lim_switch:
        odrv_shoulder.lower_limit_switch()

    if set_up_upper_lim_switch:
        odrv_shoulder.upper_limit_switch()

    # CALIBRATE -------------------------------------------------------------------------
    if do_calibration:
        print("CALIBRATING...")
        odrv_shoulder.calibrate()

    # ENTER CLOSED LOOP CONTROL ---------------------------------------------------------
    print("ENTERING CLOSED LOOP CONTROL...")
    odrv_shoulder.enter_closed_loop_control()

    # SAVE CALIBRATION -----------------------------------------------------------------
    if reapply_config:
        # odrv_shoulder.odrv.axis0.motor.config.pre_calibrated = True
        odrv_shoulder.odrv.axis0.config.startup_motor_calibration = True
        odrv_shoulder.odrv.axis0.config.startup_encoder_offset_calibration = True
        odrv_shoulder.odrv.axis0.config.startup_closed_loop_control = True
        odrv_shoulder.save_config()

    # SET ABSOLUTE POSITION ----------------------------------------------------------------
    odrv_shoulder.odrv.axis0.set_abs_pos(6.9)

    # TODO investigate more, this is not working
    # odrv_shoulder.odrv.axis0.pos_vel_mapper.config.offset = 5.5
    # odrv_shoulder.odrv.axis0.pos_vel_mapper.config.offset_valid = True
    # odrv_shoulder.odrv.axis0.pos_vel_mapper.config.approx_init_pos = 0
    # odrv_shoulder.odrv.axis0.pos_vel_mapper.config.approx_init_pos_valid = True
    # odrv_shoulder.odrv.axis0.controller.config.absolute_setpoints = True

    # START WATCHDOG THREAD FOR DEBUG INFO ---------------------------------------------------------
    ODrive_Joint_lst = [odrv_shoulder]
    watchdog_stop_event = threading.Event()
    watchdog_thread = threading.Thread(
        target=watchdog, args=(ODrive_Joint_lst, watchdog_stop_event)
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
                setpoint = odrv_shoulder.odrv.axis0.pos_vel_mapper.pos_rel + (
                    setpoint_increment * odrv_shoulder.gear_ratio
                )
                print(
                    f"INCREMENTING {setpoint_increment}, setpoint={setpoint}, pos_rel={odrv_shoulder.odrv.axis0.pos_vel_mapper.pos_rel}, current_state={odrv_shoulder.odrv.axis0.current_state}"
                )
                odrv_shoulder.odrv.axis0.controller.input_pos = setpoint

            # Setpoint command
            elif command == "s":
                setpoint = value
                print(
                    f"SETTING SETPOINT to {setpoint}, current_state={odrv_shoulder.odrv.axis0.current_state}"
                )
                odrv_shoulder.odrv.axis0.controller.input_pos = setpoint

            # Absolute command
            elif command == "a":
                print(
                    f"SETTING ABSOLUTE POSITION to {value}, current_state={odrv_shoulder.odrv.axis0.current_state}"
                )
                odrv_shoulder.odrv.axis0.set_abs_pos(value)

            # Apply the setpoint
            dump_errors(odrv_shoulder.odrv)

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
