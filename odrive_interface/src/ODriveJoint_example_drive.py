from __future__ import print_function

import odrive
from odrive.config import Axis
import odrive.enums
import odrive.utils
import odrive.config

from odrive.enums import *
from odrive.utils import dump_errors

from ODriveJoint import *

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


# SAMPLE USAGE FOR REFERENCE
def main():
    # TODO find more serial, it is a string of hex of the serial number
    arm_serial_numbers = {
        # 0x386434413539 = 62003024573753 in decimal
        "rover_arm_elbow": "383834583539",  # change as needed
        "rover_arm_shoulder": "386434413539",
        "rover_arm_waist": "0",
    }

    drive_serial_numbers = {
        "rover_drive_rf": "384F34683539",
        "rover_drive_lf": "386134503539",
        "rover_drive_rb": "387134683539",
        "rover_drive_lb": "385C347A3539",
    }
    joint_dict = {}

    # Set to True if you want to reapply the config, False if you want to skip it
    reapply_config = True

    # True by default, set to False if you don't want to calibrate
    do_calibration = True

    # SETUP ODRIVE CONNECTIONS -----------------------------------------------------
    for key, value in drive_serial_numbers.items():
        if value == 0:
            print(
                f"""Skipping connection for joint: {
                    key} due to serial_number being 0"""
            )
            # Instantiate class with odrv as None because we're skipping connection
            joint_dict[key] = ODriveJoint(
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
        joint_dict[key] = ODriveJoint(
            odrv=odrv,
        )

    # ERASE CONFIG -----------------------------------------------------------------------
    if reapply_config:
        print("ERASING CONFIG...")
        for joint_name, joint_obj in joint_dict.items():
            joint_obj.erase_config()

    # APPLY CONFIG -----------------------------------------------------------------------
    if reapply_config:
        print("APPLYING CONFIG...")
        for joint_name, joint_obj in joint_dict.items():
            joint_obj.apply_config()

    # SAVE CONFIG -----------------------------------------------------------------------
    if reapply_config:
        print("SAVING CONFIG...")
        for joint_name, joint_obj in joint_dict.items():
            joint_obj.save_config()

    # CALIBRATE -------------------------------------------------------------------------
    if do_calibration:
        print("CALIBRATING...")
        for joint_name, joint_obj in joint_dict.items():
            joint_obj.calibrate()

    # ENTER CLOSED LOOP CONTROL ---------------------------------------------------------
    print("ENTERING CLOSED LOOP CONTROL...")
    for joint_name, joint_obj in joint_dict.items():
        joint_obj.enter_closed_loop_control()

    # START WATCHDOG THREAD FOR DEBUG INFO ---------------------------------------------------------
    ODriveJoint_lst = list(joint_dict.values())
    watchdog_stop_event = threading.Event()
    watchdog_thread = threading.Thread(
        target=watchdog, args=(ODriveJoint_lst, watchdog_stop_event)
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
            # if command == "i":  # Incremental command
            #     setpoint_increment = value
            #     for joint_obj in ODriveJoint_lst:
            #         if math.isnan(joint_obj.odrv.axis0.pos_vel_mapper.pos_abs):
            #             current = joint_obj.odrv.axis0.pos_vel_mapper.pos_rel
            #         else:
            #             current = joint_obj.odrv.axis0.pos_vel_mapper.pos_abs
            #         setpoint = current + (setpoint_increment * joint_obj.gear_ratio)
            #         print(
            #             f"""INCREMENTING {setpoint_increment}, setpoint={setpoint}, pos_rel={
            #             joint_obj.odrv.axis0.pos_vel_mapper.pos_rel}, current_state={joint_obj.odrv.axis0.current_state}"""
            #         )
            #         joint_obj.odrv.axis0.controller.input_pos = setpoint

            # Setpoint command
            if command == "s":
                setpoint = value
                for joint_obj in ODriveJoint_lst:
                    print(
                        f"""SETTING SPEED to {setpoint}, current_state={
                            joint_obj.odrv.axis0.current_state}"""
                    )
                    joint_obj.odrv.axis0.controller.input_vel = setpoint

            # # Absolute command
            # elif command == "a":
            #     for joint_obj in ODriveJoint_lst:
            #         print(
            #             f"""SETTING ABSOLUTE POSITION to {value}, current_state={
            #                 joint_obj.odrv.axis0.current_state}"""
            #         )
            #         joint_obj.odrv.axis0.set_abs_pos(value)

            for joint_obj in ODriveJoint_lst:
                dump_errors(joint_obj.odrv)

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
