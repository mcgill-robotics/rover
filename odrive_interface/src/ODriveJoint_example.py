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

    # test_joint_name = "rover_arm_shoulder"
    # test_joint_name = "rover_arm_elbow"

    # Set to True if you want to reapply the config, False if you want to skip it
    reapply_config = True 

    # True by default, set to False if you don't want to calibrate
    do_calibration = True  



    test_odrv_joint = ODriveJoint(
        odrive.find_any(serial_number=arm_serial_numbers[test_joint_name], timeout=5)
    )

    # ERASE CONFIG -----------------------------------------------------------------------
    if reapply_config:  
        print("ERASING CONFIG...")
        test_odrv_joint.erase_config()   

    # APPLY CONFIG -----------------------------------------------------------------------
    if test_joint_name == "rover_arm_shoulder":  
          

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

    # START WATCHDOG THREAD FOR DEBUG INFO ---------------------------------------------------------
    ODriveJoint_lst = [test_odrv_joint]
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
