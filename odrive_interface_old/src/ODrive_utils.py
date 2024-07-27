import rospy
import odrive
from odrive.enums import AxisState, ProcedureResult 

"""
Placeholders for now. In enumerate_motors(), these motors will be returned in this order. Serial numbers are also strings,
because ODrive API requires them to be that way. Note: correct serial numbers are hex values, while odrivetool returns
decimal. So before putting the numbers in here, you should first convert them to hex.
"""

drive_serial_numbers = {"DRIVE_LB": "385C347A3539", "DRIVE_LF": "386134503539", "DRIVE_RB": "387134683539", "DRIVE_RF": "384F34683539"}
arm_serial_numbers = {"ARM_WAIST": "4", "ARM_TUMOR": "5", "ARM_ELBOW": "6"}

drive_ids = {'385C347A3539': 'DRIVE_LB', '386134503539': 'DRIVE_LF', '387134683539': 'DRIVE_RB', '384F34683539': 'DRIVE_RF'}
arm_ids = {'4': 'ARM_WAIST', '5': 'ARM_TUMOR', '6': 'ARM_ELBOW'}

# Procedure codes are simple indices to this array
procedure_codes = ["SUCCESS", "BUSY", "CANCELLED", "DISARMED", "NO_RESPONSE", "POLE_PAIR_CPR_MISMATCH",
                   "PHASE_RESISTANCE_OUT_OF_RANGE", "PHASE_INDUCTANCE_OUT_OF_RANGE", "UNBALANCED_PHASES",
                   "UNBALANCED_PHASES", "INVALID_MOTOR_TYPE", "ILLEGAL_HALL_STATE", "TIMEOUT", "HOMING_WITHOUT_ENDSTOP",
                   "INVALID_STATE", "NOT_CALIBRATED", "NOT_CONVERGING"]

# Errors are bitwise encoded
error_codes = {1: "INITIALIZING", 2: "SYSTEM_LEVEL", 4: "TIMING_ERROR",
               8: "MISSING_ESTIMATE", 16: "BAD_CONFIG", 32: "DRV_FAULT",
               64: "MISSING_INPUT", 256: "DC_BUS_OVER_VOLTAGE",
               512: "DC_BUS_UNDER_VOLTAGE", 1024: "DC_BUS_OVER_CURRENT",
               2048: "DC_BUS_OVER_REGEN_CURRENT", 4096: "CURRENT_LIMIT_VIOLATION",
               8192: "MOTOR_OVER_TEMP", 16384: "INVERTER_OVER_TEMP",
               32768: "VELOCITY_LIMIT_VIOLATION", 65536: "POSITION_LIMIT_VIOLATION",
               16777216: "WATCHDOG_TIMER_EXPIRED", 33554432: "ESTOP_REQUESTED",
               67108864: "SPINOUT_DETECTED", 134217728: "BRAKE_RESISTOR_DISARMED",
               268435456: "THERMISTOR_DISCONNECTED", 1073741824: "CALIBRATION_ERROR"}

def decode_errors(n):
    """
    Decodes the error code received by an ODrive. Errors in an ODrive are encoded bitwise, where
    each error is a power of 2, and the final error code is obtained by adding them up to a single uint32_t.
    On the ROS end, this is decoded by masking the bits one by one and referring to the error_codes dictionary.
    This is converted to a string, because it will be transmitted like that to the rest of the nodes.

    :param n: Error code to be decoded, would be an uint32_t if this wasn't Python
    :returns: A string representing the active errors on the ODrive in comma-separated format
    """
    num_bits = n.bit_length()
    errors_string = ""
    for i in range(num_bits):
        mask = 1 << i
        if n & mask:
            errors_string += (error_codes[mask] + ", ")

    # Strip the last two characters (", ") before returning the error string
    return errors_string[:-2]

def enumerate_motors(search_timeout=5):
    """
    Finds all motors by their serial numbers as given in the drive_serial_numbers dict. It will search each
    motor for search_timeout seconds, after which it returns an empty dict signifying an error. This error should
    be caught and handled downstream in the main ODrive node. On success, it returns a dict where keys are the
    motor names and values are the corresponding ODrive objects.
    
    :param search_timeout: Timeout for a single motor search. Keep in mind that the search is blocking. Default is 5 seconds.
    :returns: Dict{"LB_MOTOR": ODrive, "LF_MOTOR": ODrive...}
    """
    print("Waiting for motors...")
    found_motors = {}

    # Keep things limited to the drive motors for now.
    # TODO: Discuss with software what to do with the arm motors
    for key, value in drive_serial_numbers.items():
        print(f"Searching for motor {key} with serial number {value}...")
        found_motor = odrive.find_any(serial_number=value, timeout=search_timeout)
        if found_motor is not None:
            print(f"Motor {key} found.")
            found_motors[key] = found_motor
        else:
            print(f"Motor {key} not found. Motor enumeration failed!")
            return {}
    
    return found_motors


def calibrate_motors(motor_array):
    """
    Calibrates all motors in motor_array at once. On failure, this function sets all listed motors to
    idle and immediately return False. The errors will be handled downstream in the ODrive node.

    :param motor_array: Array of ODrive objects
    :returns: A boolean to flag success/failure of setup
    """
    # Ask the devices to calibrate if it isn't calibrating already, and wait for a short time for them to respond
    print("Calibrating encoders...")
    for odrv in motor_array:
        if odrv.axis0.current_state != AxisState.ENCODER_OFFSET_CALIBRATION:
            odrv.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    rospy.sleep(0.02)

    # Verify that the devices are in calibration mode. If the calibration fails for one motor, abort the calibration
    for odrv in motor_array:
        if odrv.axis0.current_state != AxisState.ENCODER_OFFSET_CALIBRATION:
            print("Motor {} could not enter calibration mode, calibration aborted. Error(s) occurred: {}".format(odrv.serial_number, decode_errors(odrv.axis0.active_errors)))
            for odrv in motor_array:
                odrv.axis0.requested_state = AxisState.IDLE
            break
    
    # Wait until calibration is done, abort on failure
    all_done = False
    errors_encountered = False
    while not all_done and not rospy.is_shutdown():
        all_done = True

        for odrv in motor_array:
            # Check if the motor is still calibrating
            if odrv.axis0.current_state == AxisState.ENCODER_OFFSET_CALIBRATION:
                all_done = False

            # Record errors and abort calibration if there are errors
            if odrv.axis0.active_errors != 0:
                print("Error(s) occurred with motor {} during calibration: {}".format(odrv.serial_number, decode_errors(odrv.axis0.active_errors)))
                errors_encountered = True
                for odrv in motor_array:
                    odrv.axis0.requested_state = AxisState.IDLE
                break
        
        rospy.sleep(0.1)
    
    if all_done and not errors_encountered:
        print("Calibration successful for all motors.")
    elif all_done and errors_encountered:
        print("Errors occurred during calibration routine, calibration aborted.")
        return False
    elif not all_done:
        print("Calibration routine was interrupted by shutdown signal. Calibration aborted.")
        for odrv in motor_array:
            odrv.axis0.requested_state = AxisState.IDLE
        return False
    
    # Errors didn't happen during calibration time, but this doesn't mean that calibration was successful.
    # Wait for the actual procedure result to be available from all devices
    results_available = False
    while not results_available and not rospy.is_shutdown():
        results_available = True
        for odrv in motor_array:
            if odrv.axis0.procedure_result == ProcedureResult.BUSY:
                results_available = False
        rospy.sleep(0.1)
    
    if not results_available:
        print("Calibration routine was interrupted by shutdown signal. Calibration aborted.")
        for odrv in motor_array:
            odrv.axis0.requested_state = AxisState.IDLE
        return False
    
    # Now examine each motor for calibration errors and disarm conditions
    calibration_failed = False
    for odrv in motor_array:
        result = odrv.axis0.procedure_result
        errors = odrv.axis0.active_errors
        if result != ProcedureResult.SUCCESS:
            calibration_failed = True
            if errors != 0:
                print("Motor error(s) in motor {}: {}".format(odrv.serial_number, decode_errors(errors)))
            print("Calibration procedure failed in motor {}. Reason: {}".format(odrv.serial_number, procedure_codes[result]))
            if result == ProcedureResult.DISARMED:
                print("Motor {} disarmed. Reason: {}".format(odrv.serial_number, decode_errors(odrv.axis0.disarm_reason)))
            for odrv in motor_array:
                odrv.axis0.requested_state = AxisState.IDLE
            break
    
    if calibration_failed:
        print("Calibration failed for some motors. Calibration aborted.")
        return False
    
    # If everything went well so far, request closed loop control state from each device, and wait until it happens
    for odrv in motor_array:
        odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

    # Make sure that all motors went to closed loop control mode
    close_loop_achieved = False
    errors_occurred = False
    while not close_loop_achieved and not rospy.is_shutdown():
        close_loop_achieved = True
        for odrv in motor_array:
            if odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
                close_loop_achieved = False
            if odrv.axis0.active_errors != 0:
                errors_occurred = True
                print("Error(s) occurred: ", decode_errors(odrv.axis0.active_errors))
                for odrv in motor_array:
                    odrv.axis0.requested_state = AxisState.IDLE
                break
    
    if errors_occurred:
        print("Error(s) occurred while trying to activate closed loop control. Aborting initialization.")
        return False

    # Everything went well, every motor is now ready for commands.
    return True
