import rospy
import odrive
from odrive.enums import AxisState, ProcedureResult


procedure_codes = ["SUCCESS", "BUSY", "CANCELLED", "DISARMED", "NO_RESPONSE", "POLE_PAIR_CPR_MISMATCH",
                   "PHASE_RESISTANCE_OUT_OF_RANGE", "PHASE_INDUCTANCE_OUT_OF_RANGE", "UNBALANCED_PHASES",
                   "UNBALANCED_PHASES", "INVALID_MOTOR_TYPE", "ILLEGAL_HALL_STATE", "TIMEOUT", "HOMING_WITHOUT_ENDSTOP",
                   "INVALID_STATE", "NOT_CALIBRATED", "NOT_CONVERGING"]

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

def enumerate_motors():
    # TODO: Write enumeration code with multiple motors, verifying that each one is connected
    print("Waiting for ODrives...")
    odrv0 = odrive.find_any(timeout=10)
    if odrv0 is None:
        print("Timeout: no ODrives found")
        return None
    odrv0.clear_errors()
    return odrv0


def calibrate_motors(motor_array):
    """
    Calibrates all motors in motor_array at once. On failure, this function sets all listed motors to
    idle and immediately return False.

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
