import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import odrive_utils
import odrive
import rospy
from drive_control.msg import WheelSpeed
from odrive.enums import AxisState, ProcedureResult

class Node_ODriveInterface():
    def __init__(self):
        # Start the connection with odrives

        # Initialize node and subscribe to whatever needs to be subscribed to
        self.speed_cmd = 0.0
        rospy.init_node('odrive_interface')
        self.feedback_publisher = rospy.Publisher("/wheel_velocity_feedback", WheelSpeed, queue_size=1)
        self.command_subscriber = rospy.Subscriber("/wheel_velocity_cmd", WheelSpeed, self.handle_drive_command)

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    def handle_drive_command(self, command):
        self.speed_cmd = (command.left[0])*0.159155
        pass


    def run(self):
        # Enumerate the devices
        # TODO: Replace the signature with something that actually finds more than one ODrive
        drive_lb = enumerate_motors()
        if drive_lb is None:
            raise TimeoutError("No ODrive found")

        if not calibrate_motors([drive_lb]):
            raise IOError("Motor initialization failed")
        
        while not rospy.is_shutdown():
            # Put in speed command
            drive_lb.axis0.controller.input_vel = self.speed_cmd

            # Get feedback
            feedback = WheelSpeed()
            measured_speed_lb = drive_lb.encoder_estimator0.vel_estimate
            measured_speed_lf = 0
            measured_speed_rb = 0
            measured_speed_rf = 0

            feedback.left[0], feedback.left[1] = measured_speed_lb, measured_speed_lf
            feedback.right[0], feedback.right[1] = measured_speed_rb, measured_speed_rf
            print(f"\rMeasured Speed: {measured_speed_lb}", end='')

            # See if there are any errors, try to clear them once
            if drive_lb.axis0.active_errors != 0:
                print(f"\nError(s) occurred: ", decode_errors(drive_lb.axis0.active_errors))
                drive_lb.clear_errors()

            self.rate.sleep()
        
        # On shutdown, bring motors to idle state
        print("Shutdown prompt received. Setting all motors to idle state.")
        drive_lb.axis0.requested_state = AxisState.IDLE


def enumerate_motors():
    # TODO: Write enumeration code with multiple motors
    print("Waiting for ODrives...")
    odrv0 = odrive.find_any(timeout=10)
    if odrv0 is None:
        print("Timeout: no ODrives found")
        return None
    odrv0.clear_errors()
    return odrv0

def calibrate_motors(motor_array):
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
    num_bits = n.bit_length()
    errors_string = ""
    for i in range(num_bits):
        mask = 1 << i
        if n & mask:
            errors_string += (error_codes[mask] + ", ")

    return errors_string[:-2]

if __name__ == "__main__":
    driver = Node_ODriveInterface()
    rospy.spin()
    # # TODO: Enumerate the ODrive devices for each motor
    # print("Waiting for ODrive...")
    # odrv0 = odrive.find_any()
    # print("ODrive found. Serial number: {}".format(odrv0.serial_number))
    # odrv0.clear_errors()

    # print("Calibrating encoder...")

    # # Ask the device to calibrate if isn't calibrating already, and wait until it's done
    # # TODO: In ROS, this should probably put the thread to sleep using rospy.spin()
    # if odrv0.axis0.current_state != AxisState.ENCODER_OFFSET_CALIBRATION:
    #     odrv0.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    # time.sleep(0.01)

    # if odrv0.axis0.current_state != AxisState.ENCODER_OFFSET_CALIBRATION:
    #     print("Error(s) occurred: ", decode_errors(odrv0.axis0.active_errors))

    # while odrv0.axis0.current_state == AxisState.ENCODER_OFFSET_CALIBRATION:
    #     if odrv0.axis0.active_errors != 0:
    #         print("Error(s) occurred: ", decode_errors(odrv0.axis0.active_errors))
    #     time.sleep(0.1)
    #     # rospy.sleep(0.1) # Sleep for 100ms

    # # Now wait for procedure result to be available
    # while odrv0.axis0.procedure_result == ProcedureResult.BUSY:
    #     time.sleep(0.1)

    # # Report calibration failures if any
    # if odrv0.axis0.procedure_result != ProcedureResult.SUCCESS:
    #     if odrv0.axis0.active_errors != 0:
    #         print("Error(s) occurred: ", decode_errors(odrv0.axis0.active_errors))
    #     print("Calibration failed. Reason code: {}".format(odrv0.axis0.procedure_result))
    #     if odrv0.axis0.procedure_result == ProcedureResult.DISARMED:
    #         print("Motor disarmed. Reason code: {}".format(odrv0.axis0.disarm_reason))
    #     exit()

    # # Request close loop control state, and wait until it happens
    # odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    # while odrv0.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
    #     time.sleep(0.1)
