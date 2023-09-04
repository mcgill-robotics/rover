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
