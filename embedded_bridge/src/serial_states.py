import struct

class SerialTX:    
    def __init__(self):
        """
        Unified serial transmission object that contains every command for the rover.
        Performs conversions from different integer types to byte arrays sendable
        over a serial port. Also adds a checksum byte to the end. (41 bytes)
        """
        # Drive commands
        self.driveMotorLBSpeed = 0      # int32
        self.driveMotorLFSpeed = 0      # int32
        self.driveMotorRBSpeed = 0      # int32
        self.driveMotorRFSpeed = 0      # int32

        # Arm positions
        self.waistPosition = 1.0        # float32
        self.tumorPosition = 2.0        # float32
        self.elbowPosition = 3.0        # float32
        self.wristPitchPosition = 4.0   # float32
        self.wristRollPosition = 5.0    # float32

        # End effector open?
        self.clawState = False          # boolean

        # Science commands
        self.augerDrillSpeed = 7        # int8
        self.stepperSteps = 8           # int8
        self.leadScrewSpeed = 9         # int8

        # Power command
        self.relayStates = 0            # uint8

    def encode(self):
        """
        Encodes the command fields into a payload for serial transmission, and
        adds a checksum byte computed using CRC8CCITT algorithm.

        Returns:
            bytes: The encoded payload. (42 bytes with checksum)
        """
        payload  = int32_to_bytes(self.driveMotorLBSpeed)
        payload += int32_to_bytes(self.driveMotorLFSpeed)
        payload += int32_to_bytes(self.driveMotorRBSpeed)
        payload += int32_to_bytes(self.driveMotorRFSpeed)

        payload += float_to_bytes(self.waistPosition)
        payload += float_to_bytes(self.tumorPosition)
        payload += float_to_bytes(self.elbowPosition)
        payload += float_to_bytes(self.wristPitchPosition)
        payload += float_to_bytes(self.wristRollPosition)
        payload += bool_to_bytes(self.clawState)

        payload += byte_to_bytes(self.augerDrillSpeed)
        payload += byte_to_bytes(self.stepperSteps)
        payload += byte_to_bytes(self.leadScrewSpeed)

        payload += byte_to_bytes(self.relayStates)

        crc = 0
        for byte in payload:
            crc = compute_crc8ccitt(crc, byte)
        
        payload += struct.pack("c", crc.to_bytes(1, 'big'))

        return payload
    
    def debugPrint(self):
        print("[TX]______________________________________\n")
        print(f"[Drive Commands]\n\
                LB: {self.driveMotorLBSpeed}\n\
                LF: {self.driveMotorLFSpeed}\n\
                RB: {self.driveMotorRBSpeed}\n\
                RF: {self.driveMotorRFSpeed}\n\n")
        print(f"[Arm Commands]\n\
                Claw State: {self.clawState}\n\
                Wrist Roll: {self.wristRollPosition}\n\
                Wrist Pitch: {self.wristPitchPosition}\n\
                Elbow: {self.elbowPosition}\n\
                Tumor: {self.tumorPosition}\n\
                Waist: {self.waistPosition}\n\n")
        print(f"[Science Commands]\n\
                Auger Speed: {self.augerDrillSpeed}\n\
                Stepper Steps: {self.stepperSteps}\n\
                Lead Screw Speed: {self.leadScrewSpeed}\n\n")
        print(f"[Power Commands]\n\
                Relay States: {self.relayStates}\n\n")


class SerialRX:
    """
    Unified serial transmission object that contains every type of feedback from the rover.
    Performs conversions from raw byte array received from the backplane and different
    datatypes, which it keeps as class fields. Also has the ability to perform CRC validation
    and reject corrupted packages. Currently ignores invalid packets and waits for the next one.
    (87 bytes with checksum)
    """  
    def __init__(self):
        # Drive feedback
        self.driveMotorLBSpeed = 0      # float32
        self.driveMotorLFSpeed = 0      # float32
        self.driveMotorRBSpeed = 0      # float32
        self.driveMotorRFSpeed = 0      # float32

        # Arm motor positions
        self.waistPosition = 0          # float32
        self.tumorPosition = 0          # float32
        self.elbowPosition = 0          # float32
        self.wristPitchPosition = 0     # float32
        self.wristRollPosition = 0      # float32

        # Arm motor velocities
        self.waistSpeed = 0             # float32
        self.tumorSpeed = 0             # float32
        self.elbowSpeed = 0             # float32
        self.wristPitchSpeed = 0        # float32
        self.wristRollSpeed = 0         # float32

        # Claw feedback
        self.clawState = False          # boolean

        # Science feedback
        self.moisture1 = 0              # uint16
        self.moisture2 = 0              # uint16
        self.moisture3 = 0              # uint16
        self.moisture4 = 0              # uint16
        self.limitSwitches = 0          # uint8

        # Power feedback
        self.wheelCurrentLB = 0         # uint16
        self.wheelCurrentLF = 0         # uint16
        self.wheelCurrentRB = 0         # uint16
        self.wheelCurrentRF = 0         # uint16
        self.upperArmCurrent = 0        # uint16
        self.lowerArmCurrent = 0        # uint16
        self.scienceMotorsCurrent = 0   # uint16
        self.scienceStepperCurrent = 0  # uint16

        # Battery feedback
        self.battery1Current = 0        # uint16
        self.battery2Current = 0        # uint16
    
    def debugPrint(self):
        print("[RX]______________________________________\n")
        print(f"[Drive Feedback]\n\
                LB: {self.driveMotorLBSpeed}\n\
                LF: {self.driveMotorLFSpeed}\n\
                RB: {self.driveMotorRBSpeed}\n\
                RF: {self.driveMotorRFSpeed}\n\n")

        print(f"[Arm Feedback - Positions]\n\
                Claw state: {self.clawState}\n\
                Wrist Roll: {self.wristRollPosition}\n\
                Wrist Pitch: {self.wristPitchPosition}\n\
                Elbow: {self.elbowPosition}\n\
                Tumor: {self.tumorPosition}\n\
                Waist: {self.waistPosition}\n\n")

        print(f"[Arm Feedback - Velocities]\n\
                Wrist Roll: {self.wristRollSpeed}\n\
                Wrist Pitch: {self.wristPitchSpeed}\n\
                Elbow: {self.elbowSpeed}\n\
                Tumor: {self.tumorSpeed}\n\
                Waist: {self.waistSpeed}\n\n")

        print(f"[Science Feedback]\n\
                Moisture Sensor 1: {self.moisture1}\n\
                MS2: {self.moisture2}\n\
                MS3: {self.moisture3}\n\
                MS4: {self.moisture4}\n\
                Limit Switches: {self.limitSwitches}\n\n")

        print(f"[Power Feedback - Currents]\n\
                Drive LB: {self.wheelCurrentLB}\n\
                Drive LF: {self.wheelCurrentLF}\n\
                Drive RB: {self.wheelCurrentRB}\n\
                Drive RF: {self.wheelCurrentRF}\n\
                Upper Arm: {self.upperArmCurrent}\n\
                Lower Arm: {self.lowerArmCurrent}\n\
                Science Motors: {self.scienceMotorsCurrent}\n\
                Science Steppers: {self.scienceStepperCurrent}\n\n")
        
        print(f"[Battery Feedback]\n\
                Battery 1: {self.battery1Current}\n\
                Battery 2: {self.battery2Current}\n\n")

    def decode(self, buffer):
        checksum = buffer[-1]
        crc = 0
        for byte in buffer[:-1]:
            crc = compute_crc8ccitt(crc, byte)
        
        if checksum != crc:
            print("[Checksum validation failed, skipping packet]")
            return False
        
        self.driveMotorLBSpeed = bytes_to_float(buffer[:4])
        self.driveMotorLFSpeed = bytes_to_float(buffer[4:8])
        self.driveMotorRBSpeed = bytes_to_float(buffer[8:12])
        self.driveMotorRFSpeed = bytes_to_float(buffer[12:16])

        self.waistPosition = bytes_to_float(buffer[16:20])
        self.tumorPosition = bytes_to_float(buffer[20:24])
        self.elbowPosition = bytes_to_float(buffer[24:28])
        self.wristPitchPosition = bytes_to_float(buffer[28:32])
        self.wristRollPosition = bytes_to_float(buffer[32:36])

        self.waistSpeed = bytes_to_float(buffer[36:40])
        self.tumorSpeed = bytes_to_float(buffer[40:44])
        self.elbowSpeed = bytes_to_float(buffer[44:48])
        self.wristPitchSpeed = bytes_to_float(buffer[48:52])
        self.wristRollSpeed = bytes_to_float(buffer[52:56])
        self.clawState = (buffer[56] != 0)

        self.moisture1 = bytes_to_uint16(buffer[57:59])
        self.moisture2 = bytes_to_uint16(buffer[59:61])
        self.moisture3 = bytes_to_uint16(buffer[61:63])
        self.moisture4 = bytes_to_uint16(buffer[63:65])
        self.limitSwitches = bytes_to_byte(buffer[65])

        self.wheelCurrentLB = bytes_to_uint16(buffer[66:68])
        self.wheelCurrentLF = bytes_to_uint16(buffer[68:70])
        self.wheelCurrentRB = bytes_to_uint16(buffer[70:72])
        self.wheelCurrentRF = bytes_to_uint16(buffer[72:74])
        self.upperArmCurrent = bytes_to_uint16(buffer[74:76])
        self.lowerArmCurrent = bytes_to_uint16(buffer[76:78])
        self.scienceMotorsCurrent = bytes_to_uint16(buffer[78:80])
        self.scienceStepperCurrent = bytes_to_uint16(buffer[80:82])

        self.battery1Current = bytes_to_uint16(buffer[82:84])
        self.battery2Current = bytes_to_uint16(buffer[84:86])

        return True

"""
Utility functions to convert from different types to
bytearrays
"""
def float_to_bytes(f):
    return bytearray(struct.pack('f', f))

def int32_to_bytes(f):
    byte_arr = bytearray(4)
    for i in range(4):
        byte_arr[i] = (f >> (i * 8)) & 0xff

    return byte_arr

def short_to_bytes(s):
    byte_arr = bytearray(2)
    for i in range(2):
        byte_arr[i] = (s >> (i * 8)) & 0xff

    return byte_arr

def byte_to_bytes(b):
    return bytearray(struct.pack('b', b))

def bool_to_bytes(b):
    return bytearray(struct.pack('?', b))

"""
Functions to convert from bytearrays to their relevant types
"""
def bytes_to_float(arr):
    return struct.unpack('f', arr)[0]

def bytes_to_int32(arr):
    retval = 0
    for i in range(4):
        retval |= (arr[i] << (i * 8))
    
    if arr[3] & 0x80:  # Check if the sign bit is set
        retval -= 1 << 32  # Perform sign extension

    return retval

def bytes_to_int16(arr):
    retval = 0
    for i in range(2):
        retval |= (arr[i] << (i * 8))

    if arr[1] & 0x80:  # Check if the sign bit is set
        retval -= 1 << 16  # Perform sign extension

    return retval

def bytes_to_uint16(arr):
    retval = 0
    for i in range(2):
        retval |= (arr[i] << (i * 8))

    return retval
    
def bytes_to_byte(arr):
    unsigned_value = struct.unpack('B', bytes([arr]))[0]
    signed_value = unsigned_value if unsigned_value < 128 else unsigned_value - 256

    return signed_value


def compute_crc8ccitt(crc, data):
    """Compute the CRC8-CCITT of the data for Error Flow

    Parameters
    --------
    crc : int
        Current value of the CRC8-CCITT
    data : int
        next value in the payload

    Returns
    --------
    int
        new computed crc value
    """
    index = crc ^ data
    return int.from_bytes(g_pui8Crc8CCITT[index], byteorder='big')


g_pui8Crc8CCITT = [
    b'\x00', b'\x07', b'\x0E', b'\x09', b'\x1C', b'\x1B', b'\x12', b'\x15',
    b'\x38', b'\x3F', b'\x36', b'\x31', b'\x24', b'\x23', b'\x2A', b'\x2D',
    b'\x70', b'\x77', b'\x7E', b'\x79', b'\x6C', b'\x6B', b'\x62', b'\x65',
    b'\x48', b'\x4F', b'\x46', b'\x41', b'\x54', b'\x53', b'\x5A', b'\x5D',
    b'\xE0', b'\xE7', b'\xEE', b'\xE9', b'\xFC', b'\xFB', b'\xF2', b'\xF5',
    b'\xD8', b'\xDF', b'\xD6', b'\xD1', b'\xC4', b'\xC3', b'\xCA', b'\xCD',
    b'\x90', b'\x97', b'\x9E', b'\x99', b'\x8C', b'\x8B', b'\x82', b'\x85',
    b'\xA8', b'\xAF', b'\xA6', b'\xA1', b'\xB4', b'\xB3', b'\xBA', b'\xBD',
    b'\xC7', b'\xC0', b'\xC9', b'\xCE', b'\xDB', b'\xDC', b'\xD5', b'\xD2',
    b'\xFF', b'\xF8', b'\xF1', b'\xF6', b'\xE3', b'\xE4', b'\xED', b'\xEA',
    b'\xB7', b'\xB0', b'\xB9', b'\xBE', b'\xAB', b'\xAC', b'\xA5', b'\xA2',
    b'\x8F', b'\x88', b'\x81', b'\x86', b'\x93', b'\x94', b'\x9D', b'\x9A',
    b'\x27', b'\x20', b'\x29', b'\x2E', b'\x3B', b'\x3C', b'\x35', b'\x32',
    b'\x1F', b'\x18', b'\x11', b'\x16', b'\x03', b'\x04', b'\x0D', b'\x0A',
    b'\x57', b'\x50', b'\x59', b'\x5E', b'\x4B', b'\x4C', b'\x45', b'\x42',
    b'\x6F', b'\x68', b'\x61', b'\x66', b'\x73', b'\x74', b'\x7D', b'\x7A',
    b'\x89', b'\x8E', b'\x87', b'\x80', b'\x95', b'\x92', b'\x9B', b'\x9C',
    b'\xB1', b'\xB6', b'\xBF', b'\xB8', b'\xAD', b'\xAA', b'\xA3', b'\xA4',
    b'\xF9', b'\xFE', b'\xF7', b'\xF0', b'\xE5', b'\xE2', b'\xEB', b'\xEC',
    b'\xC1', b'\xC6', b'\xCF', b'\xC8', b'\xDD', b'\xDA', b'\xD3', b'\xD4',
    b'\x69', b'\x6E', b'\x67', b'\x60', b'\x75', b'\x72', b'\x7B', b'\x7C',
    b'\x51', b'\x56', b'\x5F', b'\x58', b'\x4D', b'\x4A', b'\x43', b'\x44',
    b'\x19', b'\x1E', b'\x17', b'\x10', b'\x05', b'\x02', b'\x0B', b'\x0C',
    b'\x21', b'\x26', b'\x2F', b'\x28', b'\x3D', b'\x3A', b'\x33', b'\x34',
    b'\x4E', b'\x49', b'\x40', b'\x47', b'\x52', b'\x55', b'\x5C', b'\x5B',
    b'\x76', b'\x71', b'\x78', b'\x7F', b'\x6A', b'\x6D', b'\x64', b'\x63',
    b'\x3E', b'\x39', b'\x30', b'\x37', b'\x22', b'\x25', b'\x2C', b'\x2B',
    b'\x06', b'\x01', b'\x08', b'\x0F', b'\x1A', b'\x1D', b'\x14', b'\x13',
    b'\xAE', b'\xA9', b'\xA0', b'\xA7', b'\xB2', b'\xB5', b'\xBC', b'\xBB',
    b'\x96', b'\x91', b'\x98', b'\x9F', b'\x8A', b'\x8D', b'\x84', b'\x83',
    b'\xDE', b'\xD9', b'\xD0', b'\xD7', b'\xC2', b'\xC5', b'\xCC', b'\xCB',
    b'\xE6', b'\xE1', b'\xE8', b'\xEF', b'\xFA', b'\xFD', b'\xF4', b'\xF3'
]