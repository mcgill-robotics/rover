import struct

import serial
import sys
import serial.tools.list_ports
import struct
import time
import random

START_OF_PACKET = '~'
MAX_PACKET_SIZE = 255
NO_MSG_FRAME = ('N', -1, 'N', [], -1)
FRAME_TYPES = ('0', '1', '2', 'A', 'R', 'S', 'Y', 'F', 'Q')


class SerialInterface:
    """Serial Interface to establish connection and communicate
    with devices using the McGill Robotics Rover Serial Frame standard

    Attributes
    --------
    port : str
        port to connect to external hardware with
    baudrate : int
        data rate used to communicate with paired device
    mcu : serial.Serial
        serial object interfacing with the hardware level
    """

    WINDOW_SIZE = 32
    TIMEOUT = 10
    next_frame_id = 0
    last_sent_frame_id = 0
    sync_id = 0
    connected = False

    def __init__(self, port, baud_rate=9600, timeout=5, prints=True):
        # Port Information
        self.port = port
        self.baud_rate = baud_rate
        self.serial = serial.Serial(port, baud_rate, timeout=timeout)

        # Communication Protocol Info
        self.window = [''] * self.WINDOW_SIZE
        self.SYS_ID = 'P'
        self.peer_sys = ''

        # Others
        self.prints = prints

    def send_ack(self):
        msg = '~A'.encode('unicode_escape')
        if self.prints:
            print(f'Sending {msg}')
        self.serial.write(msg)

    def send_retransmit(self):
        msg = '~R'.encode('unicode_escape')
        if self.prints:
            print(f'Sending {msg}')
        self.serial.write(msg)

    def send_finishAck(self):
        msg = '~Q'.encode('unicode_escape')
        if self.prints:
            print(f'Sending {msg}')
        self.serial.write(msg)

    def wait_for_answer(self, timeout=5):
        for i in range(timeout):
            if s.serial.inWaiting() > 0:
                return self.read_bytes()
            time.sleep(1)
        return False, (), ''

    def read_bytes(self):
        """
         Parameters
          --------

         Returns
          --------
          valid : boolean
              if the message is a valid message

          packet : tuple
              Tuple containing the packet's data (frame_type, payload_len, system_id, payload, checksum)

                      frame_type : char
                          the frame type
                          'N' returned if not valid

                      payload_len : int
                          length of the payload (counting the system_id). 0 is returned in other cases
                          -1 returned when not valid

                      system_id : char
                          ID of the system
                          'N' returned if not valid

                      payload : bytes
                          payload of the message
                          [] returned if not valid or a no payload frame type

                      checksum : int
                          checksum of the message received to compare to what's calculated
                          -1 returned when not valid packet

          rest_of_msg : bytes
              The rest of the bytes to be read. The original message if no valid packet found.
        """
        msg = self.serial.read_until(START_OF_PACKET, self.serial.inWaiting())

        valid, packet, rest_of_msg = decode_bytes(msg)

        if valid:
            #TODO self.send_ack() might add back
            return valid, packet, rest_of_msg

        else:
            #TODO self.send_retransmit() might add back
            return valid, packet, rest_of_msg

    def send_bytes(self, packet_id, payload=[], system_id='N'):

        length = len(payload)
        payload_size = 0

        if packet_id in ('0', '1'):
            payload_size = 4 * length + 1  # Add system ID to payload size (assuming it is not in the data array)

        if packet_id in '2':
            payload_size = 2 * length + 1  # Pixels are two bytes + sys id

        packet_size = 3 + payload_size + 1

        if packet_size >= MAX_PACKET_SIZE:
            return False

        # Convert the data to bytes
        byte_array = struct.pack("cc",
                                 START_OF_PACKET.encode('ascii'),
                                 packet_id.encode('ascii'))

        if packet_id in ('0', '1'):
            byte_array += payload_size.to_bytes(1, 'big')
            byte_array += system_id.encode('ascii')
            for float_value in payload:
                byte_array += float_to_bin(float_value)

        if packet_id in '2':
            byte_array += payload_size.to_bytes(1, 'big')
            byte_array += system_id.encode('ascii')
            for pixel in payload:
                pass
                # TODO byte_array += 2_bytes_to_bin(pixel)

        if packet_id in ('0', '1', '2'):
            # Compute crc8ccitt
            crc = 0
            # crc = compute_crc8ccitt(crc, ord(packet_id)) Now matches the Arduino algorithm
            for char in byte_array[3:]:
                crc = compute_crc8ccitt(crc, char)  # compute for entire payload
            byte_array += struct.pack("c", crc.to_bytes(1, 'big'))

        self.serial.write(byte_array)
        if self.prints:
            print(f'Sending {START_OF_PACKET} {packet_id} {payload_size} {system_id} {payload} {crc} as {byte_array}')
        return True


def decode_bytes(msg: str, prints=True):
    """
     Parameters
      --------
      msg : str
          message to decode

      prints : bool
          if printing is wanted

      Returns
      --------
      correct : boolean
          if the message is a valid message

      packet : tuple
          Tuple containing the packet's data (frame_type, payload_len, system_id, payload, checksum)

                  frame_type : char
                      the frame type
                      'N' returned if not valid

                  payload_len : int
                      length of the payload (counting the system_id). 0 is returned in other cases
                      -1 returned when not valid

                  system_id : char
                      ID of the system
                      'N' returned if not valid

                  payload : bytes
                      payload of the message
                      [] returned if not valid or a no payload frame type

                  checksum : int
                      checksum of the message received to compare to what's calculated
                      -1 returned when not valid packet

      rest_of_msg : bytes
          The rest of the bytes to be read. The rest of the message if no valid packet found.
    """

    # Filter the message:
    # No Message Frame : False, ('N', -1, 'N', [], -1), rest_of_msg
    # If the message is too short (length < 2) return No Message Frame
    # If the frame_type isn't valid in return No Message Frame
    # If the frame is valid and in ('A', 'R', 'S', 'Y', 'F', 'Q') return valid and the rest of the message if there is, "" otherwise
    # If the frame type is valid but the payload length isn't valid or doesn't match the length of the message, return No Message Frame
    # If the given checksum doesn't match the calculated checksum, return No Message Frame
    # If the frame is valid return True, Frame, rest_of_msg

    # Start of filtering
    if len(msg) < 2:
        if prints:
            print(f"No frame detected: {msg}")
        return False, NO_MSG_FRAME, msg

    try:
        frame_type = chr(msg[1])
    except:
        if prints:
            print(f"No frame detected: {msg}")
        return False, NO_MSG_FRAME, msg

    if frame_type not in FRAME_TYPES:
        if prints:
            print(f"No frame detected: {msg}")
        return False, NO_MSG_FRAME, ''.join(msg[1:].partition(START_OF_PACKET)[1:])

    # Valid frames
    if frame_type in FRAME_TYPES[3:]:
        if prints:
            print(f"Arduino sent: {frame_type}")

        # If more byte on the serial
        if len(msg) > 2:
            return True, (frame_type, -1, 'N', -1, []), b''.join(msg[2:].partition(START_OF_PACKET.encode('ascii'))[1:])
        return True, (frame_type, -1, 'N', -1, []), ""

    try:
        payload_len = msg[2]
        if len(msg) < (3 + payload_len + 1):
            if prints:
                print(f"Message too short {msg}")
            return False, ('N', -1, 'N', [], -1), ''
    except:
        if prints:
            print(f"No valid payload length {msg}")
        return False, NO_MSG_FRAME, b''.join(msg[1:].partition(START_OF_PACKET.encode('ascii'))[1:])

    payload = []
    crc = 0

    # Conversion of payload based on frame type
    if frame_type in ('0', '1'):
        system_id = chr(msg[3])
        crc = compute_crc8ccitt(crc, ord(system_id))  # For sys id (part of payload)
        for i in range(4, 4 + payload_len - 1,
                       4):  # For each float (4 byte each starting at the fourth byte of the msg)
            byte_array = b'' + msg[i + 3].to_bytes(1, 'big') \
                         + msg[i + 2].to_bytes(1, 'big') \
                         + msg[i + 1].to_bytes(1, 'big') \
                         + msg[i].to_bytes(1, 'big')

            for byte in byte_array[::-1]:
                crc = compute_crc8ccitt(crc, byte)  # compute for entire payload

            raw_float = struct.unpack('!f', byte_array)
            formatted_float = float("{:.3f}".format(raw_float[0]))  # Format the float to be precise to 3 digits
            payload.append(formatted_float)
        checksum = msg[2 + payload_len + 1]

    elif frame_type in '2':
        # 3691 int16(pixels)
        checksum = msg[2 + payload_len + 1]
        pass

    # Checksum validation
    if crc != checksum:
        if prints:
            print(f"No valid checksum {msg}")
        return False, NO_MSG_FRAME, ""

    return True, (frame_type, payload_len, system_id, payload, checksum), \
           b''.join(msg[(3 + payload_len + 1):].partition(START_OF_PACKET.encode('ascii'))[1:])


# https://stackoverflow.com/questions/8751653/how-to-convert-a-binary-string-into-a-float-value
# For the next three functions
def bin_to_float(b):
    """ Convert binary string to a float. """
    bf = int_to_bytes(int(b, 2), 8)  # 8 bytes needed for IEEE 754 binary64.
    return struct.unpack('>d', bf)[0]


def int_to_bytes(n, length):  # Helper function
    """ Int/long to byte string.

        Python 3.2+ has a built-in int.to_bytes() method that could be used
        instead, but the following works in earlier versions including 2.x.
    """
    return decode('%%0%dx' % (length << 1) % n, 'hex')[-length:]


def float_to_bin(value):  # For testing.
    """ Convert float to 64-bit binary string. """
    # [d] = struct.unpack(">Q", struct.pack(">d", value))
    # return '{:064b}'.format(d)
    """ Convert float to 32-bit binary string. """
    [f] = struct.unpack(">l", struct.pack(">f", value))
    return struct.pack("i", f)  # Pack the int as 4 bytes
    # return bin(f)
    # return '{:032b}'.format(f)


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


def find_ports():
    """Finds all ports available for connection

    Returns
    --------
    list(str)
        all ports available for connection
    """
    port_list = []
    if sys.platform == "win32" or sys.platform == "win64":
        for i in range(0, 128):
            port = "COM" + str(i)
            if port_is_valid(port):
                port_list.append(port)

    elif sys.platform == "linux" or sys.platform == "linux2":
        for i in range(0, 128):
            port = "/dev/ttyUSB" + str(i)
            if port_is_valid(port):
                port_list.append(port)
            else:
                break

        for i in range(0, 128):
            port = "/dev/ttyACM" + str(i)
            if port_is_valid(port):
                port_list.append(port)
            else:
                break

    return port_list


def find_arduino_ports():
    arduino_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if 'Arduino' or 'CH340' in p.description  # may need tweaking to match new arduinos
    ]
    for p in serial.tools.list_ports.comports():
        print(p)
    if not arduino_ports:
        raise IOError("No Arduino found")
        return False, None
    if len(arduino_ports) > 1:
        print('Multiple Arduinos found')
    else:
        print("Arduino Found!")

    return True, arduino_ports


def port_is_valid(port):
    """Verifies if a port is valid
    Performs trial connection,fails if port doesn't exist

    Returns
    --------
    bool
        whether port is valid or not
    """
    try:
        p = serial.Serial(port)
        p.close()
        return True
    except serial.SerialException:
        return False


if __name__ == "__main__":
    # s = SerialInterface('/dev/ttyUSB0', 115200, 0.01)

    """
    ports = []
    for port in find_ports():
        print(port)
        ports.append(port)
    """
    # Find the Arduino Serial
    found = False
    while not found:
        found, ports = find_arduino_ports()

    s = SerialInterface(ports[0], 9600, timeout=5)

    while 1:
        frame_type = '0'
        payload = [112.5]

        valid = False

        """
        while not valid:
            s.send_bytes(frame_type, payload)
            valid, packet, rest_of_msg = s.wait_for_answer(5)
            print(packet)
        """
        time.sleep(2)

        #s.send_bytes(frame_type, payload)
        s.send_bytes(frame_type, [float(random.randint(0, 180))])

        time.sleep(3)

        read = False
        for i in range(3):
            if s.serial.inWaiting() > 0:
                read = True
                valid, packet, rest_of_msg = s.read_bytes()
                print(packet)
                str_packet = [str(i) for i in packet]
                formatted_msg = ' '.join(str_packet)
                print(f"Message from arduino: {formatted_msg}")
                while len(rest_of_msg) > 0 and valid is True:
                    valid, packet, rest_of_msg = decode_bytes(rest_of_msg)
                    str_packet = [str(i) for i in packet]
                    formatted_msg = ' '.join(str_packet)
                    print(f"Message from arduino: {formatted_msg}")
                s.serial.flush()
                break
            time.sleep(1)

        if not read:
            s.serial.flush()
            print("Nothing read")