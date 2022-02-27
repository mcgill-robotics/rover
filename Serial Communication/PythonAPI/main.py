#https://stackoverflow.com/questions/24074914/python-to-arduino-serial-read-write

#!/usr/bin/python
import sys
import warnings
import serial
import serial.tools.list_ports
import time
import struct

START_OF_PACKET = '~'
MAX_PACKET_SIZE = 255


def main():
    #The following line is for serial over GPIO
    #port = '/dev/ttyS0'

    # Find the Arduino Serial
    found = False
    while not found:
        found, ports = find_arduino_port()
    SERIAL = serial.Serial(ports[0], 9600, timeout=5)

    print(SERIAL)


    # TEST
    while 1:
        send_bytes(SERIAL, '0', [5.23, 9.6, 10.334])
        time.sleep(2)
        if SERIAL.inWaiting() > 0:
            valid, packet, rest_of_msg = read_bytes(SERIAL)
            str_packet = [str(i) for i in packet]
            formatted_msg = ' '.join(str_packet)
            print(f"Message from arduino: {formatted_msg}")
            while len(rest_of_msg) > 0 and valid is True:
                valid, packet, rest_of_msg = decode_bytes(rest_of_msg)
                str_packet = [str(i) for i in packet]
                formatted_msg = ' '.join(str_packet)
                print(f"Message from arduino: {formatted_msg}")
            time.sleep(2)


    print("This is main.py in SerialAPI")

    while 1:
        time.sleep(6)  # with the port open, the response will be buffered
                       # so wait a bit longer for response here

        while not synchronisation(SERIAL):
            time.sleep(1)

        time.sleep(2)

        # Wait for first message
        msg = 'BUF'

        #Until Arduino send a finish byte
        while not msg[1]=='F':
            while not SERIAL.inWaiting() > 0:
                time.sleep(1)
            formatted_msg, msg = read_serial_bytes(SERIAL)  # read everything in the input buffer

            # Send ack for received message
            print(f"Message from arduino: {msg} received as {formatted_msg}")
            send_ack(SERIAL)

        send_finishAck(SERIAL)
        print("Exchange over")


def find_arduino_port():
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
        warnings.warn('Multiple Arduinos found')
    else:
        print("Arduino Found!")

    return True, arduino_ports




def send_ack(ser):
    msg = '~A'.encode('unicode_escape')
    print(f'Sending {msg}')
    ser.write(msg)
    return

def send_retransmit(ser):
    msg = '~R'.encode('unicode_escape')
    print(f'Sending {msg}')
    ser.write(msg)
    return


def send_finishAck(ser):
    msg = '~Q'.encode('unicode_escape')
    print(f'Sending {msg}')
    ser.write(msg)
    return


def read_bytes(ser: serial.serialwin32.Serial, show: bool = True):
    """
     Parameters
      --------
      ser : serial.serialwin32.Serial
          serial of the sender
      show : bool
          if printing is wanted

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
    msg = ser.read_until(START_OF_PACKET, ser.inWaiting())

    valid, packet, rest_of_msg = decode_bytes(msg)

    if valid:
        send_ack(ser)
        return valid, packet, rest_of_msg

    else:
        send_retransmit(ser)
        return valid, packet, rest_of_msg


def decode_bytes(msg: str, show = True):
    """
     Parameters
      --------
      msg : str
          message to decode

      show : bool
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
          The rest of the bytes to be read. The original message if no valid packet found.
    """

    # Filter the message
    if len(msg) < 2:
        if show:
            print(f"No frame detected: {msg}")
        return False, ('N', -1, 'N', [], -1), msg

    try:
        frame_type = chr(msg[1])
    except:
        if show:
            print(f"No frame detected: {msg}")
        return False, ('N', -1, 'N', [], -1), msg

    if frame_type not in ('0', '1', '2', 'A', 'R', 'S', 'Y', 'F', 'Q'):
        if show:
            print(f"No frame detected: {msg}")
        return False, ('N', -1, 'N', [], -1), msg

    if frame_type in ('A', 'R', 'S', 'Y', 'F', 'Q'):
        if show:
            print(f"Arduino sent: {frame_type}")

        # If more byte on the serial
        if len(msg) > 2:
            return True, (frame_type, -1, 'N', -1, []), msg[2:]

        return True, (frame_type, -1, 'N', -1, []), ""

    try:
        payload_len = msg[2]
        if len(msg) < (3 + payload_len + 1):
            if show:
                print(f"Message too short {msg}")
            return False, ('N', -1, 'N', [], -1), msg
    except:
        if show:
            print(f"No valid payload length {msg}")
        return False, ('N', -1, 'N', [], -1), msg

    payload = []

    # Conversion of payload based on frame type
    if frame_type in ('0', '1'):
        system_id = chr(msg[3])
        for i in range(4, 4 + payload_len - 1, 4):  # For each float (4 byte each starting at the fourth byte of the msg)
            byte_array = b'' + msg[i + 3].to_bytes(1, 'big') \
                         + msg[i + 2].to_bytes(1, 'big') \
                         + msg[i + 1].to_bytes(1, 'big') \
                         + msg[i].to_bytes(1, 'big')
            raw_float = struct.unpack('!f', byte_array)
            formatted_float = float("{:.3f}".format(raw_float[0]))  # Format the float to be precise to 3 digits
            payload.append(formatted_float)
        checksum = msg[2 + payload_len + 1]

    elif frame_type in '2':
        # 3691 int16(pixels)
        checksum = msg[2 + payload_len + 1]
        pass

    return True, (frame_type, payload_len, system_id, payload, checksum), msg[(3 + payload_len + 1):]


def bytes_to_stringll(msg: str):
    frame_type = chr(msg[1])
    payload_len = msg[2]
    sys_id = msg[3]

    str_in = str(chr(msg[0]))+"ID:"+chr(msg[1])+"  PayloadSize:"+str(payload_len)+"  SysID:"+chr(sys_id)

    # Conversion of payload based on ID
    if frame_type in ('0', '1'):
        str_in += " Floats:"
        for i in range(4, 4+payload_len-1, 4):  # For each float (4 byte each starting at the third byte of the msg)
            str_in += "'"
            byte_array = b'' + msg[i+3].to_bytes(1, 'big') + msg[i+2].to_bytes(1, 'big') + msg[i+1].to_bytes(1, 'big') + msg[i].to_bytes(1, 'big')
            raw_float = struct.unpack('!f', byte_array)
            formatted_float = float("{:.3f}".format(raw_float[0]))  # Format the float to be precise to 3 digits
            str_in += str(formatted_float)
            str_in += "'"
        str_in += "  Checksum:"+str(msg[2+payload_len+1])

    elif frame_type == '2':
        # 3691 int16(pixels)
        pass

    return str_in, msg


def send_bytes(ser, packet_id, payload=[], system_id='N'):

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

    ser.write(byte_array)
    print(f'Sending {START_OF_PACKET} {packet_id} {payload_size} {system_id} {payload} {crc} as {byte_array}')
    return True



#https://stackoverflow.com/questions/8751653/how-to-convert-a-binary-string-into-a-float-value
#For the next three functions
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
    [f] = struct.unpack(">L", struct.pack(">f", value))
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


if __name__=="__main__":
    main()

# Serial write section
# speed = input("Enter a value between 0.0 and 1.0 (motor speed): ")
# try:
#    speed = float("{0:.3f}".format(float(speed)))
#    data = struct.pack('f', speed) #Convert speed to float
# except Exception:
#    print("Enter a valid input")
#    continue

# print("Python value sent: ")
# print(f"{speed} = {struct.unpack('f',data)}")
# SERIAL.write(data)

