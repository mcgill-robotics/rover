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
        found, port = findArduinoPort()
    SERIAL = serial.Serial(port,9600,timeout=5)


    #TEST
    while 1:
        send_bytes(SERIAL, '0', [5.23, 9.6, 10.334])
        time.sleep(2)
        if SERIAL.inWaiting() > 0:
            formatted_msg, msg = read_bytes(SERIAL)
            time.sleep(2)
            print(f"Message from arduino: {formatted_msg} received as {msg}")


    print("This is main.py in SerialAPI")

    while 1:
        time.sleep(6)  # with the port open, the response will be buffered
                       # so wait a bit longer for response here

        while not synchronisation(SERIAL):
            time.sleep(1)

        time.sleep(2)

        #Wait for first message
        msg = 'BUF'

        #Until Arduino send a finish byte
        while not msg[1]=='F':
            while not SERIAL.inWaiting() > 0:
                time.sleep(1)
            formatted_msg, msg = read_bytes(SERIAL)  # read everything in the input buffer

            # Send ack for received message
            print(f"Message from arduino: {msg} received as {formatted_msg}")
            send_ack(SERIAL)

        send_finishAck(SERIAL)
        print("Exchange over")


def findArduinoPort():
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
        warnings.warn('Multiple Arduinos found - using the first')
    else:
        print("Arduino Found!")

    return True, arduino_ports[0]




def send_ack(SERIAL):
    msg = '~A'.encode('unicode_escape')
    print(f'Sending{msg}')
    SERIAL.write(msg)
    return


def send_finishAck(SERIAL):
    msg = '~Q'.encode('unicode_escape')
    print(f'Sending{msg}')
    SERIAL.write(msg)
    return


def read_bytes(SERIAL):
    msg = SERIAL.read(SERIAL.inWaiting())
    if not (chr(msg[0]) == START_OF_PACKET) or (chr(msg[1]) not in ('0', '1','2','3','4','5','6','A','R','S','Y','F','Q')):
        print(f"No frame detected: {msg}")
        return msg

    print(msg)

    ID = chr(msg[1])
    payload_len = msg[2]
    sys_id = msg[3]

    #https://stackoverflow.com/questions/6999737/convert-from-hex-character-to-unicode-character-in-python
    str_in = str(chr(msg[0]))+"ID:"+chr(msg[1])+"  PayloadSize:"+str(payload_len)+"  SysID:"+chr(sys_id)

    #Conversion of payload based on ID

    if ID in ('0', '1'):
        str_in+=" Floats:"
        for i in range(3, 3+payload_len, 4): #For each float (4 byte each starting at the third byte of the msg)
            str_in+="'"
            byte_array = b''+ msg[i+3].to_bytes(1,'big') + msg[i+2].to_bytes(1,'big') + msg[i+1].to_bytes(1,'big') + msg[i].to_bytes(1,'big')
            raw_float = struct.unpack('!f', byte_array)
            formatted_float = float("{:.3f}".format(raw_float[0])) #Format the float to be precise to 3 digits
            str_in += str(formatted_float)
            str_in+="'"
        str_in+="  Checksum:"+str(msg[2+payload_len+1])

    if ID=='2':
        #3691 int16(pixels)
        pass

    if ID=='3':
        str_in += " Int:"
        str_in += "'"
        byte_array = b'' + msg[6].to_bytes(1, 'big') + msg[5].to_bytes(1, 'big') + msg[4].to_bytes(1, 'big') + msg[3].to_bytes(1, 'big')
        raw_int = struct.unpack('!i', byte_array)
        str_in += str(raw_int[0])
        str_in += "'"

        str_in += " Float:"
        str_in += "'"
        byte_array = b'' + msg[10].to_bytes(1, 'big') + msg[9].to_bytes(1, 'big') + msg[8].to_bytes(1,'big') + msg[7].to_bytes(1, 'big')
        raw_float = struct.unpack('!f', byte_array)
        formatted_float = float("{:.4f}".format(raw_float[0]))  # Format the float to be precise to 3 digits
        str_in += str(formatted_float)
        str_in += "'"
        str_in += "  Checksum:" + str(msg[2 + payload_len + 1])

    if ID=='4':
        str_in+=" Floats:"
        for i in range(3, 3+payload_len, 4): #For each float (4 byte each starting at the third byte of the msg)
            str_in+="'"
            byte_array = b'' + msg[i+3].to_bytes(1,'big') + msg[i+2].to_bytes(1,'big') + msg[i+1].to_bytes(1,'big') + msg[i].to_bytes(1,'big')
            raw_float = struct.unpack('!f', byte_array)
            formatted_float = float("{:.4f}".format(raw_float[0])) #Format the float to be precise to 3 digits
            str_in += str(formatted_float)
            str_in+="'"
        str_in+="  Checksum:"+str(msg[2+payload_len+1])

    if ID=='5':
        str_in+=" Bits:"
        for i in range(3, 3+payload_len):
            str_in += "'"
            str_in += msg[i]
            str_in += "'"
        str_in += "  Checksum:" + str(msg[2 + payload_len + 1])

    if ID in ('A', 'R', 'S', 'Y', 'F', 'Q'):
        str_in += "  Checksum:" + str(msg[3])

    return str_in, msg


def send_bytes(SERIAL, packet_id, data):

    if packet_id in ('0', '1', '6'):
        length = len(data)


    packet_size = 2*length + 4 #2 character for each float (hex notation) so length * 2 for packet_size
    if packet_size >= MAX_PACKET_SIZE:
        return False


    #Convert the data to bytes
    data_hex_array = []
    for i in range(length):
        hex_value = str(hex(struct.unpack('<I', struct.pack('<f', data[i]))[0])) #Use [2:] to convert to hex string without the 0x
        data_hex_array.append(hex_value)

    output_byte_string = ''.join([START_OF_PACKET, packet_id, chr(packet_size)]+data_hex_array)

    #output_buffer[packet_size - 1] = crc8ccitt(output_buffer + DATA_SEGMENT_OFFSET, len);

    byte_array = bytes(output_byte_string, 'unicode_escape')
    SERIAL.write(byte_array)
    print(f'Sending {START_OF_PACKET + packet_id + " " + str(packet_size) + " " + str(data)} as {byte_array}')
    return True



def synchronisation(SERIAL):
    # Serial read section
    if not SERIAL.inWaiting() > 0:
        return False

    # Read everything in the input buffer
    msg = read_bytes(SERIAL)
    print("Message from arduino: " + msg)
    if not msg[1] == 'S':
        print("No synchronisation")
        return False

    print("Synchronisation starts")

    #Send SYN-ACK
    msg = '~Y'.encode('unicode_escape')
    print(f'Sending {msg}')
    SERIAL.write(msg)

    time.sleep(2)

    # Wait for response
    while not SERIAL.inWaiting() > 0:
        time.sleep(1)

    #Read message
    msg = read_bytes(SERIAL)
    print("Message from arduino: " + msg)
    if not msg[1] == 'A':
        print("Message not acknowledged")
        return False
    print("Message acknowledged")
    print("Synchronisation Completed")
    return True



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