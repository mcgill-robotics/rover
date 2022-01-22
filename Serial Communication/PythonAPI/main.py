#https://stackoverflow.com/questions/24074914/python-to-arduino-serial-read-write

#!/usr/bin/python
import warnings
import serial
import serial.tools.list_ports
import time
import struct

START_OF_PACKET = '~'


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
        #msg = f'{}'.encode()
        ba = bytearray(struct.pack("d", 5.23))
        SERIAL.write(ba)
        time.sleep(2)
        if SERIAL.inWaiting() > 0:
            msg = read_serial(SERIAL)
            time.sleep(2)
            print("Message from arduino: " + msg)


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
            msg = read_serial(SERIAL)  # read everything in the input buffer

            # Send ack for received message
            print("Message from arduino: " + msg)
            send_ack(SERIAL)

        send_finishAck(SERIAL)
        print("Exchange over")


def findArduinoPort():
    arduino_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if 'Arduino' in p.description  # may need tweaking to match new arduinos
    ]
    if not arduino_ports:
        #raise IOError("No Arduino found")
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


def read_serial(SERIAL):
    msg = SERIAL.read(SERIAL.inWaiting())
    if not (chr(msg[0])==START_OF_PACKET) and (chr(msg[1]) in ('0', '1','2','3','4','5','6','A','R','S','Y','F','Q')):
        print(f"No frame detected: {msg}")
        return msg


    #https://stackoverflow.com/questions/6999737/convert-from-hex-character-to-unicode-character-in-python
    str_in = ""+str(chr(msg[0]))+str(chr(msg[1]))+str(msg[2])
    for i in range(msg[2]):

        #Conversion of payload based on ID
        ID = str_in[1]
        len = msg[2]

        if ID in ('0', '1', '6'):
            # https://stackoverflow.com/questions/5415/convert-bytes-to-floating-point-numbers
            # https://docs.python.org/3/library/struct.html#format-characters

            """
            -Check how float are encoded in c when writing to serial in byte
            -From this format, decode in python
            """
            for i in range(3, 3+len):
                #str_in+=str(struct.unpack('d', msg[i])[0])
                str_in += str(msg[i])
            str_in+=str(msg[2+len+1])

        if ID=='2':
            pass

        if ID=='3':
            pass

        if ID=='4':
            pass

        if ID=='5':
            pass

        if ID in ('A', 'R', 'S', 'Y', 'F', 'Q'):
            pass

        if ID=='STRING':
            for i in range(3, 3+len):
                str_in += chr(msg[i])
            str_in+=str(msg[2+len+1])

        return str_in
    return ""



def synchronisation(SERIAL):
    # Serial read section
    if not SERIAL.inWaiting() > 0:
        return False

    # Read everything in the input buffer
    msg = read_serial(SERIAL)
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
    msg = read_serial(SERIAL)
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